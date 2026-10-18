// Motor sound map: runs each servo on its own through its speed range and
// records what the INMP441 hears (level and dominant pitch) at each step.
// No sound-driven control. This is characterisation, the map comes after.
//
//   ch0  continuous: pulse 1500..2000 and 1500..1000 us in 50 us steps,
//        both directions
//   ch11 MG996R and ch15 180 (both positional): sweeps 30..150 deg and back
//        at set rates, plus a full-speed jump and a powered hold
// Motors not under test are switched fully off so they stay silent.
//
// Output lines, one per step:
//   map,<ch>,<motor>,<mode>,<setting>,<dir>,<dBFS>,<dB over room>,<band dB over room>,<Hz>,<blocks>
// dBFS and Hz are medians over the blocks recorded during that step.
// Band is energy in BAND_LO..BAND_HI Hz: room noise is mostly low rumble and
// motor whine is high, so the band picks motors out where full level can't.
// Hz is nan unless the full or band level is PITCH_SNR dB over the room,
// because the peak frequency of something barely audible is just noise.
//
// Wiring:
//   INMP441: SCK GPIO11, WS GPIO12, SD GPIO13, L/R GND, VCC 3V3
//   PCA9685: SDA GPIO10, SCL GPIO9, VCC 3V3, V+ unconnected, servos on 6V terminal
//
// Serial commands: run (all), run 0 | run 11 | run 15, room

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <driver/i2s.h>
#include <arduinoFFT.h>

#define VERSION "0.3.0"

#define PIN_SCK 11
#define PIN_WS  12
#define PIN_SD  13
#define PIN_SDA 10
#define PIN_SCL 9
#define CH_CONT 0
#define CH_MG   11
#define CH_180  15

#define SAMPLE_RATE 16000
#define N_SAMPLES   2048          // 128 ms block
#define CHUNK       256           // 16 ms, servo steps happen between chunks
#define FREQ_MIN_HZ 60
#define FREQ_MAX_HZ 6000
#define MAX_BLOCKS  64
#define PITCH_SNR   6.0f          // dB over room before a pitch is reported
#define BAND_LO     1000
#define BAND_HI     6000

float roomDb = NAN, roomBand = NAN;

static int32_t raw[N_SAMPLES];
static float vReal[N_SAMPLES];
static float vImag[N_SAMPLES];
ArduinoFFT<float> FFT(vReal, vImag, N_SAMPLES, (float)SAMPLE_RATE);
Adafruit_PWMServoDriver pwm(0x40);

float dbs[MAX_BLOCKS], hzs[MAX_BLOCKS], bands[MAX_BLOCKS];
int nBlocks = 0;

void setupI2S() {
  i2s_config_t cfg = {
    .mode                 = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate          = SAMPLE_RATE,
    .bits_per_sample      = I2S_BITS_PER_SAMPLE_32BIT,
    .channel_format       = I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format = I2S_COMM_FORMAT_STAND_I2S,
    .intr_alloc_flags     = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count        = 8,
    .dma_buf_len          = 256,
    .use_apll             = false,
    .tx_desc_auto_clear   = false,
    .fixed_mclk           = 0,
  };
  i2s_driver_install(I2S_NUM_0, &cfg, 0, NULL);
  i2s_pin_config_t pins = {
    .mck_io_num   = I2S_PIN_NO_CHANGE,
    .bck_io_num   = PIN_SCK,
    .ws_io_num    = PIN_WS,
    .data_out_num = I2S_PIN_NO_CHANGE,
    .data_in_num  = PIN_SD,
  };
  i2s_set_pin(I2S_NUM_0, &pins);
  i2s_zero_dma_buffer(I2S_NUM_0);
}

// Reads one 128 ms block in 16 ms chunks, calling tick() between chunks so a
// servo can keep moving while we listen. Records dBFS and peak Hz.
void recordBlock(void (*tick)()) {
  size_t got;
  for (int c = 0; c < N_SAMPLES / CHUNK; c++) {
    i2s_read(I2S_NUM_0, raw + c * CHUNK, CHUNK * sizeof(int32_t), &got, portMAX_DELAY);
    if (tick) tick();
  }
  double mean = 0;
  for (int i = 0; i < N_SAMPLES; i++) { vReal[i] = (float)(raw[i] >> 8); mean += vReal[i]; }
  mean /= N_SAMPLES;
  double sq = 0;
  for (int i = 0; i < N_SAMPLES; i++) { vReal[i] -= mean; sq += (double)vReal[i] * vReal[i]; vImag[i] = 0; }
  float db = 20.0f * log10f(sqrt(sq / N_SAMPLES) / 8388608.0 + 1e-9);

  FFT.windowing(FFTWindow::Hann, FFTDirection::Forward);
  FFT.compute(FFTDirection::Forward);
  FFT.complexToMagnitude();
  int lo = FREQ_MIN_HZ * N_SAMPLES / SAMPLE_RATE + 1;
  int hi = min(FREQ_MAX_HZ * N_SAMPLES / SAMPLE_RATE, N_SAMPLES / 2 - 2);
  int k = lo;
  for (int i = lo; i <= hi; i++) if (vReal[i] > vReal[k]) k = i;
  float a = vReal[k - 1], b = vReal[k], cc = vReal[k + 1];
  float d = a - 2 * b + cc;
  float hz = (k + (d != 0 ? 0.5f * (a - cc) / d : 0)) * SAMPLE_RATE / N_SAMPLES;

  double be = 0;               // relative units, only ever compared with the room
  for (int i = BAND_LO * N_SAMPLES / SAMPLE_RATE; i <= BAND_HI * N_SAMPLES / SAMPLE_RATE; i++)
    be += (double)vReal[i] * vReal[i];
  float band = 10.0f * log10f(be + 1e-9);

  if (nBlocks < MAX_BLOCKS) { dbs[nBlocks] = db; hzs[nBlocks] = hz; bands[nBlocks] = band; nBlocks++; }
}

float median(float *v, int n) {
  float t[MAX_BLOCKS];
  memcpy(t, v, n * sizeof(float));
  for (int i = 1; i < n; i++) for (int j = i; j > 0 && t[j] < t[j - 1]; j--) { float x = t[j]; t[j] = t[j - 1]; t[j - 1] = x; }
  return n ? t[n / 2] : NAN;
}

void report(int ch, const char *motor, const char *mode, float setting, const char *dir) {
  float db = median(dbs, nBlocks);
  float snr = isnan(roomDb) ? 0 : db - roomDb;
  float bsnr = isnan(roomBand) ? 0 : median(bands, nBlocks) - roomBand;
  float hz = (snr >= PITCH_SNR || bsnr >= PITCH_SNR) ? median(hzs, nBlocks) : NAN;
  Serial.printf("map,%d,%s,%s,%.0f,%s,%.1f,%.1f,%.1f,%.1f,%d\n",
                ch, motor, mode, setting, dir, db, snr, bsnr, hz, nBlocks);
  nBlocks = 0;
}

void listenFor(uint32_t ms) {
  nBlocks = 0;
  uint32_t t = millis();
  while (millis() - t < ms) recordBlock(nullptr);
}

void allOff() {
  for (int ch : {CH_CONT, CH_MG, CH_180}) pwm.setPWM(ch, 0, 4096);
}

// ── positional sweep state, driven from tick() ──────────────────────────────
int swCh, swUsMin, swUsMax;
float swPos, swTarget, swStep;     // degrees, degrees per 16 ms chunk
bool swMoving;

void swWrite() {
  pwm.writeMicroseconds(swCh, (uint16_t)(swUsMin + swPos / 180.0f * (swUsMax - swUsMin)));
}
void swTick() {
  if (!swMoving) return;
  float d = swTarget - swPos;
  if (fabsf(d) <= swStep) { swPos = swTarget; swMoving = false; }
  else swPos += (d > 0 ? swStep : -swStep);
  swWrite();
}

// Sweep from..to at rate deg/s, recording only while moving (at least one block).
void sweep(float from, float to, float rate) {
  swPos = from; swWrite(); delay(700);          // park at the start, let it settle
  i2s_zero_dma_buffer(I2S_NUM_0);
  swTarget = to;
  swStep = rate > 0 ? rate * CHUNK / (float)SAMPLE_RATE : 999;
  swMoving = true;
  nBlocks = 0;
  if (rate <= 0) { recordBlock(swTick); recordBlock(nullptr); }   // jump: a servo needs ~0.3 s
  else while (swMoving || nBlocks == 0) recordBlock(swTick);
}

void testPositional(int ch, const char *motor, int usMin, int usMax) {
  Serial.printf("# %s ch%d: sweeps 30..150 deg\n", motor, ch);
  allOff(); delay(400);
  swCh = ch; swUsMin = usMin; swUsMax = usMax;

  swPos = 90; swWrite(); delay(700);
  listenFor(1500);
  report(ch, motor, "hold", 0, "-");

  const float rates[] = {30, 60, 120, 240, 0};   // 0 = jump at full speed
  for (float r : rates) {
    sweep(30, 150, r);  report(ch, motor, r > 0 ? "sweep" : "jump", r, "up");
    sweep(150, 30, r);  report(ch, motor, r > 0 ? "sweep" : "jump", r, "down");
  }
  allOff();
}

// Each direction starts from a settled stop, so the first reading isn't the
// jolt of the servo powering up from fully off.
void testContinuous(int ch, const char *motor) {
  Serial.printf("# %s ch%d: pulse steps, both directions, 0.5 s settle, 1.5 s listen\n", motor, ch);
  allOff(); delay(400);
  for (int dir = 0; dir < 2; dir++) {
    pwm.writeMicroseconds(ch, 1500);
    delay(1500);
    for (int off = 0; off <= 500; off += 50) {
      int us = dir == 0 ? 1500 + off : 1500 - off;
      pwm.writeMicroseconds(ch, us);
      delay(500);
      listenFor(1500);
      report(ch, motor, "pulse", us, dir == 0 ? "fwd" : "rev");
    }
  }
  pwm.writeMicroseconds(ch, 1500); delay(300);
  allOff();
}

void measureRoom() {
  allOff(); delay(500);
  float prev = roomDb;
  roomDb = NAN; roomBand = NAN;   // report the room against nothing
  listenFor(1500);
  roomDb = median(dbs, nBlocks);
  roomBand = median(bands, nBlocks);
  report(-1, "room", "off", 0, "-");
  if (!isnan(prev)) Serial.printf("# room drift since last: %+.1f dB\n", roomDb - prev);
}

void runAll() {
  Serial.println("# start, header: map,ch,motor,mode,setting,dir,dBFS,dB_over_room,band_over_room,Hz,blocks");
  measureRoom();
  testContinuous(CH_CONT, "continuous");
  measureRoom();
  testPositional(CH_MG, "mg996r", 600, 2400);
  measureRoom();
  testPositional(CH_180, "servo180", 500, 2500);
  measureRoom();
  Serial.println("# done");
}

void handleSerial() {
  static String line;
  while (Serial.available()) {
    char ch = Serial.read();
    if (ch != '\n' && ch != '\r') { line += ch; continue; }
    line.trim();
    if (line == "run") runAll();
    else if (line == "run 0") { measureRoom(); testContinuous(CH_CONT, "continuous"); }
    else if (line == "run 11") { measureRoom(); testPositional(CH_MG, "mg996r", 600, 2400); }
    else if (line == "run 15") { measureRoom(); testPositional(CH_180, "servo180", 500, 2500); }
    else if (line == "room") measureRoom();
    else if (line.length()) Serial.println("? run | run 0 | run 11 | run 15 | room");
    line = "";
  }
}

void setup() {
  Serial.begin(115200);
  delay(1500);
  Serial.println("=== motor sound map v" VERSION " ===");
  Serial.println("Runs each servo alone through its speeds, logs mic level and pitch per step");
  Serial.println("https://github.com/sui001/ruckus/tree/main/motor_player_rig/gen3d_motor_sound_map");

  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setTimeOut(50);
  Wire.beginTransmission(0x40);
  if (Wire.endTransmission() != 0) {
    Serial.println("PCA9685 not found at 0x40. Halting, no servo output.");
    while (true) delay(1000);
  }
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(50);
  allOff();

  setupI2S();
  listenFor(300);               // settling reads, discarded
  nBlocks = 0;
  Serial.println("send 'run' to start (about 2 min), or run 0 / run 11 / run 15");
}

void loop() { handleSerial(); delay(10); }
