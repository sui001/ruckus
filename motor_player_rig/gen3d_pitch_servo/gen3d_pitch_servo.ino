// Sound to three servos: INMP441 on the ESP32-S3, servos on a PCA9685.
//   MG996R  ch11  pitch, log scale: LO_HZ -> 0 deg, HI_HZ -> 180 deg
//   180     ch15  note within the octave: C -> 0 deg ... B -> 180 deg
//   cont.   ch0   loudness: stopped at the gate, full speed LOUD_RANGE dB above
// Below the gate the positional servos hold and the continuous one stops.
//
// The mic sits close to the servos, so it hears them. At boot the sketch
// measures the room, then runs the servos and measures them, and sets the
// gate above both. Otherwise servo noise reads as sound and the loop feeds itself.
//
// Wiring:
//   INMP441: SCK GPIO11, WS GPIO12, SD GPIO13, L/R GND, VCC 3V3
//   PCA9685: SDA GPIO10, SCL GPIO9, VCC 3V3, V+ unconnected, servos on 6V terminal
//
// Serial commands:
//   gate <dBFS>   override the gate
//   lo <hz>       MG996R frequency at 0 deg
//   hi <hz>       MG996R frequency at 180 deg
//   cal           rerun the noise calibration

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <driver/i2s.h>
#include <arduinoFFT.h>

#define VERSION "0.2.0"

#define PIN_SCK 11
#define PIN_WS  12
#define PIN_SD  13
#define PIN_SDA 10
#define PIN_SCL 9
#define CH_CONT 0
#define CH_MG   11
#define CH_180  15

#define US_STOP    1500
#define LOUD_RANGE 30.0f  // dB above the gate for full speed

#define SAMPLE_RATE 16000
#define N_SAMPLES   2048  // 128 ms window, about 8 updates a second
#define FREQ_MIN_HZ 60
#define FREQ_MAX_HZ 4000
#define SMOOTH      0.5f  // 0..1, higher follows faster

float gateDb = -55.0f;
float loHz = 100.0f;
float hiHz = 1000.0f;
float mgDeg = 90.0f, octDeg = 90.0f;

static int32_t raw[N_SAMPLES];
static float vReal[N_SAMPLES];
static float vImag[N_SAMPLES];
ArduinoFFT<float> FFT(vReal, vImag, N_SAMPLES, (float)SAMPLE_RATE);
Adafruit_PWMServoDriver pwm(0x40);

const char *NOTES[] = {"C", "C#", "D", "D#", "E", "F", "F#", "G", "G#", "A", "A#", "B"};

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

// Fills vReal with DC-removed samples, returns RMS in dBFS, NAN if all zeros.
float readBlock() {
  size_t got = 0;
  i2s_read(I2S_NUM_0, raw, sizeof(raw), &got, portMAX_DELAY);
  int n = got / sizeof(int32_t);
  double mean = 0;
  bool allZero = true;
  for (int i = 0; i < n; i++) {
    int32_t s = raw[i] >> 8;
    if (s) allZero = false;
    vReal[i] = (float)s;
    mean += s;
  }
  if (allZero) return NAN;
  mean /= n;
  double sq = 0;
  for (int i = 0; i < N_SAMPLES; i++) {
    if (i < n) { vReal[i] -= mean; sq += (double)vReal[i] * vReal[i]; }
    else vReal[i] = 0;
    vImag[i] = 0;
  }
  float rms = sqrt(sq / n) / 8388608.0;
  return 20.0f * log10f(rms + 1e-9f);
}

float peakHz() {
  FFT.windowing(FFTWindow::Hann, FFTDirection::Forward);
  FFT.compute(FFTDirection::Forward);
  FFT.complexToMagnitude();
  int lo = FREQ_MIN_HZ * N_SAMPLES / SAMPLE_RATE + 1;
  int hi = min(FREQ_MAX_HZ * N_SAMPLES / SAMPLE_RATE, N_SAMPLES / 2 - 2);
  int k = lo;
  for (int i = lo; i <= hi; i++) if (vReal[i] > vReal[k]) k = i;
  float a = vReal[k - 1], b = vReal[k], c = vReal[k + 1];
  float d = a - 2 * b + c;
  float off = (d != 0) ? 0.5f * (a - c) / d : 0;
  return (k + off) * SAMPLE_RATE / N_SAMPLES;
}

void writeDeg(int ch, float deg, int usMin, int usMax) {
  deg = constrain(deg, 0.0f, 180.0f);
  pwm.writeMicroseconds(ch, (uint16_t)(usMin + deg / 180.0f * (usMax - usMin)));
}
void writeMG(float deg)  { writeDeg(CH_MG, deg, 600, 2400); }   // MG996R binds at the extremes
void write180(float deg) { writeDeg(CH_180, deg, 500, 2500); }
void writeCont(int us)   { pwm.writeMicroseconds(CH_CONT, constrain(us, 1500, 2000)); }

// Loudest block seen over ms milliseconds.
float listenMax(uint32_t ms) {
  float mx = -120;
  uint32_t t = millis();
  while (millis() - t < ms) { float db = readBlock(); if (!isnan(db) && db > mx) mx = db; }
  return mx;
}

void calibrate() {
  Serial.println("calibrating: stay quiet for 4 s");
  writeCont(US_STOP);
  delay(300);
  float room = listenMax(1000);
  writeCont(2000);                        // servos at their loudest
  writeMG(20); write180(20);
  float moving = listenMax(700);
  writeMG(160); write180(160);
  moving = max(moving, listenMax(700));
  writeCont(US_STOP);
  writeMG(mgDeg); write180(octDeg);
  delay(300);
  gateDb = max(room + 10.0f, moving + 6.0f);
  Serial.printf("room %.1f dBFS, servos %.1f dBFS -> gate %.1f dBFS\n", room, moving, gateDb);
  if (moving > room + 20.0f)
    Serial.println("servos are much louder than the room: move the mic away or speak up");
}

void handleSerial() {
  static String line;
  while (Serial.available()) {
    char ch = Serial.read();
    if (ch != '\n' && ch != '\r') { line += ch; continue; }
    line.trim();
    if (line.startsWith("gate ")) { gateDb = line.substring(5).toFloat(); Serial.printf("gate -> %.1f dBFS\n", gateDb); }
    else if (line.startsWith("lo ")) { loHz = line.substring(3).toFloat(); Serial.printf("MG 0 deg -> %.0f Hz\n", loHz); }
    else if (line.startsWith("hi ")) { hiHz = line.substring(3).toFloat(); Serial.printf("MG 180 deg -> %.0f Hz\n", hiHz); }
    else if (line == "cal") calibrate();
    else if (line.length()) Serial.println("? gate <dBFS> | lo <hz> | hi <hz> | cal");
    line = "";
  }
}

void setup() {
  Serial.begin(115200);
  delay(1500);
  Serial.println("=== pitch servo v" VERSION " ===");
  Serial.println("INMP441 sound drives 3 servos: pitch->MG996R ch11, note->180 ch15, loudness->continuous ch0");
  Serial.println("https://github.com/sui001/Servo-notes/tree/main/motor_player_rig/gen3d_pitch_servo");

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
  delay(10);
  writeCont(US_STOP); writeMG(mgDeg); write180(octDeg);
  Serial.println("PCA9685 ok at 0x40");

  setupI2S();
  size_t dummy;
  i2s_read(I2S_NUM_0, raw, sizeof(raw), &dummy, portMAX_DELAY);
  i2s_read(I2S_NUM_0, raw, sizeof(raw), &dummy, portMAX_DELAY);
  calibrate();
  Serial.printf("MG: %.0f Hz -> 0 deg, %.0f Hz -> 180 deg. Listening.\n", loHz, hiHz);
}

void loop() {
  handleSerial();
  float db = readBlock();
  if (isnan(db)) { Serial.println("mic sends only zeros: check SD, VCC, and L/R to GND"); return; }
  Serial.printf("%6.1f dBFS  ", db);
  if (db <= gateDb) {
    writeCont(US_STOP);
    Serial.printf("(quiet)  hold MG %3.0f  180 %3.0f  cont stop\n", mgDeg, octDeg);
    return;
  }

  float hz = peakHz();
  float midi = 69.0f + 12.0f * log2f(hz / 440.0f);
  int m = (int)lroundf(midi);
  int pc = ((m % 12) + 12) % 12;

  float t = constrain(log2f(hz / loHz) / log2f(hiHz / loHz), 0.0f, 1.0f);
  mgDeg += SMOOTH * (t * 180.0f - mgDeg);
  octDeg = pc * 180.0f / 11.0f;             // steps, no smoothing, so a scale reads as steps
  float loud = constrain((db - gateDb) / LOUD_RANGE, 0.0f, 1.0f);
  int contUs = US_STOP + (int)(loud * 500);

  writeMG(mgDeg);
  write180(octDeg);
  writeCont(contUs);
  Serial.printf("%7.1f Hz  %-2s%d  MG %3.0f  180 %3.0f  cont %d us\n",
                hz, NOTES[pc], m / 12 - 1, mgDeg, octDeg, contUs);
}
