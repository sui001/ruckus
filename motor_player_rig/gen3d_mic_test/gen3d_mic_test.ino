// INMP441 bring-up on an ESP32-S3 SuperMini: level, pitch, note name.
// Prints about 8 lines a second:
//   -32.1 dBFS |########            |  440.3 Hz  A4  +2c
// Pitch and note only print when the sound is above the gate.
//
// Wiring:
//   INMP441   ESP32-S3 SuperMini
//   SD     -> GPIO13
//   WS     -> GPIO12
//   SCK    -> GPIO11
//   L/R    -> GND   (left channel)
//   VCC    -> 3V3
//   GND    -> GND
//
// Serial commands: "gate <dBFS>" sets the pitch gate, e.g. gate -50

#include <driver/i2s.h>
#include <arduinoFFT.h>

#define VERSION "0.1.0"

#define PIN_SCK 11
#define PIN_WS  12
#define PIN_SD  13

#define SAMPLE_RATE 16000
#define N_SAMPLES   2048    // 128 ms window, 7.8 Hz bins before interpolation
#define FREQ_MIN_HZ 60
#define FREQ_MAX_HZ 4000
#define BAR_WIDTH   20
#define BAR_FLOOR   -80.0f  // dBFS at an empty bar

float gateDb = -55.0f;      // below this, no pitch is reported

static int32_t raw[N_SAMPLES];
static float vReal[N_SAMPLES];
static float vImag[N_SAMPLES];
ArduinoFFT<float> FFT(vReal, vImag, N_SAMPLES, (float)SAMPLE_RATE);

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

// Fills vReal with DC-removed samples, returns RMS in dBFS.
// Returns NAN if the mic sends nothing but zeros (usually a wiring fault).
float readBlock() {
  size_t got = 0;
  i2s_read(I2S_NUM_0, raw, sizeof(raw), &got, portMAX_DELAY);
  int n = got / sizeof(int32_t);
  double mean = 0;
  bool allZero = true;
  for (int i = 0; i < n; i++) {
    int32_t s = raw[i] >> 8;           // 24-bit signed
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
  float rms = sqrt(sq / n) / 8388608.0;  // 2^23 = full scale
  return 20.0f * log10f(rms + 1e-9f);
}

// Strongest bin in range, refined by fitting a parabola through its neighbours.
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

void printNote(float hz) {
  float midi = 69.0f + 12.0f * log2f(hz / 440.0f);
  int m = (int)lroundf(midi);
  int cents = (int)lroundf((midi - m) * 100.0f);
  Serial.printf("%7.1f Hz  %s%d  %+3dc", hz, NOTES[((m % 12) + 12) % 12], m / 12 - 1, cents);
}

void handleSerial() {
  static String line;
  while (Serial.available()) {
    char ch = Serial.read();
    if (ch != '\n' && ch != '\r') { line += ch; continue; }
    line.trim();
    if (line.startsWith("gate ")) {
      gateDb = line.substring(5).toFloat();
      Serial.printf("gate -> %.1f dBFS\n", gateDb);
    } else if (line.length()) {
      Serial.println("? gate <dBFS>");
    }
    line = "";
  }
}

void setup() {
  Serial.begin(115200);
  delay(1500);  // native USB CDC needs a moment
  Serial.println("=== INMP441 mic test v" VERSION " ===");
  Serial.println("Mic bring-up: level in dBFS, dominant pitch, nearest note");
  Serial.println("https://github.com/sui001/ruckus/tree/main/motor_player_rig/gen3d_mic_test");
  Serial.printf("SCK=%d WS=%d SD=%d, %d Hz, %d-sample window, gate %.1f dBFS\n",
                PIN_SCK, PIN_WS, PIN_SD, SAMPLE_RATE, N_SAMPLES, gateDb);
  setupI2S();
  size_t dummy;  // first reads are settling noise
  i2s_read(I2S_NUM_0, raw, sizeof(raw), &dummy, portMAX_DELAY);
  i2s_read(I2S_NUM_0, raw, sizeof(raw), &dummy, portMAX_DELAY);
}

void loop() {
  handleSerial();
  float db = readBlock();
  if (isnan(db)) {
    Serial.println("mic sends only zeros: check SD, VCC, and L/R to GND");
    return;
  }
  int fill = constrain((int)((db - BAR_FLOOR) / -BAR_FLOOR * BAR_WIDTH), 0, BAR_WIDTH);
  char bar[BAR_WIDTH + 1];
  for (int i = 0; i < BAR_WIDTH; i++) bar[i] = i < fill ? '#' : ' ';
  bar[BAR_WIDTH] = 0;
  Serial.printf("%6.1f dBFS |%s|  ", db, bar);
  if (db > gateDb) printNote(peakHz());
  else Serial.print("(quiet)");
  Serial.println();
}
