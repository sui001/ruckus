// servo_direction_test.ino
// ESP32-S3 Supermini + INMP441 + positional hobby servo (SG92R).
//
// Does the servo sound different going one way than the other?
//
// This was missed by servo_mic_test, which measured only the outbound move and
// treated every return as gap contamination to be excluded. Direction was never
// a variable. Playing a pattern made it obvious by ear, because the sequencer
// alternates direction on every hit and the two sound like a high and a low
// drum. Checking the old recordings afterwards, outbound and return jumps have
// the same 9 kHz peak but centroids 408 Hz apart, with the return about 1 dB
// louder: real, but measured on large jumps rather than on the small moves a
// pattern actually uses.
//
// So: isolated single moves, one direction at a time, at the step sizes the
// sequencer uses as velocities. Each move gets its own analysis window with
// silence either side, so up and down are never averaged together.
//
// Same serial protocol and plan shape as the other sketches, so analyze.py
// reads it unchanged. type = 'U' or 'D', p1 = move size in degrees.
//
// Wiring (avoid ESP32-S3 strapping pins 0/3/45/46):
//   INMP441   VDD->3V3  GND->GND  L/R->GND (left ch)  WS->GPIO12  SCK->GPIO11  SD->GPIO13
//   Servo     signal->GPIO8  power from the board's 5V pin, share ground.

#include <ESP32Servo.h>
#include <driver/i2s.h>

#define VERSION "1.0"

#define I2S_WS    12
#define I2S_SD    13
#define I2S_SCK   11
#define SERVO_PIN 8

#define SAMPLE_RATE 32000
#define I2S_PORT    I2S_NUM_0
// Three bits hotter than the sweep sketches. A 2-12 degree move is a fraction
// of the sound a 150 degree sweep makes: at >>17 this test peaked at 530 of
// 32767, which wastes most of the converter's range on a signal that is already
// close to the room. The largest move here is 12 degrees, so there is room.
#define MIC_SHIFT   14

Servo servo;

// The sizes the sequencer offers as velocities.
const int moveSizes[] = {2, 4, 6, 8, 12};
const int REPEATS = 8;

const uint32_t WINDOW_MS = 250;   // one move, comfortably
const uint32_t GAP_MS    = 350;   // silence, so moves never overlap
const int CENTRE_ANGLE   = 90;
const int SERVO_MIN_ANGLE = 0;
const int SERVO_MAX_ANGLE = 180;

struct LogEvent { char type; int p1; int p2; uint32_t start_ms; uint32_t dur_ms; };
LogEvent logEvents[128];
int logLen = 0;

struct Cmd { uint32_t time_ms; int angle; };
Cmd cmds[256];
int cmdLen = 0;
uint32_t totalDuration = 0;

void halt(const char *msg, int value) {
  while (true) { Serial.printf("FATAL: %s (%d)\n", msg, value); Serial.flush(); delay(1000); }
}

void addCmd(uint32_t t, int angle) {
  if (angle < SERVO_MIN_ANGLE || angle > SERVO_MAX_ANGLE)
    halt("commanded angle outside servo range", angle);
  if (cmdLen >= (int)(sizeof(cmds)/sizeof(cmds[0]))) halt("cmds array full", cmdLen);
  cmds[cmdLen++] = {t, angle};
}

void buildPlan() {
  uint32_t t = 500;
  for (int z = 0; z < (int)(sizeof(moveSizes)/sizeof(int)); z++) {
    int size = moveSizes[z];
    for (int r = 0; r < REPEATS; r++) {
      // up: centre -> centre+size
      if (logLen >= (int)(sizeof(logEvents)/sizeof(logEvents[0]))) halt("logEvents full", logLen);
      logEvents[logLen++] = {'U', size, 0, t, WINDOW_MS};
      addCmd(t, CENTRE_ANGLE + size);
      t += WINDOW_MS + GAP_MS;

      // down: centre+size -> centre, the same distance travelled back
      if (logLen >= (int)(sizeof(logEvents)/sizeof(logEvents[0]))) halt("logEvents full", logLen);
      logEvents[logLen++] = {'D', size, 0, t, WINDOW_MS};
      addCmd(t, CENTRE_ANGLE);
      t += WINDOW_MS + GAP_MS;
    }
  }
  totalDuration = t + 500;
  for (int i = 1; i < cmdLen; i++)
    if (cmds[i].time_ms < cmds[i-1].time_ms) halt("cmds are not in time order", i);
}

void printPlanJson() {
  // skip_ms 0: each event is a single short move, over within ~40 ms, so the
  // onset is the whole sound. analyze.py otherwise skips the first 100 ms.
  // sample_rate must stay the first key, since that is how the PC finds the JSON.
  Serial.printf("{\"sample_rate\":%d,\"skip_ms\":0,\"events\":[", SAMPLE_RATE);
  for (int i = 0; i < logLen; i++) {
    Serial.printf("{\"i\":%d,\"type\":\"%c\",\"p1\":%d,\"p2\":%d,\"start_ms\":%lu,\"dur_ms\":%lu}%s",
      i, logEvents[i].type, logEvents[i].p1, logEvents[i].p2,
      (unsigned long)logEvents[i].start_ms, (unsigned long)logEvents[i].dur_ms,
      (i < logLen - 1) ? "," : "");
  }
  Serial.printf("],\"total_ms\":%lu}\n", (unsigned long)totalDuration);
  Serial.println("===AUDIO_START===");
}

void i2sInit() {
  i2s_config_t cfg = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = SAMPLE_RATE,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format = I2S_COMM_FORMAT_STAND_I2S,
    .intr_alloc_flags = 0,
    .dma_buf_count = 8,
    .dma_buf_len = 256,
    .use_apll = false
  };
  i2s_pin_config_t pins = {
    .bck_io_num = I2S_SCK, .ws_io_num = I2S_WS,
    .data_out_num = I2S_PIN_NO_CHANGE, .data_in_num = I2S_SD
  };
  i2s_driver_install(I2S_PORT, &cfg, 0, NULL);
  i2s_set_pin(I2S_PORT, &pins);
}

void setup() {
  Serial.begin(921600);
  delay(1500);

  Serial.println("=== servo direction test v" VERSION " ===");
  Serial.println("Isolated up and down moves recorded separately: JSON plan, then raw 16-bit PCM");
  Serial.println("https://github.com/sui001/ruckus");

  servo.setPeriodHertz(50);
  servo.attach(SERVO_PIN, 1000, 2000);
  servo.write(CENTRE_ANGLE);
  delay(400);           // settle at centre before the first move is recorded

  i2sInit();
  buildPlan();
  printPlanJson();
}

int cmdIdx = 0;
uint32_t t0;
bool started = false;

void loop() {
  if (!started) { t0 = millis(); started = true; }
  uint32_t now = millis() - t0;

  while (cmdIdx < cmdLen && now >= cmds[cmdIdx].time_ms) {
    servo.write(cmds[cmdIdx].angle);
    cmdIdx++;
  }

  static int32_t raw[128];
  size_t bytesRead = 0;
  i2s_read(I2S_PORT, raw, sizeof(raw), &bytesRead, 0);
  int n = bytesRead / sizeof(int32_t);
  if (n > 0) {
    static int16_t out[128];
    for (int i = 0; i < n; i++) {
      int32_t s = raw[i] >> MIC_SHIFT;
      if (s >  32767) s =  32767;
      if (s < -32768) s = -32768;
      out[i] = (int16_t)s;
    }
    Serial.write((uint8_t*)out, n * sizeof(int16_t));
  }
}
