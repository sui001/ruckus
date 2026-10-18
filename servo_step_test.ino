// servo_step_test.ino
// ESP32-S3 Supermini + INMP441 + positional hobby servo (SG92R).
//
// Why this exists, separately from servo_mic_test:
// The sweep in servo_mic_test steps 1 degree at a time and assumes each step
// makes a tick, so a command every 20 ms should give 50 ticks a second. The
// recording says otherwise: bursts arrive at roughly 14 Hz, about one per
// three and a half commands. The suspected reason is the servo's deadband.
// One degree is ~5.5 us of pulse width on a 1000-2000 us range, which is at or
// under what an SG92R will act on, so the commanded error has to accumulate
// over several steps before the servo moves at all.
//
// If that is right, the tick rate is set by step SIZE against the deadband, not
// by how often commands are sent. So: hold the number of steps fixed, vary the
// size of each step and the interval between them, and see which one the tick
// rate follows. Step sizes at or above the deadband should tick once per
// command, at exactly 1000/interval Hz.
//
// Same wiring, same serial protocol and same JSON plan shape as servo_mic_test,
// so analyze.py reads this without changes. p1 = step size in degrees,
// p2 = interval between steps in ms.
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
// One extra bit of headroom over servo_mic_test. Bigger steps hit noticeably
// harder: move RMS ran 44/79/148/246 for 1/2/3/5 degree steps, and at >>16 the
// 5 degree ticks clipped. There is ~46 dB of signal over the room in the band
// that matters, so trading 6 dB of level for headroom costs nothing here.
#define MIC_SHIFT   17

Servo servo;

// ---------------- Test parameters ----------------
// Steps are held constant so every event lasts steps*interval and the travel
// is what changes with step size. Largest travel is 30 x 5 = 150 deg, which
// from HOME_ANGLE lands at 165, inside the servo's range.
const int stepSizes[]     = {1, 2, 3, 5};      // degrees per step
const int stepIntervals[] = {20, 40, 60};      // ms between steps
// 20 steps keeps the largest travel at 5 x 20 = 100 deg, landing at 115 rather
// than the 165 the first version reached. No stall was measured there, but 165
// is close enough to the end stop that a unit with less travel would grind.
const int STEPS   = 20;
const int REPEATS = 3;

const uint32_t GAP_MS    = 900;
const uint32_t SETTLE_MS = 100;
const int HOME_ANGLE      = 15;
const int SERVO_MIN_ANGLE = 0;
const int SERVO_MAX_ANGLE = 180;

struct LogEvent { char type; int p1; int p2; uint32_t start_ms; uint32_t dur_ms; };
LogEvent logEvents[64];
int logLen = 0;

struct Cmd { uint32_t time_ms; int angle; };
Cmd cmds[2048];
int cmdLen = 0;
uint32_t totalDuration = 0;

void halt(const char *msg, int value) {
  while (true) {
    Serial.printf("FATAL: %s (%d)\n", msg, value);
    Serial.flush();
    delay(1000);
  }
}

void addCmd(uint32_t t, int angle) {
  if (angle < SERVO_MIN_ANGLE || angle > SERVO_MAX_ANGLE)
    halt("commanded angle outside servo range", angle);
  if (cmdLen >= (int)(sizeof(cmds)/sizeof(cmds[0])))
    halt("cmds array full", cmdLen);
  cmds[cmdLen++] = {t, angle};
}

void buildPlan() {
  uint32_t t = 500;
  for (int r = 0; r < REPEATS; r++) {
    for (int z = 0; z < (int)(sizeof(stepSizes)/sizeof(int)); z++) {
      for (int v = 0; v < (int)(sizeof(stepIntervals)/sizeof(int)); v++) {
        int size = stepSizes[z], interval = stepIntervals[v];
        uint32_t moveDur = (uint32_t)STEPS * interval;
        uint32_t window  = moveDur + SETTLE_MS;

        if (logLen >= (int)(sizeof(logEvents)/sizeof(logEvents[0])))
          halt("logEvents array full", logLen);
        logEvents[logLen++] = {'S', size, interval, t, window};

        for (int i = 1; i <= STEPS; i++)
          addCmd(t + (uint32_t)(i - 1) * interval, HOME_ANGLE + i * size);

        addCmd(t + window + SETTLE_MS, HOME_ANGLE);
        t += window + GAP_MS;
      }
    }
  }
  totalDuration = t + 500;

  for (int i = 1; i < cmdLen; i++)
    if (cmds[i].time_ms < cmds[i-1].time_ms)
      halt("cmds are not in time order", i);
}

void printPlanJson() {
  Serial.printf("{\"sample_rate\":%d,\"events\":[", SAMPLE_RATE);
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
    .bck_io_num = I2S_SCK,
    .ws_io_num = I2S_WS,
    .data_out_num = I2S_PIN_NO_CHANGE,
    .data_in_num = I2S_SD
  };
  i2s_driver_install(I2S_PORT, &cfg, 0, NULL);
  i2s_set_pin(I2S_PORT, &pins);
}

void setup() {
  Serial.begin(921600);
  delay(1500);

  Serial.println("=== servo step-size test v" VERSION " ===");
  Serial.println("Does tick rate follow step size or step interval? JSON plan, then raw 16-bit PCM");
  Serial.println("https://github.com/sui001/ruckus");

  servo.setPeriodHertz(50);
  servo.attach(SERVO_PIN, 1000, 2000);
  servo.write(HOME_ANGLE);

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
