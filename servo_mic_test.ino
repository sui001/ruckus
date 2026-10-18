// servo_mic_test.ino
// ESP32-S3 Supermini + INMP441 (I2S mic) + 9g (or bigger) hobby servo.
//
// Runs a scheduled sequence of servo moves while continuously streaming
// mic audio out over USB serial, so movement noise is captured with no
// gaps. Prints a JSON test plan first (so the PC side knows what happened
// when), then a delimiter line, then raw 16-bit PCM forever.
//
// UNTESTED — written without hardware to compile/flash against. Likely
// things to check on first flash:
//   - Arduino-ESP32 core version: this uses the legacy `driver/i2s.h` API.
//     Core v3.x may want `driver/i2s_std.h` instead — if i2s_driver_install
//     fails to compile, that's why.
//   - ESP32Servo needs a core with LEDC available on your chosen pin.
//   - Baud 921600 over native USB CDC should be plenty for 16kHz*16bit
//     (32 KB/s), but confirm your serial monitor / pyserial can hold it.
//
// Wiring (avoid ESP32-S3 strapping pins 0/3/45/46):
//   INMP441   VDD->3V3  GND->GND  L/R->GND (left ch)  WS->GPIO12  SCK->GPIO11  SD->GPIO13
//   Servo     signal->GPIO8  power from the board's 5V pin (fine for a 9g SG92R), and
//             share ground with the ESP32.

#include <ESP32Servo.h>
#include <driver/i2s.h>

#define I2S_WS    12
#define I2S_SD    13
#define I2S_SCK   11
#define SERVO_PIN 8

#define VERSION "1.2"

// 16 kHz put Nyquist at 8 kHz, and the first real recording still had servo
// energy climbing at that edge: +34 dB over room noise in the 6-8 kHz band with
// no sign of a peak. The spectrum was being cut off rather than captured, so
// any pitch read off it was a read of the band limit. 32 kHz moves Nyquist to
// 16 kHz, past where small-servo gear noise runs out.
#define SAMPLE_RATE 32000
#define I2S_PORT    I2S_NUM_0

// INMP441 gives 24 bits left-justified in a 32-bit slot, so >>8 is the 24-bit
// value and >>16 is that scaled to 16-bit full scale. This was >>14, four times
// hotter than full scale, which went unnoticed with the mic 5 cm away and
// clipped 1.09% of samples across half the events once it moved to 1 cm.
// Clipping manufactures broadband harmonics, which is indistinguishable from
// the "is the whine noisy or tonal" answer this rig exists to measure.
#define MIC_SHIFT 16

// ---- SET THIS before flashing: which servo type is wired up ----
static const bool TEST_POSITIONAL = false;  // 0-180 deg hobby servo
static const bool TEST_CONTINUOUS = true;   // continuous-rotation servo

Servo servo;

// ---------------- Test parameters ----------------
const int distances[]  = {5, 10, 20, 30, 45, 60, 90, 120, 150}; // degrees from home
const int stepDelays[] = {0, 5, 10, 20};                        // ms/degree; 0 = jump (servo's own max speed)
const int contSpeeds[] = {50, 100, 150, 200, 300, 400, 500};    // us offset from 1500 (stop), both directions
const int REPEATS = 3;

const uint32_t MOVE_WINDOW_MS = 700;   // minimum analysis window per event
const uint32_t GAP_MS         = 900;   // silence after the window, covers the return move
const uint32_t SETTLE_MS      = 100;   // margin either side of a move

// Home sits near one end of travel so HOME_ANGLE + max(distances) stays inside
// the servo's range. At the old HOME_ANGLE of 90 the three largest distances
// asked for 180/210/240 deg; everything past 180 is unreachable on a positional
// servo, so those events commanded nothing and the servo sat still at home while
// the plan still claimed a move had happened.
const int HOME_ANGLE      = 15;
const int SERVO_MIN_ANGLE = 0;
const int SERVO_MAX_ANGLE = 180;

// Logical events, used only for the JSON plan the PC needs for slicing.
struct LogEvent { char type; int p1; int p2; uint32_t start_ms; uint32_t dur_ms; };
LogEvent logEvents[300];
int logLen = 0;

// Raw hardware commands, purely time-triggered, no blocking.
// Sized for the worst case: 3 repeats x 9 distances x (3 stepped rates x
// (distance+2) steps + 1 jump-rate x 2 steps) = ~4986 for the positional
// sweep as configured below. Bumped with headroom since this silently
// overflowed into other globals at 2000 and looked like a hardware fault.
struct Cmd { uint32_t time_ms; bool isServoWrite; int angleOrUs; bool isMicroseconds; };
Cmd cmds[6000];
int cmdLen = 0;
uint32_t totalDuration = 0;

// Stop dead and say why. A sweep that can't be executed as planned must not
// record as though it was: a silently-skipped move still produces an event in
// the plan, and the PC side then reads servo noise that never happened.
void halt(const char *msg, int value) {
  while (true) {
    Serial.printf("FATAL: %s (%d)\n", msg, value);
    Serial.flush();
    delay(1000);
  }
}

void addCmdAngle(uint32_t t, int angle) {
  if (angle < SERVO_MIN_ANGLE || angle > SERVO_MAX_ANGLE)
    halt("commanded angle outside servo range, sweep would record a move that never happened", angle);
  if (cmdLen >= (int)(sizeof(cmds)/sizeof(cmds[0])))
    halt("cmds array full, reduce the sweep or raise its size rather than letting it wrap", cmdLen);
  cmds[cmdLen++] = {t, true, angle, false};
}
void addCmdMicros(uint32_t t, int us) {
  if (cmdLen >= (int)(sizeof(cmds)/sizeof(cmds[0])))
    halt("cmds array full, reduce the sweep or raise its size rather than letting it wrap", cmdLen);
  cmds[cmdLen++] = {t, true, us, true};
}

void buildPlan() {
  uint32_t t = 500; // lead-in silence

  for (int r = 0; r < REPEATS; r++) {
    if (TEST_POSITIONAL) {
      for (int d = 0; d < (int)(sizeof(distances)/sizeof(int)); d++) {
        for (int s = 0; s < (int)(sizeof(stepDelays)/sizeof(int)); s++) {
          int target = HOME_ANGLE + distances[d];
          int stepDelay = stepDelays[s];

          // A stepped move takes distance x stepDelay ms, which for the slower
          // rates is several times MOVE_WINDOW_MS. The window has to be sized
          // from the move, not the other way round: a fixed window leaves step
          // commands still pending when the next event is due, so events fire
          // late while the plan still claims the original start_ms, and the PC
          // side slices the wrong audio for every event after the first overrun.
          uint32_t moveDur = (stepDelay == 0) ? 0 : (uint32_t)distances[d] * stepDelay;
          uint32_t window  = max(MOVE_WINDOW_MS, moveDur + SETTLE_MS);

          if (logLen >= (int)(sizeof(logEvents)/sizeof(logEvents[0])))
            halt("logEvents array full, raise its size", logLen);
          logEvents[logLen++] = {'P', distances[d], stepDelay, t, window};

          if (stepDelay == 0) {
            addCmdAngle(t, target);
          } else {
            for (int a = HOME_ANGLE, i = 0; a <= target; a++, i++) {
              addCmdAngle(t + i * (uint32_t)stepDelay, a);
            }
          }
          // Return home after the analysis window closes, so the return move's
          // own noise lands in the gap instead of the segment being measured.
          addCmdAngle(t + window + SETTLE_MS, HOME_ANGLE);
          uint32_t evEnd = t + window + GAP_MS;
          t = evEnd;
        }
      }
    }
    if (TEST_CONTINUOUS) {
      for (int s = 0; s < (int)(sizeof(contSpeeds)/sizeof(int)); s++) {
        for (int sign = -1; sign <= 1; sign += 2) {
          int us = 1500 + sign * contSpeeds[s];
          if (logLen >= (int)(sizeof(logEvents)/sizeof(logEvents[0])))
            halt("logEvents array full, raise its size", logLen);
          logEvents[logLen++] = {'C', sign * contSpeeds[s], 0, t, MOVE_WINDOW_MS + 300};
          addCmdMicros(t, us);
          uint32_t evEnd = t + MOVE_WINDOW_MS + 300 + GAP_MS;
          addCmdMicros(evEnd - GAP_MS, 1500); // stop
          t = evEnd;
        }
      }
    }
  }
  totalDuration = t + 500; // trailing silence

  // loop() walks cmds[] forward only, firing anything whose time has passed, so
  // a command sitting out of order fires at the wrong moment without any sign
  // that it did. Claim the ordering rather than assume it survived the maths.
  for (int i = 1; i < cmdLen; i++) {
    if (cmds[i].time_ms < cmds[i - 1].time_ms)
      halt("cmds are not in time order, event windows must be overlapping", i);
  }

  // Every move must finish inside the window the plan reports for it, or the
  // PC side measures a segment the servo was still moving through.
  for (int i = 0; i < logLen; i++) {
    if (logEvents[i].type != 'P' || logEvents[i].p2 == 0) continue;
    uint32_t moveDur = (uint32_t)logEvents[i].p1 * logEvents[i].p2;
    if (moveDur > logEvents[i].dur_ms)
      halt("a move outlasts its own analysis window", i);
  }
}

void printPlanJson() {
  // Report the rate actually compiled in. Hardcoding it here meant the PC side
  // could slice a 32 kHz stream as though it were 16 kHz and never know.
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

  Serial.println("=== servo note-scan test v" VERSION " ===");
  Serial.println("Servo move sweep recorded on an I2S mic: JSON plan, then raw 16-bit PCM");
  Serial.println("https://github.com/sui001/ruckus");

  servo.setPeriodHertz(50);
  servo.attach(SERVO_PIN, 1000, 2000);
  // On a continuous-rotation servo write(angle) is a speed, not a position:
  // write(HOME_ANGLE) at 15 would spin it hard from boot until the first
  // command, straight through the lead-in silence the analysis relies on.
  if (TEST_CONTINUOUS && !TEST_POSITIONAL) servo.writeMicroseconds(1500);
  else                                     servo.write(HOME_ANGLE);

  i2sInit();
  buildPlan();
  printPlanJson();
}

int cmdIdx = 0;
uint32_t t0;
bool started = false;
bool done = false;

void loop() {
  if (!started) { t0 = millis(); started = true; }
  uint32_t now = millis() - t0;

  // Fire any due hardware commands. Each is a single non-blocking call.
  while (cmdIdx < cmdLen && now >= cmds[cmdIdx].time_ms) {
    Cmd &c = cmds[cmdIdx];
    if (c.isMicroseconds) servo.writeMicroseconds(c.angleOrUs);
    else                  servo.write(c.angleOrUs);
    cmdIdx++;
  }

  // Stream mic audio continuously regardless of what the servo is doing.
  static int32_t raw[128];
  size_t bytesRead = 0;
  i2s_read(I2S_PORT, raw, sizeof(raw), &bytesRead, 0); // non-blocking (0 tick wait)
  int n = bytesRead / sizeof(int32_t);
  if (n > 0) {
    static int16_t out[128];
    for (int i = 0; i < n; i++) {
      int32_t s = raw[i] >> MIC_SHIFT;
      // Clamp rather than let the cast wrap: a sample past full scale would
      // otherwise flip polarity and read as a spike, which looks like noise
      // in the spectrum instead of looking like the overload it is.
      if (s >  32767) s =  32767;
      if (s < -32768) s = -32768;
      out[i] = (int16_t)s;
    }
    Serial.write((uint8_t*)out, n * sizeof(int16_t));
  }

  if (!done && now > totalDuration + 1000) {
    done = true;
    servo.writeMicroseconds(1500);
  }
}
