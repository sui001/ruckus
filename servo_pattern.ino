// servo_pattern.ino
// ESP32-S3 Supermini + one positional hobby servo (SG92R). No microphone.
//
// A percussion sequencer for one servo, with two voices.
//
// Measured first (see README): the noise sits at a fixed ~9 kHz whatever is
// commanded, so there is no pitch to play. Two things do vary. Loudness tracks
// move size (44/79/148/246 RMS for 1/2/3/5 degree steps), and direction changes
// the timbre: from about 6 degrees up, an upward move is clearly brighter than a
// downward one, like a high and a low tom. So a hit has a velocity and a voice.
//
// Protocol over serial at 115200, one command per line:
//   T <bpm>      tempo, 20-900 (sixteenths above 750 outrun the servo)
//   P <pattern>  one character per sixteenth, up to 64:
//                  h  hi hit (upward move)      l  lo hit (downward move)
//                  x  hit, alternating hi/lo    1-9 alternating hit at that velocity
//                  .  or -  rest
//   V <deg>      velocity in degrees, 1-20. 6-12 is where hi/lo is clearest.
//   S            start        X  stop
//   ?            status
//
// Wiring: servo signal -> GPIO8, powered from the board's 5V pin, common ground.

#include <ESP32Servo.h>

#define VERSION "1.1"
#define SERVO_PIN 8

// The servo only accepts a new position once per PWM frame. Asking for hits
// closer together than this does not play them faster, it silently drops them.
const uint32_t PWM_FRAME_MS = 20;

const int CENTRE_ANGLE = 90;
const int SAFE_MIN     = 30;   // stay well clear of the end stops
const int SAFE_MAX     = 150;
const int MAX_VELOCITY = 20;
const int MAX_SLOTS    = 64;

Servo servo;

char pattern[MAX_SLOTS + 1] = "l...h...l.l.h...";
int  patternLen = 16;
int  bpm = 120;
int  defaultVelocity = 8;
bool running = false;

int  slot = 0;
int  lastDir = -1;             // so the first alternating hit goes up
int  pos = CENTRE_ANGLE;
uint32_t nextSlotAt = 0;

uint32_t slotIntervalMs() {
  // Four slots to the beat: a sixteenth at the given tempo.
  return (uint32_t)(60000.0 / (double)bpm / 4.0);
}

void announceTiming() {
  uint32_t iv = slotIntervalMs();
  Serial.printf("tempo %d bpm, %lu ms per slot\n", bpm, (unsigned long)iv);
  if (iv < PWM_FRAME_MS) {
    Serial.printf("WARNING: %lu ms is shorter than the servo's %lu ms PWM frame. "
                  "Hits will be dropped rather than played faster. "
                  "Max usable tempo at sixteenths is %d bpm.\n",
                  (unsigned long)iv, (unsigned long)PWM_FRAME_MS,
                  (int)(60000.0 / 4.0 / (double)PWM_FRAME_MS));
  }
}

void moveTo(int target) {
  if (target < SAFE_MIN) target = SAFE_MIN;
  if (target > SAFE_MAX) target = SAFE_MAX;
  servo.write(target);
  pos = target;
}

// d = +1 for the hi voice (upward), -1 for the lo voice (downward). A run of
// one voice walks the servo away from centre and rests walk it back, so a
// pattern needs the odd rest or the other voice to stay inside its travel.
// At the edge of travel a hit clamps and goes quiet rather than jumping.
void hit(int d, int velocity) {
  if (velocity < 1) velocity = 1;
  if (velocity > MAX_VELOCITY) velocity = MAX_VELOCITY;
  moveTo(pos + d * velocity);
  lastDir = d;
}

void printStatus() {
  Serial.printf("pattern [%s] len %d, tempo %d bpm, velocity %d, at %d deg, %s\n",
                pattern, patternLen, bpm, defaultVelocity, pos,
                running ? "running" : "stopped");
}

bool validPatternChar(char c) {
  return c=='x'||c=='X'||c=='h'||c=='H'||c=='l'||c=='L'||c=='.'||c=='-'||(c>='1'&&c<='9');
}

void handleLine(char *line) {
  while (*line == ' ') line++;
  char c = *line;
  char *arg = line + 1;
  while (*arg == ' ') arg++;

  switch (c) {
    case 'T': case 't': {
      // The ceiling is deliberately past the point the servo can keep up:
      // sixteenths above ~750 bpm ask for hits closer than the 20 ms PWM
      // frame, and hearing that limit is more use than being fenced off it.
      int v = atoi(arg);
      if (v < 20 || v > 900) { Serial.println("err: tempo must be 20-900"); return; }
      bpm = v;
      announceTiming();
      Serial.println("ok");
      return;
    }
    case 'V': case 'v': {
      int v = atoi(arg);
      if (v < 1 || v > MAX_VELOCITY) {
        Serial.printf("err: velocity must be 1-%d\n", MAX_VELOCITY); return;
      }
      defaultVelocity = v;
      Serial.println("ok");
      return;
    }
    case 'P': case 'p': {
      int n = 0;
      for (char *p = arg; *p && n < MAX_SLOTS; p++) {
        if (*p == ' ') continue;
        if (validPatternChar(*p)) pattern[n++] = *p;
        else { Serial.printf("err: bad character '%c', use h l x . - or 1-9\n", *p); return; }
      }
      if (n == 0) { Serial.println("err: empty pattern"); return; }
      pattern[n] = '\0';
      patternLen = n;
      slot = 0;
      Serial.println("ok");
      printStatus();
      return;
    }
    case 'S': case 's':
      running = true; slot = 0; nextSlotAt = millis();
      announceTiming();
      Serial.println("ok running");
      return;
    case 'X':
      running = false;
      moveTo(CENTRE_ANGLE);
      Serial.println("ok stopped");
      return;
    case '?':
      printStatus();
      return;
    case '\0':
      return;
    default:
      Serial.printf("err: unknown command '%c', try T P V S X or ?\n", c);
      return;
  }
}

void setup() {
  Serial.begin(115200);
  delay(300);

  Serial.println("=== servo pattern sequencer v" VERSION " ===");
  Serial.println("Two-voice servo percussion (h = hi/up, l = lo/down), pattern over serial");
  Serial.println("https://github.com/sui001/ruckus");

  servo.setPeriodHertz(50);
  servo.attach(SERVO_PIN, 1000, 2000);
  moveTo(CENTRE_ANGLE);

  printStatus();
  Serial.println("ok ready");
}

void loop() {
  static char buf[128];
  static int len = 0;
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (len > 0) { buf[len] = '\0'; handleLine(buf); len = 0; }
    } else if (len < (int)sizeof(buf) - 1) {
      buf[len++] = c;
    }
  }

  if (!running) return;

  uint32_t now = millis();
  if ((int32_t)(now - nextSlotAt) < 0) return;

  char c = pattern[slot];
  if (c == 'h' || c == 'H')      hit(+1, defaultVelocity);
  else if (c == 'l' || c == 'L') hit(-1, defaultVelocity);
  else if (c == 'x' || c == 'X') hit(-lastDir, defaultVelocity);
  else if (c >= '1' && c <= '9') hit(-lastDir, c - '0');
  else if (pos != CENTRE_ANGLE)  moveTo(pos + (pos < CENTRE_ANGLE ? 1 : -1));

  slot = (slot + 1) % patternLen;
  nextSlotAt += slotIntervalMs();
  // If we have fallen behind (tempo faster than the servo can be driven),
  // do not try to catch up by firing a burst of hits.
  if ((int32_t)(now - nextSlotAt) > 0) nextSlotAt = now + slotIntervalMs();
}
