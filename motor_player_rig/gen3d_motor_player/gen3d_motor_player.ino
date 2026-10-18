// Motor player: the servos as instruments.
//   Melody: up to 8 continuous servos on MOTOR_CH (ch0-7). Each one's whine
//   rises with speed. At boot each motor is tuned alone, in both directions:
//   stepped through forward and reverse pulses while the INMP441 measures its
//   pitch, giving a pulse-for-pitch table per direction. A note goes to
//   whichever direction reaches it (reverse preferred). A motor that can't be
//   heard in either direction is skipped.
//   Notes rotate round the motors so each has time to spin up, and repeated
//   notes articulate. +U puts a note on every motor at once, +P with it
//   sends every other motor a fifth up (power chord).
//   Drums: MG996R ch11 and 180 ch15, each hit a full-speed jump.
//
// Song format, space separated tokens, <what>:<ms>
//   E6:150      note (A-G, optional # or b, octave)
//   R:150       rest
//   E6+K:150    note plus MG996R hit, +S for the 180, +U all motors, +P power
//   K:150       drum only
//
// Autoplay: 5 s after boot it starts "all" and loops it, so it runs standalone
// off a USB charger. Any serial input in those 5 s cancels; "auto off" saves
// it off. Each song prints a NOW PLAYING banner, on serial and on the OLED.
// Buttons: NEXT skips to the next song, BACK to the previous one, wrapping
// round the set. Both together restart the set from song 1 (or start it when
// idle). Both held 5 s stop it. When idle, a single press also starts it.
//
// Serial commands:
//   all           every song below, back to back (stop ends the set)
//   auto on|off   autoplay at boot, saved to flash
//   kick <deg>    MG996R drum swing either side of centre, 0 = off (default 10, saved)
//   snare <deg>   180 drum swing, 0 = off (default 45, saved)
//   spring        Vivaldi, Spring (Four Seasons), opening theme
//   king         Grieg, Hall of the Mountain King, long accelerating version
//   round         Frere Jacques as a 4-part round, each part a pair of motors
//                 (out of the "all" set, the overlapping parts read as noise)
//   ode           Beethoven, Ode to Joy
//   fifth         Beethoven, Symphony No. 5 opening
//   tetris        Korobeiniki, the Tetris tune
//   tell          Rossini, William Tell overture gallop
//   cancan        Offenbach, the Can-can
//   bee           Rimsky-Korsakov, Flight of the Bumblebee, as a gliding swarm
//   metal         an original thrash-style riff, all motors as power chords
//   funeral       Chopin, Funeral March
//   wedding       Wagner, Bridal Chorus (Here comes the bride)
//   valkyrie      Wagner, Ride of the Valkyries, inside one octave
//   toccata       Bach, Toccata in D minor opening, ending on a D minor chord
//   phase         Convergence, an original: motors wander, meet in a chord,
//                 drift apart and spin down
// Dropped after listening: Fur Elise, the Nokia tune, The Entertainer,
// Morning Mood. Out of the "all" set but still playable by name: round,
// tetris (too close to the Mountain King), tell, bee (a bee, not the tune),
// wedding, toccata.
//   play <song>   play a song line
//   scale         chromatic octave from the root, to hear the tuning
//   note <n>      hold a note on the first motor, e.g. note E6 (stop to end)
//   cal           retune every motor, both directions (keep quiet). The tuning
//                 is saved to flash and loaded at boot, so this is only needed
//                 when motors change or move.
//   voices <n>    play on only the first n tuned motors (voices 1 = solo)
//   snr <dB>      tuning gate over the room, default 1.5, then cal
//   gap <pct>     silence at the end of each note, default 0
//   stop          stop everything
//
// Wiring:
//   INMP441: SCK GPIO11, WS GPIO12, SD GPIO13, L/R GND, VCC 3V3
//   PCA9685: SDA GPIO10, SCL GPIO9, VCC 3V3, V+ unconnected, servos on bench supply
//   OLED:    1.5" GME128128 (SH1107, 128x128), SDA GPIO4, SCL GPIO3, VCC 3V3
//   Buttons: NEXT GPIO6, BACK GPIO7, each to GND

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <driver/i2s.h>
#include <arduinoFFT.h>
#include <Preferences.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>

#define VERSION "0.16.0"

// Front panel: 1.5" SH1107 OLED on its own I2C bus (Wire1), so a slow or
// missing screen can never hold up the PCA9685's bus. Two buttons to GND,
// internal pull-ups.
#define PIN_OLED_SDA 4
#define PIN_OLED_SCL 3
#define PIN_NEXT     6
#define PIN_BACK     7

#define PIN_SCK 11
#define PIN_WS  12
#define PIN_SD  13
#define PIN_SDA 10
#define PIN_SCL 9
#define CH_MG   11
#define CH_180  15
#define N_MOTORS 8
const int MOTOR_CH[N_MOTORS] = {0, 1, 2, 3, 4, 5, 6, 7};

#define SAMPLE_RATE 16000
#define N_SAMPLES   2048
#define BAND_LO     700       // search for the whine above room rumble
#define BAND_HI     1800      // below its 2nd harmonic, which v0.2.0 tuned to by mistake
#define MIN_RANGE_SEMIS 5     // a direction with less tuned range than this is dropped
#define CAL_STEP    20
#define MAX_CAL     20

// Pulses swept by the tuner. Pitch stops rising past about 360 us from stop.
// Starting 40 us from stop (was 100) reaches the slow, quiet, low notes,
// which the 1.5 dB gate can now hear.
#define REV_FROM 1460
#define REV_TO   1100
#define FWD_FROM 1540
#define FWD_TO   1900
#define TUNING_FORMAT 2       // bump when the sweep changes, forces a retune at boot

// dB over room for a tuning point to count. In the 700-1800 Hz band a motor
// only sits 2-5 dB over the room, so 6 dB rejected every motor in v0.2.1.
// Kept low: the rising-run filter and MIN_RANGE_SEMIS reject dead motors.
float calSnr = 1.5f;

static int32_t raw[N_SAMPLES];
static float vReal[N_SAMPLES];
static float vImag[N_SAMPLES];
ArduinoFFT<float> FFT(vReal, vImag, N_SAMPLES, (float)SAMPLE_RATE);
Adafruit_PWMServoDriver pwm(0x40);
Preferences prefs;

struct Table { int n; float hz[MAX_CAL]; int us[MAX_CAL]; };
struct Voice { int ch; Table dir[2]; };   // dir[0] reverse, dir[1] forward

// Tunes are semitones over the root and lengths in eighths. REST is a rest.
// Declared up here because the Arduino builder puts function prototypes
// (addPass takes a const N*) above anything defined further down.
struct N { int8_t off; int8_t len; };
#define REST 99
Voice voices[N_MOTORS];
int live[N_MOTORS], nLive = 0, rr = 0;
int gapPct = 0;
int maxVoices = N_MOTORS;
bool autoPlay = true;         // loaded from flash in setup
bool autoRunning = false;

Adafruit_SH1107 oled(128, 128, &Wire1);   // GME128128: 1.5" 128x128, SH1107
bool haveOled = false;
int jump = 0;                 // set by a button press: +1 next, -1 back
int jumpTo = -1;              // both pressed: restart the set from this song
bool stopReq = false;         // both held 5 s: stop the set
bool buttonsOk = true;        // false if a button reads pressed at boot (stuck or miswired)

extern bool stopFlag;         // declared further down with the other play state

// Three lines of status on the screen (boot, tuning, idle, stopped). No
// default arguments: the Arduino builder's own prototype would clash.
void oledStatus(const char *a, const char *b, const char *c) {
  if (!haveOled) return;
  oled.clearDisplay();
  oled.setTextColor(SH110X_WHITE);
  oled.setTextSize(1);
  oled.setCursor(0, 0);   oled.print("gen3d motor player");
  oled.drawLine(0, 10, 127, 10, SH110X_WHITE);
  oled.setTextSize(2);
  oled.setCursor(0, 20);  oled.print(a);
  oled.setTextSize(1);
  oled.setCursor(0, 60);  oled.print(b);
  oled.setCursor(0, 76);  oled.print(c);
  oled.setCursor(0, 118); oled.print("v" VERSION);
  oled.display();
}

// Buttons, acted on at release so a two-button press can be told from one:
//   NEXT alone -> jump +1, BACK alone -> jump -1
//   both       -> restart the set from song 1 (or start it when idle)
//   both held 5 s -> stop, fired while still held, with a countdown on screen
// Any action ends the song playing (stopFlag) and leaves the request for the
// set to act on. A press must last 30 ms to count, which rides out bounce.
bool pollButtons() {
  static bool down = false, both = false, held = false;
  static uint32_t downAt = 0, bothAt = 0, upAt = 0;
  static int first = 0, shown = -1;
  if (!buttonsOk) return false;
  bool n = digitalRead(PIN_NEXT) == LOW, b = digitalRead(PIN_BACK) == LOW;
  uint32_t now = millis();
  if (n || b) {
    if (!down) {
      if (now - upAt < 30) return false;      // bounce right after a release
      down = true; both = false; held = false; downAt = now; shown = -1;
      first = n ? 1 : -1;
    }
    if (n && b) {
      if (!both) { both = true; bothAt = now; }
      uint32_t h = now - bothAt;
      if (!held && h >= 1000 && haveOled && (int)(h / 1000) != shown) {
        shown = h / 1000;
        char c[24]; snprintf(c, sizeof(c), "stopping in %d s", 5 - shown);
        oledStatus("both buttons held", c, "let go to cancel");
      }
      if (!held && h >= 5000) {
        held = true; stopReq = true; stopFlag = true;
        return true;
      }
    } else if (both && !held) {
      bothAt = now;                           // one let go: the 5 s hold starts again
    }
    return false;
  }
  if (!down) return false;
  down = false; upAt = now;
  if (held || now - downAt < 30) return false;
  if (both) jumpTo = 0;
  else jump = first;
  stopFlag = true;
  return true;
}
bool mgHi = false, s180Hi = false;
bool stopFlag = false;

const char *NOTES[] = {"C", "C#", "D", "D#", "E", "F", "F#", "G", "G#", "A", "A#", "B"};
const char *DIR_NAME[] = {"rev", "fwd"};

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

// One block: band level (relative dB) and the peak frequency inside the band.
void listenBlock(float &bandDb, float &hz) {
  size_t got;
  i2s_read(I2S_NUM_0, raw, sizeof(raw), &got, portMAX_DELAY);
  double mean = 0;
  for (int i = 0; i < N_SAMPLES; i++) { vReal[i] = (float)(raw[i] >> 8); mean += vReal[i]; }
  mean /= N_SAMPLES;
  for (int i = 0; i < N_SAMPLES; i++) { vReal[i] -= mean; vImag[i] = 0; }
  FFT.windowing(FFTWindow::Hann, FFTDirection::Forward);
  FFT.compute(FFTDirection::Forward);
  FFT.complexToMagnitude();
  int lo = BAND_LO * N_SAMPLES / SAMPLE_RATE, hi = BAND_HI * N_SAMPLES / SAMPLE_RATE;
  int k = lo;
  double e = 0;
  for (int i = lo; i <= hi; i++) { e += (double)vReal[i] * vReal[i]; if (vReal[i] > vReal[k]) k = i; }
  float a = vReal[k - 1], b = vReal[k], c = vReal[k + 1], d = a - 2 * b + c;
  hz = (k + (d != 0 ? 0.5f * (a - c) / d : 0)) * SAMPLE_RATE / N_SAMPLES;
  bandDb = 10.0f * log10f(e + 1e-9);
}

float median3(float a, float b, float c) { return max(min(a, b), min(max(a, b), c)); }

void listen3(float &bandDb, float &hz) {
  float b[3], h[3];
  for (int i = 0; i < 3; i++) listenBlock(b[i], h[i]);
  bandDb = median3(b[0], b[1], b[2]);
  hz = median3(h[0], h[1], h[2]);
}

void quietMotors() { for (int i = 0; i < N_MOTORS; i++) pwm.writeMicroseconds(MOTOR_CH[i], 1500); }
void allOff() {
  for (int i = 0; i < N_MOTORS; i++) pwm.setPWM(MOTOR_CH[i], 0, 4096);
  pwm.setPWM(CH_MG, 0, 4096);
  pwm.setPWM(CH_180, 0, 4096);
}

// Sweep one direction and build its table. Pitch should rise as the pulse
// moves away from 1500. Keep the longest rising run of points (longest
// increasing subsequence), so stray readings are dropped without one bad
// early point throwing away the good ones after it.
bool tuneTable(int ch, Table &t, int from, int to, float room) {
  int step = to > from ? CAL_STEP : -CAL_STEP;
  float hz[MAX_CAL]; int us[MAX_CAL]; int n = 0;
  pwm.writeMicroseconds(ch, 1500); delay(500);
  for (int u = from; (step > 0 ? u <= to : u >= to) && n < MAX_CAL; u += step) {
    pwm.writeMicroseconds(ch, u);
    delay(400);
    float bandDb, h;
    listen3(bandDb, h);
    if (bandDb - room >= calSnr) { us[n] = u; hz[n] = h; n++; }
  }
  pwm.writeMicroseconds(ch, 1500); delay(400);

  int len[MAX_CAL], prev[MAX_CAL], best = -1;
  for (int i = 0; i < n; i++) {
    len[i] = 1; prev[i] = -1;
    for (int j = 0; j < i; j++)
      if (hz[i] > hz[j] + 5 && len[j] + 1 > len[i]) { len[i] = len[j] + 1; prev[i] = j; }
    if (best < 0 || len[i] > len[best]) best = i;
  }
  int keep[MAX_CAL], m = 0;
  for (int i = best; i >= 0; i = prev[i]) keep[m++] = i;
  for (int i = 0; i < m; i++) { t.hz[i] = hz[keep[m - 1 - i]]; t.us[i] = us[keep[m - 1 - i]]; }
  t.n = m;
  if (t.n < 3 || 12.0f * log2f(t.hz[t.n - 1] / t.hz[0]) < MIN_RANGE_SEMIS) { t.n = 0; return false; }
  return true;
}

float loHz(const Voice &v) {
  float lo = 1e9;
  for (int d = 0; d < 2; d++) if (v.dir[d].n) lo = min(lo, v.dir[d].hz[0]);
  return lo;
}
float hiHz(const Voice &v) {
  float hi = 0;
  for (int d = 0; d < 2; d++) if (v.dir[d].n) hi = max(hi, v.dir[d].hz[v.dir[d].n - 1]);
  return hi;
}

void printVoice(const Voice &v) {
  Serial.printf("ch%d", v.ch);
  for (int d = 0; d < 2; d++) {
    const Table &t = v.dir[d];
    if (t.n) Serial.printf("  %s %.0f..%.0f Hz (%d pts)", DIR_NAME[d], t.hz[0], t.hz[t.n - 1], t.n);
    else Serial.printf("  %s -", DIR_NAME[d]);
  }
  if (v.dir[0].n || v.dir[1].n) Serial.printf("  -> %.1f semitones\n", 12.0f * log2f(hiHz(v) / loHz(v)));
  else Serial.println("  -> skipped, can't hear it");
}

// Tuning lives in flash so a reflash doesn't cost 3.5 minutes of retuning.
// Keyed on the struct size and motor count, so a layout change forces a cal.
void saveTuning() {
  prefs.begin("tune", false);
  prefs.putInt("fmt", TUNING_FORMAT);
  prefs.putInt("n", N_MOTORS);
  prefs.putBytes("v", voices, sizeof(voices));
  prefs.end();
}

bool loadTuning() {
  prefs.begin("tune", true);
  bool ok = prefs.getInt("fmt", 0) == TUNING_FORMAT && prefs.getInt("n", 0) == N_MOTORS
            && prefs.getBytesLength("v") == sizeof(voices);
  if (ok) prefs.getBytes("v", voices, sizeof(voices));
  prefs.end();
  if (!ok) return false;
  nLive = 0;
  Serial.println("tuning loaded from flash (send cal to retune):");
  for (int i = 0; i < N_MOTORS; i++) {
    printVoice(voices[i]);
    if (voices[i].dir[0].n || voices[i].dir[1].n) live[nLive++] = i;
  }
  Serial.printf("%d motor(s) playing\n", nLive);
  return nLive > 0;
}

void calibrate() {
  Serial.println("tuning each motor alone, both directions: keep quiet");
  nLive = 0;
  for (int i = 0; i < N_MOTORS; i++) {
    Voice &v = voices[i];
    v.ch = MOTOR_CH[i];
    allOff(); delay(400);
    float room, junk;
    listen3(room, junk);
    bool r = tuneTable(v.ch, v.dir[0], REV_FROM, REV_TO, room);
    bool f = tuneTable(v.ch, v.dir[1], FWD_FROM, FWD_TO, room);
    allOff();
    printVoice(v);
    if (r || f) live[nLive++] = i;
  }
  Serial.printf("%d motor(s) playing\n", nLive);
  saveTuning();
}

int usInTable(const Table &t, float hz) {
  hz = constrain(hz, t.hz[0], t.hz[t.n - 1]);
  for (int i = 1; i < t.n; i++)
    if (hz <= t.hz[i]) {
      float a = (hz - t.hz[i - 1]) / (t.hz[i] - t.hz[i - 1]);
      return (int)lroundf(t.us[i - 1] + a * (t.us[i] - t.us[i - 1]));
    }
  return t.us[t.n - 1];
}

// Pulse for a frequency on one motor. Octave-shift into the combined range of
// both directions, then use a direction that covers it (reverse first), else
// the nearer end.
int usForHz(const Voice &v, float hz) {
  float lo = loHz(v), hi = hiHz(v);
  while (hz > hi && hz / 2 >= lo) hz /= 2;
  while (hz < lo && hz * 2 <= hi) hz *= 2;
  for (int d = 0; d < 2; d++) {
    const Table &t = v.dir[d];
    if (t.n && hz >= t.hz[0] && hz <= t.hz[t.n - 1]) return usInTable(t, hz);
  }
  int bestD = -1; float bestGap = 1e9;
  for (int d = 0; d < 2; d++) {
    const Table &t = v.dir[d];
    if (!t.n) continue;
    float gap = hz < t.hz[0] ? t.hz[0] - hz : hz - t.hz[t.n - 1];
    if (gap < bestGap) { bestGap = gap; bestD = d; }
  }
  return usInTable(v.dir[bestD], hz);
}

// "E6", "C#6", "Bb5" -> Hz, or 0 if it doesn't parse.
float noteHz(const String &s) {
  const int semis[] = {9, 11, 0, 2, 4, 5, 7};   // A B C D E F G
  if (s.length() < 2) return 0;
  char L = toupper(s[0]);
  if (L < 'A' || L > 'G') return 0;
  int semi = semis[L - 'A'], i = 1;
  if (s[i] == '#') { semi++; i++; } else if (s[i] == 'b') { semi--; i++; }
  if (i >= (int)s.length() || !isdigit(s[i])) return 0;
  int midi = 12 * (s[i] - '0' + 1) + semi;
  return 440.0f * powf(2.0f, (midi - 69) / 12.0f);
}

String midiName(int m) { return String(NOTES[m % 12]) + String(m / 12 - 1); }
float midiHz(int m) { return 440.0f * powf(2.0f, (m - 69) / 12.0f); }

// Drum hit: jump either side of centre by the drum's swing. Bigger swing is a
// louder hit. The MG996R at the old full 100 deg swing drowned the melody.
// About 11 us per degree on these servos.
int kickDeg = 10, snareDeg = 45;   // loaded from flash in setup

void hit(int ch, bool &hiSide) {
  int deg = ch == CH_MG ? kickDeg : snareDeg;
  if (deg <= 0) return;
  hiSide = !hiSide;
  int us = deg * 11;
  pwm.writeMicroseconds(ch, hiSide ? 1500 + us : 1500 - us);
}

// Waits, but a "stop" on serial or a button press ends the song.
bool waitMs(uint32_t ms) {
  uint32_t t = millis();
  while (millis() - t < ms) {
    if (pollButtons()) return false;
    if (Serial.available()) {
      String l = Serial.readStringUntil('\n'); l.trim();
      if (l == "stop") { stopFlag = true; return false; }
    }
    delay(1);
  }
  return true;
}

int nPlay() { return min(nLive, maxVoices); }

void playToken(const String &tok) {
  int colon = tok.lastIndexOf(':');
  if (colon < 0) { Serial.printf("bad token '%s'\n", tok.c_str()); return; }
  int ms = tok.substring(colon + 1).toInt();
  String what = tok.substring(0, colon);
  bool k = what.indexOf("+K") >= 0 || what == "K";
  bool s = what.indexOf("+S") >= 0 || what == "S";
  bool u = what.indexOf("+U") >= 0;
  bool pw = what.indexOf("+P") >= 0;
  int plus = what.indexOf('+');
  String note = plus >= 0 ? what.substring(0, plus) : what;
  if (note == "K" || note == "S") note = "R";

  if (k) hit(CH_MG, mgHi);
  if (s) hit(CH_180, s180Hi);
  if (note == "R" || nLive == 0) { quietMotors(); waitMs(ms); return; }
  float hz = noteHz(note);
  if (hz <= 0) { Serial.printf("bad note '%s'\n", note.c_str()); return; }

  quietMotors();
  if (u) for (int i = 0; i < nPlay(); i++) {
    Voice &v = voices[live[i]];
    pwm.writeMicroseconds(v.ch, usForHz(v, (pw && i % 2) ? hz * 1.4983f : hz));   // 1.4983 = a fifth
  }
  else { Voice &v = voices[live[rr++ % nPlay()]]; pwm.writeMicroseconds(v.ch, usForHz(v, hz)); }
  int on = ms * (100 - gapPct) / 100;
  if (!waitMs(on)) return;
  if (on < ms) { quietMotors(); waitMs(ms - on); }
}

void play(const String &song) {
  stopFlag = false;
  rr = 0;
  quietMotors();
  pwm.writeMicroseconds(CH_MG, 1500);
  pwm.writeMicroseconds(CH_180, 1500);
  delay(300);
  int i = 0;
  while (i < (int)song.length() && !stopFlag) {
    while (i < (int)song.length() && song[i] == ' ') i++;
    int j = song.indexOf(' ', i);
    if (j < 0) j = song.length();
    if (j > i) playToken(song.substring(i, j));
    i = j;
  }
  quietMotors(); delay(200);
  allOff();
  Serial.println(stopFlag ? "stopped" : "done");
}

// Root from the first live motor (ch0, nearest the mic), lowest of both
// directions. The others fold the notes into their own ranges.
int rootMidi() { return (int)ceilf(69.0f + 12.0f * log2f(loHz(voices[live[0]]) / 440.0f)); }

// One pass of a tune into song tokens. Drums on the beat grid, and flags for
// unison and power chords.
void addPass(String &s, const N *tune, int len, int root, int e, bool kick, bool snare, bool uni, bool power) {
  int beat = 0;
  for (int i = 0; i < len; i++) {
    const N &n = tune[i];
    String t = n.off == REST ? String("R") : midiName(root + n.off);
    if (kick && beat % 8 == 0) t += "+K";
    if (snare && beat % 8 == 4) t += "+S";
    if (uni && n.off != REST) t += power ? "+U+P" : "+U";
    s += t + ":" + String(e * n.len) + " ";
    beat += n.len;
  }
}

bool needMotors() {
  if (nLive == 0) { Serial.println("no motors tuned, run cal"); return false; }
  return true;
}

// Vivaldi, Spring, from The Four Seasons (public domain), E major in the original.
const N SPRING[] = {
  {4,2},{4,2},{4,2},{7,2}, {7,2},{4,2},{7,2},{11,2}, {11,2},{9,2},{7,2},{5,2},{4,4},
};

// Once plain, once with drums and the ensemble echoing in power chords, the
// way the piece itself alternates a solo line with the full orchestra.
void spring() {
  if (!needMotors()) return;
  int root = rootMidi();
  String s;
  addPass(s, SPRING, sizeof(SPRING) / sizeof(N), root, 210, false, false, false, false);
  addPass(s, SPRING, sizeof(SPRING) / sizeof(N), root, 190, true, true, true, true);
  Serial.printf("spring: go, do = %s, %d motor(s)\n", midiName(root).c_str(), nPlay());
  play(s);
}

// Grieg, In the Hall of the Mountain King (public domain), B minor in the original.
const N KING[] = {
  {0,1},{2,1},{3,1},{5,1},{7,1},{3,1},{7,2},
  {6,1},{2,1},{6,2},{5,1},{1,1},{5,2},
  {0,1},{2,1},{3,1},{5,1},{7,1},{3,1},{7,1},{12,1},
  {10,1},{7,1},{3,1},{7,1},{10,4},
};

// Ten passes alternating the home key and a fifth up, as Grieg moves it,
// accelerating from stately to frantic. Drums build in, the last two passes
// are all motors as power chords, then a hammered ending.
void king() {
  if (!needMotors()) return;
  int root = rootMidi();
  const int keys[] = {0, 0, 7, 7, 0, 0, 7, 7, 0, 0};
  String s;
  for (int p = 0; p < 10; p++) {
    int e = 280 - (280 - 85) * p / 9;
    addPass(s, KING, sizeof(KING) / sizeof(N), root + keys[p], e, p >= 2, p >= 4, p >= 8, true);
  }
  String r = midiName(root);
  for (int i = 0; i < 3; i++) s += r + "+U+P+K+S:220 R:120 ";
  s += r + "+U+P+K+S:900";
  Serial.printf("hall of the mountain king: go, root %s, %d motor(s)\n", r.c_str(), nPlay());
  play(s);
}

// Beethoven, Ode to Joy (public domain). Do = 0, the low So is -5.
const N ODE[] = {
  {4,2},{4,2},{5,2},{7,2}, {7,2},{5,2},{4,2},{2,2}, {0,2},{0,2},{2,2},{4,2}, {4,3},{2,1},{2,4},
  {4,2},{4,2},{5,2},{7,2}, {7,2},{5,2},{4,2},{2,2}, {0,2},{0,2},{2,2},{4,2}, {2,3},{0,1},{0,4},
  {2,2},{2,2},{4,2},{0,2}, {2,2},{4,1},{5,1},{4,2},{0,2}, {2,2},{4,1},{5,1},{4,2},{2,2}, {0,2},{2,2},{-5,4},
  {4,2},{4,2},{5,2},{7,2}, {7,2},{5,2},{4,2},{2,2}, {0,2},{0,2},{2,2},{4,2}, {2,3},{0,1},{0,4},
};

// Once plain, once with drums and the whole ensemble in power chords.
void ode() {
  if (!needMotors()) return;
  int root = rootMidi() + 5;       // puts the low So (-5) at the bottom of the range
  String s;
  addPass(s, ODE, sizeof(ODE) / sizeof(N), root, 210, false, false, false, false);
  addPass(s, ODE, sizeof(ODE) / sizeof(N), root, 190, true, true, true, true);
  Serial.printf("ode to joy: go, do = %s, %d motor(s)\n", midiName(root).c_str(), nPlay());
  play(s);
}

// Beethoven, Symphony No. 5 opening (public domain), C minor: G G G Eb, F F F D.
const N FIFTH[] = {
  {REST,1},{7,1},{7,1},{7,1},{3,12},
  {REST,2},{5,1},{5,1},{5,1},{2,16},
};

// Four times, every motor at once. The three short notes are 220 ms, each
// with a snare click and a short silence after it: at 150 ms with no gap the
// motors never reached speed and the da-da-da smeared into one sound.
// The long note is the stress: power chord on every motor from the first
// pass, both drums as it lands, held its full length. Breaks kept short.
void fifth() {
  if (!needMotors()) return;
  int root = rootMidi();
  const int SHORT = 220, GAP = 40;
  String s;
  for (int p = 0; p < 4; p++) {
    for (const N &n : FIFTH) {
      if (n.off == REST) { s += "R:" + String(SHORT * n.len / 2) + " "; continue; }
      if (n.len == 1) s += midiName(root + n.off) + "+U+S:" + String(SHORT) + " R:" + String(GAP) + " ";
      else s += midiName(root + n.off) + "+U+P+K+S:" + String(SHORT * n.len * 3 / 4) + " ";
    }
    s += "R:250 ";
  }
  Serial.printf("fifth: go, C = %s, %d motor(s)\n", midiName(root).c_str(), nPlay());
  play(s);
}

// Korobeiniki (Russian folk song, public domain), the Tetris tune. A = 0.
const N TETRIS[] = {
  {7,2},{2,1},{3,1},{5,2},{3,1},{2,1}, {0,2},{0,1},{3,1},{7,2},{5,1},{3,1},
  {2,3},{3,1},{5,2},{7,2}, {3,2},{0,2},{0,2},{REST,2},
  {REST,1},{5,2},{8,1},{12,2},{10,1},{8,1}, {7,3},{3,1},{7,2},{5,1},{3,1},
  {2,2},{2,1},{3,1},{5,2},{7,2}, {3,2},{0,2},{0,2},{REST,2},
};

// Three times round, faster each time, drums from the second, power chords last.
void tetris() {
  if (!needMotors()) return;
  int root = rootMidi();
  String s;
  const int e[] = {200, 160, 130};
  for (int p = 0; p < 3; p++)
    addPass(s, TETRIS, sizeof(TETRIS) / sizeof(N), root, e[p], p >= 1, p >= 1, p == 2, true);
  Serial.printf("tetris: go, A = %s, %d motor(s)\n", midiName(root).c_str(), nPlay());
  play(s);
}

// Rossini, William Tell overture, the gallop (public domain). G = 0, lengths
// in sixteenths here: the da-da-DUM rhythm is two sixteenths and an eighth.
const N TELL[] = {
  {0,1},{0,1},{0,2}, {0,1},{0,1},{0,2}, {0,1},{0,1},{5,2},{7,2},{9,4},
  {0,1},{0,1},{0,2}, {0,1},{0,1},{0,2}, {0,1},{0,1},{5,2},{9,2},{7,2},{4,4},
  {0,1},{0,1},{0,2}, {0,1},{0,1},{0,2}, {0,1},{0,1},{5,2},{7,2},{9,4},
  {0,1},{0,1},{0,2}, {0,1},{0,1},{0,2}, {0,1},{0,1},{5,2},{9,2},{7,2},{4,2},{5,8},
};

// Three gallops, each faster. Drums on the beat from the second, snare on the
// offbeat and power chords on the last.
void tell() {
  if (!needMotors()) return;
  int root = rootMidi();
  const int sixteenth[] = {110, 90, 75};
  String s;
  for (int p = 0; p < 3; p++) {
    int beat = 0;
    for (const N &n : TELL) {
      String t = midiName(root + n.off);
      if (p >= 1 && beat % 4 == 0) t += "+K";
      if (p >= 2 && beat % 4 == 2) t += "+S";
      if (p == 2) t += "+U+P";
      s += t + ":" + String(sixteenth[p] * n.len) + " ";
      beat += n.len;
    }
    s += "R:" + String(sixteenth[p] * 4) + " ";
  }
  s += midiName(root + 5) + "+U+P+K+S:700";
  Serial.printf("william tell: go, G = %s, %d motor(s)\n", midiName(root).c_str(), nPlay());
  play(s);
}

// Offenbach, Galop infernal, the Can-can (public domain), C = 0, eighths.
const N CANCAN[] = {
  {0,4}, {2,1},{5,1},{4,1},{2,1}, {7,2},{7,2}, {7,1},{9,1},{4,1},{5,1}, {2,2},{2,2},
  {2,1},{5,1},{4,1},{2,1}, {0,1},{12,1},{11,1},{9,1}, {7,1},{5,1},{4,1},{2,1},
  {0,4}, {2,1},{5,1},{4,1},{2,1}, {7,2},{7,2}, {7,1},{9,1},{4,1},{5,1}, {2,2},{2,2},
  {2,1},{5,1},{4,1},{2,1}, {0,1},{7,1},{2,1},{4,1}, {0,2},{REST,2},
};

// Three times, faster each time, kicks from the second, all-in on the third.
void cancan() {
  if (!needMotors()) return;
  int root = rootMidi();
  const int e[] = {170, 140, 115};
  String s;
  for (int p = 0; p < 3; p++)
    addPass(s, CANCAN, sizeof(CANCAN) / sizeof(N), root, e[p], p >= 1, p >= 1, p == 2, true);
  Serial.printf("can-can: go, C = %s, %d motor(s), loud kick\n", midiName(root).c_str(), nPlay());
  // The Can-can's drive was the kick. When the MG996R was turned down to
  // 10 deg for everything else, this lost its bounce, so it gets 45 back.
  int saved = kickDeg;
  kickDeg = 45;
  play(s);
  kickDeg = saved;
}

// Rimsky-Korsakov, Flight of the Bumblebee, opening (public domain), A = 0,
// sixteenths. Chromatic swirls down from E and back, then the same from A.
const N BEE_E[] = { {7,1},{6,1},{5,1},{4,1},{3,1},{8,1},{7,1},{6,1}, {7,1},{6,1},{5,1},{4,1},{3,1},{4,1},{5,1},{6,1} };
const N BEE_A[] = { {12,1},{11,1},{10,1},{9,1},{8,1},{13,1},{12,1},{11,1}, {12,1},{11,1},{10,1},{9,1},{8,1},{9,1},{10,1},{11,1} };

// Pulse for a frequency held inside the motor's own range, no octave folding.
// For glides, where a fold would be a jump.
int usInRange(const Voice &v, float hz) { return usForHz(v, constrain(hz, loHz(v), hiHz(v))); }

// Glide every motor through a tune in unison, sliding in pitch from each
// note to the next instead of stepping. Motors slide naturally but can't
// step faster than ~150 ms, so fast tunes read better as glides.
bool glide(const N *tune, int len, int root, int unitMs) {
  for (int i = 0; i < len; i++) {
    float a = log2f(midiHz(root + tune[i].off));
    float b = log2f(midiHz(root + tune[(i + 1) % len].off));
    uint32_t dur = (uint32_t)tune[i].len * unitMs, t0 = millis();
    while (millis() - t0 < dur) {
      float f = (millis() - t0) / (float)dur;
      float hz = powf(2.0f, a + (b - a) * f);
      for (int k = 0; k < nPlay(); k++) { Voice &v = voices[live[k]]; pwm.writeMicroseconds(v.ch, usInRange(v, hz)); }
      if (!waitMs(10)) return false;
    }
  }
  return true;
}

// The whole swarm sliding through the swirls in unison: E twice, A twice,
// E twice, a slow dive down, and a spin-down.
void bee() {
  if (!needMotors()) return;
  int root = rootMidi() - 3;       // lowest note used is C (3), so C lands at the bottom
  const int x = 95;
  const int nE = sizeof(BEE_E) / sizeof(N), nA = sizeof(BEE_A) / sizeof(N);
  const N DIVE[] = { {13,4},{3,4} };
  Serial.printf("bumblebee: go, gliding swarm, A = %s, %d motor(s)\n", midiName(root).c_str(), nPlay());
  stopFlag = false;
  quietMotors(); delay(300);
  bool ok = glide(BEE_E, nE, root, x) && glide(BEE_E, nE, root, x)
         && glide(BEE_A, nA, root, x) && glide(BEE_A, nA, root, x)
         && glide(BEE_E, nE, root, x) && glide(BEE_E, nE, root, x)
         && glide(DIVE, 2, root, 300);
  (void)ok;
  quietMotors(); delay(200);
  allOff();
  Serial.println(stopFlag ? "stopped" : "done");
}

// Hz for one motor's share of a chord: its preferred chord tone, or the first
// other tone (or octave of one) that sits inside its range.
float chordToneHz(const Voice &v, int root, const int *iv, int niv, int pref) {
  for (int oct = 0; oct <= 1; oct++)
    for (int j = 0; j < niv; j++) {
      int tone = iv[(pref + j) % niv];
      for (int sh : {0, -12, 12}) {
        if (oct == 0 && sh != 0) continue;
        float hz = midiHz(root + tone + sh);
        if (hz >= loHz(v) && hz <= hiHz(v)) return hz;
      }
    }
  return midiHz(root + iv[pref % niv]);
}

// Hold a real chord, the motors dealt out across its tones.
void holdChord(int root, const int *iv, int niv, uint32_t ms, bool drums) {
  if (drums) { hit(CH_MG, mgHi); hit(CH_180, s180Hi); }
  for (int k = 0; k < nPlay(); k++) {
    Voice &v = voices[live[k]];
    pwm.writeMicroseconds(v.ch, usInRange(v, chordToneHz(v, root, iv, niv, k)));
  }
  waitMs(ms);
}

// An original riff in a thrash style, not a transcription of anyone's song.
// Palm-muted gallops (eighth, two sixteenths) on the root, chromatic stabs,
// kick on every beat and snare on 2 and 4, everything as power chords.
void metal() {
  if (!needMotors()) return;
  int root = rootMidi();
  const int x = 85;                // sixteenth ms
  // Semitone offsets, lengths in sixteenths. Gallop = {0,2},{0,1},{0,1}.
  const N RIFF[] = {
    {0,2},{0,1},{0,1}, {0,2},{0,1},{0,1}, {1,4}, {0,2},{0,1},{0,1}, {3,4},
    {0,2},{0,1},{0,1}, {0,2},{0,1},{0,1}, {5,2},{4,2}, {3,2},{1,2},
    {0,2},{0,1},{0,1}, {0,2},{0,1},{0,1}, {1,4}, {0,2},{0,1},{0,1}, {6,4},
    {5,2},{3,2}, {1,2},{0,2}, {REST,4}, {0,8},
  };
  String s;
  for (int p = 0; p < 4; p++) {
    int beat = 0;
    for (const N &n : RIFF) {
      String t = n.off == REST ? String("R") : midiName(root + n.off) + "+U+P";
      if (beat % 4 == 0 && n.off != REST) t += "+K";
      if (beat % 16 == 4 || beat % 16 == 12) t += "+S";
      s += t + ":" + String(x * n.len) + " ";
      beat += n.len;
    }
  }
  s += midiName(root) + "+U+P+K+S:300 R:150 " + midiName(root) + "+U+P+K+S:1000";
  Serial.printf("metal: go, E = %s, %d motor(s)\n", midiName(root).c_str(), nPlay());
  play(s);
}

// Play with a silence at the end of every note, so repeated notes start
// cleanly from a stop instead of smearing (what went wrong with the Fifth).
void playGapped(const String &s, int pct) {
  int old = gapPct;
  gapPct = pct;
  play(s);
  gapPct = old;
}

// Chopin, Piano Sonata No. 2, the Funeral March (public domain). Bb = 0,
// sixteenths. Slow, low and narrow: dum, dum da-dum, dum.
const N FUNERAL[] = {
  {0,4},{0,3},{0,1},{0,4}, {3,3},{2,1},{2,3},{0,1},{0,3},{-1,1},{0,8},
  {0,4},{0,3},{0,1},{0,4}, {3,3},{2,1},{2,3},{0,1},{0,3},{-1,1},{0,8},
};

void funeral() {
  if (!needMotors()) return;
  int root = rootMidi() + 1;       // A (-1) at the bottom
  String s;
  addPass(s, FUNERAL, sizeof(FUNERAL) / sizeof(N), root, 170, true, false, false, false);
  addPass(s, FUNERAL, sizeof(FUNERAL) / sizeof(N), root, 170, true, false, true, true);
  Serial.printf("funeral march: go, Bb = %s, %d motor(s)\n", midiName(root).c_str(), nPlay());
  playGapped(s, 20);
}

// Wagner, Lohengrin, Bridal Chorus, 1850 (public domain): "Here comes the
// bride". Replaces the Mendelssohn fanfare, whose fast repeated triplets
// smeared on motors. C = 0, eighths. Slow, clear, fits in 10 semitones.
const N WEDDING[] = {
  {0,2},{5,3},{5,1},{5,4}, {0,2},{7,3},{4,1},{5,4},
  {0,2},{5,3},{10,1},{10,2},{9,3},{7,1},{5,2},{4,3},{5,1},{7,4},
};

void wedding() {
  if (!needMotors()) return;
  int root = rootMidi();
  String s;
  addPass(s, WEDDING, sizeof(WEDDING) / sizeof(N), root, 230, false, false, false, false);
  addPass(s, WEDDING, sizeof(WEDDING) / sizeof(N), root, 230, true, false, true, true);
  Serial.printf("bridal chorus: go, C = %s, %d motor(s)\n", midiName(root).c_str(), nPlay());
  playGapped(s, 15);
  const int major[] = {0, 4, 7};
  holdChord(root + 5, major, 3, 1800, true);   // land on an F major chord
  quietMotors(); delay(200); allOff();
}

// Wagner, Ride of the Valkyries, 1856 (public domain): the rising call,
// kept inside one octave (F#..F#) so no note folds and the shape survives.
// B = 0, sixteenths. da-da DAA da DAA.
const N VALKYRIE[] = {
  {-5,1},{0,2},{-5,1},{0,2},{3,6},  {0,1},{3,2},{0,1},{3,2},{7,6},
  {-5,1},{0,2},{-5,1},{0,2},{3,6},  {0,1},{3,2},{0,1},{3,2},{7,12},
};

// Third and last attempt (stepped notes failed twice). The rhythm goes on
// the drums, which can do it: a snare click on every short note, both drums
// on the long ones. The motors glide the melody in unison underneath,
// sliding into each note over 120 ms and holding, so the shape is heard even
// though motors can't step that fast.
void valkyrie() {
  if (!needMotors()) return;
  int root = rootMidi() + 5;       // low F# (-5) at the bottom, top F# (7) an octave up
  const int len = sizeof(VALKYRIE) / sizeof(N);
  const int x[] = {140, 130, 120};
  Serial.printf("ride of the valkyries: go, drums + glide, B = %s, %d motor(s)\n", midiName(root).c_str(), nPlay());
  stopFlag = false;
  quietMotors();
  pwm.writeMicroseconds(CH_MG, 1500);
  pwm.writeMicroseconds(CH_180, 1500);
  delay(300);
  float cur = log2f(midiHz(root + VALKYRIE[0].off));
  for (int p = 0; p < 3 && !stopFlag; p++) {
    for (int i = 0; i < len && !stopFlag; i++) {
      const N &n = VALKYRIE[i];
      uint32_t dur = (uint32_t)n.len * x[p];
      if (n.len >= 6) hit(CH_MG, mgHi);
      hit(CH_180, s180Hi);
      float tgt = log2f(midiHz(root + n.off));
      uint32_t slide = min<uint32_t>(120, dur / 2), t0 = millis();
      while (millis() - t0 < dur) {
        float f = min(1.0f, (millis() - t0) / (float)slide);
        float hz = powf(2.0f, cur + (tgt - cur) * ease(f));
        for (int k = 0; k < nPlay(); k++) { Voice &v = voices[live[k]]; pwm.writeMicroseconds(v.ch, usInRange(v, hz)); }
        if (!waitMs(10)) break;
      }
      cur = tgt;
    }
  }
  quietMotors(); delay(200);
  allOff();
  Serial.println(stopFlag ? "stopped" : "done");
}

// Bach, Toccata and Fugue in D minor (public domain): the organ opening,
// slower so the mordent can be heard, every motor as an organ stop in power
// chords, ending on a real D minor chord across the motors.
const N TOCCATA[] = {
  {7,1},{5,1},{7,10},{REST,3}, {5,1},{3,1},{2,1},{0,1},{-1,4},{0,10},{REST,4},
};

void toccata() {
  if (!needMotors()) return;
  int root = rootMidi() + 1;       // C# (-1) at the bottom
  String s;
  addPass(s, TOCCATA, sizeof(TOCCATA) / sizeof(N), root, 180, false, false, true, true);
  addPass(s, TOCCATA, sizeof(TOCCATA) / sizeof(N), root, 180, true, false, true, true);
  Serial.printf("toccata: go, D = %s, %d motor(s)\n", midiName(root).c_str(), nPlay());
  playGapped(s, 8);
  const int minor[] = {0, 3, 7};
  holdChord(root, minor, 3, 3000, true);
  quietMotors(); delay(200); allOff();
}

// Convergence: an original for these motors. They enter one at a time,
// wandering in pitch at their own rates, slide together into a chord, hold
// it, drift out of tune, then glide down and spin to a stop one by one.
float ease(float x) { x = constrain(x, 0.0f, 1.0f); return x * x * (3 - 2 * x); }

void phaseSong() {
  if (!needMotors()) return;
  int n = nPlay();
  int root = rootMidi();
  const int chord[] = {0, 4, 7, 12};
  float lo[N_MOTORS], hi[N_MOTORS], target[N_MOTORS], period[N_MOTORS], ph[N_MOTORS], drift[N_MOTORS];
  uint32_t enterAt[N_MOTORS], exitAt[N_MOTORS];
  randomSeed(esp_random());
  const uint32_t STAGGER = 1800;
  const uint32_t ENTER_END = n * STAGGER + 1500, SCATTER_END = ENTER_END + 8000;
  const uint32_t CONV_END = SCATTER_END + 12000, HOLD_END = CONV_END + 9000, DRIFT_END = HOLD_END + 8000;
  for (int i = 0; i < n; i++) {
    Voice &v = voices[live[i]];
    lo[i] = loHz(v); hi[i] = hiHz(v);
    target[i] = chordToneHz(v, root, chord, 4, i);
    period[i] = random(2500, 7500);
    ph[i] = random(0, 628) / 100.0f;
    drift[i] = (random(0, 2) ? 1 : -1) * (0.7f + random(0, 100) / 100.0f);   // semitones
    enterAt[i] = i * STAGGER + random(0, 600);
    exitAt[i] = DRIFT_END + (n - 1 - i) * 1500 + random(0, 700);            // leave in reverse order
  }
  const uint32_t END = DRIFT_END + n * 1500 + 6500;
  Serial.printf("convergence: go, chord on %s, %d motor(s), about %lu s\n", midiName(root).c_str(), n, END / 1000);
  stopFlag = false;
  quietMotors(); delay(300);
  const char *stage = "";
  uint32_t t0 = millis();
  while (!stopFlag) {
    uint32_t t = millis() - t0;
    if (t > END) break;
    const char *now = t < ENTER_END ? "entering" : t < SCATTER_END ? "wandering" : t < CONV_END ? "converging"
                    : t < HOLD_END ? "harmony" : t < DRIFT_END ? "drifting" : "spinning down";
    if (now != stage) { stage = now; Serial.printf("  %s\n", stage); }
    if (t >= CONV_END && t < CONV_END + 30) { hit(CH_180, s180Hi); }          // one click as they lock
    for (int i = 0; i < n; i++) {
      Voice &v = voices[live[i]];
      if (t < enterAt[i]) { pwm.writeMicroseconds(v.ch, 1500); continue; }
      float tt = 0.5f + 0.45f * sinf(6.2832f * t / period[i] + ph[i]);
      float wander = lo[i] * powf(hi[i] / lo[i], tt);
      float k = t < SCATTER_END ? 0 : ease((t - SCATTER_END) / (float)(CONV_END - SCATTER_END));
      float hz = powf(2.0f, log2f(wander) * (1 - k) + log2f(target[i]) * k);
      if (t > HOLD_END) hz *= powf(2.0f, drift[i] * ease((t - HOLD_END) / (float)(DRIFT_END - HOLD_END)) / 12.0f);
      int us;
      if (t > exitAt[i]) {
        float g = (t - exitAt[i]) / 3000.0f;
        if (g < 1) us = usInRange(v, powf(2.0f, log2f(hz) * (1 - ease(g)) + log2f(lo[i]) * ease(g)));
        else {
          int usLo = usInRange(v, lo[i]);
          float s = min(1.0f, (t - exitAt[i] - 3000) / 2500.0f);
          us = (int)(usLo + (1500 - usLo) * s);                            // spin down to a stop
        }
      } else us = usInRange(v, hz);
      float a = min(1.0f, (t - enterAt[i]) / 1500.0f);                      // spin up on entry
      pwm.writeMicroseconds(v.ch, (int)(1500 + (us - 1500) * ease(a)));
    }
    if (!waitMs(20)) break;
  }
  quietMotors(); delay(200);
  allOff();
  Serial.println(stopFlag ? "stopped" : "done");
}

// Frere Jacques (traditional, public domain). The low G (-5) folds up an octave
// on most motors.
const N FJ[] = {
  {0,2},{2,2},{4,2},{0,2}, {0,2},{2,2},{4,2},{0,2},
  {4,2},{5,2},{7,4}, {4,2},{5,2},{7,4},
  {7,1},{9,1},{7,1},{5,1},{4,2},{0,2}, {7,1},{9,1},{7,1},{5,1},{4,2},{0,2},
  {0,2},{-5,2},{0,4}, {0,2},{-5,2},{0,4},
};
const int FJ_LEN = sizeof(FJ) / sizeof(FJ[0]);

// A 4-part round: each part enters 2 bars (16 eighths) after the last and
// sings the tune twice. Live motors are dealt out to the parts, and within a
// part the motors take turns so each gets time to spin up.
void roundSong() {
  if (!needMotors()) return;
  const int e = 190;
  int parts = min(4, nPlay());
  int root = rootMidi();
  struct Part { int m[N_MOTORS]; int nm, idx, turn, cur; uint32_t next; };
  Part P[4];
  for (int p = 0; p < parts; p++) { P[p].nm = 0; P[p].idx = 0; P[p].turn = 0; P[p].cur = -1; }
  for (int i = 0; i < nPlay(); i++) { Part &q = P[i % parts]; q.m[q.nm++] = live[i]; }
  uint32_t t0 = millis() + 300;
  for (int p = 0; p < parts; p++) P[p].next = t0 + (uint32_t)p * 16 * e;
  Serial.printf("round: go, root %s, %d parts from %d motor(s)\n", midiName(root).c_str(), parts, nPlay());
  quietMotors();
  stopFlag = false;
  int doneParts = 0;
  while (doneParts < parts && !stopFlag) {
    uint32_t now = millis();
    for (int p = 0; p < parts; p++) {
      Part &q = P[p];
      if (q.idx > 2 * FJ_LEN || now < q.next) continue;
      if (q.cur >= 0) pwm.writeMicroseconds(voices[q.cur].ch, 1500);
      if (q.idx == 2 * FJ_LEN) { q.idx++; doneParts++; continue; }   // sung twice, fall silent
      const N &n = FJ[q.idx % FJ_LEN];
      Voice &v = voices[q.m[q.turn++ % q.nm]];
      pwm.writeMicroseconds(v.ch, usForHz(v, midiHz(root + n.off)));
      if (q.idx % 8 == 0 && p == 0) hit(CH_MG, mgHi);             // a light pulse under part 1
      q.cur = &v - voices;
      q.next += (uint32_t)n.len * e;
      q.idx++;
    }
    if (!waitMs(1)) break;
  }
  quietMotors(); delay(200);
  allOff();
  Serial.println(stopFlag ? "stopped" : "done");
}

struct Song { void (*fn)(); const char *title; const char *by; };
const Song SONGS[] = {
  {spring,    "Spring, opening theme",            "Vivaldi, 1723"},
  {king,      "In the Hall of the Mountain King", "Grieg, 1875"},
  {ode,       "Ode to Joy",                       "Beethoven, 1824"},
  {fifth,     "Symphony No. 5, opening",          "Beethoven, 1808"},
  {cancan,    "Galop infernal (the Can-can)",     "Offenbach, 1858"},
  {funeral,   "Funeral March",                    "Chopin, 1839"},
  {valkyrie,  "Ride of the Valkyries",            "Wagner, 1856"},
  {metal,     "Motor Thrash (original riff)",     "this machine, 2026"},
  {phaseSong, "Convergence (original)",           "this machine, 2026"},
};
const int N_SONGS = sizeof(SONGS) / sizeof(SONGS[0]);

void banner(int i) {
  Serial.println();
  Serial.println("========================================");
  Serial.printf("  NOW PLAYING  %d/%d\n", i + 1, N_SONGS);
  Serial.printf("  %s\n", SONGS[i].title);
  Serial.printf("  %s\n", SONGS[i].by);
  Serial.println("========================================");
  if (!haveOled) return;
  oled.clearDisplay();
  oled.setTextColor(SH110X_WHITE);
  oled.setTextSize(1);
  oled.setTextWrap(true);
  oled.setCursor(0, 0);   oled.printf("NOW PLAYING    %d/%d", i + 1, N_SONGS);
  oled.drawLine(0, 10, 127, 10, SH110X_WHITE);
  oled.setTextSize(2);                             // 10 characters a line, wraps
  oled.setCursor(0, 16);  oled.print(SONGS[i].title);
  oled.setTextSize(1);
  oled.setCursor(0, 96);  oled.print(SONGS[i].by);
  oled.drawLine(0, 108, 127, 108, SH110X_WHITE);
  oled.setCursor(0, 118); oled.print("< back        next >");
  oled.display();
}

// Every song back to back, 3 s apart. "stop" ends the whole set; the
// buttons jump to the next or previous song, wrapping round the set.
void allSongs() {
  int i = 0;
  jump = 0; jumpTo = -1; stopReq = false;
  while (i < N_SONGS) {
    banner(i);
    stopFlag = false;
    SONGS[i].fn();
    if (!jump && jumpTo < 0 && !stopFlag) waitMs(3000);   // the gap between songs, a button can cut it short
    if (stopReq) break;
    if (jumpTo >= 0) { i = jumpTo; jumpTo = -1; stopFlag = false; continue; }
    if (jump) { i = (i + jump + N_SONGS) % N_SONGS; jump = 0; stopFlag = false; continue; }
    if (stopFlag) break;
    i++;
  }
  stopReq = false;
  Serial.println(stopFlag ? "set stopped" : "set done");
  oledStatus(stopFlag ? "stopped" : "set done", "press both buttons", "to play");
}

void handle(String line) {
  line.trim();
  if (line == "all") allSongs();
  else if (line == "btn") {
    // Live button readout for 10 s, whatever buttonsOk says, then re-arm
    // the buttons if both read released at the end.
    Serial.println("btn: press the buttons, 10 s");
    int lastN = -1, lastB = -1;
    uint32_t t = millis();
    while (millis() - t < 10000) {
      int n = digitalRead(PIN_NEXT), b = digitalRead(PIN_BACK);
      if (n != lastN || b != lastB) {
        Serial.printf("  NEXT %s   BACK %s\n", n == LOW ? "PRESSED " : "released", b == LOW ? "PRESSED " : "released");
        lastN = n; lastB = b;
      }
      delay(10);
    }
    buttonsOk = digitalRead(PIN_NEXT) == HIGH && digitalRead(PIN_BACK) == HIGH;
    Serial.printf("btn done, buttons %s\n", buttonsOk ? "enabled" : "still ignored (one reads pressed)");
  }
  else if (line.startsWith("kick ") || line.startsWith("snare ")) {
    bool k = line.startsWith("kick ");
    int deg = constrain(line.substring(k ? 5 : 6).toInt(), 0, 50);
    (k ? kickDeg : snareDeg) = deg;
    prefs.begin("tune", false); prefs.putInt(k ? "kick" : "snare", deg); prefs.end();
    Serial.printf("%s swing -> %d deg%s (saved)\n", k ? "kick (MG996R)" : "snare (180)", deg, deg ? "" : ", off");
  }
  else if (line == "auto on" || line == "auto off") {
    autoPlay = line == "auto on";
    prefs.begin("tune", false); prefs.putBool("auto", autoPlay); prefs.end();
    Serial.printf("autoplay at boot -> %s\n", autoPlay ? "on" : "off");
  }
  else if (line == "spring") spring();
  else if (line == "king") king();
  else if (line == "round") roundSong();
  else if (line == "ode") ode();
  else if (line == "fifth") fifth();
  else if (line == "tetris") tetris();
  else if (line == "tell") tell();
  else if (line == "cancan") cancan();
  else if (line == "bee") bee();
  else if (line == "metal") metal();
  else if (line == "funeral") funeral();
  else if (line == "wedding") wedding();
  else if (line == "valkyrie") valkyrie();
  else if (line == "toccata") toccata();
  else if (line == "phase") phaseSong();
  else if (line.startsWith("play ")) { Serial.println("playing"); play(line.substring(5)); }
  else if (line == "scale") {
    if (!needMotors()) return;
    String s;
    int root = rootMidi();
    for (int n = 0; n <= 12; n++) s += midiName(root + n) + ":350 ";
    play(s);
  }
  else if (line.startsWith("note ")) {
    float hz = noteHz(line.substring(5));
    if (hz <= 0 || nLive == 0) { Serial.println("bad note, or no motors tuned"); return; }
    Voice &v = voices[live[0]];
    int us = usForHz(v, hz);
    pwm.writeMicroseconds(v.ch, us);
    Serial.printf("ch%d holding %s = %.0f Hz at %d us, send stop\n", v.ch, line.substring(5).c_str(), hz, us);
  }
  else if (line == "cal") calibrate();
  else if (line.startsWith("snr ")) {
    calSnr = line.substring(4).toFloat();
    Serial.printf("tuning gate -> %.1f dB over room, send cal to retune\n", calSnr);
  }
  else if (line.startsWith("voices ")) {
    maxVoices = constrain(line.substring(7).toInt(), 1, N_MOTORS);
    Serial.printf("voices -> %d (of %d tuned)\n", maxVoices, nLive);
  }
  else if (line.startsWith("gap ")) { gapPct = constrain(line.substring(4).toInt(), 0, 90); Serial.printf("gap -> %d%%\n", gapPct); }
  else if (line == "stop") { quietMotors(); delay(200); allOff(); Serial.println("stopped"); }
  else if (line.length()) Serial.println("? all | king | round | ode | fifth | tetris | tell | cancan | bee | funeral | wedding | valkyrie | toccata | metal | phase | play <song> | scale | note <n> | cal | voices <n> | snr <dB> | gap <pct> | stop");
}

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(20);
  Serial.setTxTimeoutMs(0);   // no computer attached: drop prints instead of stalling the music
  delay(1500);
  Serial.println("=== motor player v" VERSION " ===");
  Serial.println("Servos as instruments: up to 8 continuous motors on ch0-7, tuned both directions by mic, MG996R + 180 drums");
  Serial.println("https://github.com/sui001/ruckus/tree/main/motor_player_rig/gen3d_motor_player");

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

  pinMode(PIN_NEXT, INPUT_PULLUP);
  pinMode(PIN_BACK, INPUT_PULLUP);
  delay(20);
  // Nobody is pressing a button at power-on, so a pin reading pressed now is
  // stuck: typically a 4-leg tactile switch wired across one internally
  // joined pair, which shorts the pin to GND. Ignore the buttons rather than
  // let a phantom "both held" stop the set.
  bool nStuck = digitalRead(PIN_NEXT) == LOW, bStuck = digitalRead(PIN_BACK) == LOW;
  Serial.printf("buttons at boot: NEXT (GPIO%d) %s, BACK (GPIO%d) %s\n",
                PIN_NEXT, nStuck ? "PRESSED (stuck?)" : "released",
                PIN_BACK, bStuck ? "PRESSED (stuck?)" : "released");
  if (nStuck || bStuck) {
    buttonsOk = false;
    Serial.println("buttons ignored: one reads pressed with nobody touching it. Check the switch legs, then btn to test");
  }
  Wire1.begin(PIN_OLED_SDA, PIN_OLED_SCL);
  Wire1.setTimeOut(50);
  Wire1.beginTransmission(0x3C);
  if (Wire1.endTransmission() == 0 && oled.begin(0x3C, true)) {
    haveOled = true;
    oled.setRotation(0);                      // SH1107 panel already reads the right way up on the rig
    Serial.println("OLED ok at 0x3C (SH1107 128x128)");
    oledStatus("v" VERSION, "starting...", "");
  } else {
    Serial.println("no OLED at 0x3C on SDA 4 / SCL 3, carrying on without it");
  }

  setupI2S();
  float a, b;
  listenBlock(a, b); listenBlock(a, b);   // settling reads
  if (!loadTuning()) { oledStatus("tuning motors", "keep quiet", "about 4 min"); calibrate(); }
  {
    char line[24];
    snprintf(line, sizeof(line), "%d motors tuned", nLive);
    oledStatus("ready", line, "press a button");
  }
  Serial.println("ready: all | auto on|off | kick <deg> | snare <deg> | spring | king | round | ode | fifth | tetris | tell | cancan | bee | funeral | wedding | valkyrie | toccata | metal | phase | play <song> | scale | note <n> | cal | voices <n> | snr <dB> | gap <pct> | stop");

  prefs.begin("tune", true);
  autoPlay = prefs.getBool("auto", true);
  kickDeg = prefs.getInt("kick", kickDeg);
  snareDeg = prefs.getInt("snare", snareDeg);
  prefs.end();
  Serial.printf("drums: kick %d deg, snare %d deg\n", kickDeg, snareDeg);
  if (autoPlay && nLive > 0) {
    Serial.println("autoplay in 5 s, send anything to cancel");
    uint32_t t = millis();
    while (millis() - t < 5000 && !Serial.available()) delay(10);
    if (Serial.available()) Serial.println("autoplay cancelled");
    else autoRunning = true;
  }
}

void loop() {
  if (Serial.available()) {
    autoRunning = false;
    handle(Serial.readStringUntil('\n'));
  }
  if (!autoRunning && pollButtons()) {         // idle: a press starts the set, a 5 s hold does nothing
    bool start = !stopReq;
    jump = 0; jumpTo = -1; stopReq = false;
    stopFlag = false;
    autoRunning = start;
  }
  if (autoRunning) {
    allSongs();
    if (stopFlag) { autoRunning = false; Serial.println("autoplay stopped, send all or reset to resume"); }
    else if (!waitMs(10000)) { autoRunning = false; Serial.println("autoplay stopped"); }
  }
  delay(5);
}
