// PCA9685 bring-up on an ESP32-S3 SuperMini.
// Ch0 = continuous rotation servo, ch15 = 180 degree servo.
// SDA = GPIO10, SCL = GPIO9, VCC = 3V3, servo power on the terminal block.
//
// Runs a demo loop on boot. Any serial command stops the demo:
//   c <us>    continuous servo pulse, e.g. c 1500 (stop), c 2000, c 1000 (full speed)
//   s <deg>   180 servo angle, 0..180
//   demo      restart the demo loop
//   off       cut the pulse to both channels

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

#define VERSION "0.3.0"

#define PIN_SDA 10
#define PIN_SCL 9
#define CH_CONT 0
#define CH_MG   11   // MG996R, on its own supply
#define CH_180  15

// Typical hobby servo range. Trim if the 180 buzzes at the ends.
#define US_MIN  500
#define US_MAX  2500
#define US_STOP 1500   // continuous servo neutral, trim with "c" if it creeps

Adafruit_PWMServoDriver pwm(0x40);
bool demo = true;
String line;

void setAngle(int deg) {
  deg = constrain(deg, 0, 180);
  pwm.writeMicroseconds(CH_180, map(deg, 0, 180, US_MIN, US_MAX));
  Serial.printf("180 servo -> %d deg\n", deg);
}

// MG996R pulled in from the ends, most bind or buzz at 500/2500.
void setMG(int deg) {
  deg = constrain(deg, 0, 180);
  pwm.writeMicroseconds(CH_MG, map(deg, 0, 180, 600, 2400));
  Serial.printf("MG996R -> %d deg\n", deg);
}

void setCont(int us) {
  us = constrain(us, 1000, 2000);
  pwm.writeMicroseconds(CH_CONT, us);
  Serial.printf("continuous -> %d us\n", us);
}

int scanI2C() {
  Serial.print("I2C scan:");
  int n = 0;
  for (uint8_t a = 1; a < 127; a++) {
    Wire.beginTransmission(a);
    if (Wire.endTransmission() == 0) { Serial.printf(" 0x%02X", a); n++; }
  }
  Serial.println(n ? "" : " nothing found");
  return n;
}

// The breakout pulls SDA and SCL up to VCC. With a weak internal pull-down,
// any GPIO that still reads HIGH has one of those lines on it.
int findPulledUp(int *out) {
  int n = 0;
  Serial.print("GPIOs pulled high externally:");
  for (int g = 1; g <= 13; g++) {
    pinMode(g, INPUT_PULLDOWN);
    delay(2);
    if (digitalRead(g)) { Serial.printf(" %d", g); if (n < 4) out[n++] = g; }
    pinMode(g, INPUT);
  }
  Serial.println(n ? "" : " none (no wire reaches a powered pull-up)");
  return n;
}

// Try the configured pins, then every ordered pair of pulled-up pins.
bool findBus() {
  int cand[4];
  int n = findPulledUp(cand);
  int pairs[14][2] = {{PIN_SDA, PIN_SCL}};
  int np = 1;
  for (int i = 0; i < n; i++)
    for (int j = 0; j < n; j++)
      if (i != j) { pairs[np][0] = cand[i]; pairs[np][1] = cand[j]; np++; }
  for (int k = 0; k < np; k++) {
    Serial.printf("try SDA=%d SCL=%d -> ", pairs[k][0], pairs[k][1]);
    Wire.begin(pairs[k][0], pairs[k][1]);
    Wire.setTimeOut(50);
    if (scanI2C()) { Serial.printf("using SDA=%d SCL=%d\n", pairs[k][0], pairs[k][1]); return true; }
    Wire.end();
  }
  return false;
}

void handle(String cmd) {
  cmd.trim();
  if (!cmd.length()) return;
  demo = false;
  if (cmd.startsWith("c ")) setCont(cmd.substring(2).toInt());
  else if (cmd.startsWith("s ")) setAngle(cmd.substring(2).toInt());
  else if (cmd.startsWith("m ")) setMG(cmd.substring(2).toInt());
  else if (cmd == "demo") demo = true;
  else if (cmd == "off") { pwm.setPWM(CH_CONT, 0, 4096); pwm.setPWM(CH_MG, 0, 4096); pwm.setPWM(CH_180, 0, 4096); Serial.println("outputs off"); }
  else Serial.println("? c <us> | s <deg> | m <deg> | demo | off");
}

// Waits while still reading serial, so a command interrupts the demo.
bool wait(uint32_t ms) {
  uint32_t t = millis();
  while (millis() - t < ms) {
    while (Serial.available()) {
      char ch = Serial.read();
      if (ch == '\n' || ch == '\r') { handle(line); line = ""; }
      else line += ch;
    }
    if (!demo) return false;
    delay(5);
  }
  return true;
}

void setup() {
  Serial.begin(115200);
  delay(1500);  // native USB CDC needs a moment
  Serial.println("=== PCA9685 servo test v" VERSION " ===");
  Serial.println("PCA9685 bring-up: continuous servo on ch0, 180 servo on ch15");
  Serial.println("https://github.com/sui001/ruckus/tree/main/motor_player_rig/gen3d_pca9685_servo_test");

  if (!findBus()) {
    Serial.println("PCA9685 not found on any pin pair. Halting, no servo output.");
    while (true) { delay(1000); }
  }
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(50);
  delay(10);
  setCont(US_STOP);
  setAngle(90);
  setMG(90);
  Serial.println("demo running, send any command to take over");
}

void loop() {
  if (!demo) { wait(50); return; }
  Serial.println("-- demo: continuous forward");
  setCont(2000); if (!wait(2000)) return;
  setCont(US_STOP); if (!wait(1000)) return;
  Serial.println("-- demo: continuous reverse");
  setCont(1000); if (!wait(2000)) return;
  setCont(US_STOP); if (!wait(1000)) return;
  Serial.println("-- demo: 180 sweep");
  setAngle(0);   if (!wait(1000)) return;
  setAngle(180); if (!wait(1000)) return;
  setAngle(90);  if (!wait(1000)) return;
  Serial.println("-- demo: MG996R sweep");
  setMG(0);   if (!wait(1000)) return;
  setMG(180); if (!wait(1000)) return;
  setMG(90);  if (!wait(1000)) return;
}
