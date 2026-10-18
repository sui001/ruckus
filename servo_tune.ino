// servo_tune.ino
// ESP32-S3 Supermini + INMP441 + continuous-rotation servo.
//
// Plays pitches by setting rotation speed, and records itself doing it.
//
// A positional servo has no pitch to offer: its noise sat at ~9 kHz whatever
// was commanded. A continuous-rotation servo is different, because
// writeMicroseconds sets the motor's actual speed, and the whine below 2 kHz
// follows it. Measured in conttest.wav, turning one way: 898 Hz at 150 us from
// stop, rising through 1074, 1152 and 1238 to 1285 Hz at 500 us. So a note is a
// speed, looked up from that table on the PC side (tune.py).
//
// Streams the mic from boot, like the test sketches, and takes speed commands
// on the same port:
//   W <microseconds>\n     1000-2000, 1500 = stop
// Nothing is printed once the audio starts, since text would land in the PCM.
// If no command arrives for 3 s the servo stops, so a PC script that dies
// mid-tune cannot leave it spinning.
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
#define MIC_SHIFT   16

const int STOP_US = 1500;
const uint32_t WATCHDOG_MS = 3000;

Servo servo;
uint32_t lastCommandAt = 0;

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

  Serial.println("=== servo tune player v" VERSION " ===");
  Serial.println("Continuous-rotation servo played by speed, W <us> over serial, mic streamed as raw 16-bit PCM");
  Serial.println("https://github.com/sui001/ruckus");

  servo.setPeriodHertz(50);
  servo.attach(SERVO_PIN, 1000, 2000);
  servo.writeMicroseconds(STOP_US);

  i2sInit();
  Serial.printf("{\"sample_rate\":%d}\n", SAMPLE_RATE);
  Serial.println("===AUDIO_START===");
  lastCommandAt = millis();
}

void loop() {
  static char buf[32];
  static int len = 0;
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      buf[len] = '\0';
      if (len > 1 && (buf[0] == 'W' || buf[0] == 'w')) {
        int us = atoi(buf + 1);
        if (us < 1000) us = 1000;
        if (us > 2000) us = 2000;
        servo.writeMicroseconds(us);
        lastCommandAt = millis();
      }
      len = 0;
    } else if (len < (int)sizeof(buf) - 1) {
      buf[len++] = c;
    }
  }

  if (millis() - lastCommandAt > WATCHDOG_MS) servo.writeMicroseconds(STOP_US);

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
