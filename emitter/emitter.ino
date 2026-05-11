/*
 * Emitter.ino (Optimized for ESP32 Core 3.0+)
 * Dual Laser Emitter for Traffic Sensor
 */

#define LASER1_PIN 4
#define LASER2_PIN 5  // You can change this to 2 or 14 if needed

// PWM Properties
const int freq = 1000;       // 1000 Hz (1kHz)
const int resolution = 8;    // 8-bit resolution (0-255)
const int dutyCycle = 127;   // 50% duty cycle (127 is half of 255)

void setup() {
  Serial.begin(115200);
  Serial.println("Dual Laser Emitter (Core 3.0 API) Started");

  // In ESP32 Core 3.x, you attach the pin directly to a frequency and resolution.
  // The channel management is handled automatically by the core.
  
  ledcAttach(LASER1_PIN, freq, resolution);
  ledcWrite(LASER1_PIN, dutyCycle);

  ledcAttach(LASER2_PIN, freq, resolution);
  ledcWrite(LASER2_PIN, dutyCycle);

  Serial.printf("Laser 1 active on Pin %d at %d Hz\n", LASER1_PIN, freq);
  Serial.printf("Laser 2 active on Pin %d at %d Hz\n", LASER2_PIN, freq);
}

void loop() {
  // Signals run in hardware PWM, nothing needed here.
  delay(1000);
}
