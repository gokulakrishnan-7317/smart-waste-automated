/*
 * ==============================================================================
 * ESP32 I2C Scanner Tool (Diagnose SDA & SCL connections to PCA9685)
 * ==============================================================================
 * Wire connections to test:
 *   - ESP32 GPIO 21 (D21/P21) -> PCA9685 SDA
 *   - ESP32 GPIO 22 (D22/P22) -> PCA9685 SCL
 *   - ESP32 3.3V or 5V        -> PCA9685 VCC (side pin)
 *   - ESP32 GND               -> PCA9685 GND (side pin)
 * ==============================================================================
 */

#include <Wire.h>

#define SDA_PIN 21
#define SCL_PIN 22

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n========================================================");
  Serial.println("         ESP32 Hardware I2C Scanner Diagnostic");
  Serial.printf (" Probing SDA on GPIO %d, SCL on GPIO %d\n", SDA_PIN, SCL_PIN);
  Serial.println("========================================================");

  Wire.begin(SDA_PIN, SCL_PIN);
}

void loop() {
  byte error, address;
  int nDevices = 0;

  Serial.println("\nScanning I2C bus (Addresses 0x01 to 0x7F)...");

  for (address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0) {
      Serial.printf("  [SUCCESS] I2C device found at address 0x%02X", address);
      if (address == 0x40) {
        Serial.print("  <-- *** PCA9685 SERVO DRIVER FOUND! ***");
      } else if (address == 0x70) {
        Serial.print("  <-- (PCA9685 All-Call Address)");
      }
      Serial.println();
      nDevices++;
    } else if (error == 4) {
      Serial.printf("  [ERROR 4] Unknown error at address 0x%02X\n", address);
    }
  }

  if (nDevices == 0) {
    Serial.println("❌ NO I2C DEVICES FOUND!");
    Serial.println("--------------------------------------------------------");
    Serial.println("Troubleshooting Checklist:");
    Serial.println(" 1. Check SDA Wire: Must connect PCA9685 SDA to GPIO 21 (D21).");
    Serial.println(" 2. Check SCL Wire: Must connect PCA9685 SCL to GPIO 22 (D22).");
    Serial.println(" 3. Check VCC (Logic Power): PCA9685 VCC pin MUST connect to ESP32 3.3V or 5V.");
    Serial.println(" 4. Check GND: PCA9685 GND pin MUST connect to ESP32 GND.");
    Serial.println(" 5. Check if SDA and SCL are reversed (swap them and re-test).");
    Serial.println("--------------------------------------------------------");
  } else {
    Serial.printf("✅ Done. Found %d active I2C device(s).\n", nDevices);
  }

  delay(3000);
}
