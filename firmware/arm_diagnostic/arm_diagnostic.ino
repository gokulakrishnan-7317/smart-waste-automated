/*
 * ==============================================================================
 * 🛠️ ROBOTIC ARM HARDWARE DIAGNOSTIC TEST (PCA9685 + SERVOS)
 * ==============================================================================
 * This script tests your PCA9685 wiring and sweeps each servo one by one.
 * Open Arduino IDE -> Tools -> Serial Monitor (115200 baud).
 *
 * REQUIREMENTS:
 * Install "Adafruit PWM Servo Driver" library in Arduino IDE:
 * (Sketch -> Include Library -> Manage Libraries -> Search "Adafruit PWM Servo Driver")
 * ==============================================================================
 */

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pca = Adafruit_PWMServoDriver(0x40);

#define SERVO_FREQ 50  // 50 Hz analog servos
#define SERVOMIN  102  // 0.5 ms (~0 deg)
#define SERVOMAX  512  // 2.5 ms (~180 deg)

int angleToPulse(int angle) {
  angle = constrain(angle, 0, 180);
  return map(angle, 0, 180, SERVOMIN, SERVOMAX);
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n\n==================================================");
  Serial.println("   🤖 ROBOTIC ARM HARDWARE DIAGNOSTIC TEST");
  Serial.println("==================================================");

  // 1. I2C Bus Detection
  Serial.println("\n[STEP 1] Checking I2C Connection on GPIO 21 (SDA) & GPIO 22 (SCL)...");
  Wire.begin(21, 22);

  byte error, address;
  int nDevices = 0;
  for (address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    if (error == 0) {
      Serial.printf("  --> SUCCESS: Found I2C device at address 0x%02X!\n", address);
      nDevices++;
    }
  }

  if (nDevices == 0) {
    Serial.println("\n❌ CRITICAL ERROR: No I2C devices found!");
    Serial.println("   Check:");
    Serial.println("   1. ESP32 GPIO 21 connected to PCA9685 SDA?");
    Serial.println("   2. ESP32 GPIO 22 connected to PCA9685 SCL?");
    Serial.println("   3. PCA9685 VCC connected to 3.3V or 5V?");
    Serial.println("   4. PCA9685 GND connected to ESP32 GND?");
    while (1) { delay(1000); }
  }

  // 2. Initialize PCA9685
  Serial.println("\n[STEP 2] Initializing PCA9685...");
  pca.begin();
  pca.setPWMFreq(SERVO_FREQ);
  delay(100);
  Serial.println("  --> PCA9685 Initialized successfully!");

  Serial.println("\n[STEP 3] Starting Sequential Servo Sweep Test...");
  Serial.println("  Watch each joint move one-by-one.\n");
}

void loop() {
  const char* jointNames[] = {
    "Channel 0: BASE (MG996R)",
    "Channel 1: SHOULDER (MG996R)",
    "Channel 2: ELBOW (MG996R)",
    "Channel 3: WRIST PITCH (MG90S/MG996R)",
    "Channel 4: WRIST ROLL (MG90S)",
    "Channel 5: CLAW / GRIPPER (MG90S)"
  };

  for (int ch = 0; ch < 6; ch++) {
    Serial.println("--------------------------------------------------");
    Serial.printf(">>> Testing %s <<<\n", jointNames[ch]);
    Serial.println("    - Moving to 90 degrees (Center)...");
    pca.setPWM(ch, 0, angleToPulse(90));
    delay(1000);

    Serial.println("    - Moving to 45 degrees...");
    pca.setPWM(ch, 0, angleToPulse(45));
    delay(1000);

    Serial.println("    - Moving to 135 degrees...");
    pca.setPWM(ch, 0, angleToPulse(135));
    delay(1000);

    Serial.println("    - Returning to 90 degrees...");
    pca.setPWM(ch, 0, angleToPulse(90));
    delay(1000);
  }

  Serial.println("\n==================================================");
  Serial.println("Cycle completed! Pausing 3 seconds before repeat.");
  Serial.println("==================================================\n");
  delay(3000);
}
