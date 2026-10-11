/*
 * ==============================================================================
 * 6-DOF Robotic Arm Direct Hardware Diagnostic & Sweep Test
 * ==============================================================================
 * Tests the PCA9685 and all 6 arm servos directly without needing Wi-Fi or Python!
 *
 * Wire Connections:
 *   ESP32 GPIO 21 (D21) -> PCA9685 SDA
 *   ESP32 GPIO 22 (D22) -> PCA9685 SCL
 *   ESP32 3.3V or 5V    -> PCA9685 VCC (side pin)
 *   ESP32 GND           -> PCA9685 GND (side pin)
 *   External 5V-6V Batt -> PCA9685 Green Terminal Block (V+ & GND)
 * ==============================================================================
 */

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pca = Adafruit_PWMServoDriver(0x40);

#define SERVO_FREQ 50  // 50 Hz analog servos
#define SERVOMIN  102  // ~0.5ms (~0 deg)
#define SERVOMAX  512  // ~2.5ms (~180 deg)

#define CH_BASE         0  // MG996R (0 - 180 deg)
#define CH_SHOULDER     1  // MG996R (Shoulder)
#define CH_ELBOW        2  // MG996R (Elbow)
#define CH_WRIST_PITCH  3  // MG90S/MG996R
#define CH_WRIST_ROLL   4  // MG90S
#define CH_GRIPPER      5  // MG90S (Claw: 30 open, 95 clamp)

void setAngle(uint8_t ch, int angle) {
  angle = constrain(angle, 0, 180);
  int pulse = map(angle, 0, 180, SERVOMIN, SERVOMAX);
  pca.setPWM(ch, 0, pulse);
}

void sweepJoint(uint8_t ch, const char* name, int fromAng, int toAng, int stepDelay = 15) {
  Serial.printf("\n🔄 [Testing CH %d: %s] Moving %d° -> %d°...\n", ch, name, fromAng, toAng);
  int step = (toAng > fromAng) ? 1 : -1;
  for (int a = fromAng; a != toAng; a += step) {
    setAngle(ch, a);
    delay(stepDelay);
  }
  setAngle(ch, toAng);
  delay(400);

  Serial.printf("   Returning %d° -> %d°...\n", toAng, fromAng);
  step = (fromAng > toAng) ? 1 : -1;
  for (int a = toAng; a != fromAng; a += step) {
    setAngle(ch, a);
    delay(stepDelay);
  }
  setAngle(ch, fromAng);
  delay(500);
  Serial.printf("✅ CH %d (%s) finished.\n", ch, name);
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n========================================================");
  Serial.println("   6-DOF Robotic Arm Direct Hardware Diagnostic Test");
  Serial.println("========================================================");

  // 1. Initialize I2C on GPIO 21 (SDA) and GPIO 22 (SCL)
  Wire.begin(21, 22);
  delay(100);

  // 2. Check I2C address 0x40
  Wire.beginTransmission(0x40);
  byte err = Wire.endTransmission();

  if (err != 0) {
    Serial.printf("\n❌ [CRITICAL ERROR] PCA9685 NOT DETECTED AT 0x40 (Error Code: %d)!\n", err);
    Serial.println("Check these wires now:");
    Serial.println(" 1. PCA9685 SDA -> ESP32 GPIO 21 (D21)");
    Serial.println(" 2. PCA9685 SCL -> ESP32 GPIO 22 (D22)");
    Serial.println(" 3. PCA9685 VCC (side pin) -> ESP32 3.3V or 5V");
    Serial.println(" 4. PCA9685 GND -> ESP32 GND");
    Serial.println(" 5. External 5V-6V Battery -> Green Terminal Block (V+ & GND)");
    Serial.println("\nHalting test until wires are connected.");
    while (1) { delay(1000); }
  }

  Serial.println("✅ [SUCCESS] PCA9685 detected at address 0x40!");
  pca.begin();
  pca.setPWMFreq(SERVO_FREQ);
  delay(200);

  Serial.println("\n[1/7] Moving arm to HOME position...");
  setAngle(CH_GRIPPER, 30);  delay(150);
  setAngle(CH_WRIST_ROLL, 90); delay(150);
  setAngle(CH_WRIST_PITCH, 90); delay(150);
  setAngle(CH_ELBOW, 90);    delay(150);
  setAngle(CH_SHOULDER, 100); delay(150);
  setAngle(CH_BASE, 90);     delay(500);
  Serial.println("Arm is at Home Position.");
  delay(1500);
}

void loop() {
  Serial.println("\n>>> Starting Joint-by-Joint Arm Test Sequence <<<");

  // Joint 5: Gripper Claw
  sweepJoint(CH_GRIPPER, "Claw Gripper", 30, 95, 12);
  delay(1000);

  // Joint 4: Wrist Roll
  sweepJoint(CH_WRIST_ROLL, "Wrist Roll", 90, 140, 12);
  delay(1000);

  // Joint 3: Wrist Pitch
  sweepJoint(CH_WRIST_PITCH, "Wrist Pitch", 90, 120, 12);
  delay(1000);

  // Joint 2: Elbow
  sweepJoint(CH_ELBOW, "Elbow", 90, 115, 15);
  delay(1000);

  // Joint 1: Shoulder
  sweepJoint(CH_SHOULDER, "Shoulder", 100, 120, 15);
  delay(1000);

  // Joint 0: Base Yaw
  sweepJoint(CH_BASE, "Base Rotation", 90, 135, 15);
  delay(1500);

  Serial.println("\n✅ Test Cycle Complete! Repeating in 4 seconds...");
  delay(4000);
}
