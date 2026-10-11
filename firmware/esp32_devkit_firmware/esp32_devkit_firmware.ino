/*
 * ==============================================================================
 * ESP32 NodeMCU-32S / DevKit (30-PIN) - Wireless Smart Waste Robot Controller
 * ==============================================================================
 *
 *               [ ANTENNA / TOP ]
 *          (EN) [ ]            [ ] D23 (GPIO 23) -> [Free / Not Used]
 *          (VP) [ ]            [ ] D22 (GPIO 22) -> PCA9685 SCL (I2C Clock)
 *          (VN) [ ]            [ ] TX0 (GPIO 1)  [Skip]
 *          D34  [ ]            [ ] RX0 (GPIO 3)  [Skip]
 *          D35  [ ]            [ ] D21 (GPIO 21) -> PCA9685 SDA (I2C Data)
 *  IN3 Fwd D32  [ ]            [ ] D19 (GPIO 19) -> [Free / Not Used]
 *  IN4 Rev D33  [ ]            [ ] D18 (GPIO 18) -> HC-SR04 ECHO (Ultrasonic Echo)
 *  IN1 Fwd D25  [ ]            [ ] D5  (GPIO 5)  -> HC-SR04 TRIG (Ultrasonic Trigger)
 *  IN2 Rev D26  [ ]            [ ] D17 (GPIO 17) -> [Free]
 *  [Free]  D27  [ ]            [ ] D16 (GPIO 16) -> [Free]
 *  [Free]  D14  [ ]            [ ] D4  (GPIO 4)  -> [Free]
 *  ENA PWM D12  [ ]            [ ] D2  (GPIO 2)  -> [Onboard LED / Free]
 *  ENB PWM D13  [ ]            [ ] D15 (GPIO 15) -> [Free]
 *  GND     GND  [ ]            [ ] GND (GND)     -> Common GND (Battery - & ESP32 GND)
 *  VIN     VIN  [ ]            [ ] 3V3 (3.3V)    -> PCA9685 VCC (Logic Power)
 *               [  MICRO USB   ]
 * ==============================================================================
 */

#include <Adafruit_PWMServoDriver.h>
#include <Wire.h>
#include <BluetoothSerial.h>
#include "soc/soc.h"             // Power/Brownout control
#include "soc/rtc_cntl_reg.h"    // Power/Brownout control

// ------------------------------------------------------------------------------
// 1. BLUETOOTH SERIAL CONFIGURATION (100% Wireless, Zero IP Needed)
// ------------------------------------------------------------------------------
BluetoothSerial SerialBT;
const char *btDeviceName = "SMART_WASTE_ROBOT";

// ------------------------------------------------------------------------------
// 2. PCA9685 16-CHANNEL SERVO DRIVER CONFIGURATION (Address 0x40)
//
// YOUR ARM SERVO LAYOUT:
//   CH0 (CH_BASE)        : SG996R  360° CONTINUOUS rotation
//                          Neutral (STOP) = 1500 μs pulse → ~307 ticks
//                          CW  rotation  > 1500 μs pulse
//                          CCW rotation  < 1500 μs pulse
//   CH1 (CH_SHOULDER)    : SG996R  180° positional  (high torque)
//   CH2 (CH_ELBOW)       : SG996R  180° positional  (high torque)
//   CH3 (CH_WRIST_PITCH) : MG90S   180° positional  (small/light)
//   CH4 (CH_WRIST_ROLL)  : MG90S   180° positional  (small/light)
//   CH5 (CH_GRIPPER)     : MG90S   180° positional  (small/light)
// ------------------------------------------------------------------------------
Adafruit_PWMServoDriver pca = Adafruit_PWMServoDriver(0x40);

#define SERVO_FREQ 50 // 50 Hz for all analog servos

// SG996R (CH1 & CH2): 500–2500 μs gives full 180° range
//   At 50Hz  → 4096 ticks/period (20ms)
//   1 tick   = 20000μs / 4096 ≈ 4.88 μs
#define SERVO996_MIN 102 //  500 μs →  0°
#define SERVO996_MAX 512 // 2500 μs → 180°

// MG90S (CH3 CH4 CH5): standard 1000–2000 μs range
#define SERVOMG90_MIN 205 // 1000 μs →  0°
#define SERVOMG90_MAX 410 // 2000 μs → 180°

// SG996R 360° continuous rotation (CH0 BASE):
//   Neutral STOP  = 1500 μs ≈ 307 ticks
//   Full CW       = 2000 μs ≈ 410 ticks  (or higher for faster)
//   Full CCW      = 1000 μs ≈ 205 ticks  (or lower for faster)
#define BASE_NEUTRAL 307  // 1500 μs → motor stops
#define BASE_FULL_CW 410  // 2000 μs → max CW speed
#define BASE_FULL_CCW 205 // 1000 μs → max CCW speed

// 6-DOF Servo Channels on PCA9685
#define CH_BASE 0        // SG996R 360° CONTINUOUS (base yaw)
#define CH_SHOULDER 1    // SG996R 180° positional (shoulder pitch)
#define CH_ELBOW 2       // SG996R 180° positional (elbow pitch)
#define CH_WRIST_PITCH 3 // MG90S  180° positional (wrist tilt)
#define CH_WRIST_ROLL 4  // MG90S  180° positional (wrist roll)
#define CH_GRIPPER 5     // MG90S  180° positional (claw)

// Base 360° continuous rotation settings and bin calibration
#define BASE_SPEED 60 // % speed (0-100) used for base rotations
int binRight180Ms = 1100; // ms to rotate 180° CW (Turn Right) to Degradable Bin
int binLeft180Ms  = 1100; // ms to rotate 180° CCW (Turn Left) to Non-Degradable Bin

// Forward declarations for clean C++ compilation
void stopMotors();
void moveForward();
void moveBackward();
void steerLeft();
void steerRight();
void armHome();
void baseStop();
void baseSetSpeed(int speed);
void setServoAngle(uint8_t channel, int angle);
void smoothMoveServo(uint8_t channel, int targetAngle, int stepDelay = 10);
void rotateBaseToAngle(int targetAngle);
void executeSmartTargetPickSequence(String wasteClass, String offsetDir, int offsetMs);
void executePickSequence(String wasteClass);

int currentAngles[6] = {-1, -1, -1, -1, -1, -1};
bool pcaReady = false;

// Auto-recovery function for PCA9685 I2C driver on 30-Pin ESP32
bool ensurePcaReady() {
  if (pcaReady) return true;
  Wire.setTimeOut(50);
  Wire.beginTransmission(0x40);
  byte err = Wire.endTransmission();
  if (err == 0) {
    pcaReady = true;
    pca.begin();
    pca.setPWMFreq(SERVO_FREQ);
    Serial.println("✅ [AUTO-RECOVERY] PCA9685 detected & armed on the fly!");
    return true;
  }
  return false;
}

// ---------- Helper: return correct SERVOMIN for a given channel ----------
inline int servoMin(uint8_t ch) {
  return (ch <= 2) ? SERVO996_MIN : SERVOMG90_MIN;
}
inline int servoMax(uint8_t ch) {
  return (ch <= 2) ? SERVO996_MAX : SERVOMG90_MAX;
}

// ---------- Positional servo angle setter (CH0-CH5) -----------------
void setServoAngle(uint8_t channel, int angle) {
  if (channel >= 6) return;
  if (!ensurePcaReady()) return;
  angle = constrain(angle, 0, 180);
  int pulse = map(angle, 0, 180, servoMin(channel), servoMax(channel));
  pca.setPWM(channel, 0, pulse);
  currentAngles[channel] = angle;
}

void smoothMoveServo(uint8_t channel, int targetAngle, int stepDelay) {
  if (channel >= 6 || !pcaReady)
    return;
  targetAngle = constrain(targetAngle, 0, 180);
  int startAngle = currentAngles[channel];
  if (startAngle < 0) {
    setServoAngle(channel, targetAngle);
    return;
  }
  if (startAngle == targetAngle) {
    setServoAngle(channel, targetAngle);
    return;
  }
  int step = (targetAngle > startAngle) ? 1 : -1;
  for (int a = startAngle; a != targetAngle; a += step) {
    int pulse = map(a, 0, 180, servoMin(channel), servoMax(channel));
    pca.setPWM(channel, 0, pulse);
    delay(stepDelay);
  }
  setServoAngle(channel, targetAngle);
}

// ============================================================
// BASE 360° CONTINUOUS ROTATION (SG996R) CONTROL
// speed: -100 (full CCW) … 0 (STOP) … +100 (full CW)
// ============================================================
void baseStop() {
  if (!ensurePcaReady()) return;
  pca.setPWM(CH_BASE, 0, BASE_NEUTRAL);
  currentAngles[CH_BASE] = 0; // use 0 as "parked / unknown" marker
  Serial.println("[ARM BASE] STOP (neutral 1500us)");
}

void baseSetSpeed(int speed) {
  if (!ensurePcaReady()) return;
  speed = constrain(speed, -100, 100);
  int pulse;
  if (speed == 0) {
    pulse = BASE_NEUTRAL;
  } else if (speed > 0) {
    pulse = map(speed, 0, 100, BASE_NEUTRAL, BASE_FULL_CW);
  } else {
    pulse = map(speed, -100, 0, BASE_FULL_CCW, BASE_NEUTRAL);
  }
  pca.setPWM(CH_BASE, 0, pulse);
  Serial.printf("[ARM BASE] Speed=%d%%  pulse=%d ticks\n", speed, pulse);
}

void baseRotateCW(int speed, unsigned int durationMs) {
  Serial.printf("[ARM BASE] Rotate CW  speed=%d%%  for %d ms\n", speed,
                durationMs);
  int safeSpeed = (speed < 0) ? -speed : speed;
  baseSetSpeed(safeSpeed);
  delay(durationMs);
  baseStop();
}

void baseRotateCCW(int speed, unsigned int durationMs) {
  Serial.printf("[ARM BASE] Rotate CCW speed=%d%%  for %d ms\n", speed,
                durationMs);
  int safeSpeed = (speed < 0) ? -speed : speed;
  baseSetSpeed(-safeSpeed);
  delay(durationMs);
  baseStop();
}

// ------------------------------------------------------------------------------
// BASE 360° CONTINUOUS ROTATION TIMED DEGREE ANGLE CONTROLLER WITH AUTO-STOP
// ------------------------------------------------------------------------------
int currentBaseAngle = 180; // Default center (0°–360° range)
unsigned long baseRotateEndTime = 0;
bool baseIsRotating = false;

void rotateBaseToAngle(int targetAngle) {
  if (!ensurePcaReady()) return;
  targetAngle = constrain(targetAngle, 0, 360);
  int delta = targetAngle - currentBaseAngle;
  if (abs(delta) < 2) {
    baseStop();
    return;
  }

  // Calibration: binRight180Ms is milliseconds for 180° rotation at BASE_SPEED (default ~1100ms)
  float msPerDeg = (float)binRight180Ms / 180.0f;
  unsigned long durationMs = (unsigned long)(abs(delta) * msPerDeg);
  durationMs = constrain(durationMs, 20, 6000);

  if (delta > 0) {
    baseSetSpeed(BASE_SPEED); // Clockwise (CW)
    Serial.printf(">>> [ARM BASE 360°] Rotating CW: %d° -> %d° (%d° turn, %lu ms)...\n",
                  currentBaseAngle, targetAngle, delta, durationMs);
  } else {
    baseSetSpeed(-BASE_SPEED); // Counter-Clockwise (CCW)
    Serial.printf(">>> [ARM BASE 360°] Rotating CCW: %d° -> %d° (%d° turn, %lu ms)...\n",
                  currentBaseAngle, targetAngle, abs(delta), durationMs);
  }

  baseRotateEndTime = millis() + durationMs;
  baseIsRotating = true;
  currentBaseAngle = targetAngle;
  currentAngles[CH_BASE] = targetAngle;
}

// ------------------------------------------------------------------------------
// 3. MOTOR DRIVER PIN ASSIGNMENTS (SINGLE L298N Driver -> 4 DC Motors)
//
// *** SINGLE L298N WIRING WITH VOLTAGE & SPEED CONTROL VIA ENA / ENB ***
// Wiring:
//   L298N OUT1 & OUT2 ──► Left Motors (Front-Left & Rear-Left in parallel)
//   L298N OUT3 & OUT4 ──► Right Motors (Front-Right & Rear-Right in parallel)
//   L298N ENA (jumper removed!) ──► GPIO 12 (Left side PWM / voltage control)
//   L298N ENB (jumper removed!) ──► GPIO 13 (Right side PWM / voltage control)
//   L298N IN1 ──► GPIO 25 (Left Forward)
//   L298N IN2 ──► GPIO 26 (Left Reverse)
//   L298N IN3 ──► GPIO 32 (Right Forward)
//   L298N IN4 ──► GPIO 33 (Right Reverse)
//   L298N GND ──► Battery (–) AND ESP32 GND (Common Ground!)
//   L298N 12V ──► Battery (+) (7.4V - 12V)
// ------------------------------------------------------------------------------
#define ENA 12  // Left Motors Speed/Voltage PWM (GPIO 12)
#define IN1 25  // Left Motors Forward
#define IN2 26  // Left Motors Reverse
#define IN3 32  // Right Motors Forward
#define IN4 33  // Right Motors Reverse
#define ENB 13  // Right Motors Speed/Voltage PWM (GPIO 13)

// Aliases for compatibility
#define EN_LEFT  ENA
#define EN_RIGHT ENB

// LEDC PWM settings — 5 kHz is optimal for small brushed DC motors
#define PWM_FREQ 5000 // Hz
#define PWM_RES  8    // bits: 0 = stop, 255 = full speed

// ── Yellow TT Gearbox Motor Calibration (Rated: 3V - 6V DC) ──
// Power supply configuration:
const float BATTERY_VOLTAGE = 12.0; // Battery voltage (set to 7.4 if using 2S 18650)
const float L298N_DROP_V    = 1.6;  // Internal L298N bipolar transistor drop (~1.6V)

// TT Gearbox Deadband & Safety Thresholds:
// 1. TT plastic gears have internal friction: PWM below 120 produces stall hum
// 2. Safe Max: Clamped to 205 (~6.0V max) to avoid stripping plastic gears on 12V
#define TT_MIN_PWM    125  // Minimum starting PWM to overcome gear friction
#define TT_MAX_PWM    205  // Max safe PWM (prevents exceeding 6.0V on 12V supply)
#define TT_CRUISE_PWM 165  // Default cruising speed (~5.0V output, high torque)

// Converts target voltage (e.g. 5.0V, 6.0V) to 8-bit PWM value (0-255)
inline int voltageToPWM(float targetVolts) {
  float maxUsableVolts = BATTERY_VOLTAGE - L298N_DROP_V;
  if (targetVolts <= 0.0f) return 0;
  if (targetVolts > 6.0f) targetVolts = 6.0f; // HARD CLAMP: Protect 6V TT motors!
  int pwm = (int)((targetVolts / maxUsableVolts) * 255.0f);
  return constrain(pwm, TT_MIN_PWM, TT_MAX_PWM);
}

// Default drive speed for Yellow TT Motors
int motorSpeed = TT_CRUISE_PWM;

bool isDrivingForward = false;
bool isMotorsRunning  = false;

// Clamp helper ensuring TT motor never stalls in deadband or burns at >6V
inline int clampTTPWM(int spd) {
  if (spd <= 0) return 0;
  return constrain(spd, TT_MIN_PWM, TT_MAX_PWM);
}

// ---------- PWM & Voltage helpers (Dual-compatible ESP32 Core 2.x & 3.x) ----------
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
inline void setLeftPWM(int spd)   { ledcWrite(ENA, clampTTPWM(spd)); }
inline void setRightPWM(int spd)  { ledcWrite(ENB, clampTTPWM(spd)); }
inline void disableDrive() {
  ledcWrite(ENA, 0);
  ledcWrite(ENB, 0);
}
#else
#define PWM_CH_LEFT  0
#define PWM_CH_RIGHT 1
inline void setLeftPWM(int spd)   { ledcWrite(PWM_CH_LEFT, clampTTPWM(spd)); }
inline void setRightPWM(int spd)  { ledcWrite(PWM_CH_RIGHT, clampTTPWM(spd)); }
inline void disableDrive() {
  ledcWrite(PWM_CH_LEFT, 0);
  ledcWrite(PWM_CH_RIGHT, 0);
}
#endif
inline void setMotorVoltage(float leftV, float rightV) {
  setLeftPWM(voltageToPWM(leftV));
  setRightPWM(voltageToPWM(rightV));
}
inline void enableDrive() {
  setLeftPWM(motorSpeed);
  setRightPWM(motorSpeed);
}

// Safety Watchdog: Stops motors if no command received within 2500ms
unsigned long lastPacketTime = 0;
const unsigned long SAFETY_TIMEOUT_MS = 2500;

// ------------------------------------------------------------------------------
// 4. HC-SR04 ULTRASONIC DISTANCE SENSOR CONFIGURATION
// ------------------------------------------------------------------------------
#define TRIG_PIN 5   // GPIO 5  -> HC-SR04 TRIG (Trigger Pulse Output)
#define ECHO_PIN 18  // GPIO 18 -> HC-SR04 ECHO (Echo Pulse Input)

const float OBSTACLE_STOP_DISTANCE_CM = 15.0; // Auto-Brake emergency stop threshold (cm)
float lastMeasuredDistanceCm = -1.0;
bool obstacleNear = false;

// Measures distance in centimeters using HC-SR04
float readUltrasonicDistanceCm() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  
  // 10 microsecond trigger pulse
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  // Measure pulse duration (timeout 25000us ~ 4.3 meters)
  unsigned long duration = pulseIn(ECHO_PIN, HIGH, 25000);
  if (duration == 0) {
    return -1.0; // Out of range or no echo
  }

  // Distance in cm = (duration / 2) * speed of sound (0.0343 cm/us)
  float dist = (duration * 0.0343f) / 2.0f;
  return dist;
}

// Bluetooth Wireless Telemetry
unsigned long lastTelemetryTime = 0;
const unsigned long TELEMETRY_INTERVAL_MS = 1000;

void sendTelemetry() {
  float dist = readUltrasonicDistanceCm();
  lastMeasuredDistanceCm = dist;
  obstacleNear = (dist > 0.0f && dist <= OBSTACLE_STOP_DISTANCE_CM);

  // If driving forward and obstacle is too close, emergency auto-brake!
  if (isDrivingForward && obstacleNear) {
    Serial.printf("⚠️ [OBSTACLE DETECTED] Distance = %.1f cm! Auto-Braking!\n", dist);
    stopMotors();
  }

  char teleBuf[64];
  snprintf(teleBuf, sizeof(teleBuf), "TELEM:DIST=%.1f,OBST=%d", 
           dist, obstacleNear ? 1 : 0);

  // Send via Bluetooth Serial if connected
  if (SerialBT.hasClient()) {
    SerialBT.println(teleBuf);
  }
  // Also send via USB Serial (for direct cable / serial monitor)
  Serial.println(teleBuf);
}

// ==============================================================================
// MOTOR CONTROL FUNCTIONS (SMOOTH PIVOT STEERING & FULL POWER)
// ==============================================================================

// ==============================================================================
// MOTOR CONTROL FUNCTIONS (PWM speed via LEDC on EN_LEFT / EN_RIGHT)
// Wiring reminder: remove L298N ENA/ENB jumpers, connect GPIO 12 & 13
// ==============================================================================

void moveForward() {
  isDrivingForward = true;
  isMotorsRunning  = true;
  Serial.printf(">>> [MOTORS] FORWARD [W] (ALL 4 WHEELS FWD: FL, RL, FR, RR) speed=%d/255\n", motorSpeed);

  // Direction: Left and Right both FORWARD
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);

  // Enable at configured speed/voltage
  setLeftPWM(motorSpeed);
  setRightPWM(motorSpeed);
}

void moveBackward() {
  isDrivingForward = false;
  isMotorsRunning  = true;
  Serial.printf(">>> [MOTORS] BACKWARD [S] (ALL 4 WHEELS REV: FL, RL, FR, RR) speed=%d/255\n", motorSpeed);

  // Direction: Left and Right both REVERSE
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);

  setLeftPWM(motorSpeed);
  setRightPWM(motorSpeed);
}

// ------------------------------------------------------------------------------
// SPIN LEFT (TANK TURN) [A]: Right side FORWARD (FR, RR), Left side REVERSE (FL, RL)
// ------------------------------------------------------------------------------
void steerLeft() {
  isDrivingForward = false;
  isMotorsRunning  = true;
  Serial.println(">>> [MOTORS] PIVOT LEFT [A] (RIGHT FWD: FR, RR | LEFT REV: FL, RL)");

  // Left motors REVERSE, Right motors FORWARD
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);

  setLeftPWM(motorSpeed);
  setRightPWM(motorSpeed);
}

// ------------------------------------------------------------------------------
// SPIN RIGHT (TANK TURN) [D]: Left side FORWARD (FL, RL), Right side REVERSE (FR, RR)
// ------------------------------------------------------------------------------
void steerRight() {
  isDrivingForward = false;
  isMotorsRunning  = true;
  Serial.println(">>> [MOTORS] PIVOT RIGHT [D] (LEFT FWD: FL, RL | RIGHT REV: FR, RR)");

  // Left motors FORWARD, Right motors REVERSE
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);

  setLeftPWM(motorSpeed);
  setRightPWM(motorSpeed);
}

void stopMotors() {
  isDrivingForward = false;
  isMotorsRunning  = false;

  // Disable both PWM channels
  disableDrive();

  // Clear all direction pins
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

// ------------------------------------------------------------------------------
// FORWARD + CURVE LEFT (Q): Right wheels (FR, RR) rotate forward, Left stopped
// ------------------------------------------------------------------------------
void forwardCurveLeft() {
  isDrivingForward = true;
  isMotorsRunning  = true;
  Serial.println(">>> [MOTORS] CURVE LEFT [Q] (RIGHT FWD: FR, RR | LEFT STOP: FL, RL)");

  // Left stopped
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  setLeftPWM(0);

  // Right forward
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  setRightPWM(motorSpeed);
}

// ------------------------------------------------------------------------------
// FORWARD + CURVE RIGHT (E): Left wheels (FL, RL) rotate forward, Right stopped
// ------------------------------------------------------------------------------
void forwardCurveRight() {
  isDrivingForward = true;
  isMotorsRunning  = true;
  Serial.println(">>> [MOTORS] CURVE RIGHT [E] (LEFT FWD: FL, RL | RIGHT STOP: FR, RR)");

  // Left forward
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  setLeftPWM(motorSpeed);

  // Right stopped
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  setRightPWM(0);
}

// ------------------------------------------------------------------------------
// BACKWARD + CURVE LEFT (Z): Right wheels (FR, RR) rotate reverse, Left stopped
// ------------------------------------------------------------------------------
void backCurveLeft() {
  isDrivingForward = false;
  isMotorsRunning  = true;
  Serial.println(">>> [MOTORS] BACK LEFT [Z] (RIGHT REV: FR, RR | LEFT STOP: FL, RL)");

  // Left stopped
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  setLeftPWM(0);

  // Right reverse
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  setRightPWM(motorSpeed);
}

// ------------------------------------------------------------------------------
// BACKWARD + CURVE RIGHT (C): Left wheels (FL, RL) rotate reverse, Right stopped
// ------------------------------------------------------------------------------
void backCurveRight() {
  isDrivingForward = false;
  isMotorsRunning  = true;
  Serial.println(">>> [MOTORS] BACK RIGHT [C] (LEFT REV: FL, RL | RIGHT STOP: FR, RR)");

  // Left reverse
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  setLeftPWM(motorSpeed);

  // Right stopped
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  setRightPWM(0);
}

// ==============================================================================
// 6-DOF ROBOTIC ARM SEQUENCES (PCA9685)
//
// BASE (CH0) is a 360° CONTINUOUS servo → controlled by time, not angle.
// Bin rotation times are APPROXIMATE — tune BASE_CW_MS_PER_DEG for your setup.
// Measure: run baseRotateCW(60, 1000) and see how many degrees it turns.
// Then adjust BIN_x_MS to match your physical bin positions.
// ==============================================================================

// ==============================================================================
// DUAL DUSTBIN CONFIGURATION:
//   - BIN 1 (RIGHT · 180° CW)  : DEGRADABLE WASTE (Banana Peel, Food/Organic)
//   - BIN 2 (LEFT  · 180° CCW) : NON-BIODEGRADABLE WASTE (Plastics, Metals, Cans, Batteries)
// ==============================================================================
// (BASE_SPEED and bin calibration ms defined at top of file)

// Helper to determine if detected class is Degradable or Non-Degradable
inline bool isDegradableWaste(String wasteClass) {
  wasteClass.toUpperCase();
  wasteClass.trim();
  return (wasteClass == "BANANA" || 
          wasteClass == "BANANA PEEL" || 
          wasteClass == "BANANA_PEEL" || 
          wasteClass == "DEGRADABLE" || 
          wasteClass == "ORGANIC" || 
          wasteClass == "BIO" ||
          wasteClass == "FOOD");
}

void armHome() {
  if (!pcaReady)
    return;
  Serial.println(">>> [ARM] Moving to HOME position...");

  // !! BASE is 360° continuous: send neutral pulse to STOP it !!
  baseStop();

  // Return all positional joints to resting pose
  setServoAngle(CH_GRIPPER, 30);
  delay(120); // open claw
  setServoAngle(CH_WRIST_ROLL, 90);
  delay(120); // center roll
  setServoAngle(CH_WRIST_PITCH, 90);
  delay(120); // level wrist
  setServoAngle(CH_ELBOW, 90);
  delay(120); // elbow up
  setServoAngle(CH_SHOULDER, 100);
  delay(200); // shoulder raised
}

// ------------------------------------------------------------------------------
// DEPOSIT INTO RIGHT DUSTBIN (DEGRADABLE / BANANA PEEL · TURN RIGHT 180° CW)
// ------------------------------------------------------------------------------
void dropIntoRightDegradableBin(int offsetMs = 0) {
  // If base was steered CCW (LEFT) by offsetMs during approach:
  // to reach 180° CW (RIGHT), rotate CW by (binRight180Ms + offsetMs)
  // If base was steered CW (RIGHT) by offsetMs:
  // rotate CW by (binRight180Ms - offsetMs)
  int rotateMs = binRight180Ms + offsetMs;
  rotateMs = max(400, rotateMs);

  Serial.printf(">>> [ARM] DEGRADABLE: Turning RIGHT 180° CW towards Right Bin (%d ms)...\n", rotateMs);
  baseRotateCW(BASE_SPEED, rotateMs);
  delay(300);

  // Dip arm into right degradable container
  Serial.println("[ARM] Dipping inside Right Degradable Bin...");
  smoothMoveServo(CH_SHOULDER, 65, 12);
  smoothMoveServo(CH_ELBOW, 75, 12);
  smoothMoveServo(CH_WRIST_PITCH, 80, 10);
  delay(350);

  // Release object
  Serial.println("[ARM] Opening claw -> Degradable waste deposited!");
  smoothMoveServo(CH_GRIPPER, 30, 10);
  delay(400);

  // Lift arm out of container
  Serial.println("[ARM] Lifting out of Right Bin...");
  smoothMoveServo(CH_SHOULDER, 110, 12);
  smoothMoveServo(CH_ELBOW, 100, 12);
  smoothMoveServo(CH_WRIST_PITCH, 90, 10);
  delay(300);

  // Return LEFT 180° CCW back to front center
  Serial.printf("[ARM] Returning LEFT 180° CCW to front center (%d ms)...\n", binRight180Ms);
  baseRotateCCW(BASE_SPEED, binRight180Ms);
  delay(300);
}

// ------------------------------------------------------------------------------
// DEPOSIT INTO LEFT DUSTBIN (NON-BIODEGRADABLE · TURN LEFT 180° CCW)
// ------------------------------------------------------------------------------
void dropIntoLeftNonDegradableBin(int offsetMs = 0) {
  // If base was steered CW (RIGHT) by offsetMs during approach:
  // to reach 180° CCW (LEFT), rotate CCW by (binLeft180Ms + offsetMs)
  // If base was steered CCW (LEFT) by offsetMs:
  // rotate CCW by (binLeft180Ms - offsetMs)
  int rotateMs = binLeft180Ms + offsetMs;
  rotateMs = max(400, rotateMs);

  Serial.printf(">>> [ARM] NON-BIODEGRADABLE: Turning LEFT 180° CCW towards Left Bin (%d ms)...\n", rotateMs);
  baseRotateCCW(BASE_SPEED, rotateMs);
  delay(300);

  // Dip arm into left non-degradable container
  Serial.println("[ARM] Dipping inside Left Non-Degradable Bin...");
  smoothMoveServo(CH_SHOULDER, 65, 12);
  smoothMoveServo(CH_ELBOW, 75, 12);
  smoothMoveServo(CH_WRIST_PITCH, 80, 10);
  delay(350);

  // Release object
  Serial.println("[ARM] Opening claw -> Non-Degradable waste deposited!");
  smoothMoveServo(CH_GRIPPER, 30, 10);
  delay(400);

  // Lift arm out of container
  Serial.println("[ARM] Lifting out of Left Bin...");
  smoothMoveServo(CH_SHOULDER, 110, 12);
  smoothMoveServo(CH_ELBOW, 100, 12);
  smoothMoveServo(CH_WRIST_PITCH, 90, 10);
  delay(300);

  // Return RIGHT 180° CW back to front center
  Serial.printf("[ARM] Returning RIGHT 180° CW to front center (%d ms)...\n", binLeft180Ms);
  baseRotateCW(BASE_SPEED, binLeft180Ms);
  delay(300);
}

// ==============================================================================
// SMART ML TARGET PICK & DUAL DUSTBIN DISPATCH
// Grabs item, inspects classification, and triggers 180° Right or 180° Left bin!
// ==============================================================================
void executeSmartTargetPickSequence(String wasteClass, String offsetDir,
                                    int offsetMs) {
  if (!pcaReady) {
    Serial.println("[ARM ERROR] PCA9685 not ready!");
    return;
  }
  stopMotors();

  bool isDeg = isDegradableWaste(wasteClass);
  Serial.printf("\n======================================================\n");
  Serial.printf("[ARM SMART PICK] Class: %s -> %s DUSTBIN\n", 
                wasteClass.c_str(), isDeg ? "DEGRADABLE (RIGHT 180°)" : "NON-BIODEGRADABLE (LEFT 180°)");
  Serial.printf("                 Offset Dir: %s  Offset Time: %d ms\n", offsetDir.c_str(), offsetMs);
  Serial.printf("======================================================\n");

  // ── Step 1: Base stopped, gripper open, arm lifted ────────────────────
  baseStop();
  smoothMoveServo(CH_GRIPPER, 30, 8);
  smoothMoveServo(CH_WRIST_ROLL, 90, 8);
  delay(150);

  // ── Step 2: AIM BASE TOWARDS WASTE ITEM (Left / Right Offset from ML) ──
  offsetMs = constrain(offsetMs, 0, 800);
  int effectiveOffsetForBin = 0;
  if (offsetDir == "LEFT" && offsetMs > 0) {
    Serial.printf("[ARM BASE] Aiming LEFT toward item (%d ms CCW)...\n", offsetMs);
    baseRotateCCW(BASE_SPEED, offsetMs);
    effectiveOffsetForBin = isDeg ? offsetMs : -offsetMs;
    delay(200);
  } else if (offsetDir == "RIGHT" && offsetMs > 0) {
    Serial.printf("[ARM BASE] Aiming RIGHT toward item (%d ms CW)...\n", offsetMs);
    baseRotateCW(BASE_SPEED, offsetMs);
    effectiveOffsetForBin = isDeg ? -offsetMs : offsetMs;
    delay(200);
  } else {
    Serial.println("[ARM BASE] Item is CENTERED, reaching straight ahead.");
  }

  // ── Step 3: Reach forward & down to ground level ──────────────────────
  Serial.println("[ARM] Reaching down to ground level...");
  smoothMoveServo(CH_SHOULDER, 50, 15);
  smoothMoveServo(CH_ELBOW, 60, 15);
  smoothMoveServo(CH_WRIST_PITCH, 70, 12);
  delay(350);

  // ── Step 4: Clamp gripper on waste item ──────────────────────────────
  Serial.println("[ARM] Clamping gripper on object...");
  smoothMoveServo(CH_GRIPPER, 105, 12);
  delay(400);

  // ── Step 5: Lift arm up to clear chassis ──────────────────────────────
  Serial.println("[ARM] Lifting object from ground...");
  smoothMoveServo(CH_SHOULDER, 110, 15);
  smoothMoveServo(CH_ELBOW, 100, 15);
  smoothMoveServo(CH_WRIST_PITCH, 90, 12);
  delay(300);

  // ── Step 6: ROUTE TO DESIGNATED DUSTBIN ───────────────────────────────
  if (isDeg) {
    // Degradable Waste (e.g. Banana Peel) -> TURN RIGHT 180° CW
    dropIntoRightDegradableBin(effectiveOffsetForBin);
  } else {
    // Non-Biodegradable Waste (Plastics, Metals, Batteries) -> TURN LEFT 180° CCW
    dropIntoLeftNonDegradableBin(effectiveOffsetForBin);
  }

  // ── Step 7: Return arm to driving HOME pose ──────────────────────────
  armHome();
}

void executePickSequence(String wasteClass) {
  executeSmartTargetPickSequence(wasteClass, "CENTER", 0);
}

// ==============================================================================
// SETUP
// ==============================================================================
void setup() {
  Serial.begin(115200);
  delay(800);
  Serial.println("\n========================================================");
  Serial.println(" Smart Waste Management Robot Controller (ESP32)");
  Serial.println("========================================================");

  // 0. Disable brownout detector & let power supply stabilize on cold boot
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); // Prevents cold-boot brownout freeze!
  delay(1200); // 1.2s delay for power rails to reach stable 3.3V/5.0V

  // 1. Setup Single L298N Motor Direction Pins (4 pins instead of 8)
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  // 2. Setup PWM Enable channels via LEDC (ESP32 hardware PWM)
  //    Frequency: 5 kHz, 8-bit resolution (0-255)
  //    ENA -> Left motors voltage/speed, ENB -> Right motors voltage/speed
  #if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
    ledcAttach(ENA, PWM_FREQ, PWM_RES);  // ENA (GPIO 12)
    ledcAttach(ENB, PWM_FREQ, PWM_RES);  // ENB (GPIO 13)
  #else
    ledcSetup(PWM_CH_LEFT, PWM_FREQ, PWM_RES);
    ledcAttachPin(ENA, PWM_CH_LEFT);
    ledcSetup(PWM_CH_RIGHT, PWM_FREQ, PWM_RES);
    ledcAttachPin(ENB, PWM_CH_RIGHT);
  #endif

  stopMotors();
  Serial.printf("Chassis Motors Initialized. PWM speed=%d/255 (~%.0f%%)\n",
                motorSpeed, motorSpeed * 100.0 / 255.0);

  // 3. Setup HC-SR04 Ultrasonic Sensor Pins
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  digitalWrite(TRIG_PIN, LOW);
  Serial.printf("HC-SR04 Ultrasonic Initialized (TRIG=%d, ECHO=%d)\n", TRIG_PIN, ECHO_PIN);

  // 4. Setup I2C & PCA9685 Servo Driver with Timeout Protection
  Wire.setTimeOut(50);
  Wire.begin(21, 22, 100000); // SDA = GPIO 21 (D21), SCL = GPIO 22 (D22) at 100kHz standard mode
  delay(100);

  ensurePcaReady();
  if (pcaReady) {
    armHome();
    Serial.println("PCA9685 6-DOF Arm Initialized to Home Position.");
  } else {
    Serial.println("⚠️ [I2C NOTE] PCA9685 not yet detected at 0x40.");
    Serial.println("   Auto-recovery is active: it will automatically connect when wires/battery are plugged in!");
  }

  // 5. Initialize Bluetooth Serial (High Priority - Immediate Control!)
  Serial.println("[BT] Starting Bluetooth Serial...");
  SerialBT.begin(btDeviceName);
  Serial.println("========================================================");
  Serial.printf ("📶 BLUETOOTH SERIAL ACTIVE: '%s'\n", btDeviceName);
  Serial.println("   Pair your Laptop with 'SMART_WASTE_ROBOT' in Bluetooth Settings!");
  Serial.println("   100% Wireless Serial Link - Zero Wi-Fi / IP Dependency!");
  Serial.println("========================================================");
}

// ==============================================================================
// COMMAND DISPATCHER (100% Bluetooth Serial Commands)
// ==============================================================================
void executeCommand(String cmd) {
  if (cmd == "CMD:FORWARD") {
    moveForward();
  } else if (cmd == "CMD:BACKWARD" || cmd == "CMD:REVERSE") {
    moveBackward();
  } else if (cmd == "CMD:LEFT") {
    steerLeft();
  } else if (cmd == "CMD:RIGHT") {
    steerRight();
  } else if (cmd == "CMD:FORWARD_LEFT") {
    forwardCurveLeft();
  } else if (cmd == "CMD:FORWARD_RIGHT") {
    forwardCurveRight();
  } else if (cmd == "CMD:BACK_LEFT" || cmd == "CMD:BACKWARD_LEFT" || cmd == "CMD:REVERSE_LEFT") {
    backCurveLeft();
  } else if (cmd == "CMD:BACK_RIGHT" || cmd == "CMD:BACKWARD_RIGHT" || cmd == "CMD:REVERSE_RIGHT") {
    backCurveRight();
  } else if (cmd == "CMD:STOP") {
    stopMotors();

  } else if (cmd.startsWith("CMD:SPEED:") || cmd.startsWith("CMD:THROTTLE:")) {
    int val = cmd.startsWith("CMD:SPEED:") ? cmd.substring(10).toInt() : cmd.substring(13).toInt();
    if (val <= 0) {
      motorSpeed = 0;
    } else if (val <= 100) {
      // Percentage: 1%..100% mapped across safe TT motor range
      motorSpeed = map(val, 1, 100, TT_MIN_PWM, TT_MAX_PWM);
    } else {
      motorSpeed = constrain(val, TT_MIN_PWM, TT_MAX_PWM);
    }
    if (isMotorsRunning) {
      setLeftPWM(motorSpeed);
      setRightPWM(motorSpeed);
    }
    Serial.printf("[SPEED] Input=%d -> motorSpeed=%d/255 (%.0f%%)\n", val, motorSpeed,
                  (float)(motorSpeed - TT_MIN_PWM) / (TT_MAX_PWM - TT_MIN_PWM) * 100.0f);

  } else if (cmd.startsWith("CMD:VOLT:")) {
    float v = cmd.substring(9).toFloat();
    motorSpeed = voltageToPWM(v);
    Serial.printf("[VOLTAGE] Target voltage %.1fV -> PWM set to %d/255\n", v, motorSpeed);

  } else if (cmd.startsWith("ARM:PICK_TARGET:")) {
    String rest = cmd.substring(16);
    int c1 = rest.indexOf(':');
    if (c1 > 0) {
      String item = rest.substring(0, c1);
      String remain = rest.substring(c1 + 1);
      int c2 = remain.indexOf(':');
      if (c2 > 0) {
        String dir = remain.substring(0, c2);
        int offsetMs = remain.substring(c2 + 1).toInt();
        executeSmartTargetPickSequence(item, dir, offsetMs);
      } else {
        executeSmartTargetPickSequence(item, "CENTER", 0);
      }
    } else {
      executePickSequence(rest);
    }
  } else if (cmd.startsWith("ARM:PICK:")) {
    String itemType = cmd.substring(9);
    executePickSequence(itemType);
  } else if (cmd == "ARM:HOME") {
    armHome();
  } else if (cmd == "ARM:BIN:RIGHT" || cmd == "ARM:BIN:DEGRADABLE") {
    dropIntoRightDegradableBin(0);
    armHome();
  } else if (cmd == "ARM:BIN:LEFT" || cmd == "ARM:BIN:NON_DEGRADABLE") {
    dropIntoLeftNonDegradableBin(0);
    armHome();
  } else if (cmd.startsWith("ARM:CALIB:RIGHT:") || cmd.startsWith("ARM:CALIB:DEG:")) {
    int colon = cmd.lastIndexOf(':');
    int ms = cmd.substring(colon + 1).toInt();
    if (ms > 100 && ms < 10000) {
      binRight180Ms = ms;
      Serial.printf("[CALIB] binRight180Ms set to %d ms\n", binRight180Ms);
    }
  } else if (cmd.startsWith("ARM:CALIB:LEFT:") || cmd.startsWith("ARM:CALIB:NONDEG:")) {
    int colon = cmd.lastIndexOf(':');
    int ms = cmd.substring(colon + 1).toInt();
    if (ms > 100 && ms < 10000) {
      binLeft180Ms = ms;
      Serial.printf("[CALIB] binLeft180Ms set to %d ms\n", binLeft180Ms);
    }
  } else if (cmd.startsWith("ARM:CALIB:180:")) {
    int ms = cmd.substring(14).toInt();
    if (ms > 100 && ms < 10000) {
      binRight180Ms = ms;
      binLeft180Ms  = ms;
      Serial.printf("[CALIB] Dual 180° ms set to %d ms\n", ms);
    }
  } else if (cmd.startsWith("ARM:SERVO:") || cmd.startsWith("ARM:JOINT:")) {
    int prefixLen = 10;
    int firstColon = cmd.indexOf(':', prefixLen);
    if (firstColon > 0) {
      int ch = cmd.substring(prefixLen, firstColon).toInt();
      int val = cmd.substring(firstColon + 1).toInt();
      if (ch == 0) {
        rotateBaseToAngle(val);
      } else {
        setServoAngle(ch, val);
      }
    }

  } else if (cmd.startsWith("ARM:BASE:")) {
    if (cmd == "ARM:BASE:STOP") {
      baseStop();
    } else if (cmd.startsWith("ARM:BASE:ANGLE:")) {
      int target = cmd.substring(15).toInt();
      rotateBaseToAngle(target);
    } else {
      String rest = cmd.substring(9);
      int c1 = rest.indexOf(':');
      if (c1 > 0) {
        String dir = rest.substring(0, c1);
        String remain = rest.substring(c1 + 1);
        int c2 = remain.indexOf(':');
        if (c2 > 0) {
          int spd = remain.substring(0, c2).toInt();
          int ms = remain.substring(c2 + 1).toInt();
          if (dir == "CW")
            baseRotateCW(spd, ms);
          else
            baseRotateCCW(spd, ms);
        } else {
          int spd = remain.toInt();
          if (dir == "CW")
            baseSetSpeed(spd);
          else
            baseSetSpeed(-spd);
        }
      }
    }
  } else if (cmd == "CMD:TELEMETRY" || cmd == "CMD:GET_SENSORS") {
    sendTelemetry();
  } else if (cmd == "CMD:PING") {
    SerialBT.println("TELEM:PONG");
  } else if (cmd == "CMD:I2C_SCAN") {
    Wire.setTimeOut(50);
    Wire.beginTransmission(0x40);
    byte err = Wire.endTransmission();
    char res[64];
    if (err == 0) {
      pcaReady = true;
      pca.begin();
      pca.setPWMFreq(SERVO_FREQ);
      snprintf(res, sizeof(res), "I2C_STATUS:PCA9685_FOUND_AT_0x40");
      Serial.println("✅ [I2C_SCAN] PCA9685 detected at 0x40 & armed!");
    } else {
      snprintf(res, sizeof(res), "I2C_STATUS:FAILED_CODE_%d", err);
      Serial.printf("❌ [I2C_SCAN] Failed with code %d\n", err);
    }
    SerialBT.println(res);
  }
}

// ==============================================================================
// MAIN LOOP
// ==============================================================================
void loop() {
  // Auto-retry PCA9685 connection every 3 seconds if not ready yet
  if (!pcaReady) {
    static unsigned long lastI2cRetry = 0;
    if (millis() - lastI2cRetry > 3000) {
      lastI2cRetry = millis();
      ensurePcaReady();
    }
  }

  // Rapid 100ms emergency auto-brake check when driving forward
  if (isDrivingForward) {
    static unsigned long lastBrakeCheck = 0;
    if (millis() - lastBrakeCheck > 100) {
      lastBrakeCheck = millis();
      float d = readUltrasonicDistanceCm();
      if (d > 0.0f && d <= OBSTACLE_STOP_DISTANCE_CM) {
        Serial.printf("🚨 [EMERGENCY AUTO-BRAKE] Obstacle at %.1f cm! Motors stopped.\n", d);
        stopMotors();
      }
    }
  }

  // Send periodic telemetry over Bluetooth
  if (millis() - lastTelemetryTime >= TELEMETRY_INTERVAL_MS) {
    lastTelemetryTime = millis();
    sendTelemetry();
  }

  // Auto-Stop Watchdog for M1 Base 360° Motor
  if (baseIsRotating && millis() >= baseRotateEndTime) {
    baseStop();
    Serial.printf("✅ [ARM BASE 360°] Auto-Stop: target %d° reached!\n", currentBaseAngle);
  }

  // ── Receive Bluetooth Serial Commands ─────────────────────────────
  if (SerialBT.available()) {
    String btCmd = SerialBT.readStringUntil('\n');
    btCmd.trim();
    if (btCmd.length() > 0) {
      Serial.printf("📡 [BT RECEIVED] '%s'\n", btCmd.c_str());
      lastPacketTime = millis();
      executeCommand(btCmd);
    }
  }

  // ── Receive USB Serial Commands (Direct USB Cable) ────────────────
  if (Serial.available()) {
    String usbCmd = Serial.readStringUntil('\n');
    usbCmd.trim();
    if (usbCmd.length() > 0) {
      Serial.printf("🔌 [USB RECEIVED] '%s'\n", usbCmd.c_str());
      lastPacketTime = millis();
      executeCommand(usbCmd);
    }
  }

  // Safety Watchdog
  if (millis() - lastPacketTime > SAFETY_TIMEOUT_MS) {
    if (isDrivingForward) {
      stopMotors();
    }
    
  }
}