#ifndef _ADAFRUIT_PWMSERVO_DRIVER_H_
#define _ADAFRUIT_PWMSERVO_DRIVER_H_

#include <Arduino.h>
#include <Wire.h>

/**
 * Lightweight, self-contained PCA9685 I2C 16-Channel Servo Driver
 * Zero external library dependencies required!
 */
class Adafruit_PWMServoDriver {
private:
  uint8_t _i2cAddr;

  void write8(uint8_t reg, uint8_t val) {
    Wire.beginTransmission(_i2cAddr);
    Wire.write(reg);
    Wire.write(val);
    Wire.endTransmission();
  }

  uint8_t read8(uint8_t reg) {
    Wire.beginTransmission(_i2cAddr);
    Wire.write(reg);
    Wire.endTransmission();
    Wire.requestFrom(_i2cAddr, (uint8_t)1);
    return Wire.available() ? Wire.read() : 0;
  }

public:
  Adafruit_PWMServoDriver(uint8_t addr = 0x40) : _i2cAddr(addr) {}

  bool begin() {
    write8(0x00, 0x20); // MODE1: Auto-Increment
    delay(5);
    return true;
  }

  void setPWMFreq(float freq) {
    freq *= 0.95f; // Oscillator calibration
    float prescaleval = 25000000.0f;
    prescaleval /= 4096.0f;
    prescaleval /= freq;
    prescaleval -= 1.0f;
    uint8_t prescale = (uint8_t)(prescaleval + 0.5f);

    uint8_t oldmode = read8(0x00);
    uint8_t newmode = (oldmode & 0x7F) | 0x10; // sleep
    write8(0x00, newmode);                     // sleep mode to set prescale
    write8(0xFE, prescale);                    // set prescale
    write8(0x00, oldmode);
    delay(5);
    write8(0x00, oldmode | 0xA0);              // auto-increment + restart
  }

  void setPWM(uint8_t num, uint16_t on, uint16_t off) {
    Wire.beginTransmission(_i2cAddr);
    Wire.write(0x06 + 4 * num);                // LED0_ON_L + 4*channel
    Wire.write((uint8_t)(on & 0xFF));
    Wire.write((uint8_t)(on >> 8));
    Wire.write((uint8_t)(off & 0xFF));
    Wire.write((uint8_t)(off >> 8));
    Wire.endTransmission();
  }
};

#endif
