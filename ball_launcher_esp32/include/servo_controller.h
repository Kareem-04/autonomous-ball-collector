#ifndef SERVO_CONTROLLER_H
#define SERVO_CONTROLLER_H

#include <Arduino.h>
#include "config.h"

// Servo identifiers
enum ServoID {
  SERVO_HORIZONTAL = 0,
  SERVO_VERTICAL   = 1
};

class ServoController {
public:
  void begin();

  // Set servo to absolute angle (clamped to configured limits)
  void setAngle(ServoID servo, float angleDeg);

  // Smooth move towards target angle at configured speed (call repeatedly in loop)
  void smoothMove(ServoID servo, float targetDeg);

  // Get current angle
  float getAngle(ServoID servo) const;

  // Get configured limits
  float getMinAngle(ServoID servo) const;
  float getMaxAngle(ServoID servo) const;

  // Move both servos to center/home position
  void home();

  // Update smooth movement (call every loop iteration)
  void update();

private:
  float _currentAngle[2];   // Current position of each servo
  float _targetAngle[2];    // Target for smooth movement
  float _maxSpeed[2];       // Degrees per second
  unsigned long _lastUpdate; // Last update timestamp

  // Per-servo configuration
  struct ServoConfig {
    uint8_t  pin;
    uint8_t  channel;
    uint16_t minUs;
    uint16_t maxUs;
    float    minAngle;
    float    maxAngle;
    float    centerAngle;
  };

  ServoConfig _config[2];

  // Convert angle to LEDC duty cycle
  uint32_t angleToDuty(ServoID servo, float angleDeg);

  // Write duty cycle to LEDC
  void writeDuty(ServoID servo, uint32_t duty);
};

#endif // SERVO_CONTROLLER_H
