#ifndef LAUNCHER_MOTOR_H
#define LAUNCHER_MOTOR_H

#include <Arduino.h>
#include "config.h"

class LauncherMotor {
public:
  void begin();

  // Set motor speed as percentage (0-100%)
  // Both motors always run at the same speed
  void setSpeed(int percent);

  // Get current speed percentage
  int getSpeed() const;

  // Spin up to launch speed and wait for stability
  void spinUp(int speedPercent = MOTOR_LAUNCH_SPEED);

  // Emergency stop — immediate zero throttle
  void emergencyStop();

  // Check if motors are spinning
  bool isSpinning() const;

  // Check if ESCs are armed
  bool isArmed() const;

private:
  int  _currentSpeed;    // Current speed percentage (0-100)
  bool _armed;

  // Write pulse width to both ESCs
  void writeMicroseconds(uint16_t us);

  // Arm ESCs (send minimum throttle for required duration)
  void armESCs();
};

#endif // LAUNCHER_MOTOR_H
