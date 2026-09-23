#ifndef TRACKING_H
#define TRACKING_H

#include <Arduino.h>
#include "config.h"
#include "uwb_manager.h"
#include "servo_controller.h"
#include "launcher_motor.h"

// Step-and-wait tracking states
enum TrackState {
  TRACK_IDLE,         // Waiting, collecting angle samples
  TRACK_STEPPING,     // Just made a servo step, waiting to settle
  TRACK_LOCKED        // Within deadzone, on target
};

class Tracking {
public:
  void begin(UWBManager* uwb, ServoController* servos, LauncherMotor* motors);

  void update();

  void enable();
  void disable();
  bool isEnabled() const;

  float getTargetHorizontalAngle() const;
  float getTargetVerticalAngle() const;
  int   getAutoSpeed() const;

  void printStatus();

private:
  UWBManager*      _uwb;
  ServoController* _servos;
  LauncherMotor*   _motors;
  bool             _enabled;

  float _targetH;
  float _targetV;
  int   _autoSpeed;

  // Step-and-wait state
  TrackState    _state;
  unsigned long _stepTime;        // When last step was made
  float         _angleSamples;    // Sum of angle samples during settle
  int           _sampleCount;     // Number of samples collected
  float         _lastAvgAngle;    // Average angle from last settle period

  float computeVerticalTarget(float distanceMm);
  int   computeMotorSpeed(float distanceMm);
};

#endif // TRACKING_H
