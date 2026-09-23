#include "tracking.h"
#include <math.h>

void Tracking::begin(UWBManager* uwb, ServoController* servos, LauncherMotor* motors) {
  _uwb     = uwb;
  _servos  = servos;
  _motors  = motors;
  _enabled = false;

  _targetH = SERVO_H_CENTER;
  _targetV = SERVO_V_CENTER;
  _autoSpeed = 0;

  _state        = TRACK_IDLE;
  _stepTime     = 0;
  _angleSamples = 0;
  _sampleCount  = 0;
  _lastAvgAngle = 0;

  Serial.println("[TRACK] Step-and-wait tracking initialized");
  Serial.printf("[TRACK] Step size: %.1f° servo (%.2f° launcher)\n",
    TRACK_STEP_DEG, TRACK_STEP_DEG / SERVO_H_GEAR_RATIO);
  Serial.printf("[TRACK] Settle: %dms (discard first %dms)\n",
    TRACK_SETTLE_MS, TRACK_DISCARD_MS);
  Serial.printf("[TRACK] Deadzone: ±%.1f° horizontal\n", TRACKING_DEADZONE_H);
  Serial.printf("[TRACK] Gear ratio: %.1f  inverted: %s\n",
    SERVO_H_GEAR_RATIO, SERVO_H_INVERTED ? "YES" : "NO");
}

void Tracking::update() {
  if (!_enabled) return;

  if (!_uwb->hasValidData(1500)) {
    return;  // No data — hold position
  }

  // Get raw UWB angle (0° = target centered ahead of launcher)
  UWBData data = _uwb->getData();
  float rawAngle = 0;
  if (data.x_cm != 0 || data.y_cm != 0) {
    rawAngle = atan2(data.x_cm, data.y_cm) * 180.0f / PI;
  }

  float distance = _uwb->getFilteredDistance();
  unsigned long now = millis();

  // =========================================================================
  //  HORIZONTAL — Step-and-wait algorithm
  //
  //  1. IDLE: Collect angle samples over the settle window
  //  2. After settle time: average the samples
  //  3. If average > deadzone: make ONE small servo step toward target
  //  4. Enter STEPPING state, wait for settle again
  //  5. If average < deadzone: LOCKED — hold position
  // =========================================================================

  unsigned long elapsed = now - _stepTime;

  if (_state == TRACK_STEPPING) {
    // Just made a step — wait for servo to physically move
    if (elapsed < TRACK_DISCARD_MS) {
      // Still moving — discard all readings
      return;
    }

    if (elapsed < TRACK_SETTLE_MS) {
      // Servo stopped, collecting samples during settle window
      _angleSamples += rawAngle;
      _sampleCount++;
      return;
    }

    // Settle time complete — evaluate
    if (_sampleCount > 0) {
      _lastAvgAngle = _angleSamples / (float)_sampleCount;
    }

    // Reset for next cycle
    _angleSamples = 0;
    _sampleCount  = 0;

    // Check if we're on target
    if (fabs(_lastAvgAngle) <= TRACKING_DEADZONE_H) {
      _state = TRACK_LOCKED;
      Serial.printf("[TRACK] LOCKED — avg angle: %.1f° (within ±%.1f°)\n",
        _lastAvgAngle, TRACKING_DEADZONE_H);
    } else {
      _state = TRACK_IDLE;  // Not on target yet, will step again
    }
  }

  if (_state == TRACK_LOCKED) {
    // On target — keep collecting samples to detect if target moves
    _angleSamples += rawAngle;
    _sampleCount++;

    // Check every settle period if we're still on target
    if (elapsed >= TRACK_SETTLE_MS && _sampleCount > 0) {
      _lastAvgAngle = _angleSamples / (float)_sampleCount;
      _angleSamples = 0;
      _sampleCount  = 0;
      _stepTime     = now;

      if (fabs(_lastAvgAngle) > TRACKING_DEADZONE_H) {
        // Target moved — re-engage tracking
        _state = TRACK_IDLE;
        Serial.printf("[TRACK] Target moved (avg: %.1f°) — re-tracking\n", _lastAvgAngle);
      }
    }
  }

  if (_state == TRACK_IDLE) {
    // Collect samples
    _angleSamples += rawAngle;
    _sampleCount++;

    // Need enough samples for a good average
    if (elapsed >= TRACK_SETTLE_MS && _sampleCount >= 5) {
      _lastAvgAngle = _angleSamples / (float)_sampleCount;
      _angleSamples = 0;
      _sampleCount  = 0;

      if (fabs(_lastAvgAngle) <= TRACKING_DEADZONE_H) {
        // Already on target
        _state = TRACK_LOCKED;
        _stepTime = now;
      } else {
        // Make one step toward the target
        float step = TRACK_STEP_DEG;

        // Direction: if UWB says target is positive (right),
        // we need to rotate launcher right
        if (SERVO_H_INVERTED) {
          // Bevel gear: decrease servo to rotate launcher right (toward positive angle)
          if (_lastAvgAngle > 0) {
            _targetH -= step;  // Step servo down → launcher turns right
          } else {
            _targetH += step;  // Step servo up → launcher turns left
          }
        } else {
          if (_lastAvgAngle > 0) {
            _targetH += step;
          } else {
            _targetH -= step;
          }
        }

        // Clamp to servo limits
        _targetH = constrain(_targetH,
          _servos->getMinAngle(SERVO_HORIZONTAL),
          _servos->getMaxAngle(SERVO_HORIZONTAL));

        // Execute the step
        _servos->setAngle(SERVO_HORIZONTAL, _targetH);

        Serial.printf("[TRACK] STEP → SH:%.1f° (avg angle was %.1f°)\n",
          _targetH, _lastAvgAngle);

        _state    = TRACK_STEPPING;
        _stepTime = now;
      }
    }
  }

  // =========================================================================
  //  VERTICAL — Direct from distance (no feedback issue, vertical is simpler)
  // =========================================================================

  _targetV = computeVerticalTarget(distance);
  _servos->setAngle(SERVO_VERTICAL, _targetV);

  // =========================================================================
  //  AUTO MOTOR SPEED
  // =========================================================================

  if (AUTO_SPEED_ENABLED) {
    int newSpeed = computeMotorSpeed(distance);
    if (abs(newSpeed - _autoSpeed) >= MOTOR_SPEED_HYSTERESIS) {
      _autoSpeed = newSpeed;
      _motors->setSpeed(_autoSpeed);
    }
  }
}

float Tracking::computeVerticalTarget(float distanceMm) {
  float heightDiff = TARGET_HEIGHT_MM - LAUNCHER_HEIGHT_MM;

  float slantSq  = distanceMm * distanceMm;
  float heightSq = heightDiff * heightDiff;
  float horizDist = (slantSq > heightSq) ? sqrt(slantSq - heightSq) : distanceMm;

  float elevDeg = atan2(heightDiff, horizDist) * 180.0f / PI;

  float distM = distanceMm / 1000.0f;
  float gravComp = GRAVITY_COMP_FACTOR * distM * distM;

  float totalAngle = elevDeg + gravComp;

  float servoAngle;
  if (SERVO_V_INVERTED) {
    servoAngle = SERVO_V_CENTER - totalAngle;
  } else {
    servoAngle = SERVO_V_CENTER + totalAngle;
  }

  return constrain(servoAngle,
    _servos->getMinAngle(SERVO_VERTICAL),
    _servos->getMaxAngle(SERVO_VERTICAL));
}

int Tracking::computeMotorSpeed(float distanceMm) {
  float distM = distanceMm / 1000.0f;
  distM = constrain(distM, AUTO_SPEED_MIN_DIST, AUTO_SPEED_MAX_DIST);

  float fraction = (distM - AUTO_SPEED_MIN_DIST) / (AUTO_SPEED_MAX_DIST - AUTO_SPEED_MIN_DIST);
  int speed = AUTO_SPEED_MIN + (int)(fraction * (AUTO_SPEED_MAX - AUTO_SPEED_MIN));

  return constrain(speed, 0, 100);
}

void Tracking::enable() {
  _enabled = true;
  _autoSpeed = 0;

  _state        = TRACK_IDLE;
  _stepTime     = millis();
  _angleSamples = 0;
  _sampleCount  = 0;
  _lastAvgAngle = 0;

  _targetH = _servos->getAngle(SERVO_HORIZONTAL);
  _targetV = _servos->getAngle(SERVO_VERTICAL);

  Serial.println("[TRACK] *** TRACKING ENABLED ***");
  Serial.println("[TRACK] Step-and-wait: step → settle → check → repeat");
}

void Tracking::disable() {
  _enabled = false;
  _autoSpeed = 0;
  _state = TRACK_IDLE;

  if (AUTO_SPEED_ENABLED) {
    _motors->setSpeed(0);
  }
  Serial.println("[TRACK] Tracking disabled — servos hold, motors stopped");
}

bool Tracking::isEnabled() const { return _enabled; }

float Tracking::getTargetHorizontalAngle() const { return _targetH; }
float Tracking::getTargetVerticalAngle() const { return _targetV; }
int   Tracking::getAutoSpeed() const { return _autoSpeed; }

void Tracking::printStatus() {
  const char* stateStr = "?";
  switch (_state) {
    case TRACK_IDLE:     stateStr = "IDLE (collecting)"; break;
    case TRACK_STEPPING: stateStr = "STEPPING (settling)"; break;
    case TRACK_LOCKED:   stateStr = "LOCKED (on target)"; break;
  }

  Serial.println("[TRACK] --- Status ---");
  Serial.printf("[TRACK] Enabled: %s\n", _enabled ? "YES" : "NO");
  Serial.printf("[TRACK] State: %s\n", stateStr);
  Serial.printf("[TRACK] Servo H: %.1f°  V: %.1f°\n", _targetH, _targetV);
  Serial.printf("[TRACK] Last avg angle: %.1f° (deadzone ±%.1f°)\n",
    _lastAvgAngle, TRACKING_DEADZONE_H);
  Serial.printf("[TRACK] Samples: %d  Auto speed: %d%%\n", _sampleCount, _autoSpeed);

  if (_uwb->hasValidData()) {
    Serial.printf("[TRACK] UWB: dist=%.0fmm angle=%.1f°\n",
      _uwb->getFilteredDistance(), _uwb->getFilteredAngle());
  }
  Serial.println("[TRACK] ----------------");
}
