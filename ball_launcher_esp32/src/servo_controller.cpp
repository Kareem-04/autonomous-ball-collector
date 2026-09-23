#include "servo_controller.h"

void ServoController::begin() {
  // Configure horizontal servo (MG996R)
  _config[SERVO_HORIZONTAL] = {
    SERVO_H_PIN,
    SERVO_H_CHANNEL,
    SERVO_H_MIN_US,
    SERVO_H_MAX_US,
    SERVO_H_MIN_ANGLE,
    SERVO_H_MAX_ANGLE,
    SERVO_H_CENTER
  };

  // Configure vertical servo (FT5325M)
  _config[SERVO_VERTICAL] = {
    SERVO_V_PIN,
    SERVO_V_CHANNEL,
    SERVO_V_MIN_US,
    SERVO_V_MAX_US,
    SERVO_V_MIN_ANGLE,
    SERVO_V_MAX_ANGLE,
    SERVO_V_CENTER
  };

  _maxSpeed[SERVO_HORIZONTAL] = SERVO_H_SPEED;
  _maxSpeed[SERVO_VERTICAL]   = SERVO_V_SPEED;

  // Initialize LEDC channels for both servos
  for (int i = 0; i < 2; i++) {
    ledcSetup(_config[i].channel, PWM_FREQUENCY, PWM_RESOLUTION);
    ledcAttachPin(_config[i].pin, _config[i].channel);

    _currentAngle[i] = _config[i].centerAngle;
    _targetAngle[i]  = _config[i].centerAngle;
  }

  _lastUpdate = millis();

  // Move to home/center position
  home();

  Serial.println("[SERVO] Initialized — H: MG996R, V: FT5325M");
  Serial.printf("[SERVO] H limits: %.1f° - %.1f° (center %.1f°)\n",
    _config[SERVO_HORIZONTAL].minAngle,
    _config[SERVO_HORIZONTAL].maxAngle,
    _config[SERVO_HORIZONTAL].centerAngle);
  Serial.printf("[SERVO] V limits: %.1f° - %.1f° (center %.1f°)\n",
    _config[SERVO_VERTICAL].minAngle,
    _config[SERVO_VERTICAL].maxAngle,
    _config[SERVO_VERTICAL].centerAngle);
}

void ServoController::setAngle(ServoID servo, float angleDeg) {
  // Clamp to configured limits
  float clamped = constrain(angleDeg, _config[servo].minAngle, _config[servo].maxAngle);
  _currentAngle[servo] = clamped;
  _targetAngle[servo]  = clamped;

  uint32_t duty = angleToDuty(servo, clamped);
  writeDuty(servo, duty);
}

void ServoController::smoothMove(ServoID servo, float targetDeg) {
  // Set target — actual movement happens in update()
  _targetAngle[servo] = constrain(targetDeg, _config[servo].minAngle, _config[servo].maxAngle);
}

float ServoController::getAngle(ServoID servo) const {
  return _currentAngle[servo];
}

float ServoController::getMinAngle(ServoID servo) const {
  return _config[servo].minAngle;
}

float ServoController::getMaxAngle(ServoID servo) const {
  return _config[servo].maxAngle;
}

void ServoController::home() {
  setAngle(SERVO_HORIZONTAL, _config[SERVO_HORIZONTAL].centerAngle);
  setAngle(SERVO_VERTICAL,   _config[SERVO_VERTICAL].centerAngle);
  Serial.println("[SERVO] Moved to home position");
}

void ServoController::update() {
  unsigned long now = millis();
  float dt = (now - _lastUpdate) / 1000.0f;  // delta time in seconds
  _lastUpdate = now;

  if (dt <= 0.0f || dt > 0.5f) {
    dt = 0.02f;  // Fallback to ~50Hz if timing is weird
  }

  for (int i = 0; i < 2; i++) {
    float error = _targetAngle[i] - _currentAngle[i];

    if (fabs(error) < 0.1f) {
      continue;  // Close enough, no movement needed
    }

    // Limit movement speed
    float maxStep = _maxSpeed[i] * dt;
    float step = constrain(error, -maxStep, maxStep);

    float newAngle = _currentAngle[i] + step;
    newAngle = constrain(newAngle, _config[i].minAngle, _config[i].maxAngle);

    _currentAngle[i] = newAngle;

    uint32_t duty = angleToDuty((ServoID)i, newAngle);
    writeDuty((ServoID)i, duty);
  }
}

uint32_t ServoController::angleToDuty(ServoID servo, float angleDeg) {
  // Map angle to pulse width in microseconds
  float range = _config[servo].maxAngle - _config[servo].minAngle;
  if (range < 1.0f) range = 1.0f;  // Safety

  float fraction = (angleDeg - _config[servo].minAngle) / range;
  float pulseUs = _config[servo].minUs + fraction * (_config[servo].maxUs - _config[servo].minUs);

  // Convert microseconds to LEDC duty cycle
  // At 50Hz, period = 20000μs
  // With 16-bit resolution: duty = (pulseUs / 20000) * 65536
  float period = 1000000.0f / PWM_FREQUENCY;  // Period in microseconds
  uint32_t maxDuty = (1 << PWM_RESOLUTION) - 1;
  uint32_t duty = (uint32_t)((pulseUs / period) * maxDuty);

  return duty;
}

void ServoController::writeDuty(ServoID servo, uint32_t duty) {
  ledcWrite(_config[servo].channel, duty);
}
