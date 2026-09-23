#include "launcher_motor.h"

void LauncherMotor::begin() {
  _currentSpeed = 0;
  _armed = false;

  // Setup LEDC channels for both ESCs
  ledcSetup(MOTOR1_CHANNEL, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcSetup(MOTOR2_CHANNEL, PWM_FREQUENCY, PWM_RESOLUTION);

  ledcAttachPin(MOTOR1_PIN, MOTOR1_CHANNEL);
  ledcAttachPin(MOTOR2_PIN, MOTOR2_CHANNEL);

  Serial.println("[MOTOR] ESC channels initialized");
  Serial.printf("[MOTOR] Motor1=GPIO%d (CH%d), Motor2=GPIO%d (CH%d)\n",
    MOTOR1_PIN, MOTOR1_CHANNEL, MOTOR2_PIN, MOTOR2_CHANNEL);

  // Arm the ESCs
  armESCs();
}

void LauncherMotor::armESCs() {
  Serial.println("[MOTOR] Arming ESCs...");
  Serial.printf("[MOTOR] Sending %dμs for %d ms\n", MOTOR_ARM_US, MOTOR_ARM_TIME_MS);

  writeMicroseconds(MOTOR_ARM_US);
  delay(MOTOR_ARM_TIME_MS);

  _armed = true;
  Serial.println("[MOTOR] ESCs armed ✓");
}

void LauncherMotor::setSpeed(int percent) {
  if (!_armed) {
    Serial.println("[MOTOR] ERROR: ESCs not armed!");
    return;
  }

  percent = constrain(percent, 0, 100);
  _currentSpeed = percent;

  // Map 0-100% to MOTOR_MIN_US-MOTOR_MAX_US
  uint16_t pulseUs = MOTOR_MIN_US + (uint16_t)((float)percent / 100.0f * (MOTOR_MAX_US - MOTOR_MIN_US));

  writeMicroseconds(pulseUs);

  Serial.printf("[MOTOR] Speed set to %d%% (%dμs)\n", percent, pulseUs);
}

int LauncherMotor::getSpeed() const {
  return _currentSpeed;
}

void LauncherMotor::spinUp(int speedPercent) {
  if (!_armed) {
    Serial.println("[MOTOR] ERROR: ESCs not armed!");
    return;
  }

  Serial.printf("[MOTOR] Spinning up to %d%%...\n", speedPercent);

  // Gradual ramp-up to prevent current spikes
  int steps = 10;
  int delayPerStep = MOTOR_SPINUP_TIME_MS / steps;

  for (int i = 1; i <= steps; i++) {
    int intermediateSpeed = (speedPercent * i) / steps;
    setSpeed(intermediateSpeed);
    delay(delayPerStep);
  }

  setSpeed(speedPercent);
  Serial.printf("[MOTOR] Spin-up complete at %d%%\n", speedPercent);
}

void LauncherMotor::emergencyStop() {
  _currentSpeed = 0;
  writeMicroseconds(MOTOR_MIN_US);
  Serial.println("[MOTOR] *** EMERGENCY STOP ***");
}

bool LauncherMotor::isSpinning() const {
  return _currentSpeed > 0;
}

bool LauncherMotor::isArmed() const {
  return _armed;
}

void LauncherMotor::writeMicroseconds(uint16_t us) {
  // Convert microseconds to LEDC duty cycle
  // At 50Hz, period = 20000μs
  // With 16-bit resolution: duty = (us / 20000) * 65536
  float period = 1000000.0f / PWM_FREQUENCY;
  uint32_t maxDuty = (1 << PWM_RESOLUTION) - 1;
  uint32_t duty = (uint32_t)((float)us / period * maxDuty);

  ledcWrite(MOTOR1_CHANNEL, duty);
  ledcWrite(MOTOR2_CHANNEL, duty);
}
