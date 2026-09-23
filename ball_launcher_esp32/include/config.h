#ifndef CONFIG_H
#define CONFIG_H

// ============================================================================
//  BALL LAUNCHER CONTROL SYSTEM — CONFIGURATION
//  All tunable parameters in one place. Edit values here, not in code files.
// ============================================================================

// ----------------------------------------------------------------------------
//  PIN ASSIGNMENTS — ESP32 DevKitC (38-pin)
// ----------------------------------------------------------------------------

// Horizontal rotation servo (MG996R)
#define SERVO_H_PIN           33

// Vertical tilt servo (FT5325M)
#define SERVO_V_PIN           32

// Brushless motor ESCs
#define MOTOR1_PIN            25
#define MOTOR2_PIN            26

// UWB Module (BU04) — two serial ports on the module
//   Serial2 (UART2): AT commands via USART1 (pins 16/17)
//   UART1:           Data stream via P2/PA2 (pin 18)
#define UWB_TX_PIN            17    // ESP32 TX2 → BU04 USART1 RX (AT commands)
#define UWB_CMD_RX_PIN        16    // ESP32 RX2 ← BU04 USART1 TX (AT responses)
#define UWB_DATA_RX_PIN       18    // ESP32 RX  ← BU04 P2 (PA2)  (data stream)

// Launch button (active LOW with internal pull-up)
#define LAUNCH_BTN_PIN        14

// Status LED (built-in on most DevKitC)
#define STATUS_LED_PIN        2

// ----------------------------------------------------------------------------
//  HORIZONTAL SERVO — MG996R (via bevel gear)
//  Standard hobby servo, ~180° range
//  Mounted vertically, drives horizontal rotation through bevel gear
//  >>> EDIT MIN/MAX ANGLES WHEN YOU MEASURE YOUR MECHANICAL LIMITS <<<
// ----------------------------------------------------------------------------

#define SERVO_H_MIN_US        500     // Pulse width at minimum angle (microseconds)
#define SERVO_H_MAX_US        2500    // Pulse width at maximum angle (microseconds)
#define SERVO_H_MIN_ANGLE     0.0f    // Minimum allowed angle (degrees)
#define SERVO_H_MAX_ANGLE     180.0f  // Maximum allowed angle (degrees)
#define SERVO_H_CENTER        90.0f   // Center/home position (degrees)
#define SERVO_H_SPEED         200.0f  // Max movement speed (degrees/second)
#define SERVO_H_INVERTED      true    // Bevel gear reverses direction
#define SERVO_H_GEAR_RATIO    4.0f    // Gear ratio: servo_teeth / base_teeth
                                      // > 1.0 = servo moves MORE than launcher
                                      // < 1.0 = servo moves LESS than launcher
                                      // Count teeth on both gears to find ratio

// ----------------------------------------------------------------------------
//  VERTICAL SERVO — FT5325M
//  High-torque digital servo, 180° range, 900-2100μs pulse
//  >>> EDIT MIN/MAX ANGLES WHEN YOU MEASURE YOUR MECHANICAL LIMITS <<<
// ----------------------------------------------------------------------------

#define SERVO_V_MIN_US        1280     // Pulse width at minimum angle (microseconds)
#define SERVO_V_MAX_US        1600    // Pulse width at maximum angle (microseconds)
#define SERVO_V_MIN_ANGLE     95.0f    // Minimum allowed angle (degrees) — lowest elevation
#define SERVO_V_MAX_ANGLE     140.0f   // Maximum allowed angle (degrees) — highest elevation
#define SERVO_V_CENTER        110.0f   // Center/home position (degrees)
#define SERVO_V_SPEED         150.0f  // Max movement speed (degrees/second)
#define SERVO_V_INVERTED      false   // Set true if vertical servo direction is reversed

// ----------------------------------------------------------------------------
//  ESP32 LEDC PWM CONFIGURATION
// ----------------------------------------------------------------------------

#define PWM_FREQUENCY         50      // 50Hz for standard servos and ESCs
#define PWM_RESOLUTION        12      // 12-bit resolution (0-4095) — good enough for servos

// LEDC channel assignments (ESP32 has 16 channels, 0-15)
#define SERVO_H_CHANNEL       0
#define SERVO_V_CHANNEL       1
#define MOTOR1_CHANNEL        2
#define MOTOR2_CHANNEL        3

// ----------------------------------------------------------------------------
//  BRUSHLESS MOTORS / ESC CONFIGURATION
//  Standard RC ESCs: 1000μs = stop, 2000μs = full throttle
// ----------------------------------------------------------------------------

#define MOTOR_MIN_US          1000    // ESC minimum pulse (motor off)
#define MOTOR_MAX_US          2000    // ESC maximum pulse (full throttle)
#define MOTOR_ARM_US          1000    // Arming pulse width
#define MOTOR_ARM_TIME_MS     3000    // Time to hold arm pulse (milliseconds)
#define MOTOR_LAUNCH_SPEED    20      // Default launch speed (0-100%)
#define MOTOR_SPINUP_TIME_MS  1500    // Time to reach stable RPM before launch

// ----------------------------------------------------------------------------
//  UWB MODULE — BU04 G451 (PDoA / AoA)
//  Data arrives as JSON:
//  JS006E{"TWR":{"a16":"F482","R":194,"T":...,"D":135,"P":105,"Xcm":78,"Ycm":105,...}}
//    D   = distance in cm
//    P   = PDoA raw phase value
//    Xcm = X position in cm (horizontal offset)
//    Ycm = Y position in cm (forward distance)
//    Angle = atan2(Xcm, Ycm) computed by ESP32
// ----------------------------------------------------------------------------

#define UWB_BAUD_RATE         115200
#define UWB_TIMEOUT_MS        1000    // Timeout for AT command response
#define UWB_UPDATE_INTERVAL   20      // Minimum ms between UWB reads (data arrives ~every 25ms)

// Distance limits (millimeters)
#define UWB_MIN_DISTANCE      200     // Ignore readings closer than 20cm
#define UWB_MAX_DISTANCE      30000   // Ignore readings farther than 30m

// ----------------------------------------------------------------------------
//  TARGET TRACKING — STEP-AND-WAIT CONTROL
//  UWB is on the launcher, so servo movement changes UWB readings.
//  Algorithm: make one small step → wait for readings to settle → check if
//  angle improved → repeat. Prevents feedback oscillation.
// ----------------------------------------------------------------------------

// Settle time: how long to wait after a servo step before reading UWB (ms)
#define TRACK_SETTLE_MS       50     // Wait 250ms after each step (was 400)

// Step size: how many SERVO degrees to move per step
// With gear ratio 4.0, a 5° servo step = 1.25° launcher rotation
#define TRACK_STEP_DEG        5.0f    // Servo degrees per step (was 3)

// Dead-zone: stop tracking when averaged UWB angle is within ±N degrees
#define TRACKING_DEADZONE_H   8.0f    // Horizontal (degrees) — PDoA noise is ±10°
#define TRACKING_DEADZONE_V   3.0f    // Vertical (degrees)

// Discard samples during first N ms after a step (servo still moving)
#define TRACK_DISCARD_MS      80      // Ignore readings during servo motion (was 150)

// Low-pass filter alpha (used for distance/vertical only now)
#define FILTER_ALPHA_DIST     0.15f   // Distance filter
#define FILTER_ALPHA_ANGLE    0.15f   // Angle filter (moderate — step-wait handles H noise)

// Motor speed hysteresis — only update if change > this %
#define MOTOR_SPEED_HYSTERESIS 3

// Vertical angle geometry
// The vertical servo angle depends on the target distance and relative height.
// Set these based on your physical setup:
#define LAUNCHER_HEIGHT_MM    500.0f  // Height of launcher above ground (mm)
#define TARGET_HEIGHT_MM      500.0f  // Height of target above ground (mm)
// If target is at same height, vertical angle ≈ 0° (horizontal shot)
// Positive angle = aim upward, negative = aim downward

// Gravity compensation for ballistic trajectory
// The ball drops due to gravity, so the launcher must aim ABOVE the target.
// Formula: loft_degrees = factor × distance_meters²
// Examples with factor=3.0:
//   1m  → +3°  loft
//   2m  → +12° loft
//   3m  → +27° loft
// >>> TUNE THIS based on actual ball trajectory at your motor speed <<<
#define GRAVITY_COMP_FACTOR   3.0f    // Degrees per meter² (start here, tune up/down)

// Distance-based motor speed (linear ramp)
// Closer target = softer throw, further target = harder throw
//   speed% = MIN + (distance - MIN_DIST) / (MAX_DIST - MIN_DIST) × (MAX - MIN)
// >>> TUNE: if ball overshoots, lower MAX. If undershoots, raise MAX. <<<
#define AUTO_SPEED_ENABLED    true    // Auto-adjust motor speed during tracking
#define AUTO_SPEED_MIN        5      // Motor % at closest distance
#define AUTO_SPEED_MAX        10      // Motor % at furthest distance
#define AUTO_SPEED_MIN_DIST   0.5f    // Distance (m) for minimum speed
#define AUTO_SPEED_MAX_DIST   5.0f    // Distance (m) for maximum speed

// ----------------------------------------------------------------------------
//  LAUNCH SEQUENCE TIMING
// ----------------------------------------------------------------------------

#define LAUNCH_SPINUP_MS      1500    // Motor spin-up before launch
#define LAUNCH_HOLD_MS        500     // Hold at full speed during launch
#define LAUNCH_COOLDOWN_MS    2000    // Cool-down after launch before next

// ----------------------------------------------------------------------------
//  SERIAL MONITOR
// ----------------------------------------------------------------------------

#define SERIAL_BAUD           115200
#define STATUS_PRINT_INTERVAL 500     // Print status every N ms when tracking

#endif // CONFIG_H
