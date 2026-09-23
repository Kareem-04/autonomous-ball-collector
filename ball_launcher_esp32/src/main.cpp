// ============================================================================
//  BALL LAUNCHER CONTROL SYSTEM — ESP32
// ============================================================================
//  Robot-mounted ball launcher with UWB target tracking
//
//  Hardware:
//    - ESP32 DevKitC
//    - BU04 G451 UWB module (Anchor, dual antenna PDoA for AoA)
//    - MG996R servo (horizontal rotation)
//    - FT5325M servo (vertical tilt)
//    - 2x Brushless motors with standard RC ESCs (ball launcher wheels)
//    - Launch button
//
//  The launcher continuously tracks a UWB Tag on the target in both
//  horizontal (via PDoA angle) and vertical (via distance + geometry) axes.
//
//  Serial Commands (115200 baud):
//    SH <angle>     - Set horizontal servo angle (0-180)
//    SV <angle>     - Set vertical servo angle (0-90)
//    MS <speed%>    - Set motor speed (0-100)
//    MSTOP          - Emergency stop motors
//    TRACK          - Toggle auto-tracking ON/OFF
//    LAUNCH         - Execute launch sequence
//    UWB            - Show UWB status
//    UWBSETUP       - Run UWB anchor setup
//    UWBPASS        - Enter UWB AT passthrough mode (type EXIT to leave)
//    HOME           - Move servos to center
//    STATUS         - Print full system status
//    HELP           - Show command list
// ============================================================================

#include <Arduino.h>
#include "config.h"
#include "uwb_manager.h"
#include "servo_controller.h"
#include "launcher_motor.h"
#include "tracking.h"

// --- Global subsystem instances ---
UWBManager      uwb;
ServoController servos;
LauncherMotor   motors;
Tracking        tracker;

// --- System state ---
enum SystemState {
  STATE_INIT,
  STATE_IDLE,
  STATE_TRACKING,
  STATE_LAUNCHING,
  STATE_UWB_PASSTHROUGH
};

SystemState     systemState   = STATE_INIT;
bool            launchArmed   = false;
unsigned long   lastStatusPrint = 0;
unsigned long   launchStartTime = 0;
int             launchPhase     = 0;

// --- Serial command buffer ---
char cmdBuf[128];
int  cmdPos = 0;

// --- Function prototypes ---
void processSerialCommand(const char* cmd);
void handleLaunchSequence();
void printHelp();
void printFullStatus();

// ============================================================================
//  SETUP
// ============================================================================
void setup() {
  // Initialize serial monitor
  Serial.begin(SERIAL_BAUD);
  delay(500);

  Serial.println();
  Serial.println("╔══════════════════════════════════════════════╗");
  Serial.println("║     BALL LAUNCHER CONTROL SYSTEM v1.0       ║");
  Serial.println("║     ESP32 + UWB + Servo + Brushless         ║");
  Serial.println("╚══════════════════════════════════════════════╝");
  Serial.println();

  // Status LED
  pinMode(STATUS_LED_PIN, OUTPUT);
  digitalWrite(STATUS_LED_PIN, LOW);

  // Launch button (active LOW, internal pull-up)
  pinMode(LAUNCH_BTN_PIN, INPUT_PULLUP);

  // Initialize subsystems
  Serial.println("--- Initializing Servos ---");
  servos.begin();
  Serial.println();

  Serial.println("--- Initializing Brushless Motors ---");
  motors.begin();
  Serial.println();

  Serial.println("--- Initializing UWB Module ---");
  uwb.begin();
  Serial.println();

  Serial.println("--- Initializing Tracking System ---");
  tracker.begin(&uwb, &servos, &motors);
  Serial.println();

  // System ready
  systemState = STATE_IDLE;
  digitalWrite(STATUS_LED_PIN, HIGH);

  Serial.println("╔══════════════════════════════════════════════╗");
  Serial.println("║  SYSTEM READY                               ║");
  Serial.println("║                                             ║");
  Serial.println("║  Type 'HELP' for available commands         ║");
  Serial.println("║  Type 'UWBSETUP' to configure UWB module   ║");
  Serial.println("║  Type 'TRACK' to start auto-tracking        ║");
  Serial.println("╚══════════════════════════════════════════════╝");
  Serial.println();
}

// ============================================================================
//  MAIN LOOP
// ============================================================================
void loop() {
  // --- Handle UWB passthrough mode separately ---
  if (systemState == STATE_UWB_PASSTHROUGH) {
    uwb.handlePassthrough();

    // Check for EXIT command from serial monitor
    while (Serial.available()) {
      char c = Serial.read();
      if (c == '\n' || c == '\r') {
        if (cmdPos > 0) {
          cmdBuf[cmdPos] = '\0';
          if (strcasecmp(cmdBuf, "EXIT") == 0) {
            uwb.disablePassthrough();
            systemState = STATE_IDLE;
            Serial.println("[SYS] Returned to normal mode");
          } else {
            // Forward to UWB module
            uwb.sendCommand(cmdBuf);
          }
          cmdPos = 0;
        }
      } else if (cmdPos < (int)sizeof(cmdBuf) - 1) {
        cmdBuf[cmdPos++] = c;
      }
    }
    return;
  }

  // --- Read serial commands ---
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (cmdPos > 0) {
        cmdBuf[cmdPos] = '\0';
        processSerialCommand(cmdBuf);
        cmdPos = 0;
      }
    } else if (cmdPos < (int)sizeof(cmdBuf) - 1) {
      cmdBuf[cmdPos++] = c;
    }
  }

  // --- Update UWB data ---
  uwb.update();

  // --- Update tracking (computes servo targets from UWB) ---
  tracker.update();

  // --- Update servos (smooth movement) ---
  servos.update();

  // --- Handle launch button ---
  if (digitalRead(LAUNCH_BTN_PIN) == LOW && systemState == STATE_TRACKING) {
    // Debounce
    delay(50);
    if (digitalRead(LAUNCH_BTN_PIN) == LOW) {
      Serial.println("[SYS] Launch button pressed!");
      systemState = STATE_LAUNCHING;
      launchPhase = 0;
      launchStartTime = millis();
    }
  }

  // --- Handle launch sequence ---
  if (systemState == STATE_LAUNCHING) {
    handleLaunchSequence();
  }

  // --- Status LED blink pattern ---
  static unsigned long lastBlink = 0;
  unsigned long blinkInterval;

  switch (systemState) {
    case STATE_IDLE:      blinkInterval = 2000; break;  // Slow blink
    case STATE_TRACKING:  blinkInterval = 500;  break;  // Medium blink
    case STATE_LAUNCHING: blinkInterval = 100;  break;  // Fast blink
    default:              blinkInterval = 1000; break;
  }

  if (millis() - lastBlink > blinkInterval) {
    digitalWrite(STATUS_LED_PIN, !digitalRead(STATUS_LED_PIN));
    lastBlink = millis();
  }

  // --- Periodic status print when tracking ---
  if (systemState == STATE_TRACKING && millis() - lastStatusPrint > STATUS_PRINT_INTERVAL) {
    if (uwb.hasValidData()) {
      Serial.printf("[LIVE] D:%.0fcm X:%.0f Y:%.0f A:%.1f° → SH:%.1f° SV:%.1f° | M:%d%%\n",
        uwb.getFilteredDistance() / 10.0f,
        uwb.getFilteredX(),
        uwb.getFilteredY(),
        uwb.getFilteredAngle(),
        servos.getAngle(SERVO_HORIZONTAL),
        servos.getAngle(SERVO_VERTICAL),
        motors.getSpeed());
    }
    lastStatusPrint = millis();
  }

  // Small delay to prevent watchdog issues
  delay(5);
}

// ============================================================================
//  SERIAL COMMAND PROCESSOR
// ============================================================================
void processSerialCommand(const char* cmd) {
  Serial.printf("[CMD] > %s\n", cmd);

  // --- SH <angle> — Set horizontal servo ---
  if (strncasecmp(cmd, "SH ", 3) == 0) {
    float angle = atof(cmd + 3);
    servos.setAngle(SERVO_HORIZONTAL, angle);
    Serial.printf("[CMD] Horizontal servo → %.1f°\n", servos.getAngle(SERVO_HORIZONTAL));
    return;
  }

  // --- SV <angle> — Set vertical servo ---
  if (strncasecmp(cmd, "SV ", 3) == 0) {
    float angle = atof(cmd + 3);
    servos.setAngle(SERVO_VERTICAL, angle);
    Serial.printf("[CMD] Vertical servo → %.1f°\n", servos.getAngle(SERVO_VERTICAL));
    return;
  }

  // --- MS <speed%> — Set motor speed ---
  if (strncasecmp(cmd, "MS ", 3) == 0) {
    int speed = atoi(cmd + 3);
    motors.setSpeed(speed);
    return;
  }

  // --- MSTOP — Emergency stop ---
  if (strcasecmp(cmd, "MSTOP") == 0) {
    motors.emergencyStop();
    return;
  }

  // --- TRACK — Toggle tracking ---
  if (strcasecmp(cmd, "TRACK") == 0) {
    if (tracker.isEnabled()) {
      tracker.disable();
      systemState = STATE_IDLE;
    } else {
      tracker.enable();
      systemState = STATE_TRACKING;
    }
    return;
  }

  // --- LAUNCH — Execute launch sequence ---
  if (strcasecmp(cmd, "LAUNCH") == 0) {
    if (systemState == STATE_LAUNCHING) {
      Serial.println("[CMD] Already launching!");
      return;
    }
    Serial.println("[CMD] Starting launch sequence...");
    systemState = STATE_LAUNCHING;
    launchPhase = 0;
    launchStartTime = millis();
    return;
  }

  // --- UWB — Show UWB status ---
  if (strcasecmp(cmd, "UWB") == 0) {
    uwb.printStatus();
    return;
  }

  // --- UWBSETUP — Run UWB setup ---
  if (strcasecmp(cmd, "UWBSETUP") == 0) {
    uwb.setupAnchor();
    return;
  }

  // --- UWBPASS — UWB passthrough ---
  if (strcasecmp(cmd, "UWBPASS") == 0) {
    uwb.enablePassthrough();
    systemState = STATE_UWB_PASSTHROUGH;
    return;
  }

  // --- RAWUWB — Debug: dump raw bytes from data serial (GPIO 18) ---
  if (strcasecmp(cmd, "RAWUWB") == 0) {
    uwb.debugRawData();
    return;
  }

  // --- HOME — Servos to center ---
  if (strcasecmp(cmd, "HOME") == 0) {
    servos.home();
    return;
  }

  // --- STATUS — Full system status ---
  if (strcasecmp(cmd, "STATUS") == 0) {
    printFullStatus();
    return;
  }

  // --- HELP — Show commands ---
  if (strcasecmp(cmd, "HELP") == 0) {
    printHelp();
    return;
  }

  // --- CAL — Enter calibration guide ---
  if (strcasecmp(cmd, "CAL") == 0) {
    Serial.println();
    Serial.println("╔══════════════════════════════════════════════╗");
    Serial.println("║  SERVO CALIBRATION GUIDE                    ║");
    Serial.println("╚══════════════════════════════════════════════╝");
    Serial.println();
    Serial.println("  1. Use 'SH <angle>' to find horizontal limits");
    Serial.println("     Start from 90 (center) and go both ways");
    Serial.println("     Note where the mechanism hits its stops");
    Serial.println();
    Serial.println("  2. Use 'SV <angle>' to find vertical limits");
    Serial.println("     Start from 45 (mid) and go both ways");
    Serial.println("     Note where the mechanism hits its stops");
    Serial.println();
    Serial.println("  3. Edit these values in config.h:");
    Serial.println("     SERVO_H_MIN_ANGLE, SERVO_H_MAX_ANGLE");
    Serial.println("     SERVO_V_MIN_ANGLE, SERVO_V_MAX_ANGLE");
    Serial.println("     SERVO_H_CENTER, SERVO_V_CENTER");
    Serial.println();
    Serial.println("  4. Reflash and test with 'HOME' command");
    Serial.println();
    return;
  }

  Serial.printf("[CMD] Unknown command: '%s'. Type HELP for list.\n", cmd);
}

// ============================================================================
//  LAUNCH SEQUENCE STATE MACHINE
// ============================================================================
void handleLaunchSequence() {
  unsigned long elapsed = millis() - launchStartTime;

  switch (launchPhase) {
    case 0:
      // Phase 0: Spin up motors
      Serial.println("[LAUNCH] Phase 1: Spinning up motors...");
      motors.spinUp(MOTOR_LAUNCH_SPEED);
      launchPhase = 1;
      launchStartTime = millis();
      break;

    case 1:
      // Phase 1: Hold at launch speed (ball is being launched)
      if (elapsed >= LAUNCH_HOLD_MS) {
        Serial.println("[LAUNCH] Phase 2: Launch complete, cooling down...");
        launchPhase = 2;
        launchStartTime = millis();
      }
      // Keep tracking during launch
      tracker.update();
      servos.update();
      break;

    case 2:
      // Phase 2: Cool-down — reduce speed gradually
      motors.setSpeed(0);
      if (elapsed >= LAUNCH_COOLDOWN_MS) {
        Serial.println("[LAUNCH] Sequence complete ✓");
        launchPhase = 0;

        // Return to tracking if it was enabled
        if (tracker.isEnabled()) {
          systemState = STATE_TRACKING;
        } else {
          systemState = STATE_IDLE;
        }
      }
      break;
  }
}

// ============================================================================
//  STATUS DISPLAY
// ============================================================================
void printFullStatus() {
  Serial.println();
  Serial.println("╔══════════════════════════════════════════════╗");
  Serial.println("║  SYSTEM STATUS                              ║");
  Serial.println("╚══════════════════════════════════════════════╝");

  const char* stateNames[] = {"INIT", "IDLE", "TRACKING", "LAUNCHING", "UWB_PASSTHROUGH"};
  Serial.printf("  State: %s\n", stateNames[systemState]);
  Serial.printf("  Uptime: %lu sec\n", millis() / 1000);
  Serial.println();

  // Servos
  Serial.println("  ─── Servos ───");
  Serial.printf("  Horizontal (MG996R):  %.1f° [%.0f° - %.0f°]\n",
    servos.getAngle(SERVO_HORIZONTAL),
    servos.getMinAngle(SERVO_HORIZONTAL),
    servos.getMaxAngle(SERVO_HORIZONTAL));
  Serial.printf("  Vertical   (FT5325M): %.1f° [%.0f° - %.0f°]\n",
    servos.getAngle(SERVO_VERTICAL),
    servos.getMinAngle(SERVO_VERTICAL),
    servos.getMaxAngle(SERVO_VERTICAL));
  Serial.println();

  // Motors
  Serial.println("  ─── Motors ───");
  Serial.printf("  Armed: %s\n", motors.isArmed() ? "YES" : "NO");
  Serial.printf("  Speed: %d%%\n", motors.getSpeed());
  Serial.printf("  Spinning: %s\n", motors.isSpinning() ? "YES" : "NO");
  Serial.println();

  // UWB
  uwb.printStatus();

  // Tracking
  tracker.printStatus();

  Serial.println();
}

void printHelp() {
  Serial.println();
  Serial.println("╔══════════════════════════════════════════════╗");
  Serial.println("║  AVAILABLE COMMANDS                         ║");
  Serial.println("╚══════════════════════════════════════════════╝");
  Serial.println("  SH <angle>   Set horizontal servo (0-180)");
  Serial.println("  SV <angle>   Set vertical servo (0-90)");
  Serial.println("  MS <speed>   Set motor speed (0-100%)");
  Serial.println("  MSTOP        Emergency stop motors");
  Serial.println("  TRACK        Toggle auto-tracking ON/OFF");
  Serial.println("  LAUNCH       Execute launch sequence");
  Serial.println("  HOME         Move servos to center");
  Serial.println("  UWB          Show UWB distance & angle");
  Serial.println("  UWBSETUP     Configure UWB as anchor");
  Serial.println("  UWBPASS      Direct AT command mode (EXIT to leave)");
  Serial.println("  CAL          Servo calibration guide");
  Serial.println("  STATUS       Full system status");
  Serial.println("  HELP         Show this help");
  Serial.println();
}
