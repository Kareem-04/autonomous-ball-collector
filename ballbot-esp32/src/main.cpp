// ═══════════════════════════════════════════════════════════════════════════════
//  BallBot ESP32 — Mecanum Wheel Controller
//  Platform : ESP32 DevKit V1 (PlatformIO + Arduino Framework)
//  Drivers  : 2× L298N  (Left side motors + Right side motors)
//  Wheels   : 4× Mecanum 45° (standard X-configuration)
//  Sensors  : 2× HC-SR04 Ultrasonic (front corners)
//  Comms    : USB Serial at 115200 baud (commands from Android app)
// ═══════════════════════════════════════════════════════════════════════════════
//
//  OPERATING MODES
//  ─────────────────────────────────────────────────────────────────────────────
//  IDLE      : Motors off. Waiting for commands.
//  TRACKING  : Following phone commands (F/L/R/SL/SR). Ultrasonics IGNORED.
//  SEARCH    : Dance routine showcasing mecanum moves. Ultrasonics ACTIVE
//              for collision avoidance. Interrupted immediately by any
//              tracking command from the phone.
//
//  SERIAL PROTOCOL  (from Android app)
//  ─────────────────────────────────────────────────────────────────────────────
//    Format  : "CMD:SPEED\n"
//    CMD     : F | L | R | SL | SR | S | X
//    SPEED   : 0–255 PWM value (ignored for S and X)
//    Examples: "F:200\n"  "SL:155\n"  "R:130\n"  "S:0\n"  "X:0\n"
//
//  WIRING
//  ─────────────────────────────────────────────────────────────────────────────
//  L298N #1 — LEFT side (Front-Left + Rear-Left)
//  ┌──────────────┬────────────┐
//  │ L298N Pin    │ ESP32 GPIO │
//  ├──────────────┼────────────┤
//  │ ENA (FL spd) │ GPIO 23    │
//  │ IN1 (FL dir) │ GPIO 27    │
//  │ IN2 (FL dir) │ GPIO 26    │
//  │ ENB (RL spd) │ GPIO 14    │
//  │ IN3 (RL dir) │ GPIO 25    │
//  │ IN4 (RL dir) │ GPIO 33    │
//  └──────────────┴────────────┘
//
//  L298N #2 — RIGHT side (Front-Right + Rear-Right)
//  ┌──────────────┬────────────┐
//  │ L298N Pin    │ ESP32 GPIO │
//  ├──────────────┼────────────┤
//  │ ENA (FR spd) │ GPIO 12    │
//  │ IN1 (FR dir) │ GPIO 13    │
//  │ IN2 (FR dir) │ GPIO 15    │
//  │ ENB (RR spd) │ GPIO 2     │
//  │ IN3 (RR dir) │ GPIO 0     │
//  │ IN4 (RR dir) │ GPIO 4     │
//  └──────────────┴────────────┘
//
//  Ultrasonic Sensors (HC-SR04 × 2)
//  ┌──────────────────┬──────┬──────┐
//  │ Sensor           │ TRIG │ ECHO │
//  ├──────────────────┼──────┼──────┤
//  │ Left Front       │ G5   │ G18  │
//  │ Right Front      │ G19  │ G21  │
//  └──────────────────┴──────┴──────┘
//
//  IR Sensor: GPIO 22 (reserved — not used in this version)
//
//  MECANUM WHEEL LAYOUT  (top view, arrows show roller orientation)
//  ─────────────────────────────────────────────────────────────────────────────
//        FL (rollers: ╲)            FR (rollers: ╱)
//          ╲                          ╱
//           ╔══════════════════════╗
//           ║       ROBOT         ║
//           ║     (front →)       ║
//           ╚══════════════════════╝
//          ╱                          ╲
//        RL (rollers: ╱)            RR (rollers: ╲)
//
//  MECANUM INVERSE KINEMATICS
//  ─────────────────────────────────────────────────────────────────────────────
//    FL = Vy + Vx + ω       FR = Vy − Vx − ω
//    RL = Vy − Vx + ω       RR = Vy + Vx − ω
//
//    Where: Vy = forward (+),  Vx = strafe right (+),  ω = spin CW (+)
//
//  ┌──────────────────┬─────┬─────┬─────┬─────┐
//  │ Motion           │  FL │  FR │  RL │  RR │
//  ├──────────────────┼─────┼─────┼─────┼─────┤
//  │ Forward          │  ↑  │  ↑  │  ↑  │  ↑  │
//  │ Backward         │  ↓  │  ↓  │  ↓  │  ↓  │
//  │ Strafe Left      │  ↓  │  ↑  │  ↑  │  ↓  │
//  │ Strafe Right     │  ↑  │  ↓  │  ↓  │  ↑  │
//  │ Rotate CW        │  ↑  │  ↓  │  ↑  │  ↓  │
//  │ Rotate CCW       │  ↓  │  ↑  │  ↓  │  ↑  │
//  │ Diag Fwd-Left    │  0  │  ↑  │  ↑  │  0  │
//  │ Diag Fwd-Right   │  ↑  │  0  │  0  │  ↑  │
//  │ Diag Back-Left   │  ↓  │  0  │  0  │  ↓  │
//  │ Diag Back-Right  │  0  │  ↓  │  ↓  │  0  │
//  └──────────────────┴─────┴─────┴─────┴─────┘
//
//  NOTE: If a wheel spins the wrong direction after first test,
//        swap IN1↔IN2 (or IN3↔IN4) for that motor in the pin
//        definitions below. Do NOT re-wire the motor leads.
// ═══════════════════════════════════════════════════════════════════════════════

#include <Arduino.h>

// ─── Pin Definitions ─────────────────────────────────────────────────────────

// Left L298N — Front-Left (FL) motor
static const uint8_t FL_EN  = 23;
static const uint8_t FL_IN1 = 25;
static const uint8_t FL_IN2 = 33;

// Left L298N — Rear-Left (RL) motor
static const uint8_t RL_EN  = 14;
static const uint8_t RL_IN1 = 26;
static const uint8_t RL_IN2 = 27;

// Right L298N — Front-Right (FR) motor
static const uint8_t FR_EN  = 12;
static const uint8_t FR_IN1 = 4;
static const uint8_t FR_IN2 = 0;

// Right L298N — Rear-Right (RR) motor
static const uint8_t RR_EN  =  2;
static const uint8_t RR_IN1 =  15;
static const uint8_t RR_IN2 =  13;

// Ultrasonic sensors (HC-SR04)
// ► Swap LEFT ↔ RIGHT here if the sensors are physically reversed
static uint8_t US_LEFT_TRIG  =  5;
static uint8_t US_LEFT_ECHO  = 18;
static uint8_t US_RIGHT_TRIG = 19;
static uint8_t US_RIGHT_ECHO = 21;

// IR sensor — reserved for future use
static const uint8_t IR_PIN = 22;

// ─── Configuration Constants ─────────────────────────────────────────────────

static const uint32_t PWM_FREQ         = 1000;    // Hz
static const uint8_t  PWM_RESOLUTION   = 8;       // 8-bit → 0–255
static const uint32_t SERIAL_BAUD      = 115200;

static const uint16_t CMD_TIMEOUT_MS   = 500;     // Auto-stop if no tracking cmd
static const float    OBSTACLE_DIST_CM = 10.0f;   // Ultrasonic avoidance threshold
static const uint32_t US_TIMEOUT_US    = 6000;    // Pulse timeout (~1 m max range)
static const uint8_t  SEARCH_SPEED     = 170;     // PWM for dance moves
static const uint8_t  AVOIDANCE_SPEED  = 200;     // PWM for obstacle dodging
static const uint16_t US_READ_INTERVAL = 60;      // Read sensors every 60 ms

// LEDC channels (legacy API — Core 2.x)
static const uint8_t CH_FL = 0;
static const uint8_t CH_RL = 1;
static const uint8_t CH_FR = 2;
static const uint8_t CH_RR = 3;

// ─── Robot State ─────────────────────────────────────────────────────────────

enum RobotMode : uint8_t {
    MODE_IDLE,
    MODE_TRACKING,
    MODE_SEARCH
};

RobotMode     currentMode   = MODE_IDLE;
uint8_t       searchStep    = 0;
unsigned long stepStartMs   = 0;
unsigned long lastCmdMs     = 0;
unsigned long lastUsReadMs  = 0;
float         distLeftCm    = 999.0f;
float         distRightCm   = 999.0f;
bool          avoiding      = false;

// ─── Dance Sequence ──────────────────────────────────────────────────────────
// Showcases all ten mecanum movement types while covering area to search.

enum MecanumMove : uint8_t {
    MOV_FORWARD = 0,
    MOV_BACKWARD,
    MOV_STRAFE_LEFT,
    MOV_STRAFE_RIGHT,
    MOV_DIAG_FORWARD_LEFT,
    MOV_DIAG_FORWARD_RIGHT,
    MOV_DIAG_BACK_LEFT,
    MOV_DIAG_BACK_RIGHT,
    MOV_SPIN_CW,
    MOV_SPIN_CCW
};

struct DanceStep {
    MecanumMove move;
    uint16_t    durationMs;
};

static const DanceStep DANCE[] = {
    // ── Phase 1: Spin-scan + lateral slides ──────────────────────────────────
    { MOV_SPIN_CW,            800 },    // Scan right
    { MOV_STRAFE_RIGHT,       500 },    // Slide right
    { MOV_SPIN_CCW,           800 },    // Scan left
    { MOV_STRAFE_LEFT,        500 },    // Slide left

    // ── Phase 2: Diamond pattern (all four diagonals) ────────────────────────
    { MOV_DIAG_FORWARD_RIGHT, 500 },    // ↗
    { MOV_DIAG_FORWARD_LEFT,  500 },    // ↖
    { MOV_DIAG_BACK_LEFT,     500 },    // ↙
    { MOV_DIAG_BACK_RIGHT,    500 },    // ↘

    // ── Phase 3: Zigzag advance ──────────────────────────────────────────────
    { MOV_FORWARD,            500 },    // Push forward
    { MOV_SPIN_CW,            400 },    // Quick turn right
    { MOV_FORWARD,            500 },    // Push forward
    { MOV_SPIN_CCW,           400 },    // Quick turn left

    // ── Phase 4: Lateral sweep ───────────────────────────────────────────────
    { MOV_STRAFE_LEFT,        700 },    // Long strafe left
    { MOV_FORWARD,            300 },    // Step forward
    { MOV_STRAFE_RIGHT,       700 },    // Long strafe right
    { MOV_FORWARD,            300 },    // Step forward

    // ── Phase 5: Retreat + full 360° scan ────────────────────────────────────
    { MOV_BACKWARD,           400 },    // Back up
    { MOV_SPIN_CW,           1200 },    // Full rotation scan
};
static const uint8_t DANCE_LEN = sizeof(DANCE) / sizeof(DANCE[0]);

// ─── Function Declarations ──────────────────────────────────────────────────

void setupMotorPins();
void setupPWM();
void setupUltrasonic();

void processSerial();
void handleSearch();
void checkCommandTimeout();

void moveForward(uint8_t spd);
void moveBackward(uint8_t spd);
void strafeLeft(uint8_t spd);
void strafeRight(uint8_t spd);
void rotateCW(uint8_t spd);
void rotateCCW(uint8_t spd);
void diagForwardLeft(uint8_t spd);
void diagForwardRight(uint8_t spd);
void diagBackLeft(uint8_t spd);
void diagBackRight(uint8_t spd);
void stopAll();
void executeMecanumMove(MecanumMove move, uint8_t spd);

void setMotor(uint8_t enPin, uint8_t enCh,
              uint8_t in1, uint8_t in2,
              uint8_t spd, bool forward);
void pwmWrite(uint8_t pin, uint8_t channel, uint8_t duty);

float readUltrasonicCm(uint8_t trigPin, uint8_t echoPin);

// ═════════════════════════════════════════════════════════════════════════════
//  SETUP
// ═════════════════════════════════════════════════════════════════════════════

void setup() {
    Serial.begin(SERIAL_BAUD);

    setupMotorPins();
    setupPWM();
    setupUltrasonic();

    // IR sensor — input, reserved for future use
    pinMode(IR_PIN, INPUT);

    stopAll();

    Serial.println(F("═══════════════════════════════════════════"));
    Serial.println(F("  BallBot ESP32 — Mecanum Controller"));
    Serial.println(F("  Protocol: CMD:SPEED  (F/L/R/SL/SR/S/X)"));
    Serial.println(F("═══════════════════════════════════════════"));

    lastCmdMs = millis();
}

// ═════════════════════════════════════════════════════════════════════════════
//  MAIN LOOP
// ═════════════════════════════════════════════════════════════════════════════

void loop() {
    // ① Always check serial — tracking commands override everything instantly
    processSerial();

    // ② Run search dance with ultrasonic collision avoidance
    if (currentMode == MODE_SEARCH) {
        handleSearch();
    }

    // ③ Safety: auto-stop tracking if no command received in time
    //    Search mode is NOT timed out — it runs until the phone sends
    //    a tracking command or explicit stop.
    checkCommandTimeout();
}

// ═════════════════════════════════════════════════════════════════════════════
//  SERIAL COMMAND PARSER
// ═════════════════════════════════════════════════════════════════════════════

void processSerial() {
    if (!Serial.available()) return;

    String line = Serial.readStringUntil('\n');
    line.trim();
    if (line.length() == 0) return;

    // Parse "CMD:SPEED" — split on ':'
    int colonIdx = line.indexOf(':');
    String cmd   = (colonIdx >= 0) ? line.substring(0, colonIdx)  : line;
    int    spd   = (colonIdx >= 0) ? line.substring(colonIdx + 1).toInt() : 0;
    spd = constrain(spd, 0, 255);

    lastCmdMs = millis();

    // ── Tracking commands — immediately exit search, ignore ultrasonics ──────
    if (cmd == "F") {
        currentMode = MODE_TRACKING;
        avoiding = false;
        moveForward((uint8_t)spd);
    }
    else if (cmd == "L") {
        currentMode = MODE_TRACKING;
        avoiding = false;
        rotateCCW((uint8_t)spd);          // L = rotate left = counter-clockwise
    }
    else if (cmd == "R") {
        currentMode = MODE_TRACKING;
        avoiding = false;
        rotateCW((uint8_t)spd);           // R = rotate right = clockwise
    }
    else if (cmd == "SL") {
        currentMode = MODE_TRACKING;
        avoiding = false;
        strafeLeft((uint8_t)spd);         // Slide left — no rotation
    }
    else if (cmd == "SR") {
        currentMode = MODE_TRACKING;
        avoiding = false;
        strafeRight((uint8_t)spd);        // Slide right — no rotation
    }
    // ── Stop ─────────────────────────────────────────────────────────────────
    else if (cmd == "S") {
        currentMode = MODE_IDLE;
        avoiding = false;
        stopAll();
    }
    // ── Search ───────────────────────────────────────────────────────────────
    else if (cmd == "X") {
        currentMode = MODE_SEARCH;
        avoiding    = false;
        searchStep  = 0;
        stepStartMs = millis();
        executeMecanumMove(DANCE[0].move, SEARCH_SPEED);
        Serial.println(F(">> Search mode activated"));
    }
}

// ═════════════════════════════════════════════════════════════════════════════
//  COMMAND TIMEOUT — Safety auto-stop (tracking mode only)
// ═════════════════════════════════════════════════════════════════════════════
// If the phone disconnects mid-tracking, the robot stops after CMD_TIMEOUT_MS.
// Search mode is intentionally excluded — it self-manages via the dance loop.

void checkCommandTimeout() {
    if (currentMode != MODE_TRACKING) return;
    if (millis() - lastCmdMs >= CMD_TIMEOUT_MS) {
        currentMode = MODE_IDLE;
        stopAll();
        Serial.println(F(">> Tracking timeout — stopped"));
    }
}

// ═════════════════════════════════════════════════════════════════════════════
//  SEARCH DANCE + ULTRASONIC COLLISION AVOIDANCE
// ═════════════════════════════════════════════════════════════════════════════
// The dance cycles through DANCE[] showcasing all mecanum capabilities.
// Every US_READ_INTERVAL ms, both ultrasonic sensors are checked:
//   • Both blocked  → reverse
//   • Left blocked  → strafe right (dodge)
//   • Right blocked → strafe left  (dodge)
// When the obstacle clears, the current dance step resumes.

void handleSearch() {
    unsigned long now = millis();

    // ── Periodic ultrasonic check ────────────────────────────────────────────
    if (now - lastUsReadMs >= US_READ_INTERVAL) {
        lastUsReadMs = now;
        distLeftCm   = readUltrasonicCm(US_LEFT_TRIG, US_LEFT_ECHO);
        distRightCm  = readUltrasonicCm(US_RIGHT_TRIG, US_RIGHT_ECHO);

        bool leftBlocked  = (distLeftCm  < OBSTACLE_DIST_CM);
        bool rightBlocked = (distRightCm < OBSTACLE_DIST_CM);

        if (leftBlocked || rightBlocked) {
            // Obstacle detected — override current move
            if (leftBlocked && rightBlocked) {
                moveBackward(AVOIDANCE_SPEED);      // Both: reverse
            } else if (leftBlocked) {
                strafeRight(AVOIDANCE_SPEED);       // Left blocked: slide right
            } else {
                strafeLeft(AVOIDANCE_SPEED);        // Right blocked: slide left
            }

            if (!avoiding) {
                avoiding = true;
                Serial.println(F(">> Obstacle detected — avoiding"));
            }
            stepStartMs = now;   // Hold dance timer while avoiding
            return;
        }

        // Obstacle cleared — resume the dance
        if (avoiding) {
            avoiding = false;
            executeMecanumMove(DANCE[searchStep].move, SEARCH_SPEED);
            stepStartMs = now;
            Serial.println(F(">> Obstacle cleared — resuming dance"));
        }
    }

    // ── Dance step transitions (paused during avoidance) ─────────────────────
    if (!avoiding && (now - stepStartMs >= DANCE[searchStep].durationMs)) {
        searchStep = (searchStep + 1) % DANCE_LEN;
        executeMecanumMove(DANCE[searchStep].move, SEARCH_SPEED);
        stepStartMs = now;
    }
}

// ═════════════════════════════════════════════════════════════════════════════
//  MECANUM MOTION PRIMITIVES
// ═════════════════════════════════════════════════════════════════════════════
//
//  Inverse kinematics for standard X-configuration mecanum wheels:
//    FL = Vy + Vx + ω       FR = Vy − Vx − ω
//    RL = Vy − Vx + ω       RR = Vy + Vx − ω

void moveForward(uint8_t spd) {
    // Vy = +spd → all wheels forward
    setMotor(FL_EN, CH_FL, FL_IN1, FL_IN2, spd, true);
    setMotor(FR_EN, CH_FR, FR_IN1, FR_IN2, spd, true);
    setMotor(RL_EN, CH_RL, RL_IN1, RL_IN2, spd, true);
    setMotor(RR_EN, CH_RR, RR_IN1, RR_IN2, spd, true);
}

void moveBackward(uint8_t spd) {
    // Vy = −spd → all wheels backward
    setMotor(FL_EN, CH_FL, FL_IN1, FL_IN2, spd, false);
    setMotor(FR_EN, CH_FR, FR_IN1, FR_IN2, spd, false);
    setMotor(RL_EN, CH_RL, RL_IN1, RL_IN2, spd, false);
    setMotor(RR_EN, CH_RR, RR_IN1, RR_IN2, spd, false);
}

void strafeLeft(uint8_t spd) {
    // Vx = −spd → FL↓  FR↑  RL↑  RR↓
    setMotor(FL_EN, CH_FL, FL_IN1, FL_IN2, spd, false);
    setMotor(FR_EN, CH_FR, FR_IN1, FR_IN2, spd, true);
    setMotor(RL_EN, CH_RL, RL_IN1, RL_IN2, spd, true);
    setMotor(RR_EN, CH_RR, RR_IN1, RR_IN2, spd, false);
}

void strafeRight(uint8_t spd) {
    // Vx = +spd → FL↑  FR↓  RL↓  RR↑
    setMotor(FL_EN, CH_FL, FL_IN1, FL_IN2, spd, true);
    setMotor(FR_EN, CH_FR, FR_IN1, FR_IN2, spd, false);
    setMotor(RL_EN, CH_RL, RL_IN1, RL_IN2, spd, false);
    setMotor(RR_EN, CH_RR, RR_IN1, RR_IN2, spd, true);
}

void rotateCW(uint8_t spd) {
    // ω = +spd → FL↑  FR↓  RL↑  RR↓
    setMotor(FL_EN, CH_FL, FL_IN1, FL_IN2, spd, true);
    setMotor(FR_EN, CH_FR, FR_IN1, FR_IN2, spd, false);
    setMotor(RL_EN, CH_RL, RL_IN1, RL_IN2, spd, true);
    setMotor(RR_EN, CH_RR, RR_IN1, RR_IN2, spd, false);
}

void rotateCCW(uint8_t spd) {
    // ω = −spd → FL↓  FR↑  RL↓  RR↑
    setMotor(FL_EN, CH_FL, FL_IN1, FL_IN2, spd, false);
    setMotor(FR_EN, CH_FR, FR_IN1, FR_IN2, spd, true);
    setMotor(RL_EN, CH_RL, RL_IN1, RL_IN2, spd, false);
    setMotor(RR_EN, CH_RR, RR_IN1, RR_IN2, spd, true);
}

void diagForwardLeft(uint8_t spd) {
    // Vy + (−Vx) → FL = 0,  FR = 2Vy,  RL = 2Vy,  RR = 0
    setMotor(FL_EN, CH_FL, FL_IN1, FL_IN2, 0,   true);
    setMotor(FR_EN, CH_FR, FR_IN1, FR_IN2, spd, true);
    setMotor(RL_EN, CH_RL, RL_IN1, RL_IN2, spd, true);
    setMotor(RR_EN, CH_RR, RR_IN1, RR_IN2, 0,   true);
}

void diagForwardRight(uint8_t spd) {
    // Vy + (+Vx) → FL = 2Vy,  FR = 0,  RL = 0,  RR = 2Vy
    setMotor(FL_EN, CH_FL, FL_IN1, FL_IN2, spd, true);
    setMotor(FR_EN, CH_FR, FR_IN1, FR_IN2, 0,   true);
    setMotor(RL_EN, CH_RL, RL_IN1, RL_IN2, 0,   true);
    setMotor(RR_EN, CH_RR, RR_IN1, RR_IN2, spd, true);
}

void diagBackLeft(uint8_t spd) {
    // (−Vy) + (−Vx) → FL = −2Vy,  FR = 0,  RL = 0,  RR = −2Vy
    setMotor(FL_EN, CH_FL, FL_IN1, FL_IN2, spd, false);
    setMotor(FR_EN, CH_FR, FR_IN1, FR_IN2, 0,   true);
    setMotor(RL_EN, CH_RL, RL_IN1, RL_IN2, 0,   true);
    setMotor(RR_EN, CH_RR, RR_IN1, RR_IN2, spd, false);
}

void diagBackRight(uint8_t spd) {
    // (−Vy) + (+Vx) → FL = 0,  FR = −2Vy,  RL = −2Vy,  RR = 0
    setMotor(FL_EN, CH_FL, FL_IN1, FL_IN2, 0,   true);
    setMotor(FR_EN, CH_FR, FR_IN1, FR_IN2, spd, false);
    setMotor(RL_EN, CH_RL, RL_IN1, RL_IN2, spd, false);
    setMotor(RR_EN, CH_RR, RR_IN1, RR_IN2, 0,   true);
}

void stopAll() {
    setMotor(FL_EN, CH_FL, FL_IN1, FL_IN2, 0, true);
    setMotor(FR_EN, CH_FR, FR_IN1, FR_IN2, 0, true);
    setMotor(RL_EN, CH_RL, RL_IN1, RL_IN2, 0, true);
    setMotor(RR_EN, CH_RR, RR_IN1, RR_IN2, 0, true);
}

void executeMecanumMove(MecanumMove move, uint8_t spd) {
    switch (move) {
        case MOV_FORWARD:            moveForward(spd);      break;
        case MOV_BACKWARD:           moveBackward(spd);     break;
        case MOV_STRAFE_LEFT:        strafeLeft(spd);       break;
        case MOV_STRAFE_RIGHT:       strafeRight(spd);      break;
        case MOV_DIAG_FORWARD_LEFT:  diagForwardLeft(spd);  break;
        case MOV_DIAG_FORWARD_RIGHT: diagForwardRight(spd); break;
        case MOV_DIAG_BACK_LEFT:     diagBackLeft(spd);     break;
        case MOV_DIAG_BACK_RIGHT:    diagBackRight(spd);    break;
        case MOV_SPIN_CW:           rotateCW(spd);          break;
        case MOV_SPIN_CCW:          rotateCCW(spd);         break;
    }
}

// ═════════════════════════════════════════════════════════════════════════════
//  LOW-LEVEL MOTOR DRIVER
// ═════════════════════════════════════════════════════════════════════════════

void setMotor(uint8_t enPin, uint8_t enCh,
              uint8_t in1, uint8_t in2,
              uint8_t spd, bool forward) {
    if (spd == 0) {
        // Coast stop: both direction pins LOW, PWM off
        digitalWrite(in1, LOW);
        digitalWrite(in2, LOW);
        pwmWrite(enPin, enCh, 0);
    } else {
        digitalWrite(in1, forward ? HIGH : LOW);
        digitalWrite(in2, forward ? LOW  : HIGH);
        pwmWrite(enPin, enCh, spd);
    }
}

// ═════════════════════════════════════════════════════════════════════════════
//  PWM WRITE — Compatibility layer for ESP32 Arduino Core 2.x vs 3.x
// ═════════════════════════════════════════════════════════════════════════════

void pwmWrite(uint8_t pin, uint8_t channel, uint8_t duty) {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
    // Core 3.x: ledcWrite takes the GPIO pin number
    ledcWrite(pin, duty);
    (void)channel;
#else
    // Core 2.x: ledcWrite takes the LEDC channel number
    ledcWrite(channel, duty);
    (void)pin;
#endif
}

// ═════════════════════════════════════════════════════════════════════════════
//  ULTRASONIC SENSOR (HC-SR04)
// ═════════════════════════════════════════════════════════════════════════════

float readUltrasonicCm(uint8_t trigPin, uint8_t echoPin) {
    // Send 10 µs trigger pulse
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

    // Measure echo pulse width
    unsigned long duration = pulseIn(echoPin, HIGH, US_TIMEOUT_US);

    if (duration == 0) return 999.0f;   // No echo → nothing in range

    // Speed of sound ≈ 343 m/s = 0.0343 cm/µs → dist = t × 0.0343 / 2
    return duration * 0.01715f;
}

// ═════════════════════════════════════════════════════════════════════════════
//  HARDWARE SETUP HELPERS
// ═════════════════════════════════════════════════════════════════════════════

void setupMotorPins() {
    const uint8_t dirPins[] = {
        FL_IN1, FL_IN2, RL_IN1, RL_IN2,
        FR_IN1, FR_IN2, RR_IN1, RR_IN2
    };
    for (uint8_t pin : dirPins) {
        pinMode(pin, OUTPUT);
        digitalWrite(pin, LOW);
    }
}

void setupPWM() {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
    // ── ESP32 Arduino Core 3.x — pin-based LEDC API ─────────────────────────
    ledcAttach(FL_EN, PWM_FREQ, PWM_RESOLUTION);
    ledcAttach(RL_EN, PWM_FREQ, PWM_RESOLUTION);
    ledcAttach(FR_EN, PWM_FREQ, PWM_RESOLUTION);
    ledcAttach(RR_EN, PWM_FREQ, PWM_RESOLUTION);
#else
    // ── ESP32 Arduino Core 2.x — channel-based LEDC API ─────────────────────
    ledcSetup(CH_FL, PWM_FREQ, PWM_RESOLUTION);
    ledcAttachPin(FL_EN, CH_FL);
    ledcSetup(CH_RL, PWM_FREQ, PWM_RESOLUTION);
    ledcAttachPin(RL_EN, CH_RL);
    ledcSetup(CH_FR, PWM_FREQ, PWM_RESOLUTION);
    ledcAttachPin(FR_EN, CH_FR);
    ledcSetup(CH_RR, PWM_FREQ, PWM_RESOLUTION);
    ledcAttachPin(RR_EN, CH_RR);
#endif
}

void setupUltrasonic() {
    pinMode(US_LEFT_TRIG,  OUTPUT);
    pinMode(US_LEFT_ECHO,  INPUT);
    pinMode(US_RIGHT_TRIG, OUTPUT);
    pinMode(US_RIGHT_ECHO, INPUT);
}
