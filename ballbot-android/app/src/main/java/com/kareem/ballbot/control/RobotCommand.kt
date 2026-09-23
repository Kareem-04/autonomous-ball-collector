package com.kareem.ballbot.control

/**
 * Represents a command sent to the robot over USB serial.
 *
 * Wire format: "CMD:SPEED\n"  — e.g. "F:185\n", "SL:155\n", "S:0\n", "X:0\n"
 * Speed is 0–255 (PWM). Fixed-speed commands (STOP, SEARCH) always send 0.
 *
 * Replaces the old enum so wireValue can carry a dynamic speed per frame,
 * while keeping the same .wireValue and .name API that MainActivity uses.
 */
sealed class RobotCommand {
    abstract val wireValue: String
    abstract val name: String

    data class Forward    (val speed: Int) : RobotCommand() {
        override val wireValue = "F:$speed\n"
        override val name      = "FORWARD"
    }
    data class Left       (val speed: Int) : RobotCommand() {
        override val wireValue = "L:$speed\n"
        override val name      = "LEFT"
    }
    data class Right      (val speed: Int) : RobotCommand() {
        override val wireValue = "R:$speed\n"
        override val name      = "RIGHT"
    }
    data class StrafeLeft (val speed: Int) : RobotCommand() {
        override val wireValue = "SL:$speed\n"
        override val name      = "STRAFE_LEFT"
    }
    data class StrafeRight(val speed: Int) : RobotCommand() {
        override val wireValue = "SR:$speed\n"
        override val name      = "STRAFE_RIGHT"
    }
    object Stop   : RobotCommand() {
        override val wireValue = "S:0\n"
        override val name      = "STOP"
    }
    object Search : RobotCommand() {
        override val wireValue = "X:0\n"
        override val name      = "SEARCH"
    }
}