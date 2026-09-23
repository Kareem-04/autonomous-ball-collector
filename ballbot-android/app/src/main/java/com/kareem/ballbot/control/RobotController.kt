package com.kareem.ballbot.control

import com.kareem.ballbot.detect.Detection
import kotlin.math.abs

/**
 * Decides the next robot command given the best detection and frame dimensions.
 *
 * ── Tuning parameters you'll want to adjust ──────────────────────────────────
 *
 * Center bias (phone holder offset):
 *   horizontalBias : shifts the "aiming center" left/right.
 *       Positive  = target point moves RIGHT in the camera frame
 *       Negative  = target point moves LEFT
 *       Example: phone is mounted 8% left of the collector → set +0.08
 *
 *   verticalBias : shifts the "aiming center" up/down.
 *       Positive  = target point moves DOWN in the camera frame
 *       Negative  = target point moves UP
 *       Example: phone is mounted above the collector → set +0.05
 *       (Currently used only for vertical error calculation; horizontal
 *        steering is the primary effect. Adjust stopAreaThreshold if
 *        vertical offset affects distance judgement.)
 *
 * Stop threshold (when to stop chasing):
 *   stopAreaThreshold : the fraction of the frame the ball's bounding box
 *       must occupy for the robot to STOP. Lower = stop farther away.
 *       Default 0.18 means the ball fills ~18% of the frame area.
 *
 * Steering zones (based on |normalizedError|, i.e. horizontal offset / frameWidth):
 *   |error| > strafeThreshold (0.25)  →  rotate  (LEFT / RIGHT)
 *   centerTolerance < |error| ≤ strafeThreshold  →  strafe (SLIDE left/right)
 *   |error| ≤ centerTolerance         →  FORWARD
 *
 * Speed zones (linear ramp, mapped to areaRatio):
 *   areaRatio ≤ farThreshold  →  FAR_SPEED  (200) — full speed
 *   areaRatio ≥ stopAreaThreshold → STOP     (0)
 *   between                   →  linearly interpolated from FAR_SPEED → NEAR_SPEED
 */
class RobotController(
    // ── Center offset (phone holder position) ───────────────────────────────
    private val horizontalBias:    Float = 0.00f,   // +right / −left (fraction of frame width)
    private val verticalBias:      Float = 0.00f,   // +down  / −up   (fraction of frame height)

    // ── Steering thresholds ─────────────────────────────────────────────────
    private val centerTolerance:   Float = 0.10f,   // dead zone around center (no steer)
    private val strafeThreshold:   Float = 0.25f,   // beyond this → rotate instead of strafe

    // ── Distance / speed thresholds ─────────────────────────────────────────
    private val stopAreaThreshold: Float = 0.18f,   // ball area ratio to STOP (lower = stop farther)
    private val farAreaThreshold:  Float = 0.04f,   // area ratio below which FAR_SPEED is used
    private val farSpeed:          Int   = 200,
    private val nearSpeed:         Int   = 110
) {

    fun decide(target: Detection?, frameWidth: Int, frameHeight: Int): RobotCommand {
        if (target == null || frameWidth == 0 || frameHeight == 0) return RobotCommand.Search

        // Horizontal error: 0.0 = target is at the aim point, negative = left, positive = right
        val centerX = (target.boundingBox.left + target.boundingBox.right) / 2f
        val normalizedError = (centerX / frameWidth) - (0.5f + horizontalBias)
        val absError = abs(normalizedError)

        // Vertical error (available for future use / logging)
        // val centerY = (target.boundingBox.top + target.boundingBox.bottom) / 2f
        // val verticalError = (centerY / frameHeight) - (0.5f + verticalBias)

        val areaRatio = (target.boundingBox.width() * target.boundingBox.height()) /
                (frameWidth.toFloat() * frameHeight.toFloat())

        // ── Stop zone ────────────────────────────────────────────────────────
        if (areaRatio >= stopAreaThreshold) return RobotCommand.Stop

        // ── Compute speed (linear ramp) ───────────────────────────────────────
        // Maps areaRatio [farAreaThreshold .. stopAreaThreshold] → speed [farSpeed .. nearSpeed]
        val speed: Int = when {
            areaRatio <= farAreaThreshold  -> farSpeed
            else -> {
                val t = (areaRatio - farAreaThreshold) /
                        (stopAreaThreshold - farAreaThreshold)          // 0.0 (far) .. 1.0 (near)
                (farSpeed + t * (nearSpeed - farSpeed)).toInt()
                    .coerceIn(nearSpeed, farSpeed)
            }
        }

        // ── Steering decision ────────────────────────────────────────────────
        return when {
            // Ball is left of center
            normalizedError < -centerTolerance -> {
                if (absError > strafeThreshold) RobotCommand.Left(speed)
                else                            RobotCommand.StrafeLeft(speed)
            }
            // Ball is right of center
            normalizedError > centerTolerance -> {
                if (absError > strafeThreshold) RobotCommand.Right(speed)
                else                            RobotCommand.StrafeRight(speed)
            }
            // Ball is centered — drive forward
            else -> RobotCommand.Forward(speed)
        }
    }
}