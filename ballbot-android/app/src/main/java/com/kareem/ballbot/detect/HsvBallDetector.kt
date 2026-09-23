package com.kareem.ballbot.detect

import android.graphics.Bitmap
import android.graphics.RectF
import com.kareem.ballbot.control.TargetColor

/**
 * HSV-based colored ball detector — no neural network, no TFLite.
 *
 * Supports: red, blue, yellow, green, pink, purple.
 *
 * Color indices used internally:
 *   0 = red_ball   1 = blue_ball   2 = yellow_ball
 *   3 = green_ball 4 = pink_ball   5 = purple_ball
 *
 * Pink and purple tuning notes:
 *   Pink is "desaturated red shifted toward magenta" — it overlaps with red
 *   in hue (300–345°) but has lower saturation (S 0.18–0.75) and high brightness.
 *   Pure red sits at S > 0.55 with hue near 0°/360°.
 *
 *   Purple overlaps with blue at the hue boundary (~265°). The split used here
 *   is: blue = 190–265°, purple = 265–305°. Both require S > 0.30.
 *
 *   If you find false positives between pink/red or blue/purple, tweak the
 *   saturation caps and hue boundaries marked with "TUNE" comments below.
 */
class HsvBallDetector {

    private val minPixelFraction = 0.003f
    private val stride = 3
    private val numColors = 6   // red, blue, yellow, green, pink, purple

    fun detect(bitmap: Bitmap, filterColor: TargetColor = TargetColor.ANY): List<Detection> {
        val w = bitmap.width
        val h = bitmap.height

        val sampledW = (w + stride - 1) / stride
        val sampledH = (h + stride - 1) / stride
        val pixels = IntArray(sampledW * sampledH)

        val sampled = Bitmap.createScaledBitmap(bitmap, sampledW, sampledH, false)
        sampled.getPixels(pixels, 0, sampledW, 0, 0, sampledW, sampledH)
        if (sampled !== bitmap) sampled.recycle()

        val minX  = IntArray(numColors) { Int.MAX_VALUE }
        val minY  = IntArray(numColors) { Int.MAX_VALUE }
        val maxX  = IntArray(numColors) { 0 }
        val maxY  = IntArray(numColors) { 0 }
        val count = IntArray(numColors)

        for (sy in 0 until sampledH) {
            for (sx in 0 until sampledW) {
                val colorIdx = classifyPixel(pixels[sy * sampledW + sx]) ?: continue
                count[colorIdx]++
                if (sx < minX[colorIdx]) minX[colorIdx] = sx
                if (sx > maxX[colorIdx]) maxX[colorIdx] = sx
                if (sy < minY[colorIdx]) minY[colorIdx] = sy
                if (sy > maxY[colorIdx]) maxY[colorIdx] = sy
            }
        }

        val totalSampled = sampledW * sampledH
        val minCount = (totalSampled * minPixelFraction).toInt().coerceAtLeast(5)
        val detections = mutableListOf<Detection>()
        val scaleX = w.toFloat() / sampledW
        val scaleY = h.toFloat() / sampledH

        for (idx in 0 until numColors) {
            if (count[idx] < minCount) continue
            val label = COLOR_LABELS[idx]
            if (filterColor != TargetColor.ANY && !filterColor.matches(label)) continue

            val boxW = (maxX[idx] - minX[idx] + 1).coerceAtLeast(1)
            val boxH = (maxY[idx] - minY[idx] + 1).coerceAtLeast(1)
            val density = count[idx].toFloat() / (boxW * boxH)
            val score = (density * 1.5f).coerceIn(0f, 1f)

            detections += Detection(
                label = label,
                score = score,
                boundingBox = RectF(
                    minX[idx] * scaleX,
                    minY[idx] * scaleY,
                    (maxX[idx] + 1) * scaleX,
                    (maxY[idx] + 1) * scaleY
                )
            )
        }

        return detections.sortedByDescending { it.score }
    }

    // -------------------------------------------------------------------------
    // Pixel classifier — returns color index 0..5 or null
    // -------------------------------------------------------------------------

    private fun classifyPixel(argb: Int): Int? {
        val r = (argb shr 16) and 0xFF
        val g = (argb shr 8)  and 0xFF
        val b =  argb         and 0xFF

        val rf = r / 255f
        val gf = g / 255f
        val bf = b / 255f

        val max   = maxOf(rf, gf, bf)
        val min   = minOf(rf, gf, bf)
        val delta = max - min

        // Reject very dark pixels and achromatic pixels.
        // Threshold lowered slightly vs original (0.18→0.15, 0.10→0.05) to catch pastel pink.
        if (max < 0.15f || delta < 0.05f) return null
        val s = delta / max          // Saturation [0, 1]
        if (s < 0.18f) return null   // TUNE: raise to reject more grey; lower to catch pastels

        // Hue in degrees [0, 360)
        val h = when {
            delta == 0f -> 0f
            max == rf   -> ((gf - bf) / delta).let { if (it < 0) it + 6f else it } * 60f
            max == gf   -> (2f + (bf - rf) / delta) * 60f
            else        -> (4f + (rf - gf) / delta) * 60f
        }

        return when {
            // ── Red ───────────────────────────────────────────────────────────
            // Hue near 0°/360°, high saturation to stay clear of pink.
            // TUNE: raise S floor (0.55) if you get pink↔red confusion.
            (h < 18f || h >= 345f) && s > 0.55f && max > 0.30f -> 0

            // ── Blue ──────────────────────────────────────────────────────────
            // Hue 190–265° — stops before purple's territory.
            h in 190f..265f && s > 0.38f -> 1

            // ── Yellow ────────────────────────────────────────────────────────
            h in 28f..68f && s > 0.38f && max > 0.50f -> 2

            // ── Green ─────────────────────────────────────────────────────────
            h in 80f..165f && s > 0.38f -> 3

            // ── Pink ──────────────────────────────────────────────────────────
            // Hue 300–345° (magenta side of red spectrum), moderate saturation, bright.
            // Upper S cap (0.75) keeps deep magenta/vivid red from bleeding in.
            // Lower S floor (0.18) allows pastel/light pinks.
            // V > 0.55 is important — pink is always a bright color.
            // TUNE: narrow hue toward 310–340 if purple bleeds in from the left.
            h in 300f..345f && s in 0.18f..0.75f && max > 0.55f -> 4

            // ── Purple ────────────────────────────────────────────────────────
            // Hue 265–305°, sitting between blue (ends at 265°) and pink (starts at 300°).
            // Intentional 5° overlap with pink at 300–305° — the saturation difference
            // between a vivid purple (S>0.75) and light pink (S<0.75) disambiguates them.
            // Dark purples accepted (V > 0.20) since purple balls can be quite dark.
            // TUNE: move lower bound toward 270 if blue bleeds in.
            h in 265f..305f && s > 0.30f && max > 0.20f -> 5

            else -> null
        }
    }

    companion object {
        // Indices MUST match the when{} block order above exactly
        val COLOR_LABELS = arrayOf(
            "red_ball", "blue_ball", "yellow_ball",
            "green_ball", "pink_ball", "purple_ball"
        )
    }
}