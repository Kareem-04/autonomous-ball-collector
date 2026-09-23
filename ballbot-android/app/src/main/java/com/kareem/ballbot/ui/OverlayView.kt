package com.kareem.ballbot.ui

import android.content.Context
import android.graphics.Canvas
import android.graphics.Color
import android.graphics.Paint
import android.util.AttributeSet
import android.view.View
import com.kareem.ballbot.detect.Detection

class OverlayView @JvmOverloads constructor(
    context: Context,
    attrs: AttributeSet? = null
) : View(context, attrs) {

    private val boxPaint = Paint().apply {
        color = Color.GREEN
        style = Paint.Style.STROKE
        strokeWidth = 8f
    }

    private val textBgPaint = Paint().apply {
        color = Color.argb(160, 0, 0, 0)
        style = Paint.Style.FILL
    }

    private val textPaint = Paint().apply {
        color = Color.WHITE
        textSize = 40f
        style = Paint.Style.FILL
    }

    private var detections: List<Detection> = emptyList()
    private var imageWidth: Int = 0
    private var imageHeight: Int = 0

    fun setResults(results: List<Detection>, imageWidth: Int, imageHeight: Int) {
        detections = results
        this.imageWidth = imageWidth
        this.imageHeight = imageHeight
        postInvalidate()
    }

    override fun onDraw(canvas: Canvas) {
        super.onDraw(canvas)
        if (imageWidth == 0 || imageHeight == 0) return

        // Match PreviewView's FILL_CENTER scaling: uniform scale to fill,
        // center-crop any excess.  This keeps bounding boxes aligned with
        // the live camera feed regardless of aspect-ratio differences.
        val scale = maxOf(
            width.toFloat() / imageWidth,
            height.toFloat() / imageHeight
        )
        val offsetX = (width  - imageWidth  * scale) / 2f
        val offsetY = (height - imageHeight * scale) / 2f

        detections.forEach { detection ->
            val left   = detection.boundingBox.left   * scale + offsetX
            val top    = detection.boundingBox.top    * scale + offsetY
            val right  = detection.boundingBox.right  * scale + offsetX
            val bottom = detection.boundingBox.bottom * scale + offsetY
            canvas.drawRect(left, top, right, bottom, boxPaint)
            val label = "${detection.label} ${(detection.score * 100).toInt()}%"
            val textWidth = textPaint.measureText(label)
            canvas.drawRect(left, top - 52f, left + textWidth + 24f, top, textBgPaint)
            canvas.drawText(label, left + 12f, top - 14f, textPaint)
        }
    }
}
