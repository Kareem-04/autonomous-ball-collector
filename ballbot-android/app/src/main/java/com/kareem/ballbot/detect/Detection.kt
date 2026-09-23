package com.kareem.ballbot.detect

import android.graphics.RectF

data class Detection(
    val label: String,
    val score: Float,
    val boundingBox: RectF
)
