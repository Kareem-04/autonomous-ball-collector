package com.kareem.ballbot.detect

/**
 * Configuration for the YOLO detector.
 *
 * useGpu = true by default — the project already ships tensorflow-lite-gpu.
 * The detector tries NNAPI first (routes to NPU on Qualcomm/MediaTek devices,
 * fully supports INT8), then GPU, then falls back silently to CPU.
 *
 * inputSize: keep at 320 for YOLOv8n. Drop to 256 if you need more speed and can
 * accept slightly lower accuracy — re-export the model at that size.
 */
data class DetectorConfig(
    val modelAssetName: String = "best_int8.tflite",
    val labelsAssetName: String = "labels.txt",
    val inputSize: Int = 320,
    val scoreThreshold: Float = 0.30f,   // lowered from 0.40 — let NMS filter false positives
    val maxResults: Int = 5,
    val numThreads: Int = 4,
    val useGpu: Boolean = true            // was false — GPU/NNAPI delegate saves ~25ms/frame
)