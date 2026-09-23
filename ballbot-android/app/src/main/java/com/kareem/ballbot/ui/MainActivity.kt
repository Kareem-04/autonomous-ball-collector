package com.kareem.ballbot.ui

import android.Manifest
import android.content.pm.PackageManager
import android.os.Bundle
import android.os.SystemClock
import android.util.Size
import android.view.WindowManager
import android.widget.Toast
import androidx.activity.result.contract.ActivityResultContracts
import androidx.appcompat.app.AppCompatActivity
import androidx.camera.core.CameraSelector
import androidx.camera.core.ImageAnalysis
import androidx.camera.core.Preview
import androidx.camera.core.resolutionselector.ResolutionSelector
import androidx.camera.core.resolutionselector.ResolutionStrategy
import androidx.camera.lifecycle.ProcessCameraProvider
import androidx.core.content.ContextCompat
import com.kareem.ballbot.R
import com.kareem.ballbot.control.RobotController
import com.kareem.ballbot.control.TargetColor
import com.kareem.ballbot.databinding.ActivityMainBinding
import com.kareem.ballbot.detect.Detection
import com.kareem.ballbot.detect.FramePreprocessor
import com.kareem.ballbot.detect.HsvBallDetector
import com.kareem.ballbot.detect.YOLO26Detector
import com.kareem.ballbot.serial.UsbSerialManager
import java.util.concurrent.ExecutorService
import java.util.concurrent.Executors

class MainActivity : AppCompatActivity() {

    private lateinit var binding: ActivityMainBinding
    private lateinit var cameraExecutor: ExecutorService
    private lateinit var yoloDetector: YOLO26Detector
    private lateinit var hsvDetector: HsvBallDetector
    private lateinit var usbSerialManager: UsbSerialManager
    // ── Robot steering / stopping tuning ──────────────────────────────────────
    // Adjust these values to match your physical phone-holder offset and
    // the distance at which the ball should be considered "collected".
    private val robotController = RobotController(
        horizontalBias    = 0.00f,   // phone offset: +right / −left (e.g. +0.08 if phone is 8% left of collector)
        verticalBias      = 0.00f,   // phone offset: +down  / −up   (reserved for future use)
        stopAreaThreshold = 0.18f    // ball area ratio to STOP — lower = stop farther away, higher = get closer
    )

    private var trackingEnabled = false
    private var useHsvMode = false           // toggle between YOLO and HSV
    private var selectedTargetColor = TargetColor.ANY

    // Command throttle & deduplication
    private var lastCommandSentAt = 0L
    private var lastCommandRaw = ""

    // FPS counter
    private var frameCounter = 0
    private var fpsWindowStart = 0L

    // UI update throttle — avoid flooding the main thread with invalidations
    private var lastUiUpdateAt = 0L
    private val uiUpdateIntervalMs = 50L     // update UI at most 20×/s regardless of FPS

    private val permissionLauncher = registerForActivityResult(
        ActivityResultContracts.RequestPermission()
    ) { granted ->
        if (granted) startCamera() else toast("Camera permission is required")
    }

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        binding = ActivityMainBinding.inflate(layoutInflater)
        setContentView(binding.root)
        window.addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON)

        cameraExecutor = Executors.newSingleThreadExecutor()
        yoloDetector   = YOLO26Detector(this)
        hsvDetector    = HsvBallDetector()

        usbSerialManager = UsbSerialManager(this) { status ->
            runOnUiThread { binding.statusText.text = "Status: $status" }
        }

        setupColorSelector()

        binding.usbButton.setOnClickListener { usbSerialManager.connectFirstAvailable() }

        binding.trackingButton.setOnClickListener {
            trackingEnabled = !trackingEnabled
            binding.trackingButton.text = if (trackingEnabled) "Stop Tracking" else "Start Tracking"
            if (!trackingEnabled) forceCommand("S\n", "STOP")
        }

        // HSV / YOLO mode toggle button
        // Add a Button with id=modeButton to your layout XML with text "Mode: YOLO"
        binding.modeButton.setOnClickListener {
            useHsvMode = !useHsvMode
            binding.modeButton.text = if (useHsvMode) "Mode: HSV" else "Mode: YOLO"
        }

        if (hasCameraPermission()) startCamera()
        else permissionLauncher.launch(Manifest.permission.CAMERA)
    }

    // -------------------------------------------------------------------------
    // Camera
    // -------------------------------------------------------------------------

    private fun startCamera() {
        val cameraProviderFuture = ProcessCameraProvider.getInstance(this)
        cameraProviderFuture.addListener({
            val cameraProvider = cameraProviderFuture.get()

            val preview = Preview.Builder().build().also {
                it.surfaceProvider = binding.previewView.surfaceProvider
            }

            // CRITICAL CHANGE: pin the analysis resolution to 640×480.
            // Without this, CameraX defaults to the highest preview resolution
            // (often 1080p or higher on modern phones), which means the
            // FramePreprocessor converts a 2MP image every frame before
            // scaling it down to 320×320 for the model — pure wasted work.
            // At 640×480 the conversion bitmap is ~6× smaller.
            val resolutionSelector = ResolutionSelector.Builder()
                .setResolutionStrategy(
                    ResolutionStrategy(
                        Size(320, 320),
                        ResolutionStrategy.FALLBACK_RULE_CLOSEST_LOWER_THEN_HIGHER
                    )
                )
                .build()

            val analysis = ImageAnalysis.Builder()
                .setResolutionSelector(resolutionSelector)
                .setBackpressureStrategy(ImageAnalysis.STRATEGY_KEEP_ONLY_LATEST)
                .build()
                .also {
                    it.setAnalyzer(cameraExecutor) { imageProxy ->
                        try {
                            val bitmap = FramePreprocessor.imageProxyToBitmap(imageProxy)

                            val detections = if (useHsvMode) {
                                hsvDetector.detect(bitmap, selectedTargetColor)
                            } else {
                                yoloDetector.detect(bitmap)
                            }

                            val best = pickBestTarget(detections)
                            updateFps()

                            val now = SystemClock.elapsedRealtime()
                            if (now - lastUiUpdateAt >= uiUpdateIntervalMs) {
                                lastUiUpdateAt = now
                                val overlayW = bitmap.width
                                val overlayH = bitmap.height
                                runOnUiThread {
                                    binding.overlayView.setResults(detections, overlayW, overlayH)
                                    binding.targetText.text = best?.let {
                                        "Target: ${it.label} ${(it.score * 100).toInt()}%"
                                    } ?: "Target: none"
                                }
                            }

                            if (trackingEnabled) {
                                val command = robotController.decide(best, bitmap.width, bitmap.height)
                                sendCommand(command.wireValue, command.name)
                            }

                            bitmap.recycle()
                        } catch (e: Exception) {
                            runOnUiThread { binding.statusText.text = "Error: ${e.message}" }
                        } finally {
                            imageProxy.close()
                        }
                    }
                }

            cameraProvider.unbindAll()
            cameraProvider.bindToLifecycle(
                this, CameraSelector.DEFAULT_BACK_CAMERA, preview, analysis
            )
        }, ContextCompat.getMainExecutor(this))
    }

    // -------------------------------------------------------------------------
    // Target selection
    // -------------------------------------------------------------------------

    private fun pickBestTarget(detections: List<Detection>): Detection? {
        val filtered = detections.filter { selectedTargetColor.matches(it.label) }
        if (filtered.isNotEmpty()) return filtered.maxByOrNull { it.score }
        if (selectedTargetColor == TargetColor.ANY) {
            return detections.firstOrNull { it.label.contains("ball", ignoreCase = true) }
                ?: detections.maxByOrNull { it.score }
        }
        return null
    }

    // -------------------------------------------------------------------------
    // USB command sending
    // -------------------------------------------------------------------------

    /**
     * Send only when:
     *  (a) the command changed (immediate send), OR
     *  (b) same command but 80ms heartbeat has elapsed (prevents ESP from losing state).
     * This avoids flooding the USB buffer when the robot is doing the same thing
     * across many consecutive frames.
     */
    private fun sendCommand(raw: String, visible: String) {
        val now = SystemClock.elapsedRealtime()
        val changed = raw != lastCommandRaw
        if (!changed && now - lastCommandSentAt < 80) return
        lastCommandSentAt = now
        lastCommandRaw = raw
        runOnUiThread { binding.commandText.text = "Command: $visible" }
        usbSerialManager.send(raw)
    }

    /** Send a command unconditionally (used for explicit STOP). */
    private fun forceCommand(raw: String, visible: String) {
        lastCommandRaw = raw
        lastCommandSentAt = SystemClock.elapsedRealtime()
        runOnUiThread { binding.commandText.text = "Command: $visible" }
        usbSerialManager.send(raw)
    }

    // -------------------------------------------------------------------------
    // FPS counter
    // -------------------------------------------------------------------------

    private fun updateFps() {
        val now = SystemClock.elapsedRealtime()
        if (fpsWindowStart == 0L) fpsWindowStart = now
        frameCounter++
        val elapsed = now - fpsWindowStart
        if (elapsed >= 1000) {
            val fps = frameCounter * 1000f / elapsed
            runOnUiThread { binding.fpsText.text = "FPS: ${String.format("%.1f", fps)}" }
            frameCounter = 0
            fpsWindowStart = now
        }
    }

    // -------------------------------------------------------------------------
    // UI helpers
    // -------------------------------------------------------------------------

    private fun setupColorSelector() {
        binding.selectedColorText.text = "Target color: ${selectedTargetColor.displayName}"
        binding.colorChipGroup.setOnCheckedStateChangeListener { _, checkedIds ->
            selectedTargetColor = when (checkedIds.firstOrNull()) {
                R.id.chipRed    -> TargetColor.RED
                R.id.chipBlue   -> TargetColor.BLUE
                R.id.chipYellow -> TargetColor.YELLOW
                R.id.chipGreen  -> TargetColor.GREEN
                else            -> TargetColor.ANY
            }
            binding.selectedColorText.text = "Target color: ${selectedTargetColor.displayName}"
        }
    }

    private fun hasCameraPermission() =
        ContextCompat.checkSelfPermission(this, Manifest.permission.CAMERA) ==
                PackageManager.PERMISSION_GRANTED

    private fun toast(msg: String) = Toast.makeText(this, msg, Toast.LENGTH_SHORT).show()

    override fun onDestroy() {
        super.onDestroy()
        yoloDetector.close()
        usbSerialManager.close()
        cameraExecutor.shutdown()
    }
}