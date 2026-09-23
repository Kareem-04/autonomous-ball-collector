# BallBot Android Studio Project

Android app starter for a wheeled mobile robot that detects colored balls from live camera feed and sends USB serial commands to an ESP32.

## Stack
- Kotlin
- CameraX live preview + frame analysis
- TensorFlow Lite inference
- usb-serial-for-android for USB host serial

## Features included
- Live camera preview
- Real-time frame analysis
- TFLite SSD-style detector wrapper
- Bounding box overlay
- FPS display
- USB serial connect button
- Robot control policy: LEFT / RIGHT / FORWARD / STOP / SEARCH

## Before building
1. Open the project in Android Studio.
2. Let Gradle sync.
3. Put your trained model in `app/src/main/assets/ball_detector.tflite`.
4. Make sure your labels file matches your class order.
5. Connect the phone to the ESP32 using a USB-C OTG adapter.

## ESP32 serial protocol expected
The starter app sends simple newline-terminated commands:
- `F` forward
- `L` left
- `R` right
- `S` stop
- `X` search / rotate

## Important notes
- This starter assumes an SSD-style TensorFlow Lite model.
- If you export YOLO, EfficientDet-Lite metadata models, or float-input models, adapt `TFLiteBallDetector.kt`.
- For best speed, keep the model quantized and input size at 320.
- You should later replace the broad USB filter with your real vendor/product IDs.

## Suggested next improvements
- Add color/class selection from UI.
- Add target lock so the robot follows only one chosen color.
- Add smoothing so commands do not jitter.
- Add distance estimation from bounding box area.
- Move serial sending to a foreground service.
