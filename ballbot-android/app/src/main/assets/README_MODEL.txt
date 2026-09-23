Place your trained TensorFlow Lite model in this assets folder and name it:
ball_detector.tflite

Expected format in this starter project:
- SSD-style detector with outputs:
  0 -> boxes [1, N, 4]
  1 -> classes [1, N]
  2 -> scores [1, N]
  3 -> count [1]
- Input size default: 320x320 UINT8

If your exported model uses FLOAT32 input, YOLO-style outputs, or different tensor ordering,
update TFLiteBallDetector.kt.
