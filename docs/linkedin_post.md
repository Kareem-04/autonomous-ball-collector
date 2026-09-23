# LinkedIn Post — Autonomous Ball Collector

> Copy the text below into your LinkedIn post editor. Add your photos/video at the top for maximum engagement.

---

🤖 Built an autonomous ball-collecting and launching robot — here's how it works.

The robot detects colored balls using computer vision, drives toward them with mecanum wheels, collects them, then launches them at a target tracked by UWB positioning. No remote control. Fully autonomous.

Here's what's under the hood:

🔍 𝗩𝗶𝘀𝗶𝗼𝗻 — An Android phone mounted on the robot runs a YOLOv8 model (INT8 quantized, 320×320) for real-time ball detection. It identifies 6 ball colors and sends driving commands to the ESP32 over USB serial. When the ball is near the center of the frame, the robot slides laterally using its mecanum wheels instead of rotating — because mecanum wheels can do that.

🛞 𝗗𝗿𝗶𝘃𝗶𝗻𝗴 — 4 mecanum wheels give the robot full omnidirectional movement: forward, backward, strafe, diagonal, and rotation. When no ball is detected, the robot enters a "search dance" that showcases all 10 mecanum motion types while 2 ultrasonic sensors prevent collisions.

📡 𝗧𝗮𝗿𝗴𝗲𝘁 𝗧𝗿𝗮𝗰𝗸𝗶𝗻𝗴 — This was the hardest engineering challenge. The UWB antenna is mounted on the launcher itself, so every time the servo moves to aim, the UWB angle reading changes — creating a feedback loop that makes PID control oscillate violently. I developed a custom "step-and-wait" algorithm: move 5°, discard 80ms of stale readings, collect fresh samples, evaluate, repeat. It breaks the feedback loop and achieves stable tracking despite ±30° sensor noise.

🚀 𝗟𝗮𝘂𝗻𝗰𝗵𝗶𝗻𝗴 — A dual-flywheel launcher fires the collected ball toward the target. Motor speed automatically adjusts based on distance, and the vertical angle includes ballistic gravity compensation (loft increases with distance squared). The horizontal aim goes through a 4:1 bevel gear that required inverted servo logic — small details that matter in mechatronics.

𝗧𝗲𝗰𝗵 𝘀𝘁𝗮𝗰𝗸:
• Android (Kotlin) + CameraX + TensorFlow Lite
• 2× ESP32 (PlatformIO/C++)
• YOLOv8n (custom-trained, INT8 quantized)
• UWB positioning (BU04 G451 PDoA)
• L298N motor drivers + mecanum wheels
• Brushless motors + ESCs
• HC-SR04 ultrasonic sensors

This was a university course project that pushed me into control theory, embedded systems, computer vision, and mechanical design all at once. The code is open source — link in comments.

<!-- Add your project photos/video above the text for maximum engagement -->
<!-- Add the GitHub repo link as the first comment, not in the post body -->

---

### Posting Tips

1. **Upload 3–5 photos** (or a short video) AT THE TOP — posts with media get 3× more reach
2. **Post the GitHub link as the FIRST COMMENT**, not in the post body (LinkedIn suppresses external links in the body)
3. **Tag relevant hashtags** in a comment, not in the post: `#Robotics #ComputerVision #ESP32 #Android #MachineLearning #MecanumWheels #UWB #Engineering #OpenSource`
4. **Best posting time**: Tuesday–Thursday, 8–10 AM your timezone
5. **Engage with comments** in the first hour — the algorithm boosts posts with early engagement
