# Stewart Platform Tracker 🤖

This project is a real-time tracking system for a **Stewart Platform** (6-DOF robotic parallel manipulator). It combines computer vision (ArUco markers) and IMU sensor data to provide stable and accurate positioning.

## 🌟 Key Features
- **Visual Tracking:** Uses ArUco `DICT_APRILTAG_16h5` for X, Y, Z, and Yaw tracking.
- **Sensor Fusion:** Merges Camera Yaw with IMU Roll and Pitch for 6-DOF orientation.
- **Advanced Validation:** Implements a secondary ArUco tag (Tag 3) for distance verification.
- **Smoothing:** Includes a `PositionSmoother` class (Alpha-Beta filter) to eliminate camera jitter.
- **Live CSV Logging:** Automatically saves data to the `csv_outputs` folder.

## 🛠 Hardware & Software Setup
1. **Hardware:**
   - Camera (720p or 1080p).
   - Arduino (default port: `COM4`).
   - IMU sensor connected to Arduino.

2. **Dependencies:**
   Run the following command to install the required libraries:
   ```bash
   pip install opencv-python numpy pyserial
   Key,Action
Shift + C,Recalibrate: Homes the robot and zeros the coordinates.
? or /,Command Mode: Type coordinates to send to the platform.
b,Quick Buffer: Re-send the last command.
q,Quit: Safely close serial and camera streams.
