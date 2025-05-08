# ECE-445-Smart-Tripod

Smart Motorized Tripod
Overview
The Smart Motorized Tripod is a real-time camera tracking system designed to automate framing and subject centering. The system integrates an ESP32-S3, Raspberry Pi 4, stepper motors, and a custom PCB to control camera positioning and enable remote operation. Key technologies include OpenCV for object tracking, AirPlay for live streaming, and WebSockets for wireless communication.

Features
Real-Time Tracking: Uses OpenCV on a Raspberry Pi for dynamic subject detection.
Motorized Adjustment: Stepper motors adjust camera angles to maintain proper framing.
Remote Camera Control: Physical buttons enable zoom, photo capture, and video recording.
Low-Latency Communication: WebSockets provide fast, wireless command transmission.
Dedicated Wi-Fi Network: ESP32-S3 maintains a stable connection between devices.
Custom PCB: Integrates motor drivers, power management, and button inputs.
Getting Started

1. Hardware Setup
Set up the ESP32-S3, Raspberry Pi, and stepper motors.
Connect the ESP32-S3 to physical control buttons.
Configure the Raspberry Pi to receive AirPlay streams and process tracking data.
2. Software Setup
ESP32-S3 Firmware
Flash firmware using ESP-IDF.
Enable WebSockets communication and GPIO handling.
Raspberry Pi (Tracking & AirPlay)
Install RPiPlay for AirPlay streaming.
Install OpenCV and run the tracking algorithm.
iPhone App
Build and install the app using Xcode.
Ensure the app communicates with the ESP32-S3 via WebSockets.
3. Running the System
Power on all components and connect to the ESP32-S3’s Wi-Fi.
Start AirPlay on the iPhone to stream video to the Raspberry Pi.
Use the iPhone app or physical buttons to control the camera.
Verify that the tripod’s motors adjust the camera for real-time tracking.
Testing & Debugging
Latency Testing: Measure command execution delays via timestamp logging.
Network Performance: Assess WebSocket stability and Wi-Fi signal strength.
Motor Accuracy: Use iPhone gyroscope data to validate tracking precision.
License  
The final overall design image are shown below:  
![image](/images/motorized_tripod.jpg)

![image](/images/controller.jpg)
  

This project is licensed under the MIT License. See the LICENSE file for details.

