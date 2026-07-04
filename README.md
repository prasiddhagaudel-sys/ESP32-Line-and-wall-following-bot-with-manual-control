ESP32 Hybrid Robot with Integrated Web Control UI
An advanced, dual-core ESP32 robotics project featuring autonomous navigation (line tracking and wall following) and a self-hosted, high-performance web interface for manual control.

This project is built for extreme stability by separating the web stack and the motor/sensor logic across the two hardware cores of the ESP32. It also includes dynamic WiFi power management to completely eliminate brownouts during high-current motor operations.

✨ Key Features
Dual-Core Architecture for Zero Blocking:
Core 0 is dedicated exclusively to the WiFi stack, HTTP Web Server, and WebSockets. This ensures that the browser never disconnects or misses a heartbeat, even when ultrasonic sensors block the main loop.
Core 1 runs the time-sensitive motor control and sensor polling (IR line sensors and Ultrasonic sensors).
Fully Offline, Self-Contained Web UI: The entire HTML, CSS, and JavaScript interface is stored in PROGMEM. The ESP32 acts as its own Access Point (AP), meaning no external router, internet connection, or external CDNs are required.
Dynamic WiFi Power Scaling: ESP32 boards are notorious for browning out when motors and the WiFi transmitter peak at the same time. This code dynamically scales the WiFi TX power down to -1dBm during autonomous IR line following (where range isn't needed) and boosts it back to 8.5dBm for manual control modes to ensure uninterrupted range and low latency.
Hybrid Autonomous Modes:
IR Line Following: Fast and precise line tracking using a 3-channel IR sensor array.
Ultrasonic Wall Following: Seamlessly transitions into wall-following mode when obstacles are detected under 30cm, avoiding collisions using 3 HC-SR04 ultrasonic sensors.
Premium Manual Control (Web UI):
Analog Joystick: Provides smooth, proportional speed and steering control.
D-Pad: Offers fast, discrete, and precise directional movements.
Live Telemetry: The web interface displays a real-time feed of the IR sensor states and ultrasonic distance values (broadcasted every 150ms over WebSockets).
🛠️ Hardware Requirements
ESP32 Development Board
2x DC Motors with a Motor Driver (e.g., L298N)
3x IR Line Tracking Sensors
3x HC-SR04 Ultrasonic Distance Sensors
Appropriate power supply for motors & ESP32
📦 Required Libraries
Make sure to install the following libraries in your Arduino IDE Library Manager:

WebSockets by Markus Sattler
NewPing by Tim Eckel
(Note: WebServer.h and WiFi.h are built into the ESP32 Arduino Core)
🚀 How to Use
Flash the code to your ESP32.
Connect your phone or computer to the WiFi network RobotControl (Password: BMD12345).
Open a web browser and navigate to http://192.168.4.1.
Use the toggle switch in the UI to seamlessly transition between Autonomous and Manual modes!
