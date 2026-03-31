# Warehouse Drone v2.0 🚁
**Autonomous Pickup & Delivery System**

An advanced autonomous indoor drone built for navigating warehouse environments, locating packages, picking them up via a custom servo gripper, and delivering them to designated drop-off zones. 

Driven by an **ESP32** acting as a high-level Mission Commander, this drone connects to an **APM 2.8** flight controller via MAVLink to achieve autonomous indoor flight without GPS.

## 🌟 Key Features
* **Full Autonomy State Machine:** Seamlessly transitions through Takeoff, Path Navigation, IR Alignment, Package Pickup, Delivery Delivery, and automated Landing.
* **Sensor Fusion Navigation:** 
  * **TF Mini LiDAR** for forward obstacle avoidance constraints.
  * **HC-SR04 Ultrasonic** for precision low-level altitude hold during pickups/drop-offs.
  * **IR Beacon Receiver** for pinpointing pickup/delivery zones based on infrared signatures.
* **Machine Learning Inference:** Capable of running predictive models (`ml_inference.h`) to handle complex sensor inputs.
* **A* Path Planning:** Maps out coordinates for Home, Pickup, and Delivery zones and calculates the most efficient route.
* **WiFi Telemetry Dashboard:** Supports a live asynchronous Web/Telnet terminal displaying state machine status, MAVLink heartbeats, and sensor readings.

## 🛠️ Hardware Stack
* ESP32 (Mission Commander)
* APM 2.8 (Flight Controller)
* TF Mini LiDAR (Obstacle Avoidance)
* HC-SR04 Ultrasonic (Precision Alt-Hold)
* 1838 IR Receiver (Zone Alignment)
* 9g Servo (Gripper Mechanism)

## 🎮 Command Interface
Connect to the ESP32 via USB Serial or Telnet to issue commands:
* `'G'` = Start Full Autonomous Mission
* `'X'` = Emergency Abort
* `'1/2/3'` = Generate Path Navigations
* `'O/C'` = Open / Close Gripper Hook
