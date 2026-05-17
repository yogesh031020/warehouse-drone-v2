# Warehouse Drone v2: Hardware Wiring & Pinout Map

This document outlines the physical connections between the ESP32 (acting as the companion computer/navigator), the APM 2.8 flight controller, and the sensor suite.

## 🧠 Core Systems

| Component | Pin / Port | Connection | Notes |
| :--- | :--- | :--- | :--- |
| **ESP32 (Core)** | `5V` / `GND` | Power Distribution Board | Requires clean 5V, recommend decoupling capacitor. |
| **APM 2.8 (Telemetry)**| `Serial2` (TX/RX)| ESP32 `GPIO 16 / 17` | MAVLink communication (Baud: 57600). Requires Logic Level Shifter (5V to 3.3V). |

## 📡 Sensor Suite

| Sensor | ESP32 Pin | Protocol/Type | Function |
| :--- | :--- | :--- | :--- |
| **TF-Mini LiDAR** | `Serial1` (TX/RX)| UART | Precision downward altitude for indoor flight. |
| **Ultrasonic (Front)**| `GPIO 14` (Trig) | Digital Out | Forward obstacle detection. |
| **Ultrasonic (Front)**| `GPIO 27` (Echo) | Digital In | Forward obstacle distance reading. |
| **IR Beacon Receiver 1**| `GPIO 32` | Digital In (Interrupt) | Precision docking triangulation (Front). |
| **IR Beacon Receiver 2**| `GPIO 33` | Digital In (Interrupt) | Precision docking triangulation (Right). |
| **IR Beacon Receiver 3**| `GPIO 34` | Digital In (Interrupt) | Precision docking triangulation (Back). |
| **IR Beacon Receiver 4**| `GPIO 35` | Digital In (Interrupt) | Precision docking triangulation (Left). |

## ⚙️ Actuators & Debug

| Component | ESP32 Pin | Protocol/Type | Function |
| :--- | :--- | :--- | :--- |
| **Payload Gripper** | `GPIO 25` | PWM | Actuates the servo to release the package. |
| **Status LED** | `GPIO 2` | Digital Out | Onboard LED for visual debug (e.g., blinking on MAVLink heartbeat). |

## ⚠️ Important Hardware Notes
1. **Logic Level Shifting**: The APM 2.8 telemetry port operates at 5V logic. The ESP32 is NOT 5V tolerant. You MUST use a Bi-Directional Logic Level Converter between the APM TX and ESP32 RX.
2. **Power Supply**: Do not power the ESP32 directly from the APM 2.8's telemetry port, as the current draw during WiFi/ML inference can cause brown-outs. Power the ESP32 directly from a dedicated 5V BEC.
