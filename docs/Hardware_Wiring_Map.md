# 🔌 Warehouse Drone v2: Systems Wiring Map
**Detailed Avionics Pinouts, Logic Level Shifting, & Power Grid Distribution**

This document establishes the electrical interfaces and pin mapping between the **ESP32 companion computer**, the **APM 2.8 flight controller**, and the comprehensive sensor/actuator suite.

---

## 📊 Schematic Topology
```mermaid
graph TD
    subgraph Power Grid
        PDB[Power Distribution Board] -->|11.1V - 14.8V| BEC[5V 3A UBEC]
        BEC -->|Clean 5V Power| ESP32_V5[ESP32 VIN]
        BEC -->|Clean 5V Power| LiDAR_V5[TF-Mini VCC]
        BEC -->|Clean 5V Power| Ultra_V5[Ultrasonic HC-SR04 VCC]
        PDB -->|Servo Power| Servo_BEC[5V/6V BEC]
        Servo_BEC -->|Dedicated Servo Power| Servo[Gripper Servo MG90S]
    end

    subgraph MAVLink Telemetry Logic
        ESP32_RX2[ESP32 GPIO 16 RX2] <-->|3.3V Logic| LLS[Logic Level Shifter]
        ESP32_TX2[ESP32 GPIO 17 TX2] <-->|3.3V Logic| LLS
        LLS <-->|5V Logic| APM_TX2[APM Serial2 TX]
        LLS <-->|5V Logic| APM_RX2[APM Serial2 RX]
    end

    subgraph Dedicated Hardware Serial
        ESP32_RX1[ESP32 GPIO 9 RX1] <-->|3.3V Direct UART| LiDAR_TX[TF-Mini UART TX]
        ESP32_TX1[ESP32 GPIO 10 TX1] <-->|3.3V Direct UART| LiDAR_RX[TF-Mini UART RX]
    end

    subgraph Sensor Suite GPIOs
        ESP32_Trig[ESP32 GPIO 14 Out] -->|Trigger Pulse| Ultra_Trig[HC-SR04 Trig]
        ESP32_Echo[ESP32 GPIO 27 In] <--- |Echo Pulse w/ Divider| Ultra_Echo[HC-SR04 Echo]
        ESP32_IR1[ESP32 GPIO 32 Interrupt] <--- IR1[TSOP IR Receiver 1]
        ESP32_IR2[ESP32 GPIO 33 Interrupt] <--- IR2[TSOP IR Receiver 2]
        ESP32_IR3[ESP32 GPIO 34 Interrupt] <--- IR3[TSOP IR Receiver 3]
        ESP32_IR4[ESP32 GPIO 35 Interrupt] <--- IR4[TSOP IR Receiver 4]
    end
```

---

## ⚡ Core Wiring Map

### 1. Power Distribution Rail
The ESP32 processes high-frequency A* algorithms and TensorFlow Lite inference while the servo gripper actuates. This can pull significant current transients. Mismatching power lines will trigger brown-outs or reset your flight controllers.

| Source | Voltage | Destination Pin | Purpose |
| :--- | :--- | :--- | :--- |
| **5V 3A UBEC** | `5V (VCC)` | ESP32 `VIN` / `5V` | Primary companion logic power |
| **5V 3A UBEC** | `5V (VCC)` | TF-Mini `VCC` | Primary downward LiDAR power |
| **5V 3A UBEC** | `5V (VCC)` | HC-SR04 `VCC` | Forward Ultrasonic array power |
| **Dedicated Servo BEC** | `5V-6V` | MG90S Servo `VCC` | Actuator motor power (isolated from sensor rail) |
| **PDB Ground** | `GND` | Common Ground Bus | Connect all GND pins together to prevent floating signals |

---

### 2. Signal & Mapped Pinouts

| Device 1 (From) | Pin Mapped | Protocol / Bus | Device 2 (To) | Pin Mapped | Electrical Level / Details |
| :--- | :--- | :--- | :--- | :--- | :--- |
| **ESP32** | `GPIO 16 (RX2)` | MAVLink Telemetry | **APM 2.8** | `TX (Serial2)` | **Requires LLS (5V ↔ 3.3V)** |
| **ESP32** | `GPIO 17 (TX2)` | MAVLink Telemetry | **APM 2.8** | `RX (Serial2)` | **Requires LLS (5V ↔ 3.3V)** |
| **ESP32** | `GPIO 9 (RX1)` | LiDAR UART | **TF-Mini** | `TX` | Direct 3.3V UART |
| **ESP32** | `GPIO 10 (TX1)` | LiDAR UART | **TF-Mini** | `RX` | Direct 3.3V UART |
| **ESP32** | `GPIO 14` | Sonar Trigger | **HC-SR04** | `Trig` | Direct 3.3V Digital Out |
| **ESP32** | `GPIO 27` | Sonar Echo | **HC-SR04** | `Echo` | **Requires Voltage Divider (5V to 3.3V)** |
| **ESP32** | `GPIO 32` | Front IR Receiver | **TSOP38238 1** | `Out` | Interrupt-driven Digital In |
| **ESP32** | `GPIO 33` | Right IR Receiver | **TSOP38238 2** | `Out` | Interrupt-driven Digital In |
| **ESP32** | `GPIO 34` | Back IR Receiver | **TSOP38238 3** | `Out` | Interrupt-driven Digital In |
| **ESP32** | `GPIO 35` | Left IR Receiver | **TSOP38238 4** | `Out` | Interrupt-driven Digital In |
| **ESP32** | `GPIO 25` | Payload Release | **MG90S Servo** | `PWM` | Direct 3.3V PWM Out |
| **ESP32** | `GPIO 2` | Onboard visual LED | **Status LED** | `Anode` | High-frequency heartbeat pin |

---

## ⚠️ Critical Avionics Engineering Instructions

> [!WARNING]  
> **Logic Level Translation (MAVLink Telemetry)**  
> The APM 2.8 telemetry port operates at **5.0V Logic**. The ESP32 is **3.3V and NOT 5.0V tolerant**. Standard MAVLink communication between APM TX and ESP32 RX **MUST** use a bi-directional Logic Level Shifter (LLS) board to prevent chip destruction.

> [!IMPORTANT]  
> **Voltage Divider on Ultrasonic Echo**  
> The HC-SR04 sensor operates strictly at **5V** and emits a 5V pulse on its `Echo` pin. Connecting this directly to ESP32 `GPIO 27` will burn out the GPIO register. You **MUST** use a simple resistor voltage divider (e.g., 1kΩ and 2kΩ resistors) or an LLS channel to scale the echo signal down to 3.3V.

> [!TIP]  
> **Dedicated Servo Power Isolation**  
> Servos draw high inductive transient currents during motor actuation. **Never** power the MG90S payload servo from the same 5V regulator powering the ESP32 and sensors. A sudden servo load will sag the voltage, crashing your navigation stack. Connect the servo to a separate BEC.
