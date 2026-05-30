# 📦 Autonomous Warehouse Drone v2
**GPS-Denied Indoor UAV Package Delivery Platform — Dual-Core FreeRTOS A\* Autonomy, IR Beacon Triangulation, & TensorFlow Lite Edge Inference**

[![Status](https://img.shields.io/badge/Status-Completed-blue?style=for-the-badge)](https://github.com/yogesh031020/warehouse-drone-v2)
[![Hardware](https://img.shields.io/badge/Hardware-ESP32--S3-white?style=for-the-badge&logo=espressif)](https://www.espressif.com/)
[![Firmware](https://img.shields.io/badge/Firmware-ArduPilot-green?style=for-the-badge&logo=ardupilot)](https://ardupilot.org/)
[![Algorithm](https://img.shields.io/badge/Algorithm-A*%20Pathfinding-orange?style=for-the-badge)](https://github.com/yogesh031020/warehouse-drone-v2)

---

## 🚀 Project Overview
**Warehouse Drone v2** is a fully autonomous, GPS-denied indoor quadcopter designed for automated logistics and package delivery inside industrial warehouse grids.

Bypassing the need for external GPS or costly motion-capture networks, the UAV is controlled by a custom **ESP32 companion computer** running a low-latency, dual-core **FreeRTOS** operating kernel. The companion computer dynamically solves real-time 3D flight maps using an **A\* pathfinding engine**, performs high-accuracy **IR beacon triangulation** for ±3cm docking, and handles forward collision mapping using a **TensorFlow Lite Micro** edge machine learning model.

---

## 🎬 Autonomous Mission Demo

> **Recording in progress** — GIF will show: full A\* mission from takeoff → grid navigation → precision IR docking → payload drop → autonomous land.

<!-- Replace the line below with your actual GIF once recorded -->
<!-- ![Warehouse Mission Demo](docs/images/warehouse_mission_demo.gif) -->

**What to record:**
1. Arm and engage companion override from RC transmitter
2. Capture the full A\* autonomous mission: takeoff → grid waypoints → IR docking approach → MG90S servo gripper release → land
3. Optionally record the live WiFi telemetry HUD on a second screen alongside the flight
4. Export as GIF (≤15MB) and save to `docs/images/warehouse_mission_demo.gif`

---

## 📸 Avionics Showcase
<div align="center">
  <table border="0">
    <tr>
      <td><img src="docs/images/drone_front.jpg" width="380" alt="Integrated Quadcopter Frame"></td>
      <td><img src="docs/images/drone_esp32.jpg" width="380" alt="Avionics Control Stack"></td>
    </tr>
  </table>
  <p><i>Left: Dual-Beam Ultrasonic Rangefinder & Servo Package Gripper | Right: ESP32 Avionics Core Interface</i></p>
</div>

---

## 🧠 System Architecture & Multi-Tasking Loop

To achieve rapid <50ms obstacle avoidance reaction times and low-latency navigation, the ESP32's dual cores partition computational tasks using a strict **FreeRTOS task schedule**:

```mermaid
graph TD
    subgraph ESP32 Silicon Cores
        Core0[Core 0: High-Priority Autonomy Task]
        Core1[Core 1: System Communications Task]
    end

    subgraph Sensors & Signals
        IR[4x IR Beacon Receivers] -->|GPIO Interrupts| Core0
        Lidar[TF-Mini LiDAR Sensor] -->|UART Serial1| Core1
        Ultra[Ultrasonic Distance Sensor] -->|GPIO Echo/Trig| Core1
    end

    subgraph Core 0 Execution Loop
        Core0 -->|Dynamic Triangulation| Docking[Precision IR Docking Mode]
        Core0 -->|Grid Map Matrix| AStar[A* Pathfinding Solver]
        AStar -->|Generated Waypoints| State[Autonomy State Machine]
    end

    subgraph Core 1 Execution Loop
        Core1 -->|Ultrasonic Ranges| Avoider[Obstacle Avoidance Engine]
        Core1 -->|Raw Image / Range Vectors| TFLite[TensorFlow Lite Edge Inference]
        Avoider -->|Failsafe Flags| InterComm[Shared Memory Queue]
        TFLite -->|Obstacle Probability| InterComm
        InterComm <-->|Real-time Failsafe Signals| State
    end

    subgraph Actuation & Telemetry
        Core1 -->|MAVLink Serial2 Override| APM[APM 2.8 Flight Controller]
        APM -->|PWM Commands| Motors[Brushless Motors Quad-X]
        APM -->|Servo PWM| Gripper[Payload Servo Gripper]
        Core1 -.->|LAN Telemetry Debug| WebUI[Live WiFi Web Telemetry HUD]
    end
```

---

## 🔧 Systems Engineering: Key Technical Challenges

> [!WARNING]
> **Computational Latency on Single-Core Loops**
> **Challenge:** Running A\* pathfinding, MAVLink serial loops, and sensor processing in a single execution loop caused cycles to jump to ~180ms — making flight control extremely laggy, resulting in crashes.
> **Solution:** Migrated to a **dual-core FreeRTOS concurrency model**. A\* pathfinding and state-machine strictly on **Core 0**; telemetry, LiDAR UART, and Ultrasonic loops offloaded to **Core 1**. Latency cut to a clean **~40ms**.

> [!IMPORTANT]
> **IR Triangulation Ambient Interference**
> **Challenge:** Ambient light and reflection off warehouse metal columns introduced a 15cm position variance in IR beacon signals, causing docking misses.
> **Solution:** Engineered a hardware-software bandpass signal filter and implemented a 4-beacon redundancy voting array with interrupt-driven triggers, reducing alignment errors to **±3cm**.

> [!TIP]
> **TFLite Obstacle Pattern Inference**
> **Challenge:** Classic ultrasonic distance checks struggle to classify moving humans vs. static boxes.
> **Solution:** Integrated an onboard **TensorFlow Lite Micro** model running an optimized neural network — maps distance vectors to identify human motion footprints at a **<50ms reaction threshold**.

---

## 🔌 Hardware Configuration & Mappings

| Component | Hardware Module | Purpose / Role | Interface / Mapped Pins |
| :--- | :--- | :--- | :--- |
| **Companion Computer** | ESP32-WROOM-32 | Multi-tasking Navigation & A\* Planner | Core CPU |
| **Flight Controller** | APM 2.8 | Flight Dynamics & Attitude Control | Serial2 (TX: GPIO 17, RX: GPIO 16) |
| **Altitude Sensor** | TF-Mini LiDAR | Downward altitude lock | UART Serial1 (TX: GPIO 9, RX: GPIO 10) |
| **Collision Sensor** | HC-SR04 Ultrasonic | Forward obstacle avoidance | Trig: GPIO 14, Echo: GPIO 27 |
| **Docking Suite** | 4x TSOP38238 IR Rx | Grid quadrant triangulation | GPIO 32, 33, 34, 35 (Hardware interrupts) |
| **Payload Actuator** | MG90S Metal Servo | Package release mechanism | GPIO 25 (High-Frequency PWM) |
| **Visual Debug** | Onboard LED | Telemetry heartbeat indicator | GPIO 2 (Digital Out) |

---

## 🛠️ Step-by-Step "How to Run" & Deployment Guide

### 1. Configure the Arduino Development IDE
1. Open the [Arduino IDE](https://www.arduino.cc/en/software).
2. Install the **ESP32 Core** (Espressif v2.0.0+) by adding `https://dl.espressif.com/dl/package_esp32_index.json` to your Preferences.
3. Install the required **TensorFlow Lite Micro** library:
   - Go to **Sketch → Include Library → Manage Libraries...**
   - Search for `TensorFlowLite_ESP32` or import the Flatbuffer-compatible ESP32 zip.

### 2. Configure Coordinates & Flatbuffer Weights
1. Open `warehouse_drone.ino` — all tabs import automatically.
2. Edit `config.h` to set your target grid coordinates:
   ```cpp
   #define TARGET_GRID_X  4   // Destination grid x
   #define TARGET_GRID_Y  6   // Destination grid y
   #define TARGET_GRID_Z  2   // Safe flight altitude (m)
   ```
3. *(Optional)* If you retrained the obstacle classifier, update `model_data.h` with your compiled flatbuffer binary weights.

### 3. Flash the Companion Core
1. Connect the ESP32 via micro-USB data cable.
2. Under **Tools**, select **Board: ESP32 Dev Module** and the correct COM Port.
3. Set partition scheme to **Huge APP** (≥1.2MB APP + OTA) to fit the TFLite model.
4. Click **Upload** to compile and flash.

### 4. Physical Setup & Calibration
1. Mount the 4x **TSOP38238 IR sensors** on the outer frame corners, pointing outward and slightly downward.
2. Place **38kHz active IR beacons** in the four corners of your flight test area.
3. Power the **MG90S gripper servo** from a dedicated **5V BEC** rail — not the ESP32 — to prevent voltage spikes.
4. Verify HC-SR04 sonar sensors are calibrated and pointed forward.

### 5. Launch Autonomous Mission
1. Power on the beacons, then power the UAV avionics stack.
2. Enable manual RC overrides on your transmitter as a physical safety interlock.
3. Arm the APM in **ALT_HOLD** mode.
4. Once altitude hold is stable, engage the companion override switch.
5. Core 0 A\* solver calculates grid waypoints → Core 1 sonar avoids obstacles → IR triangulation docks at destination → MG90S servo releases payload → autonomous land sequence triggers.

---

## 💻 MAVLink Digital Twin Simulation (SITL)

Before uploading custom flight firmware to a physical drone, the entire mission architecture can be validated in a **Software-in-the-Loop (SITL)** digital twin environment using ArduPilot and Mission Planner. The included Python script `sitl_warehouse_mission.py` models the exact autonomous state machine, MAVLink override sequence, and grid pathfinding telemetry.

### Running the Virtual Mission:

1. **Install Dependencies:**
   Install required Python packages for MAVLink protocol parsing:
   ```bash
   pip install -r requirements.txt
   ```

2. **Launch the SITL Autopilot inside Mission Planner:**
   * Open **Mission Planner** on your PC.
   * Go to the **Simulation** tab in the top menu bar.
   * Select the **Multicopter** frame icon. Mission Planner will download and launch a local virtual ArduCopter instance.
   * Wait until the autopilot finishes calibrating its virtual navigation filters (the HUD displays `AHRS OK` and the position stabilizes).

3. **Execute the Autonomy Script:**
   Start the state-machine controller:
   ```bash
   python sitl_warehouse_mission.py
   ```

### Simulation Execution Sequence:
* **Pre-Flight Diagnostics:** Runs virtual check loops for LiDAR, ultrasonic altimeters, and package grippers.
* **Auto-Mission Upload:** Generates a 3D coordinate path to the PICKUP station, then to the DELIVERY station, and finally back HOME. Automatically uploads the waypoints directly into ArduPilot's flight sequencer over a local TCP loop (`127.0.0.1:5762`).
* **Armed Flight Launch:** Switches the virtual autopilot to `GUIDED`, arms the virtual motors, commands a takeoff rise to 10.0m cruise altitude, then switches to `AUTO` mode to hand control over to the uploaded mission array.
* **Autonomous Mission Cycle:** Navigates the coordinates, reports real-time metric distances, simulates package grasping at 1.5m, performs dropping off at the target coordinates, flies home, commands `LAND`, and disarms upon landing confirmation!

---

## 📂 Repository Directory Layout

```
warehouse-drone-v2/
├── config.h                   # Grid layout, coordinate matrices, and pin configurations
├── gripper.h                  # Servo release driver library
├── ir_beacon.h                # IR interrupt parsing and triangulation equations
├── lidar.h                    # Downward LiDAR TF-Mini distance estimator
├── mavlink_comm.h             # Advanced MAVLink telemetry protocol interface
├── ml_inference.h             # TensorFlow Lite Micro classifier interface
├── model_data.h               # Compiled flatbuffer neural network weights
├── obstacle_avoid.h           # Ultrasonic array driver and alert threshold loop
├── path_planner.h             # Optimized C++ A* 3D pathfinding engine
├── state_machine.h            # Mission state loops (Takeoff, Path, Dock, Release, Land)
├── ultrasonic.h               # High-precision HC-SR04 sonar pulse handler
├── warehouse_map.h            # 3D grid voxel matrix representation
├── wifi_debug.h               # UDP-based LAN telemetry logging engine
├── warehouse_drone.ino        # Core Arduino entry sketch orchestrating FreeRTOS tasks
├── sitl_warehouse_mission.py  # MAVLink SITL digital twin simulation controller
├── requirements.txt           # Python dependency specifications for SITL simulation
├── docs/
│   ├── Hardware_Wiring_Map.md          # Detailed system wiring and logic schematic
│   ├── warehouse_v2_execution.log      # Real-time mission execution and ML debug log
│   └── images/
│       ├── drone_front.jpg             # Front flight assembly visual showcase
│       └── drone_esp32.jpg             # Close-up companion board hardware layout
└── LICENSE                    # MIT License
```

---

### Aeronautical & Autonomy Systems Engineering Portfolio
- **Developed by:** Yogesh E S — Aeronautical Systems Engineer
- **Contact/Portfolio:** [GitHub Profile](https://github.com/yogesh031020)
