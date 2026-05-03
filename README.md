# 📦 Warehouse-Drone-V2

![Status](https://img.shields.io/badge/Status-Completed-blue)
![Hardware](https://img.shields.io/badge/Hardware-ESP32--S3-white)
![Language](https://img.shields.io/badge/Language-C++%20%7C%20Python-blue)
![Firmware](https://img.shields.io/badge/Firmware-ArduPilot-green)
![Algorithm](https://img.shields.io/badge/Algorithm-A*%20Pathfinding-orange)

# Warehouse Drone v2 — Autonomous Indoor UAV

GPS-free autonomous package delivery drone for warehouse environments. Built on ESP32 + APM 2.8 with A* pathfinding, IR beacon docking, and edge ML inference — all running without GPS or external positioning systems.

## Why I built this

After v1 (basic waypoint following) struggled with drift in GPS-denied spaces, I rearchitected around IR beacons for precision docking and a custom A* planner that runs entirely on the ESP32's dual core.

## Hardware

| Component | Purpose |
|---|---|
| ESP32 (Dual-Core) | Companion computer — pathfinding + MAVLink |
| APM 2.8 | Flight controller |
| TF-Mini LiDAR | Forward obstacle detection |
| HC-SR04 Ultrasonic | Altitude hold (indoor) |
| IR Beacons (×4) | Precision docking markers |
| Servo Gripper | Package pickup/release |

## System architecture

```
[IR Beacons] → [ESP32 Core 0: Navigation + A*]
[TF-Mini]   →         ↓ MAVLink
[Ultrasonic] →  [APM 2.8 Flight Controller]
                       ↓
               [Motors + Gripper]
```

## What worked and what didn't

The first A* implementation ran too slow on a single core (~180ms per cycle) — moved pathfinding to Core 0 and MAVLink comms to Core 1 using FreeRTOS tasks, which brought it down to ~40ms.

IR beacon triangulation initially had 15cm position error due to ambient IR interference. Fixed with a bandpass filter and 4-beacon redundancy, achieving ~3cm docking accuracy in tests.

## Setup

```bash
# Flash with Arduino IDE 2.x
# Board: ESP32 Dev Module
# Required libraries: MAVLink, TFLite_micro, IRremote
```

Edit `config.h` for your warehouse grid dimensions and beacon pin assignments.

## Results

- Navigates a 10×10 grid autonomously in ~45 seconds
- Docking accuracy: ±3cm using IR triangulation
- Obstacle avoidance reaction time: <50ms

## Status

Active — v2.1 adds WiFi debug monitor for live telemetry over LAN.
