#ifndef CONFIG_H
#define CONFIG_H

// ============================================
//  WAREHOUSE DRONE - Pin & Constant Config
// ============================================

// === TF Mini LiDAR (UART2) ===
#define LIDAR_RX_PIN    16   // ESP32 RX2 <- LiDAR TX
#define LIDAR_TX_PIN    17   // ESP32 TX2 -> LiDAR RX
#define LIDAR_BAUD      115200

// === Ultrasonic HC-SR04 ===
#define ULTRA_TRIG_PIN  12
#define ULTRA_ECHO_PIN  14   // Must use voltage divider (5V -> 3.3V)

// === IR Receiver ===
#define IR_RECV_PIN     13   // Changed from 34 (GPIO34 has no pullup)

// === Gripper Servo ===
#define SERVO_PIN       25

// === MAVLink Serial to APM 2.8 (UART1) ===
#define MAV_RX_PIN      26   // ESP32 RX <- APM TX  (Swapped for direct wire)
#define MAV_TX_PIN      27   // ESP32 TX -> APM RX  (Swapped for direct wire)
#define MAV_BAUD        57600

// === MAVLink System IDs ===
#define GCS_SYS_ID      255  // ESP32 acts as GCS (Ground Control Station)
#define GCS_COMP_ID     0    // Mission Planner often uses 0 or 190
#define APM_SYS_ID      1    // APM 2.8 default system ID
#define APM_COMP_ID     1    // APM 2.8 default component ID

// === Flight Thresholds ===
#define OBSTACLE_CLOSE_CM    50    // LiDAR: too close, stop!
#define OBSTACLE_WARN_CM     150   // LiDAR: slow down
#define CRUISE_ALTITUDE_CM   100   // Cruise altitude (1 meter)
#define PICKUP_ALTITUDE_CM   15    // Descend to this for pickup
#define LANDING_COMPLETE_CM  5     // Consider landed below this

// === Gripper Angles ===
#define GRIPPER_OPEN         90    // Servo angle - open (inverted for physical mount)
#define GRIPPER_CLOSED       0     // Servo angle - gripping

// === Timing ===
#define SENSOR_READ_INTERVAL_MS  20    // 50Hz sensor loop
#define MAV_HEARTBEAT_MS         1000  // 1Hz heartbeat
#define TELEMETRY_PRINT_MS       1000  // Serial print interval
#define STATION_TIMEOUT_MS       2000  // IR signal lost timeout
#define APPROACH_TIMEOUT_MS      10000 // Max time to find station

#endif
