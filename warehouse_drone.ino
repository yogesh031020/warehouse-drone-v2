// ============================================
//  WAREHOUSE DRONE v2.0 — FULL MISSION
//  Autonomous Pickup & Delivery System
//
//  Serial Commands:
//    'G' = GO! Start autonomous mission
//    'X' = ABORT! Emergency stop
//    'N' = Next step (simulation mode)
//    '1' = Plan: HOME → PICKUP
//    '2' = Plan: PICKUP → DELIVERY
//    '3' = Plan: DELIVERY → HOME
//    'M' = Show map
//    'O'/'C' = Open/Close gripper
// ============================================

#include "config.h"
#include "wifi_debug.h"

#include "lidar.h"
#include "ultrasonic.h"
#include "ir_beacon.h"
#include "mavlink_comm.h"
#include "gripper.h"
#include "warehouse_map.h"
#include "path_planner.h"
#include "obstacle_avoid.h"
#include "ml_inference.h"
#include "state_machine.h"

// === All modules ===
LidarSensor         lidar;
UltrasonicSensor    ultra;
IRBeacon            irBeacon;
MavlinkComm         mavlink;
Gripper             gripper;
PathPlanner         planner;
ObstacleAvoider     avoider;
MLInference         mlEngine;
MissionStateMachine mission;

// === Timing ===
unsigned long lastTelemetry = 0;

void setup() {
    Serial.begin(115200);
    delay(1000);

    // Initialize WiFi and OTA
    wifiDebug.begin();

    Serial.println();
    Serial.println("╔══════════════════════════════════════╗");
    Serial.println("║  WAREHOUSE DRONE v2.0                ║");
    Serial.println("║  Autonomous Pickup & Delivery        ║");
    Serial.println("║  Full Mission System                 ║");
    Serial.println("╚══════════════════════════════════════╝");
    Serial.println();

    // Initialize all subsystems
    Serial.print("[INIT] LiDAR........... ");
    lidar.init();
    Serial.println("OK");

    Serial.print("[INIT] Ultrasonic...... ");
    ultra.init();
    Serial.println("OK");

    Serial.print("[INIT] IR Beacon....... ");
    irBeacon.init();
    Serial.println("OK");

    Serial.print("[INIT] MAVLink......... ");
    mavlink.init();
    Serial.println("OK");

    Serial.print("[INIT] Gripper......... ");
    gripper.init();
    Serial.println("OK");

    Serial.print("[INIT] Obstacle Avoid.. ");
    avoider.init();
    Serial.println("OK");

    Serial.print("[INIT] ML Engine....... ");
    mlEngine.init();

    Serial.print("[INIT] State Machine... ");
    mission.init(&lidar, &ultra, &irBeacon, &mavlink,
                 &gripper, &planner, &avoider, &mlEngine);
    Serial.println("OK");

    Serial.println();
    Serial.println("┌──────────────────────────────────────┐");
    Serial.println("│  Commands:                           │");
    Serial.println("│  'G' = START autonomous mission     │");
    Serial.println("│  'B' = BYPASS sensor check & START  │");
    Serial.println("│  'T' = AT ALTITUDE (proceed to NAV) │");
    Serial.println("│  'X' = EMERGENCY STOP               │");
    Serial.println("│  'N' = Simulate next step           │");
    Serial.println("│  '1/2/3' = Plan paths               │");
    Serial.println("│  'M' = Show map                     │");
    Serial.println("│  'O/C' = Open/Close gripper         │");
    Serial.println("└──────────────────────────────────────┘");
    Serial.println();
    Serial.println("Status: IDLE — Press 'G' to start or 'N' to simulate");
    Serial.println();
}

// Shared command handler — called from USB Serial AND WiFi Telnet
void handleCommand(char cmd) {
    switch (cmd) {
            // Mission controls
            case 'G': case 'g':
                mission.startMission();
                break;
            case 'B': case 'b':
                mission.forceMissionStart();
                break;
            case 'T': case 't':
                mission.advanceTakeoff();   // Manual: "I've reached altitude, proceed to NAV"
                break;
            case 'X': case 'x':
                mission.abortMission();
                break;
            case 'N': case 'n':
                mission.simulateStep();
                break;

            // Manual controls
            case 'O': case 'o':
                gripper.open();
                break;
            case 'C': case 'c':
                gripper.close();
                break;

            // Navigation test
            case '1':
                Serial.println("\n=== Planning: HOME -> PICKUP ===");
                planner.findPath(HOME_POS.x, HOME_POS.y,
                                 PICKUP_POS.x, PICKUP_POS.y);
                planner.printMapWithPath();
                break;
            case '2':
                Serial.println("\n=== Planning: PICKUP -> DELIVERY ===");
                planner.findPath(PICKUP_POS.x, PICKUP_POS.y,
                                 DELIVERY_POS.x, DELIVERY_POS.y);
                planner.printMapWithPath();
                break;
            case '3':
                Serial.println("\n=== Planning: DELIVERY -> HOME ===");
                planner.findPath(DELIVERY_POS.x, DELIVERY_POS.y,
                                 HOME_POS.x, HOME_POS.y);
                planner.printMapWithPath();
                break;
            case 'M': case 'm':
                planner.printMapWithPath();
                break;
            case 'L': case 'l':
                lidar.printDiag();
                break;
            case 'P': case 'p':
                Serial.println("\n[TEST] Requesting APM parameter list...");
                Serial.println("[TEST] If TX line works -> you will see [MAV] PARAM ... = ...");
                Serial.println("[TEST] If silence for 3 seconds -> TX wire is NOT connected.");
                mavlink.requestParamList();
                break;
        }
    }
void loop() {
    // === 1. Handle WiFi OTA + HTTP + Telnet ===
    wifiDebug.update();

    // === 2. Update all sensors ===
    lidar.update();
    ultra.update();
    irBeacon.update();
    mavlink.heartbeat();

    // === 3. Run state machine ===
    mission.update();

    // === 4. Handle commands from USB Serial ===
    while (Serial.available()) {
        handleCommand((char)Serial.read());
    }

    // === 5. Handle commands from WiFi Telnet ===
    WiFiClient& wifiClient = wifiDebug.getClient();
    while (wifiClient && wifiClient.connected() && wifiClient.available()) {
        handleCommand((char)wifiClient.read());
    }

    // === 6. Handle commands from HTTP Web Dashboard ===
    char webCmd = wifiDebug.getPendingCmd();
    if (webCmd != '\0') {
        handleCommand(webCmd);
    }

    // === 7. Telemetry Dashboard ===
    if (millis() - lastTelemetry > TELEMETRY_PRINT_MS) {
        char buf[1024];
        snprintf(buf, sizeof(buf),
            "╔══════════════════════════════════════╗\n"
            "║  State: %-28s  ║\n"
            "╠══════════════════════════════════════╣\n"
            "║  LiDAR:    %4d cm  %-15s  ║\n"
            "║  Altitude: %5.1f cm (filt: %5.1f)    ║\n"
            "║  IR: %-7s Gripper: %-6s         ║\n"
            "║  Path: %d/%d  ML: %-19s║\n"
            "║  APM: %-9s Mode: %-14s  ║\n"
            "║  WiFi: %-30s║\n"
            "╚══════════════════════════════════════╝\n",
            mission.getStateName(),
            lidar.getDistance(),
            lidar.isValid() ? (lidar.isObstacleClose() ? "[!! CLOSE !!]" :
                               lidar.isObstacleWarning() ? "[! WARN !]" : "[OK]") : "[FAIL]",
            ultra.getRawAltitude(), ultra.getFilteredAltitude(),
            irBeacon.getStationName(), gripper.hasPackage() ? "HOLD" : "OPEN",
            planner.getCurrentStep(), planner.getPathLength(),
            mlEngine.isModelLoaded() ? "ACTIVE" : "RULES",
            mavlink.isConnected() ? "CONNECTED" : "NO LINK",
            mavlink.isConnected() ? mavlink.getAPMModeName() : "---",
            wifiDebug.getIP().c_str()
        );

        wifiDebug.setTelemetry(buf);
        Serial.print(buf);
        
        lastTelemetry = millis();
    }

    delay(SENSOR_READ_INTERVAL_MS);
}
