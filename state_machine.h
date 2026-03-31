#ifndef STATE_MACHINE_H
#define STATE_MACHINE_H

#include "config.h"
#include "lidar.h"
#include "ultrasonic.h"
#include "ir_beacon.h"
#include "mavlink_comm.h"
#include "gripper.h"
#include "path_planner.h"
#include "obstacle_avoid.h"
#include "ml_inference.h"

// ============================================
//  MISSION STATE MACHINE
// ============================================
//
//  The drone's full mission cycle:
//
//   ┌──────────┐
//   │   IDLE   │ ← Waiting for "GO" command
//   └────┬─────┘
//        ▼
//   ┌──────────┐
//   │ PRE-ARM  │ ← Safety checks + ARM
//   └────┬─────┘
//        ▼
//   ┌──────────┐
//   │ TAKEOFF  │ ← Switch GUIDED, climb to cruise altitude
//   └────┬─────┘
//        ▼
//   ┌──────────┐
//   │ NAV TO   │ ← A* path to PICKUP station
//   │ PICKUP   │
//   └────┬─────┘
//        ▼
//   ┌──────────┐
//   │ PICK UP  │ ← Descend, grab package, ascend
//   └────┬─────┘
//        ▼
//   ┌──────────┐
//   │ NAV TO   │ ← A* path to DELIVERY station
//   │ DELIVERY │
//   └────┬─────┘
//        ▼
//   ┌──────────┐
//   │ DROP OFF │ ← Descend, release package, ascend
//   └────┬─────┘
//        ▼
//   ┌──────────┐
//   │ NAV TO   │ ← A* path back HOME
//   │  HOME    │
//   └────┬─────┘
//        ▼
//   ┌──────────┐
//   │ LANDING  │ ← Descend slowly and disarm
//   └────┬─────┘
//        ▼
//   ┌──────────┐
//   │ COMPLETE │ ← Mission done!
//   └──────────┘
//
//  Emergency: Any state → EMERGENCY (abort mission)
//

enum MissionState {
    STATE_IDLE,
    STATE_PREARM,
    STATE_TAKEOFF,
    STATE_NAV_PICKUP,
    STATE_PICKING_UP,
    STATE_NAV_DELIVERY,
    STATE_DROPPING_OFF,
    STATE_NAV_HOME,
    STATE_LANDING,
    STATE_COMPLETE,
    STATE_EMERGENCY
};

const char* STATE_NAMES[] = {
    "IDLE", "PRE-ARM", "TAKEOFF",
    "NAV->PICKUP", "PICKING UP",
    "NAV->DELIVERY", "DROPPING OFF",
    "NAV->HOME", "LANDING",
    "COMPLETE", "EMERGENCY"
};

class MissionStateMachine {
private:
    MissionState currentState;
    MissionState previousState;
    unsigned long stateStartTime;
    unsigned long lastNavUpdate;
    int navRetries;
    bool forceStart;
    bool navInitialized;   // true after first-loop path planning in a NAV state
    bool takeoffModeRequested;
    bool takeoffAltRequested;


    // References to all subsystems
    LidarSensor*      lidar;
    UltrasonicSensor* ultra;
    IRBeacon*         irBeacon;
    MavlinkComm*      mavlink;
    Gripper*          gripper;
    PathPlanner*      planner;
    ObstacleAvoider*  avoider;
    MLInference*      ml;

    void changeState(MissionState newState) {
        previousState = currentState;
        currentState = newState;
        stateStartTime = millis();
        navInitialized = false;
        if (newState == STATE_TAKEOFF) {
            takeoffModeRequested = false;
            takeoffAltRequested = false;
        }
        Serial.printf("\n[STATE] %s -> %s\n",
            STATE_NAMES[previousState], STATE_NAMES[currentState]);
    }

    unsigned long stateElapsed() {
        return millis() - stateStartTime;
    }

public:
    void init(LidarSensor* l, UltrasonicSensor* u, IRBeacon* ir,
              MavlinkComm* mav, Gripper* g, PathPlanner* p,
              ObstacleAvoider* a, MLInference* m) {
        lidar    = l;
        ultra    = u;
        irBeacon = ir;
        mavlink  = mav;
        gripper  = g;
        planner  = p;
        avoider  = a;
        ml       = m;

        currentState   = STATE_IDLE;
        previousState  = STATE_IDLE;
        stateStartTime = millis();
        lastNavUpdate  = 0;
        navRetries     = 0;
        forceStart     = false;
        navInitialized = false;
        takeoffModeRequested = false;
        takeoffAltRequested = false;
    }

    // Main update — call every loop iteration
    void update() {
        switch (currentState) {
            case STATE_IDLE:          handleIdle();         break;
            case STATE_PREARM:        handlePrearm();       break;
            case STATE_TAKEOFF:       handleTakeoff();      break;
            case STATE_NAV_PICKUP:    handleNavPickup();    break;
            case STATE_PICKING_UP:    handlePickingUp();    break;
            case STATE_NAV_DELIVERY:  handleNavDelivery();  break;
            case STATE_DROPPING_OFF:  handleDroppingOff();  break;
            case STATE_NAV_HOME:      handleNavHome();      break;
            case STATE_LANDING:       handleLanding();      break;
            case STATE_COMPLETE:      handleComplete();     break;
            case STATE_EMERGENCY:     handleEmergency();    break;
        }
    }

    // 'T' key: manually advance from TAKEOFF to NAV when pilot has reached altitude
    void advanceTakeoff() {
        if (currentState == STATE_TAKEOFF) {
            Serial.println("[TAKEOFF] Manual advance: altitude confirmed by pilot!");
            changeState(STATE_NAV_PICKUP);
        } else {
            Serial.printf("[CMD] 'T' only works in TAKEOFF state (current: %s)\n",
                STATE_NAMES[currentState]);
        }
    }

    // ───── STATE HANDLERS ─────

    void handleIdle() {
        // Automatically start the mission if the pilot arms the drone via RC
        if (mavlink->isArmed()) {
            Serial.println("\n[IDLE] Detect armed from RC! Automatically starting mission...");
            startMission();
        }
    }

    void handlePrearm() {
        // Run sensor checks ONCE at the very start of this state
        if (stateElapsed() < 100) {
            Serial.println("[PREARM] Running safety checks...");

            lidar->update();
            if (lidar->isValid()) {
                Serial.printf("  [OK] LiDAR: %d cm\n", lidar->getDistance());
            } else if (forceStart) {
                Serial.printf("  [BYPASS] LiDAR Error: %d cm (Continuing anyway...)\n", lidar->getDistance());
            } else {
                Serial.printf("  [FAIL] LiDAR error: %d cm\n", lidar->getDistance());
                Serial.println("[PREARM] Aborting. Fix LiDAR or press 'B' to force start.");
                changeState(STATE_IDLE);
                return;
            }

            ultra->update();
            Serial.printf("  [OK] Ultrasonic: %.1f cm\n", ultra->getRawAltitude());

            gripper->open();
            Serial.println("  [OK] Gripper: OPEN");

            Serial.println("[PREARM] All checks passed!");
            Serial.println("[PREARM] Drone is ARMED. Proceeding directly to takeoff sequence!");
            changeState(STATE_TAKEOFF);
            return;
        }
    }


    void handleTakeoff() {
        // Update sensors every loop
        lidar->update();
        ultra->update();
        int lidarDist = lidar->isValid() ? lidar->getDistance() : 9999;

        // Print instructions once at the very start
        if (!takeoffModeRequested) {
            Serial.println("[TAKEOFF] Initiating Autonomous Takeoff...");
            Serial.println("[TAKEOFF] Setting APM to ALT_HOLD mode (Indoor/No-GPS flight)...");
            mavlink->setMode("ALT_HOLD");
            takeoffModeRequested = true;
            return;
        } 
        
        // Wait for APM to confirm it is actually in ALT_HOLD mode
        if (mavlink->getAPMMode() != APM_MODE_ALT_HOLD) {
            if (stateElapsed() % 2000 < 50) {
                Serial.printf("[TAKEOFF] Waiting for ALT_HOLD mode... (Current: %s)\n", mavlink->getAPMModeName());
                mavlink->setMode("ALT_HOLD"); // Remind it just in case packet dropped
            }
            return;
        }

        // Once in ALT_HOLD, we can safely command the takeoff altitude via simulated RC
        if (!takeoffAltRequested) {
            Serial.println("[TAKEOFF] APM confirmed ALT_HOLD. Simulating RC Throttle UP...");
            mavlink->ascend(200);   // Simulate throttle stick pushed up
            takeoffAltRequested = true;
        }

        float altitude = ultra->getFilteredAltitude();

        // Remind every 3 seconds
        if (stateElapsed() % 3000 < 50) {
            Serial.printf("[TAKEOFF] Climbing... LiDAR: %d cm, Alt: %.0f cm\n",
                lidarDist, altitude);
             // Re-send the ascend command periodically during climb to keep it climbing
             mavlink->ascend(200);
        }

        if (altitude >= CRUISE_ALTITUDE_CM * 0.80f) {
            Serial.printf("[TAKEOFF] Reached %.0f cm — cruising at altitude!\n", altitude);
            mavlink->hover(); // Return throttle to neutral (1500)
            changeState(STATE_NAV_PICKUP);
            return;
        }
    }


    void handleNavPickup() {
        if (!navInitialized) {
            navInitialized = true;
            Serial.println("[NAV] Planning route to PICKUP...");
            planner->findPath(HOME_POS.x, HOME_POS.y,
                             PICKUP_POS.x, PICKUP_POS.y);
            planner->printMapWithPath();
        }

        navigatePath();

        if (planner->isPathComplete()) {
            Serial.println("[NAV] Arrived at PICKUP station!");
            changeState(STATE_PICKING_UP);
        }

        if (irBeacon->getStation() == STATION_PICKUP) {
            Serial.println("[NAV] IR Beacon: PICKUP detected!");
            changeState(STATE_PICKING_UP);
        }
    }

    void handlePickingUp() {
        unsigned long elapsed = stateElapsed();

        if (elapsed < 2000) {
            if (elapsed < 500) {
                Serial.println("[PICKUP] Descending to package...");
                mavlink->moveDown(PICKUP_ALTITUDE_CM);
            }
        } else if (elapsed < 4000) {
            if (elapsed < 2500) {
                Serial.println("[PICKUP] Grabbing package...");
                gripper->close();
            }
        } else if (elapsed < 6000) {
            if (elapsed < 4500) {
                Serial.println("[PICKUP] Ascending...");
                mavlink->takeoff(CRUISE_ALTITUDE_CM);
            }
        } else {
            if (gripper->hasPackage()) {
                Serial.println("[PICKUP] Package secured!");
                changeState(STATE_NAV_DELIVERY);
            } else {
                navRetries++;
                if (navRetries < 3) {
                    Serial.println("[PICKUP] Retry grab...");
                    stateStartTime = millis();
                } else {
                    Serial.println("[PICKUP] Failed after 3 attempts!");
                    changeState(STATE_EMERGENCY);
                }
            }
        }
    }

    void handleNavDelivery() {
        if (!navInitialized) {
            navInitialized = true;
            navRetries = 0;
            Serial.println("[NAV] Planning route to DELIVERY...");
            planner->findPath(PICKUP_POS.x, PICKUP_POS.y,
                             DELIVERY_POS.x, DELIVERY_POS.y);
            planner->printMapWithPath();
        }

        navigatePath();

        if (planner->isPathComplete()) {
            Serial.println("[NAV] Arrived at DELIVERY station!");
            changeState(STATE_DROPPING_OFF);
        }

        if (irBeacon->getStation() == STATION_DELIVERY) {
            Serial.println("[NAV] IR Beacon: DELIVERY detected!");
            changeState(STATE_DROPPING_OFF);
        }
    }

    void handleDroppingOff() {
        unsigned long elapsed = stateElapsed();

        if (elapsed < 2000) {
            if (elapsed < 500) {
                Serial.println("[DELIVER] Descending...");
                mavlink->moveDown(PICKUP_ALTITUDE_CM);
            }
        } else if (elapsed < 4000) {
            if (elapsed < 2500) {
                Serial.println("[DELIVER] Releasing package...");
                gripper->open();
            }
        } else if (elapsed < 6000) {
            if (elapsed < 4500) {
                Serial.println("[DELIVER] Ascending...");
                mavlink->takeoff(CRUISE_ALTITUDE_CM);
            }
        } else {
            Serial.println("[DELIVER] Package delivered!");
            changeState(STATE_NAV_HOME);
        }
    }

    void handleNavHome() {
        if (!navInitialized) {
            navInitialized = true;
            Serial.println("[NAV] Planning route HOME...");
            planner->findPath(DELIVERY_POS.x, DELIVERY_POS.y,
                             HOME_POS.x, HOME_POS.y);
            planner->printMapWithPath();
        }

        navigatePath();

        if (planner->isPathComplete()) {
            Serial.println("[NAV] Arrived HOME!");
            changeState(STATE_LANDING);
        }

        if (irBeacon->getStation() == STATION_HOME) {
            Serial.println("[NAV] IR Beacon: HOME detected!");
            changeState(STATE_LANDING);
        }
    }

    void handleLanding() {
        unsigned long elapsed = stateElapsed();
        float altitude = ultra->getFilteredAltitude();

        if (elapsed < 1000) {
            Serial.println("[LAND] Descending to ground...");
            mavlink->setMode("LAND");
        }

        if (altitude < LANDING_COMPLETE_CM && elapsed > 3000) {
            Serial.println("[LAND] Touchdown! Disarming...");
            mavlink->disarm();
            changeState(STATE_COMPLETE);
        }

        if (stateElapsed() > 20000) {
            Serial.println("[LAND] Timeout — force disarm");
            mavlink->disarm();
            changeState(STATE_COMPLETE);
        }
    }

    void handleComplete() {
        if (stateElapsed() < 1000) {
            Serial.println();
            Serial.println("╔══════════════════════════════════╗");
            Serial.println("║   MISSION COMPLETE!              ║");
            Serial.println("║   Package delivered successfully ║");
            Serial.println("╚══════════════════════════════════╝");
            Serial.println();
            Serial.println("Press 'G' to start a new mission");
        }
    }

    void handleEmergency() {
        if (stateElapsed() < 500) {
            Serial.println();
            Serial.println("!!! EMERGENCY — LANDING NOW !!!");
            mavlink->setMode("LAND");
        }

        float altitude = ultra->getFilteredAltitude();
        if (altitude < LANDING_COMPLETE_CM && stateElapsed() > 3000) {
            mavlink->disarm();
            Serial.println("[EMERGENCY] Landed and disarmed.");
            currentState = STATE_IDLE;
        }
    }

    // ───── NAVIGATION HELPER ─────

    void navigatePath() {
        if (millis() - lastNavUpdate < 500) return;
        lastNavUpdate = millis();

        AvoidAction action = avoider->decide(lidar->getDistance(), lidar->isValid());

        if (action == ACT_STOP || action == ACT_REVERSE) {
            avoider->execute(action, mavlink);
            return;
        }

        if (!planner->isPathComplete()) {
            Position wp = planner->getNextWaypoint();

            float sensorData[6] = {
                (float)lidar->getDistance() / 100.0f,
                (float)lidar->getDistance() / 100.0f,
                (float)lidar->getDistance() / 100.0f,
                (float)(wp.x - planner->getCurrentPosition().x),
                (float)(wp.y - planner->getCurrentPosition().y),
                0.0f
            };
            int mlAction = ml->predict(sensorData, 6);

            Serial.printf("[NAV] Step %d/%d -> (%d,%d) [ML: %s]\n",
                planner->getCurrentStep() + 1,
                planner->getPathLength(),
                wp.x, wp.y,
                ml->getActionName(mlAction));

            if (action == ACT_CONTINUE) {
                planner->advanceStep();
            }
        }
    }

    // ───── EXTERNAL CONTROLS ─────

    void startMission() {
        if (currentState == STATE_IDLE || currentState == STATE_COMPLETE) {
            Serial.println("\n[MISSION] Starting autonomous mission!");
            navRetries = 0;
            changeState(STATE_PREARM);
        }
    }

    void abortMission() {
        Serial.println("\n[ABORT] Emergency stop!");
        changeState(STATE_EMERGENCY);
    }

    void forceMissionStart() {
        Serial.println("\n[MISSION] FORCE STARTING (Bypassing sensor checks)...");
        forceStart = true;
        changeState(STATE_PREARM);
    }

    void simulateStep() {
        switch (currentState) {
            case STATE_IDLE:        changeState(STATE_PREARM);       break;
            case STATE_PREARM:      changeState(STATE_TAKEOFF);      break;
            case STATE_TAKEOFF:     changeState(STATE_NAV_PICKUP);   break;
            case STATE_NAV_PICKUP:
                if (planner->isPathComplete()) changeState(STATE_PICKING_UP);
                else navigatePath();
                break;
            case STATE_PICKING_UP:  changeState(STATE_NAV_DELIVERY); break;
            case STATE_NAV_DELIVERY:
                if (planner->isPathComplete()) changeState(STATE_DROPPING_OFF);
                else navigatePath();
                break;
            case STATE_DROPPING_OFF: changeState(STATE_NAV_HOME);   break;
            case STATE_NAV_HOME:
                if (planner->isPathComplete()) changeState(STATE_LANDING);
                else navigatePath();
                break;
            case STATE_LANDING:     changeState(STATE_COMPLETE);     break;
            case STATE_COMPLETE:    changeState(STATE_IDLE);         break;
            default: break;
        }
    }

    MissionState getState()     { return currentState; }
    const char*  getStateName() { return STATE_NAMES[currentState]; }
};

#endif
