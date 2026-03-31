#ifndef OBSTACLE_AVOID_H
#define OBSTACLE_AVOID_H

#include "config.h"
#include "mavlink_comm.h"

// ============================================
//  OBSTACLE AVOIDANCE
// ============================================
//
//  This runs ON TOP of the path planner.
//  A* plans the route, but if the drone sees
//  something unexpected (person, box, etc.),
//  this kicks in to avoid a crash.
//
//  Priority: Obstacle Avoidance > Path Following
//

enum AvoidAction {
    ACT_CONTINUE,       // No obstacle — keep following path
    ACT_SLOW_DOWN,      // Object detected far away — reduce speed
    ACT_STOP,           // Object close — hover in place
    ACT_REVERSE         // Object very close — back up!
};

class ObstacleAvoider {
private:
    AvoidAction lastAction;

public:
    void init() {
        lastAction = ACT_CONTINUE;
    }

    // Decide what to do based on LiDAR reading
    AvoidAction decide(int lidarDist, bool isValid) {
        // Sensor error = be cautious
        if (!isValid) return ACT_SLOW_DOWN;

        if (lidarDist < 30) {
            return ACT_REVERSE;           // Too close! Back up!
        } else if (lidarDist < OBSTACLE_CLOSE_CM) {
            return ACT_STOP;              // Stop and hover
        } else if (lidarDist < OBSTACLE_WARN_CM) {
            return ACT_SLOW_DOWN;         // Approaching something
        }
        return ACT_CONTINUE;              // All clear
    }

    // Execute the avoidance action
    void execute(AvoidAction action, MavlinkComm* mav) {
        switch (action) {
            case ACT_CONTINUE:
                // Do nothing — path planner controls movement
                break;

            case ACT_SLOW_DOWN:
                if (lastAction != ACT_SLOW_DOWN) {
                    Serial.println("[AVOID] Slowing down...");
                }
                break;

            case ACT_STOP:
                mav->hover();
                if (lastAction != ACT_STOP) {
                    Serial.println("[AVOID] !! HOVERING - Obstacle ahead !!");
                }
                break;

            case ACT_REVERSE:
                mav->moveBack(150);
                if (lastAction != ACT_REVERSE) {
                    Serial.println("[AVOID] !! REVERSING - Too close !!");
                }
                break;
        }
        lastAction = action;
    }

    // Get action name for display
    const char* getActionName(AvoidAction action) {
        switch (action) {
            case ACT_CONTINUE:  return "OK";
            case ACT_SLOW_DOWN: return "SLOW";
            case ACT_STOP:      return "STOP!";
            case ACT_REVERSE:   return "REVERSE!";
            default:            return "??";
        }
    }
};

#endif
