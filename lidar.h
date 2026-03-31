#ifndef LIDAR_H
#define LIDAR_H

#include <HardwareSerial.h>
#include "config.h"

class LidarSensor {
private:
    HardwareSerial* serial;
    int distance_cm;
    int strength;
    bool valid;

public:
    void init() {
        serial = &Serial2;
        serial->begin(LIDAR_BAUD, SERIAL_8N1, LIDAR_RX_PIN, LIDAR_TX_PIN);
        distance_cm = 0;
        strength = 0;
        valid = false;
    }

    // Call every loop — non-blocking
    void update() {
        while (serial->available() >= 9) {
            if (serial->read() == 0x59) {
                if (serial->peek() == 0x59) {
                    serial->read(); // consume second 0x59
                    uint8_t buf[7];
                    serial->readBytes(buf, 7);

                    distance_cm = buf[0] + buf[1] * 256;
                    strength    = buf[2] + buf[3] * 256;
                    // TF-Mini uses 65535 as an error code. Valid range is 1cm to 1200cm.
                    valid = (strength > 100 && distance_cm > 0 && distance_cm < 1200);
                }
            }
        }
    }

    int  getDistance()       { return distance_cm; }
    int  getStrength()      { return strength; }
    bool isValid()          { return valid; }
    bool isObstacleClose()  { return valid && distance_cm < OBSTACLE_CLOSE_CM; }
    bool isObstacleWarning(){ return valid && distance_cm < OBSTACLE_WARN_CM; }

    void printDiag() {
        Serial.println("\n===== TF-Mini LiDAR Diagnostics =====");
        Serial.printf("  Distance : %d cm\n", distance_cm);
        Serial.printf("  Strength : %d\n", strength);
        Serial.printf("  Valid    : %s\n", valid ? "YES" : "NO");
        Serial.printf("  Bytes waiting: %d\n", serial->available());
        if (!valid) {
            if (distance_cm == 65535 || distance_cm == 0)
                Serial.println("  FAULT: Sensor returns error code. Check 5V power.");
            else if (distance_cm > 1200)
                Serial.println("  FAULT: Out of range (>12m). Point LiDAR at an object.");
            else if (strength <= 100)
                Serial.println("  FAULT: Signal too weak. Check wiring: LiDAR TX -> GPIO16.");
        }
        Serial.println("=====================================");
    }
};

#endif

