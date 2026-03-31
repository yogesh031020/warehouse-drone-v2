#ifndef GRIPPER_H
#define GRIPPER_H

#include <ESP32Servo.h>
#include "config.h"

class Gripper {
private:
    Servo servo;
    bool isGripping;

public:
    void init() {
        servo.attach(SERVO_PIN);
        open();
    }

    void open() {
        servo.write(GRIPPER_OPEN);
        isGripping = false;
        Serial.println("[GRIP] Opened");
    }

    void close() {
        servo.write(GRIPPER_CLOSED);
        isGripping = true;
        Serial.println("[GRIP] Closed - Package grabbed");
    }

    bool hasPackage() { return isGripping; }
};

#endif
