#ifndef ULTRASONIC_H
#define ULTRASONIC_H

#include "config.h"

class UltrasonicSensor {
private:
    float altitude_cm;
    float filtered_alt;
    float alpha;

public:
    void init() {
        pinMode(ULTRA_TRIG_PIN, OUTPUT);
        pinMode(ULTRA_ECHO_PIN, INPUT);
        altitude_cm = 0;
        filtered_alt = 0;
        alpha = 0.3;  // Low-pass filter (lower = smoother)
    }

    void update() {
        // Send trigger pulse
        digitalWrite(ULTRA_TRIG_PIN, LOW);
        delayMicroseconds(2);
        digitalWrite(ULTRA_TRIG_PIN, HIGH);
        delayMicroseconds(10);
        digitalWrite(ULTRA_TRIG_PIN, LOW);

        // Read echo (timeout 30ms = ~5m max range)
        long duration = pulseIn(ULTRA_ECHO_PIN, HIGH, 30000);

        if (duration > 0) {
            altitude_cm = duration * 0.034 / 2.0;

            if (altitude_cm > 0 && altitude_cm < 400) {
                filtered_alt = alpha * altitude_cm + (1 - alpha) * filtered_alt;
            }
        }
    }

    float getRawAltitude()      { return altitude_cm; }
    float getFilteredAltitude() { return filtered_alt; }
    bool  isAtPickupHeight()    { return filtered_alt > 0 && filtered_alt < PICKUP_ALTITUDE_CM; }
    bool  isLanded()            { return filtered_alt > 0 && filtered_alt < LANDING_COMPLETE_CM; }
    bool  isAtCruiseAlt()       { return filtered_alt >= CRUISE_ALTITUDE_CM; }
};

#endif
