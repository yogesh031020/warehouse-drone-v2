#ifndef IR_BEACON_H
#define IR_BEACON_H

#include <IRremote.h>
#include "config.h"

// Unique IR codes for each station
// Program these into IR LED transmitters at each station
#define PICKUP_STATION_CODE    0xAA01
#define DELIVERY_STATION_CODE  0xBB02
#define HOME_STATION_CODE      0xCC03

enum StationType {
    STATION_NONE,
    STATION_PICKUP,
    STATION_DELIVERY,
    STATION_HOME
};

class IRBeacon {
private:
    StationType detectedStation;
    unsigned long lastDetectTime;

public:
    void init() {
        IrReceiver.begin(IR_RECV_PIN);
        detectedStation = STATION_NONE;
        lastDetectTime = 0;
    }

    void update() {
        if (IrReceiver.decode()) {
            uint32_t code = IrReceiver.decodedIRData.decodedRawData;
            lastDetectTime = millis();

            switch (code) {
                case PICKUP_STATION_CODE:
                    detectedStation = STATION_PICKUP;
                    break;
                case DELIVERY_STATION_CODE:
                    detectedStation = STATION_DELIVERY;
                    break;
                case HOME_STATION_CODE:
                    detectedStation = STATION_HOME;
                    break;
                default:
                    // Any other IR code detected (e.g. TV remote)
                    // Useful for testing — treat as pickup station
                    detectedStation = STATION_PICKUP;
                    break;
            }
            IrReceiver.resume();
        }

        // Station signal timeout
        if (millis() - lastDetectTime > STATION_TIMEOUT_MS) {
            detectedStation = STATION_NONE;
        }
    }

    StationType getStation()    { return detectedStation; }
    bool isStationDetected()    { return detectedStation != STATION_NONE; }

    const char* getStationName() {
        switch (detectedStation) {
            case STATION_PICKUP:   return "PICKUP";
            case STATION_DELIVERY: return "DELIVERY";
            case STATION_HOME:     return "HOME";
            default:               return "NONE";
        }
    }
};

#endif
