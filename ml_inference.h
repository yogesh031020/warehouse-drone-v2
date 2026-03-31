#ifndef ML_INFERENCE_H
#define ML_INFERENCE_H

#include "config.h"
#include "model_data.h"

// ============================================
//  ML INFERENCE ENGINE
// ============================================
//
//  This runs the trained TFLite model on ESP32.
//
//  If no real model is loaded (model_data_len == 0),
//  it falls back to simple rule-based decisions.
//
//  Actions:
//    0 = Forward
//    1 = Turn Left
//    2 = Turn Right
//    3 = Backward
//    4 = Hover
//

// Action names for debug printing
const char* ACTION_NAMES[] = {"FORWARD", "LEFT", "RIGHT", "BACK", "HOVER"};

class MLInference {
private:
    bool modelLoaded;

public:
    bool init() {
        if (model_data_len > 0) {
            // Real TFLite model available
            // NOTE: Full TFLite Micro integration requires:
            //   1. Install "TensorFlowLite_ESP32" library
            //   2. Uncomment TFLite code in predict()
            //   3. Run export_tflite.py to generate model_data.h
            modelLoaded = true;
            Serial.printf("[ML] Model loaded (%d bytes)\n", model_data_len);
        } else {
            modelLoaded = false;
            Serial.println("[ML] No model - using rule-based fallback");
        }
        return true;
    }

    // Predict best action from sensor data
    // Input: [lidar_front, lidar_left, lidar_right, target_dx, target_dy, dist]
    // Returns: action index 0-4
    int predict(float sensorData[], int len) {
        if (modelLoaded && model_data_len > 0) {
            // ---- TFLite Micro Inference ----
            // Uncomment this when you have TFLite library installed:
            //
            // for (int i = 0; i < len && i < 6; i++) {
            //     input->data.f[i] = sensorData[i];
            // }
            // interpreter->Invoke();
            //
            // int bestAction = 0;
            // float bestProb = output->data.f[0];
            // for (int i = 1; i < 5; i++) {
            //     if (output->data.f[i] > bestProb) {
            //         bestProb = output->data.f[i];
            //         bestAction = i;
            //     }
            // }
            // return bestAction;

            // For now, use rule-based (same as below)
            return ruleBased(sensorData);
        } else {
            return ruleBased(sensorData);
        }
    }

    // Rule-based fallback (works without ML model)
    int ruleBased(float sensorData[]) {
        float lidarFront = sensorData[0];
        float lidarLeft  = sensorData[1];
        float lidarRight = sensorData[2];

        // RULE 1: Too close ahead — turn or back up
        if (lidarFront < 0.5) {
            Serial.println("[ML] Rule: TOO CLOSE -> BACK");
            return 3;  // Backward
        }

        // RULE 2: Obstacle ahead — turn toward more open side
        if (lidarFront < 1.5) {
            if (lidarLeft > lidarRight) {
                Serial.println("[ML] Rule: OBSTACLE -> LEFT");
                return 1;
            } else {
                Serial.println("[ML] Rule: OBSTACLE -> RIGHT");
                return 2;
            }
        }

        // RULE 3: Warning distance — slow down / slight turn
        if (lidarFront < 3.0) {
            if (lidarLeft > lidarRight + 1.0) {
                return 1;  // Veer left
            } else if (lidarRight > lidarLeft + 1.0) {
                return 2;  // Veer right
            }
        }

        // RULE 4: Path clear — go forward
        return 0;
    }

    const char* getActionName(int action) {
        if (action >= 0 && action < 5) return ACTION_NAMES[action];
        return "UNKNOWN";
    }

    bool isModelLoaded() { return modelLoaded; }
};

#endif
