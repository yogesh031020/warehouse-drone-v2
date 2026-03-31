#ifndef MAVLINK_COMM_H
#define MAVLINK_COMM_H

#include <HardwareSerial.h>
#include <MAVLink_ardupilotmega.h>
#include "config.h"

// Required by MAVLink library for generating correct packet CRCs
mavlink_system_t mavlink_system;

// ============================================
//  MAVLink Communication — Real Binary Protocol
//  Talks to APM 2.8 via UART1 at 57600 baud
//
//  Wire:  ESP32 GPIO4 (RX) <-- APM TX  (use voltage divider: 5V→3.3V)
//         ESP32 GPIO2 (TX) --> APM RX
// ============================================

// APM Flight Mode numbers (ArduCopter)
#define APM_MODE_STABILIZE  0
#define APM_MODE_ALT_HOLD   2
#define APM_MODE_LOITER     5
#define APM_MODE_AUTO       3
#define APM_MODE_GUIDED     4
#define APM_MODE_LAND       9

class MavlinkComm {
private:
    HardwareSerial* mavSerial;
    unsigned long   lastHeartbeatSent;
    unsigned long   lastHeartbeatRecv;  // Last time APM heartbeat arrived
    uint8_t         apmMode;            // Current APM flight mode (from its heartbeat)
    bool            apmConnected;
    uint8_t         msgSeq;             // Rolling packet sequence counter
    bool            armAcked;           // True after APM ACKs the ARM command
    uint8_t         lastAckResult;      // Result code from last COMMAND_ACK
    bool            apmArmed;           // True when APM reports armed in heartbeat
    uint8_t         targetSysId;        // APM's system ID
    uint8_t         targetCompId;       // APM's component ID

    // ── Low-level send ──────────────────────────────────────────
    void sendPacket(mavlink_message_t* msg) {
        uint8_t buf[MAVLINK_MAX_PACKET_LEN];
        uint16_t len = mavlink_msg_to_send_buffer(buf, msg);
        mavSerial->write(buf, len);
    }

public:
    void init() {
        mavSerial         = &Serial1;
        mavSerial->begin(MAV_BAUD, SERIAL_8N1, MAV_RX_PIN, MAV_TX_PIN);
        
        // Critical: MAVLink library requires the global mavlink_system struct 
        // to be initialized, or it will generate invalid packet CRCs!
        mavlink_system.sysid  = GCS_SYS_ID;
        mavlink_system.compid = GCS_COMP_ID;

        lastHeartbeatSent = 0;
        lastHeartbeatRecv = 0;
        apmMode           = 255;
        apmConnected      = false;
        msgSeq            = 0;
        armAcked          = false;
        lastAckResult     = 255;
        apmArmed          = false;
        targetSysId       = APM_SYS_ID;
        targetCompId      = APM_COMP_ID;
        Serial.println("[MAV] MAVLink init OK (binary protocol)");
    }

    void resetArmAck() { armAcked = false; lastAckResult = 255; }

    // ── Arm / Disarm ────────────────────────────────────────────
    // force=false → normal arm, force=true → bypass all safety checks (param2=21196)
    void arm(bool force = false) {
        mavlink_message_t msg;
        mavlink_msg_command_long_pack(
            GCS_SYS_ID, GCS_COMP_ID, &msg,
            APM_SYS_ID, 1, // MAV_COMP_ID_AUTOPILOT1
            MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            1,                     // param1: 1 = ARM
            force ? 21196 : 0,     // param2: 21196 = force bypass safety
            0,0,0,0,0
        );
        sendPacket(&msg);
        Serial.println(force ? "[MAV] FORCE ARM sent" : "[MAV] ARM command sent");
    }

    void disarm() {
        mavlink_message_t msg;
        mavlink_msg_command_long_pack(
            GCS_SYS_ID, GCS_COMP_ID, &msg,
            APM_SYS_ID, APM_COMP_ID,
            MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            0,      // param1: 0 = DISARM
            0,0,0,0,0,0
        );
        sendPacket(&msg);
        Serial.println("[MAV] DISARM command sent");
    }

    // ── Parameter Set/Get ────────────────────────────────────────
    // Set any APM parameter by name (e.g. "ARMING_CHECK", 0)
    void setParam(const char* paramName, float value) {
        mavlink_message_t msg;
        char name[16] = {};
        strncpy(name, paramName, 16);
        mavlink_msg_param_set_pack(
            GCS_SYS_ID, GCS_COMP_ID, &msg,
            targetSysId, targetCompId,
            name,
            value,
            MAV_PARAM_TYPE_REAL32
        );
        sendPacket(&msg);
        Serial.printf("[MAV] PARAM_SET %s = %.0f\n", paramName, value);
    }

    // Request a parameter value (response arrives via parseIncoming)
    void requestParam(const char* paramName) {
        mavlink_message_t msg;
        char name[16] = {};
        strncpy(name, paramName, 16);
        mavlink_msg_param_request_read_pack(
            GCS_SYS_ID, GCS_COMP_ID, &msg,
            targetSysId, targetCompId,
            name, -1
        );
        sendPacket(&msg);
    }

    // Request full parameter list (forces APM to talk to us)
    void requestParamList() {
        mavlink_message_t msg;
        mavlink_msg_param_request_list_pack(
            GCS_SYS_ID, GCS_COMP_ID, &msg,
            targetSysId, targetCompId
        );
        sendPacket(&msg);
        Serial.printf("[MAV] Requested parameters from SysID: %d, CompID: %d\n", targetSysId, targetCompId);
    }

    // ── Flight Mode ─────────────────────────────────────────────
    // Pass one of the APM_MODE_* constants above
    void setMode(uint8_t mode) {
        mavlink_message_t msg;
        // Old APM firmware prefers the dedicated SET_MODE packet (ID 11) over COMMAND_LONG
        mavlink_msg_set_mode_pack(
            GCS_SYS_ID, GCS_COMP_ID, &msg,
            targetSysId,
            MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,  // base_mode (1)
            (uint32_t)mode                      // custom_mode
        );
        sendPacket(&msg);

        const char* names[] = {"STABILIZE","","ALT_HOLD","AUTO","GUIDED","LOITER",
                               "","","","LAND"};
        Serial.printf("[MAV] Mode → %s\n", (mode <= 9) ? names[mode] : "?");
    }

    // Convenience string version — maps to APM mode numbers
    void setMode(const char* modeName) {
        if      (strcmp(modeName, "STABILIZE") == 0) setMode((uint8_t)APM_MODE_STABILIZE);
        else if (strcmp(modeName, "ALT_HOLD")  == 0) setMode((uint8_t)APM_MODE_ALT_HOLD);
        else if (strcmp(modeName, "GUIDED")    == 0) setMode((uint8_t)APM_MODE_GUIDED);
        else if (strcmp(modeName, "LAND")      == 0) setMode((uint8_t)APM_MODE_LAND);
        else if (strcmp(modeName, "LOITER")    == 0) setMode((uint8_t)APM_MODE_LOITER);
        else Serial.printf("[MAV] Unknown mode: %s\n", modeName);
    }

    // ── Takeoff ─────────────────────────────────────────────────
    // altitude_cm: target altitude in centimetres
    void takeoff(int altitude_cm) {
        mavlink_message_t msg;
        mavlink_msg_command_long_pack(
            GCS_SYS_ID, GCS_COMP_ID, &msg,
            targetSysId, targetCompId,
            MAV_CMD_NAV_TAKEOFF,
            0,
            0, 0, 0, 0, 0, 0,   // params 1-6 unused
            (float)(altitude_cm / 100.0f)  // param7: altitude in METRES
        );
        sendPacket(&msg);
        Serial.printf("[MAV] TAKEOFF → %.1f m\n", altitude_cm / 100.0f);
    }

    // ── Descend to height ───────────────────────────────────────
    // Uses SET_POSITION_TARGET_LOCAL_NED (MSG 84) in Body-frame
    void moveDown(int target_alt_cm) {
        mavlink_message_t msg;
        // type_mask: only altitude (Z) valid — ignore X,Y,vel,accel,yaw
        uint16_t type_mask = 0b0000111111111000; // position only, Z active
        mavlink_msg_set_position_target_local_ned_pack(
            GCS_SYS_ID, GCS_COMP_ID, &msg,
            millis(),            // time_boot_ms
            targetSysId, targetCompId,
            MAV_FRAME_LOCAL_NED, // frame
            type_mask,
            0, 0,                // x, y (ignored)
            -(target_alt_cm / 100.0f), // z: negative = up in NED; positive = down
            0, 0, 0,             // vx, vy, vz (ignored)
            0, 0, 0,             // afx, afy, afz (ignored)
            0, 0                 // yaw, yaw_rate (ignored)
        );
        sendPacket(&msg);
        Serial.printf("[MAV] DESCEND_TO → %d cm\n", target_alt_cm);
    }

    // ── RC Override (MSG 70) ─────────────────────────────────────
    // Values: 1000–2000 µs. 1500 = neutral. UINT16_MAX = release channel
    void sendRC(int roll, int pitch, int throttle, int yaw) {
        mavlink_message_t msg;
        mavlink_msg_rc_channels_override_pack(
            GCS_SYS_ID, GCS_COMP_ID, &msg,
            targetSysId, targetCompId,
            (uint16_t)roll,      // CH1
            (uint16_t)pitch,     // CH2
            (uint16_t)throttle,  // CH3
            (uint16_t)yaw,       // CH4
            UINT16_MAX, UINT16_MAX, UINT16_MAX, UINT16_MAX, // CH5-CH8:  release to RC
            UINT16_MAX, UINT16_MAX, UINT16_MAX, UINT16_MAX, UINT16_MAX, UINT16_MAX, // CH9-CH14
            UINT16_MAX, UINT16_MAX, UINT16_MAX, UINT16_MAX  // CH15-CH18: release
        );
        sendPacket(&msg);
    }

    // ── Movement Helpers ─────────────────────────────────────────
    // speed: 0–400 µs offset from 1500 neutral
    void moveForward(int speed)  { sendRC(1500, 1500 - speed, 1500, 1500); }
    void moveBack(int speed)     { sendRC(1500, 1500 + speed, 1500, 1500); }
    void moveLeft(int speed)     { sendRC(1500 - speed, 1500, 1500, 1500); }
    void moveRight(int speed)    { sendRC(1500 + speed, 1500, 1500, 1500); }
    void ascend(int speed)       { sendRC(1500, 1500, 1500 + speed, 1500); }
    void descend(int speed)      { sendRC(1500, 1500, 1500 - speed, 1500); }
    void turnLeft(int speed)     { sendRC(1500, 1500, 1500, 1500 - speed); }
    void turnRight(int speed)    { sendRC(1500, 1500, 1500, 1500 + speed); }
    void hover()                 { sendRC(1500, 1500, 1500, 1500); }

    // ── Heartbeat & Incoming Parser ──────────────────────────────
    // Call this every loop() — sends GCS heartbeat + parses APM replies
    void heartbeat() {
        // 1) Send our GCS heartbeat every 1 second
        if (millis() - lastHeartbeatSent > MAV_HEARTBEAT_MS) {
            mavlink_message_t msg;
            mavlink_msg_heartbeat_pack(
                GCS_SYS_ID, GCS_COMP_ID, &msg,
                MAV_TYPE_GCS,               // We are a GCS
                MAV_AUTOPILOT_INVALID,      // Not an autopilot
                0, 0,                       // base_mode, custom_mode
                MAV_STATE_ACTIVE
            );
            sendPacket(&msg);
            lastHeartbeatSent = millis();
        }

        // 2) Parse everything APM has sent back
        parseIncoming();
    }

    // ── Parse Incoming MAVLink from APM ─────────────────────────
    void parseIncoming() {
        mavlink_message_t msg;
        mavlink_status_t  status;

        while (mavSerial->available()) {
            uint8_t byte = mavSerial->read();
            if (mavlink_parse_char(MAVLINK_COMM_0, byte, &msg, &status)) {
                
                // Only log truly unknown/interesting message IDs for debugging
                // Remove this block to silence all MAV IN messages

                switch (msg.msgid) {

                    case MAVLINK_MSG_ID_HEARTBEAT: {
                        mavlink_heartbeat_t hb;
                        mavlink_msg_heartbeat_decode(&msg, &hb);
                        
                        // Lock onto the first APM we see
                        if (!apmConnected && hb.type != MAV_TYPE_GCS) {
                            targetSysId = msg.sysid;
                            targetCompId = msg.compid;
                            Serial.printf("[MAV] Locked to APM SysID: %d, CompID: %d\n", targetSysId, targetCompId);
                        }

                        // Only care about our locked autopilot
                        if (msg.sysid == targetSysId) {
                            apmMode      = hb.custom_mode;
                            apmConnected = true;
                            lastHeartbeatRecv = millis();
                            // MAV_MODE_FLAG_SAFETY_ARMED = 128
                            apmArmed = (hb.base_mode & MAV_MODE_FLAG_SAFETY_ARMED) != 0;
                        }
                        break;
                    }

                    case MAVLINK_MSG_ID_COMMAND_ACK: {
                        mavlink_command_ack_t ack;
                        mavlink_msg_command_ack_decode(&msg, &ack);
                        lastAckResult = ack.result;
                        Serial.printf("[MAV] ACK cmd=%d result=%d %s\n",
                            ack.command, ack.result,
                            ack.result == MAV_RESULT_ACCEPTED ? "OK" : "FAIL");
                        // Track ARM confirmation specifically
                        if (ack.command == MAV_CMD_COMPONENT_ARM_DISARM) {
                            if (ack.result == MAV_RESULT_ACCEPTED) {
                                armAcked = true;
                                apmArmed = true;
                                Serial.println("[MAV] ARMED confirmed by APM!");
                            } else {
                                Serial.printf("[MAV] ARM REFUSED (result=%d) — check pre-arm\n", ack.result);
                            }
                        }
                        break;
                    }

                    case MAVLINK_MSG_ID_PARAM_VALUE: {
                        mavlink_param_value_t pv;
                        mavlink_msg_param_value_decode(&msg, &pv);
                        char pname[17] = {};
                        strncpy(pname, pv.param_id, 16);
                        Serial.printf("[MAV] PARAM %s = %.0f\n", pname, pv.param_value);
                        break;
                    }

                    case MAVLINK_MSG_ID_SYS_STATUS: {
                        // Could read battery voltage here in future
                        break;
                    }

                    case MAVLINK_MSG_ID_STATUSTEXT: {
                        // APM text messages (pre-arm warnings, mode changes, etc.)
                        mavlink_statustext_t st;
                        mavlink_msg_statustext_decode(&msg, &st);
                        char text[51] = {};
                        strncpy(text, st.text, 50);
                        Serial.printf("[APM] %s\n", text);
                        break;
                    }

                    default:
                        break;
                }
            }
        }

        // Mark disconnected if no heartbeat for 3 seconds
        if (apmConnected && millis() - lastHeartbeatRecv > 3000) {
            apmConnected = false;
            Serial.println("[MAV] WARNING: APM heartbeat lost!");
        }
    }

    // ── Status Queries ───────────────────────────────────────────
    bool    isConnected()      { return apmConnected; }
    bool    isArmed()          { return apmArmed; }
    bool    isArmAcked()       { return armAcked; }
    uint8_t getLastAckResult() { return lastAckResult; }
    uint8_t getAPMMode()       { return apmMode; }

    const char* getAPMModeName() {
        const char* names[] = {
            "STABILIZE","ACRO","ALT_HOLD","AUTO",
            "GUIDED","LOITER","RTL","CIRCLE",
            "POSITION","LAND","OFLOITER","DRIFT"
        };
        return (apmMode < 12) ? names[apmMode] : "UNKNOWN";
    }
};

#endif
