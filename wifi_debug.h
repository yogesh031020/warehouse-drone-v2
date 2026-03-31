// ============================================
//  WAREHOUSE DRONE — WiFi Debug Module
//
//  Features:
//  1. OTA (Over-the-Air) code upload via Arduino IDE
//  2. TCP Serial Monitor via any Telnet client
//     or browser at http://<drone-ip>
//
//  How to use:
//  - All Serial.print() calls are AUTOMATICALLY
//    mirrored over WiFi via the WifiDebug class.
//  - OTA upload works from Arduino IDE:
//    Tools > Port > "WarehouseDrone at <IP>"
// ============================================
#ifndef WIFI_DEBUG_H
#define WIFI_DEBUG_H

#include <WiFi.h>
#include <ArduinoOTA.h>
#include <WebServer.h>

// ──────────────────────────────────────────
//  PUT YOUR WIFI CREDENTIALS HERE
// ──────────────────────────────────────────
#define WIFI_SSID     "NOVATECH1"
#define WIFI_PASSWORD "123456789@"
// ──────────────────────────────────────────

#define WIFI_DEBUG_PORT  23     // Telnet port
#define WIFI_HTTP_PORT   80     // Web page port
#define WIFI_CONNECT_TIMEOUT_MS 10000

// Simple circular log buffer — stores last N bytes
#define LOG_BUFFER_SIZE 8192
static char     logBuffer[LOG_BUFFER_SIZE];
static uint16_t logHead = 0;
static uint16_t logLen  = 0;

static void logWriteLogBuffer(const uint8_t* data, size_t len) {
    for (size_t i = 0; i < len; i++) {
        logBuffer[logHead] = data[i];
        logHead = (logHead + 1) % LOG_BUFFER_SIZE;
        if (logLen < LOG_BUFFER_SIZE) logLen++;
    }
}

// ──────────────────────────────────────────
//  WiFiDebug Class
// ──────────────────────────────────────────
class WiFiDebug {
private:
    WiFiServer   tcpServer;
    WebServer    httpServer;
    WiFiClient   tcpClient;
    bool         wifiOk     = false;
    bool         otaReady   = false;
    unsigned long lastHeartbeat = 0;
    char         pendingCmd = 0;
    String       telemetryData = "Waiting for telemetry...";

    // HTML page to view log in browser
    const char* HTML_HEAD = R"=====(
<!DOCTYPE html><html><head>
<meta charset="UTF-8">
<meta http-equiv="refresh" content="2">
<title>Drone Monitor</title>
<style>
  body{background:#111;color:#0f0;font-family:monospace;padding:10px;}
  h1{color:#0af; margin-bottom: 5px;}
  pre{white-space:pre-wrap;word-break:break-all;font-size:14px;}
  .telemetry-box {
      border: 2px solid #0af;
      padding: 10px;
      margin-bottom: 15px;
      background: #000;
      color: #0ff;
      display: inline-block;
  }
  .log-box {
      border: 1px dashed #555;
      padding: 10px;
      color: #aaa;
      height: 400px;
      overflow-y: scroll;
  }
  .cmdbox {margin-top: 10px; margin-bottom: 15px;}
  input, button {background:#222;color:#0f0;border:1px solid #0f0;padding:5px;font-size:16px;}
</style>
<script>
function sendCmd() {
  var v = document.getElementById('c').value;
  if(v.length>0) {
    fetch('/cmd?c=' + v[0].toUpperCase());
    document.getElementById('c').value = '';
  }
}
function copyLog() {
  var logText = document.getElementById('log').innerText;
  var textArea = document.createElement("textarea");
  textArea.value = logText;
  document.body.appendChild(textArea);
  textArea.select();
  try {
    document.execCommand('copy');
    var btn = document.getElementById('copyBtn');
    btn.innerHTML = 'Copied!';
    setTimeout(function(){ btn.innerHTML = 'Copy Log'; }, 2000);
  } catch (err) {
    alert('Oops, unable to copy');
  }
  document.body.removeChild(textArea);
}
</script>
</head><body>
<h1>&#x1F681; Warehouse Drone Dashboard</h1>
<div class="cmdbox">
  Send Command: &nbsp;<input type="text" id="c" maxlength="1" onkeydown="if(event.key==='Enter')sendCmd()">
  <button onclick="sendCmd()">Execute</button>
  &nbsp;&nbsp;&nbsp;
  <button id="copyBtn" onclick="copyLog()">Copy Log</button>
</div>
<div class="telemetry-box"><pre id="telemetry">)=====";

    const char* HTML_MID = R"=====(</pre></div>
<h3>System Log</h3>
<div class="log-box"><pre id="log">)=====";

    const char* HTML_END = R"=====(</pre></div></body></html>)=====";

    void buildLog(String& out) {
        // Reconstruct ordered log from circular buffer
        out.reserve(logLen + 1);
        if (logLen < LOG_BUFFER_SIZE) {
            out.concat(logBuffer, logLen);
        } else {
            // Buffer is full, read from head
            uint16_t start = logHead;
            for (uint16_t i = 0; i < LOG_BUFFER_SIZE; i++) {
                out += logBuffer[(start + i) % LOG_BUFFER_SIZE];
            }
        }
    }

public:
    WiFiDebug() : tcpServer(WIFI_DEBUG_PORT), httpServer(WIFI_HTTP_PORT) {}

    void begin() {
        Serial.print("\n[WIFI] Connecting to ");
        Serial.print(WIFI_SSID);
        Serial.print("...");

        WiFi.mode(WIFI_STA);
        WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

        unsigned long start = millis();
        while (WiFi.status() != WL_CONNECTED &&
               millis() - start < WIFI_CONNECT_TIMEOUT_MS) {
            delay(250);
            Serial.print(".");
        }

        if (WiFi.status() != WL_CONNECTED) {
            Serial.println("\n[WIFI] FAILED — running without WiFi.");
            return;
        }

        wifiOk = true;
        Serial.println("\n[WIFI] Connected!");
        Serial.print("[WIFI] IP Address: ");
        Serial.println(WiFi.localIP());
        Serial.println("[WIFI] Open http://" + WiFi.localIP().toString() + " to view log");
        Serial.println("[WIFI] Telnet to port 23 for raw TCP monitor");

        // ── OTA Setup ──────────────────────────
        ArduinoOTA.setHostname("WarehouseDrone");
        // ArduinoOTA.setPassword("drone1234");  // Commented out to fix authentication errors

        ArduinoOTA.onStart([]() {
            Serial.println("\n[OTA] Upload started — pausing mission...");
        });
        ArduinoOTA.onEnd([]() {
            Serial.println("\n[OTA] Upload complete! Rebooting...");
        });
        ArduinoOTA.onProgress([](unsigned int prog, unsigned int total) {
            Serial.printf("[OTA] Progress: %u%%\r", (prog * 100) / total);
        });
        ArduinoOTA.onError([](ota_error_t error) {
            Serial.printf("[OTA] Error[%u]: ", error);
            if      (error == OTA_AUTH_ERROR)    Serial.println("Auth Failed");
            else if (error == OTA_BEGIN_ERROR)   Serial.println("Begin Failed");
            else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
            else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
            else if (error == OTA_END_ERROR)     Serial.println("End Failed");
        });
        ArduinoOTA.begin();
        otaReady = true;

        // ── TCP Telnet Server ───────────────────
        tcpServer.begin();

        // ── HTTP Web Server ─────────────────────
        httpServer.on("/", [this]() {
            String page = HTML_HEAD;
            page += telemetryData;
            page += HTML_MID;

            String log;
            buildLog(log);
            // Escape HTML entities for the log box
            log.replace("&", "&amp;");
            log.replace("<", "&lt;");
            log.replace(">", "&gt;");

            page += log;
            page += HTML_END;
            httpServer.send(200, "text/html", page);
        });
        httpServer.on("/raw", [this]() {
            String log;
            buildLog(log);
            httpServer.send(200, "text/plain", log);
        });
        httpServer.on("/cmd", [this]() {
            if (httpServer.hasArg("c")) {
                pendingCmd = httpServer.arg("c").charAt(0);
                Serial.print("\n[WIFI] Received Web Command: ");
                Serial.println(String(pendingCmd).c_str());
            }
            httpServer.send(200, "text/plain", "OK");
        });
        httpServer.begin();

        Serial.println("[WIFI] All services started OK");
    }

    // Call this EVERY loop() — handles OTA and new clients
    void update() {
        if (!wifiOk) return;
        if (otaReady) ArduinoOTA.handle();
        httpServer.handleClient();

        // Accept new Telnet clients
        if (!tcpClient || !tcpClient.connected()) {
            tcpClient = tcpServer.accept();
            if (tcpClient) {
                tcpClient.println("\r\n=== Warehouse Drone Serial Monitor ===\r\n");
            }
        }

        // Forward incoming Telnet chars as serial commands
        if (tcpClient && tcpClient.available()) {
            while (tcpClient.available()) {
                char c = tcpClient.read();
                // Push to hardware serial as if user typed it
                // We store it and let the main loop pick it up
                Serial.print(c); // Echo back
            }
        }
    }

    // Mirror raw bytes to both USB serial and WiFi clients
    size_t write(uint8_t c) {
        logWriteLogBuffer(&c, 1);
        if (wifiOk && tcpClient && tcpClient.connected()) {
            tcpClient.write(c);
        }
        return 1;
    }

    size_t write(const uint8_t *buffer, size_t size) {
        logWriteLogBuffer(buffer, size);
        if (wifiOk && tcpClient && tcpClient.connected()) {
            tcpClient.write(buffer, size);
        }
        return size;
    }

    bool isConnected() { return wifiOk && WiFi.status() == WL_CONNECTED; }

    String getIP() {
        return wifiOk ? WiFi.localIP().toString() : "Not connected";
    }

    char getPendingCmd() {
        char c = pendingCmd;
        pendingCmd = 0;
        return c;
    }

    // Pass the formatted dashboard string to the Web UI
    void setTelemetry(const String& data) {
        telemetryData = data;
    }

    // Used by main loop to forward incoming Telnet commands
    WiFiClient& getClient() { return tcpClient; }
};

static WiFiDebug wifiDebug;

// ──────────────────────────────────────────
//  Serial Wrapper to intercept ALL prints
// ──────────────────────────────────────────

// Save the real hardware serial before we override it
#define HW_SERIAL Serial

class DualSerial : public Print {
public:
    void begin(unsigned long baud) {
        HW_SERIAL.begin(baud);
    }
    
    int available() {
        return HW_SERIAL.available();
    }
    
    int read() {
        return HW_SERIAL.read();
    }

    virtual size_t write(uint8_t c) override {
        HW_SERIAL.write(c);
        wifiDebug.write(c);
        return 1;
    }
    
    virtual size_t write(const uint8_t *buffer, size_t size) override {
        HW_SERIAL.write(buffer, size);
        wifiDebug.write(buffer, size);
        return size;
    }
};

static DualSerial DualSerialObj;

// Remap Serial to our DualSerial interceptor
#define Serial DualSerialObj

#endif
