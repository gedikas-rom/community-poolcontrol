/**
 * EspnowHandler.h
 *
 * Extension for the Poolcontrol code (ESP32 #1). Include in main.cpp:
 *
 *   #include "EspnowHandler.h"
 *
 *   // In setup():
 *   setupEspnowHandler();
 *
 *   // In loop():
 *   loopEspnowHandler();
 *
 * What this handler does:
 *
 *   Incoming (Bridge -> Poolcontrol):
 *     Reads uart_message bytes from Serial1 and parses the contained sensor_message.
 *     - Regular data messages (e.g. id="pool/temp") -> MQTT publish
 *     - Requests          (id="get-values")         -> Build response
 *                                                     and send back via UART
 *
 *   Outgoing (Poolcontrol -> Bridge):
 *     Send uart_message with target MAC + response sensor_message over Serial1.
 *     The bridge forwards sensor_message to the node via ESP-NOW.
 *
 * Requirements in the existing Poolcontrol code:
 *   - PubSubClient mqttClient (already present)
 *   - float getAverageTempWater()          These getter functions provide current
 *   - float getAverageTempAir()            pool measurement values from your code.
 *   - float getTargetTemp()                Adjust function names/signatures
 *   - Mode getMode()                       to your implementation if needed.
 *   - ValveState getCurrentValveState()
 *   - int   getCurrentPumpState()
 */

#include <Arduino.h>
#include <ArduinoJson.h>
#include <GlobalDefs.h>
#include "sensor_message.h"

// Configuration
#define ESPNOW_SERIAL       Serial1
#define ESPNOW_SERIAL_BAUD  115200
#define ESPNOW_RX_PIN       16
#define ESPNOW_TX_PIN       17

// Getter functions from the existing Poolcontrol code.
// Adjust signatures here if they differ:
extern void publishPavilionSensorData(float value, int battery, const char* firmware);
extern void publishGreenhouseSensorData(float value, int battery, const char* firmware);
extern void publishBridgeFirmware(const char* firmware);
extern void targetTempChangedFunction(float targetTemp);
extern void deltaTempChangedFunction(float deltaTemp);
extern void offsetWaterChangedFunction(float offsetWater);
extern void offsetAirChangedFunction(float offsetAir);
extern void modeChangedFunction(Mode mode);

extern float getAverageTempWater();
extern float getAverageTempAir();
extern float getTargetTemp();
extern Mode getMode();
extern ValveState getCurrentValveState();
extern int getCurrentPumpState();

// ─────────────────────────────────────────────────────────────────────────────

static uint8_t _pc_rxBuf[sizeof(uart_message)];
static size_t  _pc_rxLen = 0;

// Send response to a node (Poolcontrol -> Bridge -> Node)
static void sendToNode(const uint8_t *mac, const char *id, const char *payload) {
    uart_message umsg;
    memcpy(umsg.mac, mac, 6);

    strncpy(umsg.msg.id, id, sizeof(umsg.msg.id) - 1);
    umsg.msg.id[sizeof(umsg.msg.id) - 1] = '\0';

    strncpy(umsg.msg.payload, payload, sizeof(umsg.msg.payload) - 1);
    umsg.msg.payload[sizeof(umsg.msg.payload) - 1] = '\0';

    ESPNOW_SERIAL.write((uint8_t *)&umsg, sizeof(uart_message));
    ESPNOW_SERIAL.write('\n');

    Serial.printf("[ESPNOW] Response sent -> MAC=%02X:%02X:%02X:%02X:%02X:%02X"
                  "  id='%s' payload='%s'\n",
                  mac[0], mac[1], mac[2], mac[3], mac[4], mac[5], id, payload);
}

// get-values: evaluate request and respond
//
// Node sends:
//   id      = "get-values"
//   payload = {"req":["averageTempWater","averageTempAir"]}   <- requested values
//          or empty -> send all values (currently without payload parsing, idea for later)
//
// Poolcontrol responds with:
//   id      = "values"
//   payload = {"averageTempWater":24.7,"averageTempAir":38.2}
//
static void handleGetValues(const uint8_t *mac, const sensor_message &msg) {
/* 
    Feature (tbd): Payload optionally contains JSON with a "req" array, e.g. {"req":["water_temp","solar_temp"]}.
             It contains value names requested by the node. If no "req" or empty array, send all values.
*/ 
    JsonDocument req;
    bool hasReqList = false;

    if (strlen(msg.payload) > 0) {
        DeserializationError err = deserializeJson(req, msg.payload);
        hasReqList = (!err && req["req"].is<JsonArray>());
    }

    // Helper lambda: checks whether a key was requested (or if all are requested)
    auto isRequested = [&](const char *key) -> bool {
        if (!hasReqList) return true;  // no filter -> send everything
        for (JsonVariant v : req["req"].as<JsonArray>()) {
            if (strcmp(v.as<const char *>(), key) == 0) return true;
        }
        return false;
    };

    // Build response JSON
    JsonDocument resp;
    if (isRequested("averageTempWater"))    resp["averageTempWater"]  = serialized(String(getAverageTempWater(), 1));
    if (isRequested("averageTempAir"))      resp["averageTempAir"]  = serialized(String(getAverageTempAir(), 1));
    if (isRequested("targetTemp"))          resp["targetTemp"]      = serialized(String(getTargetTemp(), 1));
    if (isRequested("mode"))                resp["mode"]            = getMode();
    if (isRequested("valveState"))          resp["valveState"]      = getCurrentValveState();
    if (isRequested("pumpState"))           resp["pumpState"]       = getCurrentPumpState();

    struct tm timeinfo;
    if (getLocalTime(&timeinfo)) {
        char lastUpdate[20];  // "YYYY-MM-DD HH:MM:SS"
        strftime(lastUpdate, sizeof(lastUpdate), "%Y-%m-%d %H:%M:%S", &timeinfo);
        resp["lastUpdate"] = lastUpdate;
    } else {
        Serial.println("Failed to obtain time");
    }

    char respPayload[180];
    serializeJson(resp, respPayload, sizeof(respPayload));

    sendToNode(mac, "values", respPayload);
}

// Dispatcher: routes incoming messages
static void dispatchMessage(const uint8_t *mac, const sensor_message &msg) {
    if (strcmp(msg.id, "get-values") == 0) {
        // Request from node -> assemble values and send back
        handleGetValues(mac, msg);
        return;
    } else if (strcmp(msg.id, "bridge/hello") == 0) {
        // Bridge startup handshake: confirm that Poolcontrol UART handler is ready
        sendToNode(mac, "bridge/ready", "");
        return;
    }

    JsonDocument doc;
	DeserializationError error = deserializeJson(doc, msg.payload);
	if(error) { Serial.println("[ESPNOW] Error receiving data. Bad response"); return; }

    if (strcmp(msg.id, "set-mode") == 0) {
        String mode = doc["mode"];
        if (mode == "ON")
        {
            modeChangedFunction(ON);
        } else if (mode == "OFF")
        {
            modeChangedFunction(OFF);
        } else if (mode == "CLEANING")
        {
            modeChangedFunction(CLEANING);
        } else if (mode == "AUTO")
        {
            modeChangedFunction(AUTO);
        }
    } else if (strcmp(msg.id, "set-targettemp") == 0) {
        targetTempChangedFunction((float)doc["targetTemp"]);
    } else if (strcmp(msg.id, "set-deltatemp") == 0) {
        deltaTempChangedFunction((float)doc["deltaTemp"]);
    } else if (strcmp(msg.id, "set-offsetwater") == 0) {
        offsetWaterChangedFunction((float)doc["offsetWater"]);
    } else if (strcmp(msg.id, "set-offsetair") == 0) {
        offsetAirChangedFunction((float)doc["offsetAir"]);
    } else if (strcmp(msg.id, "set-sensordata") == 0)
    {
        if (doc["sensor"] == "temperature")
        {
            if (doc["location"] == "pavilion")
            {
                publishPavilionSensorData((float)doc["value"], (int)doc["battery"], (const char*)doc["firmware"]);
            }
            else if (doc["location"] == "greenhouse")
            {
                publishGreenhouseSensorData((float)doc["value"], (int)doc["battery"], (const char*)doc["firmware"]);
            }
        } else if (doc["sensor"] == "bridge") {
            publishBridgeFirmware((const char*)doc["firmware"]);
        }
        if (doc["return"] == "get-values")
        {
            handleGetValues(mac, msg);
        }
    }
}

// Read UART and process frames
void loopEspnowHandler() {
    while (ESPNOW_SERIAL.available()) {
        uint8_t b = (uint8_t)ESPNOW_SERIAL.read();

        if (b == '\n') {
            if (_pc_rxLen == sizeof(uart_message)) {
                uart_message umsg;
                memcpy(&umsg, _pc_rxBuf, sizeof(uart_message));
                umsg.msg.id[sizeof(umsg.msg.id) - 1]           = '\0';
                umsg.msg.payload[sizeof(umsg.msg.payload) - 1] = '\0';

                dispatchMessage(umsg.mac, umsg.msg);

            } else if (_pc_rxLen > 0) {
                Serial.printf("[ESPNOW] Invalid UART length: %d (expected %d)\n",
                              _pc_rxLen, sizeof(uart_message));
            }
            _pc_rxLen = 0;

        } else {
            if (_pc_rxLen < sizeof(_pc_rxBuf)) {
                _pc_rxBuf[_pc_rxLen++] = b;
            } else {
                Serial.println("[ESPNOW] UART Buffer Overflow – re-sync.");
                _pc_rxLen = 0;
            }
        }
    }
}

// Setup
void setupEspnowHandler() {
    ESPNOW_SERIAL.begin(ESPNOW_SERIAL_BAUD, SERIAL_8N1, ESPNOW_RX_PIN, ESPNOW_TX_PIN);
    Serial.println("[ESPNOW] Handler ready (bidirectional).");
}
