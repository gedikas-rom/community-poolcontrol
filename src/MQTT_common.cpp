#include <PubSubClient.h>
#include <MQTT_ha.h>
#include <WiFi.h>
#include <GlobalDefs.h>

PubSubClient mqtt;
const char* mqtt_server = "192.168.179.23"; //"iobroker.fritz.box";
const int mqtt_port = 1890;

const char* mqtt_user = "";      // Optional
const char* mqtt_password = "";  // Optional

const char* mqtt_topic_state = "poolcontrol/state";
const char* mqtt_topic_firmware = "poolcontrol/firmware";  // aktuelle Firmware Version
const char* mqtt_topic_mode = "poolcontrol/mode";  // mode
const char* mqtt_topic_valvestate = "poolcontrol/valvestate";  // current valvestate
const char* mqtt_topic_pumpstate = "poolcontrol/pumpstate";  // current pumpstate
const char* mqtt_topic_TempWater = "poolcontrol/TempWater";  // averageTempWater
const char* mqtt_topic_TempAir = "poolcontrol/TempAir";  // averageTempAir
const char* mqtt_topic_targetTemp = "poolcontrol/TargetTemp";  // targetTemp
const char* mqtt_topic_deltaTemp = "poolcontrol/DeltaTemp";  // deltaTemp

const char* mqtt_topic_set_mode = "poolcontrol/set/mode";  // topic for setting mode
const char* mqtt_topic_set_targetTemp = "poolcontrol/set/targettemp";  // topic for setting targetTemp
const char* mqtt_topic_set_deltaTemp = "poolcontrol/set/deltatemp";  // topic for setting deltaTemp

const char* mqtt_topic_PavilionTemp = "poolcontrol/PavilionTemp";  // PavilionTemp
const char* mqtt_topic_PavilionBattery = "poolcontrol/PavilionBattery";  // PavilionBattery
const char* mqtt_topic_PavilionFirmware = "poolcontrol/PavilionFirmware";  // PavilionFirmware
const char* mqtt_topic_GreenhouseTemp = "poolcontrol/GreenhouseTemp";  // GreenhouseTemp
const char* mqtt_topic_GreenhouseBattery = "poolcontrol/GreenhouseBattery";  // GreenhouseBattery
const char* mqtt_topic_GreenhouseFirmware = "poolcontrol/GreenhouseFirmware";  // GreenhouseFirmware


unsigned long lastMqttReconnectAttempt = 0;
unsigned long lastWaterLevelCheck = 0;
const unsigned long MQTT_RECONNECT_INTERVAL = 5000; // 5 seconds between reconnect attempts

const char* _firmware;
void (*_modeChangedFunction)(Mode mode);  
void (*_targetTempChangedFunction)(float targetTemp);
void (*_deltaTempChangedFunction)(float deltaTemp);

void publishValveState(ValveState state);
void publishPumpState(int state);

// MQTT Callback für eingehende Nachrichten
void mqttCallback(char* topic, byte* payload, unsigned int length) {
  // Payload in String umwandeln
  char message[length + 1];
  memcpy(message, payload, length);
  message[length] = '\0';
  
  Serial.printf("MQTT Message arrived [%s]: %s\n", topic, message);
  if (String(topic) == mqtt_topic_set_mode) {
    if (String(message) == "AUTO") {
      _modeChangedFunction(AUTO);
    } else if (String(message) == "ON") {
      _modeChangedFunction(ON);
    } else if (String(message) == "OFF") {
      _modeChangedFunction(OFF);
    }
  } else if (String(topic) == mqtt_topic_set_targetTemp) {
    _targetTempChangedFunction(atof(message));
  } else if (String(topic) == mqtt_topic_set_deltaTemp) {
    _deltaTempChangedFunction(atof(message));
  }
}

bool connectMQTT() {
  if (mqtt.connected()) {
    return true;
  }

  Serial.print("Connect to MQTT Broker...");
  
  // Client ID generieren
  String clientId = "Poolcontrol";
  
  // Verbindungsversuch mit Credentials
  bool connected = false;
  if (mqtt_user && mqtt_password) {
    connected = mqtt.connect(clientId.c_str(), mqtt_user, mqtt_password, 
      mqtt_topic_state, 1, true, "offline");
  } else {
    connected = mqtt.connect(clientId.c_str(), mqtt_topic_state, 1, true, "offline");
  }

  if (connected) {
    Serial.println("connected");
    // publish Online State
    mqtt.publish(mqtt_topic_state, "online", true);
    mqtt.publish(mqtt_topic_firmware, _firmware, true); // firmware
    mqtt.subscribe(mqtt_topic_set_mode);
    mqtt.subscribe(mqtt_topic_set_targetTemp);
    mqtt.subscribe(mqtt_topic_set_deltaTemp);
    
    // Setting Homeassistant sensor config
    Serial.println("--> HA Config");
    mqtt.setBufferSize(1000);
    mqtt.publish(mqtt_topic_ha_firmware.c_str(), mqtt_ha_config_firmware, true);
    mqtt.publish(mqtt_topic_ha_set_mode.c_str(), mqtt_ha_config_set_mode, true);
    mqtt.publish(mqtt_topic_ha_set_targetTemp.c_str(), mqtt_ha_config_set_targetTemp, true);
    mqtt.publish(mqtt_topic_ha_set_deltaTemp.c_str(), mqtt_ha_config_set_deltaTemp, true);
    mqtt.publish(mqtt_topic_ha_TempWater.c_str(), mqtt_ha_config_TempWater, true);
    mqtt.publish(mqtt_topic_ha_TempAir.c_str(), mqtt_ha_config_TempAir, true);
    mqtt.publish(mqtt_topic_ha_ValveState.c_str(), mqtt_ha_config_ValveState, true);
    mqtt.publish(mqtt_topic_ha_PumpState.c_str(), mqtt_ha_config_PumpState, true);
    mqtt.publish(mqtt_topic_ha_mode.c_str(), mqtt_ha_config_mode, true);
    mqtt.publish(mqtt_topic_ha_targetTemp.c_str(), mqtt_ha_config_targetTemp, true);
    mqtt.publish(mqtt_topic_ha_deltaTemp.c_str(), mqtt_ha_config_deltaTemp, true);
    mqtt.publish(mqtt_topic_ha_PavilionTemp.c_str(), mqtt_ha_config_PavilionTemp, true);
    mqtt.publish(mqtt_topic_ha_PavilionBattery.c_str(), mqtt_ha_config_PavilionBattery, true);
    mqtt.publish(mqtt_topic_ha_PavilionFirmware.c_str(), mqtt_ha_config_PavilionFirmware, true);
    mqtt.publish(mqtt_topic_ha_GreenhouseTemp.c_str(), mqtt_ha_config_GreenhouseTemp, true);
    mqtt.publish(mqtt_topic_ha_GreenhouseBattery.c_str(), mqtt_ha_config_GreenhouseBattery, true);
    mqtt.publish(mqtt_topic_ha_GreenhouseFirmware.c_str(), mqtt_ha_config_GreenhouseFirmware, true);
    Serial.println("<-- HA Config");

    return true;
  } else {
    Serial.print("failure, rc=");
    Serial.println(mqtt.state());
    return false;
  }
}

float lasttargetTemp = -1;
float lastdeltaTemp = -1;

void publishPreferences(Mode mode, ValveState valvestate, int state, float targetTemp, float deltaTemp) {
  if (!mqtt.connected()) {
    return;
  }

  Serial.printf("MQTT Update - publishPreferences: Mode: %d, TargetTemp: %.1f, DeltaTemp: %.1f\n", mode, targetTemp, deltaTemp);
  mqtt.publish(mqtt_topic_mode, mode == AUTO ? "AUTO" : mode == ON ? "ON" : "OFF", true);

  if (lasttargetTemp != targetTemp) {
    char targetTempStr[10];
    dtostrf(targetTemp, 1, 1, targetTempStr);
    mqtt.publish(mqtt_topic_targetTemp, targetTempStr, true);
    lasttargetTemp = targetTemp;          
  }
  if (lastdeltaTemp != deltaTemp) {
    char deltaTempStr[10];
    dtostrf(deltaTemp, 1, 1, deltaTempStr);
    mqtt.publish(mqtt_topic_deltaTemp, deltaTempStr, true);
    lastdeltaTemp = deltaTemp;
  }
  publishValveState(valvestate);
  publishPumpState(state);
}

void publishTemperatures(float averageTempWater, float averageTempAir) {
  if (!mqtt.connected()) {
    return;
  }
  Serial.printf("MQTT Update - publishTemperatures: TempWater: %.1f, TempAir: %.1f\n", averageTempWater, averageTempAir);
  char tempWaterStr[10];
  dtostrf(averageTempWater, 1, 1, tempWaterStr);
  mqtt.publish(mqtt_topic_TempWater, tempWaterStr, true);

  char tempAirStr[10];
  dtostrf(averageTempAir, 1, 1, tempAirStr);
  mqtt.publish(mqtt_topic_TempAir, tempAirStr, true);
}

void publishValveState(ValveState state) {
  if (!mqtt.connected()) {
    return;
  }
  Serial.printf("MQTT Update - publishValveState: %d", state);
  mqtt.publish(mqtt_topic_valvestate, state == OPEN ? "OPEN" : state == CLOSED ? "CLOSED" : state == CLOSING ? "CLOSING" : state == OPENING ? "OPENING" : "UNDEFINED", true);
}

void publishPumpState(int state) {
  if (!mqtt.connected()) {
    return;
  }
  Serial.printf("MQTT Update - publishPumpState: %d", state);
  char pumpStateStr[10];
  dtostrf(state, 1, 1, pumpStateStr);
  mqtt.publish(mqtt_topic_pumpstate, pumpStateStr, true);
}

void publishMode(Mode mode) {
  if (!mqtt.connected()) {
    return;
  }
  Serial.printf("MQTT Update - publishMode: %d\n", mode);
  mqtt.publish(mqtt_topic_mode, mode == AUTO ? "AUTO" : mode == ON ? "ON" : "OFF", true);
}

void publishTargetTemp(float targetTemp) {
  if (!mqtt.connected()) {
    return;
  }
  Serial.printf("MQTT Update - publishTargetTemp: %.1f\n", targetTemp);
  char targetTempStr[10];
  dtostrf(targetTemp, 1, 1, targetTempStr);
  mqtt.publish(mqtt_topic_targetTemp, targetTempStr, true);
}

void publishDeltaTemp(float deltaTemp) {
  if (!mqtt.connected()) {
    return;
  }
  Serial.printf("MQTT Update - publishDeltaTemp: %.1f\n", deltaTemp);
  char deltaTempStr[10];
  dtostrf(deltaTemp, 1, 1, deltaTempStr);
  mqtt.publish(mqtt_topic_deltaTemp, deltaTempStr, true);
}

void publishPavilionSensorData(float temp, int battery, const char* firmware) {
  if (!mqtt.connected()) {
    return;
  }
  Serial.printf("MQTT Update - publishPavilionTemp: %.1f, battery: %d, firmware: %s\n", temp, battery, firmware);
  char tempStr[10];
  dtostrf(temp, 1, 1, tempStr);
  mqtt.publish(mqtt_topic_PavilionTemp, tempStr, true);

  char batteryStr[5];
  dtostrf(battery, 1, 1, batteryStr);
  mqtt.publish(mqtt_topic_PavilionBattery, batteryStr, true);

  mqtt.publish(mqtt_topic_PavilionFirmware, firmware, true);
}

void publishGreenhouseSensorData(float temp, int battery, const char* firmware) {
  if (!mqtt.connected()) {
    return;
  }
  Serial.printf("MQTT Update - publishGreenhouseTemp: %.1f, battery: %d, firmware: %s\n", temp, battery, firmware);
  char tempStr[10];
  dtostrf(temp, 1, 1, tempStr);
  mqtt.publish(mqtt_topic_GreenhouseTemp, tempStr, true);

  char batteryStr[5];
  dtostrf(battery, 1, 1, batteryStr);
  mqtt.publish(mqtt_topic_GreenhouseBattery, batteryStr, true);

  mqtt.publish(mqtt_topic_GreenhouseFirmware, firmware, true);
}

void setupMQTT(WiFiClient& espClient, const char* firmware,     
    void (*modeChangedFunction)(Mode mode),
    void (*targetTempChangedFunction)(float targetTemp),
    void (*deltaTempChangedFunction)(float deltaTemp))  
{
    _modeChangedFunction = modeChangedFunction;
    _targetTempChangedFunction = targetTempChangedFunction;
    _deltaTempChangedFunction = deltaTempChangedFunction;
    _firmware = firmware;
    mqtt.setClient(espClient);
    mqtt.setServer(mqtt_server, mqtt_port);
    Serial.println("MQTT Server: " + String(mqtt_server) + ":" + String(mqtt_port));
    mqtt.setCallback(mqttCallback);
    connectMQTT();
    Serial.println("MQTT setup done");
}

void loopMQTT() {
  // check MQTT connect and reconnect if necessary
  if (!mqtt.connected()) {
    unsigned long now = millis();
    if (now - lastMqttReconnectAttempt > MQTT_RECONNECT_INTERVAL) {
      lastMqttReconnectAttempt = now;
      if (connectMQTT()) {
        lastMqttReconnectAttempt = 0;
      }
    }
  }
  mqtt.loop();
}
