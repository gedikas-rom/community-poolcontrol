#include <Arduino.h>
#include <ArduinoOTA.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include <LiquidCrystal_PCF8574.h>
#include <OneWireESP32.h>
#include <esp_now.h>
#include <WiFi.h>
#include <ESPmDNS.h>
#include <UpdateOTA.h>
#include <Credentials.h>
#include <Globals.h>
#include <MQTT_common.h>
#include "time.h"
#include "EspnowHandler.h"
#include "PCF8575.h"

void modeChangedFunction(Mode mode);
void targetTempChangedFunction(float targetTemp);
void deltaTempChangedFunction(float deltaTemp);
void offsetWaterChangedFunction(float offsetWater);
void offsetAirChangedFunction(float offsetAir);
void setupWiFi();
void initMqttAfterWifi();

static bool mqttInitialized = false;
static bool lcdInitialized = false;
static bool mdnsInitialized = false;
static unsigned long lastBridgeUartSeen = 0;

// XIAO ESP32-C6 RF switch pins:
// GPIO3  -> WiFi function enable (LOW = enabled)
// GPIO14 -> antenna select (HIGH = external antenna, LOW = onboard antenna)
static constexpr uint8_t XIAO_C6_WIFI_ENABLE_PIN = 3;
static constexpr uint8_t XIAO_C6_WIFI_ANT_PIN    = 14;

void WiFiDisconnected(WiFiEvent_t event, WiFiEventInfo_t info){
  Serial.println("[WIFI] Disconnected from access point");
  Serial.print("[WIFI] Lost connection. Reason: ");
  Serial.println(info.wifi_sta_disconnected.reason);
  Serial.println("[WIFI] Trying to reconnect...");
  WiFi.reconnect();
}

void WiFiConnected(WiFiEvent_t event, WiFiEventInfo_t info){
  Serial.println("[WIFI] Connected to AP successfully");
}

void WiFiGotIP(WiFiEvent_t event, WiFiEventInfo_t info){
  Serial.println("[WIFI] Connected");
  Serial.print("[WIFI] IP address: ");
  Serial.println(WiFi.localIP());
  Serial.print("[WIFI] MAC address: ");
  Serial.println(WiFi.macAddress());

  // init and get the time (DST-aware for Europe/Berlin)
  configTzTime(tzInfo, ntpServer);

  if (!mdnsInitialized) {
    if (MDNS.begin(hostname)) {
      // ArduinoOTA already registers the "arduino" mDNS service.
      // Only add our own service(s) here to avoid duplicates.
      MDNS.addService("http", "tcp", 80);      // future web UI / status page
      MDNS.addServiceTxt("http", "tcp", "fw", firmware);
      MDNS.addServiceTxt("http", "tcp", "device", hostname);
      Serial.printf("[mDNS] Started: %s.local\n", hostname);
      mdnsInitialized = true;
    } else {
      Serial.println("[mDNS] Failed to start");
    }
  }

  initMqttAfterWifi();
}

void initMqttAfterWifi() {
  if (mqttInitialized) return;

  Serial.println("[MQTT] Initializing after WiFi got IP");
  setupMQTT(espClient, firmware, modeChangedFunction, targetTempChangedFunction, deltaTempChangedFunction,
    offsetWaterChangedFunction, offsetAirChangedFunction);
  Serial.println("[MQTT] Publishing initial preferences");
  publishPreferences(mode, currentValveState, currentPumpState, targetTemp, deltaTemp, offsetWater, offsetAir);
  Serial.println("[MQTT] Initialization complete");
  mqttInitialized = true;
}

void setupWiFi() {
  Serial.println("[WIFI] Configuring STA mode");
  WiFi.mode(WIFI_STA);
  WiFi.hostname(hostname);
  WiFi.setAutoReconnect(true);
  
  WiFi.onEvent(WiFiConnected, WiFiEvent_t::ARDUINO_EVENT_WIFI_STA_CONNECTED);
  WiFi.onEvent(WiFiGotIP, WiFiEvent_t::ARDUINO_EVENT_WIFI_STA_GOT_IP);
  WiFi.onEvent(WiFiDisconnected, WiFiEvent_t::ARDUINO_EVENT_WIFI_STA_DISCONNECTED);

  Serial.println("[WIFI] Starting connection");
  WiFi.begin(WIFI_ssid, WIFI_password);
}

void setup() {
  int error;

  Serial.begin(115200);
  delay(1000);

#if defined(ARDUINO_XIAO_ESP32C6)
  pinMode(XIAO_C6_WIFI_ENABLE_PIN, OUTPUT);
  digitalWrite(XIAO_C6_WIFI_ENABLE_PIN, LOW);
  delay(100);
  pinMode(XIAO_C6_WIFI_ANT_PIN, OUTPUT);
  digitalWrite(XIAO_C6_WIFI_ANT_PIN, HIGH);
  Serial.println("[PC] XIAO C6 antenna set to EXTERNAL");
#endif

  Serial.printf("[PC] LCD...Probing for PCF8574 on address 0x%02X...\n", LCD_ADDRESS);

  // See http://playground.arduino.cc/Main/I2cScanner how to test for a I2C device.
  Wire.begin();
  Wire.beginTransmission(LCD_ADDRESS);
  error = Wire.endTransmission();
  Serial.print("[PC] LCD Probe: ");
  Serial.print(error);

  if (error == 0) {
    Serial.println(": LCD found.");
    lcd.begin(20, 4);  // initialize the lcd

    lcd.createChar(1, dotOff);
    lcd.createChar(2, dotOn);
    lcd.createChar(3, wifi);
    lcd.createChar(4, grad);
    lcd.createChar(5, delta);
    lcd.createChar(6, target);
    
    lcd.setBacklight(255);
    lcdInitialized = true;
  } else {
    Serial.println(": LCD not found.");
  }  // if

  //to find addresses for water temp bus
	maxDevicesWater = dsWater.search(addrWater, maxDevicesWater);
	for (uint8_t i = 0; i < maxDevicesWater; i += 1) {
		Serial.printf("[PC] Water %d: 0x%llx,\n", i, addrWater[i]);
	}
  //to find addresses for air temp bus
	maxDevicesAir = dsAir.search(addrAir, maxDevicesAir);
	for (uint8_t j = 0; j < maxDevicesAir; j += 1) {
		Serial.printf("[PC] Air %d: 0x%llx,\n", j, addrAir[j]);
	}

  // Configure pin modes first (library uses this setup during begin())
  pcf8575.pinMode(RELAY_POS2, OUTPUT, HIGH); // Set relay pin as output and initialize to HIGH (off)
  pcf8575.pinMode(RELAY_POS4, OUTPUT, HIGH); // Set relay pin as output and initialize to HIGH (off)
  pcf8575.pinMode(PUMP_OFF, OUTPUT, HIGH); // Set pump control pin as output and initialize to HIGH (off)
  pcf8575.pinMode(PUMP_1, OUTPUT, HIGH); // Set pump control pin as output and initialize to HIGH (off)
  pcf8575.pinMode(PUMP_2, OUTPUT, HIGH); // Set pump control pin as output and initialize to HIGH (off)
  pcf8575.pinMode(PUMP_3, OUTPUT, HIGH); // Set pump control pin as output and initialize to HIGH (off)
  // Button init
  pcf8575.pinMode(BUTTON_PIN, INPUT_PULLUP); // Set button pin as input with pull-up resistor

  // PCF8575 init
  pcf8575.begin();
  Serial.printf("[PC] Init IO Expander for PCF8575 on address 0x%02X\n", PCF8575_ADDRESS);
  // Pressure sensor ADC input
  pinMode(FILTER_PRESSURE_PIN, INPUT);

  Serial.println("[PC] IO Expander pin modes configured");
  
  // Wifi config
  setupWiFi();

  // OTA config
  setupOTA(hostname, []() {
        Serial.println("[OTA] Start");
      }, [](unsigned int progress, unsigned int total) {
        Serial.printf("[OTA] Progress: %u%%\r", (progress / (total / 100)));
      }, []() {
        Serial.println("[OTA] End");
      }, [](ota_error_t error) {
        Serial.printf("[OTA] Error[%u]: ", error);
        if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
        else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
        else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
        else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
        else if (error == OTA_END_ERROR) Serial.println("End Failed");
      });
  
  // Init ESP-NOW
  setupEspnowHandler();
}  // setup()

unsigned long now;
unsigned long lastPrintDisplay = 0;
unsigned long lastDisplayOff = 0;

void readTemperatures();
void readFilterPressure();
void handleModes();
void refreshDisplay();

void loop() {
  // OTA service loop
  loopOTA();
  // MQTT service loop
  loopMQTT();
  // Espnow service loop
  const uint8_t espnowEvents = loopEspnowHandler();
  if (espnowEvents & ESPNOW_EVENT_BRIDGE_UART) {
    lastBridgeUartSeen = millis();
  }
  if (espnowEvents & ESPNOW_EVENT_NODE_TRAFFIC) {
    lastESPNOWRequest = millis();
  }

  now = millis();
  if (lcdInitialized && (now - lastDisplayOff > DISPLAY_OFF_INTERVAL)) {
    lcd.setBacklight(0);
    lastDisplayOff = now;
  }
  if (now - lastPrintDisplay > MEASUREMENT_INTERVAL) {
    Serial.printf("[WIFI] Wifi signal: %d\n", WiFi.RSSI());
    readTemperatures();
    readFilterPressure();
    handleModes();
    refreshDisplay();
    lastPrintDisplay = now;
  }

  // read the state of the switch/button:
  currentState = pcf8575.digitalRead(BUTTON_PIN);

  if(lcdInitialized && lastState == HIGH && currentState == LOW)
  {
    lcd.setBacklight(255);
    lastDisplayOff = now;
  }
  // save the last state
  lastState = currentState;
  
}  // loop()

// callback when data comes in from mqtt
void modeChangedFunction(Mode incomingmode) {
  Serial.printf("[PC] Mode changed from %d to %d\n", mode, incomingmode);
  mode = incomingmode;
  publishMode(mode);
}

void targetTempChangedFunction(float incomingTargetTemp) {
  Serial.printf("[PC] Target temperature changed to: %.1f\n", incomingTargetTemp);
  if (incomingTargetTemp >= 10 && incomingTargetTemp <= 35)
  {
    targetTemp = incomingTargetTemp;
    publishTargetTemp(targetTemp);
  }
}

void deltaTempChangedFunction(float incomingDeltaTemp) {
  Serial.printf("[PC] Delta temperature changed to: %.1f\n", incomingDeltaTemp);
  if (incomingDeltaTemp > 0 && incomingDeltaTemp <= 10)
  {
    deltaTemp = incomingDeltaTemp;
    publishDeltaTemp(deltaTemp);
  }
}

void offsetWaterChangedFunction(float incomingOffsetWater) {
  Serial.printf("[PC] Offset water temperature changed to: %.1f\n", incomingOffsetWater);
  if (incomingOffsetWater >= -5 && incomingOffsetWater <= 5)
  {
    offsetWater = incomingOffsetWater;
    publishOffsetWater(offsetWater);
  }
}

void offsetAirChangedFunction(float incomingOffsetAir) {
  Serial.printf("[PC] Offset air temperature changed to: %.1f\n", incomingOffsetAir);
  if (incomingOffsetAir >= -5 && incomingOffsetAir <= 5)
  {
    offsetAir = incomingOffsetAir;
    publishOffsetAir(offsetAir);
  }
}

float getAverageTempWater() {
  return averageTempWater;
} 

float getAverageTempAir() {
  return averageTempAir;
} 

float getTargetTemp() {
  return targetTemp;
}

float getFilterPressureMpa() {
  return filterPressureMpa;
}

Mode getMode() {
  return mode;
}

ValveState getCurrentValveState() {
  return currentValveState;
}

int getCurrentPumpState() {
  return currentPumpState;
} 

bool isValidTemperatureWater(float temp){
  return temp >= 0 && temp <= 35;
}

bool isValidTemperatureAir(float temp){
  return temp >= 0 && temp <= 50;
}

void readTemperatures(){
  int validMeasures = 0;
  dsWater.request();
  averageTempWater = -1;
  for(byte i = 0; i < maxDevicesWater; i++){
    uint8_t err = dsWater.getTemp(addrWater[i], currTempWater[i]);
    if(err){
      const char *errt[] = {"", "CRC", "BAD","DC","DRV"};
      Serial.print(i); Serial.print("-Water: "); Serial.println(errt[err]);
    }
		else{
      if (isValidTemperatureWater(currTempWater[i]))
      {
        validMeasures++;
        averageTempWater = averageTempWater + currTempWater[i];
      }
		}
	}
  if (validMeasures > 0)
    averageTempWater = averageTempWater/validMeasures + offsetWater;
  else 
    averageTempWater = -1;

  dsAir.request();
  validMeasures = 0;  
  averageTempAir = -1;
  for(byte j = 0; j < maxDevicesAir; j++){
    uint8_t err = dsAir.getTemp(addrAir[j], currTempAir[j]);
    if(err){
      const char *errt[] = {"", "CRC", "BAD","DC","DRV"};
      Serial.print(j); Serial.print("-Air: "); Serial.println(errt[err]);
    }
		else{
      if (isValidTemperatureAir(currTempAir[j]))
      {
        averageTempAir = averageTempAir + currTempAir[j];
        validMeasures++;
	  	}
    }
	}
  if (validMeasures > 0)
    averageTempAir = averageTempAir/validMeasures + offsetAir;
  else 
    averageTempAir = -1;
  
  publishTemperatures(averageTempWater, averageTempAir);
}

void readFilterPressure() {
  const uint16_t adcMax = 4095;
  const float adcRefVoltage = 3.3f;
  const float dividerFactor = (FILTER_PRESSURE_R1_OHM + FILTER_PRESSURE_R2_OHM) / FILTER_PRESSURE_R2_OHM;
  const float slopeMpaPerVolt = (FILTER_PRESSURE_MAX_MPA - FILTER_PRESSURE_MIN_MPA) /
                                (FILTER_PRESSURE_SENSOR_MAX_V - FILTER_PRESSURE_SENSOR_MIN_V);

  uint32_t adcRawSum = 0;
  for (uint8_t sample = 0; sample < FILTER_PRESSURE_SAMPLE_COUNT; sample++) {
    adcRawSum += analogRead(FILTER_PRESSURE_PIN);
    delay(2);
  }

  const uint16_t adcRaw = adcRawSum / FILTER_PRESSURE_SAMPLE_COUNT;
  const float adcVoltage = (static_cast<float>(adcRaw) / adcMax) * adcRefVoltage;
  const float sensorVoltage = adcVoltage * dividerFactor;

  float pressureMpa = (sensorVoltage - FILTER_PRESSURE_SENSOR_MIN_V) * slopeMpaPerVolt + FILTER_PRESSURE_MIN_MPA;
  pressureMpa *= FILTER_PRESSURE_CALIBRATION_FACTOR;
  if (pressureMpa < FILTER_PRESSURE_MIN_MPA) pressureMpa = FILTER_PRESSURE_MIN_MPA;
  if (pressureMpa > FILTER_PRESSURE_MAX_MPA) pressureMpa = FILTER_PRESSURE_MAX_MPA;

  filterPressureMpa = pressureMpa;
  Serial.printf("[PC] Filter pressure raw=%u adc=%.3fV sensor=%.3fV pressure=%.3fMPa factor=%.2f\n",
                adcRaw, adcVoltage, sensorVoltage, filterPressureMpa, FILTER_PRESSURE_CALIBRATION_FACTOR);
  publishFilterPressure(filterPressureMpa);
}

unsigned long lastPumpVelocityChange = 0;
static bool pumpStateInitialized = false;

static uint8_t pumpRelayForVelocity(uint8_t velocity) {
  switch (velocity) {
    case 0: return PUMP_OFF;
    case 1: return PUMP_1;
    case 2: return PUMP_2;
    case 3: return PUMP_3;
    default: return PUMP_1;
  }
}

void handlePumpControl(uint8_t velocity){
  const bool needsInit = !pumpStateInitialized;
  const bool needsChange = currentPumpState != velocity;

  if (!needsInit && !needsChange) return;
  if (!needsInit && (now - lastPumpVelocityChange <= PUMP_INTERVAL)) return;

  const uint8_t relay = pumpRelayForVelocity(velocity);
  pcf8575.digitalWrite(relay, LOW);
  currentPumpState = velocity;
  pumpStateInitialized = true;

  if (needsInit) {
    Serial.print("[PC] Pump initialized to level...");
  } else {
    Serial.print("[PC] Pump switching to level...");
  }
  Serial.println(currentPumpState);
  publishPumpState(currentPumpState);

  delay(2000);
  pcf8575.digitalWrite(relay, HIGH);
  lastPumpVelocityChange = now;
}

unsigned long lastValveMovement = 0;
void handleValveAndPumpControl(String position) {
  Serial.printf("[PC] position: %s, now: %lu, lastValveMovement: %lu\nVALVE_INTERVAL: %lu, currentValveState: %lu\n", position, now, lastValveMovement, VALVE_INTERVAL, currentValveState);
  if (now - lastValveMovement <= VALVE_INTERVAL)
  {
    // Valve is still moving
    return;
  }

  // Only complete a movement if we were previously in a moving state.
  // On startup (UNDEFINED), this block must not toggle relays.
  if (currentValveState == OPENING || currentValveState == CLOSING)
  {
    pcf8575.digitalWrite(RELAY_POS2, HIGH); // switch off after delay
    pcf8575.digitalWrite(RELAY_POS4, HIGH); // switch off after delay
    if (currentValveState == OPENING)
      currentValveState = OPEN;
    else if (currentValveState == CLOSING)
      currentValveState = CLOSED;
    lastValveMovement = 0;
    Serial.printf("[PC] position: %s, now: %lu, lastValveMovement: %lu, VALVE_INTERVAL: %lu\n", position, now, lastValveMovement, VALVE_INTERVAL);
    Serial.println("Valve movement finished");
  } 

  if (position == "Pos2") {
      // Activate large loop (solar)
      if (currentValveState != OPEN)
      {
        Serial.println("[PC] Valve opening...");
        pcf8575.digitalWrite(RELAY_POS2, LOW);
        pcf8575.digitalWrite(RELAY_POS4, HIGH); // Disable small loop
        lastValveMovement = now;
        currentValveState = OPENING;
      }
      handlePumpControl(PUMPLEVEL_FULL); // Oder Stufe2 basierend auf Bedingungen
  } else if (position == "Pos4") {
      // Activate small loop
      if (currentValveState != CLOSED)
      {
        Serial.println("[PC] Valve closing...");
        pcf8575.digitalWrite(RELAY_POS4, LOW);
        pcf8575.digitalWrite(RELAY_POS2, HIGH); // Disable large loop
        lastValveMovement = now;
        currentValveState = CLOSING;
      }
      if (mode == CLEANING)
        handlePumpControl(PUMPLEVEL_CLEANING); // Pump full power for cleaning
      else
        handlePumpControl(PUMPLEVEL_ECO);
  }
  publishValveState(currentValveState);
}

void handleModes() {
  // Check valid sensor data
  bool validData =!(averageTempAir < 0 || averageTempWater < 0); 

  // Control operating modes
  if (mode == ON) {
      handleValveAndPumpControl("Pos2");
  } else if (mode == OFF || mode == CLEANING) {
      handleValveAndPumpControl("Pos4");
  } else if (mode == AUTO && validData) {
      if (averageTempAir-deltaTemp > averageTempWater && averageTempWater < targetTemp) {
          Serial.println("[PC] handleModes: AUTO - Pos2");
          handleValveAndPumpControl("Pos2"); // Activate large loop
      } else if (averageTempAir <= averageTempWater || currentValveState == UNDEFINED) {
          handleValveAndPumpControl("Pos4"); // Activate small loop
          Serial.println("[PC] handleModes: AUTO - Pos4");
      } else
      {
          handleValveAndPumpControl("----"); // No new valve position, stop valve movement if needed
          Serial.println("[PC] handleModes: AUTO - Pos2 in Delta-Temp");
      }
  }
}

void printStatusBar(){
  lcd.setCursor(0, 3);
  switch (mode) {
    case ON: lcd.print("Manuell - AN"); break;
    case OFF: lcd.print("Manuell - AUS"); break;
    case CLEANING: lcd.print("Man. - REINIGUNG"); break;
    case AUTO: lcd.print("AUTO"); break;
  }
  // WiFi Status
  lcd.setCursor(17, 3);
  Serial.printf("[WIFI] Wifi signal: %d\n", WiFi.RSSI());
  if (WiFi.status() == WL_CONNECTED)
    lcd.print("\03");
  else
    lcd.print("\01");

  // Bridge-UART Status
  lcd.setCursor(18, 3);
  if (lastBridgeUartSeen > 0 && now - lastBridgeUartSeen < 20000)
    lcd.print("B");
  else
    lcd.print(" ");

  // ESP-NOW Status
  lcd.setCursor(19, 3);
  if (lastESPNOWRequest > 0 && now - lastESPNOWRequest < 20000)
    lcd.print("\06");
  else
    lcd.print(" ");
}

void printTempOnDisplay(){
  lcd.home();
  lcd.clear();
  lcd.print("Luft:   ");
  if (averageTempAir<0)
    lcd.print("Sensorfehler");  
  else {
    lcd.printf("%.1f\04C", averageTempAir);
  }
  lcd.setCursor(0, 1);
  lcd.print("Wasser: ");
  if (averageTempWater<0)
    lcd.println("Sensorfehler");  
  else {
    lcd.printf("%.1f\04C", averageTempWater);
  }
  lcd.setCursor(0, 2);
  lcd.printf("Ziel/\05: %.1f\04C/%.1fK", targetTemp, deltaTemp);
  printStatusBar();
}

int currentPage = 0;
void refreshDisplay(){
  if (!lcdInitialized) return;

  if (currentPage == 0)
  {
    printTempOnDisplay();
  } else if (currentPage == 1)
  {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Pumpe:  ");
    if (currentPumpState == 0)
      lcd.print("STOPP");
    else 
      lcd.printf("Stufe %d", currentPumpState);

    lcd.setCursor(0, 1);
    switch (currentValveState) {
      case UNDEFINED: lcd.print("Ventil: Unbekannt"); break;
      case OPENING: lcd.print("Ventil: Oeffnet..."); break;
      case OPEN: lcd.print("Ventil: Offen"); break;
      case CLOSING: lcd.print("Ventil: Schliesst..."); break;
      case CLOSED: lcd.print("Ventil: Geschlossen"); break;   
    }
    printStatusBar();  
  } else if (currentPage == 2)
  {
    // DEBUG - Page
    // WLAN, MQTT, ESP-Now
    lcd.clear();
    lcd.setCursor(5, 1);
    lcd.print("Automatic");
  }
  currentPage = (currentPage + 1) % 2;
}
