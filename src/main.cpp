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

  // init and get the time
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);

  if (!mdnsInitialized) {
    if (MDNS.begin(hostname)) {
      MDNS.addService("arduino", "tcp", 3232); // ArduinoOTA discovery
      MDNS.addService("http", "tcp", 80);      // future web UI / status page
      MDNS.addServiceTxt("arduino", "tcp", "fw", firmware);
      MDNS.addServiceTxt("http", "tcp", "fw", firmware);
      MDNS.addServiceTxt("arduino", "tcp", "device", hostname);
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
  // turn on the external antenna
  pinMode(3, OUTPUT);
  digitalWrite(3, LOW); //turn on this function
  delay(100);
  pinMode(14, OUTPUT); 
  digitalWrite(14, HIGH);//use external antenna  
  Serial.println("[PC] External antenna enabled");
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

  // PCF8575 init
  pcf8575.begin();
  Serial.printf("[PC] Init IO Expander for PCF8575 on address 0x%02X\n", PCF8575_ADDRESS);

  // Configure pin modes
  pcf8575.pinMode(RELAY_POS2, OUTPUT, HIGH); // Set relay pin as output and initialize to HIGH (off)
  pcf8575.pinMode(RELAY_POS4, OUTPUT, HIGH); // Set relay pin as output and initialize to HIGH (off)
  pcf8575.pinMode(PUMP_OFF, OUTPUT, HIGH); // Set pump control pin as output and initialize to HIGH (off)
  pcf8575.pinMode(PUMP_1, OUTPUT, HIGH); // Set pump control pin as output and initialize to HIGH (off)
  pcf8575.pinMode(PUMP_2, OUTPUT, HIGH); // Set pump control pin as output and initialize to HIGH (off)
  pcf8575.pinMode(PUMP_3, OUTPUT, HIGH); // Set pump control pin as output and initialize to HIGH (off)
  // Button init
  pcf8575.pinMode(BUTTON_PIN, INPUT_PULLUP); // Set button pin as input with pull-up resistor
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
  loopEspnowHandler();

  now = millis();
  if (lcdInitialized && (now - lastDisplayOff > DISPLAY_OFF_INTERVAL)) {
    lcd.setBacklight(0);
    lastDisplayOff = now;
  }
  if (now - lastPrintDisplay > MEASUREMENT_INTERVAL) {
    Serial.printf("[WIFI] Wifi signal: %d\n", WiFi.RSSI());
    readTemperatures();
    //readFilterPressure();
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

  const uint16_t adcRaw = analogRead(FILTER_PRESSURE_PIN);
  const float adcVoltage = (static_cast<float>(adcRaw) / adcMax) * adcRefVoltage;
  const float sensorVoltage = adcVoltage * dividerFactor;

  float pressureMpa = (sensorVoltage - FILTER_PRESSURE_SENSOR_MIN_V) * slopeMpaPerVolt + FILTER_PRESSURE_MIN_MPA;
  if (pressureMpa < FILTER_PRESSURE_MIN_MPA) pressureMpa = FILTER_PRESSURE_MIN_MPA;
  if (pressureMpa > FILTER_PRESSURE_MAX_MPA) pressureMpa = FILTER_PRESSURE_MAX_MPA;

  filterPressureMpa = pressureMpa;
  Serial.printf("[PC] Filter pressure raw=%u adc=%.3fV sensor=%.3fV pressure=%.3fMPa\n",
                adcRaw, adcVoltage, sensorVoltage, filterPressureMpa);
  publishFilterPressure(filterPressureMpa);
}

unsigned long lastPumpVelocityChange = 0;
void handlePumpControl(uint8_t velocity){
  if (now - lastPumpVelocityChange > PUMP_INTERVAL && currentPumpState != velocity)
  {
    switch(velocity) {
      case 0: pcf8575.digitalWrite(PUMP_OFF, LOW); currentPumpState = 0; break; // Stop pump
      case 1: pcf8575.digitalWrite(PUMP_1, LOW); currentPumpState = 1; break; // Level 1 = eco
      case 2: pcf8575.digitalWrite(PUMP_2, LOW); currentPumpState = 2; break; // Level 2
      case 3: pcf8575.digitalWrite(PUMP_3, LOW); currentPumpState = 3; break; // Level 3
    }
    Serial.print("[PC] Pump switching to level...");
    Serial.println(currentPumpState);
    publishPumpState(currentPumpState);

    delay(2000);
    pcf8575.digitalWrite(PUMP_OFF, HIGH);
    pcf8575.digitalWrite(PUMP_1, HIGH);
    pcf8575.digitalWrite(PUMP_2, HIGH);
    pcf8575.digitalWrite(PUMP_3, HIGH);
    lastPumpVelocityChange = now;
  }
}

unsigned long lastValveMovement = 0;
void handleValveAndPumpControl(String position) {
  Serial.printf("[PC] position: %s, now: %lu, lastValveMovement: %lu\nVALVE_INTERVAL: %lu, currentValveState: %lu\n", position, now, lastValveMovement, VALVE_INTERVAL, currentValveState);
  if (now - lastValveMovement <= VALVE_INTERVAL)
  {
    // Valve is still moving
    return;
  }

  if (currentValveState != OPEN && currentValveState != CLOSED)
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

  // ESP-NOW Status
  lcd.setCursor(19, 3);
  if (lastESPNOWRequest > 0 && now - lastESPNOWRequest < 20000)
    lcd.print("\06");
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
