#include <Arduino.h>
#include <ArduinoOTA.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include <LiquidCrystal_PCF8574.h>
#include <OneWireESP32.h>
#include <esp_now.h>
#include <WiFi.h>
#include <UpdateOTA.h>
#include <Credentials.h>
#include <Globals.h>
#include <MQTT_common.h>
                
void OnDataRecv(const esp_now_recv_info_t *esp_now_info, const uint8_t *incomingData, int len);
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status);
void modeChangedFunction(Mode mode);
void targetTempChangedFunction(float targetTemp);
void deltaTempChangedFunction(float deltaTemp);
void offsetWaterChangedFunction(float offsetWater);
void offsetAirChangedFunction(float offsetAir);

void WiFiDisconnected(WiFiEvent_t event, WiFiEventInfo_t info){
  Serial.println("Disconnected from WiFi access point");
  Serial.print("WiFi lost connection. Reason: ");
  Serial.println(info.wifi_sta_disconnected.reason);
  Serial.println("Trying to Reconnect");
  WiFi.begin(WIFI_ssid, WIFI_password);
}

void WiFiConnected(WiFiEvent_t event, WiFiEventInfo_t info){
  Serial.println("Connected to AP successfully!");
}

void WiFiGotIP(WiFiEvent_t event, WiFiEventInfo_t info){
  Serial.println("WiFi connected");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
  Serial.print("MAC-Address: ");
  Serial.println(WiFi.macAddress());
}

unsigned long lastWiFiCheck = 0;
void checkWiFi()
{
  unsigned long now = millis();
  if ((WiFi.status() != WL_CONNECTED) && (now - lastWiFiCheck >= WIFI_RECONNECT_INTERVAL)) {
    Serial.println("Reconnecting to WiFi...");
    WiFi.disconnect();
    WiFi.reconnect();
    lastWiFiCheck = now;
  }
}

void setupWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.hostname(hostname);
  WiFi.disconnect(true);
  delay(1000);
  
  WiFi.onEvent(WiFiConnected, WiFiEvent_t::ARDUINO_EVENT_WIFI_STA_CONNECTED);
  WiFi.onEvent(WiFiGotIP, WiFiEvent_t::ARDUINO_EVENT_WIFI_STA_GOT_IP);
  WiFi.onEvent(WiFiDisconnected, WiFiEvent_t::ARDUINO_EVENT_WIFI_STA_DISCONNECTED);

  WiFi.begin(WIFI_ssid, WIFI_password);
}

void setup() {
  int error;

  Serial.begin(115200);
  // turn on the external antenna
  pinMode(3, OUTPUT);
  digitalWrite(3, LOW); //turn on this function
  delay(100);
  pinMode(14, OUTPUT); 
  digitalWrite(14, HIGH);//use external antenna  
  Serial.println("External antenna enabled");
  Serial.println("LCD...Probing for PCF8574 on address 0x27...");

  // See http://playground.arduino.cc/Main/I2cScanner how to test for a I2C device.
  Wire.begin();
  Wire.beginTransmission(0x27);
  error = Wire.endTransmission();
  Serial.print("Error: ");
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
  } else {
    Serial.println(": LCD not found.");
  }  // if

  //to find addresses for water temp bus
	maxDevicesWater = dsWater.search(addrWater, maxDevicesWater);
	for (uint8_t i = 0; i < maxDevicesWater; i += 1) {
		Serial.printf("Water %d: 0x%llx,\n", i, addrWater[i]);
	}
  //to find addresses for air temp bus
	maxDevicesAir = dsAir.search(addrAir, maxDevicesAir);
	for (uint8_t j = 0; j < maxDevicesAir; j += 1) {
		Serial.printf("Air %d: 0x%llx,\n", j, addrAir[j]);
	}

  // Pin-Modi konfigurieren
  pinMode(RELAY_POS2, OUTPUT);
  pinMode(RELAY_POS4, OUTPUT);
  pinMode(PUMP_OFF, OUTPUT);
  pinMode(PUMP_1, OUTPUT);
  pinMode(PUMP_2, OUTPUT);
  pinMode(PUMP_3, OUTPUT);

  // Relais und Pumpe initial ausschalten
  digitalWrite(RELAY_POS2, HIGH);
  digitalWrite(RELAY_POS4, HIGH);
  digitalWrite(PUMP_OFF, HIGH);
  digitalWrite(PUMP_1, HIGH);
  digitalWrite(PUMP_2, HIGH);
  digitalWrite(PUMP_3, HIGH);

  // Button init
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  // Wifi config
  setupWiFi();

  // MQTT config
  setupMQTT(espClient, firmware, modeChangedFunction, targetTempChangedFunction, deltaTempChangedFunction, 
    offsetWaterChangedFunction, offsetAirChangedFunction);
  publishPreferences(mode, currentValveState, currentPumpState, targetTemp, deltaTemp, offsetWater, offsetAir);

  // OTA config
  setupOTA(hostname, []() {
        Serial.println("OTA Start");
      }, [](unsigned int progress, unsigned int total) {
        Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
      }, []() {
        Serial.println("\nEnd");
      }, [](ota_error_t error) {
        Serial.printf("Error[%u]: ", error);
        if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
        else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
        else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
        else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
        else if (error == OTA_END_ERROR) Serial.println("End Failed");
      });
  
  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  esp_now_register_recv_cb(OnDataRecv);
  esp_now_register_send_cb(OnDataSent);
}  // setup()

unsigned long now;
unsigned long lastPrintDisplay = 0;
unsigned long lastDisplayOff = 0;

void readTemperatures();
void handleModes();
void refreshDisplay();

void loop() {
  // OTA service loop
  loopOTA();
  // MQTT service loop
  loopMQTT();
  // check WiFi status
  checkWiFi();

  now = millis();
  if (now - lastDisplayOff > DISPLAY_OFF_INTERVAL) {
    lcd.setBacklight(0);
    lastDisplayOff = now;
  }
  if (now - lastPrintDisplay > MEASUREMENT_INTERVAL) {
    readTemperatures();
    handleModes();
    refreshDisplay();
    lastPrintDisplay = now;
  }

  // read the state of the switch/button:
  currentState = digitalRead(BUTTON_PIN);

  if(lastState == HIGH && currentState == LOW)
  {
    lcd.setBacklight(255);
    lastDisplayOff = now;
  }
  // save the last state
  lastState = currentState;
  
}  // loop()

// callback when data comes in from mqtt
void modeChangedFunction(Mode incomingmode) {
  Serial.printf("Mode changed from %d to %d\n", mode, incomingmode);
  mode = incomingmode;
  publishMode(mode);
}

void targetTempChangedFunction(float incomingTargetTemp) {
  Serial.print("Target temperature changed to: ");
  Serial.println(incomingTargetTemp);
  if (incomingTargetTemp >= 10 && incomingTargetTemp <= 35)
  {
    targetTemp = incomingTargetTemp;
    publishTargetTemp(targetTemp);
  }
}

void deltaTempChangedFunction(float incomingDeltaTemp) {
  Serial.print("Delta temperature changed to: ");
  Serial.println(incomingDeltaTemp);
  if (incomingDeltaTemp > 0 && incomingDeltaTemp <= 10)
  {
    deltaTemp = incomingDeltaTemp;
    publishDeltaTemp(deltaTemp);
  }
}

void offsetWaterChangedFunction(float incomingOffsetWater) {
  Serial.print("Offset water temperature changed to: ");
  Serial.println(incomingOffsetWater);
  if (incomingOffsetWater >= -5 && incomingOffsetWater <= 5)
  {
    offsetWater = incomingOffsetWater;
    publishOffsetWater(offsetWater);
  }
}

void offsetAirChangedFunction(float incomingOffsetAir) {
  Serial.print("Offset air temperature changed to: ");
  Serial.println(incomingOffsetAir);
  if (incomingOffsetAir >= -5 && incomingOffsetAir <= 5)
  {
    offsetAir = incomingOffsetAir;
    publishOffsetAir(offsetAir);
  }
}

// callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
  lastESPNOWRequest = millis();
}

// callback function that will be executed when data is received
void OnDataRecv(const esp_now_recv_info_t *esp_now_info, const uint8_t *incomingData, int len) {
	String payload;
	payload.reserve(len);
	for(auto i = 0; i < len; i++)
	{
		payload += (char)incomingData[i];
	}

	JsonDocument doc;
	DeserializationError error = deserializeJson(doc, payload);
	if(error) { Serial.print("Error receiving data. Bad response"); return; }

  esp_now_peer_info_t peerInfo;
  memset(&peerInfo, 0, sizeof(peerInfo));
  memcpy(peerInfo.peer_addr, esp_now_info->src_addr, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false; 
  Serial.printf("src_addr: %02x:%02x:%02x:%02x:%02x:%02x\n",
                  peerInfo.peer_addr[0],peerInfo.peer_addr[1], peerInfo.peer_addr[2],
                  peerInfo.peer_addr[3], peerInfo.peer_addr[4],peerInfo.peer_addr[5]);
 
  // delete before pair
  const uint8_t *peer_addr = peerInfo.peer_addr;
  esp_err_t delStatus = esp_now_del_peer(peer_addr);
  Serial.print("Slave Delete Status: ");
  if (delStatus == ESP_OK) {
    // Delete success
    Serial.println("Success");
  } else if (delStatus == ESP_ERR_ESPNOW_NOT_INIT) {
    // How did we get so far!!
    Serial.println("ESPNOW Not Init");
  } else if (delStatus == ESP_ERR_ESPNOW_ARG) {
    Serial.println("Invalid Argument");
  } else if (delStatus == ESP_ERR_ESPNOW_NOT_FOUND) {
    Serial.println("Peer not found.");
  } else {
    Serial.println("Not sure what happened");
  }

  esp_err_t err = esp_now_add_peer(&peerInfo);
  if (err == ESP_OK || err == ESP_ERR_ESPNOW_EXIST){
    String cmd = doc["cmd"];
    Serial.printf("Received data OK: cmd: %s\n", cmd);

    if (cmd == "get-values")
    {
      // Send data as requested
      myDataSend.averageTempWater = averageTempWater;
      myDataSend.averageTempAir = averageTempAir;
      myDataSend.targetTemp = targetTemp;
      myDataSend.mode = mode;
      myDataSend.currentValveState = currentValveState;
      myDataSend.currentPumpState = currentPumpState;

      // Send message via ESP-NOW
      esp_err_t result = esp_now_send(esp_now_info->src_addr, (uint8_t *) &myDataSend, sizeof(myDataSend));
      if (result == ESP_OK) {
        Serial.println("Sent with success");
      }
      else {
        Serial.println("Error sending the data");
      }
    } else if (cmd == "set-mode")
    {
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
    } else if (cmd == "set-targettemp")
    {
      float targetTemp = doc["targetTemp"];
      targetTempChangedFunction(targetTemp);
    } else if (cmd == "set-deltatemp")
    {
      float deltaTemp = doc["deltaTemp"];
      deltaTempChangedFunction(deltaTemp);
    } else if (cmd == "set-offsetwater")
    {
      float offsetWater = doc["offsetWater"];
      offsetWaterChangedFunction(offsetWater);
    } else if (cmd == "set-offsetair")
    {
      float offsetAir = doc["offsetAir"];
      offsetAirChangedFunction(offsetAir);
    } else if (cmd == "set-sensordata")
    {
      if (doc["sensor"] == "temperature")
      {
        if (doc["location"] == "pavilion")
        {
          float value = doc["value"];
          int battery = doc["battery"];
          const char* firmware = doc["firmware"];
          publishPavilionSensorData(value, battery, firmware);
        }
        else if (doc["location"] == "greenhouse")
        {
          float value = doc["value"];
          int battery = doc["battery"];
          const char* firmware = doc["firmware"];
          publishGreenhouseSensorData(value, battery, firmware);
        }
      }
    }
  }
  else
  {
    Serial.println("Failed to add peer");
  } 
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

unsigned long lastPumpVelocityChange = 0;
void handlePumpControl(uint8_t velocity){
  if (now - lastPumpVelocityChange > PUMP_INTERVAL && currentPumpState != velocity)
  {
    switch(velocity) {
      case 0: digitalWrite(PUMP_OFF, LOW); currentPumpState = 0; break; // Pumpe stoppen
      case 1: digitalWrite(PUMP_1, LOW); currentPumpState = 1; break; // Stufe 1 = eco
      case 2: digitalWrite(PUMP_2, LOW); currentPumpState = 2; break; // Stufe 2
      case 3: digitalWrite(PUMP_3, LOW); currentPumpState = 3; break; // Stufe 3
    }
    Serial.print("Pumpe schaltet auf...");
    Serial.println(currentPumpState);
    publishPumpState(currentPumpState);

    delay(2000);
    digitalWrite(PUMP_OFF, HIGH);
    digitalWrite(PUMP_1, HIGH);
    digitalWrite(PUMP_2, HIGH);
    digitalWrite(PUMP_3, HIGH);
    lastPumpVelocityChange = now;
  }
}

unsigned long lastValveMovement = 0;
void handleValveAndPumpControl(String position) {
  Serial.printf("position: %s, now: %lu, lastValveMovement: %lu\nVALVE_INTERVAL: %lu, currentValveState: %lu\n", position, now, lastValveMovement, VALVE_INTERVAL, currentValveState);
  if (now - lastValveMovement <= VALVE_INTERVAL)
  {
    // Ventilbewegung läuft noch 
    return;
  }

  if (currentValveState != OPEN && currentValveState != CLOSED)
  {
    digitalWrite(RELAY_POS2, HIGH); // Abschalten nach delay
    digitalWrite(RELAY_POS4, HIGH); // Abschalten nach delay
    if (currentValveState == OPENING)
      currentValveState = OPEN;
    else if (currentValveState == CLOSING)
      currentValveState = CLOSED;
    lastValveMovement = 0;
    Serial.printf("position: %s, now: %lu, lastValveMovement: %lu, VALVE_INTERVAL: %lu\n", position, now, lastValveMovement, VALVE_INTERVAL);
    Serial.println("Ventilbewegung endet");
  } 

  if (position == "Pos2") {
      // Großer Kreislauf (Solar) aktivieren
      if (currentValveState != OPEN)
      {
        Serial.println("Ventil öffnet...");
        digitalWrite(RELAY_POS2, LOW);
        digitalWrite(RELAY_POS4, HIGH); // Kleiner Kreislauf deaktivieren
        lastValveMovement = now;
        currentValveState = OPENING;
      }
      handlePumpControl(PUMPLEVEL_FULL); // Oder Stufe2 basierend auf Bedingungen
  } else if (position == "Pos4") {
      // Kleiner Kreislauf aktivieren
      if (currentValveState != CLOSED)
      {
        Serial.println("Ventil schließt...");
        digitalWrite(RELAY_POS4, LOW);
        digitalWrite(RELAY_POS2, HIGH); // Großer Kreislauf deaktivieren
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

  // Betriebsmodi steuern
  if (mode == ON) {
      handleValveAndPumpControl("Pos2");
  } else if (mode == OFF || mode == CLEANING) {
      handleValveAndPumpControl("Pos4");
  } else if (mode == AUTO && validData) {
      if (averageTempAir-deltaTemp > averageTempWater && averageTempWater < targetTemp) {
          Serial.println("handleModes: AUTO - Pos2");
          handleValveAndPumpControl("Pos2"); // Großer Kreislauf aktivieren
      } else if (averageTempAir <= averageTempWater || currentValveState == UNDEFINED) {
          handleValveAndPumpControl("Pos4"); // Kleiner Kreislauf aktivieren
          Serial.println("handleModes: AUTO - Pos4");
      } else
      {
          handleValveAndPumpControl("----"); // Keine neue Ventilstellung, Ventilbewegung stoppen wenn nötig
          Serial.println("handleModes: AUTO - Pos2 in Delta-Temp");
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
  Serial.printf("WiFi signal: %d\n", WiFi.RSSI());
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
