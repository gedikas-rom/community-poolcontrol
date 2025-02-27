#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_PCF8574.h>
#include <OneWireESP32.h>
#include <esp_now.h>
#include <WiFi.h>

// Pin-Definitionen
#define BUTTON_PIN 16 // GPIO16 pin connected to button
#define RELAY_POS2 2    // 3-Wege-Ventil (Position offen = Pos2)
#define RELAY_POS4 21    // 3-Wege-Ventil (Position zu = Pos4)
#define PUMP_OFF 18     // Pumpe 
#define PUMP_1 20     // Pumpenstufe 1
#define PUMP_2 19     // Pumpenstufe 2
#define PUMP_3 17     // Pumpenstufe 3
#define TEMP_WATER_PIN 1    // DS18B20 Datenleitung Wassertemperatur
#define TEMP_AIR_PIN 0    // DS18B20 Datenleitung Lufttemperatur

#define DISPLAY_OFF_INTERVAL 600000 // Auto Display off after 60 seconds 
#define MEASUREMENT_INTERVAL 3000 // Refesh measurements and display refresh 
#define VALVE_INTERVAL 40000 // Valve movement time
#define PUMP_INTERVAL 5000 // Pump movement time

// Variables will change:
int lastState = HIGH; // the previous state from the input pin
int currentState;     // the current reading from the input pin
unsigned long lastESPNOWRequest = 0;

// Setup a oneWire instances to communicate with any OneWire devices
uint8_t maxDevicesWater = 3;  // max devices per bus
uint8_t maxDevicesAir = 3;  // max devices per bus
const uint8_t maxDevices = 3;  // max devices per bus
OneWire32 dsWater(TEMP_WATER_PIN);
OneWire32 dsAir(TEMP_AIR_PIN);

enum ValveState {
  UNDEFINED,
  OPENING,
  OPEN,
  CLOSING,
  CLOSED
};

enum Mode {
  AUTO,   // automatic mode
  ON,     // Solar heating on, pump full power
  OFF     // Solar heating off, pump in eco mode (lowest level)
};

// ESP-NOW config
// Define data structure
typedef struct struct_message_send {
  float averageTempWater;
  float averageTempAir;
  float targetTemp;
  Mode mode;
  ValveState currentValveState;
  int currentPumpState;
} struct_message_send;

typedef struct struct_message_receive {
  char command[32];
} struct_message_receive;

struct_message_send myDataSend;
struct_message_receive myDataReceive;

uint64_t addrWater[maxDevices];
uint64_t addrAir[maxDevices];
float currTempWater[maxDevices];
float currTempAir[maxDevices];
float averageTempWater = -1;
float averageTempAir = -1;
float targetTemp = 25.0f; // Target water temperature
float deltaTemp = 2.0f; // Temp delta to prevent many valve switching
ValveState currentValveState = UNDEFINED;
int currentPumpState = 1;
Mode mode = AUTO; 

LiquidCrystal_PCF8574 lcd(0x27);  // set the LCD address to 0x27 for a 16 chars and 2 line display

// custom characters
byte dotOff[] = { 0b00000, 0b01110, 0b10001, 0b10001,
                  0b10001, 0b01110, 0b00000, 0b00000 };
byte dotOn[] = { 0b00000, 0b01110, 0b11111, 0b11111,
                 0b11111, 0b01110, 0b00000, 0b00000 };
byte wifi[] = { 0b00000, 0b00000, 0b11111, 0b00000,
                0b01110, 0b00000, 0b00100, 0b00000 };
byte grad[] = { 0b00100,0b01010,0b00100, 0b00000,
                0b00000,0b00000,0b00000,0b00000 };
byte delta[] = { 	0b00000, 0b00100, 0b00100, 0b01010,
	              0b01010, 0b10001, 0b11111, 0b00000 };
byte target[] = { 	0b00001, 0b00001, 0b00001, 0b00101,
	              0b00101, 0b00101, 0b10101, 0b10101 };
                
void OnDataRecv(const esp_now_recv_info_t *esp_now_info, const uint8_t *incomingData, int len);
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status);

void setup() {
  int error;

  Serial.begin(115200);
  Serial.println("LCD...");
  Serial.println("Probing for PCF8574 on address 0x27...");

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
  digitalWrite(PUMP_OFF, LOW);
  digitalWrite(PUMP_1, LOW);
  digitalWrite(PUMP_2, LOW);
  digitalWrite(PUMP_3, LOW);

  // Button init
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  // Wifi config
  WiFi.mode(WIFI_STA);
  Serial.print("MAC-Address: ");
  Serial.println(WiFi.macAddress());

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

// callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
  lastESPNOWRequest = millis();
}

// callback function that will be executed when data is received
void OnDataRecv(const esp_now_recv_info_t *esp_now_info, const uint8_t *incomingData, int len) {
  memcpy(&myDataReceive, incomingData, sizeof(myDataReceive));
  Serial.print("Message: ");
  Serial.println(myDataReceive.command);
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
    Serial.println("Received data OK");
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
  } else
  {
    Serial.println("Failed to add peer");
  }
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
      averageTempWater = averageTempWater + currTempWater[i];
      validMeasures++;
		}
	}
  if (validMeasures > 0)
    averageTempWater = averageTempWater/validMeasures;
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
      averageTempAir = averageTempAir + currTempAir[j];
      validMeasures++;
		}
	}
  if (validMeasures > 0)
    averageTempAir = averageTempAir/validMeasures;
  else 
    averageTempAir = -1;
}

unsigned long lastPumpVelocityChange = 0;
void handlePumpControl(uint8_t velocity){
  if (now - lastPumpVelocityChange > PUMP_INTERVAL && currentPumpState != velocity)
  {
    switch(velocity) {
      case 0: digitalWrite(PUMP_OFF, HIGH); currentPumpState = 0; break; // Pumpe stoppen
      case 1: digitalWrite(PUMP_1, HIGH); currentPumpState = 1; break; // Stufe 1 = eco
      case 2: digitalWrite(PUMP_2, HIGH); currentPumpState = 2; break; // Stufe 2
      case 3: digitalWrite(PUMP_3, HIGH); currentPumpState = 3; break; // Stufe 3
    }
    Serial.print("Pumpe schaltet auf...");
    Serial.println(currentPumpState);

    delay(1000);
    digitalWrite(PUMP_OFF, LOW);
    digitalWrite(PUMP_1, LOW);
    digitalWrite(PUMP_2, LOW);
    digitalWrite(PUMP_3, LOW);
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
      handlePumpControl(3); // Oder Stufe3 basierend auf Bedingungen
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
      handlePumpControl(1);
  }
}

void handleModes() {
  // Check valid sensor data
  bool validData =!(averageTempAir < 0 || averageTempAir > 50 || averageTempWater < 0 || averageTempWater > 50); 

  // Betriebsmodi steuern
  if (mode == ON) {
      handleValveAndPumpControl("Pos2");
  } else if (mode == OFF) {
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
    case AUTO: lcd.print("AUTO"); break;
  }
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
