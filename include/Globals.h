#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <OneWireESP32.h>
#include <LiquidCrystal_PCF8574.h>
#include <GlobalDefs.h>

#ifndef GLOBALS_
#define GLOBALS_ 

const char* hostname = "poolcontrol";
const char* firmware = "0.4.2";

// Pin-Definitionen
#define BUTTON_PIN 16 // GPIO16 pin connected to button
#define RELAY_POS2 2    // 3-Wege-Ventil (Position offen = Pos2)
#define RELAY_POS4 21    // 3-Wege-Ventil (Position zu = Pos4)
#define PUMP_OFF 18     // Pumpe red
#define PUMP_1 20     // Pumpenstufe 1 brown
#define PUMP_2 19     // Pumpenstufe 2 green
#define PUMP_3 17     // Pumpenstufe 3 white   
#define TEMP_WATER_PIN 1    // DS18B20 Datenleitung Wassertemperatur
#define TEMP_AIR_PIN 0    // DS18B20 Datenleitung Lufttemperatur

#define DISPLAY_OFF_INTERVAL 600000 // Auto Display off after 10 minutes 
#define MEASUREMENT_INTERVAL 3000 // Refesh measurements and display refresh 
#define VALVE_INTERVAL 70000 // Valve movement time
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

struct_message_send myDataSend;

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

// LCD config
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

// WiFi
WiFiClient espClient;

#endif // GLOBALS_