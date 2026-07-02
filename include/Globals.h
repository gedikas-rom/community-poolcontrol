#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <OneWireESP32.h>
#include <LiquidCrystal_PCF8574.h>
#include <GlobalDefs.h>
#include <PCF8575.h>

#ifndef GLOBALS_
#define GLOBALS_ 

const char* hostname = "poolcontrol";
const char* firmware = "1.0.12"; // Firmware version

// Pin definitions
#define FILTER_PRESSURE_PIN 2   // Pin for filter pressure sensor (analog input)
#define BUTTON_PIN P13          // pin P10 on PCF8575 connected to button
#define RELAY_POS2 P15           // 3-way valve (open position = Pos2)
#define RELAY_POS4 P14           // 3-way valve (closed position = Pos4)
#define PUMP_OFF P0             // Pump off relay (red) pin P4 on PCF8575
#define PUMP_1 P1               // Pump level 1 relay (brown) pin P5 on PCF8575
#define PUMP_2 P2               // Pump level 2 relay (green) pin P6 on PCF8575
#define PUMP_3 P3               // Pump level 3 relay (white) pin P7 on PCF8575
#define TEMP_WATER_PIN 1        // DS18B20 water temperature data line
#define TEMP_AIR_PIN 0          // DS18B20 air temperature data line
#define PUMPLEVEL_ECO 1         // Pump level 1 = eco
#define PUMPLEVEL_FULL 2        // Pump level 2 = full power
#define PUMPLEVEL_CLEANING 2    // Pump level 2 = cleaning mode

#define DISPLAY_OFF_INTERVAL 600000 // Auto Display off after 10 minutes 
#define MEASUREMENT_INTERVAL 3000 // Refesh measurements and display refresh 
#define WIFI_RECONNECT_INTERVAL 5000
#define VALVE_INTERVAL 90000 // Valve movement time
#define PUMP_INTERVAL 5000 // Pump movement time

// Pressure sensor calibration (nominal sensor: 0.5V..4.5V => 0..4 MPa at sensor output)
// Voltage divider correction: Vadc = Vsensor * (R2 / (R1 + R2))
// Example values: R1=10k (sensor->ADC), R2=27k (ADC->GND)
// MIN_V is adjusted below the nominal 0.5V zero point for the installed sensor/divider.
#define FILTER_PRESSURE_R1_OHM 10000.0f
#define FILTER_PRESSURE_R2_OHM 27000.0f
#define FILTER_PRESSURE_SENSOR_MIN_V_DEFAULT 0.45f
#define FILTER_PRESSURE_SENSOR_MAX_V 4.5f
#define FILTER_PRESSURE_MIN_MPA 0.0f
#define FILTER_PRESSURE_MAX_MPA 4.0f
#define FILTER_PRESSURE_CALIBRATION_FACTOR_DEFAULT 1.8f
#define FILTER_PRESSURE_SAMPLE_COUNT 16

#define PCF8575_ADDRESS 0x20 // PCF8575 I2C address (default: 0x20, can be 0x21-0x27 depending on A0-A2 pin configuration)
#define LCD_ADDRESS 0x27 // LCD I2C address (default: 0x27, can be 0x3F depending on the module)

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
float filterPressureMpa = -1;
float filterPressureSensorMinV = FILTER_PRESSURE_SENSOR_MIN_V_DEFAULT;
float filterPressureCalibrationFactor = FILTER_PRESSURE_CALIBRATION_FACTOR_DEFAULT;
float offsetWater = 1.5f; // Offset for water temperature
float offsetAir = 0.0f; // Offset for air temperature
float targetTemp = 25.0f; // Target water temperature
float deltaTemp = 2.0f; // Temp delta to prevent many valve switching
ValveState currentValveState = UNDEFINED;
int currentPumpState = 1;
Mode mode = AUTO; 

// Time zone (DST-aware): Central European Time with summer time rules
const char* tzInfo = "CET-1CEST,M3.5.0/2,M10.5.0/3";
// NTP server to get the time
const char* ntpServer = "pool.ntp.org"; // NTP server address

// create PCF8575 object
PCF8575 pcf8575(PCF8575_ADDRESS);

// LCD config
LiquidCrystal_PCF8574 lcd(LCD_ADDRESS);  // set the LCD address to 0x27 for a 16 chars and 4 line display

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
