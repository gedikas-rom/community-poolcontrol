enum ValveState {
    UNDEFINED,
    OPENING,
    OPEN,
    CLOSING,
    CLOSED
  };
  
enum Mode {
    AUTO,         // automatic mode
    ON,           // Solar heating on, pump full power
    OFF,          // Solar heating off, pump in eco mode (lowest level)
    CLEANING     // Solar heating off, pump full power for max cleaning
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
    tm lastUpdate; // Last update time
  } struct_message_send;