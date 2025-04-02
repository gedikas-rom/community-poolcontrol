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