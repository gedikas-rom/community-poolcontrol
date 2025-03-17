#include <Arduino.h>

void setupMQTT(WiFiClient& espClient, const char* firmware,    
    void (*modeChangedFunction)(Mode mode),
    void (*targetTempChangedFunction)(float targetTemp),
    void (*deltaTempChangedFunction)(float deltaTemp));
void loopMQTT();
void publishPreferences(Mode mode, ValveState valvestate, int state, float targetTemp, float deltaTemp);
void publishTemperatures(float averageTempWater, float averageTempAir);
void publishValveState(ValveState state);
void publishPumpState(int state);
void publishMode(Mode mode);
void publishTargetTemp(float targetTemp);
void publishDeltaTemp(float deltaTemp);