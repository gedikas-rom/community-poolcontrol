#include <ArduinoOTA.h>

void setupOTA(const char* hostname, 
    void (*startFunction)(void), 
    void (*progressFunction)(unsigned int, unsigned int), 
    void (*endFunction)(void), 
    void (*errorFunction)(ota_error_t)) 
{
    ArduinoOTA.setHostname(hostname);
    
    ArduinoOTA
      .onStart(startFunction)
      .onProgress(progressFunction)
      .onEnd(endFunction)
      .onError(errorFunction);
  
    ArduinoOTA.begin();
}

void loopOTA() {
    ArduinoOTA.handle();
}