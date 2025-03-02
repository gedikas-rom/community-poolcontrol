void setupOTA(const char* hostname, 
    void (*startFunction)(void), 
    void (*progressFunction)(unsigned int, unsigned int), 
    void (*endFunction)(void), 
    void (*errorFunction)(ota_error_t));
void loopOTA();