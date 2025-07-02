// Homeassistant sensor config
#include <string>
// MQTT config
// Base topic for Homeassistant
const char* mqtt_topic_ha_base = "hass";

// Concatenated topics
std::string mqtt_topic_ha_firmware = std::string(mqtt_topic_ha_base) + "/sensor/poolcontrol/firmware/config";
std::string mqtt_topic_ha_set_mode = std::string(mqtt_topic_ha_base) + "/select/poolcontrol/mode/config";
std::string mqtt_topic_ha_set_targetTemp = std::string(mqtt_topic_ha_base) + "/number/poolcontrol/targetTemp/config";
std::string mqtt_topic_ha_set_deltaTemp = std::string(mqtt_topic_ha_base) + "/number/poolcontrol/deltaTemp/config";
std::string mqtt_topic_ha_set_offsetWater = std::string(mqtt_topic_ha_base) + "/number/poolcontrol/offsetWater/config";
std::string mqtt_topic_ha_set_offsetAir = std::string(mqtt_topic_ha_base) + "/number/poolcontrol/offsetAir/config";

std::string mqtt_topic_ha_TempWater = std::string(mqtt_topic_ha_base) + "/sensor/poolcontrol/TempWater/config";
std::string mqtt_topic_ha_TempAir = std::string(mqtt_topic_ha_base) + "/sensor/poolcontrol/TempAir/config";
std::string mqtt_topic_ha_ValveState = std::string(mqtt_topic_ha_base) + "/sensor/poolcontrol/valvestate/config";
std::string mqtt_topic_ha_PumpState = std::string(mqtt_topic_ha_base) + "/sensor/poolcontrol/pumpstate/config";
std::string mqtt_topic_ha_mode = std::string(mqtt_topic_ha_base) + "/sensor/poolcontrol/mode/config";
std::string mqtt_topic_ha_targetTemp = std::string(mqtt_topic_ha_base) + "/sensor/poolcontrol/targetTemp/config";
std::string mqtt_topic_ha_deltaTemp = std::string(mqtt_topic_ha_base) + "/sensor/poolcontrol/deltaTemp/config";
std::string mqtt_topic_ha_offsetWater = std::string(mqtt_topic_ha_base) + "/sensor/poolcontrol/offsetWater/config";
std::string mqtt_topic_ha_offsetAir = std::string(mqtt_topic_ha_base) + "/sensor/poolcontrol/offsetAir/config";

std::string mqtt_topic_ha_PavilionTemp = std::string(mqtt_topic_ha_base) + "/sensor/poolcontrol/PavilionTemp/config";
std::string mqtt_topic_ha_PavilionBattery = std::string(mqtt_topic_ha_base) + "/sensor/poolcontrol/PavilionBattery/config";
std::string mqtt_topic_ha_PavilionFirmware = std::string(mqtt_topic_ha_base) + "/sensor/poolcontrol/PavilionFirmware/config";
std::string mqtt_topic_ha_GreenhouseTemp = std::string(mqtt_topic_ha_base) + "/sensor/poolcontrol/GreenhouseTemp/config";
std::string mqtt_topic_ha_GreenhouseBattery = std::string(mqtt_topic_ha_base) + "/sensor/poolcontrol/GreenhouseBattery/config";
std::string mqtt_topic_ha_GreenhouseFirmware = std::string(mqtt_topic_ha_base) + "/sensor/poolcontrol/GreenhouseFirmware/config";


const char* mqtt_ha_config_firmware = R"rawliteral({
  "availability": [
    {
      "topic": "poolcontrol/state"
    }
  ],
  "availability_mode": "all",
  "device": {
    "identifiers": [
      "poolcontrol"
    ],
    "manufacturer": "Ron",
    "model": "PoolControl V1",
    "name": "Poolcontrol"
  },
  "enabled_by_default": true,
  "entity_category": "diagnostic",
  "object_id": "poolcontrol_firmware",
  "origin": {
    "name": "ESP32-C6",
    "sw": "1.0.0",
    "url": "https://wiki.seeedstudio.com/xiao_pin_multiplexing_esp33c6"
  },
  "name": "Firmware",
  "state_topic": "poolcontrol/firmware",
  "unique_id": "poolcontrol_firmware",
  "value_template": "{{ value }}"
})rawliteral";

const char* mqtt_ha_config_set_mode = R"rawliteral({
  "availability": [
    {
      "topic": "poolcontrol/state"
    }
  ],
  "availability_mode": "all",
  "device": {
    "identifiers": [
      "poolcontrol"
    ],
    "manufacturer": "Ron",
    "model": "PoolControl V1",
    "name": "Poolcontrol"
  },
  "command_topic": "poolcontrol/set/mode",
  "enabled_by_default": true,
  "entity_category": "config",
  "icon": "mdi:tune",
  "name": "Modus",
  "object_id": "poolcontrol_set_mode",
  "options": [
    "AUTO",
    "ON",
    "OFF",
    "CLEANING"
  ],
  "origin": {
    "name": "ESP32-C6",
    "sw": "1.0.0",
    "url": "https://wiki.seeedstudio.com/xiao_pin_multiplexing_esp33c6"
  },
  "state_topic": "poolcontrol/mode",
  "unique_id": "poolcontrol_set_mode",
  "value_template": "{{ value }}"
})rawliteral";

const char* mqtt_ha_config_set_targetTemp = R"rawliteral({
  "availability": [
    {
      "topic": "poolcontrol/state"
    }
  ],
  "availability_mode": "all",
  "device": {
    "identifiers": [
      "poolcontrol"
    ],
    "manufacturer": "Ron",
    "model": "PoolControl V1",
    "name": "Poolcontrol"
  },
  "command_topic": "poolcontrol/set/targettemp",
  "min": 10,
  "max": 35,
  "enabled_by_default": true,
  "device_class": "temperature",
  "entity_category": "config",
  "icon": "mdi:tune",
  "name": "Solltemperatur",
  "object_id": "poolcontrol_set_targetTemp",
  "origin": {
    "name": "ESP32-C6",
    "sw": "1.0.0",
    "url": "https://wiki.seeedstudio.com/xiao_pin_multiplexing_esp33c6"
  },
  "state_topic": "poolcontrol/TargetTemp",
  "unit_of_measurement": "°C",
  "unique_id": "poolcontrol_set_targetTemp",
  "value_template": "{{ value }}"
})rawliteral";

const char* mqtt_ha_config_set_deltaTemp = R"rawliteral({
  "availability": [
    {
      "topic": "poolcontrol/state"
    }
  ],
  "availability_mode": "all",
  "device": {
    "identifiers": [
      "poolcontrol"
    ],
    "manufacturer": "Ron",
    "model": "PoolControl V1",
    "name": "Poolcontrol"
  },
  "command_topic": "poolcontrol/set/deltatemp",
  "min": 1,
  "max": 10,
  "enabled_by_default": true,
  "device_class": "temperature",
  "entity_category": "config",
  "icon": "mdi:tune",
  "name": "Temperaturdifferenz",
  "object_id": "poolcontrol_set_deltaTemp",
  "origin": {
    "name": "ESP32-C6",
    "sw": "1.0.0",
    "url": "https://wiki.seeedstudio.com/xiao_pin_multiplexing_esp33c6"
  },
  "state_topic": "poolcontrol/DeltaTemp",
  "unit_of_measurement": "K",
  "unique_id": "poolcontrol_set_deltaTemp",
  "value_template": "{{ value }}"
})rawliteral";

const char* mqtt_ha_config_set_offsetWater = R"rawliteral({
  "availability": [
    {
      "topic": "poolcontrol/state"
    }
  ],
  "availability_mode": "all",
  "device": {
    "identifiers": [
      "poolcontrol"
    ],
    "manufacturer": "Ron",
    "model": "PoolControl V1",
    "name": "Poolcontrol"
  },
  "command_topic": "poolcontrol/set/offsetWater",
  "mode": "slider",
  "min": -5,
  "max": 5,
  "step": 0.1,
  "enabled_by_default": true,
  "device_class": "temperature",
  "entity_category": "config",
  "icon": "mdi:tune",
  "name": "Offset Wassertemperatur",
  "object_id": "poolcontrol_set_offsetWater",
  "origin": {
    "name": "ESP32-C6",
    "sw": "1.0.0",
    "url": "https://wiki.seeedstudio.com/xiao_pin_multiplexing_esp33c6"
  },
  "state_topic": "poolcontrol/offsetWater",
  "unit_of_measurement": "",
  "unique_id": "poolcontrol_set_offsetWater",
  "value_template": "{{ value }}"
})rawliteral";

const char* mqtt_ha_config_set_offsetAir = R"rawliteral({
  "availability": [
    {
      "topic": "poolcontrol/state"
    }
  ],
  "availability_mode": "all",
  "device": {
    "identifiers": [
      "poolcontrol"
    ],
    "manufacturer": "Ron",
    "model": "PoolControl V1",
    "name": "Poolcontrol"
  },
  "command_topic": "poolcontrol/set/offsetAir",
  "mode": "slider",
  "min": -5,
  "max": 5,
  "step": 0.1,
  "enabled_by_default": true,
  "device_class": "temperature",
  "entity_category": "config",
  "icon": "mdi:tune",
  "name": "Offset Lufttemperatur",
  "object_id": "poolcontrol_set_offsetAir",
  "origin": {
    "name": "ESP32-C6",
    "sw": "1.0.0",
    "url": "https://wiki.seeedstudio.com/xiao_pin_multiplexing_esp33c6"
  },
  "state_topic": "poolcontrol/offsetAir",
  "unit_of_measurement": "",
  "unique_id": "poolcontrol_set_offsetAir",
  "value_template": "{{ value }}"
})rawliteral";

const char* mqtt_ha_config_TempWater = R"rawliteral({
  "availability": [
    {
      "topic": "poolcontrol/state"
    }
  ],
  "availability_mode": "all",
  "device": {
    "identifiers": [
      "poolcontrol"
    ],
    "manufacturer": "Ron",
    "model": "PoolControl V1",
    "name": "Poolcontrol"
  },
  "enabled_by_default": true,
  "device_class": "temperature",
  "name": "Wassertemperatur",
  "object_id": "poolcontrol_TempWater",
  "origin": {
    "name": "ESP32-C6",
    "sw": "1.0.0",
    "url": "https://wiki.seeedstudio.com/xiao_pin_multiplexing_esp33c6"
  },
  "state_topic": "poolcontrol/TempWater",
  "unit_of_measurement": "°C",
  "unique_id": "poolcontrol_TempWater",
  "value_template": "{{ value }}"
})rawliteral";

const char* mqtt_ha_config_TempAir = R"rawliteral({
  "availability": [
    {
      "topic": "poolcontrol/state"
    }
  ],
  "availability_mode": "all",
  "device": {
    "identifiers": [
      "poolcontrol"
    ],
    "manufacturer": "Ron",
    "model": "PoolControl V1",
    "name": "Poolcontrol"
  },
  "enabled_by_default": true,
  "device_class": "temperature",
  "name": "Lufttemperatur",
  "object_id": "poolcontrol_TempAir",
  "origin": {
    "name": "ESP32-C6",
    "sw": "1.0.0",
    "url": "https://wiki.seeedstudio.com/xiao_pin_multiplexing_esp33c6"
  },
  "state_topic": "poolcontrol/TempAir",
  "unit_of_measurement": "°C",
  "unique_id": "poolcontrol_TempAir",
  "value_template": "{{ value }}"
})rawliteral";

const char* mqtt_ha_config_ValveState = R"rawliteral({
  "availability": [
    {
      "topic": "poolcontrol/state"
    }
  ],
  "availability_mode": "all",
  "device": {
    "identifiers": [
      "poolcontrol"
    ],
    "manufacturer": "Ron",
    "model": "PoolControl V1",
    "name": "Poolcontrol"
  },
  "enabled_by_default": true,
  "icon": "mdi:valve",
  "name": "Ventilstellung",
  "object_id": "poolcontrol_ValveState",
  "origin": {
    "name": "ESP32-C6",
    "sw": "1.0.0",
    "url": "https://wiki.seeedstudio.com/xiao_pin_multiplexing_esp33c6"
  },
  "state_topic": "poolcontrol/valvestate",
  "unique_id": "poolcontrol_ValveState",
  "value_template": "{{ value }}"
})rawliteral";

const char* mqtt_ha_config_PumpState = R"rawliteral({
  "availability": [
    {
      "topic": "poolcontrol/state"
    }
  ],
  "availability_mode": "all",
  "device": {
    "identifiers": [
      "poolcontrol"
    ],
    "manufacturer": "Ron",
    "model": "PoolControl V1",
    "name": "Poolcontrol"
  },
  "enabled_by_default": true,
  "icon": "mdi:pump",
  "name": "Pumpenstufe",
  "object_id": "poolcontrol_PumpState",
  "origin": {
    "name": "ESP32-C6",
    "sw": "1.0.0",
    "url": "https://wiki.seeedstudio.com/xiao_pin_multiplexing_esp33c6"
  },
  "state_topic": "poolcontrol/pumpstate",
  "unique_id": "poolcontrol_PumpState",
  "value_template": "{{ value }}"
})rawliteral";

const char* mqtt_ha_config_mode = R"rawliteral({
  "availability": [
    {
      "topic": "poolcontrol/state"
    }
  ],
  "availability_mode": "all",
  "device": {
    "identifiers": [
      "poolcontrol"
    ],
    "manufacturer": "Ron",
    "model": "PoolControl V1",
    "name": "Poolcontrol"
  },
  "enabled_by_default": true,
  "icon": "mdi:tune",
  "name": "Modus",
  "object_id": "poolcontrol_mode",
  "origin": {
    "name": "ESP32-C6",
    "sw": "1.0.0",
    "url": "https://wiki.seeedstudio.com/xiao_pin_multiplexing_esp33c6"
  },
  "state_topic": "poolcontrol/mode",
  "unique_id": "poolcontrol_mode",
  "value_template": "{{ value }}"
})rawliteral";

const char* mqtt_ha_config_targetTemp = R"rawliteral({
  "availability": [
    {
      "topic": "poolcontrol/state"
    }
  ],
  "availability_mode": "all",
  "device": {
    "identifiers": [
      "poolcontrol"
    ],
    "manufacturer": "Ron",
    "model": "PoolControl V1",
    "name": "Poolcontrol"
  },
  "enabled_by_default": true,
  "device_class": "temperature",
  "entity_category": "diagnostic",
  "icon": "mdi:tune",
  "name": "Solltemperatur",
  "object_id": "poolcontrol_targetTemp",
  "origin": {
    "name": "ESP32-C6",
    "sw": "1.0.0",
    "url": "https://wiki.seeedstudio.com/xiao_pin_multiplexing_esp33c6"
  },
  "state_topic": "poolcontrol/TargetTemp",
  "unit_of_measurement": "°C",
  "unique_id": "poolcontrol_targetTemp",
  "value_template": "{{ value }}"
})rawliteral";

const char* mqtt_ha_config_deltaTemp = R"rawliteral({
  "availability": [
    {
      "topic": "poolcontrol/state"
    }
  ],
  "availability_mode": "all",
  "device": {
    "identifiers": [
      "poolcontrol"
    ],
    "manufacturer": "Ron",
    "model": "PoolControl V1",
    "name": "Poolcontrol"
  },
  "enabled_by_default": true,
  "device_class": "temperature",
  "entity_category": "diagnostic",
  "icon": "mdi:tune",
  "name": "Temperaturdifferenz",
  "object_id": "poolcontrol_deltaTemp",
  "origin": {
    "name": "ESP32-C6",
    "sw": "1.0.0",
    "url": "https://wiki.seeedstudio.com/xiao_pin_multiplexing_esp33c6"
  },
  "state_topic": "poolcontrol/DeltaTemp",
  "unit_of_measurement": "K",
  "unique_id": "poolcontrol_deltaTemp",
  "value_template": "{{ value }}"
})rawliteral";

const char* mqtt_ha_config_offsetWater = R"rawliteral({
  "availability": [
    {
      "topic": "poolcontrol/state"
    }
  ],
  "availability_mode": "all",
  "device": {
    "identifiers": [
      "poolcontrol"
    ],
    "manufacturer": "Ron",
    "model": "PoolControl V1",
    "name": "Poolcontrol"
  },
  "enabled_by_default": true,
  "device_class": "temperature",
  "entity_category": "diagnostic",
  "icon": "mdi:tune",
  "name": "Offset Wassertemperatur",
  "object_id": "poolcontrol_offsetWater",
  "origin": {
    "name": "ESP32-C6",
    "sw": "1.0.0",
    "url": "https://wiki.seeedstudio.com/xiao_pin_multiplexing_esp33c6"
  },
  "state_topic": "poolcontrol/offsetWater",
  "unit_of_measurement": "",
  "unique_id": "poolcontrol_offsetWater",
  "value_template": "{{ value }}"
})rawliteral";

const char* mqtt_ha_config_offsetAir = R"rawliteral({
  "availability": [
    {
      "topic": "poolcontrol/state"
    }
  ],
  "availability_mode": "all",
  "device": {
    "identifiers": [
      "poolcontrol"
    ],
    "manufacturer": "Ron",
    "model": "PoolControl V1",
    "name": "Poolcontrol"
  },
  "enabled_by_default": true,
  "device_class": "temperature",
  "entity_category": "diagnostic",
  "icon": "mdi:tune",
  "name": "Offset Lufttemperatur",
  "object_id": "poolcontrol_offsetAir",
  "origin": {
    "name": "ESP32-C6",
    "sw": "1.0.0",
    "url": "https://wiki.seeedstudio.com/xiao_pin_multiplexing_esp33c6"
  },
  "state_topic": "poolcontrol/offsetAir",
  "unit_of_measurement": "",
  "unique_id": "poolcontrol_offsetAir",
  "value_template": "{{ value }}"
})rawliteral";

const char* mqtt_ha_config_PavilionTemp = R"rawliteral({
  "availability": [
    {
      "topic": "poolcontrol/state"
    }
  ],
  "availability_mode": "all",
  "device": {
    "identifiers": [
      "poolcontrol"
    ],
    "manufacturer": "Ron",
    "model": "PoolControl V1",
    "name": "Poolcontrol"
  },
  "enabled_by_default": true,
  "device_class": "temperature",
  "name": "Pavillion Temperatur",
  "object_id": "poolcontrol_PavilionTemp",
  "origin": {
    "name": "ESP32-C6",
    "sw": "1.0.0",
    "url": "https://wiki.seeedstudio.com/xiao_pin_multiplexing_esp33c6"
  },
  "state_topic": "poolcontrol/PavilionTemp",
  "unit_of_measurement": "°C",
  "unique_id": "poolcontrol_PavilionTemp",
  "value_template": "{{ value }}"
})rawliteral";

const char* mqtt_ha_config_PavilionBattery = R"rawliteral({
  "availability": [
    {
      "topic": "poolcontrol/state"
    }
  ],
  "availability_mode": "all",
  "device": {
    "identifiers": [
      "poolcontrol"
    ],
    "manufacturer": "Ron",
    "model": "PoolControl V1",
    "name": "Poolcontrol"
  },
  "enabled_by_default": true,
  "device_class": "battery",
  "entity_category": "diagnostic",
  "name": "Pavillion Batterie",
  "object_id": "poolcontrol_PavilionBattery",
  "origin": {
    "name": "ESP32-C6",
    "sw": "1.0.0",
    "url": "https://wiki.seeedstudio.com/xiao_pin_multiplexing_esp33c6"
  },
  "state_topic": "poolcontrol/PavilionBattery",
  "unit_of_measurement": "%",
  "unique_id": "poolcontrol_PavilionBattery",
  "value_template": "{{ value }}"
})rawliteral";

const char* mqtt_ha_config_PavilionFirmware = R"rawliteral({
  "availability": [
    {
      "topic": "poolcontrol/state"
    }
  ],
  "availability_mode": "all",
  "device": {
    "identifiers": [
      "poolcontrol"
    ],
    "manufacturer": "Ron",
    "model": "PoolControl V1",
    "name": "Poolcontrol"
  },
  "enabled_by_default": true,
  "entity_category": "diagnostic",
  "name": "Pavillion Firmware",
  "object_id": "poolcontrol_PavilionFirmware",
  "origin": {
    "name": "ESP32-C6",
    "sw": "1.0.0",
    "url": "https://wiki.seeedstudio.com/xiao_pin_multiplexing_esp33c6"
  },
  "state_topic": "poolcontrol/PavilionFirmware",
  "unique_id": "poolcontrol_PavilionFirmware",
  "value_template": "{{ value }}"
})rawliteral";

const char* mqtt_ha_config_GreenhouseTemp = R"rawliteral({
  "availability": [
    {
      "topic": "poolcontrol/state"
    }
  ],
  "availability_mode": "all",
  "device": {
    "identifiers": [
      "poolcontrol"
    ],
    "manufacturer": "Ron",
    "model": "PoolControl V1",
    "name": "Poolcontrol"
  },
  "enabled_by_default": true,
  "device_class": "temperature",
  "name": "Gewächshaus Temperatur",
  "object_id": "poolcontrol_GreenhouseTemp",
  "origin": {
    "name": "ESP32-C6",
    "sw": "1.0.0",
    "url": "https://wiki.seeedstudio.com/xiao_pin_multiplexing_esp33c6"
  },
  "state_topic": "poolcontrol/GreenhouseTemp",
  "unit_of_measurement": "°C",
  "unique_id": "poolcontrol_GreenhouseTemp",
  "value_template": "{{ value }}"
})rawliteral";

const char* mqtt_ha_config_GreenhouseBattery = R"rawliteral({
  "availability": [
    {
      "topic": "poolcontrol/state"
    }
  ],
  "availability_mode": "all",
  "device": {
    "identifiers": [
      "poolcontrol"
    ],
    "manufacturer": "Ron",
    "model": "PoolControl V1",
    "name": "Poolcontrol"
  },
  "enabled_by_default": true,
  "device_class": "battery",
  "entity_category": "diagnostic",
  "name": "Gewächshaus Batterie",
  "object_id": "poolcontrol_GreenhouseBattery",
  "origin": {
    "name": "ESP32-C6",
    "sw": "1.0.0",
    "url": "https://wiki.seeedstudio.com/xiao_pin_multiplexing_esp33c6"
  },
  "state_topic": "poolcontrol/GreenhouseBattery",
  "unit_of_measurement": "%",
  "unique_id": "poolcontrol_GreenhouseBattery",
  "value_template": "{{ value }}"
})rawliteral";

const char* mqtt_ha_config_GreenhouseFirmware = R"rawliteral({
  "availability": [
    {
      "topic": "poolcontrol/state"
    }
  ],
  "availability_mode": "all",
  "device": {
    "identifiers": [
      "poolcontrol"
    ],
    "manufacturer": "Ron",
    "model": "PoolControl V1",
    "name": "Poolcontrol"
  },
  "enabled_by_default": true,
  "entity_category": "diagnostic",
  "name": "Gewächshaus Firmware",
  "object_id": "poolcontrol_GreenhouseFirmware",
  "origin": {
    "name": "ESP32-C6",
    "sw": "1.0.0",
    "url": "https://wiki.seeedstudio.com/xiao_pin_multiplexing_esp33c6"
  },
  "state_topic": "poolcontrol/GreenhouseFirmware",
  "unique_id": "poolcontrol_GreenhouseFirmware",
  "value_template": "{{ value }}"
})rawliteral";