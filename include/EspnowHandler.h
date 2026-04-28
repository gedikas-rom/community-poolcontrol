#pragma once

#include <stdint.h>

enum EspnowEventFlags : uint8_t {
  ESPNOW_EVENT_NONE = 0,
  ESPNOW_EVENT_BRIDGE_UART = 1 << 0,
  ESPNOW_EVENT_NODE_TRAFFIC = 1 << 1,
};

void setupEspnowHandler();
uint8_t loopEspnowHandler();
