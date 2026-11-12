#pragma once

#if defined(TARGET_RX) && defined(PLATFORM_ESP32)

#include <stdint.h>

namespace RxUpdateReceiver
{
// Stop the RC link and wait for the TX over ESP-NOW. Refused while armed.
bool enter();
// main loop while connectionState == rxUpdate
void loop(uint32_t now);
}

#endif
