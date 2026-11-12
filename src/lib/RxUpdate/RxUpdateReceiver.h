#pragma once

#if defined(TARGET_RX) && (defined(PLATFORM_ESP32) || defined(PLATFORM_ESP8266))

#include <stdint.h>

namespace RxUpdateReceiver
{
// Stop the RC link and wait for the TX over ESP-NOW. Refused while armed.
bool enter();
// main loop while connectionState == rxUpdate
void loop(uint32_t now);
// End of setup(): listen for a TX's HELLO for BOOT_LISTEN_MS while the link starts,
// so an RX that can't link (another major version, say) can still be updated
void bootListen();
// every loop() while that lasts: a HELLO enters update mode, the first valid RC
// packet or the end of the window turns WiFi back off
void bootListenLoop(uint32_t now);
}

#endif
