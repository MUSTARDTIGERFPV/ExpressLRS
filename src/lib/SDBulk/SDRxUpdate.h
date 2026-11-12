#pragma once

#if defined(TARGET_TX) && defined(PLATFORM_ESP32)

#include "SDBulkLink.h"

// RX_UPDATE job. `linked`: the RX has already been told over the RC link to stop and
// listen; otherwise keep calling until it's powered on and hears us during its boot
// window. Find it over ESP-NOW, pick its image off the handset's card (newest under
// /ELRS/FIRMWARE/<its key>/, or `path` if given), inflate if it's .gz, and send it.
bool runRxUpdate(SDBulkLink &link, const char *path, bool linked, char *result, size_t resultLen);

#endif
