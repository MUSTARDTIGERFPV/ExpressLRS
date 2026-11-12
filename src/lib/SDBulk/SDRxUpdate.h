#pragma once

#if defined(TARGET_TX) && defined(PLATFORM_ESP32)

#include "SDBulkLink.h"

// RX_UPDATE job: the RX has already been told over the RC link to stop and listen.
// Find it over ESP-NOW, pick its image off the handset's card (newest under
// /ELRS/FIRMWARE/<its key>/, or `path` if given), inflate if it's .gz, and send it.
bool runRxUpdate(SDBulkLink &link, const char *path, char *result, size_t resultLen);

#endif
