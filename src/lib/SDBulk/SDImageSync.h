#pragma once

#if defined(TARGET_TX) && defined(PLATFORM_ESP32)

#include "SDBulkLink.h"

// IMAGE_SYNC job: join WiFi as a station, fetch a release manifest over HTTP and
// fill /ELRS/FIRMWARE/<image-key>/<version>.bin.gz on the handset's card.
//
// manifest.json (URLs in "file" are relative to the manifest's own URL):
//   { "release": "4.1.0",
//     "images": { "esp32-2400": { "version": "4.1.0", "file": "esp32-2400/4.1.0.bin.gz",
//                                 "size": 712345, "sha256": "<64 hex>" }, ... } }
//
// Credentials and the manifest URL come from, in order: the session request,
// /ELRS/WIFI.TXT (ssid, password, manifest URL, one per line), the firmware options.

bool runImageSync(SDBulkLink &link, const char *manifestUrl, char *result, size_t resultLen);

#endif
