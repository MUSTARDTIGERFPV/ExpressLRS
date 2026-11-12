#if defined(TARGET_TX) && defined(PLATFORM_ESP32)

#include "SDImageSync.h"

#include <Arduino.h>
#include "logging.h"
#include "options.h"

#include <ArduinoJson.h>
#include <HTTPClient.h>
#include <Preferences.h>
#include <WiFi.h>
#include <mbedtls/sha256.h>

// The handset gives up on a silent module after 3 s; stay well inside that
static constexpr uint32_t KEEPALIVE_MS = 800;
static constexpr uint32_t WIFI_JOIN_TIMEOUT_MS = 20000;
static constexpr uint32_t HTTP_STALL_TIMEOUT_MS = 10000;

static uint32_t lastCall;

static int keepalive(SDBulkLink &link, const char *text, uint32_t done = 0, uint32_t total = 0, bool force = false)
{
    if (!force && millis() - lastCall < KEEPALIVE_MS)
        return SDBulk::OK;
    lastCall = millis();
    return link.progress(done, total, text);
}

static bool safeName(const char *s)
{
    if (!*s || strlen(s) > 24)
        return false;
    for (; *s; s++)
        if (!isalnum((unsigned char)*s) && *s != '.' && *s != '_' && *s != '-')
            return false;
    return !strstr(s, "..");
}

// small whole-file read off the card, NUL terminated; false if missing
static bool readSmallFile(SDBulkLink &link, const char *path, char *out, size_t outLen)
{
    uint8_t h;
    uint32_t size;
    if (link.open(path, SDBulk::OPEN_READ, &h, &size) != SDBulk::OK)
        return false;
    uint32_t off = 0;
    while (off < size && off < outLen - 1)
    {
        uint16_t got = 0;
        const uint16_t want = std::min<uint32_t>(link.chunkSize(), outLen - 1 - off);
        if (link.read(h, off, (uint8_t *)out + off, want, &got) != SDBulk::OK || !got)
            break;
        off += got;
    }
    link.close(h);
    out[off] = '\0';
    return true;
}

static bool writeFileAtomically(SDBulkLink &link, const char *path, const uint8_t *data, size_t len)
{
    char part[72];
    snprintf(part, sizeof(part), "%s.part", path);
    uint8_t h;
    if (link.open(part, SDBulk::OPEN_TRUNCATE, &h, nullptr) != SDBulk::OK)
        return false;
    bool ok = true;
    for (size_t off = 0; ok && off < len; off += link.chunkSize())
        ok = link.write(h, off, data + off, std::min<size_t>(link.chunkSize(), len - off)) == SDBulk::OK;
    link.close(h);
    return ok && link.rename(part, path) == SDBulk::OK;
}

static bool joinWifi(SDBulkLink &link, const char *ssid, const char *password)
{
    WiFi.persistent(false);
    WiFi.mode(WIFI_STA);
    // several APs share one SSID here: scan them all and take the strongest
    WiFi.setScanMethod(WIFI_ALL_CHANNEL_SCAN);
    WiFi.setSortMethod(WIFI_CONNECT_AP_BY_SIGNAL);
    WiFi.begin(ssid, password);

    char text[33];
    snprintf(text, sizeof(text), "Joining %.24s", ssid);
    keepalive(link, text, 0, 0, true);
    const uint32_t start = millis();
    while (WiFi.status() != WL_CONNECTED)
    {
        if (millis() - start > WIFI_JOIN_TIMEOUT_MS)
            return false;
        if (keepalive(link, text) == SDBulk::CANCELLED)
            return false;
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    DBGLN("SD sync: joined %s as %s, RSSI %d", ssid, WiFi.localIP().toString().c_str(), WiFi.RSSI());
    return true;
}

static void stopWifi()
{
    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);
}

enum FetchResult
{
    FETCH_OK,
    FETCH_CURRENT,
    FETCH_FAILED,
    FETCH_CANCELLED,
};

static uint8_t chunk[SDBulk::MAX_PAYLOAD];

// Streams one image from HTTP into <dir>/<version>.bin.gz.part, checks size and
// SHA-256, then renames it into place. A bad download never gets the real name.
static FetchResult fetchImage(SDBulkLink &link, const String &url, const char *key, const char *version,
                              uint32_t size, const char *sha256hex, uint32_t &bytesFetched)
{
    char dir[40], path[72], part[80];
    snprintf(dir, sizeof(dir), "FIRMWARE/%s", key);
    snprintf(path, sizeof(path), "%s/%s.bin.gz", dir, version);
    snprintf(part, sizeof(part), "%s.part", path);

    // Already there with the right size: we only ever rename after a hash check
    uint32_t have = 0;
    bool isDir = false;
    if (link.stat(path, &have, &isDir) == SDBulk::OK && !isDir && have == size)
        return FETCH_CURRENT;

    link.mkdir(dir);
    uint8_t h;
    if (link.open(part, SDBulk::OPEN_TRUNCATE, &h, nullptr) != SDBulk::OK)
        return FETCH_FAILED;

    HTTPClient http;
    WiFiClient client;
    http.setTimeout(HTTP_STALL_TIMEOUT_MS);
    http.begin(client, url);
    const int code = http.GET();
    if (code != HTTP_CODE_OK)
    {
        DBGLN("SD sync: GET %s -> %d", url.c_str(), code);
        http.end();
        link.close(h);
        link.remove(part);
        return FETCH_FAILED;
    }

    mbedtls_sha256_context sha;
    mbedtls_sha256_init(&sha);
    mbedtls_sha256_starts_ret(&sha, 0);

    WiFiClient *stream = http.getStreamPtr();
    const uint16_t step = link.chunkSize();
    uint32_t off = 0, lastByte = millis(), lastShown = 0;
    uint16_t fill = 0;
    FetchResult res = FETCH_OK;
    char text[33];
    while (off + fill < size)
    {
        const int avail = stream->available();
        if (avail > 0)
        {
            fill += stream->read(chunk + fill, std::min<uint32_t>(std::min(avail, step - fill), size - off - fill));
            lastByte = millis();
        }
        else if (!stream->connected() || millis() - lastByte > HTTP_STALL_TIMEOUT_MS)
        {
            res = FETCH_FAILED;
            break;
        }

        const bool last = off + fill == size;
        if (fill == step || (last && fill))
        {
            mbedtls_sha256_update_ret(&sha, chunk, fill);
            if (link.write(h, off, chunk, fill) != SDBulk::OK)
            {
                res = FETCH_FAILED;
                break;
            }
            lastCall = millis();
            off += fill;
            fill = 0;
        }

        if (off - lastShown >= 16384 || millis() - lastCall > KEEPALIVE_MS)
        {
            snprintf(text, sizeof(text), "%.14s %u/%u KB", key, off / 1024, size / 1024);
            if (keepalive(link, text, off, size, true) == SDBulk::CANCELLED)
            {
                res = FETCH_CANCELLED;
                break;
            }
            lastShown = off;
        }
        if (avail <= 0)
            vTaskDelay(1);
    }
    http.end();
    link.close(h);
    bytesFetched += off;

    uint8_t digest[32];
    mbedtls_sha256_finish_ret(&sha, digest);
    mbedtls_sha256_free(&sha);
    char hex[65];
    for (int i = 0; i < 32; i++)
        sprintf(&hex[i * 2], "%02x", digest[i]);

    if (res == FETCH_OK && (off != size || strcasecmp(hex, sha256hex) != 0))
    {
        DBGLN("SD sync: %s hash/size mismatch (%u/%u)", key, off, size);
        res = FETCH_FAILED;
    }
    if (res != FETCH_OK)
    {
        link.remove(part);
        return res;
    }
    return link.rename(part, path) == SDBulk::OK ? FETCH_OK : FETCH_FAILED;
}

bool runImageSync(SDBulkLink &link, const char *manifestUrl, char *result, size_t resultLen)
{
    lastCall = millis();
    uint32_t freeKb = 0;
    if (link.hello(&freeKb) != SDBulk::OK)
    {
        snprintf(result, resultLen, "HELLO failed");
        return false;
    }

    // credentials: WIFI.TXT on the card wins over the flashed home network
    static char wifiTxt[256];
    String ssid = firmwareOptions.home_wifi_ssid;
    String password = firmwareOptions.home_wifi_password;
    String url = manifestUrl ? manifestUrl : "";
    if (readSmallFile(link, "WIFI.TXT", wifiTxt, sizeof(wifiTxt)))
    {
        char *save = nullptr;
        const char *l1 = strtok_r(wifiTxt, "\r\n", &save);
        const char *l2 = strtok_r(nullptr, "\r\n", &save);
        const char *l3 = strtok_r(nullptr, "\r\n", &save);
        if (l1 && l2)
        {
            ssid = l1;
            password = l2;
        }
        if (l3 && url.isEmpty())
            url = l3;
    }
    // No URL from the request or the card: use the last one that synced fine. Until
    // releases publish a manifest at a fixed address, that's the menu's default.
    Preferences prefs;
    prefs.begin("sdsync", false);
    if (url.isEmpty())
        url = prefs.getString("url", "");
    if (ssid.isEmpty() || url.isEmpty())
    {
        snprintf(result, resultLen, ssid.isEmpty() ? "no WiFi network set" : "no manifest URL set");
        link.end(1, result);
        return false;
    }

    if (!joinWifi(link, ssid.c_str(), password.c_str()))
    {
        stopWifi();
        snprintf(result, resultLen, "could not join %.24s", ssid.c_str());
        link.end(1, result);
        return false;
    }

    // the manifest is small; take it whole
    keepalive(link, "Fetching manifest", 0, 0, true);
    String manifest;
    {
        HTTPClient http;
        WiFiClient client;
        http.setTimeout(HTTP_STALL_TIMEOUT_MS);
        http.begin(client, url);
        const int code = http.GET();
        if (code == HTTP_CODE_OK)
            manifest = http.getString();
        http.end();
        if (code != HTTP_CODE_OK)
        {
            stopWifi();
            snprintf(result, resultLen, "manifest GET %d", code);
            link.end(1, result);
            return false;
        }
    }
    lastCall = 0;
    keepalive(link, "Reading manifest");

    JsonDocument doc;
    if (deserializeJson(doc, manifest) != DeserializationError::Ok || !doc["images"].is<JsonObject>())
    {
        stopWifi();
        snprintf(result, resultLen, "bad manifest");
        link.end(1, result);
        return false;
    }

    const String base = url.substring(0, url.lastIndexOf('/') + 1);
    unsigned fetched = 0, current = 0, failed = 0;
    uint32_t bytes = 0;
    const uint32_t t0 = millis();
    bool cancelled = false;
    for (JsonPair kv : doc["images"].as<JsonObject>())
    {
        const char *key = kv.key().c_str();
        const char *version = kv.value()["version"] | "";
        const char *file = kv.value()["file"] | "";
        const char *sha = kv.value()["sha256"] | "";
        const uint32_t size = kv.value()["size"] | 0;
        if (!safeName(key) || !safeName(version) || !*file || strlen(sha) != 64 || !size)
        {
            failed++;
            continue;
        }
        const String imageUrl = strncmp(file, "http", 4) == 0 ? String(file) : base + file;
        switch (fetchImage(link, imageUrl, key, version, size, sha, bytes))
        {
        case FETCH_OK: fetched++; break;
        case FETCH_CURRENT: current++; break;
        case FETCH_CANCELLED: cancelled = true; break;
        default: failed++; break;
        }
        if (cancelled)
            break;
    }
    const uint32_t dt = millis() - t0;
    stopWifi();

    // keep a copy of what we synced against: it describes what's on the card, so
    // only when every image in it actually landed
    if (!cancelled && !failed)
    {
        link.mkdir("FIRMWARE");
        writeFileAtomically(link, "FIRMWARE/manifest.json", (const uint8_t *)manifest.c_str(), manifest.length());
    }

    if (!cancelled && !failed)
        prefs.putString("url", url);
    prefs.end();

    if (cancelled)
        snprintf(result, resultLen, "Cancelled");
    else
        snprintf(result, resultLen, "Synced %u, %u current, %u failed", fetched, current, failed);
    link.end(cancelled || failed ? 1 : 0, result);

    // the USB status line has room for the numbers too
    snprintf(result, resultLen, "sync %u new %u cur %u fail %uKB %ums", fetched, current, failed, bytes / 1024, dt);
    return !cancelled && !failed;
}

#endif
