#if defined(TARGET_TX) && defined(PLATFORM_ESP32)

#include "SDRxUpdate.h"

#include <Arduino.h>
#include "OTA.h"
#include "RxUpdateProto.h"
#include "logging.h"

#include <ArduinoJson.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <mbedtls/sha256.h>
#if defined(PLATFORM_ESP32_S3)
#include <esp32s3/rom/miniz.h>
#elif defined(PLATFORM_ESP32_C3)
#include <esp32c3/rom/miniz.h>
#else
#include <esp32/rom/miniz.h>
#endif

using namespace RxUpdate;

static constexpr uint32_t KEEPALIVE_MS = 800;
static constexpr uint32_t FIND_RX_TIMEOUT_MS = 10000;
// how long to wait for someone to power the receiver on
static constexpr uint32_t FIND_BOOTING_RX_TIMEOUT_MS = 30000;

static QueueHandle_t ackQueue;
static volatile uint32_t sendOk = 0, sendFail = 0;

static void onSent(const uint8_t *, esp_now_send_status_t status)
{
    if (status == ESP_NOW_SEND_SUCCESS)
        sendOk++;
    else
        sendFail++;
}
static uint8_t rxMac[6];
static uint8_t seq;
static uint32_t lastCall;

struct Reply
{
    uint8_t len;
    uint8_t data[250];
};

static void onRecv(const uint8_t *mac, const uint8_t *data, int len)
{
    if (!valid(data, len) || len > 250 || memcmp(mac, rxMac, 6) != 0)
        return;
    Reply r;
    r.len = len;
    memcpy(r.data, data, len);
    xQueueSend(ackQueue, &r, 0);
}

// esp_now_send() only queues; with a window in flight its buffer can be full for a moment
static bool sendRaw(const void *pkt, size_t len)
{
    for (int t = 0; t < 50; t++)
    {
        const esp_err_t err = esp_now_send(rxMac, (const uint8_t *)pkt, len);
        if (err == ESP_OK)
            return true;
        if (err != ESP_ERR_ESPNOW_NO_MEM)
            return false;
        vTaskDelay(1);
    }
    return false;
}

static int keepalive(SDBulkLink &link, const char *text, uint32_t done = 0, uint32_t total = 0, bool force = false)
{
    if (!force && millis() - lastCall < KEEPALIVE_MS)
        return SDBulk::OK;
    lastCall = millis();
    return link.progress(done, total, text);
}

// One stop-and-wait exchange: send, wait for the matching answer type and seq
static bool exchange(void *pkt, size_t len, Type want, Reply &out, uint32_t timeoutMs, int tries)
{
    auto h = (Header *)pkt;
    h->magic[0] = MAGIC0;
    h->magic[1] = MAGIC1;
    h->seq = ++seq;
    for (int t = 0; t < tries; t++)
    {
        esp_now_send(rxMac, (const uint8_t *)pkt, len);
        const uint32_t start = millis();
        while (millis() - start < timeoutMs)
        {
            if (xQueueReceive(ackQueue, &out, pdMS_TO_TICKS(5)) == pdTRUE)
            {
                const auto r = (const Header *)out.data;
                if (r->type == want && r->seq == seq)
                    return true;
            }
        }
    }
    return false;
}

static bool startEspNow()
{
    ackQueue = xQueueCreate(32, sizeof(Reply));
    macFromUid(UID, rxMac);
    WiFi.persistent(false);
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    esp_wifi_set_promiscuous(true);
    esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);
    esp_wifi_set_promiscuous(false);
    if (esp_now_init() != ESP_OK)
        return false;
    esp_wifi_config_espnow_rate(WIFI_IF_STA, ESPNOW_RATE);
    esp_now_register_recv_cb(onRecv);
    esp_now_register_send_cb(onSent);
    sendOk = sendFail = 0;
    esp_now_peer_info_t info = {};
    memcpy(info.peer_addr, rxMac, 6);
    info.channel = ESPNOW_CHANNEL;
    info.ifidx = WIFI_IF_STA;
    return esp_now_add_peer(&info) == ESP_OK;
}

static void stopEspNow()
{
    esp_now_deinit();
    WiFi.mode(WIFI_OFF);
    if (ackQueue)
    {
        vQueueDelete(ackQueue);
        ackQueue = nullptr;
    }
}

// Tell the RX we're giving up, so it reboots into what it has now rather than sitting
// in update mode until its idle timeout. Best effort: an RX that's gone is fine too.
static void cancelRx(CancelReason reason, const char *text)
{
    Cancel c = {};
    c.h.type = CANCEL_UPDATE;
    c.reason = reason;
    strncpy(c.text, text, sizeof(c.text) - 1);
    Reply r;
    xQueueReset(ackQueue);
    exchange(&c, sizeof(c), CANCEL_UPDATE_ACK, r, 200, 3);
}

// The image the last sync left for this key, as FIRMWARE/manifest.json records it
// (version names from CI don't sort by age); false if there's no such record
static bool syncedImage(SDBulkLink &link, const char *key, bool gzOnly, char *path, size_t pathLen)
{
    static char manifest[4096];
    uint8_t h;
    uint32_t size;
    if (link.open("FIRMWARE/manifest.json", SDBulk::OPEN_READ, &h, &size) != SDBulk::OK)
        return false;
    uint32_t off = 0;
    while (off < size && off < sizeof(manifest) - 1)
    {
        uint16_t got = 0;
        const uint16_t want = std::min<uint32_t>(link.chunkSize(), sizeof(manifest) - 1 - off);
        if (link.read(h, off, (uint8_t *)manifest + off, want, &got) != SDBulk::OK || !got)
            break;
        off += got;
    }
    link.close(h);
    manifest[off] = '\0';

    JsonDocument doc;
    if (deserializeJson(doc, manifest) != DeserializationError::Ok)
        return false;
    const char *version = doc["images"][key]["version"] | "";
    if (!*version)
        return false;
    // the sync stores every image as .bin.gz
    snprintf(path, pathLen, "FIRMWARE/%s/%s.bin.gz", key, version);
    uint32_t have = 0;
    bool isDir = false;
    return link.stat(path, &have, &isDir) == SDBulk::OK && !isDir && have > 0;
}

// Images copied by hand, with no sync record: the lexically greatest finished file
static bool pickImage(SDBulkLink &link, const char *key, bool gzOnly, char *path, size_t pathLen)
{
    if (syncedImage(link, key, gzOnly, path, pathLen))
        return true;
    char dir[40];
    snprintf(dir, sizeof(dir), "FIRMWARE/%s", key);
    static uint8_t resp[SDBulk::MAX_PAYLOAD];
    char best[56] = "";
    uint16_t cookie = 0;
    while (cookie != 0xFFFF)
    {
        uint8_t req[48];
        memcpy(req, &cookie, 2);
        const size_t dl = strlen(dir);
        memcpy(&req[2], dir, dl);
        uint16_t n = 0;
        if (link.call(SDBulk::LIST, req, 2 + dl, resp, &n) != SDBulk::OK || n < 2)
            return false;
        memcpy(&cookie, resp, 2);
        for (uint16_t i = 2; i + 6 <= n;)
        {
            const bool isDir = resp[i];
            const uint8_t nl = resp[i + 5];
            char name[65];
            memcpy(name, &resp[i + 6], std::min<uint8_t>(nl, 64));
            name[std::min<uint8_t>(nl, 64)] = '\0';
            i += 6 + nl;
            const size_t len = strlen(name);
            const bool image = (!gzOnly && len > 4 && strcmp(name + len - 4, ".bin") == 0) ||
                               (len > 7 && strcmp(name + len - 7, ".bin.gz") == 0);
            if (!isDir && image && strlen(name) < sizeof(best) && strcmp(name, best) > 0)
                strcpy(best, name);
        }
    }
    if (!best[0])
        return false;
    snprintf(path, pathLen, "%s/%s", dir, best);
    return true;
}

// Streams file bytes in, raw firmware bytes out. Handles .bin.gz with the ROM inflater.
class ImageSource
{
public:
    // passthrough: hand out the file's bytes as they are, even if it's gzip
    bool open(SDBulkLink &l, const char *path, uint32_t &rawSize, bool passthrough)
    {
        link = &l;
        const size_t pl = strlen(path);
        gz = !passthrough && pl > 3 && strcmp(path + pl - 3, ".gz") == 0;
        if (link->open(path, SDBulk::OPEN_READ, &h, &fileSize) != SDBulk::OK)
            return false;
        fileOff = inPos = inLen = 0;
        if (!gz)
        {
            rawSize = fileSize;
            return true;
        }
        // gzip: the raw size is the last 4 bytes; skip the header
        uint16_t got;
        uint8_t tail[4];
        if (link->read(h, fileSize - 4, tail, 4, &got) != SDBulk::OK || got != 4)
            return false;
        memcpy(&rawSize, tail, 4);
        if (!fill() || inLen < 10 || in[0] != 0x1F || in[1] != 0x8B || in[2] != 8)
            return false;
        const uint8_t flags = in[3];
        inPos = 10;
        if (flags & 0x04) // FEXTRA
            inPos += 2 + (in[inPos] | (in[inPos + 1] << 8));
        if (flags & 0x08) // FNAME
            while (inPos < inLen && in[inPos++]) {}
        if (flags & 0x10) // FCOMMENT
            while (inPos < inLen && in[inPos++]) {}
        if (flags & 0x02) // FHCRC
            inPos += 2;
        tinfl_init(&inflator);
        dictOfs = outPos = outLen = 0;
        done = false;
        return true;
    }

    // up to `max` raw bytes; 0 at the end, -1 on error
    int read(uint8_t *out, size_t max)
    {
        if (!gz)
        {
            if (fileOff >= fileSize)
                return 0;
            uint16_t got = 0;
            const uint16_t want = std::min<uint32_t>(std::min<size_t>(max, link->chunkSize()), fileSize - fileOff);
            if (link->read(h, fileOff, out, want, &got) != SDBulk::OK)
                return -1;
            fileOff += got;
            return got;
        }
        while (outPos == outLen)
        {
            if (done)
                return 0;
            if (inPos == inLen && fileOff < fileSize && !fill())
                return -1;
            size_t inBytes = inLen - inPos;
            size_t outBytes = TINFL_LZ_DICT_SIZE - dictOfs;
            const tinfl_status st = tinfl_decompress(&inflator, &in[inPos], &inBytes, dict, &dict[dictOfs], &outBytes,
                                                     fileOff < fileSize ? TINFL_FLAG_HAS_MORE_INPUT : 0);
            inPos += inBytes;
            outPos = dictOfs;
            outLen = dictOfs + outBytes;
            dictOfs = (dictOfs + outBytes) & (TINFL_LZ_DICT_SIZE - 1);
            if (st == TINFL_STATUS_DONE)
                done = true;
            else if (st < 0)
                return -1;
        }
        const size_t n = std::min(max, outLen - outPos);
        memcpy(out, &dict[outPos], n);
        outPos += n;
        return n;
    }

    void close() { link->close(h); }

private:
    bool fill()
    {
        uint16_t got = 0;
        if (link->read(h, fileOff, in, link->chunkSize(), &got) != SDBulk::OK || !got)
            return false;
        fileOff += got;
        inPos = 0;
        inLen = got;
        return true;
    }

    SDBulkLink *link = nullptr;
    uint8_t h = 0;
    bool gz = false, done = false;
    uint32_t fileSize = 0, fileOff = 0;
    uint8_t in[SDBulk::MAX_PAYLOAD];
    size_t inPos = 0, inLen = 0;
    tinfl_decompressor inflator;
    uint8_t dict[TINFL_LZ_DICT_SIZE];
    size_t dictOfs = 0, outPos = 0, outLen = 0;
};

static ImageSource *src;

bool runRxUpdate(SDBulkLink &link, const char *path, bool linked, char *result, size_t resultLen)
{
    lastCall = millis();
    if (link.hello() != SDBulk::OK)
    {
        snprintf(result, resultLen, "Radio didn't answer");
        return false;
    }

    auto fail = [&](const char *msg) {
        snprintf(result, resultLen, "%s", msg);
        stopEspNow();
        link.end(1, msg);
        return false;
    };
    // once the RX may be in update mode: let it go before we do
    auto failRx = [&](CancelReason reason, const char *msg) {
        cancelRx(reason, msg);
        return fail(msg);
    };

    if (!startEspNow())
        return fail("ESP-NOW init failed");

    // Find the receiver. A linked one switches over ~0.5 s after acking the link
    // command; any other listens for BOOT_LISTEN_MS after power-on, so ask often.
    Reply r;
    HelloAck info = {};
    const char *finding = linked ? "Finding receiver" : "Power on the receiver now";
    keepalive(link, finding, 0, 0, true);
    const uint32_t t0 = millis();
    bool found = false;
    while (!found && millis() - t0 < (linked ? FIND_RX_TIMEOUT_MS : FIND_BOOTING_RX_TIMEOUT_MS))
    {
        Hello hello = {};
        hello.h.type = HELLO;
        hello.proto = PROTO_VERSION;
        if (exchange(&hello, sizeof(hello), HELLO_ACK, r, linked ? 150 : 50, 1))
        {
            memcpy(&info, r.data, std::min<size_t>(r.len, sizeof(info)));
            found = true;
        }
        if (keepalive(link, finding) == SDBulk::CANCELLED)
            return failRx(CANCEL_USER, "Cancelled");
    }
    if (!found)
    {
        DBGLN("RX update: no answer from %02x:%02x (tx ok %u fail %u)", rxMac[4], rxMac[5], (unsigned)sendOk,
              (unsigned)sendFail);
        // a linked RX that did switch over but can't hear us still gets told
        return linked ? failRx(CANCEL_OTHER, "Receiver not found") : fail("Receiver not found at power-on");
    }
    info.key[sizeof(info.key) - 1] = info.version[sizeof(info.version) - 1] = '\0';
    // where its images live on the card: EU (LBT) builds have their own
    char key[24];
    snprintf(key, sizeof(key), "%s%s", info.key, info.domain == DOMAIN_LBT ? "-lbt" : "");
    char rxCommit[9] = {};
    memcpy(rxCommit, info.commit, 8);
    DBGLN("RX update: found %s %s (%s), max %u", info.key, info.version, rxCommit, info.maxImage);

    const bool wantsGzip = info.flags & FLAG_WANTS_GZIP;
    char imagePath[96];
    if (path && *path)
        snprintf(imagePath, sizeof(imagePath), "%s", path);
    else if (!pickImage(link, key, wantsGzip, imagePath, sizeof(imagePath)))
    {
        char msg[48];
        // A sync only brings this TX's own regulatory build, so syncing won't help
        // a receiver of the other one. Never fall back to it: that would quietly
        // change the receiver's domain.
        if (info.domain != buildDomain())
            snprintf(msg, sizeof(msg), "%s receiver - this TX only syncs %s",
                     info.domain == DOMAIN_LBT ? "LBT" : "FCC", buildDomain() == DOMAIN_LBT ? "LBT" : "FCC");
        else
            snprintf(msg, sizeof(msg), "No %.18s image on SD - sync first", key);
        return failRx(CANCEL_NO_IMAGE, msg);
    }

    if (!src)
        src = new ImageSource();
    const size_t ipl = strlen(imagePath);
    if (wantsGzip && !(ipl > 3 && strcmp(imagePath + ipl - 3, ".gz") == 0))
        return failRx(CANCEL_BAD_IMAGE, "RX needs a .bin.gz image");
    // "rawSize" is what goes over the air: the inflated image, or the .gz itself
    uint32_t rawSize = 0;
    if (!src->open(link, imagePath, rawSize, wantsGzip))
        return failRx(CANCEL_CARD_ERROR, "Can't read image from SD");
    if (rawSize > info.maxImage)
    {
        src->close();
        return failRx(CANCEL_BAD_IMAGE, "Image too big for RX");
    }

    // The RX erases the whole target range before it answers (~1-2 s per MB); the
    // retried BEGINs queue up behind it and get the same answer
    Begin b = {};
    b.h.type = BEGIN;
    b.size = rawSize;
    bool begun = false;
    xQueueReset(ackQueue);
    keepalive(link, "Erasing on RX", 0, rawSize, true);
    for (int t = 0; t < 40 && !begun; t++)
    {
        if (exchange(&b, sizeof(b), BEGIN_ACK, r, 500, 1))
        {
            if (((StatusOnly *)r.data)->status != ST_OK)
                break;
            begun = true;
        }
        keepalive(link, "Erasing on RX", 0, rawSize);
    }
    if (!begun)
    {
        src->close();
        return failRx(CANCEL_BAD_IMAGE, "RX refused the image");
    }

    mbedtls_sha256_context sha;
    mbedtls_sha256_init(&sha);
    mbedtls_sha256_starts_ret(&sha, 0);

    // Sliding window, go-back-N. Packets stay in `win` until the RX's cumulative ack
    // passes them, since the inflated stream can't be rewound.
    struct Slot
    {
        uint16_t len;
        Data d;
    };
    static Slot win[WINDOW];
    uint8_t tail = 0, count = 0;
    uint32_t sent = 0, acked = 0, lastShown = 0, retries = 0, stalls = 0;
    bool eof = false;
    const uint32_t ts = millis();
    bool ok = true;
    bool cancelled = false;
    char text[33] = "Sending";

    auto transmit = [](Slot &slot) {
        slot.d.h.magic[0] = MAGIC0;
        slot.d.h.magic[1] = MAGIC1;
        slot.d.h.type = DATA;
        slot.d.h.seq = ++seq;
        return sendRaw(&slot.d, offsetof(Data, data) + slot.len);
    };

    while (ok && (acked < rawSize))
    {
        // top the window up from the (possibly inflated) stream
        while (!eof && count < WINDOW && sent < rawSize)
        {
            Slot &slot = win[(tail + count) % WINDOW];
            int n = 0;
            while (n < (int)DATA_MAX)
            {
                const int got = src->read(slot.d.data + n, DATA_MAX - n);
                if (got < 0)
                {
                    ok = false;
                    break;
                }
                if (got == 0)
                {
                    eof = true;
                    break;
                }
                n += got;
            }
            if (!ok || n == 0)
                break;
            mbedtls_sha256_update_ret(&sha, slot.d.data, n);
            slot.len = n;
            slot.d.offset = sent;
            sent += n;
            count++;
            if (!transmit(slot))
                ok = false;
        }
        if (!ok || count == 0)
            break;

        // collect acks; any progress slides the window
        const uint32_t before = acked;
        const uint32_t waitStart = millis();
        while (acked == before && millis() - waitStart < 60)
        {
            if (xQueueReceive(ackQueue, &r, pdMS_TO_TICKS(2)) != pdTRUE)
                continue;
            const auto h = (const Header *)r.data;
            if (h->type != DATA_ACK)
                continue;
            const auto a = (const DataAck *)r.data;
            if (a->status != ST_OK)
            {
                ok = false;
                break;
            }
            if (a->next > acked)
                acked = a->next;
        }
        while (count && win[tail].d.offset + win[tail].len <= acked)
        {
            tail = (tail + 1) % WINDOW;
            count--;
        }

        if (acked == before && ok)
        {
            // nothing moved: resend everything still in flight, oldest first
            if (++stalls > 100)
            {
                ok = false;
                break;
            }
            for (uint8_t i = 0; i < count; i++)
            {
                retries++;
                transmit(win[(tail + i) % WINDOW]);
            }
        }
        else
        {
            stalls = 0;
        }

        lastCall = millis(); // the SD reads above are the handset's keepalive
        if (acked - lastShown >= 16384 || acked == rawSize || millis() - lastCall > KEEPALIVE_MS)
        {
            snprintf(text, sizeof(text), "Sending %u/%u KB", acked / 1024, rawSize / 1024);
            if (keepalive(link, text, acked, rawSize, true) == SDBulk::CANCELLED)
            {
                ok = false;
                cancelled = true;
                break;
            }
            lastShown = acked;
        }
    }
    src->close();
    const uint32_t dt = millis() - ts;

    Finish f = {};
    f.h.type = FINISH;
    mbedtls_sha256_finish_ret(&sha, f.sha256);
    mbedtls_sha256_free(&sha);
    if (cancelled)
        return failRx(CANCEL_USER, "Cancelled");
    if (!ok)
        return failRx(CANCEL_TRANSFER, "Transfer failed");

    // By now the queue is full of stale DATA_ACKs (the RX acks every duplicate too); a
    // full queue silently drops the FINISH_ACK, so start clean. The RX verifies the
    // whole image (~1-2 s) before it answers, then keeps answering repeats for a few
    // seconds before it reboots: ask often, and keep the handset's session alive.
    xQueueReset(ackQueue);
    keepalive(link, "Verifying on RX", rawSize, rawSize, true);
    bool confirmed = false;
    for (int t = 0; t < 12 && !confirmed; t++)
    {
        confirmed = exchange(&f, sizeof(f), FINISH_ACK, r, 1000, 1);
        keepalive(link, "Verifying on RX", rawSize, rawSize);
    }
    if (!confirmed)
        return failRx(CANCEL_TRANSFER, "RX did not confirm");
    const auto fa = (const FinishAck *)r.data;
    char rxText[41] = {};
    memcpy(rxText, fa->text, 40);
    // a failed FINISH leaves the RX in update mode: send it back to its old image
    if (fa->status != ST_OK)
        cancelRx(CANCEL_TRANSFER, rxText);
    stopEspNow();

    const char *file = strrchr(imagePath, '/');
    file = file ? file + 1 : imagePath;
    if (fa->status != ST_OK)
    {
        snprintf(result, resultLen, "RX: %.40s", rxText);
        link.end(1, result);
        return false;
    }
    char msg[49];
    snprintf(msg, sizeof(msg), "RX updated to %.30s", file);
    link.end(0, msg);
    snprintf(result, resultLen, "rxupd %.8s->%.16s %uKB %ums %ur", rxCommit, file, rawSize / 1024, dt, retries);
    return true;
}

#endif
