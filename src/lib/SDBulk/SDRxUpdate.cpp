#if defined(TARGET_TX) && defined(PLATFORM_ESP32)

#include "SDRxUpdate.h"

#include <Arduino.h>
#include "OTA.h"
#include "RxUpdateProto.h"
#include "logging.h"

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

// newest image for this key: the lexically greatest finished file in the directory
static bool pickImage(SDBulkLink &link, const char *key, char *path, size_t pathLen)
{
    char dir[40];
    snprintf(dir, sizeof(dir), "FIRMWARE/%s", key);
    static uint8_t resp[SDBulk::MAX_PAYLOAD];
    char best[48] = "";
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
            const bool image = (len > 4 && strcmp(name + len - 4, ".bin") == 0) ||
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
    bool open(SDBulkLink &l, const char *path, uint32_t &rawSize)
    {
        link = &l;
        const size_t pl = strlen(path);
        gz = pl > 3 && strcmp(path + pl - 3, ".gz") == 0;
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

bool runRxUpdate(SDBulkLink &link, const char *path, char *result, size_t resultLen)
{
    lastCall = millis();
    if (link.hello() != SDBulk::OK)
    {
        snprintf(result, resultLen, "HELLO failed");
        return false;
    }

    auto fail = [&](const char *msg) {
        snprintf(result, resultLen, "%s", msg);
        stopEspNow();
        link.end(1, msg);
        return false;
    };

    if (!startEspNow())
        return fail("ESP-NOW init failed");

    // find the receiver: it switches over ~0.5 s after acking the link command
    Reply r;
    HelloAck info = {};
    keepalive(link, "Finding receiver", 0, 0, true);
    const uint32_t t0 = millis();
    bool found = false;
    while (!found && millis() - t0 < FIND_RX_TIMEOUT_MS)
    {
        Header hello = {};
        hello.type = HELLO;
        if (exchange(&hello, sizeof(hello), HELLO_ACK, r, 150, 1))
        {
            memcpy(&info, r.data, std::min<size_t>(r.len, sizeof(info)));
            found = true;
        }
        if (keepalive(link, "Finding receiver") == SDBulk::CANCELLED)
            return fail("Cancelled");
    }
    if (!found)
    {
        char msg[64];
        snprintf(msg, sizeof(msg), "Receiver did not answer (%02x:%02x tx ok %u fail %u)", rxMac[4], rxMac[5],
                 (unsigned)sendOk, (unsigned)sendFail);
        return fail(msg);
    }
    info.key[sizeof(info.key) - 1] = info.version[sizeof(info.version) - 1] = '\0';
    char rxCommit[9] = {};
    memcpy(rxCommit, info.commit, 8);
    DBGLN("RX update: found %s %s (%s), max %u", info.key, info.version, rxCommit, info.maxImage);

    char imagePath[80];
    if (path && *path)
        snprintf(imagePath, sizeof(imagePath), "%s", path);
    else if (!pickImage(link, info.key, imagePath, sizeof(imagePath)))
    {
        snprintf(result, resultLen, "no image for %.16s", info.key);
        stopEspNow();
        link.end(1, result);
        return false;
    }

    if (!src)
        src = new ImageSource();
    uint32_t rawSize = 0;
    if (!src->open(link, imagePath, rawSize))
        return fail("Can't open image");
    if (rawSize > info.maxImage)
    {
        src->close();
        return fail("Image too big for RX");
    }

    // The RX erases the whole target range before it answers (~1-2 s per MB); the
    // retried BEGINs queue up behind it and get the same answer
    Begin b = {};
    b.h.type = BEGIN;
    b.size = rawSize;
    bool begun = false;
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
        return fail("RX refused the image");
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
    if (!ok)
        return fail("Transfer failed");

    keepalive(link, "Verifying on RX", rawSize, rawSize, true);
    // the RX verifies the whole image before it answers
    if (!exchange(&f, sizeof(f), FINISH_ACK, r, 4000, 3))
        return fail("RX did not confirm");
    const auto fa = (const FinishAck *)r.data;
    char rxText[41] = {};
    memcpy(rxText, fa->text, 40);
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
