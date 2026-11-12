#if defined(TARGET_TX) && defined(PLATFORM_ESP32)

#include "SDSession.h"

#include "CRSFRouter.h"
#include "OTA.h"
#include "common.h"
#include "hwTimer.h"
#include "logging.h"

#include <mbedtls/sha256.h>

namespace SDSession
{

enum State : uint8_t
{
    IDLE,
    REQUESTING,  // asking the handset for an offer
    ACCEPTING,   // ACCEPT queued, waiting for it to leave the UART
    ACTIVE,      // job task owns the port
    FINISHED,    // job done, waiting for CRSF to be restored
};

static constexpr uint32_t REQUEST_INTERVAL_MS = 100;
static constexpr uint32_t REQUEST_TIMEOUT_MS = 3000;

static volatile State state = IDLE;
static SDBulkTransport *transport = nullptr;
static uint8_t kind = KIND_IMAGE_SYNC;
static crsf_addr_e reportTo = CRSF_ADDRESS_BROADCAST;
static uint32_t requestStart = 0, requestLast = 0;
static char jobPath[65] = "LOG/bench.bin";
static char result[96];
static volatile bool resultPending = false;
static uint32_t restoredAt = 0;

static void sendSd(crsf_addr_e dest, uint8_t sub, const uint8_t *body, uint8_t len)
{
    uint8_t buf[CRSF_MAX_PACKET_LEN];
    if (len > CRSF_MAX_PACKET_LEN - 8)
        len = CRSF_MAX_PACKET_LEN - 8;
    auto h = (crsf_ext_header_t *)buf;
    h->device_addr = CRSF_SYNC_BYTE;
    h->frame_size = len + 5; // type, dest, orig, subtype, body, crc
    h->type = CRSF_FRAMETYPE_SD_SESSION;
    h->dest_addr = dest;
    h->orig_addr = CRSF_ADDRESS_CRSF_TRANSMITTER;
    h->payload[0] = sub;
    memcpy(&h->payload[1], body, len);
    buf[h->frame_size + 1] = crsfRouter.crsf_crc.calc(&buf[2], h->frame_size - 1);
    crsfRouter.deliverMessageTo(dest, (crsf_header_t *)buf);
}

static void report(const char *text)
{
    DBGLN("SD: %s", text);
    if (reportTo != CRSF_ADDRESS_BROADCAST)
        sendSd(reportTo, SUB_STATUS, (const uint8_t *)text, strlen(text));
}

void setTransport(SDBulkTransport *t)
{
    transport = t;
}

void handleCrsf(const crsf_ext_header_t *message)
{
    const uint8_t sub = message->payload[0];
    const uint8_t argLen = message->frame_size - 5;  // after the subtype

    if (message->orig_addr != CRSF_ADDRESS_RADIO_TRANSMITTER)
    {
        // USB (or anything else that isn't the handset) asking us to start one:
        // [REQUEST][kind][path...]
        if (sub == SUB_REQUEST && state == IDLE)
        {
            reportTo = message->orig_addr;
            kind = argLen ? message->payload[1] : KIND_IMAGE_SYNC;
            if (argLen > 1)
            {
                const size_t pl = std::min<size_t>(argLen - 1, sizeof(jobPath) - 1);
                memcpy(jobPath, &message->payload[2], pl);
                jobPath[pl] = '\0';
            }
            requestStart = millis();
            requestLast = 0;
            state = REQUESTING;
            report("requesting session");
        }
        return;
    }

    if (sub == SUB_OFFER && (state == IDLE || state == REQUESTING))
    {
        // [OFFER][kind][proto][max payload u16]
        if (isArmed)
        {
            const uint8_t body[] = {message->payload[1], 1, 'A', 'r', 'm', 'e', 'd'};
            sendSd(CRSF_ADDRESS_RADIO_TRANSMITTER, SUB_REJECT, body, sizeof(body));
            state = IDLE;
            report("offer rejected: armed");
            return;
        }
        kind = message->payload[1];
        const uint8_t body[] = {kind, 1, (uint8_t)SDBulk::MAX_PAYLOAD, (uint8_t)(SDBulk::MAX_PAYLOAD >> 8)};
        sendSd(CRSF_ADDRESS_RADIO_TRANSMITTER, SUB_ACCEPT, body, sizeof(body));
        state = ACCEPTING;
    }
}

void loop(uint32_t now)
{
    if (state == REQUESTING)
    {
        // signed: `now` is sampled before HandleUARTin(), so a request that just
        // arrived over USB can have requestStart a few ms *after* it
        if ((int32_t)(now - requestStart) > (int32_t)REQUEST_TIMEOUT_MS)
        {
            state = IDLE;
            report("no offer from handset");
        }
        else if ((int32_t)(now - requestLast) >= (int32_t)REQUEST_INTERVAL_MS)
        {
            // the handset only hears us in its telemetry slot, and a request can be
            // lost to a line turnaround: keep asking until it offers
            sendSd(CRSF_ADDRESS_RADIO_TRANSMITTER, SUB_REQUEST, &kind, 1);
            requestLast = now;
        }
    }

    if (resultPending && state == IDLE)
    {
        resultPending = false;
        report(result);
    }

    // a couple of seconds after a session: did the TX really go back to work?
    if (restoredAt && (int32_t)(now - restoredAt) > 2000)
    {
        restoredAt = 0;
        char text[40];
        snprintf(text, sizeof(text), "after: state %d rf %d", (int)connectionState, (int)hwTimer::running);
        report(text);
    }
}

// ---------------------------------------------------------------------------
// The job. For now: read a file off the card, hash it, and write the hash back.
// ---------------------------------------------------------------------------

static uint8_t buf[SDBulk::MAX_PAYLOAD];

static bool runJob(SDBulkLink &link)
{
    uint32_t freeKb = 0, rtc = 0;
    int st = link.hello(&freeKb, &rtc);
    if (st != SDBulk::OK)
    {
        snprintf(result, sizeof(result), "HELLO failed (%d)", st);
        return false;
    }

    uint8_t h;
    uint32_t size = 0;
    st = link.open(jobPath, SDBulk::OPEN_READ, &h, &size);
    if (st != SDBulk::OK)
    {
        snprintf(result, sizeof(result), "open %s failed (%d)", jobPath, st);
        link.end(1, "SD test: open failed");
        return false;
    }

    mbedtls_sha256_context sha;
    mbedtls_sha256_init(&sha);
    mbedtls_sha256_starts_ret(&sha, 0);

    const uint32_t t0 = millis();
    uint32_t off = 0, lastProgress = 0;
    bool cancelled = false;
    while (off < size)
    {
        if (off == 0 || off - lastProgress >= 32768)
        {
            char text[33];
            snprintf(text, sizeof(text), "Reading %u/%u KB", off / 1024, size / 1024);
            if (link.progress(off, size, text) == SDBulk::CANCELLED)
            {
                cancelled = true;
                break;
            }
            lastProgress = off;
        }
        uint16_t got = 0;
        st = link.read(h, off, buf, link.chunkSize(), &got);
        if (st == SDBulk::END_OF_FILE)
            break;
        if (st != SDBulk::OK || got == 0)
            break;
        mbedtls_sha256_update_ret(&sha, buf, got);
        off += got;
    }
    const uint32_t dt = millis() - t0;
    link.close(h);

    uint8_t digest[32];
    mbedtls_sha256_finish_ret(&sha, digest);
    mbedtls_sha256_free(&sha);
    char hex[65];
    for (int i = 0; i < 32; i++)
        sprintf(&hex[i * 2], "%02x", digest[i]);

    if (cancelled || off != size)
    {
        snprintf(result, sizeof(result), cancelled ? "cancelled at %u" : "read failed at %u (%d)", off, st);
        link.end(1, cancelled ? "Cancelled" : "SD test: read failed");
        return false;
    }

    // write the answer back next to it, the way a download will: .part, then rename
    char line[160];
    const int ll = snprintf(line, sizeof(line), "%s %u %s %u ms %u retries rtc %u\n", jobPath, size, hex, dt,
                            link.retries, rtc);
    link.mkdir("LOG");
    st = link.open("LOG/sd-test.txt.part", SDBulk::OPEN_TRUNCATE, &h, nullptr);
    if (st == SDBulk::OK)
    {
        st = link.write(h, 0, (const uint8_t *)line, ll);
        link.close(h);
        if (st == SDBulk::OK)
            st = link.rename("LOG/sd-test.txt.part", "LOG/sd-test.txt");
    }

    // a status frame carries ~54 characters
    snprintf(result, sizeof(result), "ok %uB %ums %ur sha %.8s log %d", size, dt, link.retries, hex, st);
    char msg[49];
    snprintf(msg, sizeof(msg), "Read %u KB, %.12s", size / 1024, hex);
    link.end(st == SDBulk::OK ? 0 : 1, msg);
    return st == SDBulk::OK;
}

static void jobTask(void *)
{
    static SDBulkLink *link = nullptr;
    if (!link)
        link = new SDBulkLink(transport);
    link->retries = link->calls = 0;
    link->maxPayload = SDBulk::MAX_PAYLOAD;

    runJob(*link);
    resultPending = true;
    state = FINISHED;
    vTaskDelete(nullptr);
}

bool wantsPort()
{
    return state == ACCEPTING;
}

void portAcquired()
{
    // RF stops for the length of the session; the handset's mixer is stopped anyway
    hwTimer::stop();
    setConnectionState(sdBulk);
    state = ACTIVE;
    xTaskCreatePinnedToCore(jobTask, "sdbulk", 6144, nullptr, 5, nullptr, 1);
}

bool portReleased()
{
    return state == FINISHED;
}

void portRestored()
{
    state = IDLE;
    restoredAt = millis();
}

} // namespace SDSession

#endif
