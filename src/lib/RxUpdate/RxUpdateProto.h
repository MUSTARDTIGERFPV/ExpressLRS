#pragma once

#include <stdint.h>
#include <string.h>
#if defined(PLATFORM_ESP32)
#include <esp_wifi_types.h>
#endif

// TX -> RX firmware transfer over ESP-NOW. The RX listens either after it was told
// over the RC link (MSP_ELRS_ENTER_RX_UPDATE) to stop the radio, or for a moment
// after every boot (BOOT_LISTEN_MS), which reaches an RX that can't link at all:
// one a major version ahead of or behind the TX, say.
//
// The RX takes a station MAC derived from the binding UID, so the TX knows where to
// send without any discovery, and only a TX with the same bind phrase can reach it.
// Every TX packet gets an answer; HELLO/BEGIN/FINISH are stop-and-wait, DATA is windowed.
//
//   TX  HELLO        ->  RX  HELLO_ACK   (image key, version, commit, max image size)
//   TX  BEGIN(size)  ->  RX  BEGIN_ACK
//   TX  DATA(off)    ->  RX  DATA_ACK(next expected offset)   ... up to WINDOW in flight
//   TX  FINISH(sha)  ->  RX  FINISH_ACK(status, text), then the RX reboots into it
//   TX  CANCEL_UPDATE(why)  ->  RX  CANCEL_UPDATE_ACK, then the RX reboots into what it has
//
// This protocol has to outlive the RC link protocol: an RX that can't link is
// exactly the one that needs it. So it only ever grows. Packets keep their types
// and fields, new fields go on the end, and both sides ignore what they don't know.
// PROTO_VERSION in HELLO/HELLO_ACK (0 from the first receivers, which don't send it)
// says what the other end understands.
//
// The image is the bare firmware (no options/hardware trailer); the RX appends its
// own trailer so it keeps its layout, bind phrase and settings.

namespace RxUpdate
{
constexpr uint8_t PROTO_VERSION = 1;
constexpr uint8_t ESPNOW_CHANNEL = 1;
// how long an RX listens for a HELLO after boot, alongside starting the link; the
// first valid RC packet ends it early
constexpr uint32_t BOOT_LISTEN_MS = 1500;
constexpr uint8_t MAGIC0 = 'E', MAGIC1 = 'U';
constexpr uint16_t DATA_MAX = 232;
// packets the TX keeps in flight; the RX acks cumulatively ("next offset I expect")
// and only ever writes in order, so a gap means go-back-N from the oldest unacked
constexpr uint8_t WINDOW = 8;
#if defined(PLATFORM_ESP32)
// the default 1 Mbps caps a 232 B packet stream at ~80 KB/s before acks
constexpr wifi_phy_rate_t ESPNOW_RATE = WIFI_PHY_RATE_12M;
#endif

// HelloAck.flags
// The RX wants the file's bytes as they are on the card (.bin.gz), not inflated: the
// ESP8285 has no second app slot, the raw image doesn't fit next to the running one,
// and its bootloader inflates gzip images itself.
constexpr uint8_t FLAG_WANTS_GZIP = 0x01;

enum Type : uint8_t
{
    HELLO = 1,
    HELLO_ACK,
    BEGIN,
    BEGIN_ACK,
    DATA,
    DATA_ACK,
    FINISH,
    FINISH_ACK,
    CANCEL_UPDATE,
    CANCEL_UPDATE_ACK,
};

// Cancel.reason: why the TX gave up, so the RX's log says the same as the handset
enum CancelReason : uint8_t
{
    CANCEL_USER = 0,     // the user cancelled on the handset
    CANCEL_NO_IMAGE,     // nothing on the card for this RX
    CANCEL_BAD_IMAGE,    // wrong format, or too big for this RX
    CANCEL_CARD_ERROR,   // reading the card failed
    CANCEL_TRANSFER,     // the transfer itself failed
    CANCEL_OTHER,
};

enum Status : uint8_t
{
    ST_OK = 0,
    ST_TOO_BIG,
    ST_NO_PARTITION,
    ST_WRITE_FAILED,
    ST_BAD_HASH,
    ST_NOT_STARTED,
    ST_ARMED,
};

struct __attribute__((packed)) Header
{
    uint8_t magic[2];
    uint8_t type;
    uint8_t seq;
};

struct __attribute__((packed)) Hello
{
    Header h;
    uint8_t proto;     // PROTO_VERSION; absent (0) from the first transmitters
};

struct __attribute__((packed)) HelloAck
{
    Header h;
    char key[16];      // e.g. "esp32-2400"
    char version[24];
    char commit[8];
    uint32_t maxImage;
    uint8_t flags;     // FLAG_*; absent (0) from older receivers
    uint8_t proto;     // PROTO_VERSION; absent (0) from older receivers
    uint8_t domain;    // DOMAIN_*: which regulatory build this is; absent (0) means FCC
};

// HelloAck.domain. The 2.4 GHz regulatory domain is compiled in, not set in the
// trailer, so an EU receiver must only ever get the LBT image: "<key>-lbt" on the card.
constexpr uint8_t DOMAIN_FCC = 0;
constexpr uint8_t DOMAIN_LBT = 1;
inline uint8_t buildDomain()
{
#if defined(Regulatory_Domain_EU_CE_2400)
    return DOMAIN_LBT;
#else
    return DOMAIN_FCC;
#endif
}

struct __attribute__((packed)) Begin
{
    Header h;
    uint32_t size;
};

struct __attribute__((packed)) StatusOnly
{
    Header h;
    uint8_t status;
};

struct __attribute__((packed)) Data
{
    Header h;
    uint32_t offset;
    uint8_t data[DATA_MAX];
};

struct __attribute__((packed)) DataAck
{
    Header h;
    uint32_t next;
    uint8_t status;
};

struct __attribute__((packed)) Finish
{
    Header h;
    uint8_t sha256[32];
};

struct __attribute__((packed)) FinishAck
{
    Header h;
    uint8_t status;
    char text[40];
};

struct __attribute__((packed)) Cancel
{
    Header h;
    uint8_t reason;    // CancelReason
    char text[40];     // what the handset shows, NUL-terminated
};

inline void macFromUid(const uint8_t *uid, uint8_t *mac)
{
    memcpy(mac, uid, 6);
    mac[0] = (mac[0] & 0xFE) | 0x02; // unicast, locally administered
}

inline bool valid(const uint8_t *p, int len)
{
    return len >= (int)sizeof(Header) && p[0] == MAGIC0 && p[1] == MAGIC1;
}
}
