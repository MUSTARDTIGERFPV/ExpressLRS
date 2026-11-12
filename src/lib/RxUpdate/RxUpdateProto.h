#pragma once

#include <stdint.h>
#include <string.h>

// TX -> RX firmware transfer over ESP-NOW, after the RX was told over the RC link
// (MSP_ELRS_ENTER_RX_UPDATE) to stop the radio and listen.
//
// The RX takes a station MAC derived from the binding UID, so the TX knows where to
// send without any discovery, and only a TX with the same bind phrase can reach it.
// Strict stop-and-wait: every TX packet gets an answer, the TX retries on silence.
//
//   TX  HELLO        ->  RX  HELLO_ACK   (image key, version, commit, max image size)
//   TX  BEGIN(size)  ->  RX  BEGIN_ACK
//   TX  DATA(off)    ->  RX  DATA_ACK(next expected offset)   ... repeated
//   TX  FINISH(sha)  ->  RX  FINISH_ACK(status, text), then the RX reboots into it
//
// The image is the bare firmware (no options/hardware trailer); the RX appends its
// own trailer so it keeps its layout, bind phrase and settings.

namespace RxUpdate
{
constexpr uint8_t ESPNOW_CHANNEL = 1;
constexpr uint8_t MAGIC0 = 'E', MAGIC1 = 'U';
constexpr uint16_t DATA_MAX = 232;

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

struct __attribute__((packed)) HelloAck
{
    Header h;
    char key[16];      // e.g. "esp32-2400"
    char version[24];
    char commit[8];
    uint32_t maxImage;
};

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
