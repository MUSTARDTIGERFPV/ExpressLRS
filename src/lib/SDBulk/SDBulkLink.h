#pragma once

#include <stdint.h>
#include <stddef.h>

// Client side of the handset SD bulk protocol. The handset (EdgeTX) is a dumb file
// server for /ELRS/ on its SD card; the TX sends requests and waits for answers.
//
//   [0xEB][len u16][type][seq][payload][crc16 u16]
//   len counts type..payload, crc16 is CRC-16/CCITT-FALSE over the same bytes,
//   responses have bit 7 of type set and start the payload with a status byte.

class SDBulkTransport
{
public:
    virtual void bulkSend(const uint8_t *data, size_t len) = 0;
    virtual int bulkRead(uint8_t *data, size_t len) = 0;
};

namespace SDBulk
{
enum Op : uint8_t
{
    HELLO = 0x01,
    STAT = 0x02,
    LIST = 0x03,
    OPEN = 0x04,
    READ = 0x05,
    WRITE = 0x06,
    CLOSE = 0x07,
    RENAME = 0x08,
    REMOVE = 0x09,
    MKDIR = 0x0A,
    PROGRESS = 0x0B,
    END = 0x0C,
};

enum Status : int
{
    NO_ANSWER = -1,
    OK = 0,
    END_OF_FILE = 1,
    NOT_FOUND = 2,
    DENIED = 3,
    NO_SPACE = 4,
    BAD_HANDLE = 5,
    IO_ERROR = 6,
    BUSY = 7,
    CANCELLED = 8,
    BAD_REQUEST = 9,
};

enum OpenMode : uint8_t
{
    OPEN_READ = 0,
    OPEN_WRITE = 1,     // create if missing, keep contents
    OPEN_TRUNCATE = 2,  // create or truncate
};

constexpr uint16_t MAX_PAYLOAD = 512;
}

class SDBulkLink
{
public:
    explicit SDBulkLink(SDBulkTransport *transport) : transport(transport) {}

    // Sends a request and waits for its answer, retrying with the same seq. Returns the
    // status byte, or NO_ANSWER. The answer's payload (after the status) lands in resp.
    int call(uint8_t op, const uint8_t *body, uint16_t len, uint8_t *resp = nullptr,
             uint16_t *respLen = nullptr, uint32_t timeoutMs = 250, int tries = 8);

    int hello(uint32_t *freeKb = nullptr, uint32_t *rtc = nullptr);
    int stat(const char *path, uint32_t *size, bool *isDir);
    int mkdir(const char *path);
    int open(const char *path, uint8_t mode, uint8_t *handle, uint32_t *size);
    int read(uint8_t handle, uint32_t offset, uint8_t *data, uint16_t want, uint16_t *got);
    int write(uint8_t handle, uint32_t offset, const uint8_t *data, uint16_t len);
    int close(uint8_t handle);
    int rename(const char *from, const char *to);
    int remove(const char *path);
    int progress(uint32_t done, uint32_t total, const char *text);
    // best effort: the handset hands the port back to CRSF as soon as it answers
    void end(uint8_t result, const char *text);

    // largest READ/WRITE data that fits the agreed payload
    uint16_t chunkSize() const { return maxPayload - 8; }

    uint16_t maxPayload = SDBulk::MAX_PAYLOAD;
    uint32_t retries = 0;
    // what end() last told the handset; it's also the line to show a person
    char endText[49] = "";
    uint32_t calls = 0;

private:
    bool parse(uint8_t c);

    SDBulkTransport *transport;
    uint8_t seq = 0;
    uint8_t tx[SDBulk::MAX_PAYLOAD + 16];
    uint8_t rx[SDBulk::MAX_PAYLOAD + 16];
    uint16_t rxLen = 0;
};
