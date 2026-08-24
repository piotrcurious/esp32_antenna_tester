#ifndef RF_TEST_PROTOCOL_H
#define RF_TEST_PROTOCOL_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include <string.h>

#if defined(ARDUINO)
#include <Arduino.h>
#endif

#ifndef min
#define min(a,b) ((a)<(b)?(a):(b))
#endif
#ifndef max
#define max(a,b) ((a)>(b)?(a):(b))
#endif

// ============================================================
// CONSTANTS & MAGICS
// ============================================================

static const uint16_t RF_MAGIC           = 0x5246; // "RF"
static const uint8_t  PROTOCOL_VERSION     = 2;

static const uint8_t  SERIAL_MAGIC1        = 0xA5;
static const uint8_t  SERIAL_MAGIC2        = 0x5A;

static const uint8_t  CONTROL_CHANNEL_DEFAULT = 1;

static const uint8_t  EXPORT_CSI_BINS      = 32;
static const uint8_t  MAX_CSI_PAIRS        = 64;

static const uint16_t SERIAL_MAX_PAYLOAD   = 220;

// OUI for custom 802.11 action frames
static const uint8_t  TEST_OUI[3]          = { 0xDE, 0xAD, 0xBE };
static const uint8_t  TEST_FRAME_MAGIC[4]   = { 'R', 'F', 'T', 'S' };

// ============================================================
// ENUMERATIONS
// ============================================================

enum Phase : uint8_t {
    PHASE_REF_V_TX  = 0, // Reference V-pol antenna transmits
    PHASE_REF_H_TX  = 1, // Reference H-pol antenna transmits
    PHASE_TEST_A_TX = 2, // Test antenna transmits
    PHASE_TEST_R_TX = 3  // Test-node reference antenna transmits
};

enum ControlType : uint8_t {
    CMD_SYNC_REQ   = 1,
    CMD_SYNC_RESP  = 2,
    CMD_PREPARE    = 3,
    CMD_READY      = 4,
    CMD_START_AT   = 5,
    CMD_START_ACK  = 6,
    CMD_RESULT     = 7,
    CMD_ABORT      = 8,
    CMD_ERROR_MSG  = 9
};

enum LocalType : uint8_t {
    LOCAL_PREPARE  = 1,
    LOCAL_READY    = 2,
    LOCAL_START_AT = 3,
    LOCAL_RESULT   = 4,
    LOCAL_ABORT    = 5,
    LOCAL_SYNC_REQ = 6,
    LOCAL_SYNC_RESP= 7
};

enum RadioMode : uint8_t {
    RADIO_MODE_NONE = 0,
    RADIO_MODE_RX   = 1,
    RADIO_MODE_TX   = 2
};

// ============================================================
// COMMON MEASUREMENT ACCUMULATORS
// ============================================================

struct CsiAccumulator {
    volatile uint32_t frames;
    volatile int32_t  rssiSum;
    volatile uint32_t rssiSumSq;
    volatile int8_t   rssiMin;
    volatile int8_t   rssiMax;
    volatile uint32_t pairCountTotal;
    int32_t sumI[64];
    int32_t sumQ[64];
    uint16_t count[64];
};

struct TestPacketStats {
    volatile uint32_t packets;
    volatile uint32_t matchingPackets;
    volatile uint32_t crcPackets;
    volatile uint32_t duplicates;
    volatile uint32_t lastSequence;
    volatile bool     haveSequence;
};

// ============================================================
// PACKED PROTOCOL STRUCTURES
// ============================================================

#pragma pack(push, 1)

struct SyncRequest {
    uint16_t magic;
    uint8_t  version;
    uint8_t  type;
    uint32_t sequence;
    uint64_t t1;
};

struct SyncResponse {
    uint16_t magic;
    uint8_t  version;
    uint8_t  type;
    uint32_t sequence;
    uint64_t t1;
    uint64_t t2;
    uint64_t t3;
};

struct PrepareCommand {
    uint16_t magic;
    uint8_t  version;
    uint8_t  type;
    uint32_t runId;
    uint16_t testId;
    uint8_t  phase;
    uint8_t  channel;
    uint16_t windowMs;
    uint32_t guardUs;
    uint32_t txIntervalUs;
    uint8_t  bandwidth;
    uint8_t  txRate;
    uint8_t  targetMac[6];
    uint8_t  reserved[4];
};

struct ReadyMessage {
    uint16_t magic;
    uint8_t  version;
    uint8_t  type;
    uint32_t runId;
    uint16_t testId;
    uint8_t  status;
    uint8_t  reserved[12];
};

struct StartAtCommand {
    uint16_t magic;
    uint8_t  version;
    uint8_t  type;
    uint32_t runId;
    uint16_t testId;
    uint64_t startAtUs;
    uint32_t windowUs;
    uint32_t guardAfterUs;
};

struct StartAck {
    uint16_t magic;
    uint8_t  version;
    uint8_t  type;
    uint32_t runId;
    uint16_t testId;
    uint64_t startAtUs;
    uint8_t  status;
    uint8_t  reserved[7];
};

struct RFResultSummary {
    uint8_t  mode;              // RadioMode
    uint8_t  csiValid;          // 1 if CSI data present
    uint16_t csiFrames;        // Total CSI frames captured
    uint32_t testPackets;      // Total packets sent or received
    uint32_t matchingPackets;  // Matching RFTS test packets
    uint32_t crcPackets;       // Packets with CRC errors
    uint32_t duplicatePackets; // Duplicate packets
    int32_t  rssiSum;           // Sum of RSSI
    uint32_t rssiSumSq;        // Sum of RSSI^2
    int8_t   rssiMin;          // Min RSSI
    int8_t   rssiMax;          // Max RSSI
    uint16_t csiPairs;         // Average CSI pair count per frame
    int8_t   avgI[32];         // Exported subcarrier I (real) averages
    int8_t   avgQ[32];         // Exported subcarrier Q (imag) averages
};

struct ResultPacket {
    uint16_t magic;
    uint8_t  version;
    uint8_t  type;
    uint32_t runId;
    uint16_t testId;
    uint8_t  phase;
    uint8_t  status;
    RFResultSummary radio0;     // Master radio summary
    RFResultSummary radio1;     // Slave radio summary
    uint8_t  reserved[8];
};

struct SerialHeader {
    uint8_t magic1; // SERIAL_MAGIC1 (0xA5)
    uint8_t magic2; // SERIAL_MAGIC2 (0x5A)
    uint8_t type;   // LocalType
    uint8_t length; // Payload length
};

#pragma pack(pop)

// ============================================================
// UTILITY FUNCTIONS
// ============================================================

inline uint16_t calculate_crc16(const uint8_t *data, size_t len) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; ++i) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; ++j) {
            if (crc & 1) {
                crc = (crc >> 1) ^ 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}

inline uint16_t build_raw_80211_action_frame(
    uint8_t *buffer,
    const uint8_t *srcMac,
    const uint8_t *dstMac,
    uint32_t runId,
    uint16_t testId,
    uint8_t phase,
    uint32_t sequence,
    uint8_t channel,
    uint8_t senderRole
) {
    uint16_t p = 0;

    // Frame Control: Management Action (0x00D0)
    uint16_t fc = 0x00D0;
    memcpy(buffer + p, &fc, 2); p += 2;

    // Duration
    uint16_t duration = 0;
    memcpy(buffer + p, &duration, 2); p += 2;

    // Destination MAC
    if (dstMac) {
        memcpy(buffer + p, dstMac, 6);
    } else {
        memset(buffer + p, 0xFF, 6); // Broadcast fallback
    }
    p += 6;

    // Source MAC
    if (srcMac) {
        memcpy(buffer + p, srcMac, 6);
    } else {
        memset(buffer + p, 0x00, 6);
    }
    p += 6;

    // BSSID MAC
    if (dstMac) {
        memcpy(buffer + p, dstMac, 6);
    } else {
        memset(buffer + p, 0xFF, 6);
    }
    p += 6;

    // Sequence Control (0)
    uint16_t seqCtrl = 0;
    memcpy(buffer + p, &seqCtrl, 2); p += 2;

    // Action Category: Vendor Specific (127)
    buffer[p++] = 127;

    // Test OUI
    buffer[p++] = TEST_OUI[0];
    buffer[p++] = TEST_OUI[1];
    buffer[p++] = TEST_OUI[2];

    // Protocol Revision
    buffer[p++] = PROTOCOL_VERSION;

    // Magic "RFTS"
    buffer[p++] = TEST_FRAME_MAGIC[0];
    buffer[p++] = TEST_FRAME_MAGIC[1];
    buffer[p++] = TEST_FRAME_MAGIC[2];
    buffer[p++] = TEST_FRAME_MAGIC[3];

    // Run ID
    memcpy(buffer + p, &runId, 4); p += 4;

    // Test ID
    memcpy(buffer + p, &testId, 2); p += 2;

    // Phase
    buffer[p++] = phase;

    // Sequence Number
    memcpy(buffer + p, &sequence, 4); p += 4;

    // Channel
    buffer[p++] = channel;

    // Sender Role
    buffer[p++] = senderRole;

    // Filler up to 90 bytes for consistent packet size
    while (p < 90) {
        buffer[p] = (uint8_t)(0xA5 ^ (sequence & 0xFF));
        p++;
    }

    return p;
}

#endif // RF_TEST_PROTOCOL_H
