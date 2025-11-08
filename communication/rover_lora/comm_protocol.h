

#ifndef COMM_PROTOCOL_H
#define COMM_PROTOCOL_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Packet types
#define PKT_TYPE_HEARTBEAT  0x01
#define PKT_TYPE_ANALYSIS   0x02
#define PKT_TYPE_CMD        0x03
#define PKT_TYPE_ACK        0x04

// Max payload sizes
#define MAX_PAYLOAD 200

#pragma pack(push,1)
typedef struct {
    uint16_t device_id;   // unique id for robot
    uint32_t seq;         // monotonic sequence
    uint32_t ts;          // seconds since boot/mission
    uint8_t  phase;       // mission phase enum
    uint8_t  batt_pct;    // battery percentage 0-100
    int16_t  rssi;        // optional sender-side rssi placeholder
    uint16_t crc;         // CRC16 over preceding bytes
} Heartbeat_t;

typedef struct {
    uint16_t device_id;
    uint32_t seq;
    uint32_t ts;
    int16_t  pH_x100;     // pH * 100 (e.g., 7.34 -> 734)
    uint16_t turbidity;   // NTU
    uint16_t ATP;         // RLU or similar
    uint8_t  flags;       // bitflags
    uint16_t crc;
} Analysis_t;

typedef struct {
    uint8_t cmd;          // 1=E_STOP,2=PAUSE,3=BURST_OPEN,4=SET_SF
    uint8_t param;        // e.g., seconds or SF value
    uint32_t ack_seq;     // sequence to ack (if any)
    uint16_t crc;
} Cmd_t;

typedef struct {
    uint8_t type;         // PKT_TYPE_* to identify payload
    uint16_t length;      // length of payload following header
    uint8_t payload[MAX_PAYLOAD];
} GenericPkt_t;
#pragma pack(pop)

// CRC-16 CCITT (0x1021) implementation
static inline uint16_t crc16_ccitt(const uint8_t *data, uint16_t len) {
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x8000) crc = (crc << 1) ^ 0x1021;
            else crc <<= 1;
        }
    }
    return crc;
}

// Helper to compute and append CRC to heartbeat struct (caller must ensure memory)
static inline void hb_compute_crc(Heartbeat_t *hb) {
    hb->crc = 0;
    hb->crc = crc16_ccitt((const uint8_t*)hb, sizeof(Heartbeat_t) - sizeof(uint16_t));
}
static inline int hb_check_crc(const Heartbeat_t *hb) {
    uint16_t crc = crc16_ccitt((const uint8_t*)hb, sizeof(Heartbeat_t) - sizeof(uint16_t));
    return (crc == hb->crc);
}

static inline void an_compute_crc(Analysis_t *a) {
    a->crc = 0;
    a->crc = crc16_ccitt((const uint8_t*)a, sizeof(Analysis_t) - sizeof(uint16_t));
}
static inline int an_check_crc(const Analysis_t *a) {
    uint16_t crc = crc16_ccitt((const uint8_t*)a, sizeof(Analysis_t) - sizeof(uint16_t));
    return (crc == a->crc);
}

static inline void cmd_compute_crc(Cmd_t *c) {
    c->crc = 0;
    c->crc = crc16_ccitt((const uint8_t*)c, sizeof(Cmd_t) - sizeof(uint16_t));
}
static inline int cmd_check_crc(const Cmd_t *c) {
    uint16_t crc = crc16_ccitt((const uint8_t*)c, sizeof(Cmd_t) - sizeof(uint16_t));
    return (crc == c->crc);
}

#ifdef __cplusplus
}
#endif

#endif // COMM_PROTOCOL_H

