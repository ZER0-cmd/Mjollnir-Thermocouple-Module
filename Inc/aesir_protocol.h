#ifndef AESIR_PROTOCOL_H
#define AESIR_PROTOCOL_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define AESIR_MAX_FRAME_BYTES       256u
#define AESIR_HEADER_BYTES          9u
#define AESIR_CRC_BYTES             2u
#define AESIR_MAX_PAYLOAD_BYTES     (AESIR_MAX_FRAME_BYTES - AESIR_HEADER_BYTES - AESIR_CRC_BYTES)
#define AESIR_MAX_FRAGMENT_COUNT    255u
#define AESIR_BROADCAST_ADDRESS     0xffu

typedef enum {
    AESIR_TYPE_COMMAND = 1,
    AESIR_TYPE_DATA = 2,
    AESIR_TYPE_ACK = 3
} aesir_message_type_t;

typedef enum {
    AESIR_PRIORITY_RF_COMMAND = 0,
    AESIR_PRIORITY_CONTROL = 1,
    AESIR_PRIORITY_SENSOR = 2,
    AESIR_PRIORITY_LOG = 3
} aesir_priority_t;

typedef enum {
    AESIR_STATUS_MESSAGE_READY = 1,
    AESIR_STATUS_WAITING = 0,
    AESIR_ERR_NULL = -1,
    AESIR_ERR_ARGUMENT = -2,
    AESIR_ERR_FRAME_TOO_SHORT = -3,
    AESIR_ERR_LENGTH = -4,
    AESIR_ERR_CRC = -5,
    AESIR_ERR_METADATA = -6,
    AESIR_ERR_CAPACITY = -7,
    AESIR_ERR_DUPLICATE = -8,
    AESIR_ERR_TIMEOUT = -9,
    AESIR_ERR_TRANSPORT = -10
} aesir_status_t;

typedef struct {
    uint8_t address;
    uint8_t type;
    uint8_t priority;
    uint16_t message_id;
    const uint8_t *payload;
    size_t payload_length;
} aesir_message_t;

typedef int (*aesir_write_frame_fn)(
    const uint8_t *frame,
    size_t frame_length,
    void *context);

typedef struct {
    aesir_write_frame_fn write_frame;
    void *context;
} aesir_transport_t;

typedef struct {
    uint8_t *storage;
    size_t capacity;
    uint32_t timeout_ms;
    uint32_t last_activity_ms;
    uint8_t address;
    uint8_t type;
    uint8_t priority;
    uint16_t message_id;
    uint8_t fragment_count;
    uint8_t received_count;
    uint8_t received[AESIR_MAX_FRAGMENT_COUNT];
    uint16_t fragment_lengths[AESIR_MAX_FRAGMENT_COUNT];
    size_t total_length;
    int active;
} aesir_receiver_t;

/* CRC-16/CCITT-FALSE: poly 0x1021, init 0xffff, no reflection, xorout 0. */
uint16_t aesir_crc16(const uint8_t *data, size_t length);

void aesir_receiver_reset(aesir_receiver_t *receiver);

aesir_status_t aesir_receiver_init(
    aesir_receiver_t *receiver,
    uint8_t *storage,
    size_t storage_capacity,
    uint32_t timeout_ms);

/*
 * Send one logical message. The callback must copy or consume the frame
 * before it returns. A non-zero callback result becomes AESIR_ERR_TRANSPORT.
 */
aesir_status_t aesir_send(
    const aesir_transport_t *transport,
    const aesir_message_t *message);

/*
 * Feed exactly one encoded frame. now_ms is supplied by the caller.
 * A complete result points into receiver-owned storage and remains valid
 * until the next receive/reset operation.
 */
aesir_status_t aesir_receive(
    aesir_receiver_t *receiver,
    const uint8_t *frame,
    size_t frame_length,
    uint32_t now_ms,
    aesir_message_t *complete_message);

const char *aesir_status_name(aesir_status_t status);

#ifdef __cplusplus
}
#endif

#endif
