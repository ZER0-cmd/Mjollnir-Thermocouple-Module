#include "aesir_protocol.h"

#include <string.h>

#define AESIR_CRC_POLY 0x1021u
#define AESIR_CRC_INIT 0xffffu

typedef struct {
    uint8_t address;
    uint16_t payload_length;
    uint8_t type;
    uint8_t priority;
    uint16_t message_id;
    uint8_t fragment_index;
    uint8_t fragment_count;
    const uint8_t *payload;
} aesir_decoded_frame_t;

static uint16_t read_u16_be(const uint8_t *data)
{
    return (uint16_t)(((uint16_t)data[0] << 8) | data[1]);
}

static void write_u16_be(uint8_t *data, uint16_t value)
{
    data[0] = (uint8_t)(value >> 8);
    data[1] = (uint8_t)value;
}

uint16_t aesir_crc16(const uint8_t *data, size_t length)
{
    uint16_t crc = AESIR_CRC_INIT;
    size_t i;

    if (data == NULL && length != 0u) {
        return 0u;
    }

    for (i = 0u; i < length; ++i) {
        unsigned int bit;

        crc ^= (uint16_t)data[i] << 8;
        for (bit = 0u; bit < 8u; ++bit) {
            if ((crc & 0x8000u) != 0u) {
                crc = (uint16_t)((crc << 1) ^ AESIR_CRC_POLY);
            } else {
                crc = (uint16_t)(crc << 1);
            }
        }
    }

    return crc;
}

static aesir_status_t encode_frame(
    const aesir_message_t *message,
    uint8_t fragment_index,
    uint8_t fragment_count,
    const uint8_t *payload,
    size_t payload_length,
    uint8_t *frame,
    size_t frame_capacity,
    size_t *frame_length)
{
    size_t needed;
    uint16_t crc;

    if (message == NULL || frame == NULL || frame_length == NULL) {
        return AESIR_ERR_NULL;
    }
    if (payload == NULL && payload_length != 0u) {
        return AESIR_ERR_NULL;
    }
    if (payload_length > AESIR_MAX_PAYLOAD_BYTES ||
        fragment_count == 0u ||
        fragment_index >= fragment_count) {
        return AESIR_ERR_ARGUMENT;
    }

    needed = AESIR_HEADER_BYTES + payload_length + AESIR_CRC_BYTES;
    if (frame_capacity < needed) {
        return AESIR_ERR_CAPACITY;
    }

    frame[0] = message->address;
    write_u16_be(&frame[1], (uint16_t)payload_length);
    frame[3] = message->type;
    frame[4] = message->priority;
    write_u16_be(&frame[5], message->message_id);
    frame[7] = fragment_index;
    frame[8] = fragment_count;
    if (payload_length != 0u) {
        memcpy(&frame[AESIR_HEADER_BYTES], payload, payload_length);
    }

    crc = aesir_crc16(frame, AESIR_HEADER_BYTES + payload_length);
    write_u16_be(&frame[AESIR_HEADER_BYTES + payload_length], crc);
    *frame_length = needed;
    return AESIR_STATUS_MESSAGE_READY;
}

static aesir_status_t decode_frame(
    const uint8_t *frame,
    size_t frame_length,
    aesir_decoded_frame_t *decoded)
{
    size_t expected_length;
    uint16_t received_crc;
    uint16_t calculated_crc;

    if (frame == NULL || decoded == NULL) {
        return AESIR_ERR_NULL;
    }
    if (frame_length < AESIR_HEADER_BYTES + AESIR_CRC_BYTES) {
        return AESIR_ERR_FRAME_TOO_SHORT;
    }

    decoded->payload_length = read_u16_be(&frame[1]);
    if (decoded->payload_length > AESIR_MAX_PAYLOAD_BYTES) {
        return AESIR_ERR_LENGTH;
    }

    expected_length =
        AESIR_HEADER_BYTES + (size_t)decoded->payload_length + AESIR_CRC_BYTES;
    if (frame_length != expected_length) {
        return AESIR_ERR_LENGTH;
    }

    received_crc = read_u16_be(&frame[AESIR_HEADER_BYTES + decoded->payload_length]);
    calculated_crc = aesir_crc16(frame, AESIR_HEADER_BYTES + decoded->payload_length);
    if (received_crc != calculated_crc) {
        return AESIR_ERR_CRC;
    }

    decoded->address = frame[0];
    decoded->type = frame[3];
    decoded->priority = frame[4];
    decoded->message_id = read_u16_be(&frame[5]);
    decoded->fragment_index = frame[7];
    decoded->fragment_count = frame[8];
    if (decoded->fragment_count == 0u ||
        decoded->fragment_index >= decoded->fragment_count) {
        return AESIR_ERR_METADATA;
    }

    decoded->payload = &frame[AESIR_HEADER_BYTES];
    return AESIR_STATUS_MESSAGE_READY;
}

static int same_message(
    const aesir_receiver_t *receiver,
    const aesir_decoded_frame_t *frame)
{
    return receiver->address == frame->address &&
           receiver->type == frame->type &&
           receiver->priority == frame->priority &&
           receiver->message_id == frame->message_id &&
           receiver->fragment_count == frame->fragment_count;
}

static aesir_status_t begin_message(
    aesir_receiver_t *receiver,
    const aesir_decoded_frame_t *frame)
{
    size_t required_capacity;

    if (frame->fragment_count == 1u) {
        required_capacity = frame->payload_length;
    } else {
        if ((size_t)frame->fragment_count > SIZE_MAX / AESIR_MAX_PAYLOAD_BYTES) {
            return AESIR_ERR_CAPACITY;
        }
        required_capacity =
            (size_t)frame->fragment_count * AESIR_MAX_PAYLOAD_BYTES;
    }

    if (required_capacity > receiver->capacity) {
        return AESIR_ERR_CAPACITY;
    }

    memset(receiver->received, 0, sizeof(receiver->received));
    memset(receiver->fragment_lengths, 0, sizeof(receiver->fragment_lengths));
    receiver->address = frame->address;
    receiver->type = frame->type;
    receiver->priority = frame->priority;
    receiver->message_id = frame->message_id;
    receiver->fragment_count = frame->fragment_count;
    receiver->received_count = 0u;
    receiver->total_length = 0u;
    receiver->active = 1;
    return AESIR_STATUS_WAITING;
}

static aesir_status_t store_fragment(
    aesir_receiver_t *receiver,
    const aesir_decoded_frame_t *frame)
{
    size_t offset;

    if ((size_t)frame->fragment_index >
        SIZE_MAX / AESIR_MAX_PAYLOAD_BYTES) {
        return AESIR_ERR_CAPACITY;
    }
    offset = (size_t)frame->fragment_index * AESIR_MAX_PAYLOAD_BYTES;
    if (offset > receiver->capacity ||
        frame->payload_length > receiver->capacity - offset) {
        return AESIR_ERR_CAPACITY;
    }

    if (receiver->received[frame->fragment_index] != 0u) {
        size_t old_length = receiver->fragment_lengths[frame->fragment_index];

        if (old_length == frame->payload_length &&
            memcmp(&receiver->storage[offset], frame->payload, old_length) == 0) {
            return AESIR_STATUS_WAITING;
        }
        return AESIR_ERR_DUPLICATE;
    }

    if (frame->payload_length > receiver->capacity - receiver->total_length) {
        return AESIR_ERR_CAPACITY;
    }

    if (frame->payload_length != 0u) {
        memcpy(&receiver->storage[offset], frame->payload, frame->payload_length);
    }
    receiver->fragment_lengths[frame->fragment_index] = frame->payload_length;
    receiver->received[frame->fragment_index] = 1u;
    ++receiver->received_count;
    receiver->total_length += frame->payload_length;
    return AESIR_STATUS_WAITING;
}

static void compact_fragments(aesir_receiver_t *receiver)
{
    size_t write_offset = 0u;
    uint8_t index;

    for (index = 0u; index < receiver->fragment_count; ++index) {
        size_t length = receiver->fragment_lengths[index];
        size_t read_offset = (size_t)index * AESIR_MAX_PAYLOAD_BYTES;

        if (length != 0u && read_offset != write_offset) {
            memmove(
                &receiver->storage[write_offset],
                &receiver->storage[read_offset],
                length);
        }
        write_offset += length;
    }
}

aesir_status_t aesir_receiver_init(
    aesir_receiver_t *receiver,
    uint8_t *storage,
    size_t storage_capacity,
    uint32_t timeout_ms)
{
    if (receiver == NULL || storage == NULL || storage_capacity == 0u) {
        return AESIR_ERR_NULL;
    }

    memset(receiver, 0, sizeof(*receiver));
    receiver->storage = storage;
    receiver->capacity = storage_capacity;
    receiver->timeout_ms = timeout_ms;
    return AESIR_STATUS_WAITING;
}

void aesir_receiver_reset(aesir_receiver_t *receiver)
{
    if (receiver != NULL) {
        receiver->active = 0;
        receiver->received_count = 0u;
        receiver->total_length = 0u;
    }
}

aesir_status_t aesir_send(
    const aesir_transport_t *transport,
    const aesir_message_t *message)
{
    uint8_t frame[AESIR_MAX_FRAME_BYTES];
    size_t fragment_count;
    size_t offset = 0u;
    size_t fragment_index;

    if (transport == NULL || message == NULL || transport->write_frame == NULL) {
        return AESIR_ERR_NULL;
    }
    if (message->payload == NULL && message->payload_length != 0u) {
        return AESIR_ERR_NULL;
    }

    fragment_count = message->payload_length / AESIR_MAX_PAYLOAD_BYTES;
    if (message->payload_length % AESIR_MAX_PAYLOAD_BYTES != 0u) {
        ++fragment_count;
    }
    if (fragment_count == 0u) {
        fragment_count = 1u;
    }
    if (fragment_count > AESIR_MAX_FRAGMENT_COUNT) {
        return AESIR_ERR_CAPACITY;
    }

    for (fragment_index = 0u;
         fragment_index < fragment_count;
         ++fragment_index) {
        size_t remaining = message->payload_length - offset;
        size_t chunk = remaining < AESIR_MAX_PAYLOAD_BYTES
                     ? remaining
                     : AESIR_MAX_PAYLOAD_BYTES;
        size_t frame_length;
        aesir_status_t status = encode_frame(
            message,
            (uint8_t)fragment_index,
            (uint8_t)fragment_count,
            message->payload == NULL ? NULL : &message->payload[offset],
            chunk,
            frame,
            sizeof(frame),
            &frame_length);

        if (status < 0) {
            return status;
        }
        if (transport->write_frame(frame, frame_length, transport->context) != 0) {
            return AESIR_ERR_TRANSPORT;
        }
        offset += chunk;
    }

    return AESIR_STATUS_MESSAGE_READY;
}

aesir_status_t aesir_receive(
    aesir_receiver_t *receiver,
    const uint8_t *frame,
    size_t frame_length,
    uint32_t now_ms,
    aesir_message_t *complete_message)
{
    aesir_decoded_frame_t decoded;
    aesir_status_t status;

    if (receiver == NULL || frame == NULL || complete_message == NULL) {
        return AESIR_ERR_NULL;
    }
    *complete_message = (aesir_message_t){0};

    if (receiver->active && receiver->timeout_ms != 0u &&
        (uint32_t)(now_ms - receiver->last_activity_ms) >
        receiver->timeout_ms) {
        aesir_receiver_reset(receiver);
        return AESIR_ERR_TIMEOUT;
    }

    status = decode_frame(frame, frame_length, &decoded);
    if (status < 0) {
        return status;
    }

    if (!receiver->active) {
        status = begin_message(receiver, &decoded);
        if (status < 0) {
            return status;
        }
    } else if (!same_message(receiver, &decoded)) {
        return AESIR_ERR_METADATA;
    }

    status = store_fragment(receiver, &decoded);
    receiver->last_activity_ms = now_ms;
    if (status < 0) {
        return status;
    }

    if (receiver->received_count != receiver->fragment_count) {
        return AESIR_STATUS_WAITING;
    }

    compact_fragments(receiver);
    complete_message->address = receiver->address;
    complete_message->type = receiver->type;
    complete_message->priority = receiver->priority;
    complete_message->message_id = receiver->message_id;
    complete_message->payload = receiver->storage;
    complete_message->payload_length = receiver->total_length;
    receiver->active = 0;
    return AESIR_STATUS_MESSAGE_READY;
}

const char *aesir_status_name(aesir_status_t status)
{
    switch (status) {
    case AESIR_STATUS_MESSAGE_READY:
        return "message-ready";
    case AESIR_STATUS_WAITING:
        return "waiting";
    case AESIR_ERR_NULL:
        return "null-argument";
    case AESIR_ERR_ARGUMENT:
        return "invalid-argument";
    case AESIR_ERR_FRAME_TOO_SHORT:
        return "frame-too-short";
    case AESIR_ERR_LENGTH:
        return "invalid-length";
    case AESIR_ERR_CRC:
        return "crc-failure";
    case AESIR_ERR_METADATA:
        return "invalid-metadata";
    case AESIR_ERR_CAPACITY:
        return "capacity-exceeded";
    case AESIR_ERR_DUPLICATE:
        return "conflicting-duplicate";
    case AESIR_ERR_TIMEOUT:
        return "reassembly-timeout";
    case AESIR_ERR_TRANSPORT:
        return "transport-failure";
    default:
        return "unknown-status";
    }
}
