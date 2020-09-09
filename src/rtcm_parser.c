#include "rtcm_parser.h"
#include "crc24.h"
#include <string.h>
#include <modules/uavcan_debug/uavcan_debug.h>

enum message_validity_t {
    MESSAGE_EMPTY,
    MESSAGE_INVALID,
    MESSAGE_INCOMPLETE,
    MESSAGE_VALID
};

static enum message_validity_t check_rtcm_message(const uint8_t* frame_buf, const size_t frame_len, size_t* msg_len);

void rtcm_parser_init(struct rtcm_parser_s* instance, rtcm_frame_cb_t rtcm_frame_cb, void* ctx) {
    instance->rtcm_frame_buf_len = 0;
    instance->rtcm_frame_cb = rtcm_frame_cb;
    instance->ctx = ctx;
}

void rtcm_parser_push_byte(struct rtcm_parser_s* instance, uint8_t byte) {
    if (instance->rtcm_frame_buf_len >= RTCM_MAX_MSG_LEN) {
        return;
    }

    instance->rtcm_frame_buf[instance->rtcm_frame_buf_len++] = byte;

    size_t msg_len;
    enum message_validity_t msg_validity = check_rtcm_message(instance->rtcm_frame_buf, instance->rtcm_frame_buf_len, &msg_len);

    if (msg_validity == MESSAGE_INVALID) {
        memmove(instance->rtcm_frame_buf, &instance->rtcm_frame_buf[1], --instance->rtcm_frame_buf_len);
    } else if (msg_validity == MESSAGE_VALID) {
        instance->rtcm_frame_cb(msg_len, instance->rtcm_frame_buf, instance->ctx);
        instance->rtcm_frame_buf_len = 0;
    }
}

static enum message_validity_t check_rtcm_message(const uint8_t* frame_buf, const size_t frame_len, size_t* msg_len) {
    if (frame_len == 0) {
        return MESSAGE_EMPTY;
    }

    if (frame_len >= 1 && frame_buf[0] != 0b11010011) {
        return MESSAGE_INVALID;
    }

    if (frame_len >= 2 && (frame_buf[1]&0b11111100) != 0) { // reserved field should be 0
        return MESSAGE_INVALID;
    }

    if (frame_len < 6) {
        return MESSAGE_INCOMPLETE;
    }

    size_t len_field_val = (uint16_t)frame_buf[1]<<8 | frame_buf[2];

    if (len_field_val+6 > RTCM_MAX_MSG_LEN || len_field_val == 0) {
        return MESSAGE_INVALID;
    }

    if (frame_len < len_field_val+6) {
        return MESSAGE_INCOMPLETE;
    }

    uint32_t crc_provided = (uint32_t)frame_buf[len_field_val+3]<<16 | (uint32_t)frame_buf[len_field_val+4]<<8 | (uint32_t)frame_buf[len_field_val+5]<<0;
    uint32_t crc_computed = crc24(len_field_val+3, frame_buf, 0);

    if (crc_provided == crc_computed) {
        *msg_len = len_field_val+6;
        return MESSAGE_VALID;
    } else {
        uavcan_send_debug_msg(UAVCAN_PROTOCOL_DEBUG_LOGLEVEL_DEBUG, "", "cksum fail %u %08X %08X", len_field_val, crc_provided, crc_computed);
        return MESSAGE_INVALID;
    }
}
