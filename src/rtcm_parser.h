#pragma once
#include <stdint.h>
#include <modules/pubsub/pubsub.h>

#define RTCM_MAX_MSG_LEN 400

typedef void (*rtcm_frame_cb_t)(size_t frame_len, uint8_t* frame, void* ctx);

struct rtcm_parser_s {
    uint8_t rtcm_frame_buf[RTCM_MAX_MSG_LEN];
    uint16_t rtcm_frame_buf_len;
    rtcm_frame_cb_t rtcm_frame_cb;
    void* ctx;
};

void rtcm_parser_init(struct rtcm_parser_s* instance, rtcm_frame_cb_t rtcm_frame_cb, void* ctx);
void rtcm_parser_push_byte(struct rtcm_parser_s* instance, uint8_t byte);
