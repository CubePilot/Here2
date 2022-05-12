#include "ch.h"
#include "hal.h"
#include <modules/timing/timing.h>
#include <common/helpers.h>
#include <modules/can/can.h>
#include <modules/gps/gps.h>
#include <modules/param/param.h>
#include <modules/boot_msg/boot_msg.h>
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <modules/uavcan/uavcan.h>
#include <modules/pubsub/pubsub.h>
#include <modules/worker_thread/worker_thread.h>
#include <modules/uavcan_debug/uavcan_debug.h>
#include <modules/can/can.h>
#include <uavcan.equipment.gnss.Fix.h>
#include <uavcan.equipment.gnss.Fix2.h>
#include <uavcan.equipment.gnss.Auxiliary.h>
#include <uavcan.equipment.gnss.RTCMStream.h>
#include <ubx_msgs.h>
#include <time.h>
#include "rtcm_parser.h"

#define WT lpwork_thread
WORKER_THREAD_DECLARE_EXTERN(WT)

#define UBX_MSG_TOPIC_GROUP PUBSUB_DEFAULT_TOPIC_GROUP
PUBSUB_TOPIC_GROUP_DECLARE_EXTERN(UBX_MSG_TOPIC_GROUP)

PARAM_DEFINE_UINT8_PARAM_STATIC(nav5_dynModel, "dynModel", 8, 0, 10)
PARAM_DEFINE_UINT8_PARAM_STATIC(dgnssMode, "dgnssMode", 3, 2, 3)
PARAM_DEFINE_UINT8_PARAM_STATIC(gnssConfig, "gnssConfig", 97, 1, 127)
PARAM_DEFINE_UINT8_PARAM_STATIC(passThrough, "passThrough", 0, 0, 2)

// #define gps_debug(msg, fmt, args...) do {uavcan_send_debug_msg(LOG_LEVEL_DEBUG, msg, fmt, ## args); } while(0);
#define gps_debug(msg, fmt, args...)

#define GNSS_MEAS_RATE 200

#define GPS_CFG_BAUD 230400U
static const uint32_t baudrates[] = {9600U, 115200U, 4800U, 19200U, 38400U, 57600U, 230400U, 460800U};
char init_blob[] = "\265\142\006\001\003\000\001\006\001\022\117$PUBX,41,1,0023,0021,230400,0*1C\r\n";

// #define GPS_CFG_BAUD 115200U
// static const uint32_t baudrates[] = {9600U, 115200U, 4800U, 19200U, 38400U, 57600U, 230400U};
// char init_blob[] = "\265\142\006\001\003\000\001\006\001\022\117$PUBX,41,1,0023,0001,115200,0*1C\r\n";

// #define GPS_CFG_BAUD 57600U
// static const uint32_t baudrates[] = {9600U, 115200U, 4800U, 19200U, 38400U, 57600U, 230400U};
// char init_blob[] = "\265\142\006\001\003\000\001\006\001\022\117$PUBX,41,1,0023,0001,57600,0*2F\r\n";
#define UBX_PORT_ID 1

static SerialConfig gps_default_sercfg =
{
  9600,
  0,
  USART_CR2_STOP1_BITS,
  0
};

struct  __attribute__((__packed__)) ubx_header {
    uint8_t hdr[2];
    uint8_t class_id;
    uint8_t msg_id;
    uint16_t payload_len;
};

struct ubx_gps_handle_s {
    bool initialised;
    bool configured;
    SerialDriver *serial;
    SerialConfig sercfg;
    uint8_t baudrate_index;
    uint8_t cfg_step;
    uint8_t cfg_msg_index;
    bool do_cfg;
    uint32_t last_baud_change_ms;
    struct worker_thread_listener_task_s gps_inject_listener_task;
    uint8_t total_msg_cfgs;
    uint8_t attempt;
    uint32_t dop_iTOW;
    uint32_t pvt_iTOW;
    struct uavcan_equipment_gnss_Auxiliary_s aux_msg;
} ubx_handle;

struct ubx_msg_cfg_s {
    uint8_t class_id;
    uint8_t msg_id;
    uint8_t rate;
    struct pubsub_topic_s *topic;
    struct worker_thread_listener_task_s *listener_task;
    pubsub_message_handler_func_ptr handler;
};

//static struct rtcm_parser_s rtcm_parser;
static struct gps_handle_s gps_handle;
static void ubx_gps_spinner(void *ctx);
static void ubx_init(struct ubx_gps_handle_s *ubx_handle, SerialDriver* serial, SerialConfig *sercfg);
static void ubx_gps_init_loop(struct worker_thread_timer_task_s *task);
static void send_init_blob(SerialDriver* serial);
static void ubx_gps_configure_msgs(void);
static void send_message(uint8_t class_id, uint8_t msg_id, uint8_t* payload, size_t payload_len);
static void request_message(uint8_t class_id, uint8_t msg_id);

uint8_t parsed_msg_buffer[1024];

enum ubx_cfg_steps {
    STEP_CFG_RATE,
    STEP_CFG_NAV5,
    STEP_CFG_MSG,
    STEP_CFG_DGNSS,
    STEP_CFG_GNSS,    
    STEP_CFG_COMPLETE
};

//Msg topics and listeners
//rxm-rtcm
struct pubsub_topic_s ubx_rxm_rtcm_topic;
struct worker_thread_listener_task_s ubx_rxm_rtcm_listener;
static void ubx_rxm_rtcm_handler(size_t msg_size, const void* msg, void* ctx);

//NAV-SOL
struct pubsub_topic_s ubx_nav_sol_topic;
struct worker_thread_listener_task_s ubx_nav_sol_listener;
static void ubx_nav_sol_handler(size_t msg_size, const void* msg, void* ctx);

//ACK-ACK
struct pubsub_topic_s ubx_ack_ack_topic;
struct worker_thread_listener_task_s ubx_ack_ack_listener;
static void ubx_ack_ack_handler(size_t msg_size, const void* msg, void* ctx);

//CFG-RATE
struct pubsub_topic_s ubx_cfg_rate_topic;
struct worker_thread_listener_task_s ubx_cfg_rate_listener;
static void ubx_cfg_rate_handler(size_t msg_size, const void* msg, void* ctx);

//CFG-MSG1
struct pubsub_topic_s ubx_cfg_msg_topic;
struct worker_thread_listener_task_s ubx_cfg_msg_listener;
static void ubx_cfg_msg_handler(size_t msg_size, const void* msg, void* ctx);

//NAV-SVINFO
struct pubsub_topic_s ubx_nav_svinfo_topic;
struct worker_thread_listener_task_s ubx_nav_svinfo_listener;
static void ubx_nav_svinfo_handler(size_t msg_size, const void* msg, void* ctx);

//NAV-DOP
struct pubsub_topic_s ubx_nav_dop_topic;
struct worker_thread_listener_task_s ubx_nav_dop_listener;
static void ubx_nav_dop_handler(size_t msg_size, const void* msg, void* ctx);

// NAV-POSLLH
//static void ubx_nav_posllh_handler(size_t msg_size, const void* msg, void* ctx);
// NAV-VELNED
//static void ubx_nav_velned_handler(size_t msg_size, const void* msg, void* ctx);
// NAV-PVT
struct pubsub_topic_s ubx_nav_pvt_topic;
struct worker_thread_listener_task_s ubx_nav_pvt_listener;
static void ubx_nav_pvt_handler(size_t msg_size, const void* msg, void* ctx);
//NAV-STATUS
//static void ubx_nav_status_handler(size_t msg_size, const void* msg, void* ctx);

// CFG-NAV5
struct pubsub_topic_s ubx_cfg_nav5_topic;
struct worker_thread_listener_task_s ubx_cfg_nav5_listener;
static void ubx_cfg_nav5_handler(size_t msg_size, const void* msg, void* ctx);

// CFG-dgnss
struct pubsub_topic_s ubx_cfg_dgnss_topic;
struct worker_thread_listener_task_s ubx_cfg_dgnss_listener;
static void ubx_cfg_dgnss_handler(size_t msg_size, const void* msg, void* ctx);

// CFG-gnss
struct pubsub_topic_s ubx_cfg_gnss_topic;
struct worker_thread_listener_task_s ubx_cfg_gnss_listener;
static void ubx_cfg_gnss_handler(size_t msg_size, const void* msg, void* ctx);

// NAV-EOE
struct pubsub_topic_s ubx_nav_eoe_topic;
struct worker_thread_listener_task_s ubx_nav_eoe_listener;
static void ubx_nav_eoe_handler(size_t msg_size, const void* msg, void* ctx);

// MON-VER
struct pubsub_topic_s ubx_mon_ver_topic;
struct worker_thread_listener_task_s ubx_mon_ver_listener;
static void ubx_mon_ver_handler(size_t msg_size, const void* msg, void* ctx);

// MON-IO
struct pubsub_topic_s ubx_mon_io_topic;
struct worker_thread_listener_task_s ubx_mon_io_listener;
static void ubx_mon_io_handler(size_t msg_size, const void* msg, void* ctx);

// MON-MSGPP
struct pubsub_topic_s ubx_mon_msgpp_topic;
struct worker_thread_listener_task_s ubx_mon_msgpp_listener;
static void ubx_mon_msgpp_handler(size_t msg_size, const void* msg, void* ctx);

// MON-RXBUF
struct pubsub_topic_s ubx_mon_rxbuf_topic;
struct worker_thread_listener_task_s ubx_mon_rxbuf_listener;
static void ubx_mon_rxbuf_handler(size_t msg_size, const void* msg, void* ctx);

static void gps_inject_handler(size_t msg_size, const void* buf, void* ctx);

struct ubx_msg_cfg_s ubx_cfg_list[] = {
//     {UBX_NAV_SVINFO_CLASS_ID, UBX_NAV_SVINFO_MSG_ID, 4, &ubx_nav_svinfo_topic, &ubx_nav_svinfo_listener, ubx_nav_svinfo_handler},
    {UBX_NAV_PVT_CLASS_ID, UBX_NAV_PVT_MSG_ID, 1, &ubx_nav_pvt_topic, &ubx_nav_pvt_listener, ubx_nav_pvt_handler},
    {UBX_NAV_EOE_CLASS_ID, UBX_NAV_EOE_MSG_ID, 1, &ubx_nav_eoe_topic, &ubx_nav_eoe_listener, ubx_nav_eoe_handler},
    {UBX_NAV_DOP_CLASS_ID, UBX_NAV_DOP_MSG_ID, 5, &ubx_nav_dop_topic, &ubx_nav_dop_listener, ubx_nav_dop_handler},

/*   base only
    {0xF5, 0x4D, 1, NULL, NULL, NULL},
    {0xF5, 0x57, 1, NULL, NULL, NULL},
    {0xF5, 0x61, 1, NULL, NULL, NULL},
    {0xF5, 0x7f, 1, NULL, NULL, NULL},
    {0xF5, 0xe6, 10, NULL, NULL, NULL},
*/
    // IO debug messages
//     {UBX_RXM_RTCM_CLASS_ID, UBX_RXM_RTCM_MSG_ID, 1, &ubx_rxm_rtcm_topic, &ubx_rxm_rtcm_listener, ubx_rxm_rtcm_handler},
//     {UBX_MON_IO_CLASS_ID, UBX_MON_IO_MSG_ID, 1, &ubx_mon_io_topic, &ubx_mon_io_listener, ubx_mon_io_handler},
//     {UBX_MON_MSGPP_CLASS_ID, UBX_MON_MSGPP_MSG_ID, 1, &ubx_mon_msgpp_topic, &ubx_mon_msgpp_listener, ubx_mon_msgpp_handler},
//     {UBX_MON_RXBUF_CLASS_ID, UBX_MON_RXBUF_MSG_ID, 1, &ubx_mon_rxbuf_topic, &ubx_mon_rxbuf_listener, ubx_mon_rxbuf_handler},
};

static struct worker_thread_timer_task_s init_task;


THD_WORKING_AREA(ubx_gps_thd_wa, 512);

static void init_task_func(struct worker_thread_timer_task_s *task) {
    if (get_boot_msg_valid() && boot_msg_id == SHARED_MSG_BOOT_INFO && boot_msg.boot_info_msg.boot_reason == 127) {
        return;
    }

    bool have_received_can_msg = false;

    struct can_instance_s* can_instance = NULL;
    while (can_iterate_instances(&can_instance)) {
        if (can_get_baudrate_confirmed(can_instance)) {
            have_received_can_msg = true;
        }
    }

    if (have_received_can_msg) {
        board_gps_uart_init();

        gps_init(&gps_handle);
        ubx_init(&ubx_handle, &GPS_SERIAL, &gps_default_sercfg);
        chThdCreateStatic(ubx_gps_thd_wa,
                            sizeof(ubx_gps_thd_wa),
                            HIGHPRIO,               // Initial priority.
                            ubx_gps_spinner,             // Thread function.
                            NULL);              // Thread parameter.
        worker_thread_add_timer_task(&WT, &init_task, ubx_gps_init_loop, &ubx_handle, LL_MS2ST(100), false);
    } else {
        worker_thread_timer_task_reschedule(&WT, &init_task, LL_MS2ST(10));
    }
}

static struct uavcan_equipment_gnss_RTCMStream_s rtcm;

static void rtcm_frame_cb(size_t frame_len, uint8_t* frame, void* ctx) {
    //uint16_t type = ((uint16_t)frame[3]<<8 | frame[4]) >> 4;
    //uavcan_send_debug_msg(LOG_LEVEL_INFO, "GPS", "rtcm parse %u", (unsigned)type);
    for (uint a=0; a < frame_len; a+=128) {
        uint8_t size = frame_len - a < 128 ? frame_len - a : 128;

        rtcm.protocol_id = 3;
        rtcm.data_len = size;
        memcpy(rtcm.data, (uint8_t*)&frame[a], size);

        uavcan_broadcast(0, &uavcan_equipment_gnss_RTCMStream_descriptor, CANARD_TRANSFER_PRIORITY_HIGH, &rtcm);
    }
}

RUN_AFTER(INIT_END) {
    //rtcm_parser_init(&rtcm_parser, rtcm_frame_cb, NULL);
    worker_thread_add_timer_task(&WT, &init_task, init_task_func, NULL, LL_MS2ST(10), false);
}

static void ubx_init(struct ubx_gps_handle_s *handle, SerialDriver* serial, SerialConfig *sercfg)
{
    if (handle == NULL) {
        return;
    }
    memset(handle, 0, sizeof(struct ubx_gps_handle_s));
    memcpy(&handle->sercfg, sercfg, sizeof(SerialConfig));
    handle->serial = serial;
    handle->last_baud_change_ms = millis();
    sdStart(handle->serial, &handle->sercfg);
    send_init_blob(handle->serial);
    handle->total_msg_cfgs = sizeof(ubx_cfg_list)/sizeof(struct ubx_msg_cfg_s);
    //Setup UBX message subscribers
    //NAV-SOL
    pubsub_init_topic(&ubx_nav_sol_topic, &UBX_MSG_TOPIC_GROUP);
    if (gps_ubx_init_msg_topic(&gps_handle, UBX_NAV_SOL_CLASS_ID, UBX_NAV_SOL_MSG_ID, parsed_msg_buffer, sizeof(parsed_msg_buffer), &ubx_nav_sol_topic)) {
        worker_thread_add_listener_task(&WT, &ubx_nav_sol_listener, &ubx_nav_sol_topic, ubx_nav_sol_handler, handle);
        uavcan_send_debug_msg(LOG_LEVEL_INFO, "GPS", "Registered Topic for 0x%x 0x%x", UBX_NAV_SOL_CLASS_ID, UBX_NAV_SOL_MSG_ID);
    }
    //ACK-ACK
    pubsub_init_topic(&ubx_ack_ack_topic, &UBX_MSG_TOPIC_GROUP);
    if (gps_ubx_init_msg_topic(&gps_handle, UBX_ACK_ACK_CLASS_ID, UBX_ACK_ACK_MSG_ID, parsed_msg_buffer, sizeof(parsed_msg_buffer), &ubx_ack_ack_topic)) {
        worker_thread_add_listener_task(&WT, &ubx_ack_ack_listener, &ubx_ack_ack_topic, ubx_ack_ack_handler, handle);
        uavcan_send_debug_msg(LOG_LEVEL_INFO, "GPS", "Registered Topic for 0x%x 0x%x", UBX_ACK_ACK_CLASS_ID, UBX_ACK_ACK_MSG_ID);
    }

    //CFG-RATE
    pubsub_init_topic(&ubx_cfg_rate_topic, &UBX_MSG_TOPIC_GROUP);
    if (gps_ubx_init_msg_topic(&gps_handle, UBX_CFG_RATE_CLASS_ID, UBX_CFG_RATE_MSG_ID, parsed_msg_buffer, sizeof(parsed_msg_buffer), &ubx_cfg_rate_topic)) {
        worker_thread_add_listener_task(&WT, &ubx_cfg_rate_listener, &ubx_cfg_rate_topic, ubx_cfg_rate_handler, handle);
        uavcan_send_debug_msg(LOG_LEVEL_INFO, "GPS", "Registered Topic for 0x%x 0x%x", UBX_CFG_RATE_CLASS_ID, UBX_CFG_RATE_MSG_ID);
    }

    //CFG-MSG
    pubsub_init_topic(&ubx_cfg_msg_topic, &UBX_MSG_TOPIC_GROUP);
    if (gps_ubx_init_msg_topic(&gps_handle, UBX_CFG_MSG_CLASS_ID, UBX_CFG_MSG_MSG_ID, parsed_msg_buffer, sizeof(parsed_msg_buffer), &ubx_cfg_msg_topic)) {
        worker_thread_add_listener_task(&WT, &ubx_cfg_msg_listener, &ubx_cfg_msg_topic, ubx_cfg_msg_handler, handle);
        uavcan_send_debug_msg(LOG_LEVEL_INFO, "GPS", "Registered Topic for 0x%x 0x%x", UBX_CFG_MSG_CLASS_ID, UBX_CFG_MSG_MSG_ID);
    }

    //CFG-NAV5
    pubsub_init_topic(&ubx_cfg_nav5_topic, &UBX_MSG_TOPIC_GROUP);
    if (gps_ubx_init_msg_topic(&gps_handle, UBX_CFG_NAV5_CLASS_ID, UBX_CFG_NAV5_MSG_ID, parsed_msg_buffer, sizeof(parsed_msg_buffer), &ubx_cfg_nav5_topic)) {
        worker_thread_add_listener_task(&WT, &ubx_cfg_nav5_listener, &ubx_cfg_nav5_topic, ubx_cfg_nav5_handler, handle);
        uavcan_send_debug_msg(LOG_LEVEL_INFO, "GPS", "Registered Topic for 0x%x 0x%x", UBX_CFG_NAV5_CLASS_ID, UBX_CFG_NAV5_MSG_ID);
    }
    
    pubsub_init_topic(&ubx_cfg_dgnss_topic, &UBX_MSG_TOPIC_GROUP);
    if (gps_ubx_init_msg_topic(&gps_handle, UBX_CFG_DGNSS_CLASS_ID, UBX_CFG_DGNSS_MSG_ID, parsed_msg_buffer, sizeof(parsed_msg_buffer), &ubx_cfg_dgnss_topic)) {
        worker_thread_add_listener_task(&WT, &ubx_cfg_dgnss_listener, &ubx_cfg_dgnss_topic, ubx_cfg_dgnss_handler, handle);
        uavcan_send_debug_msg(LOG_LEVEL_INFO, "GPS", "Registered Topic for 0x%x 0x%x", UBX_CFG_DGNSS_CLASS_ID, UBX_CFG_DGNSS_MSG_ID);
    }
    
    pubsub_init_topic(&ubx_cfg_gnss_topic, &UBX_MSG_TOPIC_GROUP);
    if (gps_ubx_init_msg_topic(&gps_handle, UBX_CFG_GNSS_CLASS_ID, UBX_CFG_GNSS_MSG_ID, parsed_msg_buffer, sizeof(parsed_msg_buffer), &ubx_cfg_gnss_topic)) {
        worker_thread_add_listener_task(&WT, &ubx_cfg_gnss_listener, &ubx_cfg_gnss_topic, ubx_cfg_gnss_handler, handle);
        uavcan_send_debug_msg(LOG_LEVEL_INFO, "GPS", "Registered Topic for 0x%x 0x%x", UBX_CFG_GNSS_CLASS_ID, UBX_CFG_GNSS_MSG_ID);
    }

    //MON-VER
    pubsub_init_topic(&ubx_mon_ver_topic, &UBX_MSG_TOPIC_GROUP);
    if (gps_ubx_init_msg_topic(&gps_handle, UBX_MON_VER_CLASS_ID, UBX_MON_VER_MSG_ID, parsed_msg_buffer, sizeof(parsed_msg_buffer), &ubx_mon_ver_topic)) {
        worker_thread_add_listener_task(&WT, &ubx_mon_ver_listener, &ubx_mon_ver_topic, ubx_mon_ver_handler, handle);
        uavcan_send_debug_msg(LOG_LEVEL_INFO, "GPS", "Registered Topic for 0x%x 0x%x", UBX_MON_VER_CLASS_ID, UBX_MON_VER_MSG_ID);
    }

    //Register Messages in the cfg list
    for (uint8_t i = 0; i < handle->total_msg_cfgs; i++) {
        if(ubx_cfg_list[i].topic == NULL)
            continue;
        pubsub_init_topic(ubx_cfg_list[i].topic, &UBX_MSG_TOPIC_GROUP);
        if (gps_ubx_init_msg_topic(&gps_handle, ubx_cfg_list[i].class_id, ubx_cfg_list[i].msg_id, parsed_msg_buffer, sizeof(parsed_msg_buffer), ubx_cfg_list[i].topic)) {
            worker_thread_add_listener_task(&WT, ubx_cfg_list[i].listener_task, ubx_cfg_list[i].topic, ubx_cfg_list[i].handler, handle);
            uavcan_send_debug_msg(LOG_LEVEL_INFO, "GPS", "Registered Topic for 0x%x 0x%x", ubx_cfg_list[i].class_id, ubx_cfg_list[i].msg_id);
        }
    }

    //Register Listener to GPS Inject Message
#define UAVCAN_EQUIPMENT_GNSS_RTCM_STREAM_DTID 1062
    can_add_filter(can_get_instance(0), 0x60FFFF80, 0x20000000 | ((uint32_t)UAVCAN_EQUIPMENT_GNSS_RTCM_STREAM_DTID<<8));
    struct pubsub_topic_s* gps_inject_topic = can_get_rx_topic(can_get_instance(0));
    worker_thread_add_listener_task(&WT, &handle->gps_inject_listener_task, gps_inject_topic, gps_inject_handler, handle);
}

static void send_init_blob(SerialDriver* serial)
{
    uint8_t train[2] = {0x55,0x55};
    sdWrite(serial, train, 2);
    sdWrite(serial, (const uint8_t*)init_blob, sizeof(init_blob)-1);
}

static void send_message(uint8_t class_id, uint8_t msg_id, uint8_t* payload, size_t payload_len)
{
    struct ubx_header header;

    header.hdr[0] = 0xB5;
    header.hdr[1] = 0x62;
    header.class_id = class_id;
    header.msg_id = msg_id;
    header.payload_len = payload_len;
    uint8_t crc[2] = {0};

    for(uint16_t i = 2; i < sizeof(header); i++) {
        crc[0] += ((uint8_t*)&header)[i];
        crc[1] += crc[0];
    }
    for(uint16_t i = 0; i < payload_len; i++) {
        crc[0] += payload[i];
        crc[1] += crc[0];
    }
    sdWrite(ubx_handle.serial, (uint8_t*)&header, sizeof(header));
    if (payload_len) {
        sdWrite(ubx_handle.serial, payload, payload_len);
    }
    sdWrite(ubx_handle.serial, crc, sizeof(crc));
}

static void request_message(uint8_t class_id, uint8_t msg_id)
{
    gps_debug("CFG", "request_message = %x %x", class_id, msg_id);
    send_message(class_id, msg_id, NULL, 0);
}

static void ubx_gps_spinner(void *ctx)
{
    (void)ctx;
    int16_t byte;
    uint8_t passThrough_len = 0;
    uint8_t passthrough_buffer[50];
    uint32_t passthrough_send = millis();
    
    while(true) {        
        byte = chnGetTimeout(ubx_handle.serial, 150);
        if (byte != MSG_TIMEOUT) {
            if(passThrough)
            {
                passthrough_buffer[passThrough_len] = byte;
                passThrough_len++;
            }
            if (gps_spin(&gps_handle, (uint8_t)byte) && ((ubx_handle.sercfg.speed == GPS_CFG_BAUD)))// && passThrough != 2) || (ubx_handle.sercfg.speed == 9600 && passThrough == 2))) {
            {
                if(ubx_handle.initialised == false) {
                    uavcan_send_debug_msg(LOG_LEVEL_INFO, "GPS", "initialised");
                }
                ubx_handle.initialised = true;
                //rtcm_parser.rtcm_frame_buf_len = 0;
            }
            //rtcm_parser_push_byte(&rtcm_parser, (uint8_t)byte);
        }
        
        if(passThrough_len >= 50 || passThrough_len > 0 && (millis() - passthrough_send) > 10) {
            rtcm_frame_cb(passThrough_len, passthrough_buffer, NULL);
            passThrough_len = 0;
            passthrough_send = millis();
        }
    }
}

static void ubx_gps_init_loop(struct worker_thread_timer_task_s *task)
{
    struct ubx_gps_handle_s *handle = (struct ubx_gps_handle_s *)task->ctx;
    static uint8_t try_cnt;
    if (!handle->initialised && (try_cnt % 5) == 0) {
        sdStop(handle->serial);
        handle->baudrate_index++;
        handle->baudrate_index %= sizeof(baudrates)/sizeof(baudrates[0]);
        handle->sercfg.speed = baudrates[handle->baudrate_index];
        sdStart(handle->serial, &handle->sercfg);
        uavcan_send_debug_msg(LOG_LEVEL_INFO, "GPS", "try baud %u", handle->sercfg.speed);
        send_init_blob(handle->serial);
    } else if (handle->initialised && !handle->configured) {
        if (passThrough == 2) {
            handle->configured = true;
            // train
            uint8_t train[2] = {0x55,0x55};
            // safeboot
            request_message(0x09,0x07);
            chThdSleepMilliseconds(500);
            
            sdStop(ubx_handle.serial);
            ubx_handle.sercfg.speed = 9600;
            sdStart(ubx_handle.serial, &ubx_handle.sercfg);
            // train
            sdWrite(ubx_handle.serial, train, 2);
            // init to new baud
            send_init_blob(handle->serial);
            chThdSleepMilliseconds(50);
            sdStop(ubx_handle.serial);
            ubx_handle.sercfg.speed = GPS_CFG_BAUD;
            sdStart(ubx_handle.serial, &ubx_handle.sercfg);
            request_message(0x0a, 0x04);
        } else {
            ubx_gps_configure_msgs();
        }
    }
    try_cnt++;

    worker_thread_timer_task_reschedule(&WT, &init_task, LL_MS2ST(100));
}

//MSG Configure
static void ubx_gps_configure_msgs()
{
    struct ubx_cfg_msg1_getset_s cfg_msg1;
    switch(ubx_handle.cfg_step) {
        case STEP_CFG_RATE: { //CFG_RATE
            if (ubx_handle.do_cfg) {
                struct ubx_cfg_rate_getset_s cfg_rate = {};
                cfg_rate.measRate = GNSS_MEAS_RATE;
                cfg_rate.navRate = 1;
                cfg_rate.timeRef = 0;
                send_message(UBX_CFG_RATE_CLASS_ID, UBX_CFG_RATE_MSG_ID, (uint8_t*)&cfg_rate, sizeof(cfg_rate));
                ubx_handle.do_cfg = false;
            } else {
                request_message(UBX_CFG_RATE_CLASS_ID, UBX_CFG_RATE_MSG_ID);
            }
            break;
        }
        case STEP_CFG_NAV5: {
            if (ubx_handle.do_cfg) {
                struct ubx_cfg_nav5_getset_s cfg_nav5 = {};

                cfg_nav5.mask = 1; // apply only dynamic model settings
                cfg_nav5.dynModel = nav5_dynModel;

                send_message(UBX_CFG_NAV5_CLASS_ID, UBX_CFG_NAV5_MSG_ID, (uint8_t*)&cfg_nav5, sizeof(cfg_nav5));
                ubx_handle.do_cfg = false;
            } else {
                request_message(UBX_CFG_NAV5_CLASS_ID, UBX_CFG_NAV5_MSG_ID);
            }
            break;
        }
        case STEP_CFG_MSG: {
            if (ubx_handle.cfg_msg_index >= ubx_handle.total_msg_cfgs) {
                ubx_handle.cfg_step++;
                break;
            }
            if (ubx_handle.do_cfg) {
                cfg_msg1.msgClass = ubx_cfg_list[ubx_handle.cfg_msg_index].class_id;
                cfg_msg1.msgID = ubx_cfg_list[ubx_handle.cfg_msg_index].msg_id;
                cfg_msg1.rate = ubx_cfg_list[ubx_handle.cfg_msg_index].rate;
                send_message(UBX_CFG_MSG1_CLASS_ID, UBX_CFG_MSG1_MSG_ID, (uint8_t*)&cfg_msg1, sizeof(cfg_msg1));
                ubx_handle.do_cfg = false;
            } else { //request current config
                struct ubx_cfg_msg_pollrequest_s cfg_msg;
                cfg_msg.msgClass = ubx_cfg_list[ubx_handle.cfg_msg_index].class_id;
                cfg_msg.msgID = ubx_cfg_list[ubx_handle.cfg_msg_index].msg_id;
                send_message(UBX_CFG_MSG_CLASS_ID, UBX_CFG_MSG_MSG_ID, (uint8_t*)&cfg_msg, sizeof(cfg_msg));
                
                ubx_handle.attempt++;
                
                if(ubx_handle.attempt > 3) {
                    ubx_handle.cfg_msg_index++;
                    ubx_handle.attempt = 0;
                }
            }
            break;
        }
        case STEP_CFG_DGNSS: {
            if (ubx_handle.do_cfg) {
                uavcan_send_debug_msg(LOG_LEVEL_INFO, "GPS", "setup ubx_cfg_dgnss_getset_s");
                struct ubx_cfg_dgnss_getset_s cfg_dgnss = {};

                cfg_dgnss.dgnssMode = dgnssMode;

                send_message(UBX_CFG_DGNSS_CLASS_ID, UBX_CFG_DGNSS_MSG_ID, (uint8_t*)&cfg_dgnss, sizeof(cfg_dgnss));
                ubx_handle.do_cfg = false;
                ubx_handle.cfg_step++;
            } else {
                request_message(UBX_CFG_DGNSS_CLASS_ID, UBX_CFG_DGNSS_MSG_ID);
                
                ubx_handle.attempt++;
                
                if(ubx_handle.attempt > 3) {
                    ubx_handle.cfg_step++;
                    ubx_handle.attempt = 0;
                }
            }
            break;
        }
        case STEP_CFG_GNSS: {
            if (ubx_handle.do_cfg) {
                request_message(UBX_CFG_GNSS_CLASS_ID, UBX_CFG_GNSS_MSG_ID);
                ubx_handle.do_cfg = false;
            } else {
                request_message(UBX_CFG_GNSS_CLASS_ID, UBX_CFG_GNSS_MSG_ID);
                
                ubx_handle.attempt++;
                
                if(ubx_handle.attempt > 3) {
                    ubx_handle.cfg_step++;
                    ubx_handle.attempt = 0;
                }
            }
            break;
        }
        case STEP_CFG_COMPLETE: {
            ubx_handle.configured = true;
            uavcan_send_debug_msg(UAVCAN_PROTOCOL_DEBUG_LOGLEVEL_INFO, "", "freemem %u", chCoreGetStatusX());
        }
        default:
            break;
    }

}

static void ubx_cfg_nav5_handler(size_t msg_size, const void* msg, void* ctx) {
    (void)msg_size;
    struct ubx_gps_handle_s* _handle = (struct ubx_gps_handle_s*)ctx;
    const struct gps_msg *parsed_msg = msg;

    struct ubx_cfg_nav5_getset_s *cfg_nav5 = ubx_parse_ubx_cfg_nav5_getset(parsed_msg->frame_buffer, parsed_msg->msg_len);

    gps_debug("CFG-NAV5", "dynmodel = %u", cfg_nav5->dynModel);

    if (_handle->cfg_step == STEP_CFG_NAV5 && cfg_nav5->dynModel == nav5_dynModel) {
        _handle->cfg_step++;
    } else {
        ubx_handle.do_cfg = true;
    }
}

static void ubx_cfg_dgnss_handler(size_t msg_size, const void* msg, void* ctx) {
    (void)msg_size;
    struct ubx_gps_handle_s* _handle = (struct ubx_gps_handle_s*)ctx;
    const struct gps_msg *parsed_msg = msg;

    struct ubx_cfg_dgnss_getset_s *cfg_dgnss = ubx_parse_ubx_cfg_dgnss_getset(parsed_msg->frame_buffer, parsed_msg->msg_len);

    gps_debug("CFG-DGNSS", "dgnssMode = %u", cfg_dgnss->dgnssMode);

    if (_handle->cfg_step == STEP_CFG_DGNSS && cfg_dgnss->dgnssMode == dgnssMode) {
        _handle->cfg_step++;
    } else {
        ubx_handle.do_cfg = true;
    }
}

static void ubx_cfg_gnss_handler(size_t msg_size, const void* msg, void* ctx) {
    (void)msg_size;
    struct ubx_gps_handle_s* _handle = (struct ubx_gps_handle_s*)ctx;
    const struct gps_msg *parsed_msg = msg;

    uint8_t num_repeat_blocks = 0;

    struct ubx_cfg_gnss_getset_rep_s *cfg_gnss_rep = ubx_parse_ubx_cfg_gnss_getset_rep(parsed_msg->frame_buffer, parsed_msg->msg_len, &num_repeat_blocks);
    
    if (cfg_gnss_rep != NULL) {
        gps_debug("CFG-GNSS", "num_repeat_blocks = %u", num_repeat_blocks);
        
        struct __attribute__((__packed__)) ubx_cfg_gnss_s {
            struct ubx_cfg_gnss_getset_s cfg_gnss;
            struct ubx_cfg_gnss_getset_rep_s cfg_gnss_rep[num_repeat_blocks];
        };

        struct ubx_cfg_gnss_s cfg_gnss;
        
        memset(&cfg_gnss, 0, sizeof(cfg_gnss));
        
        cfg_gnss.cfg_gnss.numConfigBlocks = num_repeat_blocks;
        cfg_gnss.cfg_gnss.numTrkChUse = 0xff;
        
        for(int a=0;a<num_repeat_blocks;a++) {
            int id = cfg_gnss_rep[a].gnssId;
            bool enabled = (cfg_gnss_rep[a].flags & 1) > 0;
            int bitmask = 1 << id;
            gps_debug("CFG-GNSS", " %u raw flags = %u  sz %u", a, cfg_gnss_rep[a].flags, sizeof(cfg_gnss));
            uavcan_send_debug_msg(LOG_LEVEL_INFO, "GPS", " %u raw flags = %u sz %u", cfg_gnss_rep[a].gnssId, cfg_gnss_rep[a].flags, sizeof(cfg_gnss));
            if ((gnssConfig & bitmask) > 0 && !enabled) {
                //cfg_gnss_rep[a].flags |= 1 << 0;
                uavcan_send_debug_msg(LOG_LEVEL_INFO, "CFG-GNSS", "enable = %u ", id);
                ubx_handle.do_cfg = true;
            } else if ((gnssConfig & bitmask) == 0 && enabled) {
                //cfg_gnss_rep[a].flags &= ~(1 << 0);
                uavcan_send_debug_msg(LOG_LEVEL_INFO, "CFG-GNSS", "disable = %u ", id);
                ubx_handle.do_cfg = true;
            }

            cfg_gnss.cfg_gnss_rep[a].gnssId = id;
            cfg_gnss.cfg_gnss_rep[a].resTrkCh = 4;
            cfg_gnss.cfg_gnss_rep[a].maxTrkCh = 12;
            if(id==5 || id == 1){ // sbas or qzss
                cfg_gnss.cfg_gnss_rep[a].resTrkCh = 0;
                cfg_gnss.cfg_gnss_rep[a].maxTrkCh = 3;
            }
            if(id==3){ // beidou
                cfg_gnss.cfg_gnss_rep[a].resTrkCh = 4;
                cfg_gnss.cfg_gnss_rep[a].maxTrkCh = 16;
            }
            cfg_gnss.cfg_gnss_rep[a].flags = (1<<16) + (1<<24);
            if ((gnssConfig & bitmask) > 0)
                cfg_gnss.cfg_gnss_rep[a].flags += 1;
        }

        if (_handle->cfg_step == STEP_CFG_GNSS && ubx_handle.do_cfg == false) {
            _handle->cfg_step++;
        } else {
            send_message(UBX_CFG_GNSS_CLASS_ID, UBX_CFG_GNSS_MSG_ID, (uint8_t*)&cfg_gnss, sizeof(cfg_gnss));
            ubx_handle.do_cfg = true;
        }
    }
}

//MSG Handlers
//NAV-SOL
static void ubx_nav_sol_handler(size_t msg_size, const void* msg, void* ctx)
{
    (void)msg_size;
    (void)ctx;
    //struct ubx_gps_handle_s* _handle = (struct ubx_gps_handle_s*)ctx;
    const struct gps_msg *parsed_msg = msg;
    struct ubx_nav_sol_periodicpolled_s *nav_sol = ubx_parse_ubx_nav_sol_periodicpolled(parsed_msg->frame_buffer, parsed_msg->msg_len);
    if (nav_sol != NULL) {
        gps_debug("NAV-SOL", "Time: %d Fix: %d SatUsed: %d posX: %ld posY: %ld posZ: %ld", nav_sol->iTOW, nav_sol->gpsFix, nav_sol->numSV, nav_sol->ecefX, nav_sol->ecefY, nav_sol->ecefZ);
    } else {
        gps_debug("NAV-SOL", "BAD MSG");
    }
}

//NAV-SVINFO
static void ubx_nav_svinfo_handler(size_t msg_size, const void* msg, void* ctx)
{
    (void)msg_size;
    (void)ctx;
    const struct gps_msg *parsed_msg = msg;
    struct ubx_nav_svinfo_periodicpolled_s *nav_svinfo = ubx_parse_ubx_nav_svinfo_periodicpolled(parsed_msg->frame_buffer, parsed_msg->msg_len);

    if (nav_svinfo != NULL) {
        gps_debug("NAV-SVINFO", "Time: %d NumSats: %d", nav_svinfo->iTOW, nav_svinfo->numCh);
    } else {
        gps_debug("NAV-SVINFO", "BAD MSG");
    }
}

//NAV-EOE
static void ubx_nav_eoe_handler(size_t msg_size, const void* msg, void* ctx)
{
    (void)msg_size;
    (void)ctx;
    struct ubx_gps_handle_s* _handle = (struct ubx_gps_handle_s*)ctx;
    const struct gps_msg *parsed_msg = msg;
    struct ubx_nav_eoe_periodic_s *nav_eoe = ubx_parse_ubx_nav_eoe_periodic(parsed_msg->frame_buffer, parsed_msg->msg_len);

    if (nav_eoe != NULL) {
        if (_handle->configured && nav_eoe->iTOW == _handle->dop_iTOW && _handle->dop_iTOW == _handle->pvt_iTOW) {
            uavcan_broadcast(0, &uavcan_equipment_gnss_Auxiliary_descriptor, CANARD_TRANSFER_PRIORITY_MEDIUM, &_handle->aux_msg);
        }
    } else {
        gps_debug("NAV-EOE", "BAD MSG");
    }
}

//NAV-DOP
static void ubx_nav_dop_handler(size_t msg_size, const void* msg, void* ctx)
{
    (void)msg_size;
    (void)ctx;
    struct ubx_gps_handle_s* _handle = (struct ubx_gps_handle_s*)ctx;
    const struct gps_msg *parsed_msg = msg;
    struct ubx_nav_dop_periodicpolled_s *nav_dop = ubx_parse_ubx_nav_dop_periodicpolled(parsed_msg->frame_buffer, parsed_msg->msg_len);

    if (nav_dop != NULL) {
        _handle->dop_iTOW = nav_dop->iTOW;
        _handle->aux_msg.gdop = nav_dop->gDOP*0.01;
        _handle->aux_msg.pdop = nav_dop->pDOP*0.01;
        _handle->aux_msg.hdop = nav_dop->hDOP*0.01;
        _handle->aux_msg.vdop = nav_dop->vDOP*0.01;
        _handle->aux_msg.tdop = nav_dop->tDOP*0.01;
        _handle->aux_msg.ndop = nav_dop->nDOP*0.01;
        _handle->aux_msg.edop = nav_dop->eDOP*0.01;
    } else {
        gps_debug("NAV-DOP", "BAD MSG");
    }
}

//NAV-PVT
static void ubx_nav_pvt_handler(size_t msg_size, const void* msg, void* ctx)
{
    (void)msg_size;
    struct ubx_gps_handle_s* _handle = (struct ubx_gps_handle_s*)ctx;
    const struct gps_msg *parsed_msg = msg;
    struct ubx_nav_pvt_periodicpolled_s *nav_pvt = ubx_parse_ubx_nav_pvt_periodicpolled(parsed_msg->frame_buffer, parsed_msg->msg_len);

    if (nav_pvt != NULL) {
        gps_debug("NAV-PVT", "Time: %d lon: %d lat: %d height: %d", nav_pvt->iTOW, nav_pvt->lon, nav_pvt->lat, nav_pvt->height);
        if (_handle->configured) {
            struct uavcan_equipment_gnss_Fix2_s fix2;
            memset(&fix2, 0, sizeof(fix2));
            //Position
            fix2.latitude_deg_1e8 = (int64_t)nav_pvt->lat*10;
            fix2.longitude_deg_1e8 = (int64_t)nav_pvt->lon*10;
            fix2.height_ellipsoid_mm = nav_pvt->height;
            fix2.height_msl_mm = nav_pvt->hMSL;
            
            //Velocity
            fix2.ned_velocity[0] = nav_pvt->velN/1e3f;
            fix2.ned_velocity[1] = nav_pvt->velE/1e3f;
            fix2.ned_velocity[2] = nav_pvt->velD/1e3f;
            //Heading of motion
            //_handle->state.fix.heading_of_motion =
            //uncertainties
            //Position
            //uavcan_send_debug_msg(LOG_LEVEL_DEBUG, "NAV-PVT" ,"%d %d", nav_pvt->hAcc, nav_pvt->vAcc);
            fix2.covariance_len = 6;
            fix2.covariance[0] = SQ(((float)(nav_pvt->hAcc)/1e3f));
            fix2.covariance[1] = SQ(((float)(nav_pvt->hAcc)/1e3f));
            fix2.covariance[2] = SQ(((float)(nav_pvt->vAcc)/1e3f));
            //Velocity
            fix2.covariance[3] = SQ(((float)(nav_pvt->sAcc)/1e3f));
            fix2.covariance[4] = fix2.covariance[3];
            fix2.covariance[5] = fix2.covariance[3];
            //Fix Mode
            switch(nav_pvt->fixType) {
                case 2: //2D-fix
                    fix2.status = UAVCAN_EQUIPMENT_GNSS_FIX2_STATUS_2D_FIX;
                    break;
                case 3: //3D-Fix
                case 4: //GNSS + dead reckoning combined
                    fix2.status = UAVCAN_EQUIPMENT_GNSS_FIX2_STATUS_3D_FIX;
                    if (nav_pvt->flags & 0b00000010) {
                        fix2.mode = UAVCAN_EQUIPMENT_GNSS_FIX2_MODE_DGPS;
                    }
                    if (nav_pvt->flags & 0b01000000) {
                        fix2.mode = UAVCAN_EQUIPMENT_GNSS_FIX2_MODE_RTK;
                        fix2.sub_mode = UAVCAN_EQUIPMENT_GNSS_FIX2_SUB_MODE_RTK_FLOAT;
                    }
                    if (nav_pvt->flags & 0b10000000) {
                        fix2.mode = UAVCAN_EQUIPMENT_GNSS_FIX2_MODE_RTK;
                        fix2.sub_mode = UAVCAN_EQUIPMENT_GNSS_FIX2_SUB_MODE_RTK_FIXED;
                    }
                    break;
                case 5: //time only fix
                    fix2.status = UAVCAN_EQUIPMENT_GNSS_FIX2_STATUS_TIME_ONLY;
                    break;
                case 0: //no fix
                case 1: //dead reckoning only
                default:
                    fix2.status = UAVCAN_EQUIPMENT_GNSS_FIX2_STATUS_NO_FIX;
                    break;
            }

            //Misc
            fix2.sats_used = nav_pvt->numSV;
            fix2.pdop = nav_pvt->pDOP*0.01f;

            //Time
            if ((nav_pvt->valid & 0x01) && (nav_pvt->valid & 0x02)) { //Check if utc date and time valid
                fix2.gnss_timestamp.usec = (uint64_t)date_to_utc_stamp(nav_pvt->year, nav_pvt->month, nav_pvt->day, nav_pvt->hour, nav_pvt->min, nav_pvt->sec) * 1000000 + nav_pvt->nano/1000;

                fix2.gnss_time_standard = UAVCAN_EQUIPMENT_GNSS_FIX2_GNSS_TIME_STANDARD_UTC;
            } else {
                fix2.gnss_timestamp.usec = 0;
                fix2.gnss_time_standard = UAVCAN_EQUIPMENT_GNSS_FIX2_GNSS_TIME_STANDARD_NONE;
            }
            
            fix2.timestamp.usec = chVTGetSystemTimeX();

            //Publish Fix Packet over CAN
            uavcan_broadcast(0, &uavcan_equipment_gnss_Fix2_descriptor, CANARD_TRANSFER_PRIORITY_HIGH, &fix2);
        }

        // Aux message stuff
        _handle->pvt_iTOW = nav_pvt->iTOW;
        _handle->aux_msg.sats_visible = _handle->aux_msg.sats_used = nav_pvt->numSV;
    } else {
        gps_debug("NAV-POSLLH", "BAD MSG");
    }
}

//MON-VER
static void ubx_mon_ver_handler(size_t msg_size, const void* msg, void* ctx)
{
    (void)msg_size;
    struct ubx_gps_handle_s* _handle = (struct ubx_gps_handle_s*)ctx;
    (void)_handle;
    const struct gps_msg *parsed_msg = msg;
    struct ubx_mon_ver_polled_s *mon_ver = ubx_parse_ubx_mon_ver_polled(parsed_msg->frame_buffer, parsed_msg->msg_len);
    if (mon_ver != NULL) {

    } else {
        gps_debug("MON-VER", "BAD MSG");
    }
}

//MON-IO
static void ubx_mon_io_handler(size_t msg_size, const void* msg, void* ctx)
{
    (void)msg_size;
    struct ubx_gps_handle_s* _handle = (struct ubx_gps_handle_s*)ctx;
    (void)_handle;
    const struct gps_msg *parsed_msg = msg;
    uint8_t n_repeat;
    struct ubx_mon_io_periodicpolled_rep_s *mon_io = ubx_parse_ubx_mon_io_periodicpolled_rep(parsed_msg->frame_buffer, parsed_msg->msg_len, &n_repeat);
    if (mon_io != NULL) {
//         uavcan_send_debug_msg(LOG_LEVEL_DEBUG, "io", "%u %u %u %u %u", (unsigned)mon_io[1].rxBytes, (unsigned)mon_io[1].parityErrs, (unsigned)mon_io[1].framingErrs, (unsigned)mon_io[1].overrunErrs, (unsigned)mon_io[1].breakCond);
    } else {
        gps_debug("MON-VER", "BAD MSG");
    }
}

//MON-MSGPP
static void ubx_mon_msgpp_handler(size_t msg_size, const void* msg, void* ctx)
{
    (void)msg_size;
    struct ubx_gps_handle_s* _handle = (struct ubx_gps_handle_s*)ctx;
    (void)_handle;
    const struct gps_msg *parsed_msg = msg;
    struct ubx_mon_msgpp_periodicpolled_s *mon_msgpp = ubx_parse_ubx_mon_msgpp_periodicpolled(parsed_msg->frame_buffer, parsed_msg->msg_len);
    if (mon_msgpp != NULL) {
//         uavcan_send_debug_msg(LOG_LEVEL_DEBUG, "msgpp", "%u", (unsigned)mon_msgpp->skipped[1]);
//         uavcan_send_debug_msg(LOG_LEVEL_DEBUG, "cand", "%u", (unsigned)can_rx_frame_drops);
    } else {
        gps_debug("MON-VER", "BAD MSG");
    }
}

//MON-RXBUF
static void ubx_mon_rxbuf_handler(size_t msg_size, const void* msg, void* ctx)
{
    (void)msg_size;
    struct ubx_gps_handle_s* _handle = (struct ubx_gps_handle_s*)ctx;
    (void)_handle;
    const struct gps_msg *parsed_msg = msg;
    struct ubx_mon_rxbuf_periodicpolled_s *mon_rxbuf = ubx_parse_ubx_mon_rxbuf_periodicpolled(parsed_msg->frame_buffer, parsed_msg->msg_len);
    if (mon_rxbuf != NULL) {
//         uavcan_send_debug_msg(LOG_LEVEL_DEBUG, "rxbuf", "%u %u %u", (unsigned)mon_rxbuf->pending[1], (unsigned)mon_rxbuf->usage[1], (unsigned)mon_rxbuf->peakUsage[1]);
    } else {
        gps_debug("MON-VER", "BAD MSG");
    }
}

//ACK-ACK
static void ubx_ack_ack_handler(size_t msg_size, const void* msg, void* ctx)
{
    (void)msg_size;
    struct ubx_gps_handle_s* _handle = (struct ubx_gps_handle_s*)ctx;
    (void)_handle;
    const struct gps_msg *parsed_msg = msg;
    struct ubx_ack_ack_output_s *ack_ack = ubx_parse_ubx_ack_ack_output(parsed_msg->frame_buffer, parsed_msg->msg_len);
    if (ack_ack != NULL) {
        gps_debug("ACK-ACK", "CFG Ack %d %d", _handle->cfg_step, _handle->cfg_msg_index);
    } else {
        gps_debug("ACK-ACK", "BAD MSG");
    }
}

//RXM-RTCM
static void ubx_rxm_rtcm_handler(size_t msg_size, const void* msg, void* ctx)
{
    (void)msg_size;
    struct ubx_gps_handle_s* _handle = (struct ubx_gps_handle_s*)ctx;
    const struct gps_msg *parsed_msg = msg;
    struct ubx_rxm_rtcm_output_s *rxm_rtcm = ubx_parse_ubx_rxm_rtcm_output(parsed_msg->frame_buffer, parsed_msg->msg_len);
    if (rxm_rtcm != NULL) {
//         uavcan_send_debug_msg(LOG_LEVEL_DEBUG, "rtcm", "%u %u", (unsigned)rxm_rtcm->msgType, (unsigned)rxm_rtcm->flags);
    } else {
        gps_debug("RXM-RTCM", "BAD MSG");
    }
}

//CFG-MSG
static void ubx_cfg_msg_handler(size_t msg_size, const void* msg, void* ctx)
{
    (void)msg_size;
    struct ubx_gps_handle_s* _handle = (struct ubx_gps_handle_s*)ctx;
    const struct gps_msg *parsed_msg = msg;
    struct ubx_cfg_msg_getset_s *cfg_msg = ubx_parse_ubx_cfg_msg_getset(parsed_msg->frame_buffer, parsed_msg->msg_len);
    if (cfg_msg != NULL) {
        if (_handle->cfg_step == STEP_CFG_MSG && _handle->cfg_msg_index < _handle->total_msg_cfgs) {
            if (cfg_msg->msgClass == ubx_cfg_list[_handle->cfg_msg_index].class_id &&
                cfg_msg->msgID == ubx_cfg_list[_handle->cfg_msg_index].msg_id &&
                cfg_msg->rate[UBX_PORT_ID] == ubx_cfg_list[_handle->cfg_msg_index].rate) {
                gps_debug("CFG-MSG", "MSG CFG (%d/%d) for 0x%x 0x%x set", _handle->cfg_msg_index + 1, _handle->total_msg_cfgs, ubx_cfg_list[_handle->cfg_msg_index].class_id, ubx_cfg_list[_handle->cfg_msg_index].msg_id, _handle->total_msg_cfgs);
                _handle->cfg_msg_index++;
                _handle->do_cfg = false;
            } else {
                _handle->do_cfg = true;
                gps_debug("CFG-MSG", "MSG CFG for 0x%x 0x%x Not set Received 0x%x 0x%x %d", ubx_cfg_list[_handle->cfg_msg_index].class_id, ubx_cfg_list[_handle->cfg_msg_index].msg_id, cfg_msg->msgClass, cfg_msg->msgID, cfg_msg->rate[UBX_PORT_ID]);
            }
        } else {
            gps_debug("CFG-MSG", "WRONG CFG");
        }
    } else {
        gps_debug("CFG-MSG", "BAD MSG");
    }
}

//CFG-RATE
static void ubx_cfg_rate_handler(size_t msg_size, const void* msg, void* ctx)
{
    (void)msg_size;
    struct ubx_gps_handle_s* _handle = (struct ubx_gps_handle_s*)ctx;
    const struct gps_msg *parsed_msg = msg;
    struct ubx_cfg_rate_getset_s *cfg_rate = ubx_parse_ubx_cfg_rate_getset(parsed_msg->frame_buffer, parsed_msg->msg_len);
    if (cfg_rate != NULL) {
        if (cfg_rate->measRate == GNSS_MEAS_RATE && cfg_rate->navRate == 1 && cfg_rate->timeRef == 0) {
            if (_handle->cfg_step == STEP_CFG_RATE) {
                _handle->cfg_step++;
            }
            _handle->do_cfg = false;
            gps_debug("CFG-RATE", "CFG Rate Set");
        } else {
            gps_debug("CFG-RATE", "CFG Rate Not Set %d %d %d", cfg_rate->measRate, cfg_rate->navRate, cfg_rate->timeRef);
            _handle->do_cfg = true;
        }
    } else {
        gps_debug("CFG-RATE", "BAD MSG");
    }
}

static void gps_inject_handler(size_t msg_size, const void* buf, void* ctx)
{
    (void)msg_size;
    struct ubx_gps_handle_s *ubx_instance = (struct ubx_gps_handle_s*)ctx;
    const struct can_rx_frame_s* frame = buf;

    const uint32_t dtid = 1062;

    if (frame->content.RTR || !frame->content.IDE || frame->content.DLC < 2) {
        return;
    }

    if (((frame->content.EID>>7)&1) != 0) {
        return;
    }

    if (((frame->content.EID>>8)&0xffff) != dtid) {
        return;
    }

    uint8_t tail_byte = frame->content.data[frame->content.DLC-1];
    bool eot = (tail_byte>>6)&1;
    bool sot = (tail_byte>>7)&1;

    // if start of transfer and not end of transfer, this is the start of a multi frame transfer and we have to skip the crc
    uint8_t start_idx = (sot && !eot) ? 2 : 0;
    if (sot) {
        start_idx += 1;
    }
    sdWrite(ubx_instance->serial, &frame->content.data[start_idx], frame->content.DLC-1-start_idx);

//     // Loopback test code
//     struct can_tx_frame_s* tx_frame = can_allocate_tx_frames(can_get_instance(0), 1);
//
//     tx_frame->content = frame->content;
//     tx_frame->content.EID &= ~0x7F;
//     tx_frame->content.EID |= 7;
//     can_enqueue_tx_frames(can_get_instance(0), &tx_frame, TIME_INFINITE, NULL, CAN_FRAME_ORIGIN_LOCAL);
//
//     for (size_t i=start_idx; i < frame->content.DLC-1; i++) {
//         rtcm_parser_push_byte(&rtcm_parser, frame->content.data[i]);
//     }
}
