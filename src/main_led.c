#include "ch.h"
#include "hal.h"
#include <modules/timing/timing.h>
#include <common/helpers.h>
#include <modules/driver_profiLED/profiLED.h>
#include <modules/worker_thread/worker_thread.h>
#include <modules/uavcan_debug/uavcan_debug.h>
#include <uavcan.equipment.indication.LightsCommand.h>
#include <main_i2c_slave.h>
#include <modules/param/param.h>

static struct profiLED_instance_s profiled_instance;
static uint32_t color_buf[4];

PARAM_DEFINE_UINT8_PARAM_STATIC(led_mode, "LED_MODE", 1, 0, 2)
PARAM_DEFINE_BOOL_PARAM_STATIC(led_strobe, "LED_STROBE", false)

#define WT lpwork_thread
WORKER_THREAD_DECLARE_EXTERN(WT)

static struct worker_thread_timer_task_s profiled_task;
struct worker_thread_listener_task_s led_command_task;
static void profiled_task_func(struct worker_thread_timer_task_s* task);
static void led_command_handler(size_t msg_size, const void* buf, void* ctx);

RUN_AFTER(INIT_END) {
    profiLED_init(&profiled_instance, 3, BOARD_PAL_LINE_SPI3_PROFILED_CS, true, 4);
    worker_thread_add_timer_task(&WT, &profiled_task, profiled_task_func, NULL, LL_MS2ST(1), true);
    struct pubsub_topic_s* led_command_topic = uavcan_get_message_topic(0, &uavcan_equipment_indication_LightsCommand_descriptor);
    worker_thread_add_listener_task(&WT, &led_command_task, led_command_topic, led_command_handler, NULL);
}

static void profiled_task_func(struct worker_thread_timer_task_s* task) {
    static uint32_t count;
    (void)task;

    switch(led_mode) {
        case 0:
            for (uint8_t i=0; i<4; i++) {
                color_buf[i] = 0;
            }
            break;
        case 1:
            if (i2c_slave_led_updated()) {
                for (uint8_t i=0; i<4; i++) {
                    color_buf[i] = i2c_slave_retrieve_led_color_hex();
                }
            }
            break;
        case 2:
            color_buf[0] = 0x00ff00;
            color_buf[1] = 0x00ff00;
            color_buf[2] = 0xff0000;
            color_buf[3] = 0xff0000;
            break;
    }

    for (uint8_t i=0; i<4; i++) {
        profiLED_set_color_hex(&profiled_instance, i, color_buf[i]);
    }

    if (led_strobe) {
        if (count/50 == 2 || count/50 == 5) {
            for (uint8_t i=0; i<4; i++) {
                profiLED_set_color_hex(&profiled_instance, i, 0xffffff);
            }
        } else if (count/50 <= 7) {
            for (uint8_t i=0; i<4; i++) {
                profiLED_set_color_hex(&profiled_instance, i, 0);
            }
        }
    }

    count = (count+1) % 2000;

    profiLED_update(&profiled_instance);
}

static void led_command_handler(size_t msg_size, const void* buf, void* ctx)
{
    (void)msg_size;
    (void)ctx;
    const struct uavcan_deserialized_message_s* msg_wrapper = buf;
    const struct uavcan_equipment_indication_LightsCommand_s* msg = (const struct uavcan_equipment_indication_LightsCommand_s*)msg_wrapper->msg;
    if (led_mode == 1 && msg->commands_len > 0 && msg->commands[0].light_id == 0) {
        for (uint8_t i = 0; i < 4; i++) {
            color_buf[i] = 0;
            color_buf[i] |= ((uint32_t)(msg->commands[0].color.red))*8 << 16;
            color_buf[i] |= ((uint32_t)(msg->commands[0].color.green))*4 << 8;
            color_buf[i] |= ((uint32_t)(msg->commands[0].color.blue))*8;
        }
    }
}
