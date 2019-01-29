#include "ch.h"
#include "hal.h"
#include <modules/timing/timing.h>
#include <common/helpers.h>
#include <modules/driver_profiLED/profiLED.h>
#include <modules/worker_thread/worker_thread.h>
#include <modules/uavcan_debug/uavcan_debug.h>
#include <uavcan.equipment.indication.LightsCommand.h>
#include <main_i2c_slave.h>

static struct profiLED_instance_s profiled_instance;

#define WT lpwork_thread
WORKER_THREAD_DECLARE_EXTERN(WT)

static struct worker_thread_timer_task_s profiled_task;
struct worker_thread_listener_task_s led_command_task;
static void profiled_task_func(struct worker_thread_timer_task_s* task);
static void led_command_handler(size_t msg_size, const void* buf, void* ctx);

RUN_AFTER(INIT_END) {
    profiLED_init(&profiled_instance, 3, BOARD_PAL_LINE_SPI3_PROFILED_CS, true, 4);
    worker_thread_add_timer_task(&WT, &profiled_task, profiled_task_func, NULL, LL_MS2ST(10), true);
    struct pubsub_topic_s* led_command_topic = uavcan_get_message_topic(0, &uavcan_equipment_indication_LightsCommand_descriptor);
    worker_thread_add_listener_task(&WT, &led_command_task, led_command_topic, led_command_handler, NULL);
}

static void profiled_task_func(struct worker_thread_timer_task_s* task) {
    (void)task;
    if (i2c_slave_led_updated()) {
        for (uint8_t i=0; i<4; i++) {
            profiLED_set_color_hex(&profiled_instance, i, i2c_slave_retrieve_led_color_hex());
        }
    }
    profiLED_update(&profiled_instance);
}

static void led_command_handler(size_t msg_size, const void* buf, void* ctx)
{
    (void)msg_size;
    (void)ctx;
    const struct uavcan_deserialized_message_s* msg_wrapper = buf;
    const struct uavcan_equipment_indication_LightsCommand_s* msg = (const struct uavcan_equipment_indication_LightsCommand_s*)msg_wrapper->msg;
    if (msg->commands_len > 0) {
        if (msg->commands[0].light_id == 0) {
            for (uint8_t j = 0; j < 4; j++) {
                profiLED_set_color_rgb(&profiled_instance, j, ((uint32_t)(msg->commands[0].color.red))*8, ((uint32_t)(msg->commands[0].color.green))*4, ((uint32_t)(msg->commands[0].color.blue))*8);
            }
        }
    }
    profiLED_update(&profiled_instance);
}
