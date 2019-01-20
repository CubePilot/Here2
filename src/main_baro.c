#include <hal.h>
#include <modules/driver_ms5611/driver_ms5611.h>
#include <modules/uavcan_debug/uavcan_debug.h>
#include <uavcan.equipment.air_data.StaticPressure.h>
#include <uavcan.equipment.air_data.StaticTemperature.h>
#include <modules/timing/timing.h>

#define WT hpwork_thread
WORKER_THREAD_DECLARE_EXTERN(WT)

static struct ms5611_instance_s ms5611;

static struct worker_thread_listener_task_s ms5611_listener_task;
static void ms5611_listener_task_func(size_t msg_size, const void* msg, void* ctx);

static struct pubsub_topic_s ms5611_topic;

RUN_AFTER(INIT_END) {
    pubsub_init_topic(&ms5611_topic, NULL);

    worker_thread_add_listener_task(&WT, &ms5611_listener_task, &ms5611_topic, ms5611_listener_task_func, NULL);

    ms5611_init(&ms5611, 3, BOARD_PAL_LINE_SPI3_MS5611_CS, &WT, &ms5611_topic);
}

static void ms5611_listener_task_func(size_t msg_size, const void* msg, void* ctx) {
    struct ms5611_sample_s* sample = (struct ms5611_sample_s*)msg;

    struct uavcan_equipment_air_data_StaticTemperature_s temp;
    struct uavcan_equipment_air_data_StaticPressure_s press;

    press.static_pressure = sample->pressure_pa;
    temp.static_temperature = sample->temperature_K;
    uavcan_broadcast(0, &uavcan_equipment_air_data_StaticPressure_descriptor, CANARD_TRANSFER_PRIORITY_HIGH, &press);
    uavcan_broadcast(0, &uavcan_equipment_air_data_StaticTemperature_descriptor, CANARD_TRANSFER_PRIORITY_HIGH, &temp);
}
