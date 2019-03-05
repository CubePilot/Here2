#include <hal.h>
#include <modules/driver_ak09916/driver_ak09916.h>
#include <modules/worker_thread/worker_thread.h>
#include <modules/uavcan_debug/uavcan_debug.h>
#include <modules/uavcan/uavcan.h>
#include <uavcan.equipment.ahrs.MagneticFieldStrength.h>
#include <modules/timing/timing.h>

#define WT hpwork_thread
WORKER_THREAD_DECLARE_EXTERN(WT)

static struct ak09916_instance_s ak09916;
static struct icm20x48_instance_s icm20x48;
static struct uavcan_equipment_ahrs_MagneticFieldStrength_s mag;

static struct worker_thread_timer_task_s ak09916_task;
static void ak09916_task_func(struct worker_thread_timer_task_s* task);
bool ak09916_initialised;

RUN_AFTER(INIT_END) {
    for (uint8_t i = 0; i < 5; i++) {
        if (icm20x48_init(&icm20x48, 3, BOARD_PAL_LINE_SPI3_ICM_CS, ICM20x48_IMU_TYPE_ICM20948)) {
            if (ak09916_init(&ak09916, &icm20x48)) {
                ak09916_initialised = true;
                break;
            }
        }
        usleep(10000);
    }
    worker_thread_add_timer_task(&WT, &ak09916_task, ak09916_task_func, NULL, LL_MS2ST(1), true);
}

static void ak09916_task_func(struct worker_thread_timer_task_s* task) {
    (void)task;
    if (!ak09916_initialised) {
        if (icm20x48_init(&icm20x48, 3, BOARD_PAL_LINE_SPI3_ICM_CS, ICM20x48_IMU_TYPE_ICM20948)) {
            if (ak09916_init(&ak09916, &icm20x48)) {
                ak09916_initialised = true;
            }
        }
        usleep(10000);
    } else if (ak09916_update(&ak09916)) {
        mag.magnetic_field_ga[0] = -ak09916.meas.y/1000.0f;
        mag.magnetic_field_ga[1] = -ak09916.meas.x/1000.0f;
        mag.magnetic_field_ga[2] = -ak09916.meas.z/1000.0f;
        mag.magnetic_field_covariance_len = 0;
        uavcan_broadcast(0, &uavcan_equipment_ahrs_MagneticFieldStrength_descriptor, CANARD_TRANSFER_PRIORITY_HIGH, &mag);
    }
}
