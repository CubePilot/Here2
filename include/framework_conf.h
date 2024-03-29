#pragma once

//
// Configure worker threads
//

#define TIMING_WORKER_THREAD                            lpwork_thread
#define UAVCAN_NODESTATUS_PUBLISHER_WORKER_THREAD       lpwork_thread
#define CAN_AUTOBAUD_WORKER_THREAD                      lpwork_thread
#define UAVCAN_PARAM_INTERFACE_WORKER_THREAD            lpwork_thread
#define UAVCAN_GETNODEINFO_SERVER_WORKER_THREAD         lpwork_thread
#define UAVCAN_RESTART_WORKER_THREAD                    lpwork_thread
#define UAVCAN_BEGINFIRMWAREUPDATE_SERVER_WORKER_THREAD lpwork_thread
#define UAVCAN_ALLOCATEE_WORKER_THREAD                  lpwork_thread
#define PIN_CHANGE_PUBLISHER_WORKER_THREAD              lpwork_thread
#define STACK_MEASUREMENT_WORKER_THREAD                 lpwork_thread
#define LOAD_MEASUREMENT_WORKER_THREAD                  lpwork_thread
#define PUBSUB_MISS_MEASUREMENT_WORKER_THREAD           lpwork_thread

#define CAN_TRX_WORKER_THREAD                           can_thread
#define CAN_EXPIRE_WORKER_THREAD                        can_thread
#define UAVCAN_RX_WORKER_THREAD                         can_thread

#define MODULE_UAVCAN_DEBUG_ENABLED                     TRUE

//
// Configure topic groups
//

#define PUBSUB_DEFAULT_TOPIC_GROUP default_topic_group

//
// Misc configs
//

#define PARAM_MAX_NUM_PARAMS 10
#define REQUIRED_RAM_MARGIN_AFTER_INIT      256

//
// Configure debug checks
//

#define CH_DBG_SYSTEM_STATE_CHECK           FALSE
#define CH_DBG_ENABLE_CHECKS                FALSE
#define CH_DBG_ENABLE_ASSERTS               FALSE
#define CH_DBG_ENABLE_STACK_CHECK           FALSE
#define CH_DBG_FILL_THREADS                 TRUE
