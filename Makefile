CSRC = $(shell find src -name "*.c")
INCDIR = ./include
USE_OPT = -Os -g

USE_PROCESS_STACKSIZE = 1152
USE_EXCEPTIONS_STACKSIZE = 256

MODULES_ENABLED = \
chibios_sys_init \
chibios_hal_init \
app_descriptor \
boot_msg \
timing \
system \
pubsub \
worker_thread \
can_driver_stm32 \
can \
can_autobaud \
flash \
param \
uavcan \
uavcan_debug \
uavcan_nodestatus_publisher \
uavcan_getnodeinfo_server \
uavcan_beginfirmwareupdate_server \
uavcan_allocatee \
uavcan_restart \
uavcan_param_interface \
freemem_check \
gps \
spi_device \
driver_ak09916 \
driver_icm20x48 \
driver_profiLED \
driver_ms5611 \
stack_measurement \
load_measurement \
pubsub_miss_measurement

MESSAGES_ENABLED = \
uavcan.protocol.debug.LogMessage \
uavcan.protocol.debug.KeyValue \
uavcan.equipment.gnss.Fix \
uavcan.equipment.gnss.Fix2 \
uavcan.equipment.gnss.Auxiliary \
uavcan.equipment.ahrs.MagneticFieldStrength \
uavcan.equipment.gnss.RTCMStream \
uavcan.equipment.gnss.ECEFPositionVelocity \
uavcan.protocol.param.GetSet \
uavcan.equipment.indication.LightsCommand \
uavcan.equipment.indication.SingleLightCommand \
uavcan.equipment.indication.RGB565 \
uavcan.equipment.air_data.StaticPressure \
uavcan.equipment.air_data.StaticTemperature

DSDL_NAMESPACE_DIRS += dsdl/com


UBX_MESSAGES_ENABLED = \
ACK-ACK \
ACK-NAK \
CFG-CFG \
CFG-RATE \
CFG-MSG1 \
CFG-MSG \
CFG-PRT \
CFG-GNSS \
CFG-SBAS \
CFG-NAV5 \
MON-HW \
MON-HW2 \
MON-VER \
MON-IO \
MON-MSGPP \
MON-RXBUF \
NAV-SOL \
NAV-SVINFO \
NAV-STATUS \
NAV-POSLLH \
NAV-VELNED \
NAV-DOP \
NAV-PVT \
NAV-EOE \
RXM-RAWX \
RXM-RTCM

include framework/include.mk
