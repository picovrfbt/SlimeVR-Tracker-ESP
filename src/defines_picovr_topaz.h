// PicoVR V3.1 trackers (Topaz, ESP32-C3-MINI-1) with one LSM6DSV, as the web
// flasher configures them (board BOARD_TOPAZESTROGEN there): SDA 6, SCL 4, no
// IMU interrupt, LED on pin 0 (inverted), external battery sense on pin 3 with a
// 150k/150k divider. Used instead of defines.h when built with -DPICOVR_TOPAZ
// (PlatformIO env picovr_topaz).
#pragma once

#define IMU IMU_LSM6DSV
#define SECOND_IMU IMU_BMI270
#define BOARD BOARD_CUSTOM
#define IMU_ROTATION DEG_0
#define SECOND_IMU_ROTATION DEG_90

#define PRIMARY_IMU_OPTIONAL false
#define SECONDARY_IMU_OPTIONAL true

#define MAX_IMU_COUNT 1

#define PIN_IMU_SDA 6
#define PIN_IMU_SCL 4
#define PIN_IMU_INT 255
#define PIN_IMU_INT_2 255

#define IMU_DESC_LIST            \
	IMU_DESC_ENTRY(              \
		IMU,                     \
		PRIMARY_IMU_ADDRESS_ONE, \
		IMU_ROTATION,            \
		PIN_IMU_SCL,             \
		PIN_IMU_SDA,             \
		PRIMARY_IMU_OPTIONAL,    \
		PIN_IMU_INT              \
	)

#define BATTERY_MONITOR BAT_EXTERNAL
#define PIN_BATTERY_LEVEL 3
#define BATTERY_SHIELD_RESISTANCE 0
#define BATTERY_SHIELD_R1 150
#define BATTERY_SHIELD_R2 150

#define LED_PIN 0
#define LED_INVERTED true
