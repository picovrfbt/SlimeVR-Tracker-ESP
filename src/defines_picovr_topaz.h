// PicoVR V3.1 trackers (Topaz, ESP32-C3-MINI-1), the same settings the web
// flasher writes for board BOARD_TOPAZESTROGEN: SDA 6, SCL 4, LED on pin 0
// (inverted), external battery sense on pin 3 with a 150k/150k divider.
// Used instead of defines.h when built with -DPICOVR_TOPAZ.
//
// Two IMU variants, picked by PlatformIO env:
//   picovr_topaz      LSM6DSV, no interrupt pin, rotation 0  (the user's trackers)
//   picovr_topaz_bno  BNO085, interrupt on pin 10, rotation 180
//
// NOTE on rotation: the flasher's board definition carries
// DEFAULT_IMU_ROTATION 90, but that is only what the IMU row is pre-filled
// with when you pick this board. The saved config overrides it per IMU, and
// the PicoVR trackers' config sets the LSM6DSV to 0.
#pragma once

#define BOARD BOARD_CUSTOM

#ifdef PICOVR_TOPAZ_BNO
#define IMU IMU_BNO085
#define IMU_ROTATION DEG_180
#define PIN_IMU_INT 10
#else
#define IMU IMU_LSM6DSV
#define IMU_ROTATION DEG_0
#define PIN_IMU_INT 255
#endif

// One IMU is fitted; the second is listed only because defines.h expects it.
#define SECOND_IMU IMU_BMI270
#define SECOND_IMU_ROTATION DEG_90
#define PIN_IMU_INT_2 255

#define PRIMARY_IMU_OPTIONAL false
#define SECONDARY_IMU_OPTIONAL true

#define MAX_IMU_COUNT 1

#define PIN_IMU_SDA 6
#define PIN_IMU_SCL 4

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
