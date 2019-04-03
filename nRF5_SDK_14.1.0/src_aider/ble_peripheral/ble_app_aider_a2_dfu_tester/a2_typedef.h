#ifndef __A2_TYPEDEF_H__
#define __A2_TYPEDEF_H__


#include "bmi160.h"

typedef enum{
		DISPLAY_WAIT_STATE,
		DISPLAY_COUNDOWN_STATE,
		DISPLAY_OFF_STATE,
	  TIME_PAGE,
		STEP_COUNT_PAGE,
		CALORIES_PAGE,
		HEART_RATE_PAGE,
		O2_PAGE,
		TEMPERATURE_PAGE,
		BATTERY_PAGE,
}display_page_t;


typedef struct{
		uint32_t unix_time;
		uint8_t status;
		uint8_t time_h;
		uint8_t time_m;
		uint8_t time_s;
		uint8_t date_d;
		uint8_t date_m;
		uint8_t date_y;
		uint8_t heart_rate;
		uint16_t temperature;
		uint16_t step_count;
		uint16_t calories;
		uint8_t  pulse_oximeter;
		uint16_t batter_value;
		struct bmi160_sensor_data accel;
			uint8_t usb_pin_status;
		uint8_t usb_charging_status;
}a2_info_t;

typedef struct{
		uint8_t oled;
		uint8_t acc;
		uint8_t flash;
		uint8_t max30102;
		uint8_t tmp007;
		uint8_t ecc;
		uint8_t vibrator;
		uint8_t chager_detect;
		uint8_t battery_read;
}a2_testing_t;


#endif


