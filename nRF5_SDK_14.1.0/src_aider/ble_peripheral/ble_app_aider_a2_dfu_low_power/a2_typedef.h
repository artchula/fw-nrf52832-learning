#ifndef __A2_TYPEDEF_H__
#define __A2_TYPEDEF_H__


#include "bmi160.h"

typedef enum {
		NO_BT_PUSH,
		LEFT_BT_PUSH,
		RIGHT_BT_PUSH,
		ALERT_BT_PUSH,
		LEFT_RIGHT_BT_PUSH,
		USB_CHARGER_DETECTED,
		ALERT_LEVEL_3,
		ALERT_LEVEL_2,
		ALERT_LEVEL_1,
		ALERT_LEVEL_0,
		ALERT_SOS_DETECT,
		ALERT_CANCEL_DETECT,
		FRRE_FALL_DETECT,
}a2_bt_type;

typedef enum{
		ALERT_BT_CHECK_WAIT_KRY,
		ALERT_BT_CHECK_AGNORE,
		ALERT_BT_CHECK_LEVEL_3,
		ALERT_BT_CHECK_LEVEL_2,
		ALERT_BT_CHECK_LEVEL_1,
		ALERT_BT_CHECK_LEVEL_0,
		ALERT_BT_CHECK_SOS,
		ALERT_BT_CHECK_UNPRESS_SOS,
		ALERT_BT_CHECK_CALCEL,
		ALERT_BT_CHECK_UNPRESS_CANCEL,
}a2_bt_alert_state_t;

typedef enum{
		BATTERY_LEVEL_0,
		BATTERY_LEVEL_1,
		BATTERY_LEVEL_2,
		BATTERY_LEVEL_3,
		BATTERY_LEVEL_4,
		BATTERY_FULL,
}battery_level_state;

typedef enum{
		USB_NOT_FOUND,
		USB_FOUND
}a2_usb_detected_t;

typedef enum{
	  USB_CHARGER_DO_NOT_CONNECT_STATE,
		USB_CHARGER_DETECTED_STATE,
		BATTERY_CHARGING_STATE,
		BATTERY_FULLY_STATE,
}a2_usb_state_t;

typedef enum{
		DISPLAY_WAIT_STATE,
		DISPLAY_ON_STATE,
		DISPLAY_SLIDE_HOLD_STATE,
		DISPLAY_COUNDOWN_STATE,
		DISPLAY_HEART_RATE_COUNTDOWN_STAE,
		DISPLAY_HEART_RATE_SHOW_RESULT_STAE,
		DISPLAY_O2_SHOW_RESULT_STAE,
	  DISPLAY_OFF_STATE,
	  DISPLAY_CHANGE_PAGE_STATE,
		DISPLAY_BT_ALERT_STATE,
		DISPLAY_BT_SOS_PAEG_STATE,
		DISPLAY_FREE_FALL_SOS_PAGE_STATE,
		HEAERT_RATE_TASK,
}display_control_t;

typedef enum{
		DISPLAY_DUMMY_PAGE,
	  DISPLAY_TIME_PAGE,
		DISPLAY_STEP_COUNT_PAGE,
		DISPLAY_DISTANCE_CAL_PAGE,
		DISPLAY_TEMPERATURE_PAGE,
		DISPLAY_O2_PAGE,
		DISPLAY_HEART_RATE_PAGE,
		DISPLAY_BATTERY_PAGE,
		DISPLAY_BATTERY_FULL,
		DISPLAY_COUNT_DOWN_ALERT_PAGE,
		DISPLAY_BT_SOS_PAGE,
}display_page_t;

typedef enum{
		DISPlAY_SHOW_SYMBOL_VALUE,
		DISPlAY_SHOW_HR_VALUE,
		DISPlAY_DO_NOT_SHOW_HR_VALUE,
}display_hr_t;

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
		uint32_t distance;
		uint8_t  pulse_oximeter;
		uint16_t batter_value;
		struct bmi160_sensor_data accel;
		uint8_t usb_pin_status;
		uint8_t usb_charging_status;
		uint8_t touch_pin_status;
		uint8_t alert_bt_state;
		uint8_t free_fall_status;
		uint8_t emergency_status;
}a2_info_t;


typedef enum{
		NORMAL_MODE,
		RUNNING_MODE,
		POWER_SAVE_MODE,
		ULTRA_POWER_SAVE_MODE
}a2_runming_mode_t;

typedef struct{
		uint16_t software_version;
		uint32_t heart_rate_interval;
		uint32_t temperature_interal;
		uint32_t acc_gyro_step_count_interval;
		uint32_t battery_monitor_interval;
		uint8_t heart_rate_en;
		uint8_t temperature_en;
		uint8_t step_count_en;
		uint8_t acc_en;
		uint8_t gyro_en;
		uint8_t battery_en;
		uint8_t sem_get_heart_rate;
		uint8_t sem_manual_recheck_hr;
		uint8_t sem_get_temperature;
		uint8_t sem_get_acc_gyro_step_count;
		uint8_t sem_get_battery_level;
		a2_runming_mode_t power_mode;
		uint16_t crc;
}a2_sensor_config;

typedef enum{
		MAX30102,
		TMP007,
		BMI160
}sensor_id_t;

typedef enum{
		HR_INIT,
		HR_MES,
		HR_DEINIT
}heart_rate_state_t;

typedef enum{
		HR_SUCCESS,
		HR_IN_PROGRESS,
		HR_ERROR
}heart_rate_res_t;

typedef enum
{
		BLE_WAIT_CONNECT,
    BLE_CONNECTING,
		BLE_CONNECTED,
    BLE_DISCONNECT_BY_USER,      
		BLE_DISCONNECTING_BY_TIMEOUT,		   
		BLE_DISCONNECTED_BY_TIMEOUT,
} ble_connection_stete_t;

typedef enum
{
	  BLE_AIDER_EVT_NORMAL = 0x0,
    BLE_AIDER_EVT_WARNING = 0x1,                            
    BLE_AIDER_EVT_BT_ALERT = 0x02,                            
    BLE_AIDER_EVT_FALL_ALERT = 0x03,
    BLE_AIDER_EVT_CANCEL = 0x04,
    BLE_AIDER_EVT_BATT_CHARGING = 0x05,
    BLE_AIDER_EVT_BATT_FULL = 0x06,
	  BLE_AIDER_EVT_BATT_DISCHARGED = 0x07,
} ble_aider_evt_type_t;

#endif


