#ifndef __A2_TYPEDEF_H__
#define __A2_TYPEDEF_H__


#include "bmi160.h"

typedef enum {
		NO_BT_PUSH,
		LEFT_BT_PUSH,
		RIGHT_BT_PUSH,
		BT_WAIT_UNPRESS,
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
		BT_CHECK_WAIT_KRY,
		ALERT_BT_CHECK_UNPRESS,
}a2_bt_state_t;

typedef enum{
		USB_CHECK_WAIT_PLUG,
		ALERT_USB_CHECK_UNPLUG,
}a2_usb_connect_state_t;

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
		DISPLAY_DIM_STATE,
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
		DISPLAY_O2_PAGE,
		DISPLAY_HEART_RATE_PAGE,
	  DISPLAY_TEMPERATURE_PAGE,
		DISPLAY_BATTERY_PAGE,
		DISPLAY_BATTERY_FULL,
		DISPLAY_COUNT_DOWN_ALERT_PAGE,
		DISPLAY_BT_SOS_PAGE,
}display_page_t;

typedef enum{
		DISPlAY_SHOW_SYMBOL_VALUE,
		DISPlAY_SHOW_HR_VALUE,
		DISPlAY_SHOW_HR_MES_VALUE,
		DISPlAY_DO_NOT_SHOW_HR_VALUE,
		DISPLAY_GRAPH_HR,
}display_hr_t;

typedef struct{
		uint32_t unix_time;
		uint8_t status;
		uint8_t time_h;
		uint8_t time_m;
		uint8_t time_s;
		uint8_t date_d;
		uint8_t date_m;
		uint16_t date_y;
		uint8_t heart_rate;
		uint16_t temperature;
		uint16_t step_count;
		double calories_in_d;
		uint16_t calories;
		double distance_in_d;
		uint32_t distance;
		double speed_d;
		uint16_t speed;
		uint8_t  pulse_oximeter;
		uint16_t batter_value;
		struct bmi160_sensor_data accel;
		uint8_t hand_gesture_status;
		uint8_t usb_pin_status;
	  uint8_t usb_pin_state;
		uint8_t usb_charging_status;
		uint8_t touch_pin_status;
		uint8_t touch_pin_left_status;
		uint8_t touch_pin_right_status;	
		uint8_t alert_bt_state;
		uint8_t left_bt_state;
		uint8_t right_bt_state;
		uint8_t free_fall_status;
		uint8_t emergency_status;
		uint8_t acc_motion_detect_flag;
		uint32_t rest_time;
		uint32_t walking_time;
		uint32_t jogging_time;
		uint32_t running_time;
		uint8_t current_activity;
		uint8_t broken_max30120_status;
		uint8_t broken_bmi160_status;
		uint8_t broken_flash_memory_status;
		uint8_t broken_ecc_status;
		uint8_t low_battery_detected;
}a2_info_t;


typedef enum{
		MAX30102,
		TMP007,
		BMI160
}sensor_id_t;

typedef enum{
		HR_INIT,
		HR_MES,
		HR_CAL,
		HR_DEINIT,
	  HR_ADJ_LED,
		HR_SENSOR_FAIL_HANDLER,
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

typedef enum{
	A2_POEWR_FULL_MODE,
	A2_POWER_SAVE_MODE,
	A2_ULTRA_POWER_SAVE_MODE
}a2_power_mode_cfg_t;

#define A2_REG_SIZE			200
//typedef uint8_t *a2_reg_t;
typedef struct{
		uint8_t *reg[A2_REG_SIZE];
		uint8_t r_w_ctl[A2_REG_SIZE];
}a2_reg_t;

typedef enum{
		LEFT_HAND = 1,
		RIGHT_HAND = 2
}a2_wearing_position_t;

typedef enum{
		MALE = 1,
		FEMALE = 2
}a2_gender_t;
	
typedef enum{
		REG_READ_ONLY,
		REG_READ_WRITE,
		REG_WRITE_ONLY,		
}reg_r_w_ctl_t;

typedef enum{
		REG_RES_SUCCESS        = 0x53,
		REG_RES_UNSUCCESS      = 0x55,
		REG_RES_PKG_LEN_FALL   = 0x56,
}reg_res_t;

typedef enum{
		STILL_LEFT = 0x01,
		FINAL_PKG = 0x02,	
}pkg_status_t;

typedef enum{
		HAND_GESTURE_UNKNOW,
		HAND_GESTURE_WATCH_TIME,
}hand_gesture_t;

typedef struct{
		uint32_t red_value;
		uint32_t ir_value;
		uint32_t ir_non_dc;
		uint32_t ir_filter;
}hr_pkg_t;



#endif


