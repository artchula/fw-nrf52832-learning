#ifndef __A2_DISPLAY_MNGR_H__
#define __A2_DISPLAY_MNGR_H__

#include "stdint.h"
#include "nrf_error.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "a2_typedef.h"
#include "nrf_drv_ssd1306.h"
#include "ble_date_time.h"



#define SECONDS_FROM_1970_TO_2000 						946684800
#define ASIA_BANKOK_TIME_ZONE     						25200

extern display_page_t													m_display_page;
extern display_control_t                			m_display_control;
extern a2_info_t 															m_a2_info;
extern a2_sensor_config												m_a2_sensor_config;

void display_mini_time(void);
void draw_battery(void);
void draw_bt_icon(void);
void display_time(void);
void display_step_count(void);
void display_heart_reate(uint8_t state_hr);
void display_o2(void);
void display_temperature(void);
void display_distanc_cal(void);
void display_battery_full(void);
static void a2_logic_task(void);
void start_scroll(uint8_t bt);
void stop_scroll(void);
void show_display(void);
uint16_t date2days(uint16_t y, uint8_t m, uint8_t d);
void unixtimestamp_to_date_time(uint32_t unixtimestamp, ble_date_time_t * date_time);
uint32_t date_time_to_unixtimestamp(ble_date_time_t date_time);
uint8_t dayOfTheWeek(a2_info_t info_t);

void display_countdown_alert(uint8_t display_number);
void display_bt_alert(uint8_t en_invert);
void display_fall_alert(uint8_t en_invert);
	
	

#endif

