#ifndef __A2_LOGGING_MNGR_H__
#define __A2_LOGGING_MNGR_H__


#include <stdint.h>
#include "basic_fifo.h"
#include "a2_typedef.h"




// Configuration Vesion 1.0
#define CONFIG_VERSION									15
#define PAGE_BEGIN											0
#define PAGE_END												4095
#define PAGE_SIZE												264
#define CONFIG_PAGE											0
#define DEFUALT_HIGHT										170
#define DEFUALT_WIEGHT	  							65
#define DEFUALT_LENGTH									7


//Page Query configuration
#define QUERY_PAGE_ADDRESS 							99

//Page Data for button alert log
#define BUTTON_ALERT_PAGE_START  				100
#define BUTTON_ALERT_PAGE_STOP   				198

//Page Data for free fall alert log
#define FREE_FALL_ALERT_PAGE_START  		200
#define FREE_FALL_ALERT_PAGE_STOP   		298

#define RAW_ACC_GYRO_FALL_PAGE_START  	301
#define RAW_ACC_GYRO_FALL_PAGE_STOP   	390

//Page Data for pedo log   1 pages == 10 Set  : 1 day use 1440 set: config use 7 day use  10080 set. So using page 1008 page
#define PEDOMETER_PAGE_START  					400
#define PEDOMETER_PAGE_STOP   					1398

//Page Data for max30102 log
#define MAX30102_PAGE_START  						1400
#define MAX30102_PAGE_STOP   						2398



// Configuration Vesion 2.0
/* Wait implementation in next time. */



//#define MAX_ACC_GYRO_DATA_LOG_TIME_KEEPPER   10
//#define MAX_ACC_GYRO_DATA_LOG_FREQUENCY			 20
#define MAX_BUFFER_FOR_NAME_SURNAME						 15
#define MAX_ACC_GYRO_DATA_LOG_SIZE             200  // MAX_ACC_GYRO_DATA_LOG_TIME_KEEPPER*MAX_ACC_GYRO_DATA_LOG_FREQUENCY


#define REG_COMMAND_READ				0x52
#define REG_COMMAND_WRITE				0x57

#define REG_ADDR_GENDER													0x0004
#define REG_ADDR_WEIGHT													0x0005
#define REG_ADDE_HEIGHT													0x0006
#define REG_ADDE_WEARING_POSITION								0x0007
#define REG_ADDR_NAME														0x0008
#define REG_ADDR_SURNAME												0x0017
#define REG_ADDE_WORKING_HOUR										0x0027
#define REG_ADDE_CHARING_HOUR										0x002B
#define REG_ADDE_ULTRA_DEEP_SLEEP_COUNTER				0x002F
#define REG_ADDE_HEART_RATE_INTERVAL						0x0033
#define REG_ADDE_TEMPERATURE_INTERVAL						0x0037
#define REG_ADDE_ACC_GYRO_STEP_COUNT_INTERVAL		0x003B
#define REG_ADDE_BATTERY_MONITORING_INTERVAL		0x003F
#define REG_ADDE_HEART_RATE_EN									0x0043
#define REG_ADDE_TEMPERATURE_EN									0x0044
#define REG_ADDE_STEP_COUNT_EN									0x0045
#define REG_ADDE_ACC_EN													0x0046
#define REG_ADDE_GYRO_EN												0x0047
#define REG_ADDE_BATTERY_EN											0x0048
#define REG_ADDE_SEM_GET_HEART_RATE							0x0049
#define REG_ADDE_SEM_MANUAL_RECEHCK_HR					0x004A
#define REG_ADDE_SEM_GET_TEMPERATURE						0x004B
#define REG_ADDE_SEM_GET_ACC_GYRO_STEP_COUNT		0x004C
#define REG_ADDE_SEM_GET_BATTERY_LEVEL					0x004D
#define REG_ADDE_A2_RUNNING_MODE								0x004E
#define REG_ADDE_NO_MOTION_ACC_THRESHOLD				0x004F
#define REG_ADDE_NO_MOTION_TIMER_THRESHOLD			0x0053
#define REG_ADDE_ALERT_STATUS										0x0080
#define REG_ADDE_ACC_X													0x0081
#define REG_ADDE_ACC_Y													0x0083	
#define REG_ADDE_ACC_Z													0x0085
#define REG_ADDE_STEP_COUNT											0x0087	
#define REG_ADDE_SPEED													0x0089
#define REG_ADDE_CALORIES												0x008B
#define REG_ADDE_DISTANCE												0x008D
#define REG_ADDE_REST_TIME											0x0090
#define REG_ADDE_WALKING_TIME										0x0094
#define REG_ADDE_JOGGING_TIME										0x0098
#define REG_ADDE_RUNNING_TIME										0x0096
#define REG_ADDE_CURRENT_ACTIVITY								0x009C	
#define REG_ADDE_HEART_RATE											0x00A0
#define REG_ADDE_SPO2														0x00A1
#define REG_ADDE_TEMPERATURE										0x00A2

#define BEGIN_ADDR_REG_CONFIG									  REG_ADDR_GENDER
#define END_ADDR_REG_CONFIG									    REG_ADDE_NO_MOTION_TIMER_THRESHOLD + 1  


#define BEGIN_ADDR_NORMAL_REG										REG_ADDR_GENDER
#define END_ADDR_NORMAL_REG											REG_ADDE_TEMPERATURE + 1
#define LAST_ADDR_REG														REG_ADDE_TEMPERATURE + 1 // 1 is a last address of tempereture (temprature is 16 bit register)


#define BEGIN_ADDR_SPECIAL_REG									0xC9
#define END_ADDR_SPECIAL_REG										0xFF

#define REG_SPC_FREE_FALL_RAW_DATA_LOG				  0xFF
#define REG_SPC_BUTTON_ALEERT_LOG								0xFE
#define REG_SPC_FALL_ALEERT_LOG									0xFD
#define REG_SPC_PEDO_LOG												0xFC
#define REG_SPC_HR_SPO2_TEMP_LOG								0xFB



/*****************************************************************************/
/* type definitions */
typedef int8_t (*flash_comm_fptr_t)(uint16_t page_addr, uint8_t *data, uint16_t len);
typedef int8_t (*flash_erase_page_fptr_t)(uint16_t page_addr);
typedef int8_t (*flash_erase_chip_fptr_t)(void);
typedef void (*flash_wakeup_fptr_t)(void);
typedef void (*flash_sleep_fptr_t)(void);

typedef void (*flash_delay_fptr_t)(uint32_t period);


typedef struct{		
		uint32_t unix_time;
		uint16_t len;
		uint16_t fall_number;
		uint16_t ax[MAX_ACC_GYRO_DATA_LOG_SIZE];
		uint16_t ay[MAX_ACC_GYRO_DATA_LOG_SIZE];
		uint16_t az[MAX_ACC_GYRO_DATA_LOG_SIZE];
		uint16_t gx[MAX_ACC_GYRO_DATA_LOG_SIZE];
		uint16_t gy[MAX_ACC_GYRO_DATA_LOG_SIZE];
		uint16_t gz[MAX_ACC_GYRO_DATA_LOG_SIZE];
}a2_raw_acc_gyro_fall_log_t;

typedef struct{
		uint32_t page_size;
		uint32_t pn_begin;
		uint32_t pn_end;
		flash_erase_page_fptr_t erase_page;
		flash_erase_chip_fptr_t erase_chip;
		flash_comm_fptr_t read;
		flash_comm_fptr_t write;
		flash_delay_fptr_t delay;	
		flash_wakeup_fptr_t flash_wakeup;
		flash_sleep_fptr_t flash_sleep;
	  uint8_t buf_button_alert_log[PAGE_SIZE];
	  uint8_t buf_free_fall_alert_log[PAGE_SIZE];
	  uint8_t buf_pedo_log[PAGE_SIZE];
	  uint8_t buf_hr_spo2_temp_log[PAGE_SIZE];
		a2_raw_acc_gyro_fall_log_t buf_raw_acc_gyro_fall_log;
}logging_dev_t;

typedef enum{
		NORMAL_MODE,
		RUNNING_MODE,
		POWER_SAVE_MODE,
		ULTRA_POWER_SAVE_MODE
}a2_runming_mode_t;

typedef struct{
		uint32_t heart_rate_interval;
		uint32_t temperature_interal;
		uint32_t acc_gyro_step_count_interval;
		uint32_t pedometer_interval;
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
		uint8_t sem_save_pedo_to_flash;
		uint8_t sem_get_battery_level;
		uint8_t sem_reset_sensor_data;
		a2_runming_mode_t power_mode;
		uint16_t no_motion_acc_threshold;
		uint32_t no_motion_timer_threshold;
}a2_sensor_config_t;

typedef struct{
		uint8_t gender;
		uint8_t weight;
		uint8_t height;
		uint8_t wearing_position;
		uint8_t name[MAX_BUFFER_FOR_NAME_SURNAME];
		uint8_t surname[MAX_BUFFER_FOR_NAME_SURNAME];
}a2_user_data_config_t;

typedef struct{
		uint32_t working_hour;
		uint32_t charging_hour;
		uint16_t ultra_deep_sleep_counter;
}a2_working_info_t;

typedef struct{
		uint8_t config_version;
		a2_user_data_config_t a2_user_data_config;
		a2_working_info_t a2_working_info;
		a2_sensor_config_t a2_sensor_config; 
		uint16_t crc16;
}a2_config_t;

typedef enum{
		BUTTON_CANCEL_NOT_DETECT,
		BUTTON_CANCEL_DETECT,
		BUTTON_CANCEL_TIMEOUT,
}a2_cancel_type_t;

typedef enum{
		LEVEL_SOS,
		LEVEL_1,
		LEVEL_2,
		LEVEL_3
}a2_alert_level_t;

typedef enum{
		BUTTON_IS_NOT_DETECT,
		BUTTON_IS_DETECTED,
}a2_cancel_detect_t;

typedef struct{
		uint32_t unix_time;
		uint32_t cancel_time;
		uint8_t level;
		uint8_t cancel_detect;
		uint8_t sum_number_push_button;
}a2_button_alert_log_t;

typedef struct{		
		uint32_t unix_time;
	  uint32_t cancel_time;
		uint8_t level;
		uint8_t cancel_detect;
		uint16_t fall_number;
}a2_fall_alert_log_t;

typedef struct{		
		uint32_t unix_time;
		int16_t ax;
		int16_t ay;
	  int16_t az;
		int16_t gx;
		int16_t gy;
		int16_t gz;
		uint16_t step_count;
		uint16_t cal;
		uint16_t distance;
}a2_raw_acc_gyro_log_t;

typedef struct{		
		uint32_t unix_time;
		uint16_t hr;
		uint16_t spo2;
		uint16_t temperature;
		uint8_t skin_humdiy;
		double population_variance;
}a2_raw_max30102_log_t;




typedef enum{
		BUTTON_ALERT_TYPE,
		FALL_ALERT_LOG_TYPE,
		PEDOMETER_TYPE,
		HR_SPO2_TEMPERATURE_TYPE,
		FALL_ALERT_RAW_DATA_TYPE,
}a2_data_type_t;


typedef enum{
		LOG_SUCCESS,
		LOG_UNSUCCESS
}log_error_t;


typedef struct{
		uint8_t data_type;
		uint16_t page_start;
		uint16_t page_stop;
		queue_member_t q_page;
		queue_member_t q_data;
}queue_field_t;

typedef struct{
		queue_field_t button_alert;
		queue_field_t fall_alert;
		queue_field_t hr_spo2_temperature;
		queue_field_t fall_alert_raw_data;
		queue_field_t pedometer; 
		uint16_t crc16;
}queue_type_t;

	
typedef enum{
		SIZE_OF_FIFO = 0x01,
		QUEUE_R_OF_PAGES,
		QUEUE_F_OF_PAGES,
		BEGIN_PAGE_ADDR,
		END_PAGE_ADDR,
		GET_DATA_FROM_PAGE_ADDR,
		GET_DATA_FROM_FIFO,
		POP_DATA_FROM_FIFO
}command_for_reg_spc_t;


void read_fifo_logging(void);
void write_fifo_logging(void);
void read_user_config(void);
void write_user_config(void);
void read_data_logging(void);
void write_data_logging(void);

uint8_t logging_queue_initial(logging_dev_t *dev, queue_type_t *p_queue);
uint8_t a2_check_config(logging_dev_t *dev, a2_config_t *config);
uint8_t a2_read_config(logging_dev_t *dev, a2_config_t *config);
uint8_t a2_write_config(logging_dev_t *dev,const a2_config_t *config);

uint8_t a2_write_defualt_config(a2_sensor_config_t *p_config);
uint8_t a2_read_sensor_config(a2_sensor_config_t *p_config);

//Log functin for read/write queue config;
uint8_t a2_write_queue_config(logging_dev_t *dev, queue_type_t *p_queue);
uint8_t a2_read_queue_config(logging_dev_t *dev, queue_type_t *p_queue);

//Log functin for button alert
uint8_t a2_button_alert_buf_is_full(queue_field_t *p_queue);
uint8_t a2_save_button_alert_to_buf(logging_dev_t *dev, queue_field_t *p_queue, const a2_button_alert_log_t *p_data_log);
uint8_t a2_write_button_alert_log(logging_dev_t *dev, queue_field_t *p_queue);
uint8_t a2_read_button_alert_log_by_queue(logging_dev_t *dev, queue_field_t *p_queue, a2_button_alert_log_t p_data_log[]);
uint8_t a2_pop_button_alert_log(logging_dev_t *dev, queue_field_t *p_queue);
uint8_t a2_read_button_alert_log_by_page(logging_dev_t *dev, uint16_t  page_num, a2_button_alert_log_t p_data_log[]);
uint8_t a2_read_button_alert_last_buffer(logging_dev_t *dev, queue_field_t *p_queue);
uint8_t a2_write_button_alert_last_buffer(logging_dev_t *dev, queue_field_t *p_queue);

//Log functin for free fall alert
uint8_t a2_write_free_fall_alert_log(logging_dev_t *dev, queue_field_t *p_queue, const a2_fall_alert_log_t *p_data_log);
uint8_t a2_read_free_fall_alert_log_by_queue(logging_dev_t *dev, queue_field_t *p_queue, a2_fall_alert_log_t p_data_log[]);
uint8_t a2_pop_free_fall_alert_log(logging_dev_t *dev, queue_field_t *p_queue);
uint8_t a2_read_free_fall_alert_log_by_page(logging_dev_t *dev, uint16_t  page_num, a2_fall_alert_log_t p_data_log[]);

//Log functin for raw data from bmi160 
uint8_t a2_pedo_buf_is_full(queue_field_t *p_queue);
uint8_t a2_save_pedo_meter_to_buf(logging_dev_t *dev, queue_field_t *p_queue, const a2_raw_acc_gyro_log_t *p_data_log);
uint8_t a2_write_pedo_meter_log(logging_dev_t *dev, queue_field_t *p_queue);
uint8_t a2_read_pedo_meter_log_by_queue(logging_dev_t *dev, queue_field_t *p_queue, a2_raw_acc_gyro_log_t p_data_log[]);
uint8_t a2_pop_pedo_meter_log(logging_dev_t *dev, queue_field_t *p_queue);
uint8_t a2_read_pedo_meter_log_by_page(logging_dev_t *dev, uint16_t  page_num, a2_raw_acc_gyro_log_t p_data_log[]);

//Log functin for raw data from max30102
uint8_t a2_hr_spo2_buf_is_full(queue_field_t *p_queue);
uint8_t a2_save_hr_spo2_to_buf(logging_dev_t *dev, queue_field_t *p_queue, const a2_raw_max30102_log_t *p_data_log);
uint8_t a2_write_hr_spo2_temp_log(logging_dev_t *dev, queue_field_t *p_queue);
uint8_t a2_read_hr_spo2_temp_log_by_queue(logging_dev_t *dev, queue_field_t *p_queue, a2_raw_max30102_log_t p_data_log[]);
uint8_t a2_pop_hr_spo2_temp_log(logging_dev_t *dev, queue_field_t *p_queue);
uint8_t a2_read_hr_spo2_temp_log_by_page(logging_dev_t *dev, uint16_t  page_num, a2_raw_max30102_log_t p_data_log[]);

//Log functin for raw data from free fall bmi160
uint8_t a2_write_raw_data_free_fall_log(logging_dev_t *dev, queue_field_t *p_queue, const a2_raw_acc_gyro_fall_log_t *p_data_log);
uint8_t a2_read_raw_data_free_fall_log_by_queue(logging_dev_t *dev, queue_field_t *p_queue, a2_raw_acc_gyro_fall_log_t *p_data_log);
uint8_t a2_pop_raw_data_free_fall_log(logging_dev_t *dev, queue_field_t *p_queue);

void a2_reg_intial(a2_reg_t *p_reg);
uint8_t a2_read_reg(a2_reg_t *p_reg, uint8_t reg_addr, uint16_t data_len, uint8_t p_data[]);
uint8_t a2_write_reg(a2_reg_t *p_reg, uint8_t reg_addr, uint16_t data_len, uint8_t p_data[]);


extern a2_info_t 		m_a2_info;
extern a2_config_t	m_a2_config;


//#define __TEST_LOGGING_MNGR__
#ifdef __TEST_LOGGING_MNGR__
void TEST_PRINT_SIZE_OF_STRUCT(void);
void TEST_READ_WRITE_CONFIG(void);
void TEST_READ_WRITE_FIFO_ADDRESS(logging_dev_t *dev, queue_type_t *p_queue_type);
void TEST_WRITE_AND_READ_ALL_FLASH(logging_dev_t *dev);
#endif


#endif
