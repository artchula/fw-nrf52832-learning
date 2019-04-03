#ifndef __A2_LOGGING_MNGR_H__
#define __A2_LOGGING_MNGR_H__


#include <stdint.h>
#include "basic_fifo.h"

// Configuration Vesion 1.0
#define CONFIG_VERSION		1
#define PAGE_BEGIN				0
#define PAGE_END					4095
#define PAGE_SIZE					256
#define CONFIG_PAGE				0
#define DEFUALT_HIGHT			170
#define DEFUALT_WIEGHT	  65
#define DEFUALT_LENGTH		7

//Page Data
#define BUTTON_ALERT_PAGE_START  100
#define BUTTON_ALERT_PAGE_STOP   110




// Configuration Vesion 2.0
/* Wait implementation in next time. */



//#define MAX_ACC_GYRO_DATA_LOG_TIME_KEEPPER   10
//#define MAX_ACC_GYRO_DATA_LOG_FREQUENCY			 20
#define MAX_BUFFER_FOR_NAME_SURNAME					15
#define MAX_ACC_GYRO_DATA_LOG_SIZE         200  // MAX_ACC_GYRO_DATA_LOG_TIME_KEEPPER*MAX_ACC_GYRO_DATA_LOG_FREQUENCY

/*****************************************************************************/
/* type definitions */
typedef int8_t (*flash_comm_fptr_t)(uint16_t page_addr, uint8_t *data, uint16_t len);
typedef int8_t (*flash_erase_page_fptr_t)(uint16_t page_addr);
typedef int8_t (*flash_erase_chip_fptr_t)(void);


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
	  uint8_t buf_button_alert_log[256];
	  uint8_t buf_free_fall_alert_log[256];
	  uint8_t buf_pedo_log[256];
	  uint8_t buf_hr_spo2_temp_log[256];
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
		uint16_t no_motion_acc_threshold;
		uint32_t no_motion_timer_threshold;
}a2_sensor_config_t;

typedef struct{
		uint8_t gender;
		uint8_t weight;
		uint8_t hight;
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
		LEVEL_0,
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
		uint8_t level;
		uint8_t cancel_detect;
		uint32_t cancel_time;
		uint16_t fall_number;
}a2_fall_alert_log_t;



typedef struct{		
		uint32_t unix_time;
		uint16_t ax;
		uint16_t ay;
		uint16_t az;
		uint16_t gx;
		uint16_t gy;
		uint16_t gz;
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
}a2_raw_max32102_log_t;

typedef enum{
		LEFT_HAND,
		RIGHT_HAND
}a2_wearing_position_t;

typedef enum{
		MALE,
		FEMALE
}a2_gender_t;


typedef enum{
		BUTTON_ALERT_TYPE,
		FALL_ALERT_LOG_TYPE,
		HR_SPO2_TEMPERATURE_TYPE,
		FALL_ALERT_RAW_DATA_TYPE,
		PEDOMETER_TYPE
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

	
void TEST_PRINT_SIZE_OF_STRUCT(void);
void TEST_READ_WRITE_CONFIG(void);
void TEST_READ_WRITE_FIFO_ADDRESS(logging_dev_t *dev, queue_type_t *p_queue_type);
	
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
uint8_t a2_write_button_alert_log(logging_dev_t *dev, queue_field_t *p_queue, const a2_button_alert_log_t *p_data_log);
uint8_t a2_read_button_alert_log_page_fifo(logging_dev_t *dev, queue_field_t *p_queue, a2_button_alert_log_t p_data_log[]);
uint8_t a2_pop_button_alert_log(logging_dev_t *dev, queue_field_t *p_queue);
uint8_t a2_read_button_alert_log_by_page(logging_dev_t *dev, uint16_t  page_num, a2_button_alert_log_t p_data_log[]);



//uint8_t a2_read_button_alert_log(logging_dev_t *dev, const query_field_t *p_queue, const a2_button_alert_log_t *p_data_log);





#endif
