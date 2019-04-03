#ifndef BLE_BAESLAB_PROTOCOL_H
#define BLE_BAESLAB_PROTOCOL_H
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

#define MAXIMUN_SEQUENT 								 100


#define SIZE_OF_EMERGENCY  			    		(5)
#define SIZE_OF_ACTIVIY   							(16)
#define SIZE_OF_ACTIVIY_A   						(9)
#define SIZE_OF_ACTIVIY_B               (8)
#define SIZE_OF_AXES  									(7)
#define SIZE_OF_HEALTH_MEASUREMENT 	    (7)
#define SIZE_OF_WIRELESS_BUTTON 				(3)
#define SIZE_OF_SENSOR_TEMP 						(4)			
#define SIZE_OF_SENSOR_GAS 			    		(4)
//#define SIZE_OF_SCAN_DATA 				(25)

#define set_byte 		  		type.status_byte 
#define set_bit_status 			type.status_bit.status
#define set_bit_body_position 	type.status_bit.body_position 

#define d_all_data 			  		data.data_all
#define d_emergency  		  		data.data_emergency_s
#define d_activity_a  		  	data.data_activity_a_s
#define d_activity_b  		  	data.data_activity_b_s
#define d_accelerotion_axes   data.data_accelerotion_axes_s
#define d_health_measurement  data.data_health_measurement_s

enum{
	DATA_TYPE_NORMAL = 0x00,
	DATA_TYPE_ENCRYPTION = 0xFF
}data_types_ble_sicurity;

enum{
	DATA_TYPE_A1_V1 = 0,
	DATA_TYPE_A1,
	DATA_TYPE_A2,
	DATA_TYPE_WB, //Data of Wireless Button
	DATA_TYPE_SH  //Data of Sensor Hub
};

// enum{
	// DATA_TYPE_EMERGENCY = 0,
	// DATA_TYPE_ACTIVITY_A,
	// DATA_TYPE_ACTIVITY_B,
	// DATA_TYPE_AXES,
	// DATA_TYPE_HEALTH_MEASUREMENT,
	// // DATA_TYPE_WIRELESS_BUTTON,
	// // DATA_TYPE_SENSOR_HUB,
	// // DATA_TYPE_SENSOR_HUB_GAS,
// }data_types_ble_baeslab;

// enum{
	// DATA_EMERGENCY = 0,
	// DATA_ACTIVITY,
	// DATA_AXES,
	// DATA_HEALTH_MEASUREMENT
// }data_types_a2;


enum{
	DATA_TYPE_EMERGENCY = 0,
	DATA_TYPE_ACTIVITY_A,
	DATA_TYPE_ACTIVITY_B,
	DATA_TYPE_AXES,
	DATA_TYPE_HEALTH_MEASUREMENT,
}data_types_a1_a2;

enum{
	DATA_TEMP = 0,
	DATA_GAS,
	DATA_C02
}data_types_sensor_hub;

enum{
	STATUS_NORMAL = 0,
	STATUS_WARNING,
	STATUS_PUSH_ALERT,
	STATUS_FALL_ALERT,
	STATUS_CANCEL,
	STATUS_CHARGING,
	STATUS_CHARGED
}data_types_alert;

enum{
	UNKNOWN_ACTIVITY,
	REST_ACTIVITY,
	WALKING_ACTIVITY,
	JOGGING_ACTIVITY,
	RUNNING_ACTIVITY
}data_type_current_activity;

typedef struct{
	union{
		struct{
			uint8_t body_position  : 4;
			uint8_t status				 : 4;
		}status_bit;
		uint8_t status_byte;
	}type;
}status_type;


typedef struct{
	//uint8_t type;
	uint8_t battery_level;
	status_type tag_status;
	//uint8_t human_step_count[2];
}data_emergency;

typedef struct{
	//uint8_t type;
	uint8_t speed[2];
	uint8_t calories[2];
	uint8_t distance[2];
	uint8_t human_step_count[2];
  //uint8_t rest_time[2];
}data_activity_a;


typedef struct{
	//uint8_t type;
	uint8_t rest_time[2];
    uint8_t walking_time[2];
    uint8_t jogging_time[2];
	uint8_t running_time[2];
	uint8_t current_activity;
}data_activity_b;

typedef struct{
	//uint8_t type;
	uint8_t accelerotion_x[2];
	uint8_t accelerotion_y[2];
	uint8_t accelerotion_z[2];
}data_accelerotion_axes;

typedef struct{
	//uint8_t type;
	uint8_t temperature[2];
	uint8_t heart_rate[2];
	uint8_t pulse_oximeter[2];
}data_health_measurement;
	

//Data type of A1.
typedef struct{
	uint8_t type_s;
	uint8_t type_device;
	uint8_t type_data;
	uint8_t sequent;
	uint8_t fw_version;
	union{
		uint8_t data_all[12];
		data_emergency data_emergency_s;
		data_activity_a data_activity_a_s;
		data_activity_b data_activity_b_s;
		data_accelerotion_axes data_accelerotion_axes_s;
	}data;
}data_a1;


//Data encryption type of A1.
typedef struct{
	uint8_t type_s;
	uint8_t data[16];
}data_a1_encrypt;

//Data type of A2.
typedef struct{
	uint8_t type_s;
	uint8_t type_device;
	uint8_t type_data;
	uint8_t sequent;
	uint8_t fw_version;
	union{
		uint8_t data_all[12];
		data_emergency data_emergency_s;
		data_activity_a data_activity_a_s;
		data_activity_b data_activity_b_s;
		data_accelerotion_axes data_accelerotion_axes_s;
		data_health_measurement data_health_measurement_s;
	}data;
}data_a2;


//Data encryption type of A2.
typedef struct{
	uint8_t type_s;
	uint8_t data[16];
}data_a2_encrypt;

//Data type of wireless button.
typedef struct{
	uint8_t type_s; //Security
	uint8_t type_device; 
	uint8_t sequent;
	uint8_t fw_version;
	uint8_t battery_level;
	uint8_t button_status;
}data_wireless_button;


//Data encryption type of wireless button.
typedef struct{
	uint8_t type_s;
	uint8_t data[5];
}data_wireless_button_encryption;

//Data type of sensor hub.
typedef struct{
	uint8_t type_s;
	uint8_t type_device;
	uint8_t type_data;
	uint8_t sequent;
	uint8_t fw_version;
	uint8_t battery_level;
	uint8_t data[2];
}data_sensor_hub;


//Data encryption type of sensor hub.
typedef struct{
	uint8_t type_s;
	uint8_t data[7];
}data_sensor_hub_encryption;

//Special packet data
//typedef struct{
//	uint8_t unknow[2];
//	uint8_t speed[2];
//	uint8_t calories[2];
//	uint8_t distance[2];
//	uint8_t dont_use[2];
//	uint8_t rest_time[2];
//	uint8_t walking_time[2];
//  uint8_t jogging_time[2];
//	uint8_t running_time[2];
//	uint8_t accelerotion_x[2];
//	uint8_t accelerotion_y[2];
//	uint8_t accelerotion_z[2];
//}data_type_A1; 



//fix advertise data
//size 30 byte
typedef struct{
	uint8_t avd_len_1;
	uint8_t avd_type_1;
	uint8_t avd_playload_1;
	uint8_t avd_len_2;
	uint8_t avd_type_2;
	//playload 25 byte
	uint8_t avd_playload_2[4];
	uint8_t avd_playload_2_uuid[16];
	uint8_t avd_playload_2_major[2];
	uint8_t avd_playload_2_minor[2];
	uint8_t tx_power;
}adv_data_baeslab;

//fix scan data  
//size 31 byte
typedef struct{
	uint8_t scn_len_1;
	uint8_t scn_type_1;
	uint8_t scn_playload_1[2];
	uint8_t scn_len_2;
	uint8_t scn_type_2;
	uint8_t scn_playload_2[25];
}scn_data_baeslab;


#endif
