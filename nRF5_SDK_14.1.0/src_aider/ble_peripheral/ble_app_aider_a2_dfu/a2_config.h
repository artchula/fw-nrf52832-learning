#ifndef __A2_CONFIG_H__
#define __A2_CONFIG_H__

#define APP_FEATURE_NOT_SUPPORTED       BLE_GATT_STATUS_ATTERR_APP_BEGIN + 2        /**< Reply when unsupported features are requested. */
#define DEVICE_NAME                     "Aider A2"                                  /**< Name of device. Will be included in the advertising data. */
#define MANUFACTURER_NAME               "BAESLAB Co.,Ltd."                       /**< Manufacturer. Will be passed to Device Information Service. */
#define MODEL_NUMBER									  "AIDER A2"
#define HARDWARE_REVISION								"HW AIDER-A2-1.0"
#define FIRMWARE_REVISION								"26"
#define FIRMWARE_REVISION_INT						26
#define APP_ADV_INTERVAL_ALERT          160  //160 -> 100mS                   /**< The advertising interval (in units of 0.625 ms. This value corresponds to 25 ms). */
#define APP_ADV_INTERVAL                3200//15000                                       /**< The advertising interval (in units of 0.625 ms. This value corresponds to 187.5 ms). */
#define APP_ADV_TIMEOUT_IN_SECONDS      0                                           /**< The advertising timeout in units of seconds. */
#define APP_BLE_OBSERVER_PRIO           3                                           /**< Application's BLE observer priority. You shouldn't need to modify this value. */
#define APP_BLE_CONN_CFG_TAG            1                                           /**< A tag identifying the SoftDevice BLE configuration. */
#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(100, UNIT_1_25_MS)            /**< Minimum acceptable connection interval (0.1 seconds). */
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(200, UNIT_1_25_MS)            /**< Maximum acceptable connection interval (0.2 second). */
#define SLAVE_LATENCY                   0                                           /**< Slave latency. */
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(4000, UNIT_10_MS)             /**< Connection supervisory timeout (4 seconds). */
#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(5000)                       /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(30000)                      /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define DISPLAY_HOLD_SCROLLING_INTERVAL APP_TIMER_TICKS(300)                        /**< Hold display scrolling interval (ticks). */
#define IBEACON_CON_INTERVAL            APP_TIMER_TICKS(4000)  											/**< BLE Beacon interval (ticks). */ 
#define IBEACON_CON_INTERVAL_FAST       APP_TIMER_TICKS(100)  											/**< BLE Beacon interval (ticks). */ 
#define CANCEL_SOS_INTERVAL             APP_TIMER_TICKS(60000)  										/**< Cancel SOS interval (ticks). */ 
#define ACC_GYRO_INTERVAL             	APP_TIMER_TICKS(200)  										/**< Cancel SOS interval (ticks). */ 
#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                           /**< Number of attempts before giving up the connection parameter negotiation. */
#define SEC_PARAM_BOND                  1                                           /**< Perform bonding. */
#define SEC_PARAM_MITM                  0                                           /**< Man In The Middle protection not required. */
#define SEC_PARAM_LESC                  0                                           /**< LE Secure Connections not enabled. */
#define SEC_PARAM_KEYPRESS              0                                           /**< Keypress notifications not enabled. */
#define SEC_PARAM_IO_CAPABILITIES       BLE_GAP_IO_CAPS_NONE                        /**< No I/O capabilities. */
#define SEC_PARAM_OOB                   0                                           /**< Out Of Band data not available. */
#define SEC_PARAM_MIN_KEY_SIZE          7                                           /**< Minimum encryption key size. */
#define SEC_PARAM_MAX_KEY_SIZE          16                                          /**< Maximum encryption key size. */
#define DEAD_BEEF                       0xDEADBEEF                                  /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */
#define COMPARE_COUNTERTIME  					  (1UL)                                       /**< Get Compare event COMPARE_TIME seconds after the counter starts from 0. */
#define MAX30102_SAMPLING_DELTA_TIME 		10  //10mS
#define MIN_TIME_TO_ENTER_HR_WOKR				4
#define MAX_COUNTER_GO_HOME_PAGE        60
#define MAX_COUNTER_DISPLAY_OFF					20
#define MAX_COUNTER_DISPLAY_DIM			    20
#define MAX_COUNTER_DISPLAY_HR_RS  			7
#define MAX_COUNTER_DISPLAY_O2_RS  			3
#define MAX_HR_COUNTER_DISPLAY_OFF		  183
#define COUNTER_TIME_FOR_CHARGE					5400
#define PEDOMETER_PROCESS_PERIOED				5
#define LOW_BATTERY_SOC_THRESHOLD				10  // Battery less than 10%
#define MAX_POWER_LED_OF_SENSOR_HR			130
#define MIN_POWER_LED_OF_SENSOR_HR			100
#define MAX_RANGE_VALUE_OF_HR						170
#define MIN_RANGE_VALUE_OF_HR						45
#define INDEX_START_CAL_HR							50
#define DIV_POWER_LED_OF_SENSOR_HR			5
#define MAX_BUFFER_SENSOR_HR						800



#endif

