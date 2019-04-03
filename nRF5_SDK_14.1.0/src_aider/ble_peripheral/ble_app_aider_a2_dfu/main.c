/**
 * Copyright (c) 2014 - 2017, Nordic Semiconductor ASA
 * 
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 * 
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 * 
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 * 
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 * 
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 * 
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 */
/** @example examples/ble_peripheral/ble_app_buttonless_dfu
 *
 * @brief Secure DFU Buttonless Service Application main file.
 *
 * This file contains the source code for a sample application using the proprietary
 * Secure DFU Buttonless Service. This is a template application that can be modified
 * to your needs. To extend the functionality of this application, please find
 * locations where the comment "// YOUR_JOB:" is present and read the comments.
 */
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include <stdbool.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "fft.h"


#include "nrf_dfu_svci.h"
#include "nrf_svci_async_function.h"
#include "nrf_svci_async_handler.h"
#include "nordic_common.h"
#include "nrf.h"
#include "app_error.h"
#include "ble.h"
#include "ble_hci.h"
#include "ble_srv_common.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "nrf_sdh.h"
#include "nrf_sdh_soc.h"
#include "nrf_sdh_ble.h"
#include "app_timer.h"
#include "peer_manager.h"
#include "bsp_btn_ble.h"
#include "ble_hci.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_state.h"
#include "ble_dfu.h"
#include "ble_bas.h"
#include "ble_hrs.h"
#include "ble_dis.h"
#include "nrf_ble_gatt.h"
#include "fds.h"
#include "nrf_pwr_mgmt.h"
#include "nrf_drv_clock.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "nrf_delay.h"
#include "nrf_drv_rtc.h"
#include "nrf_drv_saadc.h"
#include "ble_dts.h"
#include "nrf_drv_gpiote.h"
#include <math.h>
#include "ble_nus.h"
#include "nrf_drv_wdt.h"
#include "nrf_drv_timer.h"

//Baeslab Include File
#include "a2_typedef.h"
#include "nrf_drv_spi_bmi_flash.h"
#include "bmi160.h"
#include "at45db081e.h"
#include "nrf_drv_ssd1306.h"
#include "MAX30102.h"
#include "nrf_drv_i2c_max_temp.h"
#include "a2_display_mngr.h"
#include "Font32.h"
#include "Font64.h"
#include "a2_icon.h"
#include "SpO2Calculator.h"
#include "heart_rate_cal.h"
#include "ble_baeslab_protocol.h"
#include "a2_logging_mngr.h"
#include "basic_fifo.h"
#include "a2_config.h"
#include "a2_nus_mngr.h"
#include "pedometer.h"
#include "fft.h"
#include "heartRate.h"
#include "heart_rate_cal.h"

//#define DFU_DISABLE	
#define DEPLOY_TIME											1536336660
#define A2_3V3_BYPASS_PIN								26
#define OLED_POWER_EN_PIN						   	3
#define OLED_SCL_PIN								   	6
#define OLED_SDA_PIN								   	7
#define NRF_BMI160_INT1_PIN							15
#define NRF_BMI160_INT2_PIN							16
#define NRF_BMI160_POWER_EN_PIN					17
#define NRF_SPI_SCK_PIN									18
#define NRF_SPI_MOSI_PIN								19
#define NRF_SPI_MISO_PIN								20
#define NRF_SPI_BMI160_CS_PIN						22
#define NRF_SPI_FLASH_CS_PIN						13
#define NRF_SPI_FLASH_RESET_PIN				  23
#define NRF_SPI_FLASH_WP_PIN						24	
#define USB_PIN 						         		4
#define BATT_FULL_PIN 			         		5
#define ALERT_BT_PIN										8
#define LEFT_BT_PIN											9
#define RIGHT_BT_PIN										10
#define VIBRATION_PIN										31
#define HEAET_RATE_EN_1V8_PIN						29
#define HEAET_RATE_INT_PIN							14
#define TMP007_INT_PIN									25
#define MAX_TMP_SCL_PIN									11						
#define MAX_TMP_SDA_PIN									12	
#define TIMER_CAPTURE_TIME 							2  //2mS
#define GPIO_ISR_DISABLE

/*@brief Global Variable.
*
*/ 
static a2_power_mode_cfg_t							m_a2_power_mode_cfg;
static nrf_drv_a2_spi_comm_t  					m_a2_spi_comm;
struct bmi160_dev 						          m_bmi160_dev;
struct bmi160_sensor_data 		          m_gyro;
at45dbxx_dev_t													m_at45dbxx_dev;
static nrf_drv_a2_i2c_0_comm_t 	        m_oled_comm;
static oled_ssd1306_dev_t 			        m_oled_ssd1306_dev;			
static nrf_drv_a2_max_tmp_comm_t				m_max_tmp_comm;
static max30102_dev_t										m_max30102_dev;
static nrf_saadc_value_t 								adc_buf[2];
display_page_t													m_display_page;
display_control_t                				m_display_control;
uint8_t																	m_sem_display_scrolling; 
uint8_t 																m_sem_hr_cal;
a2_info_t 															m_a2_info;
a2_config_t															m_a2_config;
volatile a2_sensor_config_t							m_a2_sensor_config;
static ble_connection_stete_t						m_status_ble;
static bool                             m_rr_interval_enabled = true; 
static uint8_t													m_sem_cancel_sos = 0;
static logging_dev_t    								m_logging_dev;
static queue_type_t											m_queue_type;
a2_reg_t															  m_a2_reg;
a2_button_alert_log_t 									m_a2_button_alert_log;
sync_nus_comm_t													m_sync_nus_comm;
pedometer_data_t                        m_pedometer_data;

uint32_t 																m_reset_reason;
uint8_t 																flag_timer2 = 0;
static uint32_t 												timer2_counter;


static uint8_t													m_flag_update_graph_hr = 0;
uint32_t 																m_raw_ir_hr_buf[MAX_BUFFER_SENSOR_HR]; 
uint32_t 																m_raw_red_hr_buf[MAX_BUFFER_SENSOR_HR]; 
float 																	m_non_dc_ir_hr_buf[MAX_BUFFER_SENSOR_HR]; 
float 																  m_non_dc_ir_spo2_buf[MAX_BUFFER_SENSOR_HR]; 
//float 																	m_non_dc_ir_hr_buf2[550]; 
int8_t 																	signal[MAX_BUFFER_SENSOR_HR];
uint32_t 																m_hr_sample_counter = 0;

BLE_DTS_C_DEF(m_dts);																							/**< Advertising module instance. */       
NRF_BLE_GATT_DEF(m_gatt);              														/**< GATT module instance. */	                                      
BLE_ADVERTISING_DEF(m_advertising);     
static uint16_t m_conn_handle = BLE_CONN_HANDLE_INVALID;          /**< Handle of the current connection. */
bool erase_bonds;


BLE_NUS_DEF(m_nus);  
BLE_HRS_DEF(m_hrs);                                                 /**< Heart rate service instance. */
BLE_BAS_DEF(m_bas);   
APP_TIMER_DEF(m_ibeacon_interval_timer_id); 
APP_TIMER_DEF(m_display_scrolling_timer_id); 
APP_TIMER_DEF(m_cancel_sos_timer_id);
APP_TIMER_DEF(m_acc_gyro_timer_id);
nrf_drv_wdt_channel_id m_channel_id;
const  nrf_drv_rtc_t rtc = NRF_DRV_RTC_INSTANCE(2); /**< Declaring an instance of nrf_drv_rtc for RTC0. */
const nrf_drv_timer_t TIMER_HR = NRF_DRV_TIMER_INSTANCE(2);


static ble_uuid_t m_adv_uuids[] =                                   /**< Universally unique service identifiers. */
{
    {BLE_UUID_HEART_RATE_SERVICE,           BLE_UUID_TYPE_BLE},
    {BLE_UUID_BATTERY_SERVICE,              BLE_UUID_TYPE_BLE},
    {BLE_UUID_DEVICE_INFORMATION_SERVICE,   BLE_UUID_TYPE_BLE}
};


/*@brief Function Prototype.
*
*/ 
static void advertising_start(bool erase_bonds);                  /**< Forward declaration of advertising start function */
static void adc_init(void);			 
static void adc_uninit(void);
void heart_rate_processing(void);


/**@brief Function for sent data to host
 *
 * @param[in]   p_data   Pointer of data.
 * @param[in]   len      length of data.
 *
 * @retval  0.
 */
uint8_t nus_sent_to_host(uint8_t *p_data, uint16_t len)
{
		ble_nus_string_send(&m_nus, p_data, &len);
		return 0;
}


/**@brief Handler for shutdown preparation.
 *
 * @details During shutdown procedures, this function will be called at a 1 second interval
 *          untill the function returns true. When the function returns true, it means that the
 *          app is ready to reset to DFU mode.
 *
 * @param[in]   event   Power manager event.
 *
 * @retval  True if shutdown is allowed by this power manager handler, otherwise false.
 */
static bool app_shutdown_handler(nrf_pwr_mgmt_evt_t event)
{
    switch (event)
    {
        case NRF_PWR_MGMT_EVT_PREPARE_DFU:
            NRF_LOG_INFO("Power management wants to reset to DFU mode.");
						NRF_LOG_FLUSH();
            // YOUR_JOB: Get ready to reset into DFU mode
            //
            // If you aren't finished with any ongoing tasks, return "false" to
            // signal to the system that reset is impossible at this stage.
            //
            // Here is an example using a variable to delay resetting the device.
            //
            // if (!m_ready_for_reset)
            // {
            //      return false;
            // }
            // else
            //{
            //
            //    // Device ready to enter
            //    uint32_t err_code;
            //    err_code = sd_softdevice_disable();
            //    APP_ERROR_CHECK(err_code);
            //    err_code = app_timer_stop_all();
            //    APP_ERROR_CHECK(err_code);
            //}
            break;

        default:
            // YOUR_JOB: Implement any of the other events available from the power management module:
            //      -NRF_PWR_MGMT_EVT_PREPARE_SYSOFF
            //      -NRF_PWR_MGMT_EVT_PREPARE_WAKEUP
            //      -NRF_PWR_MGMT_EVT_PREPARE_RESET
            return true;
    }

    NRF_LOG_INFO("Power management allowed to reset to DFU mode.");
		NRF_LOG_FLUSH();	
    return true;
}

//lint -esym(528, m_app_shutdown_handler)
/**@brief Register application shutdown handler with priority 0.
 */
NRF_PWR_MGMT_HANDLER_REGISTER(app_shutdown_handler, 0);


// YOUR_JOB: Use UUIDs for service(s) used in your application.
//static ble_uuid_t m_adv_uuids[] = {{BLE_UUID_DEVICE_INFORMATION_SERVICE, BLE_UUID_TYPE_BLE}};


#ifndef DFU_DISABLE
// YOUR_JOB: Update this code if you want to do anything given a DFU event (optional).
/**@brief Function for handling dfu events from the Buttonless Secure DFU service
 *
 * @param[in]   event   Event from the Buttonless Secure DFU service.
 */
static void ble_dfu_evt_handler(ble_dfu_buttonless_evt_type_t event)
{
    switch (event)
    {
        case BLE_DFU_EVT_BOOTLOADER_ENTER_PREPARE:
            NRF_LOG_INFO("Device is preparing to enter bootloader mode.");
						NRF_LOG_FLUSH();
            // YOUR_JOB: Disconnect all bonded devices that currently are connected.
            //           This is required to receive a service changed indication
            //           on bootup after a successful (or aborted) Device Firmware Update.
            break;

        case BLE_DFU_EVT_BOOTLOADER_ENTER:
            // YOUR_JOB: Write app-specific unwritten data to FLASH, control finalization of this
            //           by delaying reset by reporting false in app_shutdown_handler
            NRF_LOG_INFO("Device will enter bootloader mode.");
						NRF_LOG_FLUSH();
            break;

        case BLE_DFU_EVT_BOOTLOADER_ENTER_FAILED:
            NRF_LOG_ERROR("Request to enter bootloader mode failed asynchroneously.");
						NRF_LOG_FLUSH();
            // YOUR_JOB: Take corrective measures to resolve the issue
            //           like calling APP_ERROR_CHECK to reset the device.
            break;

        case BLE_DFU_EVT_RESPONSE_SEND_ERROR:
            NRF_LOG_ERROR("Request to send a response to client failed.");
						NRF_LOG_FLUSH();
            // YOUR_JOB: Take corrective measures to resolve the issue
            //           like calling APP_ERROR_CHECK to reset the device.
            APP_ERROR_CHECK(false);
            break;

        default:
            NRF_LOG_ERROR("Unknown event from ble_dfu_buttonless.");
						NRF_LOG_FLUSH();
            break;
    }
}
#endif

/**@brief Callback function for asserts in the SoftDevice.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num   Line number of the failing ASSERT call.
 * @param[in] file_name  File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}


/**@brief Function for handling Peer Manager events.
 *
 * @param[in] p_evt  Peer Manager event.
 */
static void pm_evt_handler(pm_evt_t const * p_evt)
{
    ret_code_t err_code;

    switch (p_evt->evt_id)
    {
        case PM_EVT_BONDED_PEER_CONNECTED:
        {
            NRF_LOG_INFO("Connected to a previously bonded device.");
        } break;

        case PM_EVT_CONN_SEC_SUCCEEDED:
        {
            NRF_LOG_INFO("Connection secured: role: %d, conn_handle: 0x%x, procedure: %d.",
                         ble_conn_state_role(p_evt->conn_handle),
                         p_evt->conn_handle,
                         p_evt->params.conn_sec_succeeded.procedure);
        } break;

        case PM_EVT_CONN_SEC_FAILED:
        {
            /* Often, when securing fails, it shouldn't be restarted, for security reasons.
             * Other times, it can be restarted directly.
             * Sometimes it can be restarted, but only after changing some Security Parameters.
             * Sometimes, it cannot be restarted until the link is disconnected and reconnected.
             * Sometimes it is impossible, to secure the link, or the peer device does not support it.
             * How to handle this error is highly application dependent. */
        } break;

        case PM_EVT_CONN_SEC_CONFIG_REQ:
        {
            // Reject pairing request from an already bonded peer.
            pm_conn_sec_config_t conn_sec_config = {.allow_repairing = false};
            pm_conn_sec_config_reply(p_evt->conn_handle, &conn_sec_config);
        } break;

        case PM_EVT_STORAGE_FULL:
        {
            // Run garbage collection on the flash.
            err_code = fds_gc();
            if (err_code == FDS_ERR_BUSY || err_code == FDS_ERR_NO_SPACE_IN_QUEUES)
            {
                // Retry.
            }
            else
            {
                APP_ERROR_CHECK(err_code);
            }
        } break;

        case PM_EVT_PEERS_DELETE_SUCCEEDED:
        {
            advertising_start(false);
        } break;

        case PM_EVT_LOCAL_DB_CACHE_APPLY_FAILED:
        {
            // The local database has likely changed, send service changed indications.
            pm_local_database_has_changed();
        } break;

        case PM_EVT_PEER_DATA_UPDATE_FAILED:
        {
            // Assert.
            APP_ERROR_CHECK(p_evt->params.peer_data_update_failed.error);
        } break;

        case PM_EVT_PEER_DELETE_FAILED:
        {
            // Assert.
            APP_ERROR_CHECK(p_evt->params.peer_delete_failed.error);
        } break;

        case PM_EVT_PEERS_DELETE_FAILED:
        {
            // Assert.
            APP_ERROR_CHECK(p_evt->params.peers_delete_failed_evt.error);
        } break;

        case PM_EVT_ERROR_UNEXPECTED:
        {
            // Assert.
            APP_ERROR_CHECK(p_evt->params.error_unexpected.error);
        } break;

        case PM_EVT_CONN_SEC_START:
        case PM_EVT_PEER_DATA_UPDATE_SUCCEEDED:
        case PM_EVT_PEER_DELETE_SUCCEEDED:
        case PM_EVT_LOCAL_DB_CACHE_APPLIED:
        case PM_EVT_SERVICE_CHANGED_IND_SENT:
        case PM_EVT_SERVICE_CHANGED_IND_CONFIRMED:
        default:
            break;
    }
}

/**@brief Function for handling the Heart rate measurement timer timeout.
 *
 * @details This function will be called each time the heart rate measurement timer expires.
 *          It will exclude RR Interval data from every third measurement.
 *
 * @param[in] p_context  Pointer used for passing some arbitrary information (context) from the
 *                       app_start_timer() call to the timeout handler.
 */
//static void heart_rate_meas_timeout_handler(void * p_context)
static void heart_rate_meas_timeout_handler(void)
{
    static uint32_t cnt = 0;
    ret_code_t      err_code;
    uint16_t        heart_rate;

    //UNUSED_PARAMETER(p_context);
		NRF_LOG_INFO("^");

    heart_rate = m_a2_info.heart_rate;

    cnt++;
    err_code = ble_hrs_heart_rate_measurement_send(&m_hrs, heart_rate);
    if ((err_code != NRF_SUCCESS) &&
        (err_code != NRF_ERROR_INVALID_STATE) &&
        (err_code != NRF_ERROR_RESOURCES) &&
        (err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING)
       )
    {
        APP_ERROR_HANDLER(err_code);
    }

}


/**@brief Function for handling the RR interval timer timeout.
 *
 * @details This function will be called each time the RR interval timer expires.
 *
 * @param[in] p_context  Pointer used for passing some arbitrary information (context) from the
 *                       app_start_timer() call to the timeout handler.
 */
//static void rr_interval_timeout_handler(void * p_context)
static void rr_interval_timeout_handler()
{
    //UNUSED_PARAMETER(p_context);

    if (m_rr_interval_enabled)
    {
        uint16_t rr_interval;

        rr_interval = 110;//(uint16_t)sensorsim_measure(&m_rr_interval_sim_state,
                         //                         &m_rr_interval_sim_cfg);
        ble_hrs_rr_interval_add(&m_hrs, rr_interval);
        rr_interval = 110;//(uint16_t)sensorsim_measure(&m_rr_interval_sim_state,
                          //                        &m_rr_interval_sim_cfg);
        ble_hrs_rr_interval_add(&m_hrs, rr_interval);
        rr_interval = 120;//(uint16_t)sensorsim_measure(&m_rr_interval_sim_state,
                          //                        &m_rr_interval_sim_cfg);
        ble_hrs_rr_interval_add(&m_hrs, rr_interval);
        rr_interval = 130;//(uint16_t)sensorsim_measure(&m_rr_interval_sim_state,
                           //                       &m_rr_interval_sim_cfg);
        ble_hrs_rr_interval_add(&m_hrs, rr_interval);
        rr_interval = 140;//(uint16_t)sensorsim_measure(&m_rr_interval_sim_state,
                          //                        &m_rr_interval_sim_cfg);
        ble_hrs_rr_interval_add(&m_hrs, rr_interval);
        rr_interval = 150;//(uint16_t)sensorsim_measure(&m_rr_interval_sim_state,
                          //                        &m_rr_interval_sim_cfg);
        ble_hrs_rr_interval_add(&m_hrs, rr_interval);
    }
}


/**@brief Function for the display scrolling timout handler.
 *
 * @details This function will be called for all display scrolling timeout events which are passed to
 *          the application.
 */
static void display_scrolling_timeout_handler(void * p_context)
{
		NRF_LOG_INFO("Display Scrolling timeout");
	  m_sem_display_scrolling = 1;
}


/**@brief Function for handling the iBeacon interval timer timeout.
 *
 * @details This function will be called each time the Button Interval timer expires.
 *
 * @param[in] p_context  Pointer used for passing some arbitrary information (context) from the
 *                       app_start_timer() call to the timeout handler.
 */
static void ibeacon_interval_timeout_handler(void * p_context)
{
		static uint8_t state = DATA_TYPE_EMERGENCY;
		static uint8_t sequent_emergency = 1,
					 sequent_activity_A        = 1,
					 sequent_activity_B        = 1,
					 sequent_axes              = 1,
					 sequent_health            = 1;
		
		adv_data_baeslab m_adv_data_baeslab;
		scn_data_baeslab emergency_data;
		data_a2 data_a2_emergency;
		
		//data_type_emergency m_data_type_emergency;
		status_type m_status_type;
		scn_data_baeslab activityA_data;
		data_a2 data_a2_activityA;
		
		//data_type_activity m_data_type_activity;
		scn_data_baeslab activityB_data;
		data_a2 data_a2_activityB;
		scn_data_baeslab axes_data;
		data_a2 data_a2_axes;
		scn_data_baeslab health_data;
		data_a2 data_a2_health;
		
		//data_type_accelerotion_axes m_data_type_accelerotion_axes;
		uint8_t avd_playload_2_data[4] = {0x4C, 0x00, 0x02, 0x15};
		uint8_t avd_playload_2_uuid_data[16] = {0x4A, 0x03, 0x03, 0x99,/**/ 0x00,/**/ 0x00,/**/ 0x11, 0xA0,/**/ 0x01, 0x06,/**/ 0x00, 0x42, 0x41, 0x45, 0x53, 0x00};
		uint8_t avd_playload_2_major_data[2] = {0x00, 0x01};
		uint8_t avd_playload_2_minor_data[2] = {0x00, 0x02};

		m_adv_data_baeslab.avd_len_1 = 0x02;
		m_adv_data_baeslab.avd_type_1 = 0x01;
		m_adv_data_baeslab.avd_playload_1 = 0x06;
		m_adv_data_baeslab.avd_len_2 = 0x1A;
		m_adv_data_baeslab.avd_type_2 = 0xFF;
		memcpy(m_adv_data_baeslab.avd_playload_2, avd_playload_2_data, sizeof(avd_playload_2_uuid_data));
		memcpy(m_adv_data_baeslab.avd_playload_2_uuid, avd_playload_2_uuid_data, sizeof(avd_playload_2_uuid_data));
		memcpy(m_adv_data_baeslab.avd_playload_2_major, avd_playload_2_major_data, sizeof(avd_playload_2_major_data));
		memcpy(m_adv_data_baeslab.avd_playload_2_minor, avd_playload_2_minor_data, sizeof(avd_playload_2_minor_data));
		m_adv_data_baeslab.tx_power = 0xC6;

		//Set scan data by type
		emergency_data.scn_len_1 =  0x03;
		emergency_data.scn_type_1 = BLE_GAP_AD_TYPE_COMPLETE_LOCAL_NAME;
		memcpy(emergency_data.scn_playload_1, "A2", 2);
		emergency_data.scn_len_2 = 0x1A;
		emergency_data.scn_type_2 = BLE_GAP_AD_TYPE_SERVICE_DATA;

		//Clone scan data
		memcpy((void*)&activityA_data, (void*)&emergency_data, sizeof(emergency_data));
		memcpy((void*)&activityB_data, (void*)&emergency_data, sizeof(emergency_data));
		memcpy((void*)&axes_data, 	   (void*)&emergency_data, sizeof(emergency_data));
		memcpy((void*)&health_data, 	 (void*)&emergency_data, sizeof(emergency_data));

		memset(&data_a2_emergency.d_all_data[0], 0x00, sizeof(data_a2_emergency.d_all_data)); //Clear data for union
		memset(&data_a2_activityA.d_all_data[0], 0x00, sizeof(data_a2_activityA.d_all_data)); //Clear data for union
		memset(&data_a2_activityB.d_all_data[0], 0x00, sizeof(data_a2_activityB.d_all_data)); //Clear data for union
		memset(&data_a2_axes.d_all_data[0], 0x00, sizeof(data_a2_axes.d_all_data)); //Clear data for union
		memset(&data_a2_health.d_all_data[0], 0x00, sizeof(data_a2_health.d_all_data)); //Clear data for union


		switch(state){			
			case DATA_TYPE_EMERGENCY:
					data_a2_emergency.type_s = 	DATA_TYPE_NORMAL;
					data_a2_emergency.type_device = DATA_TYPE_A2;
					data_a2_emergency.type_data = DATA_TYPE_EMERGENCY;
					data_a2_emergency.sequent = sequent_emergency;
					data_a2_emergency.fw_version = FIRMWARE_REVISION_INT;
					data_a2_emergency.d_emergency.battery_level = m_a2_info.batter_value;
					m_status_type.set_byte = 0x00;
					m_status_type.set_bit_body_position = 1;
					//Must be edit again.
					m_status_type.set_bit_status = m_a2_info.emergency_status; 
					data_a2_emergency.d_emergency.tag_status = m_status_type;
					memset(emergency_data.scn_playload_2, 0x00, sizeof(emergency_data.scn_playload_2));
					memcpy(emergency_data.scn_playload_2, (const void*)&data_a2_emergency, sizeof(data_a2_emergency));
					sd_ble_gap_adv_data_set((const uint8_t*)&m_adv_data_baeslab, sizeof(m_adv_data_baeslab), (const uint8_t*)&emergency_data, sizeof(emergency_data));
					if(sequent_emergency++ >= MAXIMUN_SEQUENT)
					{
							sequent_emergency = 1;
					}
					state = DATA_TYPE_ACTIVITY_A;
					break;
			case DATA_TYPE_ACTIVITY_A:
					data_a2_activityA.type_s = 	DATA_TYPE_NORMAL;	
					data_a2_activityA.type_device = DATA_TYPE_A2;
					data_a2_activityA.type_data = DATA_TYPE_ACTIVITY_A;
					data_a2_activityA.sequent = sequent_activity_A;
					data_a2_activityA.fw_version = 0x01;
					data_a2_activityA.d_activity_a.speed[1] 	 = m_a2_info.speed>>8; 
					data_a2_activityA.d_activity_a.speed[0] 	 = m_a2_info.speed&0xFF; 
					data_a2_activityA.d_activity_a.calories[1] = m_a2_info.calories>>8;  
					data_a2_activityA.d_activity_a.calories[0] = m_a2_info.calories&0xFF;
					data_a2_activityA.d_activity_a.distance[1] = m_a2_info.distance>>8;
					data_a2_activityA.d_activity_a.distance[0] = m_a2_info.distance&0xFF;
					data_a2_activityA.d_activity_a.human_step_count[1] = m_a2_info.step_count>>8;
					data_a2_activityA.d_activity_a.human_step_count[0] = m_a2_info.step_count&0xFF;
					memset(activityA_data.scn_playload_2, 0x00, sizeof(activityA_data.scn_playload_2));
					memcpy(activityA_data.scn_playload_2, (const void*)&data_a2_activityA, sizeof(data_a2_activityA));
					sd_ble_gap_adv_data_set((const uint8_t*)&m_adv_data_baeslab, sizeof(m_adv_data_baeslab), (const uint8_t*)&activityA_data, sizeof(activityA_data));
					if(sequent_activity_A++ >= MAXIMUN_SEQUENT)
					{					
							sequent_activity_A = 1;
					}
					state = DATA_TYPE_ACTIVITY_B;
				break;
			case DATA_TYPE_ACTIVITY_B:
					data_a2_activityB.type_s = 	DATA_TYPE_NORMAL;	
					data_a2_activityB.type_device = DATA_TYPE_A1;
					data_a2_activityB.type_data = DATA_TYPE_ACTIVITY_B;
					data_a2_activityB.sequent = sequent_activity_B;
					data_a2_activityB.fw_version = 0x01;
					data_a2_activityB.d_activity_b.rest_time[1] 		= ((m_a2_info.rest_time)/60)  >> 8;
					data_a2_activityB.d_activity_b.rest_time[0] 		= (m_a2_info.rest_time/60)   & 0xFF;
					data_a2_activityB.d_activity_b.walking_time[1] 	= (m_a2_info.walking_time/60)  >> 8;
					data_a2_activityB.d_activity_b.walking_time[0] 	= (m_a2_info.walking_time/60)  &0xFF;
					data_a2_activityB.d_activity_b.jogging_time[1] 	=  m_a2_info.jogging_time >> 8;
					data_a2_activityB.d_activity_b.jogging_time[0] 	=  m_a2_info.jogging_time & 0xFF;
					data_a2_activityB.d_activity_b.running_time[1] 	=  m_a2_info.running_time >> 8;
					data_a2_activityB.d_activity_b.running_time[0] 	=  m_a2_info.running_time &0xFF;
					data_a2_activityB.d_activity_b.current_activity =  m_a2_info.current_activity;
					memset(activityB_data.scn_playload_2, 0x00, sizeof(activityB_data.scn_playload_2));
					memcpy(activityB_data.scn_playload_2, (const void*)&data_a2_activityB, sizeof(data_a2_activityB));
					sd_ble_gap_adv_data_set((const uint8_t*)&m_adv_data_baeslab, sizeof(m_adv_data_baeslab), (const uint8_t*)&activityB_data, sizeof(activityB_data));
					if(sequent_activity_B++ >= MAXIMUN_SEQUENT)
					{
							sequent_activity_B = 1;
					}
					state = DATA_TYPE_AXES;
					break;
			case DATA_TYPE_AXES:
					data_a2_axes.type_s = 	DATA_TYPE_NORMAL;	
					data_a2_axes.type_device = DATA_TYPE_A2;
					data_a2_axes.type_data = DATA_TYPE_AXES;
					data_a2_axes.sequent = sequent_axes;
					data_a2_axes.fw_version = 0x01;
					data_a2_axes.d_accelerotion_axes.accelerotion_x[1] =  m_a2_info.accel.x>>8;
					data_a2_axes.d_accelerotion_axes.accelerotion_x[0] =  m_a2_info.accel.x&0xFF;
					data_a2_axes.d_accelerotion_axes.accelerotion_y[1] =  m_a2_info.accel.y>>8;
					data_a2_axes.d_accelerotion_axes.accelerotion_y[0] =  m_a2_info.accel.y&0xFF;
					data_a2_axes.d_accelerotion_axes.accelerotion_z[1] =  m_a2_info.accel.z>>8;
					data_a2_axes.d_accelerotion_axes.accelerotion_z[0] =  m_a2_info.accel.z&0xFF;
					memset(axes_data.scn_playload_2, 0x00, sizeof(axes_data.scn_playload_2));
					memcpy(axes_data.scn_playload_2, (const void*)&data_a2_axes, sizeof(data_a2_axes));
					sd_ble_gap_adv_data_set((const uint8_t*)&m_adv_data_baeslab, sizeof(m_adv_data_baeslab), (const uint8_t*)&axes_data, sizeof(axes_data));	
					if(sequent_axes++ >= MAXIMUN_SEQUENT)
					{
							sequent_axes = 1;
					}			
					state = DATA_TYPE_HEALTH_MEASUREMENT;
					break;
			case DATA_TYPE_HEALTH_MEASUREMENT:
					data_a2_health.type_s 			= DATA_TYPE_NORMAL;	
					data_a2_health.type_device 	= DATA_TYPE_A2;
					data_a2_health.type_data 		= DATA_TYPE_HEALTH_MEASUREMENT;
					data_a2_health.sequent 			= sequent_health;
					data_a2_health.fw_version 	= 0x01;
					data_a2_health.d_health_measurement.heart_rate[1] 		= 0;
					data_a2_health.d_health_measurement.heart_rate[0] 		= m_a2_info.heart_rate;
					data_a2_health.d_health_measurement.pulse_oximeter[1] = 0;
					data_a2_health.d_health_measurement.pulse_oximeter[0] = m_a2_info.pulse_oximeter;
					data_a2_health.d_health_measurement.temperature[1] 		= m_a2_info.temperature>>8;
					data_a2_health.d_health_measurement.temperature[0] 		= m_a2_info.temperature&0xFF;
				
					memset(health_data.scn_playload_2, 0x00, sizeof(health_data.scn_playload_2));
					memcpy(health_data.scn_playload_2, (const void*)&data_a2_health, sizeof(data_a2_health));
					sd_ble_gap_adv_data_set((const uint8_t*)&m_adv_data_baeslab, sizeof(m_adv_data_baeslab), (const uint8_t*)&health_data, sizeof(health_data));	
					if(sequent_health++ >= MAXIMUN_SEQUENT)
					{
							sequent_health = 1;
					}
					state = DATA_TYPE_EMERGENCY;
				break;
		}
}

/**@brief Function for handling the Cancel SOS interval timer timeout.
 *
 * @details This function will be called each time the Cancel SOS start timer expires.
 *
 * @param[in] p_context  Pointer used for passing some arbitrary information (context) from the
 *                       app_start_timer() call to the timeout handler.
 */
static void cancel_sos_timeout_handler(void * p_context)
{
		m_sem_cancel_sos = 1;
	  NRF_LOG_INFO("Timer Cancel time out.");
}


/**@brief Function for handling the get value from bmi160 interval timer timeout.
 *
 * @details This function will be called each time the bmi160 start timer expires.
 *
 * @param[in] p_context  Pointer used for passing some arbitrary information (context) from the
 *                       app_start_timer() call to the timeout handler.
 */
static void acc_gyro_timeout_handler(void * p_context)
{
		if(m_a2_sensor_config.acc_en == 1)
		{
				m_a2_sensor_config.sem_get_acc_gyro_step_count = 1;
		}
	  
}


/**@brief Function for the Timer initialization.
 *
 * @details Initializes the timer module. This creates and starts application timers.
 */
static void timers_init(void)
{
    // Initialize timer module.
    uint32_t err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);

    // Create timers.
		err_code = app_timer_create(&m_display_scrolling_timer_id,
                                APP_TIMER_MODE_SINGLE_SHOT,
                                display_scrolling_timeout_handler);
    APP_ERROR_CHECK(err_code);
	
		err_code = app_timer_create(&m_ibeacon_interval_timer_id,
																APP_TIMER_MODE_REPEATED,
																ibeacon_interval_timeout_handler);
	
		err_code = app_timer_create(&m_cancel_sos_timer_id,
																APP_TIMER_MODE_SINGLE_SHOT,
                                cancel_sos_timeout_handler);
	
		err_code = app_timer_create(&m_acc_gyro_timer_id,
																APP_TIMER_MODE_REPEATED,
                                acc_gyro_timeout_handler);
}


/**@brief Function for the GAP initialization.
 *
 * @details This function sets up all the necessary GAP (Generic Access Profile) parameters of the
 *          device including the device name, appearance, and the preferred connection parameters.
 */
static void gap_params_init(void)
{
    uint32_t                err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *)DEVICE_NAME,
                                          strlen(DEVICE_NAME));
    APP_ERROR_CHECK(err_code);

    err_code = sd_ble_gap_appearance_set(BLE_APPEARANCE_HEART_RATE_SENSOR_HEART_RATE_BELT);
    APP_ERROR_CHECK(err_code);

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}


/**@brief GATT module event handler.
 */
static void gatt_evt_handler(nrf_ble_gatt_t * p_gatt, nrf_ble_gatt_evt_t const * p_evt)
{
    if (p_evt->evt_id == NRF_BLE_GATT_EVT_ATT_MTU_UPDATED)
    {
        NRF_LOG_INFO("GATT ATT MTU on connection 0x%x changed to %d.",
                     p_evt->conn_handle,
                     p_evt->params.att_mtu_effective);
    }
    ble_hrs_on_gatt_evt(&m_hrs, p_evt);
}


/**@brief Function for handling the Current Time Service client events.
 *
 * @details This function will be called for all events in the Current Time Service client that
 *          are passed to the application.
 *
 * @param[in] p_evt Event received from the Current Time Service client.
 */
static void on_cts_c_evt(ble_dts_t *p_time, ble_date_time_t ble_date_time)
{
		NRF_LOG_DEBUG("Update date time.");
		m_a2_info.unix_time = date_time_to_unixtimestamp(ble_date_time);
}


/**@brief Function for handling the data from the Nordic UART Service.
 *
 * @details This function will process the data received from the Nordic UART BLE Service and send
 *          it to the UART module.
 *
 * @param[in] p_nus    Nordic UART Service structure.
 * @param[in] p_data   Data to be send to UART module.
 * @param[in] length   Length of the data.
 */
/**@snippet [Handling the data received over BLE] */
static void nus_data_handler(ble_nus_evt_t * p_evt)
{
    if (p_evt->type == BLE_NUS_EVT_RX_DATA)
    {
        NRF_LOG_INFO("Received data from BLE NUS. Writing data on UART.");
        NRF_LOG_HEXDUMP_INFO(p_evt->params.rx_data.p_data, p_evt->params.rx_data.length);
				sync_nus_append_command(&m_sync_nus_comm, (uint8_t*)p_evt->params.rx_data.p_data, p_evt->params.rx_data.length );
    }
}


/**@brief Function for initializing services that will be used by the application.
 */
static void services_init(void)
{
    uint32_t err_code;
	  ble_dts_init_t  bts_init;
    ble_hrs_init_t hrs_init;
    ble_bas_init_t bas_init;
    ble_dis_init_t dis_init;
    uint8_t body_sensor_location;
	  ble_nus_init_t nus_init;
	
#ifndef DFU_DISABLE
    ble_dfu_buttonless_init_t dfus_init =
    {
        .evt_handler = ble_dfu_evt_handler
    }; 
    // Initialize the async SVCI interface to bootloader.
    err_code = ble_dfu_buttonless_async_svci_init();
    APP_ERROR_CHECK(err_code);


    err_code = ble_dfu_buttonless_init(&dfus_init);
    APP_ERROR_CHECK(err_code);
#endif
	
		//last current date time
		ble_date_time_t init_date_time;

		init_date_time.year 	 = 2017;
		init_date_time.month 	 = 3;
		init_date_time.day 		 = 29;
		init_date_time.hours	 = 10;
		init_date_time.minutes = 13;
		init_date_time.seconds = 00;

		memset(&bts_init, 0, sizeof(ble_dts_init_t)); 
		bts_init.evt_handler          = on_cts_c_evt;
		bts_init.is_notification_supported = true; 
		bts_init.init_date_time = init_date_time;
		err_code = ble_dts_init(&m_dts, &bts_init);
		APP_ERROR_CHECK(err_code);
		
		 // Initialize Heart Rate Service.
    body_sensor_location = BLE_HRS_BODY_SENSOR_LOCATION_HAND;
		
		memset(&hrs_init, 0, sizeof(hrs_init));

    hrs_init.evt_handler                 = NULL;
    hrs_init.is_sensor_contact_supported = true;
    hrs_init.p_body_sensor_location      = &body_sensor_location;

    // Here the sec level for the Heart Rate Service can be changed/increased.
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&hrs_init.hrs_hrm_attr_md.cccd_write_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&hrs_init.hrs_hrm_attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&hrs_init.hrs_hrm_attr_md.write_perm);

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&hrs_init.hrs_bsl_attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&hrs_init.hrs_bsl_attr_md.write_perm);

    err_code = ble_hrs_init(&m_hrs, &hrs_init);
    APP_ERROR_CHECK(err_code);

    // Initialize Battery Service.
    memset(&bas_init, 0, sizeof(bas_init));

    // Here the sec level for the Battery Service can be changed/increased.
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&bas_init.battery_level_char_attr_md.cccd_write_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&bas_init.battery_level_char_attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&bas_init.battery_level_char_attr_md.write_perm);

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&bas_init.battery_level_report_read_perm);

    bas_init.evt_handler          = NULL;
    bas_init.support_notification = true;
    bas_init.p_report_ref         = NULL;
    bas_init.initial_batt_level   = 100;

    err_code = ble_bas_init(&m_bas, &bas_init);
    APP_ERROR_CHECK(err_code);

    // Initialize Device Information Service.
    memset(&dis_init, 0, sizeof(dis_init));

    ble_srv_ascii_to_utf8(&dis_init.manufact_name_str, (char *)MANUFACTURER_NAME);
		
		ble_srv_ascii_to_utf8(&dis_init.model_num_str, (char *)MODEL_NUMBER);
		ble_srv_ascii_to_utf8(&dis_init.hw_rev_str, (char *)HARDWARE_REVISION);
		ble_srv_ascii_to_utf8(&dis_init.fw_rev_str, (char *)FIRMWARE_REVISION);
		
		

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&dis_init.dis_attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&dis_init.dis_attr_md.write_perm);

    err_code = ble_dis_init(&dis_init);
    APP_ERROR_CHECK(err_code);
		
		
		// Initialize UART Service.
    memset(&nus_init, 0, sizeof(nus_init));
    nus_init.data_handler = nus_data_handler;

    err_code = ble_nus_init(&m_nus, &nus_init);
    APP_ERROR_CHECK(err_code);
	
}


/**@brief Function for handling the Connection Parameters Module.
 *
 * @details This function will be called for all events in the Connection Parameters Module which
 *          are passed to the application.
 *          @note All this function does is to disconnect. This could have been done by simply
 *                setting the disconnect_on_fail config parameter, but instead we use the event
 *                handler mechanism to demonstrate its use.
 *
 * @param[in] p_evt  Event received from the Connection Parameters Module.
 */
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
    uint32_t err_code;

    if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        APP_ERROR_CHECK(err_code);
    }
}


/**@brief Function for handling a Connection Parameters error.
 *
 * @param[in] nrf_error  Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for initializing the Connection Parameters module.
 */
static void conn_params_init(void)
{
    uint32_t               err_code;
    ble_conn_params_init_t cp_init;

    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params                  = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle    = m_hrs.hrm_handles.cccd_handle; //BLE_GATT_HANDLE_INVALID;
    cp_init.disconnect_on_fail             = false;
    cp_init.evt_handler                    = on_conn_params_evt;
    cp_init.error_handler                  = conn_params_error_handler;

    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for starting timers.
 */
static void application_timers_start(void)
{
		uint32_t err_code;
		err_code = app_timer_start(m_ibeacon_interval_timer_id, IBEACON_CON_INTERVAL, NULL);
		APP_ERROR_CHECK(err_code); 
		
		err_code = app_timer_start(m_acc_gyro_timer_id, ACC_GYRO_INTERVAL, NULL);
		APP_ERROR_CHECK(err_code); 
}


/**@brief Function for putting the chip into sleep mode.
 *
 * @note This function will not return.
 */
static void sleep_mode_enter(void)
{
		uint32_t err_code;
    //uint32_t err_code = bsp_indication_set(BSP_INDICATE_IDLE);

    //APP_ERROR_CHECK(err_code);

    // Prepare wakeup buttons.
    //err_code = bsp_btn_ble_sleep_mode_prepare();
    //APP_ERROR_CHECK(err_code);

    // Go to system-off mode (this function will not return; wakeup will cause a reset).
    err_code = sd_power_system_off();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling advertising events.
 *
 * @details This function will be called for advertising events which are passed to the application.
 *
 * @param[in] ble_adv_evt  Advertising event.
 */
static void on_adv_evt(ble_adv_evt_t ble_adv_evt)
{
    uint32_t err_code;

    switch (ble_adv_evt)
    { 
        case BLE_ADV_EVT_FAST:       /**< Fast advertising mode has started. */  
            err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING);
            APP_ERROR_CHECK(err_code);
						NRF_LOG_INFO("BLE_ADV_EVT_FAST");
            break;

        case BLE_ADV_EVT_IDLE:      /**< Idle; no connectable advertising is ongoing.*/
					  NRF_LOG_INFO("BLE_ADV_EVT_IDLE");
            sleep_mode_enter();
            break;
				
				case BLE_ADV_EVT_DIRECTED:  /**< Direct advertising mode has started. */
						NRF_LOG_INFO("BLE_ADV_EVT_DIRECTED");
						break;
				case BLE_ADV_EVT_DIRECTED_SLOW:       /**< Directed advertising (low duty cycle) has started. */
					  NRF_LOG_INFO("BLE_ADV_EVT_DIRECTED_SLOW");
						break;
				    
				case BLE_ADV_EVT_SLOW:                /**< Slow advertising mode has started. */
					NRF_LOG_INFO("BLE_ADV_EVT_SLOW");
					break;
				case BLE_ADV_EVT_FAST_WHITELIST:      /**< Fast advertising mode using the whitelist has started. */
					NRF_LOG_INFO("BLE_ADV_EVT_FAST_WHITELIST");
					break;
				case BLE_ADV_EVT_SLOW_WHITELIST:      /**< Slow advertising mode using the whitelist has started. */
					NRF_LOG_INFO("BLE_ADV_EVT_SLOW_WHITELIST");
					break;
				case BLE_ADV_EVT_WHITELIST_REQUEST:   /**< Request a whitelist from the main application. For whitelist advertising to work, the whitelist must be set when this event occurs. */
					NRF_LOG_INFO("BLE_ADV_EVT_WHITELIST_REQUEST");
					break;
				case BLE_ADV_EVT_PEER_ADDR_REQUEST:
					NRF_LOG_INFO("BLE_ADV_EVT_PEER_ADDR_REQUEST");
					break;
        default:
            break;
    }
}


/**@brief Function for handling BLE events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 * @param[in]   p_context   Unused.
 */
static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{
    uint32_t err_code = NRF_SUCCESS;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_DISCONNECTED:
            NRF_LOG_INFO("Disconnected, reason %d.",
                          p_ble_evt->evt.gap_evt.params.disconnected.reason);
            m_conn_handle = BLE_CONN_HANDLE_INVALID;
				    m_status_ble = BLE_WAIT_CONNECT;
				break;

        case BLE_GAP_EVT_CONNECTED:
						NRF_LOG_INFO("Connected.");
            err_code = bsp_indication_set(BSP_INDICATE_CONNECTED);
            APP_ERROR_CHECK(err_code);
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
				    m_status_ble = BLE_CONNECTED;
            break;

#ifndef S140
        case BLE_GAP_EVT_PHY_UPDATE_REQUEST:
        {
            NRF_LOG_DEBUG("PHY update request.");
            ble_gap_phys_t const phys =
            {
                .rx_phys = BLE_GAP_PHY_AUTO,
                .tx_phys = BLE_GAP_PHY_AUTO,
            };
            err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
            APP_ERROR_CHECK(err_code);
        } break;
#endif

        case BLE_GATTC_EVT_TIMEOUT:
            // Disconnect on GATT Client timeout event.
            NRF_LOG_DEBUG("GATT Client Timeout.");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
				     m_status_ble = BLE_WAIT_CONNECT;
            break;

        case BLE_GATTS_EVT_TIMEOUT:
            // Disconnect on GATT Server timeout event.
            NRF_LOG_DEBUG("GATT Server Timeout.");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
				     m_status_ble = BLE_WAIT_CONNECT;
            break;

        case BLE_EVT_USER_MEM_REQUEST:
            err_code = sd_ble_user_mem_reply(p_ble_evt->evt.gattc_evt.conn_handle, NULL);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_RW_AUTHORIZE_REQUEST:
        {
            ble_gatts_evt_rw_authorize_request_t  req;
            ble_gatts_rw_authorize_reply_params_t auth_reply;

            req = p_ble_evt->evt.gatts_evt.params.authorize_request;

            if (req.type != BLE_GATTS_AUTHORIZE_TYPE_INVALID)
            {
                if ((req.request.write.op == BLE_GATTS_OP_PREP_WRITE_REQ)     ||
                    (req.request.write.op == BLE_GATTS_OP_EXEC_WRITE_REQ_NOW) ||
                    (req.request.write.op == BLE_GATTS_OP_EXEC_WRITE_REQ_CANCEL))
                {
                    if (req.type == BLE_GATTS_AUTHORIZE_TYPE_WRITE)
                    {
                        auth_reply.type = BLE_GATTS_AUTHORIZE_TYPE_WRITE;
                    }
                    else
                    {
                        auth_reply.type = BLE_GATTS_AUTHORIZE_TYPE_READ;
                    }
                    auth_reply.params.write.gatt_status = APP_FEATURE_NOT_SUPPORTED;
                    err_code = sd_ble_gatts_rw_authorize_reply(p_ble_evt->evt.gatts_evt.conn_handle,
                                                               &auth_reply);
                    APP_ERROR_CHECK(err_code);
                }
            }
        } break; // BLE_GATTS_EVT_RW_AUTHORIZE_REQUEST

        default:
            // No implementation needed.
            break;
    }
}


/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    ret_code_t err_code;

    err_code = nrf_sdh_enable_request();
    APP_ERROR_CHECK(err_code);

    // Configure the BLE stack using the default settings.
    // Fetch the start address of the application RAM.
    uint32_t ram_start = 0;
    err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
    APP_ERROR_CHECK(err_code);

    // Enable BLE stack.
    err_code = nrf_sdh_ble_enable(&ram_start);
    APP_ERROR_CHECK(err_code);

    NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);
}


/**@brief Function for the Peer Manager initialization.
 */
static void peer_manager_init()
{
    ble_gap_sec_params_t sec_param;
    ret_code_t           err_code;

    err_code = pm_init();
    APP_ERROR_CHECK(err_code);

    memset(&sec_param, 0, sizeof(ble_gap_sec_params_t));

    // Security parameters to be used for all security procedures.
    sec_param.bond           = SEC_PARAM_BOND;
    sec_param.mitm           = SEC_PARAM_MITM;
    sec_param.lesc           = SEC_PARAM_LESC;
    sec_param.keypress       = SEC_PARAM_KEYPRESS;
    sec_param.io_caps        = SEC_PARAM_IO_CAPABILITIES;
    sec_param.oob            = SEC_PARAM_OOB;
    sec_param.min_key_size   = SEC_PARAM_MIN_KEY_SIZE;
    sec_param.max_key_size   = SEC_PARAM_MAX_KEY_SIZE;
    sec_param.kdist_own.enc  = 1;
    sec_param.kdist_own.id   = 1;
    sec_param.kdist_peer.enc = 1;
    sec_param.kdist_peer.id  = 1;

    err_code = pm_sec_params_set(&sec_param);
    APP_ERROR_CHECK(err_code);

    err_code = pm_register(pm_evt_handler);
    APP_ERROR_CHECK(err_code);
}


/** @brief Clear bonding information from persistent storage.
 */
static void delete_bonds(void)
{
    ret_code_t err_code;

    NRF_LOG_INFO("Erase bonds!");

    err_code = pm_peers_delete();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling events from the BSP module.
 *
 * @param[in]   event   Event generated when button is pressed.
 */
static void bsp_event_handler(bsp_event_t event)
{
    uint32_t err_code;

    switch (event)
    {
        case BSP_EVENT_SLEEP:
            sleep_mode_enter();
            break; // BSP_EVENT_SLEEP

        case BSP_EVENT_DISCONNECT:
            err_code = sd_ble_gap_disconnect(m_conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            if (err_code != NRF_ERROR_INVALID_STATE)
            {
                APP_ERROR_CHECK(err_code);
            }
            break; // BSP_EVENT_DISCONNECT

        case BSP_EVENT_WHITELIST_OFF:
            if (m_conn_handle == BLE_CONN_HANDLE_INVALID)
            {
                err_code = ble_advertising_restart_without_whitelist(&m_advertising);
                if (err_code != NRF_ERROR_INVALID_STATE)
                {
                    APP_ERROR_CHECK(err_code);
                }
            }
            break; // BSP_EVENT_KEY_0

        default:
            break;
    }
}


/**@brief Function for initializing the Advertising functionality.
 */
static void advertising_init(void)
{
    uint32_t               err_code;
    ble_advertising_init_t init;

    memset(&init, 0, sizeof(init));

    init.advdata.name_type               = BLE_ADVDATA_FULL_NAME;
    init.advdata.include_appearance      = true;
    init.advdata.flags                   = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;
    init.advdata.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
    init.advdata.uuids_complete.p_uuids  = m_adv_uuids;

    init.config.ble_adv_fast_enabled  = true;
    init.config.ble_adv_fast_interval = APP_ADV_INTERVAL;
    init.config.ble_adv_fast_timeout  = APP_ADV_TIMEOUT_IN_SECONDS;

    init.evt_handler = on_adv_evt;

    err_code = ble_advertising_init(&m_advertising, &init);
    APP_ERROR_CHECK(err_code);

    ble_advertising_conn_cfg_tag_set(&m_advertising, APP_BLE_CONN_CFG_TAG);
}

/**@brief Function for initializing the Advertising functionality.
 */
static void advertising_normal(void)
{
    uint32_t               err_code;
    ble_advertising_init_t init;
	  memset(&init, 0, sizeof(init));
	
	
		err_code = app_timer_stop(m_ibeacon_interval_timer_id);
		APP_ERROR_CHECK(err_code); 	
	
		err_code = app_timer_start(m_ibeacon_interval_timer_id, IBEACON_CON_INTERVAL, NULL);
		APP_ERROR_CHECK(err_code); 
	
		//IBEACON_CON_INTERVAL_FAST
	
		if(m_status_ble != BLE_CONNECTED)
		{
				sd_ble_gap_adv_stop();		
				init.advdata.name_type               = BLE_ADVDATA_FULL_NAME;
				init.advdata.include_appearance      = true;
				init.advdata.flags                   = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;
				init.advdata.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
				init.advdata.uuids_complete.p_uuids  = m_adv_uuids;

				init.config.ble_adv_fast_enabled  = true;
				init.config.ble_adv_fast_interval = APP_ADV_INTERVAL;
				init.config.ble_adv_fast_timeout  = APP_ADV_TIMEOUT_IN_SECONDS;

				init.evt_handler = on_adv_evt;

				err_code = ble_advertising_init(&m_advertising, &init);
				APP_ERROR_CHECK(err_code);

				ble_advertising_conn_cfg_tag_set(&m_advertising, APP_BLE_CONN_CFG_TAG);
				
				advertising_start(erase_bonds);
		}
}

/**@brief Function for initializing the Advertising functionality.
 */
static void advertising_alert(void)
{
    uint32_t               err_code;
    ble_advertising_init_t init;
	  memset(&init, 0, sizeof(init));
	
		err_code = app_timer_stop(m_ibeacon_interval_timer_id);
		APP_ERROR_CHECK(err_code); 	
	
		err_code = app_timer_start(m_ibeacon_interval_timer_id, IBEACON_CON_INTERVAL_FAST, NULL);
		APP_ERROR_CHECK(err_code); 
	
		if(m_status_ble != BLE_CONNECTED)
		{
				sd_ble_gap_adv_stop();		
				init.advdata.name_type               = BLE_ADVDATA_FULL_NAME;
				init.advdata.include_appearance      = true;
				init.advdata.flags                   = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;
				init.advdata.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
				init.advdata.uuids_complete.p_uuids  = m_adv_uuids;

				init.config.ble_adv_fast_enabled  = true;
				init.config.ble_adv_fast_interval = APP_ADV_INTERVAL_ALERT;
				init.config.ble_adv_fast_timeout  = APP_ADV_TIMEOUT_IN_SECONDS;

				init.evt_handler = on_adv_evt;

				err_code = ble_advertising_init(&m_advertising, &init);
				APP_ERROR_CHECK(err_code);

				ble_advertising_conn_cfg_tag_set(&m_advertising, APP_BLE_CONN_CFG_TAG);
			
				advertising_start(erase_bonds);
		}
}


/**@brief Function for initializing buttons and leds.
 *
 * @param[out] p_erase_bonds  Will be true if the clear bonding button was pressed to wake the application up.
 */
static void buttons_leds_init(bool * p_erase_bonds)
{
    uint32_t err_code;
    bsp_event_t startup_event;

    err_code = bsp_init(BSP_INIT_LED | BSP_INIT_BUTTONS, bsp_event_handler);
    APP_ERROR_CHECK(err_code);

    err_code = bsp_btn_ble_init(NULL, &startup_event);
    APP_ERROR_CHECK(err_code);

    *p_erase_bonds = (startup_event == BSP_EVENT_CLEAR_BONDING_DATA);
}


/**@brief Function for the log initial.
 */
static void log_init(void)
{
    uint32_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}


/**@brief   Function for initializing the GATT module.
 * @details The GATT module handles ATT_MTU and Data Length update procedures automatically.
 */
static void gatt_init(void)
{
    ret_code_t err_code = nrf_ble_gatt_init(&m_gatt, gatt_evt_handler);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for starting advertising.
 */
static void advertising_start(bool erase_bonds)
{
    if (erase_bonds == true)
    {
        delete_bonds();
        // Advertising is started by PM_EVT_PEERS_DELETE_SUCCEEDED event.
    }
    else
    {
        uint32_t err_code = ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST);
        APP_ERROR_CHECK(err_code);

        NRF_LOG_INFO("advertising is started");
    }
}


static void power_management_init(void)
{
    uint32_t err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);
}


#define ADC_BATT_BITPERLEVEL 		0.44324324324324
uint8_t battery_soc(uint16_t adc_value)
{
		int16_t batt_level = 0;
	
		batt_level = (adc_value - 800);
	
		if(batt_level > 100)
		{
				batt_level = 100;
		}
	
		if(batt_level < 0)
		{
				batt_level = 0;
		}
		
		return batt_level;
}


void saadc_event_handler(nrf_drv_saadc_evt_t const * p_event)
{
		static uint16_t low_battery_count = 0;
	
    if (p_event->type == NRF_DRV_SAADC_EVT_DONE)
    {
        nrf_saadc_value_t adc_result;
        uint32_t          err_code;

        adc_result = p_event->data.done.p_buffer[0];
			
				m_a2_info.batter_value = battery_soc(adc_result);
				
				//NRF_LOG_INFO("Raw %d,SOC %d,BattFullDetect %d, %dV", adc_result, m_a2_info.batter_value, nrf_gpio_pin_read(BATT_FULL_PIN) , (uint16_t)(ADC_BATT_BITPERLEVEL*adc_result));
				
        err_code = nrf_drv_saadc_buffer_convert(p_event->data.done.p_buffer, 1);
        APP_ERROR_CHECK(err_code);
			
				adc_uninit();
			
				if(m_a2_info.batter_value < 10 && nrf_gpio_pin_read(USB_PIN) != 1)
				{
						low_battery_count++;
						if(low_battery_count > 10)
						{
								m_a2_info.low_battery_detected = 1;
						}
				}else{
						low_battery_count = 0;
					  m_a2_info.low_battery_detected = 0;
				}
    }
}


/**@brief Function for configuring ADC to do battery level conversion.
 */
static void adc_init(void)
{
    ret_code_t err_code = nrf_drv_saadc_init(NULL, saadc_event_handler);
    APP_ERROR_CHECK(err_code);

    nrf_saadc_channel_config_t config =
        NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN0);
    err_code = nrf_drv_saadc_channel_init(0, &config);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_saadc_buffer_convert(&adc_buf[0], 1);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_saadc_buffer_convert(&adc_buf[1], 1);
    APP_ERROR_CHECK(err_code);
}

static void adc_uninit(void)
{
		nrf_drv_saadc_uninit();
}


/**@brief Function for update unix_timestamp.
 */
static void aider_update_time()
{
	  ble_date_time_t m_date_time;
		m_a2_info.unix_time++;

		unixtimestamp_to_date_time(m_a2_info.unix_time, &m_date_time);
		m_a2_info.time_s = m_date_time.seconds;
		m_a2_info.time_m = m_date_time.minutes;
		m_a2_info.time_h = m_date_time.hours;
	  m_a2_info.date_y = m_date_time.year;
	  m_a2_info.date_m = m_date_time.month;
		m_a2_info.date_d = m_date_time.day;
}

/**@brief Function for update unix_timestamp.
 */
static void sem_checking()
{
		if( m_a2_info.unix_time%m_a2_sensor_config.pedometer_interval == 0)
		{
				if(m_a2_sensor_config.acc_en == 1)
				{
						m_a2_sensor_config.sem_save_pedo_to_flash = 1;
				}
		}	
		
		if(m_a2_info.unix_time%m_a2_sensor_config.heart_rate_interval == 0 && m_a2_info.usb_charging_status == USB_CHARGER_DO_NOT_CONNECT_STATE)
		{
				if(m_a2_sensor_config.heart_rate_en == 1 && m_a2_info.usb_pin_status != 1)
				{
						m_a2_sensor_config.sem_get_heart_rate = 1;
				}
		}
		
		if(m_a2_info.unix_time%m_a2_sensor_config.temperature_interal == 0)
		{
				if(m_a2_sensor_config.temperature_en == 1)
				{
						m_a2_sensor_config.sem_get_temperature = 0;
				}
		}
		
		if(m_a2_info.unix_time%m_a2_sensor_config.battery_monitor_interval == 0)
		{
				m_a2_sensor_config.sem_get_battery_level = 1;
		}
}


static void aider_check_reset_sensor_data()
{
		if(m_a2_info.unix_time% 86400 == 0)	
		{
			  m_a2_config.a2_sensor_config.sem_reset_sensor_data = 1;
			  //reset step flag
				m_a2_info.distance_in_d = 0;
			  m_a2_info.calories_in_d = 0;
		}
}

volatile uint32_t rct_count = 0;

/** @brief: Function for handling the RTC0 interrupts.
 * Triggered on TICK and COMPARE0 match.
 */
static void rtc_handler(nrf_drv_rtc_int_type_t int_type)
{
		uint32_t err_code;
		if(int_type == NRF_DRV_RTC_INT_COMPARE0)
		{
			  err_code = nrf_drv_rtc_cc_set(&rtc,0,COMPARE_COUNTERTIME * 8,true);
				APP_ERROR_CHECK(err_code);
				nrf_drv_rtc_counter_clear(&rtc);
				aider_update_time();
				sem_checking();
			  aider_check_reset_sensor_data();
				rct_count++;
			  //NRF_LOG_INFO(".");
			  //NRF_LOG_FLUSH();
		}
}


/** @brief Function initialization and configuration of RTC driver instance.
 */
static void rtc_config(void)
{
    uint32_t err_code;

    //Initialize RTC instance
    nrf_drv_rtc_config_t config = NRF_DRV_RTC_DEFAULT_CONFIG;
    config.prescaler = 4095;
    err_code = nrf_drv_rtc_init(&rtc, &config, rtc_handler);
    APP_ERROR_CHECK(err_code);

    //Enable tick event & interrupt
    nrf_drv_rtc_tick_enable(&rtc,true);

    //Set compare channel to trigger interrupt after COMPARE_COUNTERTIME seconds
    err_code = nrf_drv_rtc_cc_set(&rtc,0,COMPARE_COUNTERTIME * 8,true);
    APP_ERROR_CHECK(err_code);

		
    //Power on RTC instance
    nrf_drv_rtc_enable(&rtc);
}

/**@brief Function for set gpio pin of aider a2 entry.
 */
static void a2_init_gpio(void)
{
		nrf_gpio_cfg_input(USB_PIN, NRF_GPIO_PIN_NOPULL);
		nrf_gpio_cfg_input(BATT_FULL_PIN, NRF_GPIO_PIN_PULLDOWN);
		nrf_gpio_cfg_input(LEFT_BT_PIN, NRF_GPIO_PIN_NOPULL);
		nrf_gpio_cfg_input(RIGHT_BT_PIN, NRF_GPIO_PIN_NOPULL);
		nrf_gpio_cfg_input(ALERT_BT_PIN, NRF_GPIO_PIN_NOPULL);
		nrf_gpio_cfg_input(VIBRATION_PIN, NRF_GPIO_PIN_NOPULL );
		nrf_gpio_cfg_input(OLED_POWER_EN_PIN, NRF_GPIO_PIN_NOPULL );
		nrf_gpio_cfg_input(NRF_BMI160_POWER_EN_PIN, NRF_GPIO_PIN_NOPULL );
		
		int j;
		
		for(j=0;j<32;j++)
		{
				 nrf_gpio_cfg(
						j,
						NRF_GPIO_PIN_DIR_INPUT,
						NRF_GPIO_PIN_INPUT_DISCONNECT,
						NRF_GPIO_PIN_NOPULL,
						NRF_GPIO_PIN_S0S1,
						NRF_GPIO_PIN_NOSENSE);
		}
		
		// SET for set using bypass mode
		nrf_gpio_cfg_output(A2_3V3_BYPASS_PIN);
		nrf_gpio_pin_clear(A2_3V3_BYPASS_PIN);
	
		// SET for Disable Power 3V3 to BMI160
		nrf_gpio_cfg_output(NRF_BMI160_POWER_EN_PIN);
		nrf_gpio_pin_clear(NRF_BMI160_POWER_EN_PIN);	
		
		// Clear for disable vibration
		nrf_gpio_cfg_output(VIBRATION_PIN);
		nrf_gpio_pin_clear(VIBRATION_PIN);	
		
		// Set for disable oled
		nrf_gpio_cfg_output(OLED_POWER_EN_PIN);
		nrf_gpio_pin_set(OLED_POWER_EN_PIN);
						
		nrf_gpio_cfg_output(OLED_SCL_PIN);
		nrf_gpio_pin_set(OLED_SCL_PIN);	
		nrf_gpio_cfg_output(OLED_SDA_PIN);
		nrf_gpio_pin_set(OLED_SDA_PIN);	
		
		// Clear for Disable Power 1.8V to Max30120 
		nrf_gpio_cfg_output(HEAET_RATE_EN_1V8_PIN);
		nrf_gpio_pin_clear(HEAET_RATE_EN_1V8_PIN);	
		
}

/**@brief Function for initial spi pin and cs pin for bmi160 and flash memory
 */
void a2_spi_int()
{
		m_a2_spi_comm.mosi_pin         = NRF_SPI_MOSI_PIN;
		m_a2_spi_comm.miso_pin         = NRF_SPI_MISO_PIN;
		m_a2_spi_comm.sck_pin          = NRF_SPI_SCK_PIN;
		m_a2_spi_comm.cs_bmi_pin       = NRF_SPI_BMI160_CS_PIN;
		m_a2_spi_comm.cs_flash_pin     = NRF_SPI_FLASH_CS_PIN;
		m_a2_spi_comm.power_bmi_en_pin = NRF_BMI160_POWER_EN_PIN;
		m_a2_spi_comm.reset_flash_pin  = NRF_SPI_FLASH_RESET_PIN;
		m_a2_spi_comm.wp_flash_pin     = NRF_SPI_FLASH_WP_PIN;	
		nrf_drv_a2_spi_init(&m_a2_spi_comm);	
}


uint8_t hand_gesture_recognition_with_acc_process(struct bmi160_sensor_data accel)
{
		//struct bmi160_sensor_data buf_accel[BUF_ACCEL_SIZE];
		static uint8_t hand_gesture_state;
		uint8_t res_hand_gesture;
		
		switch(hand_gesture_state)
		{
			case 0:
			case 1:
			case 2:
			case 3:
			case 4:
			case 5:
			case 6:
			case 7:
				if(accel.z > -5000 && accel.z < 5000)
				{
						//NRF_LOG_INFO("Acc %d, %d, %d, State 1", m_a2_info.accel.x, m_a2_info.accel.y, m_a2_info.accel.z);
						//NRF_LOG_FLUSH();
					  hand_gesture_state++;
				}else{
					  hand_gesture_state = 0;
					  //NRF_LOG_INFO("Acc %d, %d, %d, State 0", m_a2_info.accel.x, m_a2_info.accel.y, m_a2_info.accel.z);
						//NRF_LOG_FLUSH();
				}
				res_hand_gesture = HAND_GESTURE_UNKNOW;
				break;
			case 8:
				if(accel.z > -5000 && accel.z < 5000)
				{
						//NRF_LOG_INFO("Acc %d, %d, %d, State 2", m_a2_info.accel.x, m_a2_info.accel.y, m_a2_info.accel.z);
						//NRF_LOG_FLUSH();
					  hand_gesture_state = 8;
				}else if(accel.z < -10000)
				{
						hand_gesture_state = 9;
					  //NRF_LOG_INFO("Acc %d, %d, %d, State 2 Detect", m_a2_info.accel.x, m_a2_info.accel.y, m_a2_info.accel.z);
						//NRF_LOG_FLUSH();
				}
				res_hand_gesture = HAND_GESTURE_UNKNOW;
				break;
			case 9:
			case 10:
				if(accel.z < -8000)
				{
						hand_gesture_state++;
					  //NRF_LOG_INFO("Acc %d, %d, %d, State 3 Detect", m_a2_info.accel.x, m_a2_info.accel.y, m_a2_info.accel.z);
						//NRF_LOG_FLUSH();
				}
				res_hand_gesture = HAND_GESTURE_UNKNOW;
				break;
			case 11:
				//NRF_LOG_INFO("Acc %d, %d, %d, State 4 HAND UP DETECTED", m_a2_info.accel.x, m_a2_info.accel.y, m_a2_info.accel.z);
				//NRF_LOG_FLUSH();
				hand_gesture_state = 0;
			  res_hand_gesture = HAND_GESTURE_WATCH_TIME;
				break;
			
		}

		return res_hand_gesture;
}

/**@brief Function for initial bmi160 pedometer, acceleration  and gyrometer
 */
void pedometer_init()
{
		uint32_t res;
		
		nrf_drv_bmi160_power_and_cs_init(&m_a2_spi_comm);
	
	  res = nrf_drv_bmi160_spi_int(&m_a2_spi_comm, &m_bmi160_dev);
		if(res == NRF_SUCCESS)
		{
				NRF_LOG_INFO("[OK]\tBMI160");
		}else{
				NRF_LOG_INFO("[FALL]\tBMI160");
		}
		res = nrf_drv_bmi160_acc_config(&m_a2_spi_comm, &m_bmi160_dev);
		if(res == NRF_SUCCESS)
		{
				NRF_LOG_INFO("[OK]\tSETUP ACC");
		}else{
				NRF_LOG_INFO("[FALL]\tSETUP ACC");
		}
		res = nrf_drv_bmi160_step_count_conf(&m_a2_spi_comm, &m_bmi160_dev);
		if(res == NRF_SUCCESS)
		{
				NRF_LOG_INFO("[OK]\tSETUP STEP COUNT");
		}else{
				NRF_LOG_INFO("[FALL]\tSETUP STEP COUNT");
		}
		//set_accel_low_g_int(&int_config, &m_bmi160_dev);
		//nrf_drv_bmi160_free_fall_conf(&m_a2_spi_comm, &m_bmi160_dev);
		
}

void pedometer_sleep()
{
		m_bmi160_dev.accel_cfg.power = BMI160_ACCEL_SUSPEND_MODE;
		m_bmi160_dev.gyro_cfg.power  = BMI160_GYRO_SUSPEND_MODE;
	  bmi160_set_power_mode(&m_bmi160_dev);	
		nrf_gpio_cfg_output(NRF_BMI160_POWER_EN_PIN);
		nrf_gpio_pin_set(NRF_BMI160_POWER_EN_PIN);	
}

/**@brief Function for initial at45dbxx flash memory.
 */
void print_Hex(uint8_t buffer[], uint16_t len)
{
		NRF_LOG_INFO("RES::");

		for(uint16_t j=0;j<len;j=j+8)
		{
				for(uint16_t i=j;i<j+8&&i<len;i++)
				{
						NRF_LOG_RAW_INFO("0x%02X,",buffer[i]);
				}
				NRF_LOG_RAW_INFO("\n");
				NRF_LOG_FLUSH();
				nrf_delay_ms(10);
		}
		
}


void at45dbxx_init()
{
		uint8_t res;
	
		nrf_drv_flash_spi_init(&m_a2_spi_comm, &m_at45dbxx_dev);
		at45dbxx_wakeup_pin(&m_at45dbxx_dev);
		at45dbxx_resume_deep_power_down(&m_at45dbxx_dev);
		res = at45dbxx_is_available(&m_at45dbxx_dev);
		at45dbxx_wakeup_pin(&m_at45dbxx_dev);
	
		if(res == AT45DB_SUCCESS)
		{
				NRF_LOG_INFO("[OK]\tFLASH MEMORY");
			  //flash_erase_chip();
		}else{
				NRF_LOG_INFO("[FALL]\tFLASH MEMORY");
		}
}

void at45dbxx_uninit()
{
		at34dbxx_deep_power_down(&m_at45dbxx_dev);
	  at45dbxx_is_available(&m_at45dbxx_dev);
	  at45dbxx_is_available(&m_at45dbxx_dev);
}

/**@brief Function for initial ssd1306 oled 0.96 inch.
 */
void oled_init(void)
{
		uint8_t i2c_buf;
		uint8_t reg = 0;
	
		m_oled_comm.oled_power_en_pin = OLED_POWER_EN_PIN;
		m_oled_comm.scl_pin 					= OLED_SCL_PIN;
		m_oled_comm.sda_pin 					= OLED_SDA_PIN;
	
		nrf_drv_a2_i2c_0_init(&m_oled_comm);
		nrf_drv_a2_oled_init(&m_oled_comm, &m_oled_ssd1306_dev);		
	
		
		if( nrf_drv_i2c_0_read(OLED_SSD1306_I2C_ADDRESS,reg,&i2c_buf,sizeof(i2c_buf)) == NRF_SUCCESS)
		{
				NRF_LOG_INFO("[OK]\tOLED Initial");
		}else{
				NRF_LOG_INFO("[FALL]\tOLED Initial");
		}
		
		
		ssd1306_clear_display();
		ssd1306_set_textsize(2);
		ssd1306_set_textcolor(WHITE);
		ssd1306_set_cursor(40,25);
		ssd1306_putstring("AIDER");
		ssd1306_display();
	
		nrf_delay_ms(1000);	
		nrf_drv_a2_oled_shutdown();
		nrf_drv_a2_i2c_0_deinit(&m_oled_comm);
}

/**@brief Function for application main entry.
 */
void max30102_tmp007_init()
{
		uint8_t res;
		static double t_hr;
	
		m_max_tmp_comm.scl_pin 										= MAX_TMP_SCL_PIN;
		m_max_tmp_comm.sda_pin 										= MAX_TMP_SDA_PIN;
		m_max_tmp_comm.en_power_1V8_max30102_pin 	= HEAET_RATE_EN_1V8_PIN;
		m_max_tmp_comm.isr_max30102_pin						= HEAET_RATE_INT_PIN;
		m_max_tmp_comm.isr_temp007_pin						= TMP007_INT_PIN;
	
		nrf_drv_a2_i2c_1_init(&m_max_tmp_comm);
		nrf_drv_max30102_reg_i2c_func(&m_max_tmp_comm, &m_max30102_dev);
		res = max30102_initaial(&m_max30102_dev);
		if(res == MAX30102_SUCCESS)
		{
				max30102_setup(&m_max30102_dev);
				t_hr = max30102_readTemperature(&m_max30102_dev);
				nrf_delay_ms(25);
				t_hr = max30102_readTemperature(&m_max30102_dev);
				nrf_delay_ms(25);
				t_hr = max30102_readTemperature(&m_max30102_dev);

				m_a2_info.temperature = (uint16_t)(t_hr*100);
				nrf_drv_i2c_max30102_shutdown(&m_max_tmp_comm, &m_max30102_dev);	
				NRF_LOG_INFO("[OK]\tMax30102 Initial, Temperature %ld.%02d",(uint16_t)(t_hr), (uint8_t)( (uint16_t)(t_hr*100)%100));
				m_a2_info.broken_max30120_status = false;
		}else{
				NRF_LOG_INFO("[FALL]\tMax30102 FAIL");
				NRF_LOG_FLUSH();
			  m_a2_info.broken_max30120_status = true;
				nrf_delay_ms(30);
		}
		
		nrf_drv_a2_i2c_1_deinit(&m_max_tmp_comm);
}

static void a2_logic_task(void)
{
		static uint32_t pre_alert_pin_detected_time;	
	  static uint32_t charging_time_couter = 0;
		static uint32_t pre_time = 0;
		
		if(m_a2_sensor_config.sem_manual_recheck_hr != 1)
		{
				switch(m_a2_info.alert_bt_state)
				{
					case ALERT_BT_CHECK_WAIT_KRY:
							if(nrf_gpio_pin_read(ALERT_BT_PIN) == 0)
							{
									m_a2_info.touch_pin_status = ALERT_BT_PUSH;	
									NRF_LOG_INFO("ALERT_BT_CHECK_WAIT_KRY");
									NRF_LOG_FLUSH();
									m_a2_info.alert_bt_state = ALERT_BT_CHECK_AGNORE;
									pre_alert_pin_detected_time = m_a2_info.unix_time;	
							}else if(m_a2_info.free_fall_status == 1){
									pre_alert_pin_detected_time = m_a2_info.unix_time;
									m_a2_info.alert_bt_state = ALERT_BT_CHECK_CALCEL;
									m_a2_info.touch_pin_status = NO_BT_PUSH;
									m_a2_info.emergency_status = BLE_AIDER_EVT_FALL_ALERT;	
							}else{
									m_a2_info.alert_bt_state = ALERT_BT_CHECK_WAIT_KRY;
									m_a2_info.touch_pin_status = NO_BT_PUSH;
									m_a2_info.emergency_status = BLE_AIDER_EVT_NORMAL;	
							}
							break;
					case ALERT_BT_CHECK_AGNORE:
							if(nrf_gpio_pin_read(ALERT_BT_PIN) == 0 && nrf_gpio_pin_read(LEFT_BT_PIN) == 0 && nrf_gpio_pin_read(RIGHT_BT_PIN) == 0 && pre_alert_pin_detected_time != m_a2_info.unix_time)
							{
									NRF_LOG_INFO("ALERT_BT_CHECK_AGNORE");
									NRF_LOG_FLUSH();
									m_a2_info.alert_bt_state = ALERT_BT_CHECK_LEVEL_3;
									pre_alert_pin_detected_time = m_a2_info.unix_time;
							}else if(nrf_gpio_pin_read(ALERT_BT_PIN) == 0 && nrf_gpio_pin_read(LEFT_BT_PIN) == 0 && nrf_gpio_pin_read(RIGHT_BT_PIN) == 0 && pre_alert_pin_detected_time == m_a2_info.unix_time)
							{
									m_a2_info.alert_bt_state = ALERT_BT_CHECK_AGNORE;
							}else{
									m_a2_info.alert_bt_state = ALERT_BT_CHECK_WAIT_KRY;
									m_a2_info.touch_pin_status = NO_BT_PUSH;	
							}
							break;
					case ALERT_BT_CHECK_LEVEL_3:
							if(nrf_gpio_pin_read(ALERT_BT_PIN) == 0 && nrf_gpio_pin_read(LEFT_BT_PIN) == 0 && nrf_gpio_pin_read(RIGHT_BT_PIN) == 0 && pre_alert_pin_detected_time != m_a2_info.unix_time)
							{
									m_a2_info.touch_pin_status = ALERT_LEVEL_3;	
									NRF_LOG_INFO("ALERT_BT_CHECK_LEVEL_3");
									NRF_LOG_FLUSH();
									m_a2_info.alert_bt_state = ALERT_BT_CHECK_LEVEL_2;
									pre_alert_pin_detected_time = m_a2_info.unix_time;
							}else if(nrf_gpio_pin_read(ALERT_BT_PIN) == 0 && nrf_gpio_pin_read(LEFT_BT_PIN) == 0 && nrf_gpio_pin_read(RIGHT_BT_PIN) == 0 && pre_alert_pin_detected_time == m_a2_info.unix_time)
							{
									m_a2_info.alert_bt_state = ALERT_BT_CHECK_LEVEL_3;
							}else{
									m_a2_info.alert_bt_state = ALERT_BT_CHECK_WAIT_KRY;
									m_a2_info.touch_pin_status = NO_BT_PUSH;	
							}
							break;
					case ALERT_BT_CHECK_LEVEL_2:
							if(nrf_gpio_pin_read(ALERT_BT_PIN) == 0 && nrf_gpio_pin_read(LEFT_BT_PIN) == 0 && nrf_gpio_pin_read(RIGHT_BT_PIN) == 0 && pre_alert_pin_detected_time != m_a2_info.unix_time)
							{
									m_a2_info.touch_pin_status = ALERT_LEVEL_2;	
									NRF_LOG_INFO("ALERT_BT_CHECK_LEVEL_2");
									NRF_LOG_FLUSH();
									m_a2_info.alert_bt_state = ALERT_BT_CHECK_LEVEL_1;
									pre_alert_pin_detected_time = m_a2_info.unix_time;	
							}else if(nrf_gpio_pin_read(ALERT_BT_PIN) == 0 && nrf_gpio_pin_read(LEFT_BT_PIN) == 0 && nrf_gpio_pin_read(RIGHT_BT_PIN) == 0 && pre_alert_pin_detected_time == m_a2_info.unix_time){
									m_a2_info.alert_bt_state = ALERT_BT_CHECK_LEVEL_2;
							}else{
									m_a2_info.alert_bt_state = ALERT_BT_CHECK_WAIT_KRY;
									m_a2_info.touch_pin_status = NO_BT_PUSH;
							}
							break;
					case ALERT_BT_CHECK_LEVEL_1:
							if(nrf_gpio_pin_read(ALERT_BT_PIN) == 0 && nrf_gpio_pin_read(LEFT_BT_PIN) == 0 && nrf_gpio_pin_read(RIGHT_BT_PIN) == 0 && pre_alert_pin_detected_time != m_a2_info.unix_time)
							{
									m_a2_info.touch_pin_status = ALERT_LEVEL_1;	
									NRF_LOG_INFO("ALERT_BT_CHECK_LEVEL_1");
									NRF_LOG_FLUSH();
									m_a2_info.alert_bt_state = ALERT_BT_CHECK_LEVEL_0;
									pre_alert_pin_detected_time = m_a2_info.unix_time;
							}else if(nrf_gpio_pin_read(ALERT_BT_PIN) == 0 && nrf_gpio_pin_read(LEFT_BT_PIN) == 0 && nrf_gpio_pin_read(RIGHT_BT_PIN) == 0 && pre_alert_pin_detected_time == m_a2_info.unix_time){
									m_a2_info.alert_bt_state = ALERT_BT_CHECK_LEVEL_1;
							}else{
									m_a2_info.alert_bt_state = ALERT_BT_CHECK_WAIT_KRY;
									m_a2_info.touch_pin_status = NO_BT_PUSH;
							}
							break;				
					case ALERT_BT_CHECK_LEVEL_0:
							if(nrf_gpio_pin_read(ALERT_BT_PIN) == 0 && nrf_gpio_pin_read(LEFT_BT_PIN) == 0 && nrf_gpio_pin_read(RIGHT_BT_PIN) == 0&& pre_alert_pin_detected_time != m_a2_info.unix_time)
							{
									m_a2_info.touch_pin_status = ALERT_LEVEL_0;	
									NRF_LOG_INFO("ALERT_BT_CHECK_LEVEL_0");
									NRF_LOG_FLUSH();
									m_a2_info.alert_bt_state = ALERT_BT_CHECK_SOS;
									pre_alert_pin_detected_time = m_a2_info.unix_time;
							}else if(nrf_gpio_pin_read(ALERT_BT_PIN) == 0 && nrf_gpio_pin_read(LEFT_BT_PIN) == 0 && nrf_gpio_pin_read(RIGHT_BT_PIN) == 0 && pre_alert_pin_detected_time == m_a2_info.unix_time){
									m_a2_info.alert_bt_state = ALERT_BT_CHECK_LEVEL_0;
							}else{
									m_a2_info.alert_bt_state = ALERT_BT_CHECK_WAIT_KRY;	
									m_a2_info.touch_pin_status = NO_BT_PUSH;
							}
							break;
					case ALERT_BT_CHECK_SOS:
							if(nrf_gpio_pin_read(ALERT_BT_PIN) == 0 && nrf_gpio_pin_read(LEFT_BT_PIN) == 0 && nrf_gpio_pin_read(RIGHT_BT_PIN) == 0 && pre_alert_pin_detected_time != m_a2_info.unix_time)
							{	
									m_a2_sensor_config.sem_get_heart_rate = 1;
									m_a2_info.touch_pin_status = ALERT_SOS_DETECT;	
									NRF_LOG_INFO("ALERT_BT_CHECK_SOS");
									NRF_LOG_FLUSH();
									m_a2_info.alert_bt_state = ALERT_BT_CHECK_UNPRESS_SOS;
									pre_alert_pin_detected_time = m_a2_info.unix_time;
									advertising_alert();
									m_a2_info.emergency_status = BLE_AIDER_EVT_BT_ALERT;
									m_a2_button_alert_log.unix_time = m_a2_info.unix_time;
									m_a2_button_alert_log.sum_number_push_button = 1;
									m_a2_button_alert_log.level = LEVEL_SOS;
									m_a2_button_alert_log.cancel_detect = BUTTON_CANCEL_NOT_DETECT;
								
							}else if(nrf_gpio_pin_read(ALERT_BT_PIN) == 0 && nrf_gpio_pin_read(LEFT_BT_PIN) == 0 && nrf_gpio_pin_read(RIGHT_BT_PIN) == 0 && pre_alert_pin_detected_time == m_a2_info.unix_time){
									m_a2_info.alert_bt_state = ALERT_BT_CHECK_SOS;
							}else{
									m_a2_info.alert_bt_state = ALERT_BT_CHECK_WAIT_KRY;	
									m_a2_info.touch_pin_status = NO_BT_PUSH;
							} 
							break;
					case ALERT_BT_CHECK_UNPRESS_SOS:
							if(nrf_gpio_pin_read(ALERT_BT_PIN) == 1)
							{
									m_a2_sensor_config.sem_get_heart_rate = 1;
									NRF_LOG_INFO("ALERT_BT_CHECK_UNPRESS_SOS");
									NRF_LOG_FLUSH();
									m_a2_info.alert_bt_state = ALERT_BT_CHECK_CALCEL;	
									app_timer_start(m_cancel_sos_timer_id, CANCEL_SOS_INTERVAL, NULL);	
									m_sem_cancel_sos = 0;
							}else{
									m_a2_info.alert_bt_state = ALERT_BT_CHECK_UNPRESS_SOS;	
							}
							break;
					case ALERT_BT_CHECK_CALCEL:
							if(nrf_gpio_pin_read(ALERT_BT_PIN) == 0)
							{
									m_a2_info.touch_pin_status = ALERT_CANCEL_DETECT;
									NRF_LOG_INFO("ALERT_BT_CHECK_CALCEL");
									NRF_LOG_FLUSH();
									advertising_normal();
									m_a2_info.emergency_status = BLE_AIDER_EVT_NORMAL;
									m_a2_info.alert_bt_state = ALERT_BT_CHECK_UNPRESS_CANCEL;	
									m_a2_button_alert_log.cancel_detect = BUTTON_CANCEL_DETECT;
									m_a2_button_alert_log.cancel_time = m_a2_info.unix_time;
								
									a2_spi_int();
									at45dbxx_init();
									//Read the last page to buffer;
									a2_read_button_alert_last_buffer( &m_logging_dev, &m_queue_type.button_alert);
									//Save alert to buf
									a2_save_button_alert_to_buf(&m_logging_dev, &m_queue_type.button_alert, &m_a2_button_alert_log);						
									a2_write_button_alert_last_buffer(&m_logging_dev, &m_queue_type.button_alert);	
									NRF_LOG_INFO("Button Alert Save to buffer");
									//write buffer;
									if(a2_button_alert_buf_is_full(&m_queue_type.button_alert) == QUE_TURE)
									{
											NRF_LOG_INFO("Button Alert Save to Flash");
											a2_write_button_alert_log(&m_logging_dev, &m_queue_type.button_alert);	
									}							
									//Write Query config;
									a2_write_queue_config(&m_logging_dev, &m_queue_type);
									at45dbxx_uninit();
									nrf_drv_a2_spi_deinit(&m_a2_spi_comm);
									memset(&m_a2_button_alert_log, 0x00, sizeof(m_a2_button_alert_log));
							}if(m_sem_cancel_sos == 1){
									m_a2_info.alert_bt_state = ALERT_BT_CHECK_WAIT_KRY;
									m_sem_cancel_sos = 0;
									m_a2_info.touch_pin_status = ALERT_CANCEL_DETECT;
									NRF_LOG_INFO("ALERT_TIMEOUT_CALCEL");
									NRF_LOG_FLUSH();
									advertising_normal();
									m_a2_info.emergency_status = BLE_AIDER_EVT_NORMAL;
									m_a2_button_alert_log.cancel_detect = BUTTON_CANCEL_DETECT;
									a2_spi_int();
									at45dbxx_init();
									//Read the last page to buffer;
									a2_read_button_alert_last_buffer( &m_logging_dev, &m_queue_type.button_alert);
									//Save alert to buf
									a2_save_button_alert_to_buf(&m_logging_dev, &m_queue_type.button_alert, &m_a2_button_alert_log);
									a2_write_button_alert_last_buffer(&m_logging_dev, &m_queue_type.button_alert);	
									NRF_LOG_INFO("Button Alert Save to buffer");
									//write buffer;
									NRF_LOG_INFO("Button Alert Save to Flash");
									a2_write_button_alert_log(&m_logging_dev, &m_queue_type.button_alert);							
									//Write Query config;
									a2_write_queue_config(&m_logging_dev, &m_queue_type);
									at45dbxx_uninit();
									nrf_drv_a2_spi_deinit(&m_a2_spi_comm);
									memset(&m_a2_button_alert_log, 0x00, sizeof(m_a2_button_alert_log));
							}else{
									m_a2_info.alert_bt_state = ALERT_BT_CHECK_CALCEL;	
									m_a2_sensor_config.sem_get_heart_rate = 1;
							}
							break;
					case ALERT_BT_CHECK_UNPRESS_CANCEL:
							if(nrf_gpio_pin_read(ALERT_BT_PIN) == 1)
							{
									NRF_LOG_INFO("ALERT_BT_CHECK_UNPRESS_CANCEL");
									NRF_LOG_FLUSH();
									m_a2_info.alert_bt_state = ALERT_BT_CHECK_WAIT_KRY;
							}else{
									m_a2_info.alert_bt_state = ALERT_BT_CHECK_UNPRESS_CANCEL;	
							}
							break;
							default:
									NRF_LOG_INFO("fuck");
				}
				
					switch(m_a2_info.left_bt_state)
					{
							case BT_CHECK_WAIT_KRY:
								if(nrf_gpio_pin_read(LEFT_BT_PIN) == 0)
								{
										m_a2_info.touch_pin_left_status = LEFT_BT_PUSH;
										m_a2_info.left_bt_state = ALERT_BT_CHECK_UNPRESS;
										NRF_LOG_INFO("Left Press");
								}
								break;
							case ALERT_BT_CHECK_UNPRESS:
								if(nrf_gpio_pin_read(LEFT_BT_PIN) == 1)
								{
										m_a2_info.touch_pin_left_status = NO_BT_PUSH;
										m_a2_info.left_bt_state = BT_CHECK_WAIT_KRY;
								}else{
										m_a2_info.touch_pin_left_status = BT_WAIT_UNPRESS;
										m_a2_info.left_bt_state = ALERT_BT_CHECK_UNPRESS;
								}
								break;
							default:
									m_a2_info.left_bt_state = BT_CHECK_WAIT_KRY;
					}
					
					
					switch(m_a2_info.right_bt_state)
					{
							case BT_CHECK_WAIT_KRY:
								if(nrf_gpio_pin_read(RIGHT_BT_PIN) == 0)
								{
										m_a2_info.touch_pin_right_status = RIGHT_BT_PUSH;
										m_a2_info.right_bt_state = ALERT_BT_CHECK_UNPRESS;
										NRF_LOG_INFO("Right Press");
								}
								break;
							case ALERT_BT_CHECK_UNPRESS:
								if(nrf_gpio_pin_read(RIGHT_BT_PIN) == 1)
								{
										m_a2_info.touch_pin_right_status = NO_BT_PUSH;
										m_a2_info.right_bt_state = BT_CHECK_WAIT_KRY;
								}else{
										m_a2_info.touch_pin_right_status = BT_WAIT_UNPRESS;
										m_a2_info.right_bt_state = ALERT_BT_CHECK_UNPRESS;
								}
								break;
							default:
									m_a2_info.right_bt_state = BT_CHECK_WAIT_KRY;
					}
				

		}

		
		switch(m_a2_info.usb_pin_state)
		{
			case USB_CHECK_WAIT_PLUG:
					if(nrf_gpio_pin_read(USB_PIN) == 1)
					{
							nrf_delay_ms(20);
							if(nrf_gpio_pin_read(USB_PIN) == 1)
							{
									m_a2_info.usb_pin_status = nrf_gpio_pin_read(USB_PIN);
									m_a2_info.usb_pin_state = ALERT_USB_CHECK_UNPLUG;
									NRF_LOG_INFO("Battery charge detect");
									m_a2_info.usb_charging_status = BATTERY_CHARGING_STATE;
							}
					}else{
							m_a2_info.usb_pin_status = 0;
							m_a2_info.usb_charging_status = USB_CHARGER_DO_NOT_CONNECT_STATE;
					}
					break;
			case ALERT_USB_CHECK_UNPLUG:
					if(nrf_gpio_pin_read(USB_PIN) == 1 && pre_time != m_a2_info.unix_time)
					{
							pre_time = m_a2_info.unix_time;
							if(charging_time_couter > COUNTER_TIME_FOR_CHARGE)
							{	
									NRF_LOG_INFO("FULL CHARGE");
									m_a2_info.usb_charging_status = BATTERY_FULLY_STATE;
							}else{
									charging_time_couter++;
									//NRF_LOG_INFO("Counter Charging %ld", charging_time_couter);
									m_a2_info.usb_charging_status = BATTERY_CHARGING_STATE;
							}
					}else if(nrf_gpio_pin_read(USB_PIN) == 0)
					{
							m_a2_info.usb_pin_status = 0;
							charging_time_couter = 0;
							m_a2_info.usb_pin_state = USB_CHECK_WAIT_PLUG;
							NRF_LOG_INFO("Unplug USE Detect");	
						  m_a2_info.usb_charging_status = USB_CHARGER_DO_NOT_CONNECT_STATE;
					}else{
							//No need implement
					}
					break;
			default:
						m_a2_info.usb_pin_state = USB_CHECK_WAIT_PLUG;
		}
}

static void a2_display_task2()
{
		static uint32_t tick = 0;
		static uint32_t pre_time = 0;
		static int8_t old_bt_status = NO_BT_PUSH; 
			
		switch(m_display_control)
		{
			case DISPLAY_WAIT_STATE:
				if(m_a2_info.free_fall_status == 1){
						m_display_control = DISPLAY_ON_STATE;
				}else if(( m_a2_info.touch_pin_left_status == LEFT_BT_PUSH || m_a2_info.touch_pin_right_status == RIGHT_BT_PUSH || m_a2_info.touch_pin_status == ALERT_BT_PUSH))
				{
						m_display_control = DISPLAY_ON_STATE;
				}else if(m_a2_info.hand_gesture_status == HAND_GESTURE_WATCH_TIME)
				{
						m_display_control = DISPLAY_ON_STATE;
						m_a2_info.hand_gesture_status = HAND_GESTURE_UNKNOW;
				}else{
						m_display_control = DISPLAY_WAIT_STATE;
				}
				break;				
			case DISPLAY_ON_STATE:
				//Wake up OLED
				nrf_drv_a2_i2c_0_init(&m_oled_comm);
				nrf_drv_a2_oled_init(&m_oled_comm, &m_oled_ssd1306_dev);	
				
				if(m_a2_info.batter_value < LOW_BATTERY_SOC_THRESHOLD) 
				{
						display_battery_low();
						nrf_delay_ms(1000);
				}
				
				if(m_a2_info.free_fall_status == 1)
				{
						m_display_control = DISPLAY_FREE_FALL_SOS_PAGE_STATE;
								
				}else{
						m_display_page    = DISPLAY_TIME_PAGE;		
						show_display();
						pre_time = m_a2_info.unix_time;	
						m_display_control = DISPLAY_COUNDOWN_STATE;
				}
				break;
			case DISPLAY_COUNDOWN_STATE:
				//Check if the count is less than max when go to shutdown oled state
				if(m_a2_info.free_fall_status == 1){
						m_display_control = DISPLAY_FREE_FALL_SOS_PAGE_STATE;
				}else if(m_a2_info.touch_pin_status == ALERT_LEVEL_3){
						m_display_control = DISPLAY_BT_ALERT_STATE;
						old_bt_status = NO_BT_PUSH;
				}else if(m_a2_info.touch_pin_left_status == LEFT_BT_PUSH)
				{	
						m_display_page--;
						if(m_display_page < DISPLAY_TIME_PAGE)
						{
								m_display_page = DISPLAY_HEART_RATE_PAGE;
						}			
						pre_time = m_a2_info.unix_time;
						show_display();
				}else if(m_a2_info.touch_pin_right_status == RIGHT_BT_PUSH)
				{
						m_display_page++;
						if(m_display_page > DISPLAY_HEART_RATE_PAGE)
						{
								m_display_page = DISPLAY_TIME_PAGE;
						}
						pre_time = m_a2_info.unix_time;
						show_display();		

				}else if(m_a2_sensor_config.sem_manual_recheck_hr == 1)
				{
						if( m_sem_hr_cal !=  HR_IN_PROGRESS )
						{
								//Show HR 
								m_a2_sensor_config.sem_manual_recheck_hr = 0;
								NRF_LOG_INFO("Show HR Result");
								m_display_control = DISPLAY_HEART_RATE_SHOW_RESULT_STAE;
								pre_time = m_a2_info.unix_time;
						}else{
								if(tick != m_a2_info.unix_time)
								{
										tick = m_a2_info.unix_time;
										if(m_flag_update_graph_hr == 1)
										{
												//show_display();
												display_heart_reate(DISPlAY_SHOW_HR_MES_VALUE);
										}else{
												display_heart_reate(DISPlAY_SHOW_SYMBOL_VALUE);
										}
								}
						}
				}else if(m_a2_info.usb_charging_status == USB_CHARGER_DO_NOT_CONNECT_STATE &&  m_display_page == DISPLAY_HEART_RATE_PAGE && (m_a2_info.unix_time - pre_time) >= MIN_TIME_TO_ENTER_HR_WOKR && m_a2_info.broken_max30120_status != true)
				{
						m_display_control = HEAERT_RATE_TASK;
				}
				else if(m_a2_info.unix_time - pre_time > MAX_COUNTER_DISPLAY_OFF )
				{
						m_display_control = DISPLAY_OFF_STATE;
				}
				else{
						m_display_control = DISPLAY_COUNDOWN_STATE;
						if(tick != m_a2_info.unix_time)
						{
								tick = m_a2_info.unix_time;
								show_display();
						}
				}				
				break;
			case DISPLAY_OFF_STATE:
				nrf_drv_a2_oled_shutdown();
				nrf_drv_a2_i2c_0_deinit(&m_oled_comm);
				m_display_control = DISPLAY_WAIT_STATE;
				break;

			case HEAERT_RATE_TASK:
					display_heart_reate(DISPlAY_SHOW_SYMBOL_VALUE);	
					m_a2_sensor_config.sem_get_heart_rate = 1;
					m_a2_sensor_config.sem_manual_recheck_hr = 1;
					m_sem_hr_cal = HR_IN_PROGRESS;
					pre_time = m_a2_info.unix_time;	
					m_display_control = DISPLAY_COUNDOWN_STATE;
					
					break;
			case DISPLAY_HEART_RATE_COUNTDOWN_STAE:
				//Do not implementation
				break;
			case DISPLAY_HEART_RATE_SHOW_RESULT_STAE:
				if((m_a2_info.unix_time - pre_time) > MAX_COUNTER_DISPLAY_HR_RS)
				{
						m_display_control = DISPLAY_O2_SHOW_RESULT_STAE;
						pre_time = m_a2_info.unix_time;
				}else{
						if(tick != m_a2_info.unix_time && m_a2_info.unix_time%2 == 0)
						{
								tick = m_a2_info.unix_time;
								display_heart_reate(DISPlAY_SHOW_HR_VALUE);
						}else if(tick != m_a2_info.unix_time){
								tick = m_a2_info.unix_time;
								//display_heart_reate(DISPlAY_DO_NOT_SHOW_HR_VALUE);
							  display_heart_reate(DISPlAY_SHOW_HR_VALUE);
						}		
				}
				break;
			case DISPLAY_O2_SHOW_RESULT_STAE:
				if((m_a2_info.unix_time - pre_time) > MAX_COUNTER_DISPLAY_O2_RS)
				{
						if(m_display_page != DISPLAY_HEART_RATE_PAGE)
						{
								m_display_control = DISPLAY_COUNDOWN_STATE;
						}else{
								m_display_control = DISPLAY_OFF_STATE;
						}
				}else{
						if(tick != m_a2_info.unix_time )
						{
								tick = m_a2_info.unix_time;
								display_o2();
						}		
				}
				break;
			case DISPLAY_BT_ALERT_STATE:
				
				if(m_a2_info.touch_pin_status == ALERT_LEVEL_3)
				{
						if(old_bt_status != ALERT_LEVEL_3 )
						{
								old_bt_status = ALERT_LEVEL_3;
								display_countdown_alert(3);
								tick = m_a2_info.unix_time;
								NRF_LOG_INFO("Display Alert Level 3");
								NRF_LOG_FLUSH();
						}							
				}else if(m_a2_info.touch_pin_status == ALERT_LEVEL_2)
				{
						if(old_bt_status != ALERT_LEVEL_2)
						{	
								old_bt_status = ALERT_LEVEL_2;
								display_countdown_alert(2);
								tick = m_a2_info.unix_time;
								NRF_LOG_INFO("Display Alert Level 2");
								NRF_LOG_FLUSH();
						}	
				}else if(m_a2_info.touch_pin_status == ALERT_LEVEL_1)
				{		
						if(old_bt_status != ALERT_LEVEL_1 )
						{
								old_bt_status = ALERT_LEVEL_1;
								display_countdown_alert(1);
								tick = m_a2_info.unix_time;
								NRF_LOG_INFO("Display Alert Level 1");
								NRF_LOG_FLUSH();
						}	
				}else if(m_a2_info.touch_pin_status == ALERT_LEVEL_0){
						if(old_bt_status != ALERT_LEVEL_0 )
						{
								old_bt_status = ALERT_LEVEL_0;
								display_countdown_alert(0);
								tick = m_a2_info.unix_time;
								NRF_LOG_INFO("Display Alert Level 0");
								NRF_LOG_FLUSH();
						}	
				}else if(m_a2_info.touch_pin_status == ALERT_SOS_DETECT){
						m_display_control = DISPLAY_BT_SOS_PAEG_STATE;
						NRF_LOG_INFO("SOS Start go to sos state");
						NRF_LOG_FLUSH();
						nrf_gpio_pin_set(VIBRATION_PIN);
						nrf_delay_ms(100);
						nrf_gpio_pin_clear(VIBRATION_PIN);
				}else{
						m_display_control = DISPLAY_COUNDOWN_STATE;
						pre_time = m_a2_info.unix_time;
						NRF_LOG_INFO("Unpress Alert Button");
						NRF_LOG_FLUSH();
						old_bt_status = NO_BT_PUSH;
						nrf_gpio_pin_clear(VIBRATION_PIN);
				}
				break;
			case DISPLAY_BT_SOS_PAEG_STATE:
				if(m_a2_info.touch_pin_status == ALERT_CANCEL_DETECT)
				{
						ssd1306_invert_display(0);
						NRF_LOG_INFO("Cancel Detect");
						NRF_LOG_FLUSH();
						m_display_control = DISPLAY_COUNDOWN_STATE;	
						m_display_page = DISPLAY_TIME_PAGE;
					  m_a2_info.alert_bt_state = ALERT_BT_CHECK_WAIT_KRY;
						pre_time = m_a2_info.unix_time;
						nrf_gpio_pin_clear(VIBRATION_PIN);
				}else{
						
						if(tick != m_a2_info.unix_time )
						{
								if(m_a2_info.unix_time%2 == 0)
								{
										nrf_gpio_pin_clear(VIBRATION_PIN);
										display_bt_alert(1);
								}else{
									  nrf_gpio_pin_set(VIBRATION_PIN);
										display_bt_alert(0);
								}
								tick = m_a2_info.unix_time;
								NRF_LOG_INFO("Display SOS");
								NRF_LOG_FLUSH();
						}
				}
				break;
			case DISPLAY_FREE_FALL_SOS_PAGE_STATE:
				if(m_a2_info.touch_pin_status == ALERT_CANCEL_DETECT)
				{
						ssd1306_invert_display(0);
						NRF_LOG_INFO("Cancel Detect");
						NRF_LOG_FLUSH();
						m_display_control = DISPLAY_COUNDOWN_STATE;
						m_display_page = DISPLAY_TIME_PAGE;
						m_a2_info.alert_bt_state = ALERT_BT_CHECK_WAIT_KRY;
						pre_time = m_a2_info.unix_time;
						nrf_gpio_pin_clear(VIBRATION_PIN);
						m_a2_info.free_fall_status = 0;
				}else{
						nrf_gpio_pin_set(VIBRATION_PIN);
						if(tick != m_a2_info.unix_time )
						{
								if(m_a2_info.unix_time%2 == 0)
								{
										display_fall_alert(1);
								}else{
										display_fall_alert(0);
								}
								tick = m_a2_info.unix_time;
								NRF_LOG_INFO("Display Free fall SOS");
								NRF_LOG_FLUSH();
						}
				}
				break;
			default:
				m_display_control = DISPLAY_WAIT_STATE;
				break;
		}
}


/**
 * @brief Handler for timer events.
 */
void timer_hr_event_handler(nrf_timer_event_t event_type, void* p_context)
{
    switch (event_type)
    {
        case NRF_TIMER_EVENT_COMPARE0:
            timer2_counter = timer2_counter+TIMER_CAPTURE_TIME;
            break;

        default:
            //Do nothing.
            break;
    }
}

void start_timer(void)
{		
		uint32_t time_ms = TIMER_CAPTURE_TIME; //Time(in miliseconds) between consecutive compare events.
    uint32_t time_ticks;
    uint32_t err_code = NRF_SUCCESS;
	
   //Configure TIMER_LED for generating simple light effect - leds on board will invert his state one after the other.
    nrf_drv_timer_config_t timer_cfg = NRF_DRV_TIMER_DEFAULT_CONFIG;
    err_code = nrf_drv_timer_init(&TIMER_HR, &timer_cfg, timer_hr_event_handler);
    APP_ERROR_CHECK(err_code);

    time_ticks = nrf_drv_timer_ms_to_ticks(&TIMER_HR, time_ms);

    nrf_drv_timer_extended_compare(
         &TIMER_HR, NRF_TIMER_CC_CHANNEL0, time_ticks, NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK, true);

    nrf_drv_timer_enable(&TIMER_HR);
	
}

void stop_timer(void)
{		
	  nrf_drv_timer_disable(&TIMER_HR);
		nrf_drv_timer_uninit(&TIMER_HR);
}

uint8_t kalman_filter(float beat_rate)
{
		uint8_t res_beats_rate;
		//Kalan constant
		const float kal_ohm  = 1.0;
		const float kal_q    = 10000.0;
		const float kal_h		 = 1.0;
		const float kal_r    = 30000.0;
		//Intial value
		static float x_minus = 78.0;
		static float p_minus = 10000.0;

		float kal_k = 0;
		float x_plus = 0;
		float p_plus = 0;
		
		kal_k  = p_minus*kal_h/((kal_h*kal_h*p_minus)+kal_r);
		x_plus = x_minus+(kal_k*(beat_rate-(kal_h*x_minus)));
		p_plus = (((1-kal_k*kal_h)*(1-kal_k*kal_h))*p_minus)+(kal_k*kal_k*kal_r);
		
		x_minus = kal_ohm*x_plus;
		p_minus = (kal_ohm*kal_ohm*p_plus)+kal_q;	
		
		res_beats_rate = (uint8_t)x_plus;
		
		return res_beats_rate;
}	

//#define DEBUG_HR
//#define DEBUG_SPO2

uint8_t heart_rate_monitoring2()
{
		static spo2_data_t m_spo2_data;
		static heart_rate_state_t hr_state = HR_INIT;	
		static btw_data_t m_btw_data;
		static uint8_t powerLevel = MAX_POWER_LED_OF_SENSOR_HR;//0x1F;//0x0F; 0x1F for Wasan
		static uint32_t buf_hr_sum = 0;
		static uint32_t buf_hr_count = 0;
		static uint8_t flag_out = 0;		
		static btw_data_t m_btw_ir_spo2;
		static btw_data_t m_btw_red_spo2;
		
	
	  hr_pkg_t hr_raw_data;	
		hr_data_t hr_data;
		spo2_value_t spo2_data;
	
		uint8_t sampleAverage = 2;
		uint8_t ledMode = 2;
		int sampleRate = 	400;
		int pulseWidth = 411;
		int adcRange = 8192;	
		uint32_t buf_hr_avr = 0;
		double max_spo2 = 0;
		
		heart_rate_res_t hr_res = HR_IN_PROGRESS;
	
		switch((uint8_t)hr_state)
		{
			case HR_INIT:
					m_hr_sample_counter = 0;
					spO2Calculator_init(&m_spo2_data);
					nrf_drv_a2_i2c_1_init(&m_max_tmp_comm);
					nrf_drv_max30102_reg_i2c_func(&m_max_tmp_comm, &m_max30102_dev);
					uint32_t res;
					res = max30102_initaial(&m_max30102_dev);
					nrf_delay_ms(200);
					if(res == MAX30102_SUCCESS)
					{
							NRF_LOG_INFO("[OK]\tMax30102 Already and background working is begin\n");
							max30102_setup2(&m_max30102_dev, powerLevel, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange);
							//max30102_setup(&m_max30102_dev);
							//setPulseAmplitudeRed(&m_max30102_dev, 30);
							hr_state = HR_MES;
						  start_timer();
							flag_out = 0;
						  rct_count = 0;
							m_hr_sample_counter = 0;
							m_flag_update_graph_hr = 0;
							powerLevel = MAX_POWER_LED_OF_SENSOR_HR;
							buf_hr_sum = 0;
						  buf_hr_count = 0;
					}else{
							NRF_LOG_INFO("[FALL]\tMax30102 not Found\n");
						  hr_state = HR_SENSOR_FAIL_HANDLER;
					}
					break;
			case HR_MES:
					max30102_check(&m_max30102_dev);
					while(max30102_available(&m_max30102_dev))
					{
							hr_raw_data.red_value = getFIFORed(&m_max30102_dev);
							hr_raw_data.ir_value  = getFIFOIR(&m_max30102_dev);
							
							int32_t sample_ir_dc_remover = ir_dc_remover(hr_raw_data.ir_value );
							int32_t sample_btw           = btw_fileter_100Hz_cf_1Hz_order2(&m_btw_data, sample_ir_dc_remover);
						  
							if(rct_count >= 3 && flag_out == 0 && m_hr_sample_counter < MAX_BUFFER_SENSOR_HR)
							{
									m_raw_ir_hr_buf[m_hr_sample_counter] = hr_raw_data.ir_value;
									m_raw_red_hr_buf[m_hr_sample_counter] = hr_raw_data.red_value;
									m_non_dc_ir_hr_buf[m_hr_sample_counter] = (float)sample_btw;
									m_non_dc_ir_spo2_buf[m_hr_sample_counter] = (float)sample_ir_dc_remover;
									m_hr_sample_counter++;
							}
							
							if(rct_count > 8)  //14)
							{
									flag_out = 1;
							}
							max30102_nextSample(&m_max30102_dev);
					}
					if(flag_out == 1)
					{
							hr_state = HR_CAL;
							NRF_LOG_INFO("[OK]\tHR sample count %ld", m_hr_sample_counter); 
					}	
					break;
			case HR_CAL:
			  heart_rate_cal_v3(&m_non_dc_ir_hr_buf[0], m_hr_sample_counter, &signal[0], &hr_data );
				hr_state = HR_ADJ_LED;
				
#ifdef DEBUG_HR					
				for(uint32_t jj=0;jj<m_hr_sample_counter;jj++)
				{
						NRF_LOG_RAW_INFO("\n\rhr,%lu,%lu,%d" ,jj ,m_raw_red_hr_buf[jj],m_raw_ir_hr_buf[jj] );
						NRF_LOG_RAW_INFO("," NRF_LOG_FLOAT_MARKER ",%d,%d" , NRF_LOG_FLOAT(m_non_dc_ir_hr_buf[jj]) , signal[jj], m_non_dc_ir_hr_buf_int[jj]);
						NRF_LOG_FLUSH();
						nrf_delay_ms(10);
				}
#endif
				
				if(hr_data.hr_avr > MIN_RANGE_VALUE_OF_HR && hr_data.hr_avr < MAX_RANGE_VALUE_OF_HR)
				{	
							for(uint32_t i=0;i<hr_data.hr_len;i++)
							{
									buf_hr_sum += hr_data.hr_data[i];
									buf_hr_count++;
							}
							buf_hr_avr = buf_hr_sum/buf_hr_count;
							
							m_a2_info.heart_rate = buf_hr_avr;
							
							NRF_LOG_INFO("^_^ Final HR %d ", m_a2_info.heart_rate);
							NRF_LOG_FLUSH();
							
						m_flag_update_graph_hr = 1;
				}
				
//				for(uint32_t b=0;b<m_hr_sample_counter;b++)
//				{
//						uint32_t sample_btw_ir_spo2 = btw_fileter_100Hz_cf_1Hz_order2(&m_btw_ir_spo2, m_raw_ir_hr_buf[b]);
//						uint32_t sample_btw_red_spo2 = btw_fileter_100Hz_cf_1Hz_order2(&m_btw_red_spo2, m_raw_red_hr_buf[b]);
//					
//						m_raw_ir_hr_buf[b] = sample_btw_ir_spo2;
//						m_raw_red_hr_buf[b] = sample_btw_red_spo2;
//						
//				}
				
				spo2_cal(&m_non_dc_ir_spo2_buf[0], &m_raw_ir_hr_buf[0] ,  &m_raw_red_hr_buf[0], m_hr_sample_counter, &signal[0], &spo2_data );	
				
				m_a2_info.pulse_oximeter = (uint8_t)max_spo2;
				
				if(spo2_data.avr != 0)
				{
						max_spo2 = spo2_data.avr;
						m_a2_info.pulse_oximeter = (uint8_t)max_spo2;
				}
				
#ifdef DEBUG_SPO2			
				for(uint32_t jj=0;jj<m_hr_sample_counter;jj++)
				{
						NRF_LOG_RAW_INFO("\n\rSPO2,%lu,%lu,%lu" ,jj ,m_raw_red_hr_buf[jj],m_raw_ir_hr_buf[jj] );
						NRF_LOG_RAW_INFO("," NRF_LOG_FLOAT_MARKER ",%d", NRF_LOG_FLOAT(m_non_dc_ir_spo2_buf[jj]),signal[jj]*20);
						NRF_LOG_FLUSH();
						nrf_delay_ms(10);
				}
				NRF_LOG_INFO("\n\r");
							
#endif				
				
				break;
			case HR_ADJ_LED:
					powerLevel = powerLevel - DIV_POWER_LED_OF_SENSOR_HR;
					NRF_LOG_INFO("[OK]\tLED Power %u", powerLevel);
					NRF_LOG_FLUSH();			
					
					max30102_setup2(&m_max30102_dev, powerLevel, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange);
					//setPulseAmplitudeRed(&m_max30102_dev, powerLevel);
					//setPulseAmplitudeIR(&m_max30102_dev, powerLevel);
		
					flag_out = 0;
					rct_count = 0;
					m_hr_sample_counter = 0;
				  if(powerLevel < MIN_POWER_LED_OF_SENSOR_HR)
					{
							hr_state = HR_DEINIT;	
					}else{
							hr_state = HR_MES;		
					}			
				  break;
			case HR_DEINIT:
				  stop_timer();
					NRF_LOG_INFO("[OK]\tHR Background stop");
					NRF_LOG_FLUSH();				
					nrf_drv_i2c_max30102_shutdown(&m_max_tmp_comm, &m_max30102_dev);	
					nrf_delay_ms(10);
					nrf_drv_a2_i2c_1_deinit(&m_max_tmp_comm);
					hr_state = HR_INIT;
			    hr_res = HR_SUCCESS;
					break;
			
			case HR_SENSOR_FAIL_HANDLER:
					NRF_LOG_RAW_INFO("[FAIL] HR Sensor is broken. HR Background stop\n");
					NRF_LOG_FLUSH();				
					nrf_drv_a2_i2c_1_deinit(&m_max_tmp_comm);
					hr_state = HR_INIT;
			    hr_res = HR_ERROR;
					break;
		}

		return hr_res;
}

void enter_to_low_power_mode(void)
{
		app_timer_stop(m_acc_gyro_timer_id);
		//m_a2_sensor_config.acc_en = 0;
		m_a2_sensor_config.heart_rate_en = 0;
		m_a2_sensor_config.temperature_en = 0;
		m_a2_sensor_config.acc_en = 0;
		m_a2_sensor_config.step_count_en = 0;
		m_a2_power_mode_cfg = A2_POWER_SAVE_MODE;
//		nrf_drv_a2_oled_shutdown();
//		nrf_drv_a2_i2c_0_deinit(&m_oled_comm);
		a2_spi_int();	
		nrf_drv_bmi160_power_and_cs_init(&m_a2_spi_comm);
	  pedometer_sleep();
		nrf_drv_a2_spi_deinit(&m_a2_spi_comm);
	  nrf_drv_bmi160_power_off(&m_a2_spi_comm);
		
}

void enter_to_ultra_power_save_mode(void)
{
	
		if(m_display_control != DISPLAY_WAIT_STATE)
		{
				ssd1306_clear_display();
				ssd1306_set_textsize(2);
				ssd1306_set_textcolor(WHITE);
				ssd1306_set_cursor(35,25);
				ssd1306_putstring("Out of   battery");
				ssd1306_display();

				nrf_delay_ms(10000);	

			
				//nrf_delay_ms(4000);	
					
				nrf_drv_a2_oled_shutdown();
				nrf_drv_a2_i2c_0_deinit(&m_oled_comm);
				m_display_control = DISPLAY_WAIT_STATE;
		}
		
		m_a2_sensor_config.heart_rate_en = 0;
		m_a2_sensor_config.temperature_en = 0;
		m_a2_sensor_config.acc_en = 0;
		m_a2_sensor_config.step_count_en = 0;
		m_a2_sensor_config.battery_en = 0;
		
		NRF_LOG_INFO("Enter Ultra Power Save Mode");	
		
		if(m_a2_power_mode_cfg == A2_POEWR_FULL_MODE	)
		{
				a2_spi_int();
				nrf_drv_bmi160_power_and_cs_init(&m_a2_spi_comm);
				pedometer_sleep();
				nrf_drv_a2_spi_deinit(&m_a2_spi_comm);
				app_timer_stop(m_acc_gyro_timer_id);
		}
		
		app_timer_stop(m_ibeacon_interval_timer_id);
		
		if(m_status_ble != BLE_CONNECTED)
		{
				sd_ble_gap_adv_stop();	
		}else{
				//Wait Implement.	
		}
			
		// SET for set using bypass mode
		nrf_gpio_cfg_output(A2_3V3_BYPASS_PIN);
		nrf_gpio_pin_set(A2_3V3_BYPASS_PIN);
		
		m_a2_power_mode_cfg = A2_ULTRA_POWER_SAVE_MODE;

	  //sleep_mode_enter();
}

void enter_to_power_full_mode(void)
{
		// SET for set using bypass mode
		nrf_gpio_cfg_output(A2_3V3_BYPASS_PIN);
		nrf_gpio_pin_clear(A2_3V3_BYPASS_PIN);
	
		m_a2_sensor_config.battery_en = 1;
		m_a2_sensor_config.heart_rate_en = 1;
		m_a2_sensor_config.temperature_en = 1;
		m_a2_sensor_config.acc_en = 1;
		m_a2_sensor_config.step_count_en = 1;
		
		m_display_control = DISPLAY_WAIT_STATE;
		NRF_LOG_INFO("Enter Full Mode");
		NRF_LOG_FLUSH();
		a2_spi_int();
		nrf_drv_bmi160_power_and_cs_init(&m_a2_spi_comm);
		pedometer_init();	
		nrf_drv_a2_spi_deinit(&m_a2_spi_comm);
		app_timer_start(m_acc_gyro_timer_id, ACC_GYRO_INTERVAL, NULL);
		
		if(m_a2_power_mode_cfg == A2_ULTRA_POWER_SAVE_MODE)
		{
				advertising_normal();
		}
	
		m_a2_power_mode_cfg = A2_POEWR_FULL_MODE;	

}

static uint8_t flag_set_fifo = 0;

/* Declare memory to store the raw FIFO buffer information */
static uint8_t fifo_buff[600];

/* Modify the FIFO buffer instance and link to the device instance */
struct bmi160_fifo_frame fifo_frame;


/* Declare instances of the sensor data structure to store the parsed FIFO data */
struct bmi160_sensor_data acc_data[84]; // 300 bytes / ~7bytes per frame ~ 42 data frames
uint8_t acc_frames_req = 84; 
uint8_t acc_index;
uint16_t index = 0;

/* An example to read the Gyro data in header mode along with sensor time (if available)
 * Configure the gyro sensor as prerequisite and follow the below example to read and
 * obtain the gyro data from FIFO */
int8_t fifo_acc_header_time_data(struct bmi160_dev *dev)
{
	uint16_t buf_nus[30];
	int8_t rslt = 0;
	
	fifo_frame.data = fifo_buff;
	fifo_frame.length = 600;
	dev->fifo = &fifo_frame;

	acc_frames_req = 10; 
	index = 0;
	
	if(flag_set_fifo == 0)
	{
			/* Configure the sensor's FIFO settings */
			rslt = bmi160_set_fifo_config(BMI160_FIFO_ACCEL ,
					BMI160_ENABLE, dev);
	    flag_set_fifo = 1;
		  //bmi160_set_fifo_down(BMI160_ACCEL_FIFO_DOWN_FOUR, &m_bmi160_dev);
	}
	
	rslt = BMI160_OK; 				
	if (rslt == BMI160_OK) {
		/* At ODR of 100 Hz ,1 frame gets updated in 1/100 = 0.01s
		i.e. for 42 frames we need 42 * 0.01 = 0.42s = 420ms delay */
		//dev->delay_ms(420); 
	
		/* Read data from the sensor's FIFO and store it the FIFO buffer,"fifo_buff" */
		//NRF_LOG_RAW_INFO("\n USER REQUESTED FIFO LENGTH : %d\n",dev->fifo->length);
		//NRF_LOG_FLUSH();
		rslt = bmi160_get_fifo_data(dev);

		if (rslt == BMI160_OK) {
			//NRF_LOG_INFO("\n AVAILABLE FIFO LENGTH : %d\n",dev->fifo->length);
			//NRF_LOG_FLUSH();
			/* Print the raw FIFO data */
			if( dev->fifo->length == 0)
			{
					return rslt;
			}
		
			/* Parse the FIFO data to extract gyro data from the FIFO buffer */
			//NRF_LOG_RAW_INFO("\n REQUESTED ACC DATA FRAMES : %d\n ",acc_frames_req);
			//NRF_LOG_FLUSH();
			rslt = bmi160_extract_accel(acc_data, &acc_frames_req, dev);

			if (rslt == BMI160_OK) {
				//NRF_LOG_RAW_INFO("\n AVAILABLE ACC DATA FRAMES : %d\n ",acc_frames_req);
				//NRF_LOG_FLUSH();
				/* Print the parsed gyro data from the FIFO buffer */
				for (acc_index = 0; acc_index < acc_frames_req; acc_index++) {
					//NRF_LOG_RAW_INFO("\nFIFO ACC FRAME[%d] ",acc_index);
					//NRF_LOG_FLUSH();
					//NRF_LOG_RAW_INFO(" ,Time: %d, ", acc_data[acc_index].sensortime);
					if(acc_data[acc_index].x != -31612)
					{
							NRF_LOG_RAW_INFO("\n %ld, %ld %ld"
							,acc_data[acc_index].x ,acc_data[acc_index].y
							,acc_data[acc_index].z);
							NRF_LOG_FLUSH();
							nrf_delay_ms(4);
							//len_msg = sprintf(buf_nus, "\n%ld,%ld,%ld",acc_data[acc_index].x ,acc_data[acc_index].y, acc_data[acc_index].z);
							buf_nus[0] = acc_data[acc_index].x;
							buf_nus[1] = acc_data[acc_index].y;
							buf_nus[2] = acc_data[acc_index].z;
							
							nus_sent_to_host((uint8_t*)buf_nus, 6);
					}
					
				}
				
//				if(acc_data[0].z == -31612)
//				{
//						for (index = 0; index < dev->fifo->length; index++) {
//							NRF_LOG_RAW_INFO("\n FIFO DATA INDEX[%d] = 0x%02X", index,
//							dev->fifo->data[index]);
//							NRF_LOG_FLUSH();
//						}
//				}
			
				/* Print the special FIFO frame data like sensortime */
				//NRF_LOG_RAW_INFO("\n SENSOR TIME DATA : %d \n",dev->fifo->sensor_time);
				//NRF_LOG_FLUSH();	
				//NRF_LOG_RAW_INFO("SKIPPED FRAME COUNT : %d",dev->fifo->skipped_frame_count);
				//NRF_LOG_FLUSH();
				//NRF_LOG_RAW_INFO("\n");
				//NRF_LOG_FLUSH();
			} else {
				NRF_LOG_RAW_INFO("\n Gyro data extraction failed");
			}
		} else {
			NRF_LOG_RAW_INFO("\n Reading FIFO data failed %d", rslt);
		}
	} else {
		NRF_LOG_RAW_INFO("\n Setting FIFO configuration failed");
	}

	return rslt;
}

int16_t m_acc_x_old;
uint32_t m_acc_no_motion_counter = 0;

a2_raw_acc_gyro_log_t p_data_log[15];
a2_raw_max30102_log_t p_data_log_hr[15];

void sensor_task()
{
		static a2_raw_acc_gyro_log_t	buf_a2_raw_acc_gyro_log;
		a2_raw_max30102_log_t buf_a2_raw_max30102_log;
		uint8_t res;
		struct bmi160_sensor_data gyro;
		uint16_t step_cnt;
		static uint16_t old_step_cnt;
		static uint32_t old_pre_time;
		uint16_t buf_period_step = 0;
		//double t_hr;
	
	
		if(m_a2_sensor_config.sem_get_acc_gyro_step_count == 1){
				//NRF_LOG_INFO("BMI Working...");
				buf_a2_raw_acc_gyro_log.unix_time =	m_a2_info.unix_time;
				a2_spi_int();
				nrf_drv_bmi160_power_and_cs_init(&m_a2_spi_comm);
				if(m_a2_sensor_config.acc_en == 1)
				{
						bmi160_get_sensor_data(BMI160_ACCEL_SEL, &m_a2_info.accel, NULL, &m_bmi160_dev);
						buf_a2_raw_acc_gyro_log.ax = m_a2_info.accel.x;
						buf_a2_raw_acc_gyro_log.ay = m_a2_info.accel.y; 
						buf_a2_raw_acc_gyro_log.az = m_a2_info.accel.z;
					  //NRF_LOG_INFO("Acc %d, %d, %d", m_a2_info.accel.x, m_a2_info.accel.y, m_a2_info.accel.z);
						if(m_display_control == DISPLAY_WAIT_STATE)
						{
								m_a2_info.hand_gesture_status = HAND_GESTURE_UNKNOW;// Diable -> hand_gesture_recognition_with_acc_process(m_a2_info.accel);
						}
					  
						if(m_display_control == DISPLAY_OFF_STATE && m_a2_sensor_config.sem_get_heart_rate != 1 && m_a2_sensor_config.sem_get_heart_rate != 1)
						{
								if(abs(m_a2_info.accel.x-m_acc_x_old) < m_a2_sensor_config.no_motion_acc_threshold)
								{
										if(m_a2_info.usb_pin_status != 1)
										{
												m_acc_no_motion_counter++;		
										}else{
												m_acc_no_motion_counter = 0;
										}							
									
										uint32_t timer_threshold = m_a2_sensor_config.no_motion_timer_threshold/m_a2_sensor_config.acc_gyro_step_count_interval;
										//	NRF_LOG_INFO("No Motion Counter %ld , timer_threshold max %d",m_acc_no_motion_counter,timer_threshold);
										if(m_acc_no_motion_counter > timer_threshold)
										{
												m_acc_no_motion_counter = 0;
												m_acc_x_old = m_a2_info.accel.x;
												//			NRF_LOG_INFO("No Motion Detect");
												enter_to_low_power_mode();
										}
								}else{
										m_acc_x_old = m_a2_info.accel.x;
										m_acc_no_motion_counter = 0;
								}
						}
						//fifo_acc_header_time_data(&m_bmi160_dev);
						//wait implement add data to ble stack	
				}
				if(m_a2_sensor_config.step_count_en == 1)
				{
						res = bmi160_read_step_counter(&step_cnt, &m_bmi160_dev);
						if(res == BMI160_OK){
								
						  	//NRF_LOG_INFO("Step %ld", step_cnt);
								m_a2_info.step_count = step_cnt;
							
								if(old_pre_time != m_a2_info.unix_time && m_a2_info.unix_time%PEDOMETER_PROCESS_PERIOED == 0)
								{
										buf_period_step = 0;
										if(old_step_cnt > m_a2_info.step_count)
										{
												buf_period_step = m_a2_info.step_count;
										}else{
												buf_period_step	= m_a2_info.step_count - old_step_cnt;
										}
										
										pedometer_processing(&m_pedometer_data, PEDOMETER_PROCESS_PERIOED, buf_period_step);

										m_a2_info.calories_in_d = m_a2_info.calories_in_d + m_pedometer_data.calories;
										m_a2_info.calories = (uint16_t)m_a2_info.calories_in_d;
										
										m_a2_info.distance_in_d = m_a2_info.distance_in_d+ m_pedometer_data.distance;
										m_a2_info.distance = (uint32_t)(m_a2_info.distance_in_d);			
										
										buf_a2_raw_acc_gyro_log.step_count 	= m_a2_info.step_count;
										buf_a2_raw_acc_gyro_log.cal 				=	m_a2_info.calories;
										buf_a2_raw_acc_gyro_log.distance 		= m_a2_info.distance;
										
										switch(m_pedometer_data.activity_level)
										{
											case REST:
												m_a2_info.rest_time += PEDOMETER_PROCESS_PERIOED;
												break;
											case WALKING:
												m_a2_info.walking_time += PEDOMETER_PROCESS_PERIOED;
												break;
											case JOGGING:
												m_a2_info.jogging_time += PEDOMETER_PROCESS_PERIOED;
												break;
											case RUNNING:
												m_a2_info.running_time += PEDOMETER_PROCESS_PERIOED;
												break;
										}
										m_a2_info.current_activity = m_pedometer_data.activity_level;
										old_step_cnt = m_a2_info.step_count; 
										old_pre_time = m_a2_info.unix_time;
								}
								
						}
						if(m_a2_config.a2_sensor_config.sem_reset_sensor_data == 1)
						{
								nrf_drv_bmi160_step_count_reset_value(&m_a2_spi_comm, &m_bmi160_dev);
								m_a2_config.a2_sensor_config.sem_reset_sensor_data = 0;
								NRF_LOG_INFO("SET to BMI for rest");
						}
				}
				if(m_a2_sensor_config.gyro_en == 1)
				{
						res = bmi160_get_sensor_data(BMI160_GYRO_SEL, &gyro, NULL, &m_bmi160_dev);
						if(res == BMI160_OK)
						{
								buf_a2_raw_acc_gyro_log.gx = gyro.x;
								buf_a2_raw_acc_gyro_log.gy = gyro.y;
								buf_a2_raw_acc_gyro_log.gz = gyro.z;
						}
				}
				nrf_drv_a2_spi_deinit(&m_a2_spi_comm);	
				m_a2_sensor_config.sem_get_acc_gyro_step_count = 0;
		}
		else if(m_a2_sensor_config.sem_get_battery_level == 1){
				NRF_LOG_INFO("BATT Reading...");
				adc_init();
				uint32_t err_code = nrf_drv_saadc_sample();
				APP_ERROR_CHECK(err_code);
				m_a2_sensor_config.sem_get_battery_level = 0;
		}
		else if(m_a2_sensor_config.sem_save_pedo_to_flash == 1){
				
				//NRF_LOG_INFO("Save %ld, %ld, %ld, %ld,", buf_a2_raw_acc_gyro_log.unix_time, buf_a2_raw_acc_gyro_log.ax, buf_a2_raw_acc_gyro_log.ay, buf_a2_raw_acc_gyro_log.az );
				//NRF_LOG_FLUSH();
				res = a2_save_pedo_meter_to_buf(&m_logging_dev, &m_queue_type.pedometer, &buf_a2_raw_acc_gyro_log);
				if(res != LOG_SUCCESS)
				{
						NRF_LOG_ERROR("Pedo can not save.");
				}
				
				if(a2_pedo_buf_is_full(&m_queue_type.pedometer) == QUE_TURE)
				{
						NRF_LOG_ERROR("Pedo write to flash.");
						a2_spi_int();
						at45dbxx_init();
						if(at45dbxx_is_available(&m_at45dbxx_dev) == AT45DB_SUCCESS)
						{							
								res = a2_write_pedo_meter_log(&m_logging_dev, &m_queue_type.pedometer);
								if(res == LOG_SUCCESS)
								{
										a2_write_queue_config(&m_logging_dev, &m_queue_type);	
								}else{
										NRF_LOG_ERROR("Pedo save to flash unsuccess");
								}
						}
						at45dbxx_uninit();
						nrf_drv_a2_spi_deinit(&m_a2_spi_comm);	
				}	
				m_a2_sensor_config.sem_save_pedo_to_flash = 0;		
		}else if(m_a2_sensor_config.sem_get_heart_rate == 1)
		{
				//NRF_LOG_INFO("HR Working...");
				m_sem_hr_cal = heart_rate_monitoring2(); 
				if(m_sem_hr_cal != HR_IN_PROGRESS)
				{
						m_a2_sensor_config.sem_get_heart_rate = 0;
						
						buf_a2_raw_max30102_log.unix_time 					= m_a2_info.unix_time;
						buf_a2_raw_max30102_log.hr				 					= m_a2_info.heart_rate;
						buf_a2_raw_max30102_log.spo2 								= m_a2_info.pulse_oximeter; 
						buf_a2_raw_max30102_log.temperature 				= m_a2_info.temperature;
						buf_a2_raw_max30102_log.skin_humdiy 				= 0;  //Feature use
						buf_a2_raw_max30102_log.population_variance = 0;  //Feature use 
						a2_save_hr_spo2_to_buf(&m_logging_dev, &m_queue_type.hr_spo2_temperature, &buf_a2_raw_max30102_log);
						if(a2_pedo_buf_is_full(&m_queue_type.hr_spo2_temperature) == QUE_TURE)
						{
								a2_spi_int();
								at45dbxx_init();
								if(at45dbxx_is_available(&m_at45dbxx_dev) == AT45DB_SUCCESS)
								{
										a2_write_hr_spo2_temp_log(&m_logging_dev, &m_queue_type.hr_spo2_temperature);
										a2_write_queue_config(&m_logging_dev, &m_queue_type);
								}
								at45dbxx_uninit();
								nrf_drv_a2_spi_deinit(&m_a2_spi_comm);
						}
						heart_rate_meas_timeout_handler();
						rr_interval_timeout_handler();
				}				
		}else if(m_a2_info.low_battery_detected == 1){
				enter_to_ultra_power_save_mode();		
				m_a2_info.low_battery_detected = 0;	
		}
		else{
				//NRF_LOG_INFO("BUG Working...");
				return;
		}
}

uint8_t a2_write_defualt_config(a2_sensor_config_t *p_config)
{
		return 0;
}

uint8_t a2_read_sensor_config(a2_sensor_config_t *p_config)
{
		return 0;
}

void in_pin_handler(uint8_t pin_no, uint8_t button_action)
{
		nrf_gpio_pin_set(VIBRATION_PIN);
		m_a2_info.free_fall_status = 1;	
}

#ifndef GPIO_ISR_DISABLE
//Configure 2 buttons with pullup and detection on low state
static const app_button_cfg_t app_buttons[1] = 
{
    {NRF_BMI160_INT2_PIN, false, BUTTON_PULL, in_pin_handler},
//		{22, false, BUTTON_PULL, button_event_handler},
};

/**
 * @brief Function for configuring: PIN_IN pin for input, PIN_OUT pin for output,
 * and configures GPIOTE to give an interrupt on pin change.
 */
static void gpio_init(void)
{
    ret_code_t err_code;

    nrf_drv_gpiote_in_event_enable(NRF_BMI160_INT2_PIN, true);
	
	  nrf_gpio_cfg_input(NRF_BMI160_INT2_PIN,NRF_GPIO_PIN_NOPULL);
		err_code = app_button_init((app_button_cfg_t *)app_buttons,
                                   2,
                                   APP_TIMER_TICKS(100));
    APP_ERROR_CHECK(err_code);
    err_code = app_button_enable();
    APP_ERROR_CHECK(err_code);
}
#endif

static void logging_init()
{
		m_logging_dev.delay = nrf_delay_ms;
		m_logging_dev.erase_chip = flash_erase_chip;
		m_logging_dev.erase_page = flash_erase_page;
		m_logging_dev.read	= flash_read;
		m_logging_dev.write	= flash_write;
		m_logging_dev.flash_wakeup = at45dbxx_init;
		m_logging_dev.flash_sleep  = at45dbxx_uninit;
		m_logging_dev.page_size	= PAGE_END;
		m_logging_dev.pn_begin = PAGE_BEGIN;
		m_logging_dev.pn_end = PAGE_END;
	
		a2_check_config(&m_logging_dev, &m_a2_config);
		memcpy((void*)&m_a2_sensor_config, (void*)&m_a2_config.a2_sensor_config, sizeof(m_a2_sensor_config));
	
		logging_queue_initial(&m_logging_dev, &m_queue_type);
	
}


void aider_a2_info_init(void)
{
		
		m_a2_info.unix_time = 1532532120;
		m_a2_info.time_s = 0;
		m_a2_info.time_m = 8;
		m_a2_info.time_h = 14;
		m_a2_info.distance = 120;
		m_a2_info.calories = 421;
		m_a2_info.heart_rate = 76;
		m_a2_info.pulse_oximeter = 98;
		m_display_control = DISPLAY_ON_STATE;
		m_display_page = DISPLAY_DUMMY_PAGE;
		m_status_ble = BLE_WAIT_CONNECT;
		
		// Setup input touch pin
		nrf_gpio_cfg_input(ALERT_BT_PIN, NRF_GPIO_PIN_NOPULL);
		nrf_gpio_cfg_input(LEFT_BT_PIN, NRF_GPIO_PIN_NOPULL);
		nrf_gpio_cfg_input(RIGHT_BT_PIN, NRF_GPIO_PIN_NOPULL);
		nrf_gpio_cfg_input(USB_PIN, NRF_GPIO_PIN_NOPULL);
		nrf_gpio_cfg_input(BATT_FULL_PIN, NRF_GPIO_PIN_NOPULL);
#ifndef GPIO_ISR_DISABLE
		gpio_init();	// If use gpio e. it using power 500uA 
#endif
		m_a2_power_mode_cfg = A2_POEWR_FULL_MODE;
		
		a2_reg_intial(&m_a2_reg);
}

void wakeup_flash_memory()
{
		a2_spi_int();
		at45dbxx_init();
}

void sleep_flash_memory()
{
		at45dbxx_uninit();
		nrf_drv_a2_spi_deinit(&m_a2_spi_comm);
}

void save_config_to_flash_memory()
{
		a2_spi_int();
		at45dbxx_init();
	  if(at45dbxx_is_available(&m_at45dbxx_dev) == AT45DB_SUCCESS)
		{
				a2_write_config(&m_logging_dev, &m_a2_config);
				at45dbxx_uninit();
		}
		nrf_drv_a2_spi_deinit(&m_a2_spi_comm);
	  memcpy((void*)&m_a2_sensor_config, (void*)&m_a2_config.a2_sensor_config, sizeof(m_a2_sensor_config));
}


void sync_nus_init()
{
		m_sync_nus_comm.write							= nus_sent_to_host;
		m_sync_nus_comm.delay_ms 				  = nrf_delay_ms;
		m_sync_nus_comm.save_cfg_to_flash = save_config_to_flash_memory;	
		m_sync_nus_comm.wakeup_flash			= wakeup_flash_memory;
		m_sync_nus_comm.sleep_flash       = sleep_flash_memory;
		m_sync_nus_comm.p_logging_dev			= &m_logging_dev;
		m_sync_nus_comm.p_queue     			= &m_queue_type;
		sync_nus_initial(&m_sync_nus_comm, &m_a2_reg);
}

a2_raw_acc_gyro_fall_log_t m_a2_raw_acc_gyro_fall_log;

void wdt_event_handler(void)
{
	  //save unixtime 
		NRF_LOG_INFO("WDT_____________________\n");
		NRF_LOG_FLUSH();
		nrf_delay_ms(300);
    
}

void wdt_init(void)
{
		uint32_t err_code = NRF_SUCCESS;
	
		nrf_drv_wdt_config_t config = NRF_DRV_WDT_DEAFULT_CONFIG;
    err_code = nrf_drv_wdt_init(&config, wdt_event_handler);
    APP_ERROR_CHECK(err_code);
    err_code = nrf_drv_wdt_channel_alloc(&m_channel_id);
    APP_ERROR_CHECK(err_code);
    nrf_drv_wdt_enable();
}

void check_setup_time_from_wdt_status()
{
		a2_raw_acc_gyro_log_t buf_data_log[20];
    uint16_t page_for_read;
	
		
		NRF_LOG_INFO("[FALL] Device restart detected. It require setup timer from flash memory\n" );
		NRF_LOG_FLUSH();
		nrf_delay_ms(10);
		a2_spi_int();
		at45dbxx_init();
		if(at45dbxx_is_available(&m_at45dbxx_dev) == AT45DB_SUCCESS)
		{		
				if(que_isEmpty(&m_queue_type.pedometer.q_page) == QUE_FALSE)
				{
						uint16_t buf_q_r;
						que_r(&m_queue_type.pedometer.q_page, &buf_q_r);
						
						if(buf_q_r != 0)
						{
								page_for_read = m_queue_type.pedometer.page_start + buf_q_r -1;   // -1 is current data page.
						}else{
								page_for_read = m_queue_type.pedometer.page_stop-1;							 // -1 is current data page.
						}
						
						NRF_LOG_INFO("[OK] Pedo data with pages %d\n",page_for_read);
						a2_read_pedo_meter_log_by_page(&m_logging_dev, page_for_read, &buf_data_log[0]);
						uint16_t array_size = (PAGE_SIZE/sizeof(a2_raw_acc_gyro_log_t));
						for(uint16_t i=0;i<array_size;i++)
						{
								if(buf_data_log[i].unix_time > DEPLOY_TIME && buf_data_log[i].unix_time != 0xFFFFFFFF)
								{
										m_a2_info.unix_time = buf_data_log[i].unix_time;
										NRF_LOG_ERROR("[OK] Unix Time in flash %ld", buf_data_log[i].unix_time);
										NRF_LOG_FLUSH();
								}else{
										NRF_LOG_ERROR("[FALL] Unix Time in flash %ld", buf_data_log[i].unix_time);
										NRF_LOG_FLUSH();
								}											
						}
				}
		}
		at45dbxx_uninit();
		nrf_drv_a2_spi_deinit(&m_a2_spi_comm);	
		
}

	/**@brief Function for initializing the clock.
 */
static void clock_init(void)
{
    //ret_code_t err_code = nrf_drv_clock_init();
    //APP_ERROR_CHECK(err_code);
	
		nrf_drv_clock_lfclk_request(NULL);
		
}

/**@brief Function for application main entry.
 */
int main(void)
{
    // Initialize.
	
		clock_init();

    log_init();

		NRF_LOG_INFO("***********  Application started  ***********\n");
		NRF_LOG_FLUSH(); 
    
	  m_reset_reason = NRF_POWER->RESETREAS;
		NRF_POWER->RESETREAS = 0xffffffff;
	
		if(m_reset_reason == 0x00000001) 
		{
				NRF_LOG_INFO("[FALL] NRF Powe ON 0x%04X.\n", m_reset_reason );
		}else{
				NRF_LOG_INFO("[OK] NRF Power ON 0x%04X. ", m_reset_reason);
		}
			
    timers_init();
    power_management_init();
    buttons_leds_init(&erase_bonds);
    ble_stack_init();
    peer_manager_init();
    gap_params_init();
    gatt_init();
    advertising_init();
    services_init();
    conn_params_init();
		
		sync_nus_init();
		
		ble_gap_addr_t m_address_1;
		sd_ble_gap_addr_get(&m_address_1);
	
    NRF_LOG_INFO("Read MAC:%02X%02X%02X%02X%02X%02X", m_address_1.addr[5],m_address_1.addr[4],m_address_1.addr[3],m_address_1.addr[2],m_address_1.addr[1],m_address_1.addr[0] );
		NRF_LOG_FLUSH();
    nrf_delay_ms(30);
	
	
		// Baeslab Intialize
		NRF_LOG_INFO("***********  Sensor initial ***********");
		NRF_LOG_FLUSH(); 
		a2_init_gpio();
		a2_spi_int();
		pedometer_init();		
		//pedometer_sleep();
		at45dbxx_init();
		logging_init();
		pedometer_setup(&m_pedometer_data , m_a2_config.a2_user_data_config.gender, m_a2_config.a2_user_data_config.weight, m_a2_config.a2_user_data_config.height);
		at45dbxx_uninit();
		nrf_drv_a2_spi_deinit(&m_a2_spi_comm);
		NRF_LOG_FLUSH();
		oled_init();
		max30102_tmp007_init();
		NRF_LOG_INFO("********** End sensor initial *********\n");
		NRF_LOG_FLUSH();
		nrf_delay_ms(1000);
		
	  rtc_config();	

    // Start execution.
    application_timers_start();
    advertising_start(erase_bonds);
		aider_a2_info_init();
		m_a2_info.unix_time = DEPLOY_TIME;
		m_display_control = DISPLAY_WAIT_STATE;
		
	  adc_init();
		uint32_t err_code = nrf_drv_saadc_sample();
		APP_ERROR_CHECK(err_code);				
		
		sd_ble_gap_tx_power_set(4);
		
		for (;;)
    {
				    
        if (NRF_LOG_PROCESS() == false && m_a2_sensor_config.sem_get_heart_rate != 1)
        {
            nrf_pwr_mgmt_run();
        }		
				
				a2_logic_task();
				a2_display_task2();
	
    } 
}

