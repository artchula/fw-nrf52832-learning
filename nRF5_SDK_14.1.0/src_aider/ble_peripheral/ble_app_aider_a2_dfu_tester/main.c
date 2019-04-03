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

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

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

//Baeslab Include File
#include "a2_typedef.h"
#include "nrf_drv_spi_bmi_flash.h"
#include "bmi160.h"
#include "at45db081e.h"
#include "nrf_drv_ssd1306.h"
#include "MAX30102.h"
#include "nrf_drv_i2c_max_temp.h"
#include "a2_display_mngr.h"


#define A2_3V3_BYPASS_PIN								26
#define OLED_POWER_EN_PIN						   	3
//#define OLED_POWER_EN_PIN						   	21
#define OLED_SCL_PIN								   	6
#define OLED_SDA_PIN								   	7
#define NRF_BMI160_POWER_EN_PIN					17
#define NRF_SPI_SCK_PIN									18
#define NRF_SPI_MOSI_PIN								19
#define NRF_SPI_MISO_PIN								20
#define NRF_SPI_BMI160_CS_PIN						22
#define NRF_SPI_FLASH_CS_PIN						13
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

#define APP_FEATURE_NOT_SUPPORTED       BLE_GATT_STATUS_ATTERR_APP_BEGIN + 2        /**< Reply when unsupported features are requested. */

#define DEVICE_NAME                     "Aider A2 TESTER V6"                                  /**< Name of device. Will be included in the advertising data. */
#define MANUFACTURER_NAME               "NordicSemiconductor"                       /**< Manufacturer. Will be passed to Device Information Service. */
#define APP_ADV_INTERVAL                300                                        /**< The advertising interval (in units of 0.625 ms. This value corresponds to 187.5 ms). */
#define APP_ADV_TIMEOUT_IN_SECONDS      0                                           /**< The advertising timeout in units of seconds. */

#define APP_BLE_OBSERVER_PRIO           3                                           /**< Application's BLE observer priority. You shouldn't need to modify this value. */
#define APP_BLE_CONN_CFG_TAG            1                                           /**< A tag identifying the SoftDevice BLE configuration. */

#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(100, UNIT_1_25_MS)            /**< Minimum acceptable connection interval (0.1 seconds). */
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(200, UNIT_1_25_MS)            /**< Maximum acceptable connection interval (0.2 second). */
#define SLAVE_LATENCY                   0                                           /**< Slave latency. */
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(4000, UNIT_10_MS)             /**< Connection supervisory timeout (4 seconds). */

#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(5000)                       /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(30000)                      /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
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

// Global Variable for bmi160 and Flash Memory 
static nrf_drv_a2_spi_comm_t  					m_a2_spi_comm;
struct bmi160_dev 						          m_bmi160_dev;
struct bmi160_sensor_data 		          m_accel;
struct bmi160_sensor_data 		          m_gyro;

static at45dbxx_dev_t										m_at45dbxx_dev;

const  nrf_drv_rtc_t rtc = 						  NRF_DRV_RTC_INSTANCE(2); /**< Declaring an instance of nrf_drv_rtc for RTC0. */

static nrf_drv_a2_i2c_0_comm_t 	        m_oled_comm;
static oled_ssd1306_dev_t 			        m_oled_ssd1306_dev;			

static nrf_drv_a2_max_tmp_comm_t				m_max_tmp_comm;
static max30102_dev_t										m_max30102_dev;
static tmp007_dev_t											m_tmp007_dev;

static display_page_t										m_display_page;


static a2_info_t 												a2_info;
static nrf_saadc_value_t 								adc_buf[2];
static a2_testing_t											m_peripheral_test;
static uint8_t 													flag_hr_temp_test = 0;
static uint8_t													hr_timer_out_count = 0;
static uint8_t													flag_hr_temp_error = 0;

NRF_BLE_GATT_DEF(m_gatt);              	/**< Advertising module instance. */                                             /**< GATT module instance. */
BLE_ADVERTISING_DEF(m_advertising);     
static uint16_t m_conn_handle = BLE_CONN_HANDLE_INVALID;                            /**< Handle of the current connection. */
static void advertising_start(bool erase_bonds);                                    /**< Forward declaration of advertising start function */
static void adc_init(void);			 
static void adc_uninit(void);

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
    return true;
}

//lint -esym(528, m_app_shutdown_handler)
/**@brief Register application shutdown handler with priority 0.
 */
NRF_PWR_MGMT_HANDLER_REGISTER(app_shutdown_handler, 0);


// YOUR_JOB: Use UUIDs for service(s) used in your application.
static ble_uuid_t m_adv_uuids[] = {{BLE_UUID_DEVICE_INFORMATION_SERVICE, BLE_UUID_TYPE_BLE}};


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
            // YOUR_JOB: Disconnect all bonded devices that currently are connected.
            //           This is required to receive a service changed indication
            //           on bootup after a successful (or aborted) Device Firmware Update.
            break;

        case BLE_DFU_EVT_BOOTLOADER_ENTER:
            // YOUR_JOB: Write app-specific unwritten data to FLASH, control finalization of this
            //           by delaying reset by reporting false in app_shutdown_handler
            NRF_LOG_INFO("Device will enter bootloader mode.");
            break;

        case BLE_DFU_EVT_BOOTLOADER_ENTER_FAILED:
            NRF_LOG_ERROR("Request to enter bootloader mode failed asynchroneously.");
            // YOUR_JOB: Take corrective measures to resolve the issue
            //           like calling APP_ERROR_CHECK to reset the device.
            break;

        case BLE_DFU_EVT_RESPONSE_SEND_ERROR:
            NRF_LOG_ERROR("Request to send a response to client failed.");
            // YOUR_JOB: Take corrective measures to resolve the issue
            //           like calling APP_ERROR_CHECK to reset the device.
            APP_ERROR_CHECK(false);
            break;

        default:
            NRF_LOG_ERROR("Unknown event from ble_dfu_buttonless.");
            break;
    }
}


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

    /* YOUR_JOB: Create any timers to be used by the application.
                 Below is an example of how to create a timer.
                 For every new timer needed, increase the value of the macro APP_TIMER_MAX_TIMERS by
                 one.
       uint32_t err_code;
       err_code = app_timer_create(&m_app_timer_id, APP_TIMER_MODE_REPEATED, timer_timeout_handler);
       APP_ERROR_CHECK(err_code); */
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

    /* YOUR_JOB: Use an appearance value matching the application's use case.
       err_code = sd_ble_gap_appearance_set(BLE_APPEARANCE_);
       APP_ERROR_CHECK(err_code); */

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling the YYY Service events.
 * YOUR_JOB implement a service handler function depending on the event the service you are using can generate
 *
 * @details This function will be called for all YY Service events which are passed to
 *          the application.
 *
 * @param[in]   p_yy_service   YY Service structure.
 * @param[in]   p_evt          Event received from the YY Service.
 *
 *
   static void on_yys_evt(ble_yy_service_t     * p_yy_service,
                          ble_yy_service_evt_t * p_evt)
   {
    switch (p_evt->evt_type)
    {
        case BLE_YY_NAME_EVT_WRITE:
            APPL_LOG("[APPL]: charact written with value %s. ", p_evt->params.char_xx.value.p_str);
            break;

        default:
            // No implementation needed.
            break;
    }
   }*/


/**@brief Function for initializing services that will be used by the application.
 */
static void services_init(void)
{
    uint32_t err_code;
    ble_dfu_buttonless_init_t dfus_init =
    {
        .evt_handler = ble_dfu_evt_handler
    };

    // Initialize the async SVCI interface to bootloader.
    err_code = ble_dfu_buttonless_async_svci_init();
    APP_ERROR_CHECK(err_code);


    err_code = ble_dfu_buttonless_init(&dfus_init);
    APP_ERROR_CHECK(err_code);

    /* YOUR_JOB: Add code to initialize the services used by the application.
       uint32_t                           err_code;
       ble_xxs_init_t                     xxs_init;
       ble_yys_init_t                     yys_init;

       // Initialize XXX Service.
       memset(&xxs_init, 0, sizeof(xxs_init));

       xxs_init.evt_handler                = NULL;
       xxs_init.is_xxx_notify_supported    = true;
       xxs_init.ble_xx_initial_value.level = 100;

       err_code = ble_bas_init(&m_xxs, &xxs_init);
       APP_ERROR_CHECK(err_code);

       // Initialize YYY Service.
       memset(&yys_init, 0, sizeof(yys_init));
       yys_init.evt_handler                  = on_yys_evt;
       yys_init.ble_yy_initial_value.counter = 0;

       err_code = ble_yy_service_init(&yys_init, &yy_init);
       APP_ERROR_CHECK(err_code);
     */
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
    cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
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
    /* YOUR_JOB: Start your timers. below is an example of how to start a timer.
       uint32_t err_code;
       err_code = app_timer_start(m_app_timer_id, TIMER_INTERVAL, NULL);
       APP_ERROR_CHECK(err_code); */
}


/**@brief Function for putting the chip into sleep mode.
 *
 * @note This function will not return.
 */
static void sleep_mode_enter(void)
{
    uint32_t err_code = bsp_indication_set(BSP_INDICATE_IDLE);

    APP_ERROR_CHECK(err_code);

    // Prepare wakeup buttons.
    err_code = bsp_btn_ble_sleep_mode_prepare();
    APP_ERROR_CHECK(err_code);

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
        case BLE_ADV_EVT_FAST:
            err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_ADV_EVT_IDLE:
            sleep_mode_enter();
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
            // LED indication will be changed when advertising starts.
            break;

        case BLE_GAP_EVT_CONNECTED:
            err_code = bsp_indication_set(BSP_INDICATE_CONNECTED);
            APP_ERROR_CHECK(err_code);
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
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
            break;

        case BLE_GATTS_EVT_TIMEOUT:
            // Disconnect on GATT Server timeout event.
            NRF_LOG_DEBUG("GATT Server Timeout.");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
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


/**@brief Function for the Power manager.
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
    ret_code_t err_code = nrf_ble_gatt_init(&m_gatt, NULL);
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

        NRF_LOG_DEBUG("advertising is started");
    }
}

static void power_management_init(void)
{
    uint32_t err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);
}

void saadc_event_handler(nrf_drv_saadc_evt_t const * p_event)
{
    if (p_event->type == NRF_DRV_SAADC_EVT_DONE)
    {
        nrf_saadc_value_t adc_result;
        uint16_t          batt_lvl_in_milli_volts;
        uint8_t           percentage_batt_lvl;
        uint32_t          err_code;

        adc_result = p_event->data.done.p_buffer[0];

				NRF_LOG_INFO("ADC %d\n\r", adc_result);
				
				a2_info.batter_value = adc_result;
			
        err_code = nrf_drv_saadc_buffer_convert(p_event->data.done.p_buffer, 1);
        APP_ERROR_CHECK(err_code);
			
				adc_uninit();
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



/**@brief Function for set gpio pin of aider a2 entry.
 */
static void aider_update_time()
{
		a2_info.unix_time++;
	
		a2_info.time_s++;
		if(a2_info.time_s>59)
		{
				a2_info.time_s = 0;
				a2_info.time_m++;
				if(a2_info.time_m>59)
				{
						a2_info.time_m = 0;
						a2_info.time_h++;
						if(a2_info.time_h>23)
						{
								a2_info.time_h = 0;
						}
				}
		}
}

/** @brief: Function for handling the RTC0 interrupts.
 * Triggered on TICK and COMPARE0 match.
 */
static void rtc_handler(nrf_drv_rtc_int_type_t int_type)
{
		uint32_t err_code;
		if(int_type == NRF_DRV_RTC_INT_COMPARE0)
		{
				
				//nrf_gpio_pin_toggle(COMPARE_EVENT_OUTPUT);
			  err_code = nrf_drv_rtc_cc_set(&rtc,0,COMPARE_COUNTERTIME * 8,true);
				APP_ERROR_CHECK(err_code);
				nrf_drv_rtc_counter_clear(&rtc);
			
				aider_update_time();
				//NRF_LOG_INFO("RTC %d\n", a2_info.unix_time);
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
		nrf_gpio_pin_set(A2_3V3_BYPASS_PIN);
	
		// SET for Disable Power 3V3 to BMI160
		nrf_gpio_cfg_output(NRF_BMI160_POWER_EN_PIN);
		nrf_gpio_pin_set(NRF_BMI160_POWER_EN_PIN);	
		
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
		nrf_gpio_pin_set(HEAET_RATE_EN_1V8_PIN);	
		
		
		
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
		nrf_drv_a2_spi_init(&m_a2_spi_comm);	
}



/**@brief Function for initial bmi160 pedometer, acceleration  and gyrometer
 */
void pedometer_init()
{
		uint32_t res;
		NRF_LOG_INFO("BMI160 Test \n");
	
	  res = nrf_drv_bmi160_spi_int(&m_a2_spi_comm, &m_bmi160_dev);
	  
		
		if(res == NRF_SUCCESS)
		{
				NRF_LOG_INFO("BMI160 >> PASS\n");
				m_peripheral_test.acc = 1;
		}else{
				NRF_LOG_INFO("BMI160 >> FAIL");
				m_peripheral_test.acc = 0;
		}
	
		res = nrf_drv_bmi160_acc_config(&m_a2_spi_comm, &m_bmi160_dev);

		nrf_drv_bmi160_step_count_conf(&m_a2_spi_comm, &m_bmi160_dev);
		NRF_LOG_RAW_INFO("\n\n");
}

/**@brief Function for initial at45dbxx flash memory.
 */
void at45dbxx_init()
{
		NRF_LOG_INFO("Flash Memory Test \n");
		nrf_drv_flash_spi_init(&m_a2_spi_comm, &m_at45dbxx_dev);
	  //nrf_delay_ms(1);
		at45dbxx_wakeup_pin(&m_at45dbxx_dev);
		//nrf_delay_ms(50);
		at45dbxx_resume_deep_power_down(&m_at45dbxx_dev);
	  //nrf_delay_ms(50);
		
		if(at45dbxx_is_available(&m_at45dbxx_dev) == AT45DB_SUCCESS)
		{
				NRF_LOG_INFO("Flash Memory >> PASS");
				m_peripheral_test.flash = 1;
		}else{
				NRF_LOG_INFO("Flash Memory >> FAIL");
				m_peripheral_test.flash = 0;
		}
	
		at34dbxx_deep_power_down(&m_at45dbxx_dev);
		//at45dbxx_ultra_deep_power_down(&m_at45dbxx_dev);
	  NRF_LOG_RAW_INFO("\n\n");
}

/**@brief Function for initial ssd1306 oled 0.96 inch.
 */
void oled_init(void)
{
		uint8_t ii;
		uint8_t addr;
		uint8_t reg = 0;
		uint8_t i2c_buf;
	
		m_oled_comm.oled_power_en_pin = OLED_POWER_EN_PIN;
		m_oled_comm.scl_pin 					= OLED_SCL_PIN;
		m_oled_comm.sda_pin 					= OLED_SDA_PIN;
	
		nrf_drv_a2_i2c_0_init(&m_oled_comm);
		nrf_drv_a2_oled_init(&m_oled_comm, &m_oled_ssd1306_dev);		
//		ssd1306_clear_display();
//		ssd1306_set_textsize(2);
//		ssd1306_set_textcolor(WHITE);
//		ssd1306_set_cursor(20,25);
//		ssd1306_putstring("Baeslab");
//		ssd1306_display();
//		nrf_delay_ms(1000);

	
		NRF_LOG_INFO("OLED and Cryto Test\n",addr);
		m_peripheral_test.oled = 0;
		m_peripheral_test.ecc = 0;	
		for(ii=0;ii<127;ii++)
		{
				addr = ii;
				if( nrf_drv_i2c_0_read(addr,reg,&i2c_buf,1) == NRF_SUCCESS)
				{
						NRF_LOG_INFO("Found Device 0x%02X\n",addr);
						if(addr == 0x3C)
						{
								m_peripheral_test.oled = 1;
						}
						if(addr == 0xC0)
						{
								m_peripheral_test.ecc = 1;	
						}
				}
		}
		
		if(m_peripheral_test.oled == 1)
		{
				NRF_LOG_INFO("OLED >> PASS\n");
		}else{
				NRF_LOG_INFO("OLED >> FAIL\n");
		}
		
		if(m_peripheral_test.ecc == 0)
		{
				NRF_LOG_INFO("ECC >> PASS\n");
		}else{
				NRF_LOG_INFO("ECC >> FAIL\n");
		}
		if(flag_hr_temp_error != 1)
		{
				nrf_drv_a2_oled_shutdown();
				nrf_drv_a2_i2c_0_deinit(&m_oled_comm);
				m_peripheral_test.oled = 1;
		}
		NRF_LOG_RAW_INFO("\n\n",addr);
}


/**@brief Function for application main entry.
 */
void max30102_tmp007_init()
{
		uint8_t res;
		m_max_tmp_comm.scl_pin 										= MAX_TMP_SCL_PIN;
		m_max_tmp_comm.sda_pin 										= MAX_TMP_SDA_PIN;
		m_max_tmp_comm.en_power_1V8_max30102_pin 	= HEAET_RATE_EN_1V8_PIN;
		m_max_tmp_comm.isr_max30102_pin						= HEAET_RATE_INT_PIN;
		m_max_tmp_comm.isr_temp007_pin						= TMP007_INT_PIN;
	
	  NRF_LOG_INFO("Max30102 and Tmp007 Test\n");
		nrf_drv_a2_i2c_1_init(&m_max_tmp_comm);
		nrf_drv_max30102_reg_i2c_func(&m_max_tmp_comm, &m_max30102_dev);
		flag_hr_temp_test = 1;
		nrf_delay_ms(200);
		res = max30102_initaial(&m_max30102_dev);
		if(res == MAX30102_SUCCESS)
		{		
				NRF_LOG_INFO("Max30102 >> PASS\n");
				m_peripheral_test.max30102 = 1;
		}else{
				NRF_LOG_INFO("Max30102 >> FAIL\n");
				m_peripheral_test.max30102 = 0;
		}
		
		max30102_setup(&m_max30102_dev);

		nrf_delay_ms(500);
		nrf_drv_i2c_max30102_shutdown(&m_max_tmp_comm, &m_max30102_dev);	


//		nrf_drv_temp007_reg_i2c_func(&m_max_tmp_comm, &m_tmp007_dev);
//		res = tmp007_initial(&m_tmp007_dev);

//		if(res == TMP007_SUCCESS)
//		{
//				m_peripheral_test.tmp007 = 1;
//				NRF_LOG_INFO("TMP007 >> PASS\n");
//				double die_temp = tmp007_readDieTempC(&m_tmp007_dev);

//				uint16_t h_point = (uint16_t)die_temp;
//				uint16_t l_point = (uint16_t)(die_temp*100) % 100;


//				double obj_temp = tmp007_readObjTempC(&m_tmp007_dev);

//				uint16_t h_point2 = (uint16_t)obj_temp;
//				uint16_t l_point2 = (uint16_t)(obj_temp*100) % 100;


//				NRF_LOG_INFO("TMP007 DIE Temperature %d.%02d , OBJ Temperature %d.%02d\r\n", h_point, l_point,h_point2, l_point2);
//				NRF_LOG_FLUSH();	

//				//while(1)
//				{
//						//uint32_t res = tmp007_power_down(&m_tmp007_dev);
//						//nrf_delay_ms(1000);
//				}

//				//NRF_LOG_INFO("TMP007 Device Not Found ");

//		}else{
//				NRF_LOG_INFO("TMP007 >> Fail");
//				m_peripheral_test.tmp007 = 0;
//		}

		nrf_drv_a2_i2c_1_deinit(&m_max_tmp_comm);	
		NRF_LOG_RAW_INFO("\n\n");
		
		
}

/**@brief Function for application main entry.
 */
void display_testing()
{
	
		ssd1306_clear_display();
		ssd1306_set_textsize(1);
		ssd1306_set_textcolor(WHITE);
		ssd1306_set_cursor(0,0);
		ssd1306_putstring("OLED         > pass\n");
		if(m_peripheral_test.acc == 1)
		{
				ssd1306_putstring("Pedometer   >  pass\n");	
		}else{
				ssd1306_putstring("Pedometer   > Fail\n");
		}
		
//		if(m_peripheral_test.tmp007 == 1)
//		{
//				ssd1306_putstring("Temperature >  pass\n");
//		}else{
//				ssd1306_putstring("Temperature > Fail\n");
//		}
		
		if(m_peripheral_test.flash == 1)
		{
				ssd1306_putstring("Flash Memory>  pass\n");
		}else{
				ssd1306_putstring("Flash Memory> Fail\n");
		}
		
		if(m_peripheral_test.max30102 == 1)
		{
				ssd1306_putstring("Heart Rate  >  pass\n");
		}else{
				ssd1306_putstring("Heart Rate  > Fail\n");
		}
	  
		if(m_peripheral_test.ecc == 1)
		{
				ssd1306_putstring("Cryto Engin >  pass\n");
		}else{
				ssd1306_putstring("Cryto Engin > Fail\n");
		}
		
		nrf_gpio_cfg_input(USB_PIN, NRF_GPIO_PIN_NOPULL);
		if(nrf_gpio_pin_read(USB_PIN) == 1)
		{
				ssd1306_putstring("USB Charging >  true\n");
		}else{
				ssd1306_putstring("USB Charging >  false\n");
		}
		
		ssd1306_display();
		//nrf_delay_ms(2000);
		//nrf_drv_a2_oled_shutdown();
		//nrf_drv_a2_i2c_0_deinit(&m_oled_comm);
}

typedef enum{
		BATTERY_LEVEL_0,
		BATTERY_LEVEL_1,
		BATTERY_LEVEL_2,
		BATTERY_LEVEL_3,
		BATTERY_LEVEL_4,
		BATTERY_FULL,
}battery_level_state;


/**@brief Function for application main entry.
 */
int main(void)
{
    bool erase_bonds;
		uint8_t flag_display_on = 0;
    // Initialize.
    log_init();

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

    NRF_LOG_INFO("Application started\n");

		// Baeslab Intialize
		memset(&m_peripheral_test,0x00,sizeof(m_peripheral_test));
		rtc_config();
		a2_init_gpio();
		oled_init();
		//a2_spi_int();
		//pedometer_init();	
		//at45dbxx_init();
		//nrf_drv_a2_spi_deinit(&m_a2_spi_comm);
		
		//max30102_tmp007_init();

		a2_info.unix_time = 7;
		a2_info.time_s = 0;
		a2_info.time_m = 25;
		a2_info.time_h = 14;
		
		// Enter main loop.
		//adc_init();
		//uint32_t err_code = nrf_drv_saadc_sample();
		//APP_ERROR_CHECK(err_code);
		//nrf_delay_ms(100);
		
		m_display_page = TIME_PAGE;
		
		//nrf_drv_a2_i2c_0_init(&m_oled_comm);
		//nrf_drv_a2_oled_init(&m_oled_comm, &m_oled_ssd1306_dev);		
		//nrf_delay_ms(200);
		
		//display_testing();
		
		// Setup input touch pin
		nrf_gpio_cfg_input(ALERT_BT_PIN, NRF_GPIO_PIN_PULLUP);
		nrf_gpio_cfg_input(LEFT_BT_PIN, NRF_GPIO_PIN_PULLUP);
		nrf_gpio_cfg_input(RIGHT_BT_PIN, NRF_GPIO_PIN_PULLUP);
		
		m_display_page = TIME_PAGE;
		
    // Start execution.
    //application_timers_start();
    advertising_start(erase_bonds);

	  unsigned short test = 10;
	 
		NRF_LOG_INFO("Testing Program Start");
		NRF_LOG_FLUSH();
		
//    Enter main loop.
//		while(a2_info.unix_time != 10)
//		{
//				//nrf_delay_ms(10);
//			  nrf_pwr_mgmt_run();
//		}
		
		NRF_LOG_INFO("Begin test %d", a2_info.unix_time);
		NRF_LOG_FLUSH();
		
		a2_spi_int();
		pedometer_init();	
		at45dbxx_init();
		nrf_drv_a2_spi_deinit(&m_a2_spi_comm);
		max30102_tmp007_init();
		
	  nrf_drv_a2_i2c_0_init(&m_oled_comm);
		nrf_drv_a2_oled_init(&m_oled_comm, &m_oled_ssd1306_dev);
		display_testing();
		flag_display_on = 1;
		
		sd_ble_gap_tx_power_set(4);
		
//		nrf_gpio_pin_set(VIBRATION_PIN);
//		nrf_delay_ms(500);
//		nrf_gpio_pin_clear(VIBRATION_PIN);
//		nrf_delay_ms(500);				
//		
    for (;;)
    {			
        if (NRF_LOG_PROCESS() == false)
        {
            nrf_pwr_mgmt_run();
        }
				
				if(nrf_gpio_pin_read(ALERT_BT_PIN) == 0)
				{
//						nrf_gpio_pin_set(VIBRATION_PIN);
//						nrf_delay_ms(300);
//					
						
						m_display_page = TIME_PAGE;
						NRF_LOG_INFO("ALERT_BT_PIN Detected\n");
						if(flag_display_on == 0)
						{
								nrf_drv_a2_i2c_0_init(&m_oled_comm);
								nrf_drv_a2_oled_init(&m_oled_comm, &m_oled_ssd1306_dev);
								flag_display_on = 1;
						}
						ssd1306_clear_display();
						ssd1306_set_textsize(2);
						ssd1306_set_textcolor(WHITE);
						ssd1306_set_cursor(50,50);
						ssd1306_putstring("Central Button\n");
						ssd1306_display();
						nrf_delay_ms(2000);
						a2_spi_int();
						pedometer_init();	
						at45dbxx_init();
						nrf_drv_a2_spi_deinit(&m_a2_spi_comm);
						max30102_tmp007_init();
						display_testing();
						nrf_delay_ms(2000);
						nrf_drv_a2_oled_shutdown();
						nrf_drv_a2_i2c_0_deinit(&m_oled_comm);
						flag_display_on = 0;
//						nrf_gpio_pin_clear(VIBRATION_PIN);						
				}
				
				if(nrf_gpio_pin_read(LEFT_BT_PIN) == 0)
				{
//						nrf_gpio_pin_set(VIBRATION_PIN);
						
						m_display_page = TIME_PAGE;
						NRF_LOG_INFO("LEFT_BT_PIN Detected\n");
						if(flag_display_on == 0)
						{
								nrf_drv_a2_i2c_0_init(&m_oled_comm);
								nrf_drv_a2_oled_init(&m_oled_comm, &m_oled_ssd1306_dev);
								flag_display_on = 1;
						}
						ssd1306_clear_display();
						ssd1306_set_textsize(2);
						ssd1306_set_textcolor(WHITE);
						ssd1306_set_cursor(50,30);
						ssd1306_putstring("LEFT \n");
						ssd1306_display();
						nrf_delay_ms(2000);
						a2_spi_int();
						pedometer_init();	
						at45dbxx_init();
						nrf_drv_a2_spi_deinit(&m_a2_spi_comm);
						max30102_tmp007_init();
						display_testing();
						nrf_delay_ms(2000);
						nrf_drv_a2_oled_shutdown();
						nrf_drv_a2_i2c_0_deinit(&m_oled_comm);
						flag_display_on = 0;	
						
						nrf_gpio_pin_set(VIBRATION_PIN);
						nrf_delay_ms(300);
						while(nrf_gpio_pin_read(LEFT_BT_PIN) == 0);
						nrf_gpio_pin_clear(VIBRATION_PIN);
						
						
				}
				
				if(nrf_gpio_pin_read(RIGHT_BT_PIN) == 0)
				{
//						nrf_gpio_pin_set(VIBRATION_PIN);

						m_display_page = TIME_PAGE;
						NRF_LOG_INFO("RIGHT_BT_PIN Detected\n");
						if(flag_display_on == 0)
						{
								nrf_drv_a2_i2c_0_init(&m_oled_comm);
								nrf_drv_a2_oled_init(&m_oled_comm, &m_oled_ssd1306_dev);
								flag_display_on = 1;
						}
						ssd1306_clear_display();
						ssd1306_set_textsize(2);
						ssd1306_set_textcolor(WHITE);
						ssd1306_set_cursor(10,50);
						ssd1306_putstring("RIGHT \n");
						ssd1306_display();
						nrf_delay_ms(2000);
						a2_spi_int();
						pedometer_init();	
						at45dbxx_init();
						nrf_drv_a2_spi_deinit(&m_a2_spi_comm);
						max30102_tmp007_init();
						display_testing();
						nrf_delay_ms(2000);
						nrf_drv_a2_oled_shutdown();
						nrf_drv_a2_i2c_0_deinit(&m_oled_comm);
						flag_display_on = 0;	
//						nrf_gpio_pin_clear(VIBRATION_PIN);
						
						nrf_gpio_pin_set(VIBRATION_PIN);
						nrf_delay_ms(300);
						while(nrf_gpio_pin_read(RIGHT_BT_PIN) == 0);
						nrf_gpio_pin_clear(VIBRATION_PIN);
						
						
				}
				
				
				 
				//if right and left is pressed
//				if()
//				{
//						nrf_delay_ms(200);
//						if()
//						{
//								nrf_delay_ms(1000);
//								if()
//								{
//										//Toggle Max30102 LED
//									  
//								}
//						}
//				}
				
	
				
				
				
				
//				if(nrf_gpio_pin_read(ALERT_BT_PIN) == 0)
//				{
//						m_display_page = TIME_PAGE;
//						NRF_LOG_INFO("ALERT_BT_PIN Detected\n");
//				}
//				
//				display_task(&m_display_page, &a2_info, &m_oled_comm, &m_oled_ssd1306_dev);
//				
//				if(a2_info.time_s%10 == 0 && m_old_time != a2_info.time_s)
//				{		 
//						m_old_time = a2_info.time_s;
//					  a2_spi_int();
//						uint16_t step_cnt;
//						bmi160_read_step_counter(&step_cnt, &m_bmi160_dev);
//						bmi160_get_sensor_data(BMI160_ACCEL_SEL, &m_accel, NULL, &m_bmi160_dev);
//						a2_info.step_count = step_cnt;
//						NRF_LOG_INFO("Acc %d,%d,%d STEP %d \r\n",m_accel.x, m_accel.y, m_accel.z, step_cnt);	
//					  nrf_drv_a2_spi_deinit(&m_a2_spi_comm);
//						adc_init();
//						uint32_t err_code = nrf_drv_saadc_sample();
//						APP_ERROR_CHECK(err_code);		
//				}
    }
}

/**
 * @}
 */
