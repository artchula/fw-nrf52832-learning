/* Copyright (c) 2012 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 */

/** @file
 *
 * @defgroup ble_sdk_srv_time current time Service
 * @{
 * @ingroup ble_sdk_srv
 * @brief current timer module.
 *
 * @details This module implements the current timer Service with the current time 
 *    , local time information(option) and reference time information(option) characteristic.
 *          During initialization it adds the current time Service and current time characteristic
 *          to the BLE stack database. Optionally it can also add local time,reference time descriptor
 *          to the current time server.
 *
 *          If specified, the module will support notification of the current time characteristic
 *          through the ble_time_current_time_update() function.
 *          If an event handler is supplied by the application, the current time Service will
 *          generate current timer Service events to the application.
 *
 * @note The application must propagate BLE stack events to the current timer Service module by calling
 *       ble_time_on_ble_evt() from the from the @ref ble_stack_handler callback.
 */

#ifndef _BLE_DTS_H_
#define _BLE_DTS_H_

#include <stdint.h>
#include <stdbool.h>
#include "ble.h"
#include "ble_srv_common.h"
#include "ble_gattc.h"
#include "ble_date_time.h"


/**@brief   Macro for defining a ble_bas instance.
 *
 * @param   _name   Name of the instance.
 * @hideinitializer
 */
#define BLE_DTS_C_DEF(_name)                                                                        \
static ble_dts_t _name;                                                                           \
NRF_SDH_BLE_OBSERVER(_name ## _obs,                                                                 \
                     BLE_CTS_C_BLE_OBSERVER_PRIO,                                                   \
                     ble_dts_on_ble_evt, &_name)

/**@brief current timer Service event type. */
typedef enum
{
    BLE_TIME_EVT_NOTIFICATION_ENABLED,                             /**< current time notification enabled event. */
    BLE_TIME_EVT_NOTIFICATION_DISABLED                             /**< current time notification disabled event. */
} ble_dts_evt_type_t;


/**@brief time Service event. */
typedef struct
{
    ble_dts_evt_type_t evt_type;                                  /**< Type of event. */
} ble_dts_evt_t;

// Forward declaration of the ble_time_t type. 
typedef struct ble_dts_s ble_dts_t;


/**@brief current time Service event handler type. */
//typedef void (*ble_time_evt_handler_t) (ble_dts_t *p_time, ble_dts_evt_t * p_evt);
typedef void (*ble_time_evt_handler_t) (ble_dts_t *p_time, ble_date_time_t ble_date_time);


/**@brief current time Service init structure. This contains all options and data needed for
 *        initialization of the service.*/
typedef struct
{
    ble_time_evt_handler_t   evt_handler;     /**< Event handler to be called for handling events in the current time Service. */
    bool                 is_notification_supported;    /**< TRUE if notification of current time measurement is supported. */
    ble_date_time_t      init_date_time;
} ble_dts_init_t;


/**@brief current time Service structure. This contains various status information for the service. */
struct ble_dts_s
{
    ble_time_evt_handler_t        evt_handler;                    /**< Event handler to be called for handling events in the current time Service. */
    uint16_t                      service_handle;                 /**< Handle of current time Service (as provided by the BLE stack). */
    ble_gatts_char_handles_t      date_time_handles;              /**< Handles related to the current time characteristic. */
    ble_gatts_char_handles_t      day_of_week_handles;            /**< Handles related to the day of week characteristic. */
    uint16_t                      conn_handle;                    /**< Handle of the current connection (as provided by the BLE stack, is BLE_CONN_HANDLE_INVALID if not in a connection). */
    bool                          is_notification_supported;      /**< TRUE if notification of current time is supported. */
    ble_date_time_t               current_date_time;             /**< Last current time passed to the current time Service. */
    //uint8_t data_test[2];
};


void ble_dts_on_ble_evt(ble_evt_t const * p_ble_evt, void * p_context);

uint32_t ble_dts_init(ble_dts_t *p_dts, const ble_dts_init_t *p_dts_init);

//void ble_dts_on_ble_evt(ble_dts_t *p_dts, ble_evt_t *p_ble_evt);

uint32_t ble_dts_update(ble_dts_t *p_dts, ble_date_time_t *p_current_time);

#endif // _BLE_DTS_H_

/** @} */



