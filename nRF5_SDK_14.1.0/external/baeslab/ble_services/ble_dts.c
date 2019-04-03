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

#include "ble_dts.h"
#include <string.h>
#include "nordic_common.h"
#include "nrf_log.h"
//#include "app_util.h"


static void on_connect(ble_dts_t *p_dts, ble_evt_t const * p_ble_evt)
{
    p_dts->conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
}/*on_connect*/


static void on_disconnect(ble_dts_t *p_dts, ble_evt_t const * p_ble_evt)
{
    UNUSED_PARAMETER(p_ble_evt);
    p_dts->conn_handle = BLE_CONN_HANDLE_INVALID;
}/*on_disconnect*/


static void on_write(ble_dts_t *p_dts, ble_evt_t const * p_ble_evt)
{
	 ble_date_time_t ble_date_time; 
   uint8_t size = sizeof(ble_date_time_t) - 1;	
   if (p_dts->is_notification_supported)
    {
        ble_gatts_evt_write_t const * p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;
			   if(p_evt_write->len == size){
					  memcpy(&ble_date_time, p_evt_write->data, size);
					  p_dts->evt_handler(p_dts, ble_date_time);
				 }
//        if (p_evt_write->handle == p_dts->date_time_handles.cccd_handle)
//        {
//					
//            // CCCD written, call application event handler
//            if (p_dts->evt_handler != NULL)
//            {
//						//	NRF_LOG_INFO("ok33---------\r\n");
//                ble_dts_evt_t evt;
//                
//                if (ble_srv_is_notification_enabled(p_evt_write->data))
//                {
//                    evt.evt_type = BLE_TIME_EVT_NOTIFICATION_ENABLED;
//                }
//                else
//                {
//                    evt.evt_type = BLE_TIME_EVT_NOTIFICATION_DISABLED;
//                }

//                p_dts->evt_handler(p_dts, &evt);
//            }
//        }/*if */
				
			}
}/*on_write*/


void ble_dts_on_ble_evt(ble_evt_t const * p_ble_evt, void * p_context)
{

	  ble_dts_t * p_dts = (ble_dts_t *) p_context;
	  switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            on_connect(p_dts, p_ble_evt);
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            on_disconnect(p_dts, p_ble_evt);
            break;

        case BLE_GATTS_EVT_WRITE:
            on_write(p_dts, p_ble_evt);
            break;

        default:
            // No implementation needed.
            break;
    }
	
}

//void ble_dts_on_ble_evt(ble_dts_t *p_date_time_svc, ble_evt_t * p_ble_evt)
//{

//    switch (p_ble_evt->header.evt_id)
//    {
//        case BLE_GAP_EVT_CONNECTED:
//            on_connect(p_date_time_svc, p_ble_evt);
//            break;
//            
//        case BLE_GAP_EVT_DISCONNECTED:
//            on_disconnect(p_date_time_svc, p_ble_evt);
//            break;
//            
//        case BLE_GATTS_EVT_WRITE:
//            on_write(p_date_time_svc, p_ble_evt);
//            break;
//				
//        default:
//            break;
//    }
//}


/**@brief Add current time characteristic.
 *
 * @param[in]   p_dts     Date and time Service structure.
 * @param[in]   p_time_init   Information needed to initialize the service.
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
static uint32_t date_and_time_char_add(ble_dts_t *p_dts, 
 const ble_dts_init_t * p_dts_init)
{
    uint32_t            err_code;
    ble_uuid_t          ble_uuid;
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_md_t cccd_md;
    ble_gatts_attr_t    attr_char_value;
    ble_gatts_attr_md_t attr_md;

    
    // Add date and time characteristic
    if (false != p_dts->is_notification_supported)
    {
        memset(&cccd_md, 0, sizeof(cccd_md));
  
        BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
        BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);
        cccd_md.vloc = BLE_GATTS_VLOC_STACK;
    }
    
    memset(&char_md, 0, sizeof(char_md));
    
    char_md.char_props.read   = 1;
    char_md.char_props.write  = 1;
    char_md.char_props.notify = (p_dts->is_notification_supported) ? 1 : 0;
    char_md.p_char_user_desc  = NULL;
    char_md.p_char_pf         = NULL;
    char_md.p_user_desc_md    = NULL;
    char_md.p_cccd_md         = (p_dts->is_notification_supported) ? &cccd_md : NULL;
    char_md.p_sccd_md         = NULL;
    
 
 
    BLE_UUID_BLE_ASSIGN(ble_uuid, BLE_UUID_DATE_TIME_CHAR);
    
    memset(&attr_md, 0, sizeof(attr_md));

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm); 
    attr_md.vloc       = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth    = 0;
    attr_md.wr_auth    = 0;
    attr_md.vlen       = 0;
 
    
    memset(&attr_char_value, 0, sizeof(attr_char_value));

    attr_char_value.p_uuid       = &ble_uuid;
    attr_char_value.p_attr_md    = &attr_md;
    attr_char_value.init_len     = sizeof(ble_date_time_t);
    attr_char_value.init_offs    = 0;
    attr_char_value.max_len      = sizeof(ble_date_time_t);
    attr_char_value.p_value      = (uint8_t*)&p_dts_init->init_date_time;
 
 
    err_code = sd_ble_gatts_characteristic_add(p_dts->service_handle, &char_md,
                                               &attr_char_value,
                                               &p_dts->date_time_handles);
    if (err_code != NRF_SUCCESS)
				return err_code;
        
    return NRF_SUCCESS;
}/*current_time_char_add*/


static uint8_t determine_day_of_week_by_gauss(ble_date_time_t *p_date)
{
	 int16_t first_2_digits_of_year, last_2_digits_of_year;
	 int16_t shift_month, shift_year;
	 
	 int16_t temp;
	 
	 int16_t day_of_week;
	 
	 shift_year = p_date->year;
	 if(1== p_date->month || 2 == p_date->month)
		shift_year -= 1;
	 
	 first_2_digits_of_year = shift_year/100;
	 last_2_digits_of_year = shift_year%100;
	 
	 shift_month = (p_date->month + 10)%12;
	 if(0 == shift_month)
		shift_month = 12;
	 
	 temp = 2.6*shift_month - 0.2;
	 
	 day_of_week = (p_date->day + temp + last_2_digits_of_year 
		+ (last_2_digits_of_year/4) + (first_2_digits_of_year/4) - 2*first_2_digits_of_year);
	 day_of_week = day_of_week % 7;
	 
	 if(0 == day_of_week)
		day_of_week = 7;
	 
 return (uint8_t)(day_of_week);
 
}/*determine_day_of_week_by_gauss*/



static uint32_t week_of_day_char_add(ble_dts_t *p_dts, const ble_dts_init_t *p_time_init)
{
     uint32_t            err_code;
     ble_uuid_t          ble_uuid;
     ble_gatts_char_md_t char_md;
//    ble_gatts_attr_md_t cccd_md;
     ble_gatts_attr_t    attr_char_value;
     ble_gatts_attr_md_t attr_md;
     
 
     BLE_UUID_BLE_ASSIGN(ble_uuid, BLE_UUID_DAY_OF_WEEK_CHAR);
 
     memset(&char_md, 0, sizeof(char_md));
    
    char_md.char_props.read   = 1;
    char_md.char_props.notify = 0;
    char_md.p_char_user_desc  = NULL;
    char_md.p_char_pf         = NULL;
    char_md.p_user_desc_md    = NULL;
    char_md.p_cccd_md         = NULL;
    char_md.p_sccd_md         = NULL;
 
 
    memset(&attr_md, 0, sizeof(attr_md));

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);
    attr_md.vloc       = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth    = 0;
    attr_md.wr_auth    = 0;
    attr_md.vlen       = 0;
 
    
    memset(&attr_char_value, 0, sizeof(attr_char_value));

    uint8_t day_of_week;
 
    day_of_week = determine_day_of_week_by_gauss(&p_dts->current_date_time);
 
    attr_char_value.p_uuid       = &ble_uuid;
    attr_char_value.p_attr_md    = &attr_md;
    attr_char_value.init_len     = sizeof(uint8_t);
    attr_char_value.init_offs    = 0;
    attr_char_value.max_len      = sizeof(uint8_t);
    attr_char_value.p_value      = (uint8_t*)&day_of_week;
 
 
    err_code = sd_ble_gatts_characteristic_add(p_dts->service_handle, &char_md,
                                               &attr_char_value,
                                               &p_dts->day_of_week_handles);
    if (err_code != NRF_SUCCESS)
			return err_code;
 
    return NRF_SUCCESS;
}/*week_of_day_char_add*/


uint32_t ble_dts_init(ble_dts_t *p_dts, const ble_dts_init_t *p_dts_init)
{
    uint32_t   err_code;
    ble_uuid_t ble_uuid;

    // Initialize service structure
    p_dts->evt_handler                = p_dts_init->evt_handler;
    p_dts->conn_handle                = BLE_CONN_HANDLE_INVALID;
    p_dts->is_notification_supported  = p_dts_init->is_notification_supported;
    p_dts->current_date_time    			= p_dts_init->init_date_time;
 
    // Add service
    BLE_UUID_BLE_ASSIGN(ble_uuid, BLE_UUID_CURRENT_TIME_SERVICE);

    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY, 
    &ble_uuid, &p_dts->service_handle);
    if (err_code != NRF_SUCCESS)
        return err_code;
    
    
    err_code = date_and_time_char_add(p_dts, p_dts_init);
    if(err_code != NRF_SUCCESS)
    return err_code;
 
		err_code = week_of_day_char_add(p_dts, p_dts_init);
		if(err_code != NRF_SUCCESS)
		return err_code;
 
 return NRF_SUCCESS;
}/*ble_time_init*/


uint32_t ble_dts_update(ble_dts_t *p_dts, ble_date_time_t *p_current_date_time)
{
 uint32_t err_code;
 err_code = NRF_SUCCESS;
 ble_gatts_value_t gatts_value;
 
  if (0 == memcmp(&p_dts->current_date_time, p_current_date_time, sizeof(ble_date_time_t)))
    return NRF_SUCCESS;
 
  p_dts->current_date_time = *p_current_date_time;
 
 
  gatts_value.len     = sizeof(ble_date_time_t);
  gatts_value.offset  = 0;
  gatts_value.p_value = (uint8_t*)p_current_date_time;

  
  err_code = sd_ble_gatts_value_set(p_dts->conn_handle, 
  p_dts->date_time_handles.value_handle, &gatts_value);
 
	if (err_code != NRF_SUCCESS)
		return err_code;
 
	 // Send value if connected and notifying
	 if ((p_dts->conn_handle != BLE_CONN_HANDLE_INVALID) && p_dts->is_notification_supported)
	 {
		ble_gatts_hvx_params_t hvx_params;
		uint16_t len;
		
		memset(&hvx_params, 0, sizeof(hvx_params));
		len = sizeof(ble_date_time_t);
		
		hvx_params.handle   = p_dts->date_time_handles.value_handle;
		hvx_params.type     = BLE_GATT_HVX_NOTIFICATION;
		hvx_params.offset   = 0;
		hvx_params.p_len    = &len;
		hvx_params.p_data   = (uint8_t*)p_current_date_time;
		
		err_code = sd_ble_gatts_hvx(p_dts->conn_handle, &hvx_params);
	 }else{
		err_code = NRF_ERROR_INVALID_STATE;
	 }
 
 

	 uint8_t day_of_week;
	 
	 day_of_week = determine_day_of_week_by_gauss(p_current_date_time);
	 
	 gatts_value.len     = sizeof(uint8_t);
	 gatts_value.offset  = 0;
	 gatts_value.p_value = (uint8_t*)&day_of_week;
	 
	 err_code = sd_ble_gatts_value_set(p_dts->conn_handle, 
		p_dts->day_of_week_handles.value_handle, &gatts_value);
	 
	 if (err_code != NRF_SUCCESS)
			return err_code; 
	 
	 return NRF_SUCCESS;
}/*ble_data_time_update*/

