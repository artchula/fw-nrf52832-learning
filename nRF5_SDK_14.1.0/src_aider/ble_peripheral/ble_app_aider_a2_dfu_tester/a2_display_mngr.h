#ifndef __A2_DISPLAY_MNGR_H__
#define __A2_DISPLAY_MNGR_H__

#include "stdint.h"
#include "nrf_error.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "a2_typedef.h"
#include "nrf_drv_ssd1306.h"

void display_task(display_page_t *page, a2_info_t *p_a2_info, nrf_drv_a2_i2c_0_comm_t *p_comm, oled_ssd1306_dev_t *oled_dev);

void display_date_time(a2_info_t *p_a2_info);
	

#endif

