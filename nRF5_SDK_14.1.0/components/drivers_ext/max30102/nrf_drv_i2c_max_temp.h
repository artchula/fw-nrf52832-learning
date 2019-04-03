#ifndef __NRF_DRV_I2C_MAX_TEMP_H__
#define __NRF_DRV_I2C_MAX_TEMP_H__

#include <stdio.h>
#include "boards.h"
#include "app_util_platform.h"
#include "app_timer.h"
#include "nrf_drv_clock.h"
#include "app_error.h"
#include "app_timer.h"
#include "nrf_spi_mngr.h"
#include "bsp.h"
#include "nrf_delay.h"
#include "nrf_twi_mngr.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"

#include "MAX30102.h"
#include "tmp007.h"

#define MAX_PENDING_TRANSACTIONS    		5
#define I2C_INSTANCE_MAX30102_TMP 			1 											     	       /**< I2C instance index. */

NRF_TWI_MNGR_DEF(m_app_twi_1, MAX_PENDING_TRANSACTIONS, I2C_INSTANCE_MAX30102_TMP);

typedef struct{
		uint8_t scl_pin;
		uint8_t sda_pin;
		uint8_t isr_max30102_pin;
	  uint8_t isr_temp007_pin;
		uint8_t en_power_1V8_max30102_pin;
}nrf_drv_a2_max_tmp_comm_t;


uint32_t nrf_drv_a2_i2c_1_init(nrf_drv_a2_max_tmp_comm_t *p_comm);
uint32_t nrf_drv_a2_i2c_1_deinit(nrf_drv_a2_max_tmp_comm_t *p_comm);

uint32_t nrf_drv_max30102_reg_i2c_func(nrf_drv_a2_max_tmp_comm_t *p_comm, max30102_dev_t *max_dev);
uint8_t nrf_drv_i2c_max30102_read(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t len);
uint8_t nrf_drv_i2c_max30102_write(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t len);
uint8_t nrf_drv_i2c_max30102_shutdown(nrf_drv_a2_max_tmp_comm_t *p_comm, max30102_dev_t *max_dev);


uint32_t nrf_drv_temp007_reg_i2c_func(nrf_drv_a2_max_tmp_comm_t *p_comm, tmp007_dev_t *tmp_dev);
uint8_t nrf_drv_i2c_temp007_read(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t len);
uint8_t nrf_drv_i2c_temp007_write(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t len);


		
#endif

