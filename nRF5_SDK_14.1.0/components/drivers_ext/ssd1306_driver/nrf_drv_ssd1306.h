#ifndef NRF_DRV_SSD1306_H__
#define NRF_DRV_SSD1306_H__

#include <stdio.h>
#include "boards.h"
#include "app_util_platform.h"
#include "nrf_drv_rtc.h"
#include "nrf_drv_clock.h"
#include "bsp.h"
#include "app_error.h"
#include "app_timer.h"
#include "nrf_twi_mngr.h"
#include "compiler_abstraction.h"
//#define NRF_LOG_MODULE_NAME "APP"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "ssd1306.h"
#include "nrf_delay.h"


#define OLED_SSD1306_I2C_ADDRESS		0x3C

typedef struct{
		uint8_t scl_pin;
		uint8_t sda_pin;
		uint8_t oled_power_en_pin;
}nrf_drv_a2_i2c_0_comm_t;

int8_t nrf_drv_i2c_0_read(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t len);
int8_t nrf_drv_i2c_0_write(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t len);

uint8_t nrf_drv_a2_i2c_0_init(nrf_drv_a2_i2c_0_comm_t *p_comm);
uint8_t nrf_drv_a2_i2c_0_deinit(nrf_drv_a2_i2c_0_comm_t *p_comm);

uint8_t nrf_drv_a2_oled_init(nrf_drv_a2_i2c_0_comm_t *p_comm, oled_ssd1306_dev_t *oled_dev);


uint8_t nrf_drv_a2_oled_shutdown(void);
uint8_t nrf_drv_a2_oled_dim(uint8_t enable);

#endif

