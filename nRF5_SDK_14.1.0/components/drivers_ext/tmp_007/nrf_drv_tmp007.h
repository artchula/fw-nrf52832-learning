#ifndef __NRF_DRV_TMP_TMP007_H__
#define __NRF_DRV_TMP_TMP007_H__


#include "stdint.h"
#include "bsp.h"
#include "app_error.h"
#include "app_timer.h"
#include "app_twi.h"
#include "tmp007.h"


int8_t tmp007_i2c_read(uint8_t dev_addr, uint8_t reg_addr,
		uint8_t *data, uint16_t len);
		
int8_t tmp007_i2c_write(uint8_t dev_addr, uint8_t reg_addr,
		uint8_t *data, uint16_t len);
uint8_t tmp007_i2c_init(app_twi_t *p_twi);		


#endif

