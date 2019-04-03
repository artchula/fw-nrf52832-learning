/**
 * Copyright (c) 2015 - 2017, Nordic Semiconductor ASA
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
 
#ifndef NRF_DRV_SPI_BMI_FLASH_H__
#define NRF_DRV_SPI_BMI_FLASH_H__

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

#include "nrf_log.h"
#include "nrf_log_ctrl.h"

#include "bmi160.h"
#include "at45db081e.h"

typedef struct bmi160_dev bmi160_dev_t;


typedef struct{
		uint8_t mosi_pin;
		uint8_t miso_pin;
		uint8_t sck_pin;
		uint8_t cs_bmi_pin;
		uint8_t cs_flash_pin;
		uint8_t int1_bmi_pin;
		uint8_t int2_bmi_pin;
		uint8_t power_bmi_en_pin;
		uint8_t wp_flash_pin;
		uint8_t reset_flash_pin;
}nrf_drv_a2_spi_comm_t;

void nrf_drv_spi_bmi_cs_low(void * p_user_data);
void nrf_drv_spi_flash_cs_low(void * p_user_data);
void nrf_drv_spi_cs_all_high(void * p_user_data);
	
int8_t nrf_drv_spi_bmi160_read(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t len);
int8_t nrf_drv_spi_bmi160_write(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t len);

uint8_t nrf_drv_a2_spi_init2(void);

uint8_t nrf_drv_a2_spi_init(nrf_drv_a2_spi_comm_t *p_comm);
uint8_t nrf_drv_a2_spi_deinit(nrf_drv_a2_spi_comm_t *p_comm);

uint8_t nrf_drv_bmi160_spi_int(nrf_drv_a2_spi_comm_t *p_comm, bmi160_dev_t *p_bmi160_dev);
uint8_t nrf_drv_bmi160_acc_config(nrf_drv_a2_spi_comm_t *p_comm,  bmi160_dev_t *p_bmi160_dev);
uint8_t nrf_drv_bmi160_step_count_conf(nrf_drv_a2_spi_comm_t *p_comm, bmi160_dev_t *p_bmi160_dev);
uint8_t nrf_drv_bmi160_free_fall_conf(nrf_drv_a2_spi_comm_t *p_comm, bmi160_dev_t *p_bmi160_dev);
uint8_t nrf_drv_bmi160_motion_conf(nrf_drv_a2_spi_comm_t *p_comm, bmi160_dev_t *p_bmi160_dev);
uint8_t nrf_drv_bmi160_no_motion_conf(nrf_drv_a2_spi_comm_t *p_comm, bmi160_dev_t *p_bmi160_dev);
uint8_t nrf_drv_bmi160_step_count_reset_value(nrf_drv_a2_spi_comm_t *p_comm, bmi160_dev_t *p_bmi160_dev);
void nrf_drv_bmi160_power_and_cs_init(nrf_drv_a2_spi_comm_t *p_comm);
void nrf_drv_bmi160_power_off(nrf_drv_a2_spi_comm_t *p_comm);

uint8_t nrf_drv_flash_spi_init(nrf_drv_a2_spi_comm_t *p_comm, at45dbxx_dev_t *p_at45dbxx_dev);
uint8_t nrf_dev_flash_spi_wakeup(at45dbxx_dev_t *p_at45dbxx_dev);
int8_t nrf_drv_spi_flash_read(uint8_t command[], uint16_t command_len, uint8_t data[], uint16_t data_len, uint8_t en_continues_cs_low);
int8_t nrf_drv_spi_flash_write(uint8_t command[], uint16_t command_len, uint8_t data[], uint16_t data_len, uint8_t en_continues_cs_low);

int8_t flash_erase_page(uint16_t page_addr);
int8_t flash_erase_chip(void);
int8_t flash_write(uint16_t page_addr, uint8_t write_data[], uint16_t len);
int8_t flash_read(uint16_t page_addr, uint8_t read_data[], uint16_t len);


#endif
