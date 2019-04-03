#include "nrf_drv_spi_bmi_flash.h"

#define ST7565_QUEUE_LENGTH     10
#define ST7565_SPI_INSTANCE_ID  2

NRF_SPI_MNGR_DEF(m_nrf_spi_mngr2, ST7565_QUEUE_LENGTH, ST7565_SPI_INSTANCE_ID);

extern at45dbxx_dev_t	m_at45dbxx_dev;


static uint8_t tx_buf[255];// = {0x00, 0x00};
static uint8_t rx_buf[255];			


static nrf_drv_a2_spi_comm_t *p_a2_spi_comm;				// Pointer pin for bmi160 and at45db081 flash memory. 


// Function called before SPI command is send to LCD display. I is used to drive A0 pin low.
void bmi160_set_bmi160_cs_pin(void)
{
    nrf_gpio_pin_set(p_a2_spi_comm->cs_bmi_pin);
}

// Function called before SPI command is send to LCD display. I is used to drive A0 pin low.
void bmi160_clear_bmi160_cs_pin(void)
{
		nrf_gpio_pin_clear(p_a2_spi_comm->cs_bmi_pin);
}

// Function called before SPI command is send to LCD display. I is used to drive A0 pin low.
void bmi160_set_flash_cs_pin(void)
{
    nrf_gpio_pin_set(p_a2_spi_comm->cs_flash_pin);
}

// Function called before SPI command is send to LCD display. I is used to drive A0 pin low.
void bmi160_clear_flash_cs_pin(void)
{
		nrf_gpio_pin_clear(p_a2_spi_comm->cs_flash_pin);
}


int8_t nrf_drv_spi_bmi160_read(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t len)
{
	
		uint32_t err_code;
		
		
		memset(tx_buf, 0, 255);
		memset(rx_buf, 0, 255);
	
	  tx_buf[0] = reg_addr;
	
		const uint8_t tx_len = 1;
		const uint8_t rx_len = len + 1;
		
		nrf_spi_mngr_transfer_t const transfer_cmd[] =
    {
				NRF_SPI_MNGR_TRANSFER(&tx_buf[0], tx_len, rx_buf, rx_len),
    };
		
		bmi160_clear_bmi160_cs_pin();
		err_code = nrf_spi_mngr_perform(&m_nrf_spi_mngr2, &transfer_cmd[0], 1, NULL);
		APP_ERROR_CHECK(err_code);
		bmi160_set_bmi160_cs_pin();
		
		memcpy(&data[0], &rx_buf[1], len );

//		uint32_t err_code; 
//		uint8_t dummy = 0x55;
//		
//		nrf_spi_mngr_transfer_t transfer_cmd[3];
//	
//		if(len<256)
//		{
////				memset(tx_buf, 0xFF, 255);
////			
////				uint8_t len_for_read = len%255;
////			
////				transfer_cmd[0].p_tx_data = &reg_addr;
////				transfer_cmd[0].tx_length = sizeof(reg_addr);
////				transfer_cmd[0].p_rx_data = rx_buf;
////				transfer_cmd[0].rx_length = sizeof(reg_addr);
////			
////				transfer_cmd[1].p_tx_data = tx_buf;
////				transfer_cmd[1].tx_length = len;
////				transfer_cmd[1].p_rx_data = &data[0];
////				transfer_cmd[1].rx_length = len;
////			
////				bmi160_clear_bmi160_cs_pin();
////				err_code = nrf_spi_mngr_perform(&m_nrf_spi_mngr2, &transfer_cmd[0], 1, NULL);
////				err_code = nrf_spi_mngr_perform(&m_nrf_spi_mngr2, &transfer_cmd[1], 1, NULL);
////				bmi160_set_bmi160_cs_pin();
//			  memset(tx_buf, 0xFF, 255);
//			
//				uint8_t len_for_read = len%255;
//			
//				transfer_cmd[0].p_tx_data = &reg_addr;
//				transfer_cmd[0].tx_length = sizeof(reg_addr);
//				transfer_cmd[0].p_rx_data = rx_buf;
//				transfer_cmd[0].rx_length = sizeof(reg_addr) + len;
//		
//				bmi160_clear_bmi160_cs_pin();
//				err_code = nrf_spi_mngr_perform(&m_nrf_spi_mngr2, &transfer_cmd[0], 1, NULL);
//				bmi160_set_bmi160_cs_pin();
//				
//				memcpy(data, &rx_buf[1], len);
//		}
//		else if(len<512)
//		{
//				
////				uint8_t len_for_read = len%255;
////			
////				memset(tx_buf, 0xFF, 255);
////			
////				transfer_cmd[0].p_tx_data = &reg_addr;
////				transfer_cmd[0].tx_length = sizeof(reg_addr);
////				transfer_cmd[0].p_rx_data = 0;
////				transfer_cmd[0].rx_length = 0;
////			
////				transfer_cmd[1].p_tx_data = tx_buf;
////				transfer_cmd[1].tx_length = 255;
////				transfer_cmd[1].p_rx_data = &data[0];
////				transfer_cmd[1].rx_length = 255;
////				
////				transfer_cmd[2].p_tx_data = tx_buf;
////				transfer_cmd[2].tx_length = len_for_read;
////				transfer_cmd[2].p_rx_data = &data[255];
////				transfer_cmd[2].rx_length = len_for_read;
////				bmi160_clear_bmi160_cs_pin();
////				err_code = nrf_spi_mngr_perform(&m_nrf_spi_mngr2, &transfer_cmd[0], 1, NULL);
////				err_code = nrf_spi_mngr_perform(&m_nrf_spi_mngr2, &transfer_cmd[1], 1, NULL);
////				err_code = nrf_spi_mngr_perform(&m_nrf_spi_mngr2, &transfer_cmd[2], 1, NULL);
////				bmi160_set_bmi160_cs_pin();
//		}
		
		return 0;
}

int8_t nrf_drv_spi_bmi160_write(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t len)
{			
		uint32_t err_code;
		
		memset(tx_buf, 0, 15);
		memset(rx_buf, 0, 15);
		
		tx_buf[0] = reg_addr;
		
		memcpy(&tx_buf[1], &data[0], len);
		
		const uint8_t tx_len = len+1;
		const uint8_t rx_len = 2;
		
		nrf_spi_mngr_transfer_t const transfer_cmd[] =
    {
        NRF_SPI_MNGR_TRANSFER(&tx_buf[0], tx_len, rx_buf, rx_len),
    };
		bmi160_clear_bmi160_cs_pin();
	  err_code = nrf_spi_mngr_perform(&m_nrf_spi_mngr2, &transfer_cmd[0], 1, NULL);
		APP_ERROR_CHECK(err_code);
		bmi160_set_bmi160_cs_pin();
		
		return 0;
}

uint8_t nrf_drv_a2_spi_init(nrf_drv_a2_spi_comm_t *p_comm)
{
    nrf_drv_spi_config_t const m_master2_config =
    {
        .sck_pin        = p_comm->sck_pin,
        .mosi_pin       = p_comm->mosi_pin,
        .miso_pin       = p_comm->miso_pin,
        .ss_pin         = NRF_DRV_SPI_PIN_NOT_USED, //p_comm->cs_bmi_pin,
        .irq_priority   = APP_IRQ_PRIORITY_LOWEST,
        .orc            = 0xFF,
        .frequency      = NRF_DRV_SPI_FREQ_1M,
        .mode           = NRF_DRV_SPI_MODE_0,
        .bit_order      = NRF_DRV_SPI_BIT_ORDER_MSB_FIRST
    };
		p_a2_spi_comm = p_comm;
		
    return nrf_spi_mngr_init(&m_nrf_spi_mngr2, &m_master2_config);
}

uint8_t nrf_drv_a2_spi_deinit(nrf_drv_a2_spi_comm_t *p_comm)
{
		nrf_spi_mngr_uninit(&m_nrf_spi_mngr2);
		
//		nrf_gpio_cfg(
//        p_comm->mosi_pin,
//        NRF_GPIO_PIN_DIR_INPUT,
//        NRF_GPIO_PIN_INPUT_DISCONNECT,
//        NRF_GPIO_PIN_NOPULL,
//        NRF_GPIO_PIN_S0S1,
//        NRF_GPIO_PIN_NOSENSE);
//	
//		nrf_gpio_cfg(
//        p_comm->miso_pin,
//        NRF_GPIO_PIN_DIR_INPUT,
//        NRF_GPIO_PIN_INPUT_DISCONNECT,
//        NRF_GPIO_PIN_NOPULL,
//        NRF_GPIO_PIN_S0S1,
//        NRF_GPIO_PIN_NOSENSE);
//		
//		nrf_gpio_cfg(
//        p_comm->sck_pin,
//        NRF_GPIO_PIN_DIR_INPUT,
//        NRF_GPIO_PIN_INPUT_DISCONNECT,
//        NRF_GPIO_PIN_NOPULL,
//        NRF_GPIO_PIN_S0S1,
//        NRF_GPIO_PIN_NOSENSE);
//		
		nrf_gpio_cfg_output(p_comm->sck_pin);
		nrf_gpio_cfg_output(p_comm->miso_pin);
		nrf_gpio_cfg_output(p_comm->mosi_pin);
		nrf_gpio_pin_set(p_comm->sck_pin);
		nrf_gpio_pin_set(p_comm->miso_pin);
		nrf_gpio_pin_set(p_comm->mosi_pin);
		
		
		nrf_gpio_cfg(
        p_comm->cs_flash_pin,
        NRF_GPIO_PIN_DIR_INPUT,
        NRF_GPIO_PIN_INPUT_DISCONNECT,
        NRF_GPIO_PIN_PULLUP,
        NRF_GPIO_PIN_S0S1,
        NRF_GPIO_PIN_NOSENSE);
				
		nrf_gpio_cfg(
        p_comm->cs_bmi_pin,
        NRF_GPIO_PIN_DIR_INPUT,
        NRF_GPIO_PIN_INPUT_DISCONNECT,
        NRF_GPIO_PIN_PULLUP,
        NRF_GPIO_PIN_S0S1,
        NRF_GPIO_PIN_NOSENSE);
				
		nrf_gpio_cfg(
        p_comm->reset_flash_pin,
        NRF_GPIO_PIN_DIR_INPUT,
        NRF_GPIO_PIN_INPUT_DISCONNECT,
        NRF_GPIO_PIN_NOPULL,
        NRF_GPIO_PIN_S0S1,
        NRF_GPIO_PIN_NOSENSE);
				
		nrf_gpio_cfg(
        24,
        NRF_GPIO_PIN_DIR_INPUT,
        NRF_GPIO_PIN_INPUT_DISCONNECT,
        NRF_GPIO_PIN_NOPULL,
        NRF_GPIO_PIN_S0S1,
        NRF_GPIO_PIN_NOSENSE);				
	
	
		return NRF_SUCCESS;
}

uint8_t nrf_drv_a2_spi_init2(void)
{
		// SPI0 (with transaction manager) initialization.
    static nrf_drv_spi_config_t const m_master2_config =
    {
        .sck_pin        = 18,
        .mosi_pin       = 19,
        .miso_pin       = 20,
        .ss_pin         = 22,
        .irq_priority   = APP_IRQ_PRIORITY_LOWEST,
        .orc            = 0xFF,
        .frequency      = NRF_DRV_SPI_FREQ_250K,
        .mode           = NRF_DRV_SPI_MODE_0,
        .bit_order      = NRF_DRV_SPI_BIT_ORDER_MSB_FIRST
    };
    return nrf_spi_mngr_init(&m_nrf_spi_mngr2, &m_master2_config);
}

/**
 * @brief BMI160 Accelerometer Sensor functional.
 * @param nrf_drv_a2_spi_config_t*
 */
uint8_t nrf_drv_bmi160_spi_int(nrf_drv_a2_spi_comm_t *p_comm, bmi160_dev_t *p_bmi160_dev)
{
		int8_t rslt;
		nrf_gpio_cfg_output(p_comm->power_bmi_en_pin);
		nrf_gpio_pin_clear(p_comm->power_bmi_en_pin);
		nrf_gpio_cfg_output(p_comm->cs_bmi_pin);
		nrf_gpio_pin_set(p_comm->cs_bmi_pin);

	
		p_bmi160_dev->id        = 0;
		p_bmi160_dev->interface = BMI160_SPI_INTF;
		p_bmi160_dev->read      = (bmi160_com_fptr_t)nrf_drv_spi_bmi160_read;
		p_bmi160_dev->write     = (bmi160_com_fptr_t)nrf_drv_spi_bmi160_write;
		p_bmi160_dev->delay_ms  = nrf_delay_ms;
		
		//nrf_drv_spi_bmi_cs_low(p_comm);
		rslt = bmi160_init(p_bmi160_dev);
		//nrf_drv_spi_cs_all_high(p_comm);
	
		if(rslt != BMI160_OK)
		{
				return NRF_ERROR_TIMEOUT;
		}
		return NRF_SUCCESS;
}

void nrf_drv_bmi160_power_and_cs_init(nrf_drv_a2_spi_comm_t *p_comm)
{
		nrf_gpio_cfg_output(p_comm->power_bmi_en_pin);
		nrf_gpio_pin_clear(p_comm->power_bmi_en_pin);
		nrf_gpio_cfg_output(p_comm->cs_bmi_pin);
		nrf_gpio_pin_clear(p_comm->cs_bmi_pin);
}

void nrf_drv_bmi160_power_off(nrf_drv_a2_spi_comm_t *p_comm)
{
		nrf_gpio_cfg_output(p_comm->power_bmi_en_pin);
		nrf_gpio_pin_set(p_comm->power_bmi_en_pin);
}
	
	
uint8_t nrf_drv_bmi160_acc_config(nrf_drv_a2_spi_comm_t *p_comm, bmi160_dev_t *p_bmi160_dev)
{
		int8_t rslt;

		//Wait Imprementation
		/*
		//nrf_gpio_cfg_output(config->cs_bmi_pin);	
		//nrf_gpio_pin_clear(config->cs_bmi_pin);  	
		*/
	
		/* Select the Output data rate, range of accelerometer sensor */
//		p_bmi160_dev->accel_cfg.odr = BMI160_ACCEL_ODR_200HZ; //;
//		p_bmi160_dev->accel_cfg.range = BMI160_ACCEL_RANGE_2G;
//		p_bmi160_dev->accel_cfg.bw = BMI160_ACCEL_BW_NORMAL_AVG4;//;
//	
	  p_bmi160_dev->accel_cfg.odr = BMI160_ACCEL_ODR_200HZ; //;
		p_bmi160_dev->accel_cfg.range = BMI160_ACCEL_RANGE_2G;
		p_bmi160_dev->accel_cfg.bw = BMI160_ACCEL_BW_NORMAL_AVG4;//;

		/* Select the power mode of accelerometer sensor */
		p_bmi160_dev->accel_cfg.power = BMI160_ACCEL_NORMAL_MODE;

		/* Select the Output data rate, range of Gyroscope sensor */
		p_bmi160_dev->gyro_cfg.odr = BMI160_GYRO_ODR_3200HZ;
		p_bmi160_dev->gyro_cfg.range = BMI160_GYRO_RANGE_2000_DPS;
		p_bmi160_dev->gyro_cfg.bw = BMI160_GYRO_BW_NORMAL_MODE;

		/* Select the power mode of Gyroscope sensor */
		p_bmi160_dev->gyro_cfg.power = BMI160_GYRO_SUSPEND_MODE; 

		uint8_t chip_id;
		
		//nrf_drv_spi_bmi_cs_low(p_comm);
		nrf_drv_spi_bmi160_read(0, BMI160_CHIP_ID_ADDR | BMI160_SPI_RD_MASK, &chip_id, 1);
		//nrf_drv_spi_cs_all_high(p_comm);
		
		/* Set the sensor configuration */
		//nrf_drv_spi_bmi_cs_low(p_comm);
		rslt = bmi160_set_sens_conf(p_bmi160_dev);
		//nrf_drv_spi_cs_all_high(p_comm);
	
		if(rslt != BMI160_OK)
		{
				return NRF_ERROR_TIMEOUT;
		}
		
		return NRF_SUCCESS;
}

uint8_t nrf_drv_bmi160_step_count_conf(nrf_drv_a2_spi_comm_t *p_comm, bmi160_dev_t *p_bmi160_dev)
{
		int8_t rslt;
		uint8_t reg0, reg1;
		
		//Normal Mode
		reg0 = 0x15;
		reg1 = 0x08;
	
		//Sensitive Mode
		//reg0 = 0x2D;
		//reg1 = 0x00;
	
		//Sensitive Mode
		//reg0 = 0x2D;
		//reg1 = 0x00;
	
	
		rslt = bmi160_set_regs(BMI160_INT_STEP_CONFIG_0_ADDR, &reg0, 1, p_bmi160_dev);
		rslt = bmi160_set_regs(BMI160_INT_STEP_CONFIG_1_ADDR, &reg1, 1, p_bmi160_dev);
	
		if(rslt != BMI160_OK)
		{
				return NRF_ERROR_TIMEOUT;
		}
		
		return NRF_SUCCESS;
}

uint8_t nrf_drv_bmi160_step_count_reset_value(nrf_drv_a2_spi_comm_t *p_comm, bmi160_dev_t *p_bmi160_dev)
{
		int8_t rslt;
		uint8_t reg;
	
		reg = BMI160_STEP_COUNT_CLR;
	
		rslt = bmi160_set_regs(BMI160_COMMAND_REG_ADDR, &reg, 1, p_bmi160_dev);
	
		if(rslt != BMI160_OK)
		{
				return NRF_ERROR_TIMEOUT;
		}
		
		return NRF_SUCCESS;
}
uint8_t nrf_drv_bmi160_free_fall_conf(nrf_drv_a2_spi_comm_t *p_comm, bmi160_dev_t *p_bmi160_dev)
{
		struct bmi160_int_settg int_config;
		int8_t rslt;
	
		int_config.fifo_full_int_en = 0;
		int_config.fifo_WTM_int_en 	= 0;
		int_config.int_channel			= BMI160_INT_CHANNEL_2;	
		int_config.int_type					= BMI160_ACC_LOW_G_INT;

		int_config.int_type_cfg.acc_low_g_int.low_dur				= 0x0F;//0x01;//0x0F;  //50cm
		int_config.int_type_cfg.acc_low_g_int.low_thres			= 0x09;//0x3F;//0x09;  //50cm 
		int_config.int_type_cfg.acc_low_g_int.low_hyst			= 1;
		int_config.int_type_cfg.acc_low_g_int.low_mode			= 0;
		int_config.int_type_cfg.acc_low_g_int.low_data_src	= 0;
		int_config.int_type_cfg.acc_low_g_int.low_en				= 1;
		
		int_config.int_pin_settg.output_en		= 1;
		int_config.int_pin_settg.output_mode	= 0;
		int_config.int_pin_settg.output_type	= 1; 									//1 set to active high, 0 set to active low
		int_config.int_pin_settg.edge_ctrl		= 0; 									//I'don't understand	
		int_config.int_pin_settg.input_en			= 0; 									//I'don't understand	
		int_config.int_pin_settg.latch_dur    = 0x0D; 							//I'don't understand	
		
		bmi160_set_int_config(&int_config ,p_bmi160_dev);
		
		if(rslt != BMI160_OK)
		{
				return NRF_ERROR_TIMEOUT;
		}
		
		return NRF_SUCCESS;
}

uint8_t nrf_drv_bmi160_motion_conf(nrf_drv_a2_spi_comm_t *p_comm, bmi160_dev_t *p_bmi160_dev)
{
	  int8_t rslt;
//		struct bmi160_int_settg int_config;
//		
//	
//		int_config.fifo_full_int_en = 0;
//		int_config.fifo_WTM_int_en 	= 0;
//		int_config.int_channel			= BMI160_INT_CHANNEL_1;	
//		int_config.int_type					= BMI160_ACC_ANY_MOTION_INT;
//	
//		int_config.int_type_cfg.acc_any_motion_int.anymotion_data_src
//		int_config.int_type_cfg.acc_any_motion_int.anymotion_dur
//	  int_config.int_type_cfg.acc_any_motion_int.anymotion_en
//		int_config.int_type_cfg.acc_any_motion_int.anymotion_thr
//		int_config.int_type_cfg.acc_any_motion_int.anymotion_x
//		int_config.int_type_cfg.acc_any_motion_int.anymotion_y
//		int_config.int_type_cfg.acc_any_motion_int.anymotion_z
	
		if(rslt != BMI160_OK)
		{
				return NRF_ERROR_TIMEOUT;
		}
		
		return NRF_SUCCESS;
}

uint8_t nrf_drv_bmi160_no_motion_conf(nrf_drv_a2_spi_comm_t *p_comm, bmi160_dev_t *p_bmi160_dev)
{
		struct bmi160_int_settg int_config;
		int8_t rslt;
	
		int_config.fifo_full_int_en = 0;
		int_config.fifo_WTM_int_en 	= 0;
		int_config.int_channel			= BMI160_INT_CHANNEL_1;	
		int_config.int_type					= BMI160_ACC_SLOW_NO_MOTION_INT;
	
		int_config.int_type_cfg.acc_no_motion_int.no_motion_dur = 0x00;  	/*! Set duration time to 20.48S. Please see detial datasheet page 69.*/
		int_config.int_type_cfg.acc_no_motion_int.no_motion_sel = 1;        	/*! no motion sel , 1 - enable no-motion ,0- enable slow-motion */    
		int_config.int_type_cfg.acc_no_motion_int.no_motion_src = 1;					/*! data source 0- filter & 1 pre-filter*/
	  int_config.int_type_cfg.acc_no_motion_int.no_motion_thres = 0x0F; 	/*! no motion threshold */
	  int_config.int_type_cfg.acc_no_motion_int.no_motion_x = 1;            /*! no motion interrupt x */
	  int_config.int_type_cfg.acc_no_motion_int.no_motion_y = 0;            /*! no motion interrupt x */
		int_config.int_type_cfg.acc_no_motion_int.no_motion_z = 0;            /*! no motion interrupt x */   
		
	
		int_config.int_pin_settg.output_en		= 1;
		int_config.int_pin_settg.output_mode	= 1;
		int_config.int_pin_settg.output_type	= 1; 									//1 set to active high, 0 set to active low
		int_config.int_pin_settg.edge_ctrl		= 0; 									//I'don't understand	
		int_config.int_pin_settg.input_en			= 0; 									//I'don't understand	
		int_config.int_pin_settg.latch_dur    = 0x0D; 							//I'don't understand
		
		bmi160_set_int_config(&int_config ,p_bmi160_dev);
		
		if(rslt != BMI160_OK)
		{
				return NRF_ERROR_TIMEOUT;
		}
		
		return NRF_SUCCESS;
}

int8_t nrf_drv_spi_flash_read(uint8_t command[], uint16_t command_len, uint8_t data[], uint16_t data_len, uint8_t en_continues_cs_low)
{
		uint32_t err_code; 
		uint8_t dummy = 0x55;
		
		nrf_spi_mngr_transfer_t transfer_cmd[3];
	
		if(data_len<256)
		{
				transfer_cmd[0].p_tx_data = &command[0];
				transfer_cmd[0].tx_length = command_len;
				transfer_cmd[0].p_rx_data = 0;
				transfer_cmd[0].rx_length = 0;
			
				transfer_cmd[1].p_tx_data = 0;
				transfer_cmd[1].tx_length = 0;
				transfer_cmd[1].p_rx_data = &data[0];
				transfer_cmd[1].rx_length = data_len;
			
				bmi160_clear_flash_cs_pin();
				err_code = nrf_spi_mngr_perform(&m_nrf_spi_mngr2, &transfer_cmd[0], 1, NULL);
				APP_ERROR_CHECK(err_code);
				err_code = nrf_spi_mngr_perform(&m_nrf_spi_mngr2, &transfer_cmd[1], 1, NULL);
				APP_ERROR_CHECK(err_code);
				bmi160_set_flash_cs_pin();
		}
		else if(data_len<512)
		{
				uint8_t len_for_read = data_len%255;
			
				transfer_cmd[0].p_tx_data = &command[0];
				transfer_cmd[0].tx_length = command_len;
				transfer_cmd[0].p_rx_data = 0;
				transfer_cmd[0].rx_length = 0;
			
				transfer_cmd[1].p_tx_data = 0;
				transfer_cmd[1].tx_length = 0;
				transfer_cmd[1].p_rx_data = &data[0];
				transfer_cmd[1].rx_length = 255;
				
				transfer_cmd[2].p_tx_data = &dummy;
				transfer_cmd[2].tx_length = 1;
				transfer_cmd[2].p_rx_data = &data[255];
				transfer_cmd[2].rx_length = len_for_read;
				bmi160_clear_flash_cs_pin();
				err_code = nrf_spi_mngr_perform(&m_nrf_spi_mngr2, &transfer_cmd[0], 1, NULL);
				APP_ERROR_CHECK(err_code);
				err_code = nrf_spi_mngr_perform(&m_nrf_spi_mngr2, &transfer_cmd[1], 1, NULL);
			  APP_ERROR_CHECK(err_code);
				err_code = nrf_spi_mngr_perform(&m_nrf_spi_mngr2, &transfer_cmd[2], 1, NULL);
				APP_ERROR_CHECK(err_code); 
				bmi160_set_flash_cs_pin();
		}
		
	
//		if(data_len<256)
//		{
//			
//				
//				nrf_spi_mngr_transfer_t transfer_cmd[2] =
//				{
//						NRF_SPI_MNGR_TRANSFER(&command[0], command_len, NULL, 0),
//						NRF_SPI_MNGR_TRANSFER(dummy, 0, &data[0], data_len)
//				};
//				bmi160_clear_flash_cs_pin();
//				err_code = nrf_spi_mngr_perform(&m_nrf_spi_mngr2, &transfer_cmd[0], 2, NULL);
//				APP_ERROR_CHECK(err_code);
//				bmi160_set_flash_cs_pin();
//		}else if(data_len<512)
//		{
//				uint8_t len_for_read = data_len%255;
//				NRF_LOG_INFO("Len Read %d", len_for_read);
//				nrf_spi_mngr_transfer_t transfer_cmd[3] =
//				{
//						NRF_SPI_MNGR_TRANSFER(&command[0], command_len, NULL, 0),
//						NRF_SPI_MNGR_TRANSFER(NULL, 0, &data[0], 255),
//					  NRF_SPI_MNGR_TRANSFER(dummy, 1, &data[255], 1),
//				};
//				bmi160_clear_flash_cs_pin();
//				//err_code = nrf_spi_mngr_perform(&m_nrf_spi_mngr2, &transfer_cmd[0], 1, NULL);
//				//err_code = nrf_spi_mngr_perform(&m_nrf_spi_mngr2, &transfer_cmd[1], 1, NULL);
//				err_code = nrf_spi_mngr_perform(&m_nrf_spi_mngr2, &transfer_cmd[2], 1, NULL);
//				APP_ERROR_CHECK(err_code);
//				bmi160_set_flash_cs_pin();
//		}else{
//				//You can impredament this when data more then 1023
//		}
		
		return NRF_SUCCESS;
}

int8_t nrf_drv_spi_flash_write(uint8_t command[], uint16_t command_len, uint8_t data[], uint16_t data_len, uint8_t en_continues_cs_low)
{
		uint32_t err_code; 
	
		if(command_len<256)
		{
				nrf_spi_mngr_transfer_t const transfer_cmd[1] =
				{
						NRF_SPI_MNGR_TRANSFER(&command[0], command_len, &data[0], data_len),
				};
				bmi160_clear_flash_cs_pin();
				err_code = nrf_spi_mngr_perform(&m_nrf_spi_mngr2, &transfer_cmd[0], 1, NULL);
				APP_ERROR_CHECK(err_code);
				if(en_continues_cs_low == 0)
				{
						bmi160_set_flash_cs_pin();
				}
		}
		else if(command_len<512){
				uint8_t len = (command_len%255);
				nrf_spi_mngr_transfer_t const transfer_cmd[2] =
				{
						NRF_SPI_MNGR_TRANSFER(&command[0], 255, NULL, 0),
						NRF_SPI_MNGR_TRANSFER(&command[255], len, NULL, 0),
				};
				bmi160_clear_flash_cs_pin();
				err_code = nrf_spi_mngr_perform(&m_nrf_spi_mngr2, &transfer_cmd[0], 2, NULL);
				//err_code = nrf_spi_mngr_perform(&m_nrf_spi_mngr2, &transfer_cmd[1], 1, NULL);
				
				APP_ERROR_CHECK(err_code);
				if(en_continues_cs_low == 0)
				{
						bmi160_set_flash_cs_pin();
				}
		}
		else{
				//You can impredament this when data more then 1023
		}
	
		return NRF_SUCCESS;
}

uint8_t nrf_dev_flash_spi_wakeup(at45dbxx_dev_t *p_at45dbxx_dev)
{
		nrf_gpio_pin_clear(p_a2_spi_comm->cs_flash_pin);
		p_at45dbxx_dev->delay_ms(1);
		nrf_gpio_pin_set(p_a2_spi_comm->cs_flash_pin);
	
		return NRF_SUCCESS;
}

uint8_t nrf_drv_flash_spi_init(nrf_drv_a2_spi_comm_t *p_comm, at45dbxx_dev_t *p_at45dbxx_dev)
{
		nrf_gpio_cfg_output(p_comm->cs_flash_pin);
		nrf_gpio_cfg_output(p_comm->wp_flash_pin);
		nrf_gpio_cfg_output(p_comm->reset_flash_pin);
		nrf_gpio_pin_set(p_comm->cs_flash_pin);
		nrf_gpio_pin_set(p_comm->wp_flash_pin);
		nrf_gpio_pin_set(p_comm->reset_flash_pin);
	
		p_at45dbxx_dev->read 				= nrf_drv_spi_flash_read;
		p_at45dbxx_dev->write 			= nrf_drv_spi_flash_write;
		p_at45dbxx_dev->delay_ms 		= nrf_delay_ms;
		
		return 0;
}



int8_t flash_erase_page(uint16_t page_addr)
{
		uint8_t res;
		
		res = at45db_erase_page(&m_at45dbxx_dev, 0);
		
		return res;
}

void flash_wakeup()
{
		
}

void flash_sleep()
{

}
	
int8_t flash_erase_chip(void)
{
		uint8_t res;
		
		res = at45db_erase_chip(&m_at45dbxx_dev);
	
		return res;
}

int8_t flash_write(uint16_t page_addr, uint8_t write_data[], uint16_t len)
{
		uint8_t res;
	
		res = at45db_write_buffer(&m_at45dbxx_dev, 0, write_data, len);
		res = at45db_buffer_to_page(&m_at45dbxx_dev, page_addr);
	
		return res;
}

int8_t flash_read(uint16_t page_addr, uint8_t read_data[], uint16_t len)
{
		uint8_t res; 
		
		res = at45db_page_to_buf(&m_at45dbxx_dev, page_addr);
	
		//for(uint16_t i=0;i<len;i++)
		{
				at45db_read_buffer(&m_at45dbxx_dev, 0, &read_data[0], len);	
		}
   // res = at45db_read_buffer(&m_at45dbxx_dev, 0, read_data, len);	
		//at45db_read_page_bypassed(&m_at45dbxx_dev, page_addr, read_data, len);
	
	  return res;
}


//uint8_t a2_write_defualt_config(const a2_sensor_config *p_config)
//{
//		//Wait Implement
//		return 0;
//}

//uint8_t a2_read_sensor_config(a2_sensor_config *p_config)
//{
//		//Wait Implement
//		return 0;
//}
