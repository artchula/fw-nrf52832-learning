#include "nrf_drv_ssd1306.h"



#define MAX_PENDING_TRANSACTIONS    5
#define SPI_BUFFER_SIZE							50

#define MAX_PENDING_TRANSACTIONS    		5
#define I2C_INSTANCE_OLED_ECC 					0 											     	       /**< I2C instance index. */
//static app_twi_t *p_app_twi;
NRF_TWI_MNGR_DEF(m_app_twi, MAX_PENDING_TRANSACTIONS, I2C_INSTANCE_OLED_ECC);


int8_t nrf_drv_i2c_0_read(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t len)
{
		ret_code_t err_code;
		
		NRF_LOG_FLUSH();
    nrf_twi_mngr_transfer_t const transfers[] =
    {
        NRF_TWI_MNGR_WRITE(dev_addr, &reg_addr, 1, NRF_TWI_MNGR_NO_STOP),
        NRF_TWI_MNGR_READ (dev_addr, &data[0], len, 0)
    };

    err_code = nrf_twi_mngr_perform(&m_app_twi, transfers, sizeof(transfers) / sizeof(transfers[0]), NULL);
    
		//NRF_LOG_INFO("Error Code %d\r\n",err_code);
		//NRF_LOG_FLUSH();
	
		return err_code;		
}

int8_t nrf_drv_i2c_0_write(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t len)
{
		ret_code_t err_code;
		uint8_t i2c_buffer[len+1];

		i2c_buffer[0] = reg_addr;
		memcpy(&i2c_buffer[1],&data[0],len);

		nrf_twi_mngr_transfer_t const transfers[] = 
		{
			NRF_TWI_MNGR_WRITE(dev_addr, i2c_buffer, sizeof(i2c_buffer), 0)
		};

		err_code = nrf_twi_mngr_perform(&m_app_twi, transfers, sizeof(transfers) / sizeof(transfers[0]), NULL);
		
		//NRF_LOG_INFO("Error Code %d\r\n",err_code);
		//NRF_LOG_FLUSH();
		
		return (uint8_t)err_code;
}


uint8_t nrf_drv_a2_i2c_0_init(nrf_drv_a2_i2c_0_comm_t *p_comm)
{
		uint32_t err_code;

	
	
		nrf_drv_twi_config_t const config = {
			.scl                = p_comm->scl_pin,
			.sda                = p_comm->sda_pin,
			.frequency          = NRF_TWI_FREQ_400K,
			.interrupt_priority = APP_IRQ_PRIORITY_LOWEST,
			.clear_bus_init     = false
		};

		err_code = nrf_twi_mngr_init(&m_app_twi, &config);
		APP_ERROR_CHECK(err_code);
				
	
		
		return NRF_SUCCESS;
}

uint8_t nrf_drv_a2_i2c_0_deinit(nrf_drv_a2_i2c_0_comm_t *p_comm)
{
	  nrf_twi_mngr_uninit(&m_app_twi);
		
		nrf_gpio_cfg_output(p_comm->oled_power_en_pin);
		nrf_gpio_pin_set(p_comm->oled_power_en_pin);
		nrf_gpio_cfg_output(p_comm->scl_pin);
		nrf_gpio_pin_set(p_comm->scl_pin);	
		nrf_gpio_cfg_output(p_comm->sda_pin);
		nrf_gpio_pin_set(p_comm->sda_pin);	
		
//			nrf_gpio_cfg_output(OLED_POWER_EN_PIN);
//		nrf_gpio_pin_set(OLED_POWER_EN_PIN);
//						
//		nrf_gpio_cfg_output(OLED_SCL_PIN);
//		nrf_gpio_pin_set(OLED_SCL_PIN);	
//		nrf_gpio_cfg_output(OLED_SDA_PIN);
//		nrf_gpio_pin_set(OLED_SDA_PIN);	
		
	
		return 0;
}

uint8_t nrf_drv_a2_oled_init(nrf_drv_a2_i2c_0_comm_t *p_comm, oled_ssd1306_dev_t *oled_dev)
{
		nrf_gpio_cfg_output(p_comm->oled_power_en_pin);
		nrf_gpio_pin_clear(p_comm->oled_power_en_pin);
		nrf_delay_ms(100);
	
	
		oled_dev->id 				= OLED_SSD1306_I2C_ADDRESS;
		oled_dev->delay_ms 	= nrf_delay_ms;
		oled_dev->read 			= nrf_drv_i2c_0_read;
		oled_dev->write 		= nrf_drv_i2c_0_write;
		oled_dev->vccstate 	= SSD1306_SWITCHCAPVCC;
	
		ssd1306_begin(oled_dev, oled_dev->vccstate, oled_dev->id, false);
	
		return 0;
}

uint8_t nrf_drv_a2_oled_shutdown(void)
{
		ssd1306_command(SSD1306_DISPLAYOFF);//--turn ff oled panel
		//ssd1306_dim(1);
		return 0;
}

uint8_t nrf_drv_a2_oled_dim(uint8_t enable)
{
		ssd1306_dim(enable);
		return 0;
}










