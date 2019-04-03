#include "nrf_drv_i2c_max_temp.h"



//static app_twi_t *p_app_twi;



uint32_t nrf_drv_a2_i2c_1_init(nrf_drv_a2_max_tmp_comm_t *p_comm)
{
		uint32_t err_code;

		nrf_drv_twi_config_t const config = {
			.scl                = p_comm->scl_pin,
			.sda                = p_comm->sda_pin,
			.frequency          = NRF_TWI_FREQ_400K,
			.interrupt_priority = APP_IRQ_PRIORITY_LOWEST,
			.clear_bus_init     = false
		};

		err_code = nrf_twi_mngr_init(&m_app_twi_1, &config);
		APP_ERROR_CHECK(err_code);

		return err_code;
}

uint32_t nrf_drv_a2_i2c_1_deinit(nrf_drv_a2_max_tmp_comm_t *p_comm)
{
		nrf_twi_mngr_uninit(&m_app_twi_1);
		
	
		nrf_gpio_cfg_output(p_comm->scl_pin);
		nrf_gpio_cfg_output(p_comm->sda_pin);
	
		nrf_gpio_cfg_output(p_comm->isr_max30102_pin);
		nrf_gpio_cfg_output(p_comm->isr_temp007_pin);
		
		nrf_gpio_pin_set(p_comm->scl_pin);
		nrf_gpio_pin_set(p_comm->sda_pin);
		nrf_gpio_pin_set(p_comm->isr_max30102_pin);
		nrf_gpio_pin_set(p_comm->isr_temp007_pin);
	
		
		return NRF_SUCCESS;
}

uint8_t nrf_drv_i2c_max30102_read(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t len)
{
		ret_code_t err_code;
		
    nrf_twi_mngr_transfer_t const transfers[] =
    {
        NRF_TWI_MNGR_WRITE(dev_addr, &reg_addr, 1, NRF_TWI_MNGR_NO_STOP),
        NRF_TWI_MNGR_READ (dev_addr, &data[0], len, 0)
    };

    err_code = nrf_twi_mngr_perform(&m_app_twi_1, transfers, sizeof(transfers) / sizeof(transfers[0]), NULL);
    
		if(err_code != NRF_SUCCESS)
		{
				return MAX30102_ERROR_NOT_FOUND;
		}
		
		return MAX30102_SUCCESS;
}

uint8_t nrf_drv_i2c_max30102_write(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t len)
{
		ret_code_t err_code;
		uint8_t i2c_buffer[len+1];  

		i2c_buffer[0] = reg_addr;
		memcpy(&i2c_buffer[1],&data[0],len);

		nrf_twi_mngr_transfer_t const transfers[] = 
		{
			NRF_TWI_MNGR_WRITE(dev_addr, i2c_buffer, sizeof(i2c_buffer), 0)
		};

		err_code = nrf_twi_mngr_perform(&m_app_twi_1, transfers, sizeof(transfers) / sizeof(transfers[0]), NULL);
			
		if(err_code != NRF_SUCCESS)
		{
				return MAX30102_ERROR_NOT_FOUND;
		}
		
		return MAX30102_SUCCESS;
}

uint32_t nrf_drv_max30102_reg_i2c_func(nrf_drv_a2_max_tmp_comm_t *p_comm, max30102_dev_t *max_dev)
{
		max_dev->device_addr = MAX30102_I2C_ADDR;
		max_dev->delay_ms 	 = nrf_delay_ms;
		max_dev->read 			 = nrf_drv_i2c_max30102_read;
		max_dev->write 			 = nrf_drv_i2c_max30102_write;
	
	  nrf_gpio_cfg_output(p_comm->en_power_1V8_max30102_pin);
		nrf_gpio_pin_set(p_comm->en_power_1V8_max30102_pin);
	
		return NRF_SUCCESS;
}

uint8_t nrf_drv_i2c_max30102_shutdown(nrf_drv_a2_max_tmp_comm_t *p_comm, max30102_dev_t *max_dev)
{
		setPulseAmplitudeRed(max_dev, 0);
		setPulseAmplitudeIR(max_dev, 0);
		setPulseAmplitudeGreen(max_dev, 0);
		setPulseAmplitudeProximity(max_dev, 0);
	
		max30102_shutDown(max_dev);
	  max30102_shutDown(max_dev);
	  nrf_delay_ms(100);
		nrf_gpio_cfg_output(p_comm->en_power_1V8_max30102_pin);
		nrf_gpio_pin_clear(p_comm->en_power_1V8_max30102_pin);
		return NRF_SUCCESS;
}


uint32_t nrf_drv_temp007_reg_i2c_func(nrf_drv_a2_max_tmp_comm_t *p_comm, tmp007_dev_t *tmp_dev)
{
		tmp_dev->dev_id			= 0;
		tmp_dev->addr				= TMP007_I2CADDR;
		tmp_dev->read 			=	nrf_drv_i2c_temp007_read;	
		tmp_dev->write			= nrf_drv_i2c_temp007_write;
		tmp_dev->delay_ms		= nrf_delay_ms;
		tmp_dev->samplerate = TMP007_CFG_1SAMPLE;
	
		return NRF_SUCCESS;
}

uint8_t nrf_drv_i2c_temp007_read(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t len)
{
		ret_code_t err_code;
		
		NRF_LOG_FLUSH();
    nrf_twi_mngr_transfer_t const transfers[] =
    {
        NRF_TWI_MNGR_WRITE(dev_addr, &reg_addr, 1, NRF_TWI_MNGR_NO_STOP),
        NRF_TWI_MNGR_READ (dev_addr, &data[0], len, 0)
    };

    err_code = nrf_twi_mngr_perform(&m_app_twi_1, transfers, sizeof(transfers) / sizeof(transfers[0]), NULL);
    
		if(err_code != NRF_SUCCESS)
		{
				return TMP007_DEVICE_NOT_FOUND;
		}
		return TMP007_SUCCESS;
}

uint8_t nrf_drv_i2c_temp007_write(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t len)
{
		ret_code_t err_code;
		uint8_t i2c_buffer[len+1];  

		i2c_buffer[0] = reg_addr;
		memcpy(&i2c_buffer[1],&data[0],len);

		nrf_twi_mngr_transfer_t const transfers[] = 
		{
			NRF_TWI_MNGR_WRITE(dev_addr, i2c_buffer, sizeof(i2c_buffer), 0)
		};

		err_code = nrf_twi_mngr_perform(&m_app_twi_1, transfers, sizeof(transfers) / sizeof(transfers[0]), NULL);
			
		if(err_code != NRF_SUCCESS)
		{
				return TMP007_DEVICE_NOT_FOUND;
		}
		return TMP007_SUCCESS;
}		






