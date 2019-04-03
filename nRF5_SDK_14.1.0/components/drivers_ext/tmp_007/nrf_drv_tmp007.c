#include "nrf_drv_tmp007.h"

static app_twi_t *p_tmp_twi;

int8_t tmp007_i2c_read(uint8_t dev_addr, uint8_t reg_addr,
		uint8_t *data, uint16_t len)
{
		ret_code_t err_code;
		uint8_t buffer[255];
	
    app_twi_transfer_t const transfers[] =
    {
        APP_TWI_WRITE(dev_addr, &reg_addr, 1, APP_TWI_NO_STOP),
        APP_TWI_READ (dev_addr, &buffer[0], len, 0)
    };

    err_code = app_twi_perform(p_tmp_twi, transfers, sizeof(transfers) / sizeof(transfers[0]), NULL);
		
		if (err_code != NRF_SUCCESS)
		{
				return err_code;
		}

		memcpy(&data[0], &buffer[0],len);
		
		return TMP007_SUCCESS;
}

int8_t tmp007_i2c_write(uint8_t dev_addr, uint8_t reg_addr,
		uint8_t *data, uint16_t len)
{
		ret_code_t err_code;
		uint8_t tx_buffer[255];
	
		tx_buffer[0] = reg_addr;
		memcpy(&tx_buffer[1], &data[0],len);
		
    app_twi_transfer_t const transfers[] = 
    {
        APP_TWI_WRITE(dev_addr, &tx_buffer[0], len+1, 0)
    };

    err_code = app_twi_perform(p_tmp_twi, transfers, sizeof(transfers) / sizeof(transfers[0]), NULL);
		
		return 0;
}

uint8_t tmp007_i2c_init(app_twi_t *p_twi)
{
		p_tmp_twi = p_twi;
}


