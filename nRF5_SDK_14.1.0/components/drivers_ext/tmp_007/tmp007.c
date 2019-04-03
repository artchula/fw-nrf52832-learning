#include "tmp007.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "nrf_delay.h"


static uint16_t read16(tmp007_dev_t *dev, uint8_t a) {
		uint16_t ret;
		uint8_t rx_i2c[2];
	
		dev->read(dev->addr, a, &rx_i2c[0], sizeof(rx_i2c));
	
		ret = rx_i2c[0] << 8;
		ret = ret | rx_i2c[1]; 

		return ret;
}

static uint32_t write16(tmp007_dev_t *dev, uint8_t a, uint16_t d) {
		uint32_t ret;
		uint8_t tx_i2c[2];
		
		tx_i2c[0] = d>>8;
		tx_i2c[1] = (uint8_t)(d&0x00FF);
		
		ret = dev->write(dev->addr, a, tx_i2c, sizeof(tx_i2c));
	
		return ret;
}

uint32_t tmp007_initial(tmp007_dev_t *dev)
{
		
		uint16_t did;
		did = read16( dev, TMP007_DEVID);
	
		if (did != TMP007_DF_DEVID){
				return TMP007_DEVICE_NOT_FOUND;
		} 
		//NRF_LOG_INFO("TMP007 Device Found");
		//NRF_LOG_FLUSH();
		
		dev->dev_id = did;
		
		
		//uint16_t mem = read16(dev, TMP007_MEMORY );
		
		
		//NRF_LOG_INFO("TMP007 MEM 0x%04X", mem);
		//NRF_LOG_FLUSH();
		
		
		//write16(dev, TMP007_CONFIG, TMP007_CFG_RESET);
		
		//NRF_LOG_INFO("TMP007 Device ::TMP007_CFG_RESET");
		//NRF_LOG_FLUSH();
		//nrf_delay_ms(3000);
		
		//uint16_t volt = read16(dev, TMP007_VOBJ );
		
		
		//NRF_LOG_INFO("TMP007 Device ::%d",volt);
		//NRF_LOG_FLUSH();
		
		
		write16( dev, TMP007_CONFIG, TMP007_CFG_MODEON | TMP007_CFG_ALERTEN | 
		TMP007_CFG_TRANSC | dev->samplerate);
	
	
		//write16( dev, TMP007_CONFIG, TMP007_CFG_MODEON | TMP007_CFG_ALERTEN | 
		//TMP007_CFG_TRANSC | dev->samplerate);
	
		write16( dev, TMP007_STATMASK, TMP007_STAT_ALERTEN |TMP007_STAT_CRTEN);
		// enable conversion ready alert
		
		return TMP007_SUCCESS;
}

uint16_t tmp007_readDeviceID(tmp007_dev_t *dev)
{
		uint16_t device_id;
		
		device_id = read16( dev, TMP007_DEVID);
	
		return device_id;
}

uint16_t tmp007_readStatus(tmp007_dev_t *dev)
{
		uint16_t status;
		
		status = read16( dev, TMP007_STATUS);
	
		return status;
}

int16_t tmp007_readRawDieTemperature(tmp007_dev_t *dev)
{
		int16_t raw;
		
		raw = read16(dev, TMP007_TDIE);
		raw >>= 2;
		
		return raw;
}

int16_t tmp007_readRawVoltage(tmp007_dev_t *dev)
{
		int16_t raw;

		raw = read16(dev, TMP007_VOBJ);
	
		return raw; 
}

double  tmp007_readObjTempC(tmp007_dev_t *dev)
{
		int16_t raw = read16(dev, TMP007_TOBJ);
		// invalid
		if (raw & 0x1){
				return 0;
		}	
		raw >>=2;

		double Tobj = raw;
		Tobj *= 0.03125; // convert to celsius
		
		return Tobj;
}

double  tmp007_readDieTempC(tmp007_dev_t *dev)
{
		double Tdie = tmp007_readRawDieTemperature(dev);
		Tdie *= 0.03125; // convert to celsius
		return Tdie;
}

uint32_t tmp007_power_down(tmp007_dev_t *dev)
{
		uint32_t ret;
			
		uint16_t reg = read16(dev, TMP007_CONFIG);
	
		NRF_LOG_INFO("Config 0x%04X\n",reg);
		//ret = write16( dev, TMP007_CONFIG, reg&0xEFFF); 
		
	
		//ret = write16( dev, TMP007_CONFIG, TMP007_CFG_MODEOFF |TMP007_CFG_ALERT_DISEN); 
		
		return ret;
}

