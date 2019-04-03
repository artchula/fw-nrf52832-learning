#ifndef __TMP007_H__
#define __TMP007_H__

#include "stdint.h"

// uncomment for debugging!
//#define TMP007_DEBUG 1

#define TMP007_VOBJ       			0x00
#define TMP007_TDIE       			0x01
#define TMP007_CONFIG     			0x02
#define TMP007_TOBJ       			0x03
#define TMP007_STATUS     			0x04
#define TMP007_STATMASK   			0x05
#define TMP007_MEMORY      			0x2A


#define TMP007_CFG_RESET    		0x8000
#define TMP007_CFG_MODEON   		0x1000
#define TMP007_CFG_1SAMPLE  		0x0000
#define TMP007_CFG_2SAMPLE  		0x0200
#define TMP007_CFG_4SAMPLE  		0x0400
#define TMP007_CFG_8SAMPLE  		0x0600
#define TMP007_CFG_16SAMPLE 		0x0800
#define TMP007_CFG_ALERTEN  		0x0100
#define TMP007_CFG_ALERTF   		0x0080
#define TMP007_CFG_TRANSC   		0x0040

#define TMP007_STAT_ALERTEN 		0x8000
#define TMP007_STAT_CRTEN   		0x4000

#define TMP007_CFG_MODEOFF     	0x0000
#define TMP007_CFG_ALERT_DISEN	0x0000

#define TMP007_I2CADDR 					0x40
#define TMP007_DEVID 						0x1F


#define TMP007_DF_DEVID				0x0078

typedef enum{
		TMP007_SUCCESS,
		TMP007_DEVICE_NOT_FOUND,
		TMP007_LEN_NOT_MATCH,
}tmp007_status_t;

typedef struct{
		uint16_t samplerate;
}tmp007_cfg_t;

typedef uint8_t (*tmp007_com_fptr_t)(uint8_t dev_addr, uint8_t reg_addr,
		uint8_t *data, uint16_t len);

typedef void (*tmp007_delay_fptr_t)(uint32_t period);

typedef struct{
		/*! Device Address */
		uint8_t addr;
		/*! Chip Id */
		uint8_t menufac_id;
		/*! Device Id */
		uint8_t dev_id;
		/*! Sample Rate */
		uint16_t samplerate;
		/*! Read function pointer */
		tmp007_com_fptr_t read;
		/*! Write function pointer */
		tmp007_com_fptr_t write;
		/*!  Delay function pointer */
		tmp007_delay_fptr_t delay_ms;
}tmp007_dev_t;


uint32_t tmp007_initial(tmp007_dev_t *dev);  // by default go highres
int16_t tmp007_readRawDieTemperature(tmp007_dev_t *dev);
int16_t tmp007_readRawVoltage(tmp007_dev_t *dev);
double  tmp007_readObjTempC(tmp007_dev_t *dev);
double  tmp007_readDieTempC(tmp007_dev_t *dev);
uint32_t tmp007_power_down(tmp007_dev_t *dev);
uint16_t tmp007_readDeviceID(tmp007_dev_t *dev);
uint16_t tmp007_readStatus(tmp007_dev_t *dev);

//uint8_t _addr;
//uint16_t read16(uint8_t addr);
//void write16(uint8_t addr, uint16_t data);

#endif

