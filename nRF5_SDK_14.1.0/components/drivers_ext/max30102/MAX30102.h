/* Copyright (c) 2016 BAESlab Co.,Ltd. All Rights Reserved.
 *
 * The information contained herein is property of BAESlab Co.,Ltd ASA.
 * Terms and conditions of usage are described in detail in BAESlab Co.,Ltd
 * STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted not free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 * Name:  Mr. Wasan Wiyarun Embedded System Engineer
 * Email: w.wiyarun@gmail.com 
 */
 
#ifndef MAX30102_H__
#define MAX30102_H__

#include <stdint.h>
#include <stdbool.h>
#include <nrf_error.h>

#define MAX30102_I2C_ADDR	               0x57 


/*****************************************************************************/
/* type definitions */

typedef enum{
		MAX30102_SUCCESS,
		MAX30102_ERROR_UNSUCCESS,
		MAX30102_ERROR_NOT_FOUND,
		MAX30102_ERROR_NULL_POINTER,
		MAX30102_ERROR_TIMEOUT,
		MAX30102_ERROR_BUSY,
		MAX30102_ERROR_INVALID_ADDR,
		MAX30102_ERROR_INVALID_PARAM,
		MAX30102_ERROR_INVALID_LENGTH 
}max30102_status_t;

//typedef struct 
//{
//		uint8_t menu_fac;
//		uint8_t device_id[2];
//		uint8_t edi[2];
//}at45dbxx_device_info_t;

typedef uint8_t (*max30102_com_fptr_t)(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t len);

typedef void (*max30102_delay_fptr_t)(uint32_t period);

typedef struct {
	/*! Menufacturer ID */
	uint8_t  manufac_id;
	/*! Device Address */	
	uint8_t  device_addr;
	/*! Revission ID */	
	uint8_t revisionID;
	/*! Number of ON LED*/
	uint8_t activeLEDs;
	/*! Read function pointer */
	max30102_com_fptr_t read;
	/*! Write function pointer */
	max30102_com_fptr_t write;
	/*!  Delay function pointer */
	max30102_delay_fptr_t delay_ms;
}max30102_dev_t;

/**@brief Function for configuration isr pin of max30102.
 *
 * @param[in]   max30102_dev_t  Pointer of reading and writing i2c. 
 * 
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
uint8_t max30102_isr_config(max30102_dev_t *max_dev);

/**@brief Function for initializing the max30102 peripheral.
 *
 * @param[in]   max30102_dev_t  Pointer of reading and writing i2c. 
 * 
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
uint8_t max30102_initaial(max30102_dev_t *max_dev);

//uint8_t begin(TwoWire &wirePort = Wire, uint32_t i2cSpeed = I2C_SPEED_STANDARD, uint8_t i2caddr = MAX30102_ADDRESS);
//uint8_t max30102_init(void);

/**@brief Function for report the most recent red value of sensor.
 *
 * @param[in]   NULL.
 * 
 * @return      Returns immediate red value.
 */
uint32_t max30102_getRed(max30102_dev_t *max_dev);

/**@brief Function for report the most recent IR value of sensor.
 *
 * @param[in]   NULL.
 * 
 * @return      Returns immediate IR value.
 */
uint32_t max30102_getIR(max30102_dev_t *max_dev); 

/**@brief Function for report the most recent Green value of sensor.  
 *
 * @param[in]   NULL.
 * 
 * @return      Returns immediate green value.
 */
uint32_t max30102_getGreen(max30102_dev_t *max_dev); 

/**@brief Function for Check for new data but give up after a certain amount of time.
 *
 * @param[in]   maxTimeToCheck the maxinum time to check: uint8_t
 * 
 * @return      Returns true if new data was found, Returns false if new data was not found.
 */
uint8_t safeCheck(max30102_dev_t *max_dev, uint8_t maxTimeToCheck); //Given a max amount of time, check for new data



//////////////// Configuration  /////////////////////////////

/**@brief Function for command to reset sensor device.
 *
 * @param[in]   NULL.
 * 
 * @return      void.
 */
void max30102_softReset(max30102_dev_t *max_dev);

/**@brief Function for command to shutdonw sensor device.
 *
 * @param[in]   NULL.
 * 
 * @return      void.
 */
void max30102_shutDown(max30102_dev_t *max_dev);

/**@brief Function for command to wakeup sensor device.
 *
 * @param[in]   NULL
 * 
 * @return      void.
 */
void max30102_wakeUp(max30102_dev_t *max_dev); 

/**@brief Function for Set which LEDs are used for sampling -- Red only, RED+IR only, or custom.
 *        See datasheet, page 19
 * 
 * @param[in]   mode that is a mode configuration : uint8_t
 * 
 * @return      void.
 */
void setLEDMode(max30102_dev_t *max_dev, uint8_t mode);

/**@brief Function for adcRange: one of MAX30102_ADCRANGE_2048, _4096, _8192, _16384.
 *
 * @param[in]   adcRange is a value for set adc range of sensor: uint8 _t
 * 
 * @return      void.
 */
void setADCRange(max30102_dev_t *max_dev, uint8_t adcRange);

/**@brief Function for sampleRate: one of MAX30102_SAMPLERATE_50, _100, _200, _400, _800, _1000, _1600, _3200.
 *
 * @param[in]   sampleRate is a value for set sample rate of sensor: uint8_t
 * 
 * @return      void.
 */
void setSampleRate(max30102_dev_t *max_dev, uint8_t sampleRate);

/**@brief Function for pulseWidth: one of MAX30102_PULSEWIDTH_69, _188, _215, _411.
 *
 * @param[in]   pulseWidth is a value for set pulse width of sensor: uint8_t
 * 
 * @return      void.
 */
void setPulseWidth(max30102_dev_t *max_dev, uint8_t pulseWidth);

/**@brief Function for set Amplitude values: 0x00 = 0mA, 0x7F = 25.4mA, 0xFF = 50mA (typical).
 *        See datasheet, page 21
 *  
 * @param[in]   value is a value for set pulse amplitude of red led: uint8_t
 * 
 * @return      void.
 */
void setPulseAmplitudeRed(max30102_dev_t *max_dev, uint8_t amplitude);

/**@brief Function for set pulse applitude of IR LED.
 *
 * @param[in]   value is a value for set pulse amplictude of IR: uint8_t
 * 
 * @return      void.
 */
void setPulseAmplitudeIR(max30102_dev_t *max_dev, uint8_t value);

/**@brief Function for set pulse applitude of Green LED.
 *
 * @param[in]   value is a value for set pulse amplictude of green led: uint8_t
 * 
 * @return      void.
 */
void setPulseAmplitudeGreen(max30102_dev_t *max_dev, uint8_t value);

/**@brief Function for set pulse applitude of Proximity.
 *
 * @param[in]   value is a value for set pulse amplictude of proximity: uint8_t
 * 
 * @return      void.
 */
void setPulseAmplitudeProximity(max30102_dev_t *max_dev, uint8_t value);

/**@brief Function for initializing the KIWI board peripheral.
 *
 * @param[in]   comm_params  Pointer to a LED, LDR and PIR communication structure: app_carrot_comm_params_t
 * 
 * @return      void.
 */
void setProximityThreshold(max30102_dev_t *max_dev, uint8_t threshMSB);

//Multi-led configuration mode (page 22)
/**@brief Function for initializing the KIWI board peripheral.
 *
 * @param[in]   comm_params  Pointer to a LED, LDR and PIR communication structure: app_carrot_comm_params_t
 * 
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
void enableSlot(max30102_dev_t *max_dev, uint8_t slotNumber, uint8_t device); //Given slot number, assign a device to slot

/**@brief Function for initializing the KIWI board peripheral.
 *
 * @param[in]   comm_params  Pointer to a LED, LDR and PIR communication structure: app_carrot_comm_params_t
 * 
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
void disableSlots(max30102_dev_t *max_dev);

// Data Collection

//Interrupts (page 13, 14)
/**@brief Function for initializing the KIWI board peripheral.
 *
 * @param[in]   comm_params  Pointer to a LED, LDR and PIR communication structure: app_carrot_comm_params_t
 * 
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
uint8_t max30102_getINT1(max30102_dev_t *max_dev); //Returns the main interrupt group

/**@brief Function for initializing the KIWI board peripheral.
 *
 * @param[in]   comm_params  Pointer to a LED, LDR and PIR communication structure: app_carrot_comm_params_t
 * 
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
uint8_t max30102_getINT2(max30102_dev_t *max_dev); //Returns the temp ready interrupt

/**@brief Function for initializing the KIWI board peripheral.
 *
 * @param[in]   comm_params  Pointer to a LED, LDR and PIR communication structure: app_carrot_comm_params_t
 * 
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
void enableAFULL(max30102_dev_t *max_dev); //Enable/disable individual interrupts

/**@brief Function for initializing the KIWI board peripheral.
 *
 * @param[in]   comm_params  Pointer to a LED, LDR and PIR communication structure: app_carrot_comm_params_t
 * 
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
void disableAFULL(max30102_dev_t *max_dev);

/**@brief Function for initializing the KIWI board peripheral.
 *
 * @param[in]   comm_params  Pointer to a LED, LDR and PIR communication structure: app_carrot_comm_params_t
 * 
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
void enableDATARDY(max30102_dev_t *max_dev);

/**@brief Function for initializing the KIWI board peripheral.
 *
 * @param[in]   comm_params  Pointer to a LED, LDR and PIR communication structure: app_carrot_comm_params_t
 * 
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
void disableDATARDY(max30102_dev_t *max_dev);

/**@brief Function for initializing the KIWI board peripheral.
 *
 * @param[in]   comm_params  Pointer to a LED, LDR and PIR communication structure: app_carrot_comm_params_t
 * 
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
void enableALCOVF(max30102_dev_t *max_dev);

/**@brief Function for initializing the KIWI board peripheral.
 *
 * @param[in]   comm_params  Pointer to a LED, LDR and PIR communication structure: app_carrot_comm_params_t
 * 
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
void disableALCOVF(max30102_dev_t *max_dev);

/**@brief Function for initializing the KIWI board peripheral.
 *
 * @param[in]   comm_params  Pointer to a LED, LDR and PIR communication structure: app_carrot_comm_params_t
 * 
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
void enablePROXINT(max30102_dev_t *max_dev);

/**@brief Function for initializing the KIWI board peripheral.
 *
 * @param[in]   comm_params  Pointer to a LED, LDR and PIR communication structure: app_carrot_comm_params_t
 * 
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
void disablePROXINT(max30102_dev_t *max_dev);

/**@brief Function for initializing the KIWI board peripheral.
 *
 * @param[in]   comm_params  Pointer to a LED, LDR and PIR communication structure: app_carrot_comm_params_t
 * 
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
void enableDIETEMPRDY(max30102_dev_t *max_dev);

/**@brief Function for initializing the KIWI board peripheral.
 *
 * @param[in]   comm_params  Pointer to a LED, LDR and PIR communication structure: app_carrot_comm_params_t
 * 
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
void disableDIETEMPRDY(max30102_dev_t *max_dev);

//FIFO Configuration (page 18)
/**@brief Function for initializing the KIWI board peripheral.
 *
 * @param[in]   comm_params  Pointer to a LED, LDR and PIR communication structure: app_carrot_comm_params_t
 * 
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
void setFIFOAverage(max30102_dev_t *max_dev, uint8_t samples);

/**@brief Function for initializing the KIWI board peripheral.
 *
 * @param[in]   comm_params  Pointer to a LED, LDR and PIR communication structure: app_carrot_comm_params_t
 * 
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
void enableFIFORollover(max30102_dev_t *max_dev);

/**@brief Function for initializing the KIWI board peripheral.
 *
 * @param[in]   comm_params  Pointer to a LED, LDR and PIR communication structure: app_carrot_comm_params_t
 * 
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
void disableFIFORollover(max30102_dev_t *max_dev);

/**@brief Function for initializing the KIWI board peripheral.
 *
 * @param[in]   comm_params  Pointer to a LED, LDR and PIR communication structure: app_carrot_comm_params_t
 * 
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
void setFIFOAlmostFull(max30102_dev_t *max_dev, uint8_t samples);



//FIFO Reading
/**@brief Function for initializing the KIWI board peripheral.
 *
 * @param[in]   comm_params  Pointer to a LED, LDR and PIR communication structure: app_carrot_comm_params_t
 * 
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
uint16_t check(max30102_dev_t *max_dev); //Checks for new data and fills FIFO

/**@brief Function for initializing the KIWI board peripheral.
 *
 * @param[in]   comm_params  Pointer to a LED, LDR and PIR communication structure: app_carrot_comm_params_t
 * 
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
uint8_t max30102_available(max30102_dev_t *max_dev); //Tells caller how many new samples are available (head - tail)

/**@brief Function for initializing the KIWI board peripheral.
 *
 * @param[in]   comm_params  Pointer to a LED, LDR and PIR communication structure: app_carrot_comm_params_t
 * 
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
void max30102_nextSample(max30102_dev_t *max_dev); //Advances the tail of the sense array

/**@brief Function for initializing the KIWI board peripheral.
 *
 * @param[in]   comm_params  Pointer to a LED, LDR and PIR communication structure: app_carrot_comm_params_t
 * 
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
uint32_t max30102_getFIFORed(max30102_dev_t *max_dev); //Returns the FIFO sample pointed to by tail

/**@brief Function for initializing the KIWI board peripheral.
 *
 * @param[in]   comm_params  Pointer to a LED, LDR and PIR communication structure: app_carrot_comm_params_t
 * 
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
uint32_t max30102_getFIFOIR(max30102_dev_t *max_dev); //Returns the FIFO sample pointed to by tail

/**@brief Function for initializing the KIWI board peripheral.
 *
 * @param[in]   comm_params  Pointer to a LED, LDR and PIR communication structure: app_carrot_comm_params_t
 * 
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
uint32_t max30102_getFIFOGreen(max30102_dev_t *max_dev); //Returns the FIFO sample pointed to by tail

/**@brief Function for initializing the KIWI board peripheral.
 *
 * @param[in]   comm_params  Pointer to a LED, LDR and PIR communication structure: app_carrot_comm_params_t
 * 
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
uint8_t getWritePointer(max30102_dev_t *max_dev);

/**@brief Function for initializing the KIWI board peripheral.
 *
 * @param[in]   comm_params  Pointer to a LED, LDR and PIR communication structure: app_carrot_comm_params_t
 * 
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
uint8_t getReadPointer(max30102_dev_t *max_dev);

/**@brief Function for initializing the KIWI board peripheral.
 *
 * @param[in]   comm_params  Pointer to a LED, LDR and PIR communication structure: app_carrot_comm_params_t
 * 
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
void clearFIFO(max30102_dev_t *max_dev); //Sets the read/write pointers to zero

//Proximity Mode Interrupt Threshold
/**@brief Function for initializing the KIWI board peripheral.
 *
 * @param[in]   comm_params  Pointer to a LED, LDR and PIR communication structure: app_carrot_comm_params_t
 * 
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
void setPROXINTTHRESH(max30102_dev_t *max_dev, uint8_t val);

// Die Temperature
/**@brief Function for initializing the KIWI board peripheral.
 *
 * @param[in]   comm_params  Pointer to a LED, LDR and PIR communication structure: app_carrot_comm_params_t
 * 
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
double max30102_readTemperature(max30102_dev_t *max_dev);

/**@brief Function for initializing the KIWI board peripheral.
 *
 * @param[in]   comm_params  Pointer to a LED, LDR and PIR communication structure: app_carrot_comm_params_t
 * 
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
double max30102_readTemperatureF(max30102_dev_t *max_dev);

// Detecting ID/Revision
/**@brief Function for initializing the KIWI board peripheral.
 *
 * @param[in]   comm_params  Pointer to a LED, LDR and PIR communication structure: app_carrot_comm_params_t
 * 
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
uint8_t getRevisionID(max30102_dev_t *max_dev);

/**@brief Function for initializing the KIWI board peripheral.
 *
 * @param[in]   comm_params  Pointer to a LED, LDR and PIR communication structure: app_carrot_comm_params_t
 * 
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
uint8_t max30102_readPartID(max30102_dev_t *max_dev);  


// Setup the IC with user selectable settings
/**@brief Function for initializing the KIWI board peripheral.
 *
 * @param[in]   comm_params  Pointer to a LED, LDR and PIR communication structure: app_carrot_comm_params_t
 * 
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
void max30102_setup(max30102_dev_t *max_dev);
void max30102_setup2(max30102_dev_t *max_dev, uint8_t powerLevel, uint8_t sampleAverage, uint8_t ledMode, int sampleRate, int pulseWidth, int adcRange);

// Low-level I2C communication
/**@brief Function for initializing the KIWI board peripheral.
 *
 * @param[in]   comm_params  Pointer to a LED, LDR and PIR communication structure: app_carrot_comm_params_t
 * 
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
uint8_t readRegister8(max30102_dev_t *max_dev, uint8_t address, uint8_t reg);

// Low-level I2C communication
/**@brief Function for initializing the KIWI board peripheral.
 *
 * @param[in]   comm_params  Pointer to a LED, LDR and PIR communication structure: app_carrot_comm_params_t
 * 
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
uint8_t readRegister(max30102_dev_t *max_dev, uint8_t address, uint8_t reg, uint8_t res[], uint8_t len);

/**@brief Function for initializing the KIWI board peripheral.
 *
 * @param[in]   comm_params  Pointer to a LED, LDR and PIR communication structure: app_carrot_comm_params_t
 * 
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
uint8_t writeRegister8(max30102_dev_t *max_dev, uint8_t address, uint8_t reg, uint8_t value);


//activeLEDs is the number of channels turned on, and can be 1 to 3. 2 is common for Red+IR.
/**@brief Function for initializing the KIWI board peripheral.
 *
 * @param[in]   comm_params  Pointer to a LED, LDR and PIR communication structure: app_carrot_comm_params_t
 * 
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
//int8_t activeLEDs; //Gets set during setup. Allows check() to calculate how many bytes to read from FIFO

/**@brief Function for initializing the KIWI board peripheral.
 *
 * @param[in]   comm_params  Pointer to a LED, LDR and PIR communication structure: app_carrot_comm_params_t
 * 
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
 
//extern uint8_t revisionID; 

/**@brief Function for initializing the KIWI board peripheral.
 *
 * @param[in]   comm_params  Pointer to a LED, LDR and PIR communication structure: app_carrot_comm_params_t
 * 
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
void max30102_readRevisionID(max30102_dev_t *max_dev);

/**@brief Function for initializing the KIWI board peripheral.
 *
 * @param[in]   comm_params  Pointer to a LED, LDR and PIR communication structure: app_carrot_comm_params_t
 * 
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
void bitMask(max30102_dev_t *max_dev, uint8_t reg, uint8_t mask, uint8_t thing);

/**@brief Function for initializing the KIWI board peripheral.
 *
 * @param[in]   comm_params  Pointer to a LED, LDR and PIR communication structure: app_carrot_comm_params_t
 * 
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
uint8_t max30102_read_isr_flag(max30102_dev_t *max_dev);

uint8_t max30102_read_fifo(max30102_dev_t *max_dev);


uint16_t max30102_check(max30102_dev_t *max_dev);

uint32_t getFIFORed(max30102_dev_t *max_dev);

uint32_t getFIFOIR(max30102_dev_t *max_dev);

#endif



