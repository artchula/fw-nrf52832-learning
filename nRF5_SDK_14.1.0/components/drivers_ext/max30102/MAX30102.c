#include "MAX30102.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"

// Max30102 POR STATE
const uint8_t MAX30102_PART_ID	= 0x15;

// Status Registers
const uint8_t MAX30102_INTSTAT1 =		0x00;
const uint8_t MAX30102_INTSTAT2 =		0x01;
const uint8_t MAX30102_INTENABLE1 =		0x02;
const uint8_t MAX30102_INTENABLE2 =		0x03;

// FIFO Registers
const uint8_t MAX30102_FIFOWRITEPTR = 	0x04;
const uint8_t MAX30102_FIFOOVERFLOW = 	0x05;
const uint8_t MAX30102_FIFOREADPTR = 	0x06;
const uint8_t MAX30102_FIFODATA =		0x07;

// Configuration Registers
const uint8_t MAX30102_FIFOCONFIG = 		0x08;
const uint8_t MAX30102_MODECONFIG = 		0x09;
const uint8_t MAX30102_PARTICLECONFIG = 	0x0A;    // Note, sometimes listed as "SPO2" config in datasheet (pg. 11)
const uint8_t MAX30102_LED1_PULSEAMP = 	0x0C;
const uint8_t MAX30102_LED2_PULSEAMP = 	0x0D;
const uint8_t MAX30102_LED3_PULSEAMP = 	0x0E;
const uint8_t MAX30102_LED_PROX_AMP = 	0x10;
const uint8_t MAX30102_MULTILEDCONFIG1 = 0x11;
const uint8_t MAX30102_MULTILEDCONFIG2 = 0x12;

// Die Temperature Registers
const uint8_t MAX30102_DIETEMPINT = 		0x1F;
const uint8_t MAX30102_DIETEMPFRAC = 	0x20;
const uint8_t MAX30102_DIETEMPCONFIG = 	0x21;

// Proximity Function Registers
const uint8_t MAX30102_PROXINTTHRESH = 	0x30;

// Part ID Registers
const uint8_t MAX30102_REVISIONID = 		0xFE;
const uint8_t MAX30102_PARTID = 			0xFF;    // Should always be 0x15. Identical to MAX30102.

// MAX30102 Commands
// Interrupt configuration (pg 13, 14)
const uint8_t MAX30102_INT_A_FULL_MASK =		(uint8_t)(~0x80);
const uint8_t MAX30102_INT_A_FULL_ENABLE = 	0x80;
const uint8_t MAX30102_INT_A_FULL_DISABLE = 	0x00;

const uint8_t MAX30102_INT_DATA_RDY_MASK = (uint8_t)~0x40;
const uint8_t MAX30102_INT_DATA_RDY_ENABLE =	0x40;
const uint8_t MAX30102_INT_DATA_RDY_DISABLE = 0x00;

const uint8_t MAX30102_INT_ALC_OVF_MASK = (uint8_t)~0x20;
const uint8_t MAX30102_INT_ALC_OVF_ENABLE = 	0x20;
const uint8_t MAX30102_INT_ALC_OVF_DISABLE = 0x00;

const uint8_t MAX30102_INT_PROX_INT_MASK = (uint8_t)~0x10;
const uint8_t MAX30102_INT_PROX_INT_ENABLE = 0x10;
const uint8_t MAX30102_INT_PROX_INT_DISABLE = 0x00;

const uint8_t MAX30102_INT_DIE_TEMP_RDY_MASK = (uint8_t)~0x02;
const uint8_t MAX30102_INT_DIE_TEMP_RDY_ENABLE = 0x02;
const uint8_t MAX30102_INT_DIE_TEMP_RDY_DISABLE = 0x00;

const uint8_t MAX30102_SAMPLEAVG_MASK =	(int8_t)~0xE0;
const uint8_t MAX30102_SAMPLEAVG_1 = 	0x00;
const uint8_t MAX30102_SAMPLEAVG_2 = 	0x20;
const uint8_t MAX30102_SAMPLEAVG_4 = 	0x40;
const uint8_t MAX30102_SAMPLEAVG_8 = 	0x60;
const uint8_t MAX30102_SAMPLEAVG_16 = 	0x80;
const uint8_t MAX30102_SAMPLEAVG_32 = 	0xA0;

const uint8_t MAX30102_ROLLOVER_MASK = 	0xEF;
const uint8_t MAX30102_ROLLOVER_ENABLE = 0x10;
const uint8_t MAX30102_ROLLOVER_DISABLE = 0x00;

const uint8_t MAX30102_A_FULL_MASK = 	0xF0;

// Mode configuration commands (page 19)
const uint8_t MAX30102_SHUTDOWN_MASK = 	0x7F;
const uint8_t MAX30102_SHUTDOWN = 		0x80;
const uint8_t MAX30102_WAKEUP = 			0x00;

const uint8_t MAX30102_RESET_MASK = 		0xBF;
const uint8_t MAX30102_RESET = 			0x40;

const uint8_t MAX30102_MODE_MASK = 		0xF8;
const uint8_t MAX30102_MODE_REDONLY = 	0x02;
const uint8_t MAX30102_MODE_REDIRONLY = 	0x03;
const uint8_t MAX30102_MODE_MULTILED = 	0x07;

// Particle sensing configuration commands (pgs 19-20)
const uint8_t MAX30102_ADCRANGE_MASK = 	0x9F;
const uint8_t MAX30102_ADCRANGE_2048 = 	0x00;
const uint8_t MAX30102_ADCRANGE_4096 = 	0x20;
const uint8_t MAX30102_ADCRANGE_8192 = 	0x40;
const uint8_t MAX30102_ADCRANGE_16384 = 	0x60;

const uint8_t MAX30102_SAMPLERATE_MASK = 0xE3;
const uint8_t MAX30102_SAMPLERATE_50 = 	0x00;
const uint8_t MAX30102_SAMPLERATE_100 = 	0x04;
const uint8_t MAX30102_SAMPLERATE_200 = 	0x08;
const uint8_t MAX30102_SAMPLERATE_400 = 	0x0C;
const uint8_t MAX30102_SAMPLERATE_800 = 	0x10;
const uint8_t MAX30102_SAMPLERATE_1000 = 0x14;
const uint8_t MAX30102_SAMPLERATE_1600 = 0x18;
const uint8_t MAX30102_SAMPLERATE_3200 = 0x1C;

const uint8_t MAX30102_PULSEWIDTH_MASK = 0xFC;
const uint8_t MAX30102_PULSEWIDTH_69 = 	0x00;
const uint8_t MAX30102_PULSEWIDTH_118 = 	0x01;
const uint8_t MAX30102_PULSEWIDTH_215 = 	0x02;
const uint8_t MAX30102_PULSEWIDTH_411 = 	0x03;

//Multi-LED Mode configuration (pg 22)
const uint8_t MAX30102_SLOT1_MASK = 		0xF8;
const uint8_t MAX30102_SLOT2_MASK = 		0x8F;
const uint8_t MAX30102_SLOT3_MASK = 		0xF8;
const uint8_t MAX30102_SLOT4_MASK = 		0x8F;

const uint8_t SLOT_NONE = 				0x00;
const uint8_t SLOT_RED_LED = 			0x01;
const uint8_t SLOT_IR_LED = 				0x02;
const uint8_t SLOT_GREEN_LED = 			0x03;
const uint8_t SLOT_NONE_PILOT = 			0x04;
const uint8_t SLOT_RED_PILOT =			0x05;
const uint8_t SLOT_IR_PILOT = 			0x06;
const uint8_t SLOT_GREEN_PILOT = 		0x07;

const uint8_t MAX_30102_EXPECTEDPARTID = 0x15;
const uint8_t I2C_BUFFER_LENGTH = 32;
//The MAX30102 stores up to 32 samples on the IC
//This is additional local storage to the microcontroller
const int STORAGE_SIZE = 32; //Each long is 4 bytes so limit this to fit on your micro
struct Record
{
  uint32_t red[STORAGE_SIZE];
  uint32_t IR[STORAGE_SIZE];
  uint32_t green[STORAGE_SIZE];
  uint8_t head;
  uint8_t tail;
} sense; //This is our circular buffer of readings from the sensor

//static uint8_t revisionID;
//static uint8_t activeLEDs;	

#define MAX_PENDING_TRANSACTIONS    		5
#define I2C_INSTANCE_MAX_TEMP 					1 											     	       /**< I2C instance index. */											     	       /**< I2C instance index. */
//static app_twi_t *p_app_twi;
//APP_TWI_DEF(m_app_twi_1, MAX_PENDING_TRANSACTIONS, I2C_INSTANCE_MAX_TEMP);




//uint8_t begin(TwoWire &wirePort = Wire, uint32_t i2cSpeed = I2C_SPEED_STANDARD, uint8_t i2caddr = MAX30102_ADDRESS);
//uint8_t max30102_init(void);


uint32_t max30102_getRed(max30102_dev_t *max_dev)
{
		//Check the sensor for new data for 250ms
		if(safeCheck(max_dev, 250))
				return (sense.red[sense.head]);
		else
				return(0); //Sensor failed to find new data
}

uint32_t max30102_getIR(max30102_dev_t *max_dev)
{
		//Check the sensor for new data for 250ms
		if(safeCheck(max_dev, 250))
				return (sense.IR[sense.head]);
		else
				return(0); //Sensor failed to find new data
}
uint32_t max30102_getGreen(max30102_dev_t *max_dev)
{
		//Check the sensor for new data for 250ms
		if(safeCheck(max_dev, 250))
				return (sense.green[sense.head]);
		else
				return(0); //Sensor failed to find new data
}

uint8_t safeCheck(max30102_dev_t *max_dev, uint8_t maxTimeToCheck)
{
		uint32_t timeout = maxTimeToCheck;

		while(timeout--)
		{
				if(check(max_dev) > 0) //We found new data!
						return 1;
				max_dev->delay_ms(1);
		}
		return 0;
}


//////////////// Configuration  ////////////////

void max30102_softReset(max30102_dev_t *max_dev)
{
		bitMask(max_dev, MAX30102_MODECONFIG, MAX30102_RESET_MASK, MAX30102_RESET);

		// Poll for bit to clear, reset is then complete
		// Timeout after 100ms
		uint8_t timeout = 100;
		while (timeout--)
		{
				uint8_t response = readRegister8(max_dev, max_dev->device_addr, MAX30102_MODECONFIG);

				if ((response & MAX30102_RESET) == 0)
				{
						break;
				}
				max_dev->delay_ms(1); //Let's not over burden the I2C bus
		}
}

void max30102_shutDown(max30102_dev_t *max_dev)
{
		// Put IC into low power mode (datasheet pg. 19)
		// During shutdown the IC will continue to respond to I2C commands but will
		// not update with or take new readings (such as temperature)
		bitMask(max_dev, MAX30102_MODECONFIG, MAX30102_SHUTDOWN_MASK, MAX30102_SHUTDOWN);
}

void max30102_wakeUp(max30102_dev_t *max_dev)
{
		// Pull IC out of low power mode (datasheet pg. 19)
		bitMask(max_dev, MAX30102_MODECONFIG, MAX30102_SHUTDOWN_MASK, MAX30102_WAKEUP);
}

void setLEDMode(max30102_dev_t *max_dev, uint8_t mode)
{
		// Set which LEDs are used for sampling -- Red only, RED+IR only, or custom.
		// See datasheet, page 19
		bitMask(max_dev, MAX30102_MODECONFIG, MAX30102_MODE_MASK, mode);
}

void setADCRange(max30102_dev_t *max_dev, uint8_t adcRange)
{
		// adcRange: one of MAX30102_ADCRANGE_2048, _4096, _8192, _16384
		bitMask(max_dev, MAX30102_PARTICLECONFIG, MAX30102_ADCRANGE_MASK, adcRange);	
}

void setSampleRate(max30102_dev_t *max_dev, uint8_t sampleRate)
{
		// sampleRate: one of MAX30102_SAMPLERATE_50, _100, _200, _400, _800, _1000, _1600, _3200
		bitMask(max_dev, MAX30102_PARTICLECONFIG, MAX30102_SAMPLERATE_MASK, sampleRate);
}

void setPulseWidth(max30102_dev_t *max_dev, uint8_t pulseWidth)
{
		// pulseWidth: one of MAX30102_PULSEWIDTH_69, _188, _215, _411
		bitMask(max_dev, MAX30102_PARTICLECONFIG, MAX30102_PULSEWIDTH_MASK, pulseWidth);
}

// NOTE: Amplitude values: 0x00 = 0mA, 0x7F = 25.4mA, 0xFF = 50mA (typical)
// See datasheet, page 21
void setPulseAmplitudeRed(max30102_dev_t *max_dev, uint8_t amplitude)
{
		writeRegister8(max_dev, max_dev->device_addr, MAX30102_LED1_PULSEAMP, amplitude);
}

void setPulseAmplitudeIR(max30102_dev_t *max_dev, uint8_t amplitude)
{
		writeRegister8(max_dev, max_dev->device_addr, MAX30102_LED2_PULSEAMP, amplitude);		
}

void setPulseAmplitudeGreen(max30102_dev_t *max_dev, uint8_t amplitude)
{
		writeRegister8(max_dev, max_dev->device_addr, MAX30102_LED3_PULSEAMP, amplitude);
}
void setPulseAmplitudeProximity(max30102_dev_t *max_dev, uint8_t amplitude)
{
		writeRegister8(max_dev, max_dev->device_addr, MAX30102_LED_PROX_AMP, amplitude);
}

void setProximityThreshold(max30102_dev_t *max_dev, uint8_t threshMSB)
{
		// Set the IR ADC count that will trigger the beginning of particle-sensing mode.
		// The threshMSB signifies only the 8 most significant-bits of the ADC count.
		// See datasheet, page 24.
		writeRegister8(max_dev, max_dev->device_addr, MAX30102_PROXINTTHRESH, threshMSB);
}

//Multi-led configuration mode (page 22)
void enableSlot(max30102_dev_t *max_dev, uint8_t slotNumber, uint8_t device)
{

		switch (slotNumber) {
				case (1):
						bitMask(max_dev, MAX30102_MULTILEDCONFIG1, MAX30102_SLOT1_MASK, device);
						break;
				case (2):
						bitMask(max_dev, MAX30102_MULTILEDCONFIG1, MAX30102_SLOT2_MASK, device << 4);
						break;
				case (3):
						bitMask(max_dev, MAX30102_MULTILEDCONFIG2, MAX30102_SLOT3_MASK, device);
						break;
				case (4):
						bitMask(max_dev, MAX30102_MULTILEDCONFIG2, MAX30102_SLOT4_MASK, device << 4);
						break;
				default:
						//Shouldn't be here!
						break;
		}
}

void disableSlots(max30102_dev_t *max_dev)
{
		writeRegister8(max_dev, max_dev->device_addr, MAX30102_MULTILEDCONFIG1, 0);
		writeRegister8(max_dev, max_dev->device_addr, MAX30102_MULTILEDCONFIG2, 0);
}

// Data Collection

//Interrupts (page 13, 14)
uint8_t max30102_getINT1(max30102_dev_t *max_dev)
{
		return (readRegister8(max_dev, max_dev->device_addr, MAX30102_INTSTAT1));
}

uint8_t max30102_getINT2(max30102_dev_t *max_dev)
{
		return (readRegister8(max_dev, max_dev->device_addr, MAX30102_INTSTAT2));
}

void enableAFULL(max30102_dev_t *max_dev)
{
		bitMask(max_dev, MAX30102_INTENABLE1, MAX30102_INT_A_FULL_MASK, MAX30102_INT_A_FULL_ENABLE);
}

void disableAFULL(max30102_dev_t *max_dev)
{
		bitMask(max_dev, MAX30102_INTENABLE1, MAX30102_INT_A_FULL_MASK, MAX30102_INT_A_FULL_DISABLE);
}

void enableDATARDY(max30102_dev_t *max_dev)
{
		bitMask(max_dev, MAX30102_INTENABLE1, MAX30102_INT_DATA_RDY_MASK, MAX30102_INT_DATA_RDY_ENABLE);
}

void disableDATARDY(max30102_dev_t *max_dev)
{
		bitMask(max_dev, MAX30102_INTENABLE1, MAX30102_INT_DATA_RDY_MASK, MAX30102_INT_DATA_RDY_DISABLE);
}

void enableALCOVF(max30102_dev_t *max_dev)
{
		bitMask(max_dev, MAX30102_INTENABLE1, MAX30102_INT_ALC_OVF_MASK, MAX30102_INT_ALC_OVF_ENABLE);
}

void disableALCOVF(max30102_dev_t *max_dev)
{
		bitMask(max_dev, MAX30102_INTENABLE1, MAX30102_INT_ALC_OVF_MASK, MAX30102_INT_ALC_OVF_DISABLE);
}

void enablePROXINT(max30102_dev_t *max_dev)
{
		bitMask(max_dev, MAX30102_INTENABLE1, MAX30102_INT_PROX_INT_MASK, MAX30102_INT_PROX_INT_ENABLE);
}

void disablePROXINT(max30102_dev_t *max_dev)
{
		bitMask(max_dev, MAX30102_INTENABLE1, MAX30102_INT_PROX_INT_MASK, MAX30102_INT_PROX_INT_DISABLE);
}

void enableDIETEMPRDY(max30102_dev_t *max_dev)
{
		bitMask(max_dev, MAX30102_INTENABLE2, MAX30102_INT_DIE_TEMP_RDY_MASK, MAX30102_INT_DIE_TEMP_RDY_ENABLE);
}

void disableDIETEMPRDY(max30102_dev_t *max_dev)
{
		bitMask(max_dev, MAX30102_INTENABLE2, MAX30102_INT_DIE_TEMP_RDY_MASK, MAX30102_INT_DIE_TEMP_RDY_DISABLE);
}

void setFIFOAverage(max30102_dev_t *max_dev, uint8_t numberOfSamples)
{
		bitMask(max_dev, MAX30102_FIFOCONFIG, MAX30102_SAMPLEAVG_MASK, numberOfSamples);
}

void clearFIFO(max30102_dev_t *max_dev)
{
		writeRegister8(max_dev, max_dev->device_addr, MAX30102_FIFOWRITEPTR, 0);
		writeRegister8(max_dev, max_dev->device_addr, MAX30102_FIFOOVERFLOW, 0);
		writeRegister8(max_dev, max_dev->device_addr, MAX30102_FIFOREADPTR, 0);
}

void enableFIFORollover(max30102_dev_t *max_dev)
{
		bitMask(max_dev, MAX30102_FIFOCONFIG, MAX30102_ROLLOVER_MASK, MAX30102_ROLLOVER_ENABLE);
}

void disableFIFORollover(max30102_dev_t *max_dev)
{
		bitMask(max_dev, MAX30102_FIFOCONFIG, MAX30102_ROLLOVER_MASK, MAX30102_ROLLOVER_DISABLE);
}

void setFIFOAlmostFull(max30102_dev_t *max_dev, uint8_t numberOfSamples)
{
		bitMask(max_dev,MAX30102_FIFOCONFIG, MAX30102_A_FULL_MASK, numberOfSamples);
}

uint8_t getWritePointer(max30102_dev_t *max_dev)
{
		return (readRegister8(max_dev, max_dev->device_addr, MAX30102_FIFOWRITEPTR));
}

uint8_t getReadPointer(max30102_dev_t *max_dev)
{
		return (readRegister8(max_dev, max_dev->device_addr, MAX30102_FIFOREADPTR));
}

uint8_t max30102_read_fifo(max30102_dev_t *max_dev)
{
//		uint8_t buffer_i2c[250];
//		uint32_t tempLong;
		
		
//START;
//Send device address + write mode
//Send address of FIFO_WR_PTR;
//REPEATED_START;
//Send device address + read mode
//Read FIFO_WR_PTR;
//STOP;
	
	
//The central processor evaluates the number of samples to be read from the FIFO:
//NUM_AVAILABLE_SAMPLES = FIFO_WR_PTR – FIFO_RD_PTR
//(Note: pointer wrap around should be taken into account)
//NUM_SAMPLES_TO_READ = < less than or equal to NUM_AVAILABLE_SAMPLES >
//Second transaction: Read NUM_SAMPLES_TO_READ samples from the FIFO:
//START;
//Send device address + write mode
//Send address of FIFO_DATA;
//REPEATED_START;
//Send device address + read mode
//for (i = 0; i < NUM_SAMPLES_TO_READ; i++) {
//Read FIFO_DATA;
//Save LED1[23:16];
//Read FIFO_DATA;
//Save LED1[15:8];
//Read FIFO_DATA;
//Save LED1[7:0];
//Read FIFO_DATA;
//Save LED2[23:16];
//Read FIFO_DATA;
//Save LED2[15:8];
//Read FIFO_DATA;
//Save LED2[7:0];
//Read FIFO_DATA;
//}
//STOP;
//START;
//Send device address + write mode
//Send address of FIFO_RD_PTR;
//Write FIFO_RD_PTR;
//STOP;
//		
		return 0;
}

uint16_t max30102_check(max30102_dev_t *max_dev)
{
		uint8_t buffer_i2c[250];
		uint32_t tempLong;
	
		//Read register FIDO_DATA in (3-byte * number of active LED) chunks
		//Until FIFO_RD_PTR = FIFO_WR_PTR
		uint8_t readPointer = getReadPointer(max_dev);
		uint8_t writePointer = getWritePointer(max_dev);
	
	
		int numberOfSamples = 0;
	
		if (readPointer != writePointer)
		{
				//Calculate the number of readings we need to get from sensor
				numberOfSamples = writePointer - readPointer;
				if (numberOfSamples < 0) numberOfSamples += 32; //Wrap condition
			
				//We now have the number of readings, now calc bytes to read
				//For this example we are just doing Red and IR (3 bytes each)
	
  			int bytesLeftToRead = numberOfSamples * max_dev->activeLEDs * 3;
			
			
				//Get ready to read a burst of data from the FIFO register
				//_i2cPort->beginTransmission(MAX30102_ADDRESS);
				//_i2cPort->write(MAX30102_FIFODATA);
				//_i2cPort->endTransmission();
				//nrf_drv_twi_tx(max_twi, _i2caddr, &MAX30102_FIFODATA, sizeof(MAX30102_FIFODATA), false);
				//NRF_LOG_RAW_INFO("\n$writePointer %d,readPointer %d, Get::%d",writePointer, readPointer, numberOfSamples);	
			
			  if(bytesLeftToRead > 0)
				{
						int toGet = bytesLeftToRead;
					
						if (toGet > I2C_BUFFER_LENGTH)
						{
								//If toGet is 32 this is bad because we read 6 bytes (Red+IR * 3 = 6) at a time
								//32 % 6 = 2 left over. We don't want to request 32 bytes, we want to request 30.
								//32 % 9 (Red+IR+GREEN) = 5 left over. We want to request 27.
								toGet = I2C_BUFFER_LENGTH - (I2C_BUFFER_LENGTH % (max_dev->activeLEDs * 3)); //Trim toGet to be a multiple of the samples we need to read
						}
						
	
						//nrf_drv_twi_tx(max_twi, _i2caddr, &MAX30102_FIFODATA, sizeof(MAX30102_FIFODATA), false);
						//nrf_drv_twi_rx(max_twi, _i2caddr, &buffer_i2c[0], toGet);
						
						readRegister(max_dev, max_dev->device_addr, MAX30102_FIFODATA, &buffer_i2c[0], toGet);
						
						
						for(uint16_t i=0;i<toGet;i = i+ ( max_dev->activeLEDs * 3) )
						{
								sense.head++; //Advance the head of the storage struct
								sense.head %= STORAGE_SIZE; //Wrap condition
							  
							  tempLong = (buffer_i2c[i]<<16) | (buffer_i2c[i+1]<<8) | (buffer_i2c[i+2]);

								tempLong &= 0x3FFFF; //Zero out all but 18 bits
							
								sense.red[sense.head] = tempLong; //Store this reading into the sense array
							
								if(max_dev->activeLEDs > 1)
								{
										tempLong = (buffer_i2c[i+3]<<16) | (buffer_i2c[i+4]<<8) | (buffer_i2c[i+5]);
										tempLong &= 0x3FFFF; //Zero out all but 18 bits          
										sense.IR[sense.head] = tempLong;
								}
								
								if(max_dev->activeLEDs > 2)
								{
										tempLong = (buffer_i2c[i+6]<<16) | (buffer_i2c[i+7]<<8) | (buffer_i2c[i+8]);
										tempLong &= 0x3FFFF; //Zero out all but 18 bits
										sense.green[sense.head] = tempLong;
								}
						}
				}
		}
	
		//NRF_LOG_INFO("Read %d\tWrite %d\t numberOfSamples %d\n",readPointer,writePointer,numberOfSamples);
	
		
		return (numberOfSamples);
	
}

uint16_t check(max30102_dev_t *max_dev)
{
		uint8_t buffer_i2c[250];
		uint32_t tempLong;
	
		//Read register FIDO_DATA in (3-byte * number of active LED) chunks
		//Until FIFO_RD_PTR = FIFO_WR_PTR
		uint8_t readPointer = getReadPointer(max_dev);
		uint8_t writePointer = getWritePointer(max_dev);
	
	
		int numberOfSamples = 0;
	
		if (readPointer != writePointer)
		{
				//Calculate the number of readings we need to get from sensor
				numberOfSamples = writePointer - readPointer;
				if (numberOfSamples < 0) numberOfSamples += 32; //Wrap condition
			
				//We now have the number of readings, now calc bytes to read
				//For this example we are just doing Red and IR (3 bytes each)
	
  			int bytesLeftToRead = numberOfSamples * max_dev->activeLEDs * 3;
			
			
				//Get ready to read a burst of data from the FIFO register
				//_i2cPort->beginTransmission(MAX30102_ADDRESS);
				//_i2cPort->write(MAX30102_FIFODATA);
				//_i2cPort->endTransmission();
				//nrf_drv_twi_tx(max_twi, _i2caddr, &MAX30102_FIFODATA, sizeof(MAX30102_FIFODATA), false);
				//NRF_LOG_RAW_INFO("\n$writePointer %d,readPointer %d, Get::%d",writePointer, readPointer, numberOfSamples);	
			
			  if(bytesLeftToRead > 0)
				{
						int toGet = bytesLeftToRead;
					
						if (toGet > I2C_BUFFER_LENGTH)
						{
								//If toGet is 32 this is bad because we read 6 bytes (Red+IR * 3 = 6) at a time
								//32 % 6 = 2 left over. We don't want to request 32 bytes, we want to request 30.
								//32 % 9 (Red+IR+GREEN) = 5 left over. We want to request 27.
								toGet = I2C_BUFFER_LENGTH - (I2C_BUFFER_LENGTH % (max_dev->activeLEDs * 3)); //Trim toGet to be a multiple of the samples we need to read
						}
						
	
						//nrf_drv_twi_tx(max_twi, _i2caddr, &MAX30102_FIFODATA, sizeof(MAX30102_FIFODATA), false);
						//nrf_drv_twi_rx(max_twi, _i2caddr, &buffer_i2c[0], toGet);
						
						readRegister(max_dev, max_dev->device_addr, MAX30102_FIFODATA, &buffer_i2c[0], toGet);
						
						
						for(uint16_t i=0;i<toGet;i = i+ ( max_dev->activeLEDs * 3) )
						{
								sense.head++; //Advance the head of the storage struct
								sense.head %= STORAGE_SIZE; //Wrap condition
							  
							  tempLong = (buffer_i2c[i]<<16) | (buffer_i2c[i+1]<<8) | (buffer_i2c[i+2]);
								tempLong &= 0x3FFFF; //Zero out all but 18 bits
							
								sense.red[sense.head] = tempLong; //Store this reading into the sense array
							
								if(max_dev->activeLEDs > 1)
								{
										tempLong = (buffer_i2c[i+3]<<16) | (buffer_i2c[i+4]<<8) | (buffer_i2c[i+5]);
										tempLong &= 0x3FFFF; //Zero out all but 18 bits          
										sense.IR[sense.head] = tempLong;
								}
								
								if(max_dev->activeLEDs > 2)
								{
										tempLong = (buffer_i2c[i+6]<<16) | (buffer_i2c[i+7]<<8) | (buffer_i2c[i+8]);
										tempLong &= 0x3FFFF; //Zero out all but 18 bits
										sense.green[sense.head] = tempLong;
								}
						}
				}
		}else{
				
		}
	
		//NRF_LOG_INFO("Read %d\tWrite %d\t numberOfSamples %d\n",readPointer,writePointer,numberOfSamples);
	
		
		return (numberOfSamples);
	
}

uint8_t max30102_available(max30102_dev_t *max_dev)
{
		uint8_t numberOfSamples;		
		int16_t dataSize = sense.head - sense.tail;
		if (dataSize < 0) 
				numberOfSamples += STORAGE_SIZE;
		else
				numberOfSamples = dataSize;
		return (numberOfSamples);
}

void max30102_nextSample(max30102_dev_t *max_dev)
{
		if(max30102_available(max_dev)) //Only advance the tail if new data is available
		{
				sense.tail++;
				sense.tail %= STORAGE_SIZE; //Wrap condition
		}
}

uint32_t getFIFORed(max30102_dev_t *max_dev)
{
		return (sense.red[sense.tail]);
}

uint32_t getFIFOIR(max30102_dev_t *max_dev)
{
		return (sense.IR[sense.tail]);
}

uint32_t getFIFOGreen(max30102_dev_t *max_dev)
{
		return (sense.green[sense.tail]);
}


void setPROXINTTHRESH(max30102_dev_t *max_dev, uint8_t val)
{
		writeRegister8(max_dev, max_dev->device_addr, MAX30102_PROXINTTHRESH, val);
}

// Die Temperature
double max30102_readTemperature(max30102_dev_t *max_dev)
{
		// Step 1: Config die temperature register to take 1 temperature sample
		writeRegister8(max_dev, max_dev->device_addr, MAX30102_DIETEMPCONFIG, 0x01);

		// Poll for bit to clear, reading is then complete
		// Timeout after 100ms
		uint8_t timeout = 100;
		while (timeout--)
		{
				uint8_t response = readRegister8(max_dev, max_dev->device_addr, MAX30102_DIETEMPCONFIG);
				if ((response & 0x01) == 0) break; //We're done!
				max_dev->delay_ms(1); //Let's not over burden the I2C bus
		}
		//TODO How do we want to fail? With what type of error?
		//? if(millis() - startTime >= 100) return(-999.0);

		// Step 2: Read die temperature register (integer)
		int8_t tempInt = readRegister8(max_dev, max_dev->device_addr, MAX30102_DIETEMPINT);
		uint8_t tempFrac = readRegister8(max_dev, max_dev->device_addr, MAX30102_DIETEMPFRAC);

		// Step 3: Calculate temperature (datasheet pg. 23)
		return (double)tempInt + ((double)tempFrac * 0.0625);
}

double max30102_readTemperatureF(max30102_dev_t *max_dev)
{
		double temp = max30102_readTemperature(max_dev);

		if (temp != -999.0) temp = temp * 1.8 + 32.0;

		return (temp);
}

uint8_t max30102_readPartID(max30102_dev_t *max_dev)
{
		uint8_t b_partid;	
	
	  b_partid = readRegister8(max_dev, max_dev->device_addr,MAX30102_PARTID);
		
		return b_partid;
}  

uint8_t max30102_isr_config(max30102_dev_t *max_dev)
{
		uint8_t config = 0x80;	
		writeRegister8(max_dev, max_dev->device_addr, MAX30102_INTENABLE1, config);
	
		config = 0x00;
		writeRegister8(max_dev, max_dev->device_addr, MAX30102_INTENABLE2, config);
	
//		uint8_t res;
//		res = readRegister8(max_dev, max_dev->device_addr, 0x02);
	
//		NRF_LOG_INFO("\nRegister ISR 02::%02X\n",res);
//		NRF_LOG_FLUSH();
//	
//		res = readRegister8(max_dev, max_dev->device_addr, 0x03);
//		
//		NRF_LOG_INFO("\nRegister ISR 03::%02X\n",res);
//		NRF_LOG_FLUSH();
//	
//		res = readRegister8(max_dev, max_dev->device_addr, 0x08);
//		
//		NRF_LOG_INFO("\nRegister ISR 08::%02X\n",res);
//		NRF_LOG_FLUSH();
//		
//		res = readRegister8(max_dev, max_dev->device_addr, 0x09);
//		
//		NRF_LOG_INFO("\nRegister ISR 09::%02X\n",res);
//		NRF_LOG_FLUSH();
		
		return MAX30102_SUCCESS;
}

uint8_t max30102_initaial(max30102_dev_t *max_dev)
{
		//float temperature;

		max_dev->delay_ms(100);
	
		uint8_t partID = max30102_readPartID(max_dev);
		if (partID != MAX_30102_EXPECTEDPARTID) {
				NRF_LOG_INFO("[FALL]\tMAX30102_readPartID() != MAX_30102_EXPECTEDPARTID");
				NRF_LOG_FLUSH();
				return MAX30102_ERROR_NOT_FOUND;
		}
		
		max30102_readRevisionID(max_dev);
		
		//temperature = max30102_readTemperature(max_dev);
		
		return MAX30102_SUCCESS;
}

void max30102_setup(max30102_dev_t *max_dev)
{
		uint8_t powerLevel = 0x0F;//0x0F;
		uint8_t sampleAverage = 4;
	  uint8_t ledMode = 2;
		int sampleRate = 	400;
	  int pulseWidth = 411;
	  int adcRange = 8192;
		
		max30102_softReset(max_dev); //Reset all configuration, threshold, and data registers to POR values

		//FIFO Configuration
		//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
		//The chip will average multiple samples of same type together if you wish
		if (sampleAverage == 1) setFIFOAverage(max_dev, MAX30102_SAMPLEAVG_1); //No averaging per FIFO record
		else if (sampleAverage == 2) setFIFOAverage(max_dev, MAX30102_SAMPLEAVG_2);
		else if (sampleAverage == 4) setFIFOAverage(max_dev, MAX30102_SAMPLEAVG_4);
		else if (sampleAverage == 8) setFIFOAverage(max_dev, MAX30102_SAMPLEAVG_8);
		else if (sampleAverage == 16) setFIFOAverage(max_dev, MAX30102_SAMPLEAVG_16);
		else if (sampleAverage == 32) setFIFOAverage(max_dev, MAX30102_SAMPLEAVG_32);
		else setFIFOAverage(max_dev, MAX30102_SAMPLEAVG_4);

		//setFIFOAlmostFull(2); //Set to 30 samples to trigger an 'Almost Full' interrupt
		enableFIFORollover(max_dev); //Allow FIFO to wrap/roll over
		//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

		//Mode Configuration
		//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
		if (ledMode == 3) setLEDMode(max_dev, MAX30102_MODE_MULTILED); //Watch all three LED channels
		else if (ledMode == 2) setLEDMode(max_dev, MAX30102_MODE_REDIRONLY); //Red and IR
		else setLEDMode(max_dev, MAX30102_MODE_REDONLY); //Red only
		max_dev->activeLEDs = ledMode; //Used to control how many bytes to read from FIFO buffer
		//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

		//Particle Sensing Configuration
		//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
		if(adcRange < 4096) setADCRange(max_dev, MAX30102_ADCRANGE_2048); //7.81pA per LSB
		else if(adcRange < 8192) setADCRange(max_dev, MAX30102_ADCRANGE_4096); //15.63pA per LSB
		else if(adcRange < 16384) setADCRange(max_dev, MAX30102_ADCRANGE_8192); //31.25pA per LSB
		else if(adcRange == 16384) setADCRange(max_dev, MAX30102_ADCRANGE_16384); //62.5pA per LSB
		else setADCRange(max_dev, MAX30102_ADCRANGE_2048);

		if (sampleRate < 100) setSampleRate(max_dev, MAX30102_SAMPLERATE_50); //Take 50 samples per second
		else if (sampleRate < 200) setSampleRate(max_dev, MAX30102_SAMPLERATE_100);
		else if (sampleRate < 400) setSampleRate(max_dev, MAX30102_SAMPLERATE_200);
		else if (sampleRate < 800) setSampleRate(max_dev, MAX30102_SAMPLERATE_400);
		else if (sampleRate < 1000) setSampleRate(max_dev, MAX30102_SAMPLERATE_800);
		else if (sampleRate < 1600) setSampleRate(max_dev, MAX30102_SAMPLERATE_1000);
		else if (sampleRate < 3200) setSampleRate(max_dev, MAX30102_SAMPLERATE_1600);
		else if (sampleRate == 3200) setSampleRate(max_dev, MAX30102_SAMPLERATE_3200);
		else setSampleRate(max_dev, MAX30102_SAMPLERATE_50);

		//The longer the pulse width the longer range of detection you'll have
		//At 69us and 0.4mA it's about 2 inches
		//At 411us and 0.4mA it's about 6 inches
		if (pulseWidth < 118) setPulseWidth(max_dev, MAX30102_PULSEWIDTH_69); //Page 26, Gets us 15 bit resolution
		else if (pulseWidth < 215) setPulseWidth(max_dev, MAX30102_PULSEWIDTH_118); //16 bit resolution
		else if (pulseWidth < 411) setPulseWidth(max_dev, MAX30102_PULSEWIDTH_215); //17 bit resolution
		else if (pulseWidth == 411) setPulseWidth(max_dev, MAX30102_PULSEWIDTH_411); //18 bit resolution
		else setPulseWidth(max_dev, MAX30102_PULSEWIDTH_69);
		//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

		//LED Pulse Amplitude Configuration
		//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
		//Default is 0x1F which gets us 6.4mA
		//powerLevel = 0x02, 0.4mA - Presence detection of ~4 inch
		//powerLevel = 0x1F, 6.4mA - Presence detection of ~8 inch
		//powerLevel = 0x7F, 25.4mA - Presence detection of ~8 inch
		//powerLevel = 0xFF, 50.0mA - Presence detection of ~12 inch

		setPulseAmplitudeRed(max_dev, powerLevel);
		setPulseAmplitudeIR(max_dev, powerLevel);
		setPulseAmplitudeGreen(max_dev, powerLevel);
		setPulseAmplitudeProximity(max_dev, powerLevel);
		//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

		//Multi-LED Mode Configuration, Enable the reading of the three LEDs
		//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
		enableSlot(max_dev, 1, SLOT_RED_LED);
		if (ledMode > 1) enableSlot(max_dev, 2, SLOT_IR_LED);
		if (ledMode > 2) enableSlot(max_dev, 3, SLOT_GREEN_LED);
		//enableSlot(1, SLOT_RED_PILOT);
		//enableSlot(2, SLOT_IR_PILOT);
		//enableSlot(3, SLOT_GREEN_PILOT);
		//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

		clearFIFO(max_dev); //Reset the FIFO before we begin checking the sensor

		max30102_isr_config(max_dev);
}

void max30102_setup2(max30102_dev_t *max_dev, uint8_t powerLevel, uint8_t sampleAverage, uint8_t ledMode, int sampleRate, int pulseWidth, int adcRange)
{
		
		max30102_softReset(max_dev); //Reset all configuration, threshold, and data registers to POR values

		//FIFO Configuration
		//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
		//The chip will average multiple samples of same type together if you wish
		if (sampleAverage == 1) setFIFOAverage(max_dev, MAX30102_SAMPLEAVG_1); //No averaging per FIFO record
		else if (sampleAverage == 2) setFIFOAverage(max_dev, MAX30102_SAMPLEAVG_2);
		else if (sampleAverage == 4) setFIFOAverage(max_dev, MAX30102_SAMPLEAVG_4);
		else if (sampleAverage == 8) setFIFOAverage(max_dev, MAX30102_SAMPLEAVG_8);
		else if (sampleAverage == 16) setFIFOAverage(max_dev, MAX30102_SAMPLEAVG_16);
		else if (sampleAverage == 32) setFIFOAverage(max_dev, MAX30102_SAMPLEAVG_32);
		else setFIFOAverage(max_dev, MAX30102_SAMPLEAVG_4);

		//setFIFOAlmostFull(2); //Set to 30 samples to trigger an 'Almost Full' interrupt
		enableFIFORollover(max_dev); //Allow FIFO to wrap/roll over
		//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

		//Mode Configuration
		//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
		if (ledMode == 3) setLEDMode(max_dev, MAX30102_MODE_MULTILED); //Watch all three LED channels
		else if (ledMode == 2) setLEDMode(max_dev, MAX30102_MODE_REDIRONLY); //Red and IR
		else setLEDMode(max_dev, MAX30102_MODE_REDONLY); //Red only
		max_dev->activeLEDs = ledMode; //Used to control how many bytes to read from FIFO buffer
		//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

		//Particle Sensing Configuration
		//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
		if(adcRange < 4096) setADCRange(max_dev, MAX30102_ADCRANGE_2048); //7.81pA per LSB
		else if(adcRange < 8192) setADCRange(max_dev, MAX30102_ADCRANGE_4096); //15.63pA per LSB
		else if(adcRange < 16384) setADCRange(max_dev, MAX30102_ADCRANGE_8192); //31.25pA per LSB
		else if(adcRange == 16384) setADCRange(max_dev, MAX30102_ADCRANGE_16384); //62.5pA per LSB
		else setADCRange(max_dev, MAX30102_ADCRANGE_2048);

		if (sampleRate < 100) setSampleRate(max_dev, MAX30102_SAMPLERATE_50); //Take 50 samples per second
		else if (sampleRate < 200) setSampleRate(max_dev, MAX30102_SAMPLERATE_100);
		else if (sampleRate < 400) setSampleRate(max_dev, MAX30102_SAMPLERATE_200);
		else if (sampleRate < 800) setSampleRate(max_dev, MAX30102_SAMPLERATE_400);
		else if (sampleRate < 1000) setSampleRate(max_dev, MAX30102_SAMPLERATE_800);
		else if (sampleRate < 1600) setSampleRate(max_dev, MAX30102_SAMPLERATE_1000);
		else if (sampleRate < 3200) setSampleRate(max_dev, MAX30102_SAMPLERATE_1600);
		else if (sampleRate == 3200) setSampleRate(max_dev, MAX30102_SAMPLERATE_3200);
		else setSampleRate(max_dev, MAX30102_SAMPLERATE_50);

		//The longer the pulse width the longer range of detection you'll have
		//At 69us and 0.4mA it's about 2 inches
		//At 411us and 0.4mA it's about 6 inches
		if (pulseWidth < 118) setPulseWidth(max_dev, MAX30102_PULSEWIDTH_69); //Page 26, Gets us 15 bit resolution
		else if (pulseWidth < 215) setPulseWidth(max_dev, MAX30102_PULSEWIDTH_118); //16 bit resolution
		else if (pulseWidth < 411) setPulseWidth(max_dev, MAX30102_PULSEWIDTH_215); //17 bit resolution
		else if (pulseWidth == 411) setPulseWidth(max_dev, MAX30102_PULSEWIDTH_411); //18 bit resolution
		else setPulseWidth(max_dev, MAX30102_PULSEWIDTH_69);
		//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

		//LED Pulse Amplitude Configuration
		//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
		//Default is 0x1F which gets us 6.4mA
		//powerLevel = 0x02, 0.4mA - Presence detection of ~4 inch
		//powerLevel = 0x1F, 6.4mA - Presence detection of ~8 inch
		//powerLevel = 0x7F, 25.4mA - Presence detection of ~8 inch
		//powerLevel = 0xFF, 50.0mA - Presence detection of ~12 inch

		setPulseAmplitudeRed(max_dev, powerLevel);
		setPulseAmplitudeIR(max_dev, powerLevel);
		setPulseAmplitudeGreen(max_dev, powerLevel);
		setPulseAmplitudeProximity(max_dev, powerLevel);
		//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

		//Multi-LED Mode Configuration, Enable the reading of the three LEDs
		//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
		enableSlot(max_dev, 1, SLOT_RED_LED);
		if (ledMode > 1) enableSlot(max_dev, 2, SLOT_IR_LED);
		if (ledMode > 2) enableSlot(max_dev, 3, SLOT_GREEN_LED);
		//enableSlot(1, SLOT_RED_PILOT);
		//enableSlot(2, SLOT_IR_PILOT);
		//enableSlot(3, SLOT_GREEN_PILOT);
		//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

		clearFIFO(max_dev); //Reset the FIFO before we begin checking the sensor

		max30102_isr_config(max_dev);
}

// Low-level I2C communication
uint8_t readRegister8(max30102_dev_t *max_dev, uint8_t address, uint8_t reg)
{
		uint8_t buffer = 0;

		max_dev->read(address, reg, &buffer, sizeof(buffer));
		
		return buffer;
}

uint8_t readRegister(max30102_dev_t *max_dev, uint8_t address, uint8_t reg, uint8_t res[], uint8_t len)
{
		uint8_t err_code;

		err_code = max_dev->read(address, reg, res, len);

		return err_code;
}

uint8_t writeRegister8(max30102_dev_t *max_dev, uint8_t address, uint8_t reg, uint8_t value)
{
		uint8_t err_code;
		err_code = max_dev->write(max_dev->device_addr, reg, &value, sizeof(value));
		return err_code;
}

void max30102_readRevisionID(max30102_dev_t *max_dev)
{
		max_dev->revisionID = readRegister8(max_dev, max_dev->device_addr, MAX30102_REVISIONID);
}

uint8_t getRevisionID(max30102_dev_t *max_dev)
{
		return max_dev->revisionID;
}

void bitMask(max30102_dev_t *max_dev, uint8_t reg, uint8_t mask, uint8_t thing)
{
		// Grab current register context
		uint8_t originalContents = readRegister8(max_dev, max_dev->device_addr, reg);

		// Zero-out the portions of the register we're interested in
		originalContents = originalContents & mask;

		// Change contents
		writeRegister8(max_dev, max_dev->device_addr, reg, originalContents | thing);
}

uint8_t max30102_read_isr_flag(max30102_dev_t *max_dev)
{
		uint8_t res;
		readRegister(max_dev, max_dev->device_addr, MAX30102_INTSTAT1,&res,sizeof(res));
	
		return res;
}





