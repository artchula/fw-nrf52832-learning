#ifndef __AT45DB081E_H__
#define __AT45DB081E_H__


#include "stdio.h"
#include "stdint.h"


/*****************************************************************************/
/* The DEFUALT register value when device is reset Definitions */
#define	DEFAULT_Manufacturer		  						0x1F
#define	DEFAULT_DeviceID_1		  							0x25
#define	DEFAULT_DeviceID_2		  							0x00
#define	DEFAULT_EDI_1		  										0x01
#define	DEFAULT_EDI_2		  										0x00


/*****************************************************************************/
/* Additional Command Definitions */
#define MainMemoryPageBuffer_1_Transer				0x53
#define MainMemoryPageBuffer_2_Transer				0x55
#define MainMemoryPageBuffer_1_Compare				0x60
#define MainMemoryPageBuffer_2_Compare				0x61
#define Auto_Page_Rewrite_1										0x58
#define Auto_Page_Rewrite_2										0x59
#define Deep_Power_Down												0xB9
#define Resume_From_Deep_Power								0xAB
#define Ultra_Deep_Power_Down									0x79
#define Status_Reg_Read												0xD7
#define MANUFACTURER_DEVICE_ID								0x9F
#define Config_Power_of_2_Page_Size						0x3D2A80A6
#define Config_Standard_DataFlash_Page_Size 	0x3D2A80A7
#define Software_Reset												0xF0000000
#define TIME_OUT_COUNTER_MAX									3000
#define AT45DB_BLOCK_ERASE                    0x50    // Block Erase Opcode
#define AT45DB_PAGE_ERASE                     0x81    // Page Erase Opcode 
#define AT45DB_BUFFER_1                       0x84
#define AT45DB_BUFFER_2                       0x87    // Write byte(s) to buffer 2 opcode
#define AT45DB_BUF_1_TO_PAGE            			0x83 		// Copy Buffer 1 to page Opcode, 0x88 without auto erase
#define AT45DB_BUF_2_TO_PAGE            			0x86 		// Copy Buffer 2 to page Opcode, 0x89 without auto erase
#define AT45DB_PAGE_READ                      0xD2 		// Read direct from Flash EEPROM page Opcode
#define AT45DB_STATUS_REG											0x57



/*!
 * Transfer page to buffer 2 Opcode
 * \note Only Buffer 2 is used to readout a page, because the read
 * respectively transfer latency is only about 200us
 *
 */
#define AT45DB_PAGE_TO_BUF                    0x55 //use buffer 2
/*!
 * Read buffer 2 opcode
 * \note Only Buffer 2 is used to readout a page, because the read
 * respectively transfer latency is only about 200us
 */
#define AT45DB_READ_BUFFER              		  0xD6



/*****************************************************************************/
/* type definitions */
typedef int8_t (*at45dbxx_com_fptr_t)(uint8_t command[], uint16_t command_len, uint8_t data[], uint16_t data_len, uint8_t en_continues_cs_low);

typedef void (*at45dbxx_delay_fptr_t)(uint32_t period);

typedef struct{
/*!
 * Holds the active buffer
 *  <ul>
 * <li> 0 : Active Buffer = Buffer 1
 * <li> 1 : Active Buffer = Buffer 2
 * </ul>
 */
        volatile uint8_t active_buffer;
/*!
 * The specific "byte(s) to buffer" opcode for buffer 1
 * and buffer 2
 */
        volatile uint8_t buffer_addr[2];
/*!
 * The specific "buffer to page" opcode for buffer 1 and
 * buffer 2
 */
        volatile uint8_t buf_to_page_addr[2];
}bufmgr_t;

typedef enum{
		AT45DB_SUCCESS,
		AT45DB_ERROR_UNSUCCESS,
		AT45DB_ERROR_NOT_FOUND,
		AT45DB_ERROR_NULL_POINTER,
		AT45DB_ERROR_TIMEOUT,
		AT45DB_ERROR_BUSY,
		AT45DB_ERROR_INVALID_ADDR,
		AT45DB_ERROR_INVALID_PARAM,
		AT45DB_ERROR_INVALID_LENGTH 
}at45db_status_t;

typedef enum{
		AT45DB_DISABLE_CONTINUES_CS_LOW,
		AT45DB_ENABLE_CONTINUES_CS_LOW
}en_continues_cs_low_t;

typedef struct 
{
		uint8_t menu_fac;
		uint8_t device_id[2];
		uint8_t edi[2];
}at45dbxx_device_info_t;
	
/**@brief AT45DBxx Status Register. */
typedef struct 
{
		union{
				uint8_t data;
				struct{
					uint8_t page_size:1;
					uint8_t protect:1;
					uint8_t desity:4;
					uint8_t comp:1;
					uint8_t rdy_busy:1;
				}bits;
		}byte1;
		union{
				uint8_t data;
				struct{
					uint8_t es:1;
					uint8_t ps1:1;
					uint8_t ps2:1;
					uint8_t sle:1;
					uint8_t res2:1;
					uint8_t epe:1;
					uint8_t res1:1;
					uint8_t rdy_busy:1;
				}bits;
		}byte2;
}at45dbxx_status_reg_t;


typedef struct {
	/*! Menufacturer and Device ID */
	at45dbxx_device_info_t menufac_device_id;
	/*! Read function pointer */
	at45dbxx_com_fptr_t read;
	/*! Write function pointer */
	at45dbxx_com_fptr_t write;
	/*!  Delay function pointer */
	at45dbxx_delay_fptr_t delay_ms;
	/*!  Buffer manager */
	bufmgr_t buffer_mgr;
}at45dbxx_dev_t;

 
/*****************************************************************************/
/* Function definitions */
uint32_t at45dbxx_initial(at45dbxx_dev_t *dev);
uint32_t at45dbxx_deinitial(at45dbxx_dev_t *dev);
uint32_t at45dbxx_deep_sleep(at45dbxx_dev_t *dev);
uint32_t at45dbxx_wakeup(at45dbxx_dev_t *dev);
uint32_t at45dbxx_wakeup_pin(at45dbxx_dev_t *dev);
uint32_t at45dbxx_is_available(at45dbxx_dev_t *dev);
uint32_t at45dbxx_get_menufac_and_device_id(at45dbxx_dev_t *dev, at45dbxx_device_info_t *device_info);
uint32_t at34dbxx_deep_power_down(at45dbxx_dev_t *dev);
uint32_t at45dbxx_ultra_deep_power_down(at45dbxx_dev_t *dev);
uint32_t at45dbxx_resume_deep_power_down(at45dbxx_dev_t *dev);
uint32_t at45dbxx_resume_ultra_deep_power_down(at45dbxx_dev_t *dev);

uint32_t at45db_busy_wait(at45dbxx_dev_t *dev);
uint32_t at45db_erase_chip(at45dbxx_dev_t *dev);
uint32_t at45db_erase_block(at45dbxx_dev_t *dev, uint16_t addr);
uint32_t at45db_erase_page(at45dbxx_dev_t *dev, uint16_t addr) ;
uint32_t at45db_write_buffer(at45dbxx_dev_t *dev, uint16_t offset, uint8_t *buffer, uint16_t bytes);
uint32_t at45db_buffer_to_page(at45dbxx_dev_t *dev, uint16_t addr);
uint32_t at45db_read_page_buffered(at45dbxx_dev_t *dev, uint16_t p_addr, uint16_t b_addr, uint8_t *buffer, uint16_t bytes);
uint32_t at45db_page_to_buf(at45dbxx_dev_t *dev, uint16_t addr);
uint32_t at45db_read_buffer(at45dbxx_dev_t *dev, uint16_t b_offset, uint8_t *buffer, uint16_t bytes);
uint32_t at45db_read_page_bypassed(at45dbxx_dev_t *dev, uint16_t p_addr, uint16_t b_addr, uint8_t *buffer, uint16_t bytes);
uint32_t at45db_read_status_reg(at45dbxx_dev_t *dev);

#endif

