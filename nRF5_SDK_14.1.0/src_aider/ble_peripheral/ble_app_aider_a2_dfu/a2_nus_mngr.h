#ifndef __A2_NUS_MNGR_H__
#define __A2_NUS_MNGR_H__

#include <stdint.h>
#include "basic_fifo.h"
#include "a2_logging_mngr.h"

#define MAX_DATA_COMMAND 			17
#define MAX_DATA_PAYLOAD 			17

#define BUFFER_SIZE 					20
#define INPUT_FIFO_SIZE 	 		10 
#define MIN_INPUT_PKG_SIZE     3


/*****************************************************************************/
/* type definitions */
typedef uint8_t (*sync_read_nus_fptr_t)(uint8_t *p_data, uint16_t len);
typedef uint8_t (*sync_write_nus_fptr_t)(uint8_t *p_data, uint16_t len);
typedef void (*sync_delay_ms_fptr_t)(uint32_t delay);
typedef void (*sync_save_cfg_to_flash_fptr_t)(void);
typedef void (*sync_wakeup_flash_fptr_t)(void);
typedef void (*sync_sleep_flash_fptr_t)(void);

typedef struct{
		uint8_t register_id;	
		uint8_t command_read_write;
		uint8_t command_len;
		uint8_t data[MAX_DATA_COMMAND];
}sync_nus_input_t;

typedef struct{
		uint8_t register_id;
		uint8_t reg_ack;
		uint8_t pkg_status;
		uint8_t data[MAX_DATA_COMMAND];
}sync_nus_output_t;

typedef struct{
		sync_read_nus_fptr_t read;
		sync_write_nus_fptr_t write;
		sync_delay_ms_fptr_t delay_ms; 
		sync_save_cfg_to_flash_fptr_t save_cfg_to_flash;
		sync_wakeup_flash_fptr_t wakeup_flash;
		sync_sleep_flash_fptr_t sleep_flash;
		sync_nus_input_t input_buf[INPUT_FIFO_SIZE];
		uint8_t input_len[INPUT_FIFO_SIZE];
		sync_nus_output_t output_buf;
		queue_member_t que_input;
	  logging_dev_t *p_logging_dev;
		queue_type_t *p_queue;
}sync_nus_comm_t;

/**
 * @brief Function for initial sync nus service protocols.
 *
 * @param[in]   sync_nus_comm_t       pointer struct for communication with ble connect.
 */
void sync_nus_initial(sync_nus_comm_t *p_sync_nus_comm, a2_reg_t *p_a2_reg);

/**
 * @brief Function for chcek command from mobile phone or gate and sent result to host.
 *
 * @param[in]   sync_nus_comm_t       pointer struct for communication with ble connect.
 */
void sync_nus_task(sync_nus_comm_t *p_sync_nus_comm, a2_reg_t *p_a2_reg);

/**
 * @brief Function for add data buffer to queue
 *
 * @param[in]   p_sync_nus_comm       pointer struct for communication with ble connect.
 * @param[in]   data       						pointer data.
 * @param[in]   len       						lenth of data.
 */
void sync_nus_append_command(sync_nus_comm_t *p_sync_nus_comm, uint8_t *p_data, uint16_t len);

void TEST_NUS_READ_FUNCTION(uint8_t *data, uint16_t len);
uint8_t TEST_NUS_WRITE_FUNCTION(uint8_t *data, uint16_t len);

#endif
