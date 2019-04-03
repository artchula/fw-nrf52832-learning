#include "a2_nus_mngr.h"
#include "a2_logging_mngr.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "nrf_delay.h"


static void sync_nus_normal_register_mngr(sync_nus_comm_t *p_sync_nus_comm, a2_reg_t *p_a2_reg);
static void sync_nus_special_register_mngr(sync_nus_comm_t *p_sync_nus_comm, a2_reg_t *p_a2_reg);
static void sync_nus_special_button_alert_reg_mngr(sync_nus_comm_t *p_sync_nus_comm);
static void sync_nus_special_pedometer_reg_mngr(sync_nus_comm_t *p_sync_nus_comm);
static void sync_nus_special_hr_spo2_temp_reg_mngr(sync_nus_comm_t *p_sync_nus_comm);
static void sync_nus_special_free_fall_reg_mngr(sync_nus_comm_t *p_sync_nus_comm);
static void sync_nus_special_free_fall_raw_data_reg_mngr(sync_nus_comm_t *p_sync_nus_comm);
	


void sync_nus_initial(sync_nus_comm_t *p_sync_nus_comm, a2_reg_t *p_a2_reg)
{
		que_initial( &p_sync_nus_comm->que_input, INPUT_FIFO_SIZE);
}

void sync_nus_task(sync_nus_comm_t *p_sync_nus_comm, a2_reg_t *p_a2_reg)
{
		uint16_t buf_q_r;
		
		//Check Empty FIFO 
		if(que_isEmpty(&p_sync_nus_comm->que_input) == QUE_TURE)
		{
				//Data is empty for process
				return;
		}
		
		//Check package lenght
		que_front(&p_sync_nus_comm->que_input, &buf_q_r);
		uint16_t all_input_pkg_len = p_sync_nus_comm->input_len[buf_q_r];
		if(all_input_pkg_len < MIN_INPUT_PKG_SIZE)
		{
				//ACK FALL
				p_sync_nus_comm->output_buf.register_id = 0;
				p_sync_nus_comm->output_buf.reg_ack 		= REG_RES_UNSUCCESS;
				p_sync_nus_comm->output_buf.pkg_status 	= FINAL_PKG;				
				p_sync_nus_comm->write(&p_sync_nus_comm->output_buf.register_id, MIN_INPUT_PKG_SIZE);
				NRF_LOG_INFO("Fail sync MIN_INPUT_PKG_SIZE.");
				return;
		}
		//Get register, command and len of command
		sync_nus_input_t *p_input_pkg =  &p_sync_nus_comm->input_buf[buf_q_r];

		//Normal register mngr
		if(p_input_pkg->register_id >= BEGIN_ADDR_NORMAL_REG && p_input_pkg->register_id <= END_ADDR_NORMAL_REG)
		{
				sync_nus_normal_register_mngr(p_sync_nus_comm, p_a2_reg);
		}
		else if(p_input_pkg->register_id >= BEGIN_ADDR_SPECIAL_REG && p_input_pkg->register_id <= END_ADDR_SPECIAL_REG)  //Special register mngr
		{	
				sync_nus_special_register_mngr(p_sync_nus_comm, p_a2_reg);
		}else{
				//ACK FALL
				p_sync_nus_comm->output_buf.register_id = p_input_pkg->register_id;
				p_sync_nus_comm->output_buf.reg_ack 		= REG_RES_UNSUCCESS;
				p_sync_nus_comm->output_buf.pkg_status 	= FINAL_PKG;				
				p_sync_nus_comm->write(&p_sync_nus_comm->output_buf.register_id, MIN_INPUT_PKG_SIZE);
				NRF_LOG_INFO("Fail sync addres reg.");
				return;
		}
		que_pop(&p_sync_nus_comm->que_input);
		//End switch case	
}


static void sync_nus_normal_register_mngr(sync_nus_comm_t *p_sync_nus_comm, a2_reg_t *p_a2_reg)
{
		uint16_t buf_q_r;
		uint8_t buf_data[20];
	
		//Check package lenght
		que_front(&p_sync_nus_comm->que_input, &buf_q_r);
	
		//Get command and len of command
		sync_nus_input_t *p_input_pkg =  &p_sync_nus_comm->input_buf[buf_q_r];
	
		if(p_input_pkg->command_len > MAX_DATA_COMMAND)
		{
				//ACK FALL
				p_sync_nus_comm->output_buf.register_id = p_input_pkg->register_id;
				p_sync_nus_comm->output_buf.reg_ack 		= REG_RES_UNSUCCESS;
				p_sync_nus_comm->output_buf.pkg_status 	= FINAL_PKG;				
				p_sync_nus_comm->write(&p_sync_nus_comm->output_buf.register_id, MIN_INPUT_PKG_SIZE);
				return;
		}
		
		if(p_input_pkg->command_read_write == REG_COMMAND_READ)
		{
				//Read Reg 
				a2_read_reg(p_a2_reg, p_input_pkg->register_id, p_input_pkg->command_len, &buf_data[0]);
				//Pack data to payload
				p_sync_nus_comm->output_buf.register_id = p_input_pkg->register_id;
				p_sync_nus_comm->output_buf.reg_ack 		= REG_RES_SUCCESS;
				p_sync_nus_comm->output_buf.pkg_status 	= FINAL_PKG;
				memcpy(&p_sync_nus_comm->output_buf.data[0], &buf_data[0], p_input_pkg->command_len);
				//Sent data to host
				p_sync_nus_comm->write(&p_sync_nus_comm->output_buf.register_id, MIN_INPUT_PKG_SIZE+ p_input_pkg->command_len);
		}else if(p_input_pkg->command_read_write == REG_COMMAND_WRITE){
				//For loop write
			  uint16_t flag_write_config_to_flash = 0;
				for(uint8_t index =0;index < p_input_pkg->command_len; index++)
				{
						uint16_t reg_for_write = p_input_pkg->register_id+index;
						//check register is not out of ragnge.
						if(reg_for_write > END_ADDR_NORMAL_REG)
						{
								p_sync_nus_comm->output_buf.reg_ack 		= REG_RES_UNSUCCESS;
								break;
						}
						//Check Permisstion 
						if( p_a2_reg->r_w_ctl[reg_for_write] == REG_READ_ONLY )
						{
							  //Skip to write register
								continue;
						}
						//Write data to register
						*p_a2_reg->reg[reg_for_write] = p_input_pkg->data[index];	
						//Check register address for need to save configuration in flash memory.
						if(reg_for_write >= BEGIN_ADDR_REG_CONFIG && reg_for_write <= END_ADDR_REG_CONFIG)
						{
							 flag_write_config_to_flash = 1;
						}
						p_sync_nus_comm->output_buf.reg_ack 		= REG_RES_SUCCESS;
				}
				//Check req save to flash
				if(flag_write_config_to_flash == 1)
				{
						p_sync_nus_comm->save_cfg_to_flash();
				}
				//Pack data to payload
			  p_sync_nus_comm->output_buf.register_id = p_input_pkg->register_id;
				p_sync_nus_comm->output_buf.pkg_status 	= FINAL_PKG;
				//Sent data to host
				p_sync_nus_comm->write(&p_sync_nus_comm->output_buf.register_id, MIN_INPUT_PKG_SIZE);
				NRF_LOG_INFO("OK sync write.");
		}else{
				//ACK FALL
				p_sync_nus_comm->output_buf.register_id = p_input_pkg->register_id;
				p_sync_nus_comm->output_buf.reg_ack 		= REG_RES_UNSUCCESS;
				p_sync_nus_comm->output_buf.pkg_status 	= FINAL_PKG;				
				p_sync_nus_comm->write(&p_sync_nus_comm->output_buf.register_id, MIN_INPUT_PKG_SIZE);
			  NRF_LOG_INFO("Fail sync read/write.");
				return;
		}
}

static void sync_nus_special_register_mngr(sync_nus_comm_t *p_sync_nus_comm, a2_reg_t *p_a2_reg)
{
		uint16_t buf_q_r;
	
		//Check package lenght
		que_front(&p_sync_nus_comm->que_input, &buf_q_r);
	
		//Get command and len of command
		sync_nus_input_t *p_input_pkg =  &p_sync_nus_comm->input_buf[buf_q_r];
	
		if(p_input_pkg->command_len > MAX_DATA_COMMAND)
		{
				//ACK FALL
				p_sync_nus_comm->output_buf.register_id = p_input_pkg->register_id;
				p_sync_nus_comm->output_buf.reg_ack 		= REG_RES_UNSUCCESS;
				p_sync_nus_comm->output_buf.pkg_status 	= FINAL_PKG;				
				p_sync_nus_comm->write(&p_sync_nus_comm->output_buf.register_id, MIN_INPUT_PKG_SIZE);
				return;
		}
		
		switch(p_input_pkg->register_id)
		{
			case REG_SPC_FREE_FALL_RAW_DATA_LOG:
				/*Waiting Implement*/
				sync_nus_special_free_fall_raw_data_reg_mngr(p_sync_nus_comm);
				break;
			case REG_SPC_BUTTON_ALEERT_LOG:
				sync_nus_special_button_alert_reg_mngr(p_sync_nus_comm);
				break;
			case REG_SPC_FALL_ALEERT_LOG:
				sync_nus_special_free_fall_reg_mngr(p_sync_nus_comm);
				break;
			case REG_SPC_PEDO_LOG:
				sync_nus_special_pedometer_reg_mngr(p_sync_nus_comm);
				break;
			case REG_SPC_HR_SPO2_TEMP_LOG:
				sync_nus_special_hr_spo2_temp_reg_mngr(p_sync_nus_comm);
				break;
			default:
				//ACK FALL
				return;
		}
}

static void sync_nus_special_button_alert_reg_mngr(sync_nus_comm_t *p_sync_nus_comm)
{
		a2_button_alert_log_t buf_data_log[PAGE_SIZE/sizeof(a2_button_alert_log_t)];
		uint16_t buf_q_r;
		uint16_t buf_fifo_size;
		uint16_t buf_que_r;
		uint16_t buf_que_f;
		uint16_t buf_page_begin;
		uint16_t buf_page_end;
		uint16_t size_of_data;
		uint16_t round;
		uint16_t page_number;
	
		//Check package lenght
		que_front(&p_sync_nus_comm->que_input, &buf_q_r);
	
		//Get command and len of command
		sync_nus_input_t *p_input_pkg =  &p_sync_nus_comm->input_buf[buf_q_r];	

		switch(p_input_pkg->command_read_write)
		{
			case SIZE_OF_FIFO:
				buf_fifo_size = que_size( &p_sync_nus_comm->p_queue->button_alert.q_page);
				p_sync_nus_comm->output_buf.register_id = p_input_pkg->register_id;
				p_sync_nus_comm->output_buf.reg_ack 		= REG_RES_SUCCESS; 
				p_sync_nus_comm->output_buf.pkg_status 	= FINAL_PKG;
				memcpy( &p_sync_nus_comm->output_buf.data[0], &buf_fifo_size, sizeof(buf_fifo_size));
				p_sync_nus_comm->write(&p_sync_nus_comm->output_buf.register_id, MIN_INPUT_PKG_SIZE+sizeof(buf_fifo_size) );
				break;
			case QUEUE_R_OF_PAGES:
				buf_que_r = p_sync_nus_comm->p_queue->button_alert.q_page.q_r;
				p_sync_nus_comm->output_buf.register_id = p_input_pkg->register_id;
				p_sync_nus_comm->output_buf.reg_ack 		= REG_RES_SUCCESS; 
				p_sync_nus_comm->output_buf.pkg_status 	= FINAL_PKG;
				memcpy( &p_sync_nus_comm->output_buf.data[0], &buf_que_r, sizeof(buf_que_r));
				p_sync_nus_comm->write(&p_sync_nus_comm->output_buf.register_id, MIN_INPUT_PKG_SIZE+sizeof(buf_que_r) );
				break;
			case QUEUE_F_OF_PAGES:
			  buf_que_f = p_sync_nus_comm->p_queue->button_alert.q_page.q_f;
				p_sync_nus_comm->output_buf.register_id = p_input_pkg->register_id;
				p_sync_nus_comm->output_buf.reg_ack 		= REG_RES_SUCCESS; 
				p_sync_nus_comm->output_buf.pkg_status 	= FINAL_PKG;
				memcpy( &p_sync_nus_comm->output_buf.data[0], &buf_que_f, sizeof(buf_que_f));
				p_sync_nus_comm->write(&p_sync_nus_comm->output_buf.register_id, MIN_INPUT_PKG_SIZE+sizeof(buf_que_f) );
				break;
			case BEGIN_PAGE_ADDR:
				buf_page_begin = p_sync_nus_comm->p_queue->button_alert.page_start;
				p_sync_nus_comm->output_buf.register_id = p_input_pkg->register_id;
				p_sync_nus_comm->output_buf.reg_ack 		= REG_RES_SUCCESS; 
				p_sync_nus_comm->output_buf.pkg_status 	= FINAL_PKG;
				memcpy( &p_sync_nus_comm->output_buf.data[0], &buf_page_begin, sizeof(buf_page_begin));
				p_sync_nus_comm->write(&p_sync_nus_comm->output_buf.register_id, MIN_INPUT_PKG_SIZE+sizeof(buf_page_begin) );
				break;
			case END_PAGE_ADDR:
				buf_page_end = p_sync_nus_comm->p_queue->button_alert.page_stop;
				p_sync_nus_comm->output_buf.register_id = p_input_pkg->register_id;
				p_sync_nus_comm->output_buf.reg_ack 		= REG_RES_SUCCESS; 
				p_sync_nus_comm->output_buf.pkg_status 	= FINAL_PKG;
				memcpy( &p_sync_nus_comm->output_buf.data[0], &buf_page_end, sizeof(buf_page_end));
				p_sync_nus_comm->write(&p_sync_nus_comm->output_buf.register_id, MIN_INPUT_PKG_SIZE+sizeof(buf_page_end) );
				break;
			case GET_DATA_FROM_PAGE_ADDR: 
				NRF_LOG_INFO("Button read from page %lu", page_number);
				NRF_LOG_FLUSH();
				nrf_delay_ms(300);
				memcpy( (uint8_t*)&page_number,  &p_input_pkg->data[0], sizeof(page_number));
				memset(&buf_data_log[0], 0x55, sizeof(buf_data_log));
				p_sync_nus_comm->wakeup_flash();
				a2_read_button_alert_log_by_page(p_sync_nus_comm->p_logging_dev, page_number, &buf_data_log[0]);
				p_sync_nus_comm->sleep_flash();
				NRF_LOG_INFO("HEX");
				NRF_LOG_HEXDUMP_INFO((uint8_t*)&buf_data_log[0], sizeof(buf_data_log[0])*2);
				NRF_LOG_FLUSH();
				nrf_delay_ms(300);
				size_of_data = sizeof(buf_data_log);
			  round = size_of_data/MAX_DATA_PAYLOAD;
				p_sync_nus_comm->output_buf.register_id = p_input_pkg->register_id;
				p_sync_nus_comm->output_buf.reg_ack 		= REG_RES_SUCCESS; 
				
				if(size_of_data%MAX_DATA_PAYLOAD == 0)
				{
						uint8_t *p_buffer = (uint8_t*)&buf_data_log[0];
						for(uint16_t i = 0;i < round -1;i++)
						{
								p_sync_nus_comm->output_buf.pkg_status 	= STILL_LEFT;
								memcpy( &p_sync_nus_comm->output_buf.data[0], p_buffer,MAX_DATA_PAYLOAD);
								p_sync_nus_comm->write(&p_sync_nus_comm->output_buf.register_id, MIN_INPUT_PKG_SIZE + MAX_DATA_PAYLOAD );
								p_buffer = p_buffer+MAX_DATA_PAYLOAD;
						}
						p_sync_nus_comm->output_buf.pkg_status 	= FINAL_PKG;
						memcpy( &p_sync_nus_comm->output_buf.data[0], p_buffer,MAX_DATA_PAYLOAD);
					  p_sync_nus_comm->write(&p_sync_nus_comm->output_buf.register_id, MIN_INPUT_PKG_SIZE + MAX_DATA_PAYLOAD );	
				}else{
						uint8_t *p_buffer = (uint8_t*)&buf_data_log[0];
						for(uint16_t i = 0;i < round;i++)
						{
								p_sync_nus_comm->output_buf.pkg_status 	= STILL_LEFT;
								memcpy(&p_sync_nus_comm->output_buf.data[0], p_buffer,MAX_DATA_PAYLOAD);
								p_sync_nus_comm->write(&p_sync_nus_comm->output_buf.register_id, MIN_INPUT_PKG_SIZE + MAX_DATA_PAYLOAD );
								p_buffer = p_buffer+MAX_DATA_PAYLOAD;
						}
						p_sync_nus_comm->output_buf.pkg_status 	= FINAL_PKG;
						memcpy(&p_sync_nus_comm->output_buf.data[0], p_buffer, size_of_data%MAX_DATA_PAYLOAD);
					  p_sync_nus_comm->write(&p_sync_nus_comm->output_buf.register_id, MIN_INPUT_PKG_SIZE + (size_of_data%MAX_DATA_PAYLOAD) );	
				}	  
				break;
			case GET_DATA_FROM_FIFO:
				if(que_isEmpty(&p_sync_nus_comm->p_queue->button_alert.q_page) == QUE_TURE)
				{
						//ACK FALL
						return;
				}
				p_sync_nus_comm->wakeup_flash();
				a2_read_button_alert_log_by_queue(p_sync_nus_comm->p_logging_dev, &p_sync_nus_comm->p_queue->button_alert, &buf_data_log[0]);
				p_sync_nus_comm->sleep_flash();
			  size_of_data = sizeof(buf_data_log);
			  round = size_of_data/MAX_DATA_PAYLOAD;
				p_sync_nus_comm->output_buf.register_id = p_input_pkg->register_id;
				p_sync_nus_comm->output_buf.reg_ack 		= REG_RES_SUCCESS; 
				
				if(size_of_data%MAX_DATA_PAYLOAD == 0)
				{
						uint8_t *p_buffer = (uint8_t*)&buf_data_log[0];
						for(uint16_t i = 0;i < round -1;i++)
						{
								p_sync_nus_comm->output_buf.pkg_status 	= STILL_LEFT;
								memcpy( &p_sync_nus_comm->output_buf.data[0], p_buffer,MAX_DATA_PAYLOAD);
								p_sync_nus_comm->write(&p_sync_nus_comm->output_buf.register_id, MIN_INPUT_PKG_SIZE + MAX_DATA_PAYLOAD );
								p_buffer = p_buffer+MAX_DATA_PAYLOAD;
						}
						p_sync_nus_comm->output_buf.pkg_status 	= FINAL_PKG;
						memcpy( &p_sync_nus_comm->output_buf.data[0], p_buffer,MAX_DATA_PAYLOAD);
					  p_sync_nus_comm->write(&p_sync_nus_comm->output_buf.register_id, MIN_INPUT_PKG_SIZE + MAX_DATA_PAYLOAD );	
				}else{
						uint8_t *p_buffer = (uint8_t*)&buf_data_log[0];
						for(uint16_t i = 0;i < round;i++)
						{
								p_sync_nus_comm->output_buf.pkg_status 	= STILL_LEFT;
								memcpy(&p_sync_nus_comm->output_buf.data[0], p_buffer,MAX_DATA_PAYLOAD);
								p_sync_nus_comm->write(&p_sync_nus_comm->output_buf.register_id, MIN_INPUT_PKG_SIZE + MAX_DATA_PAYLOAD );
								p_buffer = p_buffer+MAX_DATA_PAYLOAD;
						}
						p_sync_nus_comm->output_buf.pkg_status 	= FINAL_PKG;
						memcpy(&p_sync_nus_comm->output_buf.data[0], p_buffer, size_of_data%MAX_DATA_PAYLOAD);
					  p_sync_nus_comm->write(&p_sync_nus_comm->output_buf.register_id, MIN_INPUT_PKG_SIZE + (size_of_data%MAX_DATA_PAYLOAD) );	
				}	  
				break;
			case POP_DATA_FROM_FIFO:
				if(que_isEmpty(&p_sync_nus_comm->p_queue->button_alert.q_page) == QUE_TURE)
				{
						//ACK FALL
						p_sync_nus_comm->output_buf.register_id = p_input_pkg->register_id;
						p_sync_nus_comm->output_buf.reg_ack 		= REG_RES_UNSUCCESS; 
						p_sync_nus_comm->output_buf.pkg_status 	= FINAL_PKG;
						p_sync_nus_comm->write(&p_sync_nus_comm->output_buf.register_id, MIN_INPUT_PKG_SIZE);
						return;
				}	
				que_pop(&p_sync_nus_comm->p_queue->button_alert.q_page);
				p_sync_nus_comm->output_buf.register_id = p_input_pkg->register_id;
				p_sync_nus_comm->output_buf.reg_ack 		= REG_RES_SUCCESS; 
				p_sync_nus_comm->output_buf.pkg_status 	= FINAL_PKG;
				p_sync_nus_comm->write(&p_sync_nus_comm->output_buf.register_id, MIN_INPUT_PKG_SIZE);
				p_sync_nus_comm->wakeup_flash();
				a2_write_queue_config(p_sync_nus_comm->p_logging_dev, p_sync_nus_comm->p_queue);
				p_sync_nus_comm->sleep_flash();
				
				break;
		}
}

static void sync_nus_special_pedometer_reg_mngr(sync_nus_comm_t *p_sync_nus_comm)
{
		a2_raw_acc_gyro_log_t buf_data_log[PAGE_SIZE/sizeof(a2_raw_acc_gyro_log_t)];
		uint16_t buf_q_r;
		uint16_t buf_fifo_size;
		uint16_t buf_que_r;
		uint16_t buf_que_f;
		uint16_t buf_page_begin;
		uint16_t buf_page_end;
		uint16_t size_of_data;
		uint16_t round;
		uint16_t page_number;
	
		//Check package lenght
		que_front(&p_sync_nus_comm->que_input, &buf_q_r);
	
		//Get command and len of command
		sync_nus_input_t *p_input_pkg =  &p_sync_nus_comm->input_buf[buf_q_r];	

		switch(p_input_pkg->command_read_write)
		{
			case SIZE_OF_FIFO:
				buf_fifo_size = que_size( &p_sync_nus_comm->p_queue->pedometer.q_page);
				p_sync_nus_comm->output_buf.register_id = p_input_pkg->register_id;
				p_sync_nus_comm->output_buf.reg_ack 		= REG_RES_SUCCESS; 
				p_sync_nus_comm->output_buf.pkg_status 	= FINAL_PKG;
				memcpy( &p_sync_nus_comm->output_buf.data[0], &buf_fifo_size, sizeof(buf_fifo_size));
				p_sync_nus_comm->write(&p_sync_nus_comm->output_buf.register_id, MIN_INPUT_PKG_SIZE+sizeof(buf_fifo_size) );
				break;
			case QUEUE_R_OF_PAGES:
				buf_que_r = p_sync_nus_comm->p_queue->pedometer.q_page.q_r;
				p_sync_nus_comm->output_buf.register_id = p_input_pkg->register_id;
				p_sync_nus_comm->output_buf.reg_ack 		= REG_RES_SUCCESS; 
				p_sync_nus_comm->output_buf.pkg_status 	= FINAL_PKG;
				memcpy( &p_sync_nus_comm->output_buf.data[0], &buf_que_r, sizeof(buf_que_r));
				p_sync_nus_comm->write(&p_sync_nus_comm->output_buf.register_id, MIN_INPUT_PKG_SIZE+sizeof(buf_que_r) );
				break;
			case QUEUE_F_OF_PAGES:
			  buf_que_f = p_sync_nus_comm->p_queue->pedometer.q_page.q_f;
				p_sync_nus_comm->output_buf.register_id = p_input_pkg->register_id;
				p_sync_nus_comm->output_buf.reg_ack 		= REG_RES_SUCCESS; 
				p_sync_nus_comm->output_buf.pkg_status 	= FINAL_PKG;
				memcpy( &p_sync_nus_comm->output_buf.data[0], &buf_que_f, sizeof(buf_que_f));
				p_sync_nus_comm->write(&p_sync_nus_comm->output_buf.register_id, MIN_INPUT_PKG_SIZE+sizeof(buf_que_f) );
				break;
			case BEGIN_PAGE_ADDR:
				buf_page_begin = p_sync_nus_comm->p_queue->pedometer.page_start;
				p_sync_nus_comm->output_buf.register_id = p_input_pkg->register_id;
				p_sync_nus_comm->output_buf.reg_ack 		= REG_RES_SUCCESS; 
				p_sync_nus_comm->output_buf.pkg_status 	= FINAL_PKG;
				memcpy( &p_sync_nus_comm->output_buf.data[0], &buf_page_begin, sizeof(buf_page_begin));
				p_sync_nus_comm->write(&p_sync_nus_comm->output_buf.register_id, MIN_INPUT_PKG_SIZE+sizeof(buf_page_begin) );
				break;
			case END_PAGE_ADDR:
				buf_page_end = p_sync_nus_comm->p_queue->pedometer.page_stop;
				p_sync_nus_comm->output_buf.register_id = p_input_pkg->register_id;
				p_sync_nus_comm->output_buf.reg_ack 		= REG_RES_SUCCESS; 
				p_sync_nus_comm->output_buf.pkg_status 	= FINAL_PKG;
				memcpy( &p_sync_nus_comm->output_buf.data[0], &buf_page_end, sizeof(buf_page_end));
				p_sync_nus_comm->write(&p_sync_nus_comm->output_buf.register_id, MIN_INPUT_PKG_SIZE+sizeof(buf_page_end) );
				break;
			case GET_DATA_FROM_PAGE_ADDR: 
				memcpy( (uint8_t*)&page_number,  &p_input_pkg->data[0], sizeof(page_number));
				memset(&buf_data_log[0], 0x55, sizeof(buf_data_log));
				p_sync_nus_comm->wakeup_flash();
				a2_read_pedo_meter_log_by_page(p_sync_nus_comm->p_logging_dev, page_number, &buf_data_log[0]);
				p_sync_nus_comm->sleep_flash();
				size_of_data = sizeof(buf_data_log);
			  round = size_of_data/MAX_DATA_PAYLOAD;
				p_sync_nus_comm->output_buf.register_id = p_input_pkg->register_id;
				p_sync_nus_comm->output_buf.reg_ack 		= REG_RES_SUCCESS; 
				
				if(size_of_data%MAX_DATA_PAYLOAD == 0)
				{
						uint8_t *p_buffer = (uint8_t*)&buf_data_log[0];
						for(uint16_t i = 0;i < round -1;i++)
						{
								p_sync_nus_comm->output_buf.pkg_status 	= STILL_LEFT;
								memcpy( &p_sync_nus_comm->output_buf.data[0], p_buffer,MAX_DATA_PAYLOAD);
								p_sync_nus_comm->write(&p_sync_nus_comm->output_buf.register_id, MIN_INPUT_PKG_SIZE + MAX_DATA_PAYLOAD );
								p_buffer = p_buffer+MAX_DATA_PAYLOAD;
						}
						p_sync_nus_comm->output_buf.pkg_status 	= FINAL_PKG;
						memcpy( &p_sync_nus_comm->output_buf.data[0], p_buffer,MAX_DATA_PAYLOAD);
					  p_sync_nus_comm->write(&p_sync_nus_comm->output_buf.register_id, MIN_INPUT_PKG_SIZE + MAX_DATA_PAYLOAD );	
				}else{
						uint8_t *p_buffer = (uint8_t*)&buf_data_log[0];
						for(uint16_t i = 0;i < round;i++)
						{
								p_sync_nus_comm->output_buf.pkg_status 	= STILL_LEFT;
								memcpy(&p_sync_nus_comm->output_buf.data[0], p_buffer,MAX_DATA_PAYLOAD);
								p_sync_nus_comm->write(&p_sync_nus_comm->output_buf.register_id, MIN_INPUT_PKG_SIZE + MAX_DATA_PAYLOAD );
								p_buffer = p_buffer+MAX_DATA_PAYLOAD;
						}
						p_sync_nus_comm->output_buf.pkg_status 	= FINAL_PKG;
						memcpy(&p_sync_nus_comm->output_buf.data[0], p_buffer, size_of_data%MAX_DATA_PAYLOAD);
					  p_sync_nus_comm->write(&p_sync_nus_comm->output_buf.register_id, MIN_INPUT_PKG_SIZE + (size_of_data%MAX_DATA_PAYLOAD) );	
				}	  
				break;
			case GET_DATA_FROM_FIFO:
				if(que_isEmpty(&p_sync_nus_comm->p_queue->pedometer.q_page) == QUE_TURE)
				{
						//ACK FALL
						return;
				}
				p_sync_nus_comm->wakeup_flash();
				a2_read_pedo_meter_log_by_queue(p_sync_nus_comm->p_logging_dev, &p_sync_nus_comm->p_queue->pedometer, &buf_data_log[0]);
				p_sync_nus_comm->sleep_flash();
			  size_of_data = sizeof(buf_data_log);
			  round = size_of_data/MAX_DATA_PAYLOAD;
				p_sync_nus_comm->output_buf.register_id = p_input_pkg->register_id;
				p_sync_nus_comm->output_buf.reg_ack 		= REG_RES_SUCCESS; 
				
				if(size_of_data%MAX_DATA_PAYLOAD == 0)
				{
						uint8_t *p_buffer = (uint8_t*)&buf_data_log[0];
						for(uint16_t i = 0;i < round -1;i++)
						{
								p_sync_nus_comm->output_buf.pkg_status 	= STILL_LEFT;
								memcpy( &p_sync_nus_comm->output_buf.data[0], p_buffer,MAX_DATA_PAYLOAD);
								p_sync_nus_comm->write(&p_sync_nus_comm->output_buf.register_id, MIN_INPUT_PKG_SIZE + MAX_DATA_PAYLOAD );
								p_buffer = p_buffer+MAX_DATA_PAYLOAD;
						}
						p_sync_nus_comm->output_buf.pkg_status 	= FINAL_PKG;
						memcpy( &p_sync_nus_comm->output_buf.data[0], p_buffer,MAX_DATA_PAYLOAD);
					  p_sync_nus_comm->write(&p_sync_nus_comm->output_buf.register_id, MIN_INPUT_PKG_SIZE + MAX_DATA_PAYLOAD );	
				}else{
						uint8_t *p_buffer = (uint8_t*)&buf_data_log[0];
						for(uint16_t i = 0;i < round;i++)
						{
								p_sync_nus_comm->output_buf.pkg_status 	= STILL_LEFT;
								memcpy(&p_sync_nus_comm->output_buf.data[0], p_buffer,MAX_DATA_PAYLOAD);
								p_sync_nus_comm->write(&p_sync_nus_comm->output_buf.register_id, MIN_INPUT_PKG_SIZE + MAX_DATA_PAYLOAD );
								p_buffer = p_buffer+MAX_DATA_PAYLOAD;
						}
						p_sync_nus_comm->output_buf.pkg_status 	= FINAL_PKG;
						memcpy(&p_sync_nus_comm->output_buf.data[0], p_buffer, size_of_data%MAX_DATA_PAYLOAD);
					  p_sync_nus_comm->write(&p_sync_nus_comm->output_buf.register_id, MIN_INPUT_PKG_SIZE + (size_of_data%MAX_DATA_PAYLOAD) );	
				}	  
				break;
			case POP_DATA_FROM_FIFO:
				if(que_isEmpty(&p_sync_nus_comm->p_queue->pedometer.q_page) == QUE_TURE)
				{
						//ACK FALL
						p_sync_nus_comm->output_buf.register_id = p_input_pkg->register_id;
						p_sync_nus_comm->output_buf.reg_ack 		= REG_RES_UNSUCCESS; 
						p_sync_nus_comm->output_buf.pkg_status 	= FINAL_PKG;
						p_sync_nus_comm->write(&p_sync_nus_comm->output_buf.register_id, MIN_INPUT_PKG_SIZE);
						return;
				}	
				que_pop(&p_sync_nus_comm->p_queue->pedometer.q_page);
				p_sync_nus_comm->output_buf.register_id = p_input_pkg->register_id;
				p_sync_nus_comm->output_buf.reg_ack 		= REG_RES_SUCCESS; 
				p_sync_nus_comm->output_buf.pkg_status 	= FINAL_PKG;
				p_sync_nus_comm->write(&p_sync_nus_comm->output_buf.register_id, MIN_INPUT_PKG_SIZE);
				p_sync_nus_comm->wakeup_flash();
				a2_write_queue_config(p_sync_nus_comm->p_logging_dev, p_sync_nus_comm->p_queue);
				p_sync_nus_comm->sleep_flash();
				
				break;
		}
}

static void sync_nus_special_hr_spo2_temp_reg_mngr(sync_nus_comm_t *p_sync_nus_comm)
{
		a2_raw_max30102_log_t buf_data_log[PAGE_SIZE/sizeof(a2_raw_max30102_log_t)];
		uint16_t buf_q_r;
		uint16_t buf_fifo_size;
		uint16_t buf_que_r;
		uint16_t buf_que_f;
		uint16_t buf_page_begin;
		uint16_t buf_page_end;
		uint16_t size_of_data;
		uint16_t round;
		uint16_t page_number;
	
		//Check package lenght
		que_front(&p_sync_nus_comm->que_input, &buf_q_r);
	
		//Get command and len of command
		sync_nus_input_t *p_input_pkg =  &p_sync_nus_comm->input_buf[buf_q_r];	

		switch(p_input_pkg->command_read_write)
		{
			case SIZE_OF_FIFO:
				buf_fifo_size = que_size( &p_sync_nus_comm->p_queue->hr_spo2_temperature.q_page);
				p_sync_nus_comm->output_buf.register_id = p_input_pkg->register_id;
				p_sync_nus_comm->output_buf.reg_ack 		= REG_RES_SUCCESS; 
				p_sync_nus_comm->output_buf.pkg_status 	= FINAL_PKG;
				memcpy( &p_sync_nus_comm->output_buf.data[0], &buf_fifo_size, sizeof(buf_fifo_size));
				p_sync_nus_comm->write(&p_sync_nus_comm->output_buf.register_id, MIN_INPUT_PKG_SIZE+sizeof(buf_fifo_size) );
				break;
			case QUEUE_R_OF_PAGES:
				buf_que_r = p_sync_nus_comm->p_queue->hr_spo2_temperature.q_page.q_r;
				p_sync_nus_comm->output_buf.register_id = p_input_pkg->register_id;
				p_sync_nus_comm->output_buf.reg_ack 		= REG_RES_SUCCESS; 
				p_sync_nus_comm->output_buf.pkg_status 	= FINAL_PKG;
				memcpy( &p_sync_nus_comm->output_buf.data[0], &buf_que_r, sizeof(buf_que_r));
				p_sync_nus_comm->write(&p_sync_nus_comm->output_buf.register_id, MIN_INPUT_PKG_SIZE+sizeof(buf_que_r) );
				break;
			case QUEUE_F_OF_PAGES:
			  buf_que_f = p_sync_nus_comm->p_queue->hr_spo2_temperature.q_page.q_f;
				p_sync_nus_comm->output_buf.register_id = p_input_pkg->register_id;
				p_sync_nus_comm->output_buf.reg_ack 		= REG_RES_SUCCESS; 
				p_sync_nus_comm->output_buf.pkg_status 	= FINAL_PKG;
				memcpy( &p_sync_nus_comm->output_buf.data[0], &buf_que_f, sizeof(buf_que_f));
				p_sync_nus_comm->write(&p_sync_nus_comm->output_buf.register_id, MIN_INPUT_PKG_SIZE+sizeof(buf_que_f) );
				break;
			case BEGIN_PAGE_ADDR:
				buf_page_begin = p_sync_nus_comm->p_queue->hr_spo2_temperature.page_start;
				p_sync_nus_comm->output_buf.register_id = p_input_pkg->register_id;
				p_sync_nus_comm->output_buf.reg_ack 		= REG_RES_SUCCESS; 
				p_sync_nus_comm->output_buf.pkg_status 	= FINAL_PKG;
				memcpy( &p_sync_nus_comm->output_buf.data[0], &buf_page_begin, sizeof(buf_page_begin));
				p_sync_nus_comm->write(&p_sync_nus_comm->output_buf.register_id, MIN_INPUT_PKG_SIZE+sizeof(buf_page_begin) );
				break;
			case END_PAGE_ADDR:
				buf_page_end = p_sync_nus_comm->p_queue->hr_spo2_temperature.page_stop;
				p_sync_nus_comm->output_buf.register_id = p_input_pkg->register_id;
				p_sync_nus_comm->output_buf.reg_ack 		= REG_RES_SUCCESS; 
				p_sync_nus_comm->output_buf.pkg_status 	= FINAL_PKG;
				memcpy( &p_sync_nus_comm->output_buf.data[0], &buf_page_end, sizeof(buf_page_end));
				p_sync_nus_comm->write(&p_sync_nus_comm->output_buf.register_id, MIN_INPUT_PKG_SIZE+sizeof(buf_page_end) );
				break;
			case GET_DATA_FROM_PAGE_ADDR: 
				memcpy( (uint8_t*)&page_number,  &p_input_pkg->data[0], sizeof(page_number));
				memset(&buf_data_log[0], 0x55, sizeof(buf_data_log));
				p_sync_nus_comm->wakeup_flash();
				a2_read_hr_spo2_temp_log_by_page(p_sync_nus_comm->p_logging_dev, page_number, &buf_data_log[0]);
				p_sync_nus_comm->sleep_flash();
				size_of_data = sizeof(buf_data_log);
			  round = size_of_data/MAX_DATA_PAYLOAD;
				p_sync_nus_comm->output_buf.register_id = p_input_pkg->register_id;
				p_sync_nus_comm->output_buf.reg_ack 		= REG_RES_SUCCESS; 
				
				if(size_of_data%MAX_DATA_PAYLOAD == 0)
				{
						uint8_t *p_buffer = (uint8_t*)&buf_data_log[0];
						for(uint16_t i = 0;i < round -1;i++)
						{
								p_sync_nus_comm->output_buf.pkg_status 	= STILL_LEFT;
								memcpy( &p_sync_nus_comm->output_buf.data[0], p_buffer,MAX_DATA_PAYLOAD);
								p_sync_nus_comm->write(&p_sync_nus_comm->output_buf.register_id, MIN_INPUT_PKG_SIZE + MAX_DATA_PAYLOAD );
								p_buffer = p_buffer+MAX_DATA_PAYLOAD;
						}
						p_sync_nus_comm->output_buf.pkg_status 	= FINAL_PKG;
						memcpy( &p_sync_nus_comm->output_buf.data[0], p_buffer,MAX_DATA_PAYLOAD);
					  p_sync_nus_comm->write(&p_sync_nus_comm->output_buf.register_id, MIN_INPUT_PKG_SIZE + MAX_DATA_PAYLOAD );	
				}else{
						uint8_t *p_buffer = (uint8_t*)&buf_data_log[0];
						for(uint16_t i = 0;i < round;i++)
						{
								p_sync_nus_comm->output_buf.pkg_status 	= STILL_LEFT;
								memcpy(&p_sync_nus_comm->output_buf.data[0], p_buffer,MAX_DATA_PAYLOAD);
								p_sync_nus_comm->write(&p_sync_nus_comm->output_buf.register_id, MIN_INPUT_PKG_SIZE + MAX_DATA_PAYLOAD );
								p_buffer = p_buffer+MAX_DATA_PAYLOAD;
						}
						p_sync_nus_comm->output_buf.pkg_status 	= FINAL_PKG;
						memcpy(&p_sync_nus_comm->output_buf.data[0], p_buffer, size_of_data%MAX_DATA_PAYLOAD);
					  p_sync_nus_comm->write(&p_sync_nus_comm->output_buf.register_id, MIN_INPUT_PKG_SIZE + (size_of_data%MAX_DATA_PAYLOAD) );	
				}	  
				break;
			case GET_DATA_FROM_FIFO:
				if(que_isEmpty(&p_sync_nus_comm->p_queue->hr_spo2_temperature.q_page) == QUE_TURE)
				{
						//ACK FALL
						return;
				}
				p_sync_nus_comm->wakeup_flash();
				a2_read_hr_spo2_temp_log_by_queue(p_sync_nus_comm->p_logging_dev, &p_sync_nus_comm->p_queue->hr_spo2_temperature, &buf_data_log[0]);
				p_sync_nus_comm->sleep_flash();
			  size_of_data = sizeof(buf_data_log);
			  round = size_of_data/MAX_DATA_PAYLOAD;
				p_sync_nus_comm->output_buf.register_id = p_input_pkg->register_id;
				p_sync_nus_comm->output_buf.reg_ack 		= REG_RES_SUCCESS; 
				
				if(size_of_data%MAX_DATA_PAYLOAD == 0)
				{
						uint8_t *p_buffer = (uint8_t*)&buf_data_log[0];
						for(uint16_t i = 0;i < round -1;i++)
						{
								p_sync_nus_comm->output_buf.pkg_status 	= STILL_LEFT;
								memcpy( &p_sync_nus_comm->output_buf.data[0], p_buffer,MAX_DATA_PAYLOAD);
								p_sync_nus_comm->write(&p_sync_nus_comm->output_buf.register_id, MIN_INPUT_PKG_SIZE + MAX_DATA_PAYLOAD );
								p_buffer = p_buffer+MAX_DATA_PAYLOAD;
						}
						p_sync_nus_comm->output_buf.pkg_status 	= FINAL_PKG;
						memcpy( &p_sync_nus_comm->output_buf.data[0], p_buffer,MAX_DATA_PAYLOAD);
					  p_sync_nus_comm->write(&p_sync_nus_comm->output_buf.register_id, MIN_INPUT_PKG_SIZE + MAX_DATA_PAYLOAD );	
				}else{
						uint8_t *p_buffer = (uint8_t*)&buf_data_log[0];
						for(uint16_t i = 0;i < round;i++)
						{
								p_sync_nus_comm->output_buf.pkg_status 	= STILL_LEFT;
								memcpy(&p_sync_nus_comm->output_buf.data[0], p_buffer,MAX_DATA_PAYLOAD);
								p_sync_nus_comm->write(&p_sync_nus_comm->output_buf.register_id, MIN_INPUT_PKG_SIZE + MAX_DATA_PAYLOAD );
								p_buffer = p_buffer+MAX_DATA_PAYLOAD;
						}
						p_sync_nus_comm->output_buf.pkg_status 	= FINAL_PKG;
						memcpy(&p_sync_nus_comm->output_buf.data[0], p_buffer, size_of_data%MAX_DATA_PAYLOAD);
					  p_sync_nus_comm->write(&p_sync_nus_comm->output_buf.register_id, MIN_INPUT_PKG_SIZE + (size_of_data%MAX_DATA_PAYLOAD) );	
				}	  
				break;
			case POP_DATA_FROM_FIFO:
				if(que_isEmpty(&p_sync_nus_comm->p_queue->pedometer.q_page) == QUE_TURE)
				{
						//ACK FALL
						p_sync_nus_comm->output_buf.register_id = p_input_pkg->register_id;
						p_sync_nus_comm->output_buf.reg_ack 		= REG_RES_UNSUCCESS; 
						p_sync_nus_comm->output_buf.pkg_status 	= FINAL_PKG;
						p_sync_nus_comm->write(&p_sync_nus_comm->output_buf.register_id, MIN_INPUT_PKG_SIZE);
						return;
				}	
				que_pop(&p_sync_nus_comm->p_queue->hr_spo2_temperature.q_page);
				p_sync_nus_comm->output_buf.register_id = p_input_pkg->register_id;
				p_sync_nus_comm->output_buf.reg_ack 		= REG_RES_SUCCESS; 
				p_sync_nus_comm->output_buf.pkg_status 	= FINAL_PKG;
				p_sync_nus_comm->write(&p_sync_nus_comm->output_buf.register_id, MIN_INPUT_PKG_SIZE);
				p_sync_nus_comm->wakeup_flash();
				a2_write_queue_config(p_sync_nus_comm->p_logging_dev, p_sync_nus_comm->p_queue);
				p_sync_nus_comm->sleep_flash();
				
				break;
		}
}

static void sync_nus_special_free_fall_reg_mngr(sync_nus_comm_t *p_sync_nus_comm)
{
		a2_fall_alert_log_t buf_data_log[PAGE_SIZE/sizeof(a2_fall_alert_log_t)];
		uint16_t buf_q_r;
		uint16_t buf_fifo_size;
		uint16_t buf_que_r;
		uint16_t buf_que_f;
		uint16_t buf_page_begin;
		uint16_t buf_page_end;
		uint16_t size_of_data;
		uint16_t round;
		uint16_t page_number;
	
		//Check package lenght
		que_front(&p_sync_nus_comm->que_input, &buf_q_r);
	
		//Get command and len of command
		sync_nus_input_t *p_input_pkg =  &p_sync_nus_comm->input_buf[buf_q_r];	

		switch(p_input_pkg->command_read_write)
		{
			case SIZE_OF_FIFO:
				buf_fifo_size = que_size( &p_sync_nus_comm->p_queue->fall_alert.q_page);
				p_sync_nus_comm->output_buf.register_id = p_input_pkg->register_id;
				p_sync_nus_comm->output_buf.reg_ack 		= REG_RES_SUCCESS; 
				p_sync_nus_comm->output_buf.pkg_status 	= FINAL_PKG;
				memcpy( &p_sync_nus_comm->output_buf.data[0], &buf_fifo_size, sizeof(buf_fifo_size));
				p_sync_nus_comm->write(&p_sync_nus_comm->output_buf.register_id, MIN_INPUT_PKG_SIZE+sizeof(buf_fifo_size) );
				break;
			case QUEUE_R_OF_PAGES:
				buf_que_r = p_sync_nus_comm->p_queue->fall_alert.q_page.q_r;
				p_sync_nus_comm->output_buf.register_id = p_input_pkg->register_id;
				p_sync_nus_comm->output_buf.reg_ack 		= REG_RES_SUCCESS; 
				p_sync_nus_comm->output_buf.pkg_status 	= FINAL_PKG;
				memcpy( &p_sync_nus_comm->output_buf.data[0], &buf_que_r, sizeof(buf_que_r));
				p_sync_nus_comm->write(&p_sync_nus_comm->output_buf.register_id, MIN_INPUT_PKG_SIZE+sizeof(buf_que_r) );
				break;
			case QUEUE_F_OF_PAGES:
			  buf_que_f = p_sync_nus_comm->p_queue->fall_alert.q_page.q_f;
				p_sync_nus_comm->output_buf.register_id = p_input_pkg->register_id;
				p_sync_nus_comm->output_buf.reg_ack 		= REG_RES_SUCCESS; 
				p_sync_nus_comm->output_buf.pkg_status 	= FINAL_PKG;
				memcpy( &p_sync_nus_comm->output_buf.data[0], &buf_que_f, sizeof(buf_que_f));
				p_sync_nus_comm->write(&p_sync_nus_comm->output_buf.register_id, MIN_INPUT_PKG_SIZE+sizeof(buf_que_f) );
				break;
			case BEGIN_PAGE_ADDR:
				buf_page_begin = p_sync_nus_comm->p_queue->fall_alert.page_start;
				p_sync_nus_comm->output_buf.register_id = p_input_pkg->register_id;
				p_sync_nus_comm->output_buf.reg_ack 		= REG_RES_SUCCESS; 
				p_sync_nus_comm->output_buf.pkg_status 	= FINAL_PKG;
				memcpy( &p_sync_nus_comm->output_buf.data[0], &buf_page_begin, sizeof(buf_page_begin));
				p_sync_nus_comm->write(&p_sync_nus_comm->output_buf.register_id, MIN_INPUT_PKG_SIZE+sizeof(buf_page_begin) );
				break;
			case END_PAGE_ADDR:
				buf_page_end = p_sync_nus_comm->p_queue->fall_alert.page_stop;
				p_sync_nus_comm->output_buf.register_id = p_input_pkg->register_id;
				p_sync_nus_comm->output_buf.reg_ack 		= REG_RES_SUCCESS; 
				p_sync_nus_comm->output_buf.pkg_status 	= FINAL_PKG;
				memcpy( &p_sync_nus_comm->output_buf.data[0], &buf_page_end, sizeof(buf_page_end));
				p_sync_nus_comm->write(&p_sync_nus_comm->output_buf.register_id, MIN_INPUT_PKG_SIZE+sizeof(buf_page_end) );
				break;
			case GET_DATA_FROM_PAGE_ADDR: 
				memcpy( (uint8_t*)&page_number,  &p_input_pkg->data[0], sizeof(page_number));
				memset(&buf_data_log[0], 0x55, sizeof(buf_data_log));
				p_sync_nus_comm->wakeup_flash();
				a2_read_free_fall_alert_log_by_page(p_sync_nus_comm->p_logging_dev, page_number, &buf_data_log[0]);
				p_sync_nus_comm->sleep_flash();
				size_of_data = sizeof(buf_data_log);
			  round = size_of_data/MAX_DATA_PAYLOAD;
				p_sync_nus_comm->output_buf.register_id = p_input_pkg->register_id;
				p_sync_nus_comm->output_buf.reg_ack 		= REG_RES_SUCCESS; 
				
				if(size_of_data%MAX_DATA_PAYLOAD == 0)
				{
						uint8_t *p_buffer = (uint8_t*)&buf_data_log[0];
						for(uint16_t i = 0;i < round -1;i++)
						{
								p_sync_nus_comm->output_buf.pkg_status 	= STILL_LEFT;
								memcpy( &p_sync_nus_comm->output_buf.data[0], p_buffer,MAX_DATA_PAYLOAD);
								p_sync_nus_comm->write(&p_sync_nus_comm->output_buf.register_id, MIN_INPUT_PKG_SIZE + MAX_DATA_PAYLOAD );
								p_buffer = p_buffer+MAX_DATA_PAYLOAD;
						}
						p_sync_nus_comm->output_buf.pkg_status 	= FINAL_PKG;
						memcpy( &p_sync_nus_comm->output_buf.data[0], p_buffer,MAX_DATA_PAYLOAD);
					  p_sync_nus_comm->write(&p_sync_nus_comm->output_buf.register_id, MIN_INPUT_PKG_SIZE + MAX_DATA_PAYLOAD );	
				}else{
						uint8_t *p_buffer = (uint8_t*)&buf_data_log[0];
						for(uint16_t i = 0;i < round;i++)
						{
								p_sync_nus_comm->output_buf.pkg_status 	= STILL_LEFT;
								memcpy(&p_sync_nus_comm->output_buf.data[0], p_buffer,MAX_DATA_PAYLOAD);
								p_sync_nus_comm->write(&p_sync_nus_comm->output_buf.register_id, MIN_INPUT_PKG_SIZE + MAX_DATA_PAYLOAD );
								p_buffer = p_buffer+MAX_DATA_PAYLOAD;
						}
						p_sync_nus_comm->output_buf.pkg_status 	= FINAL_PKG;
						memcpy(&p_sync_nus_comm->output_buf.data[0], p_buffer, size_of_data%MAX_DATA_PAYLOAD);
					  p_sync_nus_comm->write(&p_sync_nus_comm->output_buf.register_id, MIN_INPUT_PKG_SIZE + (size_of_data%MAX_DATA_PAYLOAD) );	
				}	  
				break;
			case GET_DATA_FROM_FIFO:
				if(que_isEmpty(&p_sync_nus_comm->p_queue->fall_alert.q_page) == QUE_TURE)
				{
						//ACK FALL
						return;
				}
				p_sync_nus_comm->wakeup_flash();
				a2_read_free_fall_alert_log_by_queue(p_sync_nus_comm->p_logging_dev, &p_sync_nus_comm->p_queue->fall_alert, &buf_data_log[0]);
				p_sync_nus_comm->sleep_flash();
			  size_of_data = sizeof(buf_data_log);
			  round = size_of_data/MAX_DATA_PAYLOAD;
				p_sync_nus_comm->output_buf.register_id = p_input_pkg->register_id;
				p_sync_nus_comm->output_buf.reg_ack 		= REG_RES_SUCCESS; 
				
				if(size_of_data%MAX_DATA_PAYLOAD == 0)
				{
						uint8_t *p_buffer = (uint8_t*)&buf_data_log[0];
						for(uint16_t i = 0;i < round -1;i++)
						{
								p_sync_nus_comm->output_buf.pkg_status 	= STILL_LEFT;
								memcpy( &p_sync_nus_comm->output_buf.data[0], p_buffer,MAX_DATA_PAYLOAD);
								p_sync_nus_comm->write(&p_sync_nus_comm->output_buf.register_id, MIN_INPUT_PKG_SIZE + MAX_DATA_PAYLOAD );
								p_buffer = p_buffer+MAX_DATA_PAYLOAD;
						}
						p_sync_nus_comm->output_buf.pkg_status 	= FINAL_PKG;
						memcpy( &p_sync_nus_comm->output_buf.data[0], p_buffer,MAX_DATA_PAYLOAD);
					  p_sync_nus_comm->write(&p_sync_nus_comm->output_buf.register_id, MIN_INPUT_PKG_SIZE + MAX_DATA_PAYLOAD );	
				}else{
						uint8_t *p_buffer = (uint8_t*)&buf_data_log[0];
						for(uint16_t i = 0;i < round;i++)
						{
								p_sync_nus_comm->output_buf.pkg_status 	= STILL_LEFT;
								memcpy(&p_sync_nus_comm->output_buf.data[0], p_buffer,MAX_DATA_PAYLOAD);
								p_sync_nus_comm->write(&p_sync_nus_comm->output_buf.register_id, MIN_INPUT_PKG_SIZE + MAX_DATA_PAYLOAD );
								p_buffer = p_buffer+MAX_DATA_PAYLOAD;
						}
						p_sync_nus_comm->output_buf.pkg_status 	= FINAL_PKG;
						memcpy(&p_sync_nus_comm->output_buf.data[0], p_buffer, size_of_data%MAX_DATA_PAYLOAD);
					  p_sync_nus_comm->write(&p_sync_nus_comm->output_buf.register_id, MIN_INPUT_PKG_SIZE + (size_of_data%MAX_DATA_PAYLOAD) );	
				}	  
				break;
			case POP_DATA_FROM_FIFO:
				if(que_isEmpty(&p_sync_nus_comm->p_queue->fall_alert.q_page) == QUE_TURE)
				{
						//ACK FALL
						p_sync_nus_comm->output_buf.register_id = p_input_pkg->register_id;
						p_sync_nus_comm->output_buf.reg_ack 		= REG_RES_UNSUCCESS; 
						p_sync_nus_comm->output_buf.pkg_status 	= FINAL_PKG;
						p_sync_nus_comm->write(&p_sync_nus_comm->output_buf.register_id, MIN_INPUT_PKG_SIZE);
						return;
				}	
				que_pop(&p_sync_nus_comm->p_queue->fall_alert.q_page);
				p_sync_nus_comm->output_buf.register_id = p_input_pkg->register_id;
				p_sync_nus_comm->output_buf.reg_ack 		= REG_RES_SUCCESS; 
				p_sync_nus_comm->output_buf.pkg_status 	= FINAL_PKG;
				p_sync_nus_comm->write(&p_sync_nus_comm->output_buf.register_id, MIN_INPUT_PKG_SIZE);
				p_sync_nus_comm->wakeup_flash();
				a2_write_queue_config(p_sync_nus_comm->p_logging_dev, p_sync_nus_comm->p_queue);
				p_sync_nus_comm->sleep_flash();
				
				break;
		}	
}

a2_raw_acc_gyro_fall_log_t raw_free_fall_buf_data_log;

static void sync_nus_special_free_fall_raw_data_reg_mngr(sync_nus_comm_t *p_sync_nus_comm)
{

		uint16_t buf_q_r;
		uint16_t buf_fifo_size;
		uint16_t buf_que_r;
		uint16_t buf_que_f;
		uint16_t buf_page_begin;
		uint16_t buf_page_end;
		uint16_t size_of_data;
		uint16_t round;
	
		//Check package lenght
		que_front(&p_sync_nus_comm->que_input, &buf_q_r);
	
		//Get command and len of command
		sync_nus_input_t *p_input_pkg =  &p_sync_nus_comm->input_buf[buf_q_r];	

		
		
		switch(p_input_pkg->command_read_write)
		{
			case SIZE_OF_FIFO:
				buf_fifo_size = que_size( &p_sync_nus_comm->p_queue->fall_alert_raw_data.q_data);
				p_sync_nus_comm->output_buf.register_id = p_input_pkg->register_id;
				p_sync_nus_comm->output_buf.reg_ack 		= REG_RES_SUCCESS; 
				p_sync_nus_comm->output_buf.pkg_status 	= FINAL_PKG;
				memcpy( &p_sync_nus_comm->output_buf.data[0], &buf_fifo_size, sizeof(buf_fifo_size));
				p_sync_nus_comm->write(&p_sync_nus_comm->output_buf.register_id, MIN_INPUT_PKG_SIZE+sizeof(buf_fifo_size) );
				break;
			case QUEUE_R_OF_PAGES:
				buf_que_r = p_sync_nus_comm->p_queue->fall_alert_raw_data.q_data.q_r;
				p_sync_nus_comm->output_buf.register_id = p_input_pkg->register_id;
				p_sync_nus_comm->output_buf.reg_ack 		= REG_RES_SUCCESS; 
				p_sync_nus_comm->output_buf.pkg_status 	= FINAL_PKG;
				memcpy( &p_sync_nus_comm->output_buf.data[0], &buf_que_r, sizeof(buf_que_r));
				p_sync_nus_comm->write(&p_sync_nus_comm->output_buf.register_id, MIN_INPUT_PKG_SIZE+sizeof(buf_que_r) );
				break;
			case QUEUE_F_OF_PAGES:
			  buf_que_f = p_sync_nus_comm->p_queue->fall_alert_raw_data.q_data.q_f;
				p_sync_nus_comm->output_buf.register_id = p_input_pkg->register_id;
				p_sync_nus_comm->output_buf.reg_ack 		= REG_RES_SUCCESS; 
				p_sync_nus_comm->output_buf.pkg_status 	= FINAL_PKG;
				memcpy( &p_sync_nus_comm->output_buf.data[0], &buf_que_f, sizeof(buf_que_f));
				p_sync_nus_comm->write(&p_sync_nus_comm->output_buf.register_id, MIN_INPUT_PKG_SIZE+sizeof(buf_que_f) );
				break;
			case BEGIN_PAGE_ADDR:
				buf_page_begin = p_sync_nus_comm->p_queue->fall_alert_raw_data.page_start;
				p_sync_nus_comm->output_buf.register_id = p_input_pkg->register_id;
				p_sync_nus_comm->output_buf.reg_ack 		= REG_RES_SUCCESS; 
				p_sync_nus_comm->output_buf.pkg_status 	= FINAL_PKG;
				memcpy( &p_sync_nus_comm->output_buf.data[0], &buf_page_begin, sizeof(buf_page_begin));
				p_sync_nus_comm->write(&p_sync_nus_comm->output_buf.register_id, MIN_INPUT_PKG_SIZE+sizeof(buf_page_begin) );
				break;
			case END_PAGE_ADDR:
				buf_page_end = p_sync_nus_comm->p_queue->fall_alert_raw_data.page_stop;
				p_sync_nus_comm->output_buf.register_id = p_input_pkg->register_id;
				p_sync_nus_comm->output_buf.reg_ack 		= REG_RES_SUCCESS; 
				p_sync_nus_comm->output_buf.pkg_status 	= FINAL_PKG;
				memcpy( &p_sync_nus_comm->output_buf.data[0], &buf_page_end, sizeof(buf_page_end));
				p_sync_nus_comm->write(&p_sync_nus_comm->output_buf.register_id, MIN_INPUT_PKG_SIZE+sizeof(buf_page_end) );
				break;
			case GET_DATA_FROM_PAGE_ADDR: 
				//Not Support ACK FALL to host
				p_sync_nus_comm->output_buf.register_id = p_input_pkg->register_id;
				p_sync_nus_comm->output_buf.reg_ack 		= REG_RES_UNSUCCESS; 
				p_sync_nus_comm->output_buf.pkg_status 	= FINAL_PKG;
				p_sync_nus_comm->write(&p_sync_nus_comm->output_buf.register_id, MIN_INPUT_PKG_SIZE);
				break;
			case GET_DATA_FROM_FIFO:
				
				if(que_isEmpty(&p_sync_nus_comm->p_queue->fall_alert_raw_data.q_data) == QUE_TURE)
				{
						//ACK FALL
						return;
				}
				p_sync_nus_comm->wakeup_flash();
				a2_read_raw_data_free_fall_log_by_queue(p_sync_nus_comm->p_logging_dev, &p_sync_nus_comm->p_queue->fall_alert_raw_data, &raw_free_fall_buf_data_log);
				p_sync_nus_comm->sleep_flash();
			  size_of_data = sizeof(raw_free_fall_buf_data_log);
			  round = size_of_data/MAX_DATA_PAYLOAD;
				p_sync_nus_comm->output_buf.register_id = p_input_pkg->register_id;
				p_sync_nus_comm->output_buf.reg_ack 		= REG_RES_SUCCESS; 
				
				NRF_LOG_INFO("Size of pkg %ld, round, %ld", sizeof(raw_free_fall_buf_data_log), round); 
				if(size_of_data%MAX_DATA_PAYLOAD == 0)
				{
						uint8_t *p_buffer = (uint8_t*)&raw_free_fall_buf_data_log;
						for(uint16_t i = 0;i < round -1;i++)
						{
								p_sync_nus_comm->output_buf.pkg_status 	= STILL_LEFT;
								memcpy( &p_sync_nus_comm->output_buf.data[0], p_buffer,MAX_DATA_PAYLOAD);
								p_sync_nus_comm->write(&p_sync_nus_comm->output_buf.register_id, MIN_INPUT_PKG_SIZE + MAX_DATA_PAYLOAD );
								p_buffer = p_buffer+MAX_DATA_PAYLOAD;
								nrf_delay_ms(10);
						}
						p_sync_nus_comm->output_buf.pkg_status 	= FINAL_PKG;
						memcpy( &p_sync_nus_comm->output_buf.data[0], p_buffer,MAX_DATA_PAYLOAD);
					  p_sync_nus_comm->write(&p_sync_nus_comm->output_buf.register_id, MIN_INPUT_PKG_SIZE + MAX_DATA_PAYLOAD );	
				}else{
						uint8_t *p_buffer = (uint8_t*)&raw_free_fall_buf_data_log;
						for(uint16_t i = 0;i < round;i++)
						{
								p_sync_nus_comm->output_buf.pkg_status 	= STILL_LEFT;
								memcpy(&p_sync_nus_comm->output_buf.data[0], p_buffer,MAX_DATA_PAYLOAD);
								p_sync_nus_comm->write(&p_sync_nus_comm->output_buf.register_id, MIN_INPUT_PKG_SIZE + MAX_DATA_PAYLOAD );
								p_buffer = p_buffer+MAX_DATA_PAYLOAD;
							 nrf_delay_ms(10);
						}
						p_sync_nus_comm->output_buf.pkg_status 	= FINAL_PKG;
						memcpy(&p_sync_nus_comm->output_buf.data[0], p_buffer, size_of_data%MAX_DATA_PAYLOAD);
					  p_sync_nus_comm->write(&p_sync_nus_comm->output_buf.register_id, MIN_INPUT_PKG_SIZE + (size_of_data%MAX_DATA_PAYLOAD) );	
				}	 
				NRF_LOG_INFO("GET_DATA_FROM_FIFO");		
				break;
			case POP_DATA_FROM_FIFO:
				if(que_isEmpty(&p_sync_nus_comm->p_queue->fall_alert_raw_data.q_data) == QUE_TURE)
				{
						//ACK FALL
						p_sync_nus_comm->output_buf.register_id = p_input_pkg->register_id;
						p_sync_nus_comm->output_buf.reg_ack 		= REG_RES_UNSUCCESS; 
						p_sync_nus_comm->output_buf.pkg_status 	= FINAL_PKG;
						p_sync_nus_comm->write(&p_sync_nus_comm->output_buf.register_id, MIN_INPUT_PKG_SIZE);
						return;
				}	
				que_pop(&p_sync_nus_comm->p_queue->fall_alert_raw_data.q_data);
				p_sync_nus_comm->output_buf.register_id = p_input_pkg->register_id;
				p_sync_nus_comm->output_buf.reg_ack 		= REG_RES_SUCCESS; 
				p_sync_nus_comm->output_buf.pkg_status 	= FINAL_PKG;
				p_sync_nus_comm->write(&p_sync_nus_comm->output_buf.register_id, MIN_INPUT_PKG_SIZE);
				p_sync_nus_comm->wakeup_flash();
				a2_write_queue_config(p_sync_nus_comm->p_logging_dev, p_sync_nus_comm->p_queue);
				p_sync_nus_comm->sleep_flash();
				
				break;
		}			
}
	
	

void sync_nus_append_command(sync_nus_comm_t *p_sync_nus_comm, uint8_t *p_data, uint16_t len)
{
		uint16_t buf_q_r;
	
		if(que_isFull(&p_sync_nus_comm->que_input) == QUE_TURE)
		{
				return;
		}
		
		que_r(&p_sync_nus_comm->que_input, &buf_q_r);	
		memcpy(&p_sync_nus_comm->input_buf[buf_q_r].register_id, p_data, len);
		p_sync_nus_comm->input_len[buf_q_r] = len;
		que_append(&p_sync_nus_comm->que_input,0, 0, 0);	
	
}

uint8_t TEST_NUS_WRITE_FUNCTION(uint8_t *data, uint16_t len)
{
	 NRF_LOG_INFO("TEST_NUS_WRITE_FUNCTION");
	 NRF_LOG_HEXDUMP_INFO(data,len);
	 NRF_LOG_FLUSH();
	 
		for(uint16_t i=0;i<30;i++)
		{
				NRF_LOG_PROCESS();
				nrf_delay_ms(10);
		}
		
	
	return 0;
}
