#include "at45db081e.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "nrf_delay.h"


uint32_t at45dbxx_initial(at45dbxx_dev_t *dev)
{
		dev->buffer_mgr.active_buffer = 0;
		dev->buffer_mgr.buffer_addr[0] = AT45DB_BUFFER_1;
		dev->buffer_mgr.buffer_addr[1] = AT45DB_BUFFER_2;
		dev->buffer_mgr.buf_to_page_addr[0] = AT45DB_BUF_1_TO_PAGE;
		dev->buffer_mgr.buf_to_page_addr[1] = AT45DB_BUF_2_TO_PAGE;
		return AT45DB_SUCCESS;
}

uint32_t at45dbxx_deinitial(at45dbxx_dev_t *dev)
{
		
		return AT45DB_SUCCESS;
}

uint32_t at45dbxx_deep_sleep(at45dbxx_dev_t *dev)
{

		return AT45DB_SUCCESS;
}

uint32_t at45dbxx_wakeup(at45dbxx_dev_t *dev)
{

		return AT45DB_SUCCESS;
}

uint32_t at45dbxx_wakeup_pin(at45dbxx_dev_t *dev)
{
		uint8_t at45_tx_buf = 0xFF;
		uint8_t at45_rx_buf;
	
		dev->write(&at45_tx_buf, sizeof(at45_tx_buf), &at45_rx_buf, 0 , 0);
	
		return 0;
}
uint32_t at45dbxx_is_available(at45dbxx_dev_t *dev)
{
		uint8_t tx_buf = MANUFACTURER_DEVICE_ID;
		at45dbxx_device_info_t device_info;
	
		dev->read(&tx_buf, sizeof(tx_buf), (uint8_t*)&device_info, sizeof(device_info), 0);
	
		NRF_LOG_INFO("Flash Manufac, 0x%02X ,Device ID 0x%02X%02X", device_info.menu_fac, device_info.device_id[0], device_info.device_id[1]);
		NRF_LOG_FLUSH();
	  nrf_delay_ms(10);
		if(device_info.menu_fac != DEFAULT_Manufacturer)
		{
				return AT45DB_ERROR_NOT_FOUND;
		}
		return AT45DB_SUCCESS;
}

uint32_t at45dbxx_get_menufac_and_device_id(at45dbxx_dev_t *dev, at45dbxx_device_info_t *device_info)
{
	
		return AT45DB_SUCCESS;
}

//int32_t at45dbxx_wakeup(at45dbxx_dev_t *dev)
//{
//		
//		return AT45DB_SUCCESS;
//}

uint32_t at34dbxx_deep_power_down(at45dbxx_dev_t *dev)
{
		uint8_t tx_buf = Deep_Power_Down;
		uint8_t rx_buf;
	
		dev->write(&tx_buf, sizeof(tx_buf), &rx_buf , sizeof(rx_buf), 0);
	
		
		return AT45DB_SUCCESS;
}
uint32_t at45dbxx_ultra_deep_power_down(at45dbxx_dev_t *dev)
{
		uint8_t command = Ultra_Deep_Power_Down;
		uint8_t res;
	
		dev->write(&command, 1, &res , 0, 0);
		
		return AT45DB_SUCCESS;
}
	
uint32_t at45dbxx_resume_deep_power_down(at45dbxx_dev_t *dev)
{
		uint8_t command = Resume_From_Deep_Power;
		uint8_t res;
	
		dev->write(&command, sizeof(command), &res , 0, 0);
		return AT45DB_SUCCESS;
}

uint32_t at45dbxx_resume_ultra_deep_power_down(at45dbxx_dev_t *dev)
{
	
		return AT45DB_SUCCESS;
}

uint32_t at45db_busy_wait(at45dbxx_dev_t *dev)
{
		uint32_t time_out_counter = TIME_OUT_COUNTER_MAX;
		uint8_t tx_buf = Status_Reg_Read;
		at45dbxx_status_reg_t status_reg;
	
		do
		{
				dev->read(&tx_buf, sizeof(tx_buf), (uint8_t*)&status_reg, sizeof(status_reg), 0);
				time_out_counter--;
				if(status_reg.byte1.bits.rdy_busy == 0){
						dev->delay_ms(100);  
						//NRF_LOG_INFO("Chip flash busy");
						//NRF_LOG_FLUSH();
				}else if(time_out_counter == 0)
				{
					  NRF_LOG_INFO("Chip flash timeout");
						return AT45DB_ERROR_TIMEOUT;
				}else{
						//NRF_LOG_INFO("Chip flash ready");
				}
		}while(status_reg.byte1.bits.rdy_busy == 0);
		
		return AT45DB_SUCCESS;	
}


uint32_t at45db_erase_chip(at45dbxx_dev_t *dev) 
{
		uint8_t res;
	
		/*chip erase command consists of 4 byte*/
		uint8_t cmd[4] = { 0xC7, 0x94, 0x80, 0x9A };
		dev->write(&cmd[0], sizeof(cmd), &res , 0 , 0);
		res = at45db_busy_wait(dev);
				
		return res;		
}

uint32_t at45db_erase_block(at45dbxx_dev_t *dev, uint16_t addr) 
{
		uint8_t res,dummy;	
		/*block erase command consists of 4 byte*/
		uint8_t cmd[4] = { AT45DB_BLOCK_ERASE, (uint8_t) (addr >> 3), (uint8_t) (addr << 5), 0x00};
		dev->write(&cmd[0], sizeof(cmd), &dummy , 0, 0);
		res = at45db_busy_wait(dev);
		
		return res;
}
uint32_t at45db_erase_page(at45dbxx_dev_t *dev, uint16_t addr) 
{
		uint8_t res, dummy;	
		/*block erase command consists of 4 byte*/
		uint8_t cmd[4] = { AT45DB_PAGE_ERASE, (uint8_t) (addr >> 7),(uint8_t) (addr << 1), 0x00 };
		dev->write(&cmd[0], sizeof(cmd), &dummy , 0, 0);
		res = at45db_busy_wait(dev);
		
		return res;
}

uint32_t at45db_write_buffer(at45dbxx_dev_t *dev, uint16_t offset, uint8_t buffer[264], uint16_t bytes) 
{
		uint8_t res, dummy;
	
		/*block erase command consists of 4 byte*/
		//uint8_t cmd[4] = { dev->buffer_mgr.buffer_addr[dev->buffer_mgr.active_buffer], 0x00,(uint8_t) (addr >> 8), (uint8_t) (addr) };
		uint8_t cmd[4] = { AT45DB_BUFFER_2, 0x00,(uint8_t) (offset >> 8), (uint8_t) (offset) };
		dev->write(&cmd[0], sizeof(cmd), &dummy , 0, AT45DB_ENABLE_CONTINUES_CS_LOW);
		dev->write(&buffer[0], bytes, &dummy , 0, AT45DB_DISABLE_CONTINUES_CS_LOW);
		res = at45db_busy_wait(dev);
		
		return res;
}

uint32_t at45db_buffer_to_page(at45dbxx_dev_t *dev, uint16_t addr) 
{
		uint8_t res;
		uint8_t dummy;
		/*wait until AT45DB161 is ready again*/
		at45db_busy_wait(dev);
		/*write active buffer to page command consists of 4 byte*/
//		uint8_t cmd[4] = { dev->buffer_mgr.buf_to_page_addr[dev->buffer_mgr.active_buffer],
//										(uint8_t) (addr >> 6), (uint8_t) (addr << 2), 0x00 };
//		
		//uint8_t cmd[4] = { AT45DB_BUFFER_2, (uint8_t) (addr >> 6), (uint8_t) (addr << 2), 0x00 };
		uint8_t cmd[4] = { AT45DB_BUF_2_TO_PAGE, (uint8_t) (addr >> 7), (uint8_t) (addr << 1), 0x00 };
		
										
		res = dev->write(&cmd[0], sizeof(cmd), &dummy , 0, AT45DB_DISABLE_CONTINUES_CS_LOW);
		/* switch active buffer to allow the other one to be written,
		 * while these buffer is copied to the Flash EEPROM page*/
		dev->buffer_mgr.active_buffer ^= 1;
				
		at45db_busy_wait(dev);
		
		return res;
}

uint32_t at45db_read_page_buffered(at45dbxx_dev_t *dev, uint16_t p_addr, uint16_t b_addr,
                uint8_t *buffer, uint16_t bytes) 
{
		uint8_t res;
		/*wait until AT45DB161 is ready again*/
		res = at45db_busy_wait(dev);
		res = at45db_page_to_buf(dev, p_addr);
		res = at45db_read_buffer(dev, b_addr, buffer, bytes);
									
		return res;				
}
								
uint32_t at45db_page_to_buf(at45dbxx_dev_t *dev, uint16_t addr) 
{
			uint8_t res,buf;
			/*write active buffer to page command consists of 4 byte*/
			uint8_t cmd[4] = { AT45DB_PAGE_TO_BUF, (uint8_t) (addr >> 7),	(uint8_t) (addr << 1), 0x00 };
			res = dev->write(&cmd[0], sizeof(cmd), &buf , 0, AT45DB_DISABLE_CONTINUES_CS_LOW);

			at45db_busy_wait(dev);	
			/* switch active buffer to allow the other one to be written,
			* while these buffer is copied to the Flash EEPROM page*/
			//buffer_mgr.active_buffer ^= 1;
					
			return res;		
}
uint32_t at45db_read_buffer(at45dbxx_dev_t *dev, uint16_t b_offset, uint8_t *buffer, uint16_t bytes) 
{
		uint8_t res;
		uint8_t dummy[6];
	
		uint8_t cmd[4] = { AT45DB_READ_BUFFER, 0x00, (uint8_t) (b_offset >> 8),(uint8_t) (b_offset) };
		//at45db_busy_wait(dev);
		res = dev->write(&cmd[0], sizeof(cmd), &dummy[0] , 5, AT45DB_ENABLE_CONTINUES_CS_LOW);
 		res = dev->read(0, 0 ,  &buffer[0], bytes, AT45DB_DISABLE_CONTINUES_CS_LOW);
		
		return res;		
}


uint32_t at45db_read_page_bypassed(at45dbxx_dev_t *dev, uint16_t p_addr, uint16_t b_addr,
                uint8_t *buffer, uint16_t bytes) 
{
		uint16_t res;
		/*wait until AT45DB161 is ready again*/
		at45db_busy_wait(dev);
		/* read bytes directly from page command consists of 4 cmd bytes and
		* 4 don't care */
		uint8_t cmd[4] = { AT45DB_PAGE_READ, (uint8_t) (p_addr >> 6),
				(((uint8_t) (p_addr << 2)) & 0xFC) | ((uint8_t) (b_addr >> 8)),
				(uint8_t) (b_addr) };
		res = dev->write(&cmd[0], sizeof(cmd), buffer , bytes, AT45DB_DISABLE_CONTINUES_CS_LOW);
				
		return res;
}

//		NRF_LOG_INFO("Status flash memory bytes1 0x%02X :: 0x%02X ", status_reg.byte1.data, status_reg.byte2.data);
//	
//		NRF_LOG_INFO("Status flash memory rdy_busy %d", status_reg.byte1.bits.rdy_busy);
//		NRF_LOG_FLUSH();
//		NRF_LOG_INFO("Status flash memory comp %d", status_reg.byte1.bits.comp);
//		NRF_LOG_FLUSH();
//		NRF_LOG_INFO("Status flash memory desity 0x%02X", status_reg.byte1.bits.desity);
//		NRF_LOG_FLUSH();
//		NRF_LOG_INFO("Status flash memory protect %d", status_reg.byte1.bits.protect);
//		NRF_LOG_FLUSH();
//		NRF_LOG_INFO("Status flash memory page_size %d", status_reg.byte1.bits.page_size);
//	
//		NRF_LOG_FLUSH();
//	
//		NRF_LOG_INFO("Status flash memory rdy_busy %d", status_reg.byte2.bits.rdy_busy);
//		NRF_LOG_FLUSH();
//		NRF_LOG_INFO("Status flash memory res1 %d", status_reg.byte2.bits.res1);
//		NRF_LOG_FLUSH();	
//		NRF_LOG_INFO("Status flash memory epe %d", status_reg.byte2.bits.epe);
//		NRF_LOG_FLUSH();	
//		NRF_LOG_INFO("Status flash memory res2 %d", status_reg.byte2.bits.res2);
//		NRF_LOG_FLUSH();
//		NRF_LOG_INFO("Status flash memory sle %d", status_reg.byte2.bits.sle);
//		NRF_LOG_FLUSH();
//		NRF_LOG_INFO("Status flash memory ps2 %d", status_reg.byte2.bits.ps2);
//		NRF_LOG_FLUSH();
//		NRF_LOG_INFO("Status flash memory ps1 %d", status_reg.byte2.bits.ps1);
//		
//		NRF_LOG_INFO("Status flash memory es %d", status_reg.byte2.bits.es);
//		
//		
//		NRF_LOG_FLUSH();
