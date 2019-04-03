#include "a2_logging_mngr.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "nrf_delay.h"
#include "nrf_drv_spi_bmi_flash.h"

const uint8_t DEFUALT_NAME[DEFUALT_LENGTH]			 = {'B','a','e','s','L','A','B'};
const uint8_t DEFUALT_SURNAME[DEFUALT_LENGTH]    = {'C','o','m','p','a','n','y'};

#ifdef __TEST_LOGGING_MNGR__

static uint8_t buf_write[300];
static uint8_t buf_read[300];
static a2_button_alert_log_t  	read_buf_test_data_button_log[30];
static a2_fall_alert_log_t   	read_buf_test_data_fall_alert_log[30];
static a2_raw_acc_gyro_log_t		read_buf_test_data_raw_acc_gyro_log[30];
static a2_raw_max30102_log_t		read_buf_test_raw_max30102_log[30];
static uint8_t res_log;

void print_out()
{
		uint8_t counter=255;
		while(counter--)
		{
				NRF_LOG_PROCESS();
				nrf_delay_ms(1);
		}
}
void TEST_PRINT_SIZE_OF_STRUCT(void)
{
		NRF_LOG_INFO("Size %ld ::a2_sensor_config", sizeof(a2_sensor_config_t));
		print_out();
		NRF_LOG_INFO("Size %ld ::a2_user_data_config_t", sizeof(a2_user_data_config_t));
		print_out();
		NRF_LOG_INFO("Size %ld ::a2_fifo_address_t", sizeof(a2_working_info_t));
		print_out();
		NRF_LOG_INFO("Size %ld ::a2_button_alert_log_t", sizeof(a2_button_alert_log_t));
		print_out();
		NRF_LOG_INFO("Size %ld ::a2_fall_alert_log_t", sizeof(a2_fall_alert_log_t));
	
		NRF_LOG_INFO("Size %ld ::a2_raw_acc_gyro_fall_log_t", sizeof(a2_raw_acc_gyro_fall_log_t));
		print_out();
		NRF_LOG_INFO("Size %ld ::a2_raw_acc_gyro_log_t", sizeof(a2_raw_acc_gyro_log_t));
		print_out();
		NRF_LOG_INFO("Size %ld ::a2_raw_max30102_log_t", sizeof(a2_raw_max30102_log_t));
		print_out();
	
		NRF_LOG_INFO("Size %ld ::a2_config_t", sizeof(a2_config_t));
		print_out();
	
}

void TEST_READ_WRITE_CONFIG(void)
{
		
}


void TEST_WRITE_AND_READ_ALL_FLASH(logging_dev_t *dev)
{
		int16_t count, count2;
		int16_t res_cmp;
	
		NRF_LOG_INFO("*******  TEST Write and Read all flash memory **********");
		NRF_LOG_FLUSH(); 
		nrf_delay_ms(300);
		NRF_LOG_INFO("Erasing chip..");
		NRF_LOG_FLUSH(); 
		nrf_delay_ms(300);
	  flash_erase_chip();
	
		NRF_LOG_INFO("Write Page Start");
		NRF_LOG_FLUSH();
		
		for(count = 0;count<=4095;count++)
		{
				if(count%2==0)
				{
					for(count2 = 0;count2<264;count2++)
					{
							buf_write[count2] = count2&0xFF; 	
					}	
				}else{
					for(count2 = 263;count2>=0;count2--)
					{
							buf_write[263-count2] = count2&0xFF; 	
					}		
				}
				flash_write(count, (uint8_t*)&buf_write[0], 264);
				NRF_LOG_RAW_INFO(".");
				NRF_LOG_FLUSH(); 
				nrf_delay_ms(10);
		}
		
		NRF_LOG_INFO("Write Page END");
		NRF_LOG_FLUSH();
		for(count = 0;count<=4095;count++)
		{
				if(count%2==0)
				{
					for(count2 = 0;count2<264;count2++)
					{
							buf_write[count2] = count2&0xFF; 	
					}	
				}else{
					for(count2 = 263;count2>=0;count2--)
					{
							buf_write[263-count2] = count2&0xFF; 	
					}		
				}
				flash_read(count, (uint8_t*)&buf_read[0], 264);	
				
				if(count == 5)
				{
						buf_read[100] = 0;
							buf_read[200] = 0;
					
				}
				res_cmp = 0;
				for(count2 = 0;count2<264;count2++)
				{
						if(buf_read[count2] != buf_write[count2])
						{
								res_cmp++;
						}
				}
				NRF_LOG_INFO("Read Page,%d,with cmp,%d,", count, res_cmp);
				NRF_LOG_FLUSH(); 
				nrf_delay_ms(100);
		}
		
		NRF_LOG_INFO("END TEST Write and Read all flash memory");	

}

void TEST_READ_WRITE_FIFO_ADDRESS(logging_dev_t *dev, queue_type_t *p_queue_type)
{
//		//Test write button alert log
//		a2_button_alert_log_t  	test_data_button_log;
//		test_data_button_log.unix_time = 0x11223344;
//		test_data_button_log.cancel_time = 0x99887766;
//		test_data_button_log.level = 0x55;
//		test_data_button_log.cancel_detect = 0x66;//BUTTON_IS_NOT_DETECT;
//		test_data_button_log.sum_number_push_button = 0x5E;
//		
//		for(uint16_t i=0;i<30;i++)
//		{
//				test_data_button_log.level = i;
//				a2_write_button_alert_log(dev, &p_queue_type->button_alert, &test_data_button_log);
//				NRF_LOG_INFO("Button Round %d", i);
//				NRF_LOG_PROCESS();
//				nrf_delay_ms(500);
//		}
//		
//		res_log = a2_read_button_alert_log_by_queue(dev, &p_queue_type->button_alert, read_buf_test_data_button_log);
//		uint16_t test_pages = 0;
//		memset((void*)read_buf_test_data_button_log,0x00,sizeof(read_buf_test_data_button_log));
//		res_log = que_front(&p_queue_type->button_alert.q_page, &test_pages); 
//		uint16_t page_num_for_read = test_pages+ p_queue_type->button_alert.page_start;
//		a2_read_button_alert_log_by_page(dev, page_num_for_read, read_buf_test_data_button_log);
//		NRF_LOG_INFO("%d", res_log);
//		
//		//Test write free fall alert log
//		a2_fall_alert_log_t   	test_data_fall_alert_log;
//		test_data_fall_alert_log.unix_time 		= 0x11223344;
//		test_data_fall_alert_log.cancel_time 	= 0x99887766;
//		test_data_fall_alert_log.level 				= 0x55;
//		test_data_fall_alert_log.cancel_detect = 0x66;
//		test_data_fall_alert_log.fall_number = 0x5E;
//		
//		for(uint16_t i=0;i<30;i++)
//		{
//				test_data_fall_alert_log.fall_number = i;
//				a2_write_free_fall_alert_log(dev, &p_queue_type->fall_alert, &test_data_fall_alert_log);
//				NRF_LOG_INFO("Free Fall %d", i);
//				NRF_LOG_PROCESS();
//				nrf_delay_ms(500);
//		}
//		
//		res_log = a2_read_free_fall_alert_log_by_queue(dev, &p_queue_type->fall_alert, read_buf_test_data_fall_alert_log);
//		test_pages = 0;
//		memset((void*)read_buf_test_data_fall_alert_log,0x00,sizeof(read_buf_test_data_fall_alert_log));
//		res_log = que_front(&p_queue_type->fall_alert.q_page, &test_pages); 
//		page_num_for_read = test_pages+ p_queue_type->fall_alert.page_start;
//		a2_read_free_fall_alert_log_by_page(dev, page_num_for_read, read_buf_test_data_fall_alert_log);
//		NRF_LOG_INFO("%d", res_log);
//		
//		//Test write pedo log
//		a2_raw_acc_gyro_log_t		test_data_raw_acc_gyro_log;
//		test_data_raw_acc_gyro_log.unix_time 		= 0x12345678;
//		test_data_raw_acc_gyro_log.ax 	= 0x1122;
//		test_data_raw_acc_gyro_log.ay 	= 0x3344;
//		test_data_raw_acc_gyro_log.az 	= 0x5566;
//		test_data_raw_acc_gyro_log.gx 	= 0x1122;
//		test_data_raw_acc_gyro_log.gy 	= 0x3344;
//		test_data_raw_acc_gyro_log.gz 	= 0x5566;
//		test_data_raw_acc_gyro_log.step_count = 0x1234;
//		test_data_raw_acc_gyro_log.cal 				= 0x5678;
//		test_data_raw_acc_gyro_log.distance		= 0x8765;
//		
//		for(uint16_t i=0;i<30;i++)
//		{
//				a2_write_pedo_meter_log(dev, &p_queue_type->pedometer, &test_data_raw_acc_gyro_log);
//				NRF_LOG_INFO("Pedometer %d", i);
//				NRF_LOG_PROCESS();
//				nrf_delay_ms(500);
//		}
//		
//		res_log = a2_read_pedo_meter_log_by_queue(dev, &p_queue_type->pedometer, read_buf_test_data_raw_acc_gyro_log);
//		test_pages = 0;
//		memset((void*)read_buf_test_data_raw_acc_gyro_log,0x00,sizeof(read_buf_test_data_raw_acc_gyro_log));
//		res_log = que_front(&p_queue_type->pedometer.q_page, &test_pages); 
//		page_num_for_read = test_pages+ p_queue_type->pedometer.page_start;
//		a2_read_pedo_meter_log_by_page(dev, page_num_for_read, read_buf_test_data_raw_acc_gyro_log);
//		NRF_LOG_INFO("%d", res_log);		
//		
//		//Test write max30102 log
//	  a2_raw_max30102_log_t		test_raw_max30102_log;
//		test_raw_max30102_log.unix_time 		= 0x12345678;
//		test_raw_max30102_log.hr 	= 0x1122;
//		test_raw_max30102_log.spo2 	= 0x3344;
//		test_raw_max30102_log.temperature 	= 0x5566;
//		test_raw_max30102_log.skin_humdiy 	= 0x13;
//		test_raw_max30102_log.population_variance 	= 0.4321;
//		
//		for(uint16_t i=0;i<30;i++)
//		{
//				a2_write_hr_spo2_temp_log(dev, &p_queue_type->hr_spo2_temperature, &test_raw_max30102_log);
//				NRF_LOG_INFO("Max30102 %d", i);
//				NRF_LOG_PROCESS();
//				nrf_delay_ms(500);
//		}
//		
//		res_log = a2_read_hr_spo2_temp_log_by_queue(dev, &p_queue_type->hr_spo2_temperature, read_buf_test_raw_max30102_log);
//		test_pages = 0;
//		memset((void*)read_buf_test_raw_max30102_log,0x00,sizeof(read_buf_test_raw_max30102_log));
//		res_log = que_front(&p_queue_type->hr_spo2_temperature.q_page, &test_pages); 
//		page_num_for_read = test_pages+ p_queue_type->pedometer.page_start;
//		a2_read_hr_spo2_temp_log_by_page(dev, page_num_for_read, read_buf_test_raw_max30102_log);
//		NRF_LOG_INFO("%d", res_log);		
		
}

a2_raw_acc_gyro_fall_log_t read_test_data_a2_raw_acc_gyro_fall_log[3];
a2_raw_acc_gyro_fall_log_t test_data_a2_raw_acc_gyro_fall_log;




void TEST_READ_RAW_FREE_FALL_LOG(logging_dev_t *dev, queue_type_t *p_queue_type)
{
		test_data_a2_raw_acc_gyro_fall_log.unix_time 		= 0x12345678;
		test_data_a2_raw_acc_gyro_fall_log.len 					= 10;
		test_data_a2_raw_acc_gyro_fall_log.fall_number 	= 0xFE;
		for(int i=0;i<MAX_ACC_GYRO_DATA_LOG_SIZE;i++)
		{
				test_data_a2_raw_acc_gyro_fall_log.ax[i] = i;
				test_data_a2_raw_acc_gyro_fall_log.ay[i] = i;
				test_data_a2_raw_acc_gyro_fall_log.az[i] = i;
				test_data_a2_raw_acc_gyro_fall_log.gx[i] = i;
				test_data_a2_raw_acc_gyro_fall_log.gy[i] = i;
				test_data_a2_raw_acc_gyro_fall_log.gz[i] = i;
		}
		

		a2_write_raw_data_free_fall_log(dev, &p_queue_type->fall_alert_raw_data, &test_data_a2_raw_acc_gyro_fall_log);
		a2_write_raw_data_free_fall_log(dev, &p_queue_type->fall_alert_raw_data, &test_data_a2_raw_acc_gyro_fall_log);
		a2_write_raw_data_free_fall_log(dev, &p_queue_type->fall_alert_raw_data, &test_data_a2_raw_acc_gyro_fall_log);
		a2_write_raw_data_free_fall_log(dev, &p_queue_type->fall_alert_raw_data, &test_data_a2_raw_acc_gyro_fall_log);
		a2_write_raw_data_free_fall_log(dev, &p_queue_type->fall_alert_raw_data, &test_data_a2_raw_acc_gyro_fall_log);
		a2_write_raw_data_free_fall_log(dev, &p_queue_type->fall_alert_raw_data, &test_data_a2_raw_acc_gyro_fall_log);
		a2_write_raw_data_free_fall_log(dev, &p_queue_type->fall_alert_raw_data, &test_data_a2_raw_acc_gyro_fall_log);
		a2_write_raw_data_free_fall_log(dev, &p_queue_type->fall_alert_raw_data, &test_data_a2_raw_acc_gyro_fall_log);
		a2_write_raw_data_free_fall_log(dev, &p_queue_type->fall_alert_raw_data, &test_data_a2_raw_acc_gyro_fall_log);
		a2_write_raw_data_free_fall_log(dev, &p_queue_type->fall_alert_raw_data, &test_data_a2_raw_acc_gyro_fall_log);
		a2_write_raw_data_free_fall_log(dev, &p_queue_type->fall_alert_raw_data, &test_data_a2_raw_acc_gyro_fall_log);
		a2_read_raw_data_free_fall_log_by_queue(dev, &p_queue_type->fall_alert_raw_data, read_test_data_a2_raw_acc_gyro_fall_log);
		a2_pop_raw_data_free_fall_log(dev, &p_queue_type->fall_alert_raw_data);
		a2_read_raw_data_free_fall_log_by_queue(dev, &p_queue_type->fall_alert_raw_data, read_test_data_a2_raw_acc_gyro_fall_log);
		a2_pop_raw_data_free_fall_log(dev, &p_queue_type->fall_alert_raw_data);
		a2_write_raw_data_free_fall_log(dev, &p_queue_type->fall_alert_raw_data, &test_data_a2_raw_acc_gyro_fall_log);
		a2_write_raw_data_free_fall_log(dev, &p_queue_type->fall_alert_raw_data, &test_data_a2_raw_acc_gyro_fall_log);
		a2_write_raw_data_free_fall_log(dev, &p_queue_type->fall_alert_raw_data, &test_data_a2_raw_acc_gyro_fall_log);		
		
		
		NRF_LOG_INFO("");	
}
#endif

uint8_t logging_queue_initial(logging_dev_t *dev, queue_type_t *p_queue_type)
{
		uint32_t size_of_queue_setting = 0;
		uint8_t *p_queue_in_byte;
		uint16_t crc = 0;
		uint32_t k = 0;
		queue_type_t buf_queue_type;
		
		size_of_queue_setting = sizeof(buf_queue_type);
		a2_read_queue_config(dev, &buf_queue_type);

		crc = 0;
		p_queue_in_byte = (uint8_t*)&buf_queue_type;
		for( k=0; k < size_of_queue_setting-2; k++)
		{
				crc = crc ^ *p_queue_in_byte;
				p_queue_in_byte++;
		}
 
		if(crc != buf_queue_type.crc16)
		{
				NRF_LOG_INFO("[FALL]\tQueue setting in flash CRC not match crc, CRC with cal 0x%04X, CRC in flash 0x%04X", crc, buf_queue_type.crc16);
				
				//Write defualt queue setting
				memset(p_queue_type, 0x00, sizeof(*p_queue_type));

				//Intial Button Logging Queue Pages
				p_queue_type->button_alert.data_type  = BUTTON_ALERT_TYPE;
				p_queue_type->button_alert.page_start = BUTTON_ALERT_PAGE_START;
				p_queue_type->button_alert.page_stop  = BUTTON_ALERT_PAGE_STOP;
				que_initial(&p_queue_type->button_alert.q_page, BUTTON_ALERT_PAGE_STOP-BUTTON_ALERT_PAGE_START);
				que_initial(&p_queue_type->button_alert.q_data, (PAGE_SIZE/sizeof(a2_button_alert_log_t)) + 1);   // +1 for use full queue, In normal case fifo have 1 fifo gab.

				//Intial Free Fall Logging Queue Pages
				p_queue_type->fall_alert.data_type  = FALL_ALERT_LOG_TYPE;
				p_queue_type->fall_alert.page_start = FREE_FALL_ALERT_PAGE_START;
				p_queue_type->fall_alert.page_stop  = FREE_FALL_ALERT_PAGE_STOP;
				que_initial(&p_queue_type->fall_alert.q_page, FREE_FALL_ALERT_PAGE_STOP-FREE_FALL_ALERT_PAGE_START);
				que_initial(&p_queue_type->fall_alert.q_data, (PAGE_SIZE/sizeof(a2_fall_alert_log_t)) + 1);

				//Intial Pedo Logging Queue Pages
				p_queue_type->pedometer.data_type  = PEDOMETER_TYPE;
				p_queue_type->pedometer.page_start = PEDOMETER_PAGE_START;
				p_queue_type->pedometer.page_stop  = PEDOMETER_PAGE_STOP;
				que_initial(&p_queue_type->pedometer.q_page, PEDOMETER_PAGE_STOP-PEDOMETER_PAGE_START);
				que_initial(&p_queue_type->pedometer.q_data, (PAGE_SIZE/sizeof(a2_raw_acc_gyro_log_t)) + 1);

				//Intial Max30102 Logging Queue Pages
				p_queue_type->hr_spo2_temperature.data_type  = HR_SPO2_TEMPERATURE_TYPE;
				p_queue_type->hr_spo2_temperature.page_start = MAX30102_PAGE_START;
				p_queue_type->hr_spo2_temperature.page_stop  = MAX30102_PAGE_STOP;
				que_initial(&p_queue_type->hr_spo2_temperature.q_page, MAX30102_PAGE_STOP-MAX30102_PAGE_START);
				que_initial(&p_queue_type->hr_spo2_temperature.q_data, (PAGE_SIZE/sizeof(a2_raw_max30102_log_t)) + 1);

				p_queue_type->fall_alert_raw_data.data_type  = FALL_ALERT_RAW_DATA_TYPE;
				p_queue_type->fall_alert_raw_data.page_start = RAW_ACC_GYRO_FALL_PAGE_START;
				p_queue_type->fall_alert_raw_data.page_stop  = RAW_ACC_GYRO_FALL_PAGE_STOP;

				uint16_t num_pages_for_write = sizeof(a2_raw_acc_gyro_fall_log_t)/PAGE_SIZE; 
				if((sizeof(a2_raw_acc_gyro_fall_log_t)/PAGE_SIZE)%PAGE_SIZE != 0)
				{
						num_pages_for_write = num_pages_for_write+1;
				}

				p_queue_type->fall_alert_raw_data.q_page.q_n = RAW_ACC_GYRO_FALL_PAGE_STOP-RAW_ACC_GYRO_FALL_PAGE_START+1;
				p_queue_type->fall_alert_raw_data.q_page.q_f = 0;
				p_queue_type->fall_alert_raw_data.q_page.q_r = 0;

				p_queue_type->fall_alert_raw_data.q_data.q_n = (p_queue_type->fall_alert_raw_data.q_page.q_n/num_pages_for_write);
				p_queue_type->fall_alert_raw_data.q_data.q_f = 0;
				p_queue_type->fall_alert_raw_data.q_data.q_r = 0;

				uint32_t size_of_queue_setting = sizeof(*p_queue_type);
				p_queue_in_byte = (uint8_t*)p_queue_type;
				crc = 0;
				for(k=0;k<size_of_queue_setting-2;k++)
				{
						crc ^= *p_queue_in_byte;
						p_queue_in_byte++;
				}
				p_queue_type->crc16 = crc;
				
				a2_write_queue_config(dev, p_queue_type);
				NRF_LOG_INFO("[OK] Using default configulation", crc);
				
				
		}else{
				NRF_LOG_INFO("[OK]\tUsing queue setting from flash memory");
				memcpy( p_queue_type,&buf_queue_type, sizeof(*p_queue_type));
		}
	
		return 1;
}


uint8_t a2_read_config(logging_dev_t *dev, a2_config_t *config)
{
		dev->read(CONFIG_PAGE, (uint8_t*)config, sizeof(*config));
		return LOG_SUCCESS;
}

uint8_t a2_write_config(logging_dev_t *dev,const a2_config_t *config)
{
		dev->write(CONFIG_PAGE, (uint8_t*)config, sizeof(*config));
		return LOG_SUCCESS;
}

uint8_t a2_check_config(logging_dev_t *dev, a2_config_t *config)
{
		a2_config_t buf_config;
	
		a2_read_config(dev, &buf_config);
	
		if(buf_config.config_version != CONFIG_VERSION)
		{
				NRF_LOG_INFO("[FALL]\tThe version of configuration is not match.");
				memset( config ,0x00, sizeof(*config));
				config->config_version = CONFIG_VERSION;
				
				config->a2_sensor_config.gyro_en = 0;
				config->a2_sensor_config.acc_en = 1;
				config->a2_sensor_config.step_count_en = 1;
				config->a2_sensor_config.heart_rate_en = 1;
				config->a2_sensor_config.temperature_en = 0;
				config->a2_sensor_config.battery_en = 1;
				config->a2_sensor_config.temperature_interal = 10;            // 1S
			  config->a2_sensor_config.heart_rate_interval = 7200;          // 3600S or 1 Hour
				config->a2_sensor_config.battery_monitor_interval = 10;       // 60S
				config->a2_sensor_config.acc_gyro_step_count_interval = 100;  // 100mS
				config->a2_sensor_config.pedometer_interval = 10;							// 10S
				//m_a2_sensor_config.no_motion_timer_threshold = 18000;   
				config->a2_sensor_config.no_motion_acc_threshold = 500;        
				config->a2_sensor_config.no_motion_timer_threshold = 300000;   
			
				config->a2_user_data_config.gender = MALE;
				config->a2_user_data_config.height  = DEFUALT_HIGHT;
				config->a2_user_data_config.weight = DEFUALT_WIEGHT;
				config->a2_user_data_config.wearing_position = LEFT_HAND;
				memcpy( &config->a2_user_data_config.name[0], &DEFUALT_NAME[0], DEFUALT_LENGTH);
				memcpy( &config->a2_user_data_config.surname[0], &DEFUALT_SURNAME[0], DEFUALT_LENGTH);
				
				config->a2_working_info.charging_hour = 0;
				config->a2_working_info.ultra_deep_sleep_counter = 0;
				config->a2_working_info.working_hour = 0;
				
				a2_write_config(dev, config);
				
				memset(&buf_config, 0x00, sizeof(buf_config) );
				a2_read_config(dev, &buf_config);
				
				
				NRF_LOG_INFO("[OK]\tWrite Defualt config.");
		}
		else{
				memcpy(config, &buf_config, sizeof(buf_config));
		  	NRF_LOG_INFO("[OK]\tRead and using configuration in flash memory");
		}
		
		return LOG_SUCCESS;
}

/**************************************
*
*
***************************************/
uint8_t a2_button_alert_buf_is_full(queue_field_t *p_queue)
{
		uint8_t res;
		res = que_isFull(&p_queue->q_data);  // return is QUE_FALSE, QUE_TURE
		
		return res;
}


uint8_t a2_save_button_alert_to_buf(logging_dev_t *dev, queue_field_t *p_queue, const a2_button_alert_log_t *p_data_log)
{
		uint32_t res;
		//Check Data type of logger
		if(p_queue->data_type != BUTTON_ALERT_TYPE)
		{
				return LOG_UNSUCCESS;
		}
		//check fully of pages and remove old data if memmory is full.
		if(que_isFull(&p_queue->q_page) == QUE_TURE )
		{
				/*Pages is full. So we must remove the old data from flash memory.*/
				//Get old page index
				uint16_t old_page_index;
				res =  que_front(&p_queue->q_page, &old_page_index);
				if(res != QUE_TURE)
				{
						return LOG_UNSUCCESS;
				}				
				//Erease page at index old.
				flash_erase_page(old_page_index);
				//Pop data from queue.
				que_pop(&p_queue->q_page);
		}
		
		//Insert data to queue buffer
		res = que_append( &p_queue->q_data, (void*)&dev->buf_button_alert_log[0], (void*)p_data_log, sizeof(a2_button_alert_log_t) );
		if(res != QUE_TURE)
		{
				return LOG_UNSUCCESS;
		}
		
		return LOG_SUCCESS;
}
uint8_t a2_write_button_alert_log(logging_dev_t *dev, queue_field_t *p_queue)
{
		uint32_t res;
		//Check Data type of logger
		if(p_queue->data_type != BUTTON_ALERT_TYPE)
		{
				return LOG_UNSUCCESS;
		}
		
		if(que_isFull(&p_queue->q_data) != QUE_FALSE)
		{
				//Write data to flash memory
				uint16_t b_que_r;
				res = que_r(&p_queue->q_page, &b_que_r);
				if(res != QUE_TURE) 
				{
						b_que_r = 0;
				}
				uint16_t pages_for_write =  p_queue->page_start + b_que_r;			
				flash_write(pages_for_write, &dev->buf_button_alert_log[0], PAGE_SIZE );
				//append pages without data.
			  res = que_append(&p_queue->q_page, 0, 0, 0);	
				if(res != QUE_TURE) 
				{
						return LOG_UNSUCCESS;
				}
				//Set queue data to zero.
				que_reset(&p_queue->q_data);
				memset(&dev->buf_button_alert_log[0], 0x00, PAGE_SIZE);
				NRF_LOG_FLUSH();
		}
		
		return LOG_SUCCESS;
}


uint8_t a2_read_button_alert_log_by_queue(logging_dev_t *dev, queue_field_t *p_queue, a2_button_alert_log_t p_data_log[])
{
		uint32_t res;
		uint16_t que_f;
		//Check Data type of logger
		if(p_queue->data_type != BUTTON_ALERT_TYPE)
		{
				return LOG_UNSUCCESS;
		}
		res = que_front(&p_queue->q_page, &que_f);
		if(res == QUE_FALSE)
		{
				return LOG_UNSUCCESS;
		}
		uint16_t size_for_read = (PAGE_SIZE/sizeof(a2_button_alert_log_t))*sizeof(a2_button_alert_log_t);
		uint16_t pages_for_read = p_queue->page_start + que_f;
		flash_read(pages_for_read, (uint8_t*)p_data_log, size_for_read);			
			
		return LOG_SUCCESS;
}	


uint8_t a2_pop_button_alert_log(logging_dev_t *dev, queue_field_t *p_queue)
{
		if(p_queue->data_type != BUTTON_ALERT_TYPE)
		{
				return LOG_UNSUCCESS;
		}
		
		//Pop data from queue.
		que_pop(&p_queue->q_page);
		
		return LOG_SUCCESS;
}


uint8_t a2_read_button_alert_log_by_page(logging_dev_t *dev, uint16_t  page_num, a2_button_alert_log_t p_data_log[])
{
		uint16_t size_for_read = (PAGE_SIZE/sizeof(a2_button_alert_log_t))*sizeof(a2_button_alert_log_t);
		flash_read(page_num, (uint8_t*)p_data_log, size_for_read);			

		return LOG_SUCCESS;
}	

uint8_t a2_read_button_alert_last_buffer(logging_dev_t *dev, queue_field_t *p_queue)
{
		uint16_t buf_que_r;
	
		if(p_queue->data_type != BUTTON_ALERT_TYPE)
		{
				return LOG_UNSUCCESS;
		}
		
		que_r(&p_queue->q_page, &buf_que_r);
		uint16_t pages_for_read = p_queue->page_start + buf_que_r;
		flash_read(pages_for_read, &dev->buf_button_alert_log[0], PAGE_SIZE);	
			
		return LOG_SUCCESS;
}

uint8_t a2_write_button_alert_last_buffer(logging_dev_t *dev, queue_field_t *p_queue)
{
		uint16_t buf_que_r;
	
		if(p_queue->data_type != BUTTON_ALERT_TYPE)
		{
				return LOG_UNSUCCESS;
		}
		
		que_r(&p_queue->q_page, &buf_que_r);
		
		uint16_t pages_for_write = p_queue->page_start + buf_que_r;
		flash_write(pages_for_write, &dev->buf_button_alert_log[0], PAGE_SIZE);	
			
		return LOG_SUCCESS;
}




/**************************************
*
*
***************************************/
uint8_t a2_write_free_fall_alert_log(logging_dev_t *dev, queue_field_t *p_queue, const a2_fall_alert_log_t *p_data_log)
{
		uint32_t res;
		//Check Data type of logger
		if(p_queue->data_type != FALL_ALERT_LOG_TYPE)
		{
				return LOG_UNSUCCESS;
		}
		//check fully of pages and remove old data if memmory is full.
		if(que_isFull(&p_queue->q_page) == QUE_TURE )
		{
				/*Pages is full. So we must remove the old data from flash memory.*/
				//Get old page index
				uint16_t old_page_index;
				res =  que_front(&p_queue->q_page, &old_page_index);
				if(res != QUE_TURE)
				{
						return LOG_UNSUCCESS;
				}				
				//Erease page at index old.
				flash_erase_page(old_page_index);
				//Pop data from queue.
				que_pop(&p_queue->q_page);
		}
		
		//Insert data to queue buffer
		res = que_append( &p_queue->q_data, (void*)&dev->buf_free_fall_alert_log[0], (void*)p_data_log, sizeof(a2_fall_alert_log_t) );
		if(res != QUE_TURE)
		{
				return LOG_UNSUCCESS;
		}
		
		if(que_isFull(&p_queue->q_data) != QUE_FALSE)
		{
				//Write data to flash memory
				uint16_t b_que_r;
				res = que_r(&p_queue->q_page, &b_que_r);
				if(res != QUE_TURE) 
				{
						b_que_r = 0;
				}
				uint16_t pages_for_write =  p_queue->page_start + b_que_r;			
				flash_write(pages_for_write, &dev->buf_free_fall_alert_log[0], PAGE_SIZE );
				//append pages without data.
			  res = que_append(&p_queue->q_page, 0, 0, 0);	
				if(res != QUE_TURE) 
				{
						return LOG_UNSUCCESS;
				}
				//Set queue data to zero.
				que_reset(&p_queue->q_data);
				memset(&dev->buf_free_fall_alert_log[0], 0x00, PAGE_SIZE);
				NRF_LOG_FLUSH();
		}
		return LOG_SUCCESS;
}

uint8_t a2_read_free_fall_alert_log_by_queue(logging_dev_t *dev, queue_field_t *p_queue, a2_fall_alert_log_t p_data_log[])
{
		uint32_t res;
		uint16_t que_f;
		//Check Data type of logger
		if(p_queue->data_type != FALL_ALERT_LOG_TYPE)
		{
				return LOG_UNSUCCESS;
		}
		res = que_front(&p_queue->q_page, &que_f);
		if(res == QUE_FALSE)
		{
				return LOG_UNSUCCESS;
		}
		uint16_t size_for_read = (PAGE_SIZE/sizeof(a2_fall_alert_log_t))*sizeof(a2_fall_alert_log_t);
		uint16_t pages_for_read = p_queue->page_start + que_f;
		flash_read(pages_for_read, (uint8_t*)p_data_log, size_for_read);			
			
		return LOG_SUCCESS;
}

uint8_t a2_pop_free_fall_alert_log(logging_dev_t *dev, queue_field_t *p_queue)
{
		uint32_t res;
		uint16_t que_f;
	
		if(p_queue->data_type != FALL_ALERT_LOG_TYPE)
		{
				return LOG_UNSUCCESS;
		}
		res = que_front(&p_queue->q_page, &que_f);
		if(res == QUE_FALSE)
		{
				return LOG_UNSUCCESS;
		}
		uint16_t pages_for_read = p_queue->page_start + que_f;

		//Erease page at index old.
		flash_erase_page(pages_for_read);
		//Pop data from queue.
		que_pop(&p_queue->q_page);
		
		return LOG_SUCCESS;
}

uint8_t a2_read_free_fall_alert_log_by_page(logging_dev_t *dev, uint16_t  page_num, a2_fall_alert_log_t p_data_log[])
{
		uint16_t size_for_read = (PAGE_SIZE/sizeof(a2_fall_alert_log_t))*sizeof(a2_fall_alert_log_t);
		flash_read(page_num, (uint8_t*)p_data_log, size_for_read);		
	
		return LOG_SUCCESS;
}


/**************************************
*
*
***************************************/
uint8_t a2_save_pedo_meter_to_buf(logging_dev_t *dev, queue_field_t *p_queue, const a2_raw_acc_gyro_log_t *p_data_log)
{
		uint32_t res;
		//Check Data type of logger
		if(p_queue->data_type != PEDOMETER_TYPE)
		{
				return LOG_UNSUCCESS;
		}
		//check fully of pages and remove old data if memmory is full.
		if(que_isFull(&p_queue->q_page) == QUE_TURE )
		{
				/*Pages is full. So we must remove the old data from flash memory.*/
				//Get old page index
				uint16_t old_page_index;
				res =  que_front(&p_queue->q_page, &old_page_index);
				if(res != QUE_TURE)
				{
						return LOG_UNSUCCESS;
				}				
				//Erease page at index old.
				flash_erase_page(old_page_index);
				//Pop data from queue.
				que_pop(&p_queue->q_page);
		}
		
		//Insert data to queue buffer
		res = que_append( &p_queue->q_data, (void*)&dev->buf_pedo_log[0], (void*)p_data_log, sizeof(a2_raw_acc_gyro_log_t) );
		if(res != QUE_TURE)
		{
				return LOG_UNSUCCESS;
		}
		
		
		return LOG_SUCCESS;
}


uint8_t a2_pedo_buf_is_full(queue_field_t *p_queue)
{
		uint8_t res;
		res = que_isFull(&p_queue->q_data);  // return is QUE_FALSE, QUE_TURE
		
		return res;
}


uint8_t a2_write_pedo_meter_log(logging_dev_t *dev, queue_field_t *p_queue)
{	
		uint32_t res;
		//Check Data type of logger
		if(p_queue->data_type != PEDOMETER_TYPE)
		{
				return LOG_UNSUCCESS;
		}
		//Write data to flash memory
		uint16_t b_que_r;
		res = que_r(&p_queue->q_page, &b_que_r);
		if(res != QUE_TURE) 
		{
				b_que_r = 0;
		}
		uint16_t pages_for_write =  p_queue->page_start + b_que_r;		

		NRF_LOG_INFO("Page for write %d", pages_for_write);
		flash_write(pages_for_write, &dev->buf_pedo_log[0], PAGE_SIZE );
		//append pages without data.
		res = que_append(&p_queue->q_page, 0, 0, 0);	
		if(res != QUE_TURE) 
		{
				NRF_LOG_INFO("Append Fall");
				return LOG_UNSUCCESS;
		}else{
				NRF_LOG_INFO("Append Success");
		}
		//Set queue data to zero.
		que_reset(&p_queue->q_data);
		memset(&dev->buf_pedo_log[0], 0x00, PAGE_SIZE);
		NRF_LOG_INFO("Save to flash");
		NRF_LOG_FLUSH();
		
		return LOG_SUCCESS;
}



uint8_t a2_read_pedo_meter_log_by_queue(logging_dev_t *dev, queue_field_t *p_queue, a2_raw_acc_gyro_log_t p_data_log[])
{
		uint32_t res;
		uint16_t que_f;
		//Check Data type of logger
		if(p_queue->data_type != PEDOMETER_TYPE)
		{
				return LOG_UNSUCCESS;
		}
		res = que_front(&p_queue->q_page, &que_f);
		if(res == QUE_FALSE)
		{
				return LOG_UNSUCCESS;
		}
		uint16_t size_for_read = (PAGE_SIZE/sizeof(a2_raw_acc_gyro_log_t))*sizeof(a2_raw_acc_gyro_log_t);
		uint16_t pages_for_read = p_queue->page_start + que_f;
		
		NRF_LOG_INFO("Page for read %d", pages_for_read);
		flash_read(pages_for_read, (uint8_t*)p_data_log, size_for_read);			
			
		return LOG_SUCCESS;	
}


uint8_t a2_pop_pedo_meter_log(logging_dev_t *dev, queue_field_t *p_queue)
{
		if(p_queue->data_type != PEDOMETER_TYPE)
		{
				return LOG_UNSUCCESS;
		}

		que_pop(&p_queue->q_page);
		
		return LOG_SUCCESS;
}


uint8_t a2_read_pedo_meter_log_by_page(logging_dev_t *dev, uint16_t  page_num, a2_raw_acc_gyro_log_t p_data_log[])
{
		uint16_t size_for_read = (PAGE_SIZE/sizeof(a2_raw_acc_gyro_log_t))*sizeof(a2_raw_acc_gyro_log_t);
		flash_read(page_num, (uint8_t*)p_data_log, size_for_read);		
	
		return LOG_SUCCESS;
}

/**************************************
*
*
***************************************/
uint8_t a2_hr_spo2_buf_is_full(queue_field_t *p_queue)
{
		uint8_t res;
		res = que_isFull(&p_queue->q_data);  // return is QUE_FALSE, QUE_TURE
		
		return res;
}

uint8_t a2_save_hr_spo2_to_buf(logging_dev_t *dev, queue_field_t *p_queue, const a2_raw_max30102_log_t *p_data_log)
{
		uint32_t res;
		//Check Data type of logger
		if(p_queue->data_type != HR_SPO2_TEMPERATURE_TYPE)
		{
				return LOG_UNSUCCESS;
		}
		//check fully of pages and remove old data if memmory is full.
		if(que_isFull(&p_queue->q_page) == QUE_TURE )
		{
				/*Pages is full. So we must remove the old data from flash memory.*/
				//Get old page index
				uint16_t old_page_index;
				res =  que_front(&p_queue->q_page, &old_page_index);
				if(res != QUE_TURE)
				{
						return LOG_UNSUCCESS;
				}				
				//Erease page at index old.
				flash_erase_page(old_page_index);
				//Pop data from queue.
				que_pop(&p_queue->q_page);
		}
		
		//Insert data to queue buffer
		res = que_append( &p_queue->q_data, (void*)&dev->buf_hr_spo2_temp_log[0], (void*)p_data_log, sizeof(a2_raw_max30102_log_t) );
		if(res != QUE_TURE)
		{
				return LOG_UNSUCCESS;
		}
		
		return LOG_SUCCESS;
}

uint8_t a2_write_hr_spo2_temp_log(logging_dev_t *dev, queue_field_t *p_queue)
{
		uint32_t res;
		//Check Data type of logger
		if(p_queue->data_type != HR_SPO2_TEMPERATURE_TYPE)
		{
				return LOG_UNSUCCESS;
		}
		
		//Write data to flash memory
		uint16_t b_que_r;
		res = que_r(&p_queue->q_page, &b_que_r);
		if(res != QUE_TURE) 
		{
				b_que_r = 0;
		}
		uint16_t pages_for_write =  p_queue->page_start + b_que_r;			
		flash_write(pages_for_write, &dev->buf_hr_spo2_temp_log[0], PAGE_SIZE );
		//append pages without data.
		res = que_append(&p_queue->q_page, 0, 0, 0);	
		if(res != QUE_TURE) 
		{
				return LOG_UNSUCCESS;
		}
		//Set queue data to zero.
		que_reset(&p_queue->q_data);
		memset(&dev->buf_hr_spo2_temp_log[0], 0x00, PAGE_SIZE);
		
		return LOG_SUCCESS;
}


uint8_t a2_read_hr_spo2_temp_log_by_queue(logging_dev_t *dev, queue_field_t *p_queue, a2_raw_max30102_log_t p_data_log[])
{
		uint32_t res;
		uint16_t que_f;
		//Check Data type of logger
		if(p_queue->data_type != HR_SPO2_TEMPERATURE_TYPE)
		{
				return LOG_UNSUCCESS;
		}
		res = que_front(&p_queue->q_page, &que_f);
		if(res == QUE_FALSE)
		{
				return LOG_UNSUCCESS;
		}
		uint16_t size_for_read = (PAGE_SIZE/sizeof(a2_raw_max30102_log_t))*sizeof(a2_raw_max30102_log_t);
		uint16_t pages_for_read = p_queue->page_start + que_f;
		flash_read(pages_for_read, (uint8_t*)p_data_log, size_for_read);			
			
		return LOG_SUCCESS;	
}


uint8_t a2_pop_hr_spo2_temp_log(logging_dev_t *dev, queue_field_t *p_queue)
{
		uint32_t res;
		uint16_t que_f;
	
		if(p_queue->data_type != HR_SPO2_TEMPERATURE_TYPE)
		{
				return LOG_UNSUCCESS;
		}
		res = que_front(&p_queue->q_page, &que_f);
		if(res == QUE_FALSE)
		{
				return LOG_UNSUCCESS;
		}
		uint16_t pages_for_read = p_queue->page_start + que_f;

		//Erease page at index old.
		flash_erase_page(pages_for_read);
		//Pop data from queue.
		que_pop(&p_queue->q_page);
		
		return LOG_SUCCESS;
}


uint8_t a2_read_hr_spo2_temp_log_by_page(logging_dev_t *dev, uint16_t  page_num, a2_raw_max30102_log_t p_data_log[])
{
		uint16_t size_for_read = (PAGE_SIZE/sizeof(a2_raw_max30102_log_t))*sizeof(a2_raw_max30102_log_t);
		flash_read(page_num, (uint8_t*)p_data_log, size_for_read);		
	
		return LOG_SUCCESS;
}

/**************************************
*
*
***************************************/
uint8_t a2_write_raw_data_free_fall_log(logging_dev_t *dev, queue_field_t *p_queue, 
                                            const a2_raw_acc_gyro_fall_log_t *p_data_log)
{
		uint32_t res;

		uint16_t num_pages_for_write = sizeof(a2_raw_acc_gyro_fall_log_t)/PAGE_SIZE; 
	
		if(p_queue->data_type != FALL_ALERT_RAW_DATA_TYPE)
		{
				return LOG_UNSUCCESS;
		}
	
		if((sizeof(a2_raw_acc_gyro_fall_log_t)/PAGE_SIZE)%PAGE_SIZE != 0)
		{
				num_pages_for_write = num_pages_for_write+1;
		}
		
		//Check q_data is not full
		if( que_isFull(&p_queue->q_data) == QUE_TURE)
		{
				//POP Old Data
				que_pop(&p_queue->q_data);
		}
		//Write data to pages 
		uint16_t que_r_for_write;
		res = que_r(&p_queue->q_page,&que_r_for_write);
		if(res != QUE_TURE)
		{
				que_r_for_write = 0;
		}
		
		uint16_t start_page_addr_for_write = p_queue->page_start + que_r_for_write;
		uint8_t *p_buf_for_write = (uint8_t*)p_data_log;
		for(int page_index=0; page_index<num_pages_for_write; page_index++)
		{
				//Check page_index is the last index?
				if(page_index != (num_pages_for_write-1))
				{	
						//Write data 256 byte to page
						flash_write(start_page_addr_for_write+page_index, p_buf_for_write, PAGE_SIZE );
				}else{
						//Write the last pages
						uint16_t cal_num_for_write = sizeof(a2_raw_acc_gyro_fall_log_t)%PAGE_SIZE;
						if(cal_num_for_write == 0)
						{
								//Write data 256 byte to page
								flash_write(start_page_addr_for_write+page_index, p_buf_for_write, PAGE_SIZE );
						}else{
								//Write data to page with mod PAGE_SIZE
								flash_write(start_page_addr_for_write+page_index, p_buf_for_write, cal_num_for_write );
						}
				}
				p_buf_for_write = p_buf_for_write + PAGE_SIZE;
				if(que_isFull(&p_queue->q_page) == QUE_TURE)
				{
						que_pop(&p_queue->q_page);
				}
				que_append(&p_queue->q_page, 0,0,0);
		}
		que_append(&p_queue->q_data, 0,0,0);
		return LOG_SUCCESS;
}


uint8_t a2_read_raw_data_free_fall_log_by_queue(logging_dev_t *dev, queue_field_t *p_queue, 
                                            a2_raw_acc_gyro_fall_log_t *p_data_log)
{
		uint16_t que_f_for_read;
		uint16_t num_pages_for_read = sizeof(a2_raw_acc_gyro_fall_log_t)/PAGE_SIZE; 
		
		if(p_queue->data_type != FALL_ALERT_RAW_DATA_TYPE)
		{
				return LOG_UNSUCCESS;
		}
		
		if((sizeof(a2_raw_acc_gyro_fall_log_t)/PAGE_SIZE)%PAGE_SIZE != 0)
		{
				num_pages_for_read = num_pages_for_read+1;
		}
		
		que_front(&p_queue->q_data, &que_f_for_read);
		uint16_t start_page_addr_for_read = p_queue->page_start + (que_f_for_read*num_pages_for_read);
		uint8_t *p_buf_for_read = (uint8_t*)p_data_log;
	  for(int page_index=0; page_index<num_pages_for_read; page_index++)
		{
				//Check page_index is the last index?
				if(page_index != (num_pages_for_read-1))
				{	
						//Read data 256 byte to page
						flash_read(start_page_addr_for_read+page_index, p_buf_for_read, PAGE_SIZE );
				}else{
					
						//Read the last pages
						uint16_t cal_num_for_read = sizeof(a2_raw_acc_gyro_fall_log_t)%PAGE_SIZE;
						if(cal_num_for_read == 0)
						{
								//Write data 256 byte to page
								flash_read(start_page_addr_for_read+page_index, p_buf_for_read, PAGE_SIZE );
						}else{
								//Read data to page with mod PAGE_SIZE
								flash_read(start_page_addr_for_read+page_index, p_buf_for_read, cal_num_for_read );
						}
				}
				p_buf_for_read = p_buf_for_read + PAGE_SIZE;
		}
		
		return LOG_SUCCESS;
}
uint8_t a2_pop_raw_data_free_fall_log(logging_dev_t *dev, queue_field_t *p_queue)
{
		uint32_t res;
		uint16_t que_f;
	
		if(p_queue->data_type != FALL_ALERT_RAW_DATA_TYPE)
		{
				return LOG_UNSUCCESS;
		}
	
		res = que_front(&p_queue->q_data, &que_f);
		if(res == QUE_FALSE)
		{
				return LOG_UNSUCCESS;
		}
		//Pop data from queue.
		que_pop(&p_queue->q_data);
		
		return LOG_SUCCESS;
}

//m_a2_info
//m_a2_config

#define REG_CONFIG_OFFSET		4

void a2_reg_intial(a2_reg_t *p_reg)
{
		uint16_t ii;
		
		//Mapping resgister address and variable. 
		p_reg->reg[REG_ADDR_GENDER] = &m_a2_config.a2_user_data_config.gender;
		p_reg->reg[REG_ADDR_WEIGHT] = &m_a2_config.a2_user_data_config.weight;
		p_reg->reg[REG_ADDE_HEIGHT] = &m_a2_config.a2_user_data_config.height; 
		p_reg->reg[REG_ADDE_WEARING_POSITION] = &m_a2_config.a2_user_data_config.wearing_position; 
	
		for(ii=0;ii<MAX_BUFFER_FOR_NAME_SURNAME;ii++)
		{
				p_reg->reg[REG_ADDR_NAME+ii] =  &m_a2_config.a2_user_data_config.name[ii];
		}
		
		for(ii=0;ii<MAX_BUFFER_FOR_NAME_SURNAME;ii++)
		{
				p_reg->reg[REG_ADDR_SURNAME+ii] =  &m_a2_config.a2_user_data_config.surname[ii];
		}
		
		
		p_reg->reg[REG_ADDE_WORKING_HOUR]   = (uint8_t*)&m_a2_config.a2_working_info.working_hour;
		p_reg->reg[REG_ADDE_WORKING_HOUR+1] = (uint8_t*)&m_a2_config.a2_working_info.working_hour+1; 
		p_reg->reg[REG_ADDE_WORKING_HOUR+2] = (uint8_t*)&m_a2_config.a2_working_info.working_hour+2;
		p_reg->reg[REG_ADDE_WORKING_HOUR+3] = (uint8_t*)&m_a2_config.a2_working_info.working_hour+3; 
	
		p_reg->reg[REG_ADDE_CHARING_HOUR]   = (uint8_t*)&m_a2_config.a2_working_info.charging_hour;
		p_reg->reg[REG_ADDE_CHARING_HOUR+1] = (uint8_t*)&m_a2_config.a2_working_info.charging_hour+1;
		p_reg->reg[REG_ADDE_CHARING_HOUR+2] = (uint8_t*)&m_a2_config.a2_working_info.charging_hour+2;
		p_reg->reg[REG_ADDE_CHARING_HOUR+3] = (uint8_t*)&m_a2_config.a2_working_info.charging_hour+3;
		
		p_reg->reg[REG_ADDE_ULTRA_DEEP_SLEEP_COUNTER]   = (uint8_t*)&m_a2_config.a2_working_info.ultra_deep_sleep_counter;
		p_reg->reg[REG_ADDE_ULTRA_DEEP_SLEEP_COUNTER+1] = (uint8_t*)&m_a2_config.a2_working_info.ultra_deep_sleep_counter+1;
		
		p_reg->reg[REG_ADDE_HEART_RATE_INTERVAL]   = (uint8_t*)&m_a2_config.a2_sensor_config.heart_rate_interval;
		p_reg->reg[REG_ADDE_HEART_RATE_INTERVAL+1] = (uint8_t*)&m_a2_config.a2_sensor_config.heart_rate_interval+1;
		p_reg->reg[REG_ADDE_HEART_RATE_INTERVAL+2] = (uint8_t*)&m_a2_config.a2_sensor_config.heart_rate_interval+2;
		p_reg->reg[REG_ADDE_HEART_RATE_INTERVAL+3] = (uint8_t*)&m_a2_config.a2_sensor_config.heart_rate_interval+3;
		
		p_reg->reg[REG_ADDE_TEMPERATURE_INTERVAL]   = (uint8_t*)&m_a2_config.a2_sensor_config.temperature_interal;
		p_reg->reg[REG_ADDE_TEMPERATURE_INTERVAL+1] = (uint8_t*)&m_a2_config.a2_sensor_config.temperature_interal+1;
		p_reg->reg[REG_ADDE_TEMPERATURE_INTERVAL+2] = (uint8_t*)&m_a2_config.a2_sensor_config.temperature_interal+2;
		p_reg->reg[REG_ADDE_TEMPERATURE_INTERVAL+3] = (uint8_t*)&m_a2_config.a2_sensor_config.temperature_interal+3;
		
		
		p_reg->reg[REG_ADDE_ACC_GYRO_STEP_COUNT_INTERVAL]   = (uint8_t*)&m_a2_config.a2_sensor_config.acc_gyro_step_count_interval;
		p_reg->reg[REG_ADDE_ACC_GYRO_STEP_COUNT_INTERVAL+1] = (uint8_t*)&m_a2_config.a2_sensor_config.acc_gyro_step_count_interval+1;
		p_reg->reg[REG_ADDE_ACC_GYRO_STEP_COUNT_INTERVAL+2] = (uint8_t*)&m_a2_config.a2_sensor_config.acc_gyro_step_count_interval+2;
		p_reg->reg[REG_ADDE_ACC_GYRO_STEP_COUNT_INTERVAL+3] = (uint8_t*)&m_a2_config.a2_sensor_config.acc_gyro_step_count_interval+3;
		
		p_reg->reg[REG_ADDE_BATTERY_MONITORING_INTERVAL]   = (uint8_t*)&m_a2_config.a2_sensor_config.battery_monitor_interval;
		p_reg->reg[REG_ADDE_BATTERY_MONITORING_INTERVAL+1] = (uint8_t*)&m_a2_config.a2_sensor_config.battery_monitor_interval+1;
		p_reg->reg[REG_ADDE_BATTERY_MONITORING_INTERVAL+2] = (uint8_t*)&m_a2_config.a2_sensor_config.battery_monitor_interval+2;
		p_reg->reg[REG_ADDE_BATTERY_MONITORING_INTERVAL+3] = (uint8_t*)&m_a2_config.a2_sensor_config.battery_monitor_interval+3;
		
		p_reg->reg[REG_ADDE_HEART_RATE_EN] 	= (uint8_t*)&m_a2_config.a2_sensor_config.heart_rate_en;
		p_reg->reg[REG_ADDE_TEMPERATURE_EN] = (uint8_t*)&m_a2_config.a2_sensor_config.temperature_en;
		p_reg->reg[REG_ADDE_STEP_COUNT_EN] 	= (uint8_t*)&m_a2_config.a2_sensor_config.step_count_en;
		p_reg->reg[REG_ADDE_ACC_EN] 				= (uint8_t*)&m_a2_config.a2_sensor_config.acc_en;
		p_reg->reg[REG_ADDE_GYRO_EN] 				= (uint8_t*)&m_a2_config.a2_sensor_config.gyro_en;
		p_reg->reg[REG_ADDE_BATTERY_EN] 		= &m_a2_config.a2_sensor_config.battery_en;
		p_reg->reg[REG_ADDE_SEM_GET_HEART_RATE] 		= (uint8_t*)&m_a2_config.a2_sensor_config.sem_get_heart_rate;
		p_reg->reg[REG_ADDE_SEM_MANUAL_RECEHCK_HR]	= (uint8_t*)&m_a2_config.a2_sensor_config.sem_manual_recheck_hr;
		p_reg->reg[REG_ADDE_SEM_GET_TEMPERATURE] 		= (uint8_t*)&m_a2_config.a2_sensor_config.sem_get_temperature;
		p_reg->reg[REG_ADDE_SEM_GET_ACC_GYRO_STEP_COUNT]	= (uint8_t*)&m_a2_config.a2_sensor_config.sem_get_acc_gyro_step_count;
		p_reg->reg[REG_ADDE_SEM_GET_BATTERY_LEVEL]	 			= (uint8_t*)&m_a2_config.a2_sensor_config.sem_get_battery_level;
		p_reg->reg[REG_ADDE_A2_RUNNING_MODE] 							= (uint8_t*)&m_a2_config.a2_sensor_config.power_mode;
		p_reg->reg[REG_ADDE_NO_MOTION_ACC_THRESHOLD] 			= (uint8_t*)&m_a2_config.a2_sensor_config.no_motion_acc_threshold;
		p_reg->reg[REG_ADDE_NO_MOTION_TIMER_THRESHOLD] 		= (uint8_t*)&m_a2_config.a2_sensor_config.no_motion_timer_threshold;
		
		p_reg->reg[REG_ADDE_ALERT_STATUS]     = (uint8_t*)&m_a2_info.emergency_status;
		p_reg->reg[REG_ADDE_ACC_X]						= (uint8_t*)&m_a2_info.accel.x;
		p_reg->reg[REG_ADDE_ACC_X+1]					= (uint8_t*)&m_a2_info.accel.x+1;
		p_reg->reg[REG_ADDE_ACC_Y]						= (uint8_t*)&m_a2_info.accel.y;
		p_reg->reg[REG_ADDE_ACC_Y+1]					= (uint8_t*)&m_a2_info.accel.y+1;
		p_reg->reg[REG_ADDE_ACC_Z]						= (uint8_t*)&m_a2_info.accel.z;
		p_reg->reg[REG_ADDE_ACC_Z+1]					= (uint8_t*)&m_a2_info.accel.z+1;
		p_reg->reg[REG_ADDE_STEP_COUNT]				= (uint8_t*)&m_a2_info.step_count;
		p_reg->reg[REG_ADDE_STEP_COUNT+1]			= (uint8_t*)&m_a2_info.step_count+1;
		p_reg->reg[REG_ADDE_SPEED]						= (uint8_t*)&m_a2_info.speed;
		p_reg->reg[REG_ADDE_SPEED+1]					= (uint8_t*)&m_a2_info.speed+1;
		p_reg->reg[REG_ADDE_CALORIES]					= (uint8_t*)&m_a2_info.calories;
		p_reg->reg[REG_ADDE_CALORIES+1]				= (uint8_t*)&m_a2_info.calories+1;
		p_reg->reg[REG_ADDE_DISTANCE]					= (uint8_t*)&m_a2_info.distance;
		p_reg->reg[REG_ADDE_DISTANCE+1]				= (uint8_t*)&m_a2_info.distance+1;
		p_reg->reg[REG_ADDE_DISTANCE+2]				= (uint8_t*)&m_a2_info.distance+2;
		p_reg->reg[REG_ADDE_DISTANCE+3]				= (uint8_t*)&m_a2_info.distance+3;
		p_reg->reg[REG_ADDE_REST_TIME]				= (uint8_t*)&m_a2_info.rest_time;
		p_reg->reg[REG_ADDE_REST_TIME+1]			= (uint8_t*)&m_a2_info.rest_time+1;
		p_reg->reg[REG_ADDE_REST_TIME+2]			= (uint8_t*)&m_a2_info.rest_time+2;
		p_reg->reg[REG_ADDE_REST_TIME+3]			= (uint8_t*)&m_a2_info.rest_time+3;
		p_reg->reg[REG_ADDE_WALKING_TIME]			= (uint8_t*)&m_a2_info.walking_time;
		p_reg->reg[REG_ADDE_WALKING_TIME+1]		= (uint8_t*)&m_a2_info.walking_time+1;
		p_reg->reg[REG_ADDE_WALKING_TIME+2]		= (uint8_t*)&m_a2_info.walking_time+2;
		p_reg->reg[REG_ADDE_WALKING_TIME+3]		= (uint8_t*)&m_a2_info.walking_time+3;
		p_reg->reg[REG_ADDE_JOGGING_TIME]			= (uint8_t*)&m_a2_info.jogging_time;
		p_reg->reg[REG_ADDE_JOGGING_TIME+1]		= (uint8_t*)&m_a2_info.jogging_time+1;
		p_reg->reg[REG_ADDE_JOGGING_TIME+2]		= (uint8_t*)&m_a2_info.jogging_time+2;
		p_reg->reg[REG_ADDE_JOGGING_TIME+3]		= (uint8_t*)&m_a2_info.jogging_time+3;
		p_reg->reg[REG_ADDE_RUNNING_TIME]     = (uint8_t*)&m_a2_info.running_time;
		p_reg->reg[REG_ADDE_RUNNING_TIME+1]   = (uint8_t*)&m_a2_info.running_time+1;
		p_reg->reg[REG_ADDE_RUNNING_TIME+2]   = (uint8_t*)&m_a2_info.running_time+2;
		p_reg->reg[REG_ADDE_RUNNING_TIME+3]   = (uint8_t*)&m_a2_info.running_time+3;
		p_reg->reg[REG_ADDE_CURRENT_ACTIVITY] = (uint8_t*)&m_a2_info.current_activity;
		p_reg->reg[REG_ADDE_HEART_RATE]				= (uint8_t*)&m_a2_info.heart_rate;
		p_reg->reg[REG_ADDE_SPO2]							= (uint8_t*)&m_a2_info.pulse_oximeter;
		p_reg->reg[REG_ADDE_TEMPERATURE]		  = (uint8_t*)&m_a2_info.temperature;
		p_reg->reg[REG_ADDE_TEMPERATURE+1]		= (uint8_t*)&m_a2_info.temperature+1;
		
		
		//Config read write control of resgister.
		p_reg->r_w_ctl[REG_ADDR_GENDER] = REG_READ_WRITE;
		p_reg->r_w_ctl[REG_ADDR_WEIGHT] = REG_READ_WRITE;
		p_reg->r_w_ctl[REG_ADDE_HEIGHT] = REG_READ_WRITE;
		p_reg->r_w_ctl[REG_ADDE_WEARING_POSITION] = REG_READ_WRITE;
		for(ii=0;ii<MAX_BUFFER_FOR_NAME_SURNAME;ii++)
		{
				p_reg->r_w_ctl[REG_ADDR_NAME+ii] =  REG_READ_WRITE;
		}
		for(ii=0;ii<MAX_BUFFER_FOR_NAME_SURNAME;ii++)
		{
				p_reg->r_w_ctl[REG_ADDR_SURNAME+ii] =  REG_READ_WRITE;
		}
		p_reg->r_w_ctl[REG_ADDE_WORKING_HOUR]   = REG_READ_ONLY;
		p_reg->r_w_ctl[REG_ADDE_WORKING_HOUR+1] = REG_READ_ONLY;
		p_reg->r_w_ctl[REG_ADDE_WORKING_HOUR+2] = REG_READ_ONLY;
		p_reg->r_w_ctl[REG_ADDE_WORKING_HOUR+3] = REG_READ_ONLY;
		p_reg->r_w_ctl[REG_ADDE_CHARING_HOUR]   = REG_READ_ONLY;
		p_reg->r_w_ctl[REG_ADDE_CHARING_HOUR+1] = REG_READ_ONLY;
		p_reg->r_w_ctl[REG_ADDE_CHARING_HOUR+2] = REG_READ_ONLY;
		p_reg->r_w_ctl[REG_ADDE_CHARING_HOUR+3] = REG_READ_ONLY;
		p_reg->r_w_ctl[REG_ADDE_ULTRA_DEEP_SLEEP_COUNTER]   = REG_READ_ONLY;
		p_reg->r_w_ctl[REG_ADDE_ULTRA_DEEP_SLEEP_COUNTER+1] = REG_READ_ONLY;
		p_reg->r_w_ctl[REG_ADDE_HEART_RATE_INTERVAL]   = REG_READ_WRITE;
		p_reg->r_w_ctl[REG_ADDE_HEART_RATE_INTERVAL+1] = REG_READ_WRITE;
		p_reg->r_w_ctl[REG_ADDE_HEART_RATE_INTERVAL+2] = REG_READ_WRITE;
		p_reg->r_w_ctl[REG_ADDE_HEART_RATE_INTERVAL+3] = REG_READ_WRITE;
		p_reg->r_w_ctl[REG_ADDE_TEMPERATURE_INTERVAL]   = REG_READ_WRITE;
		p_reg->r_w_ctl[REG_ADDE_TEMPERATURE_INTERVAL+1] = REG_READ_WRITE;
		p_reg->r_w_ctl[REG_ADDE_TEMPERATURE_INTERVAL+2] = REG_READ_WRITE;
		p_reg->r_w_ctl[REG_ADDE_TEMPERATURE_INTERVAL+3] = REG_READ_WRITE;
		p_reg->r_w_ctl[REG_ADDE_ACC_GYRO_STEP_COUNT_INTERVAL]   = REG_READ_WRITE;
		p_reg->r_w_ctl[REG_ADDE_ACC_GYRO_STEP_COUNT_INTERVAL+1] = REG_READ_WRITE;
		p_reg->r_w_ctl[REG_ADDE_ACC_GYRO_STEP_COUNT_INTERVAL+2] = REG_READ_WRITE;
		p_reg->r_w_ctl[REG_ADDE_ACC_GYRO_STEP_COUNT_INTERVAL+3] = REG_READ_WRITE;
		p_reg->r_w_ctl[REG_ADDE_BATTERY_MONITORING_INTERVAL]   = REG_READ_WRITE;
		p_reg->r_w_ctl[REG_ADDE_BATTERY_MONITORING_INTERVAL+1] = REG_READ_WRITE;
		p_reg->r_w_ctl[REG_ADDE_BATTERY_MONITORING_INTERVAL+2] = REG_READ_WRITE;
		p_reg->r_w_ctl[REG_ADDE_BATTERY_MONITORING_INTERVAL+3] = REG_READ_WRITE;
		p_reg->r_w_ctl[REG_ADDE_HEART_RATE_EN] 	= REG_READ_WRITE;
		p_reg->r_w_ctl[REG_ADDE_TEMPERATURE_EN] = REG_READ_WRITE;
		p_reg->r_w_ctl[REG_ADDE_STEP_COUNT_EN] 	= REG_READ_WRITE;
		p_reg->r_w_ctl[REG_ADDE_ACC_EN] 				= REG_READ_WRITE;
		p_reg->r_w_ctl[REG_ADDE_GYRO_EN] 				= REG_READ_WRITE;
		p_reg->r_w_ctl[REG_ADDE_BATTERY_EN] 		= REG_READ_WRITE;
		p_reg->r_w_ctl[REG_ADDE_SEM_GET_HEART_RATE] 		= REG_READ_ONLY;
		p_reg->r_w_ctl[REG_ADDE_SEM_MANUAL_RECEHCK_HR]	= REG_READ_ONLY;
		p_reg->r_w_ctl[REG_ADDE_SEM_GET_TEMPERATURE] 		= REG_READ_ONLY;
		p_reg->r_w_ctl[REG_ADDE_SEM_GET_ACC_GYRO_STEP_COUNT]	= REG_READ_ONLY;
		p_reg->r_w_ctl[REG_ADDE_SEM_GET_BATTERY_LEVEL]	 			= REG_READ_ONLY;
		p_reg->r_w_ctl[REG_ADDE_A2_RUNNING_MODE] 							= REG_READ_ONLY;
		p_reg->r_w_ctl[REG_ADDE_NO_MOTION_ACC_THRESHOLD] 			= REG_READ_WRITE;
		p_reg->r_w_ctl[REG_ADDE_NO_MOTION_TIMER_THRESHOLD] 		= REG_READ_WRITE;
		p_reg->r_w_ctl[REG_ADDE_ALERT_STATUS]     = REG_READ_ONLY;
		p_reg->r_w_ctl[REG_ADDE_ACC_X]						= REG_READ_ONLY;
		p_reg->r_w_ctl[REG_ADDE_ACC_X+1]					= REG_READ_ONLY;
		p_reg->r_w_ctl[REG_ADDE_ACC_Y]						= REG_READ_ONLY;
		p_reg->r_w_ctl[REG_ADDE_ACC_Y+1]					= REG_READ_ONLY;
		p_reg->r_w_ctl[REG_ADDE_ACC_Z]						= REG_READ_ONLY;
		p_reg->r_w_ctl[REG_ADDE_ACC_Z+1]					= REG_READ_ONLY;
		p_reg->r_w_ctl[REG_ADDE_STEP_COUNT]				= REG_READ_ONLY;
		p_reg->r_w_ctl[REG_ADDE_STEP_COUNT+1]			= REG_READ_ONLY;
		p_reg->r_w_ctl[REG_ADDE_SPEED]						= REG_READ_ONLY;
		p_reg->r_w_ctl[REG_ADDE_SPEED+1]					= REG_READ_ONLY;
		p_reg->r_w_ctl[REG_ADDE_CALORIES]					= REG_READ_ONLY;
		p_reg->r_w_ctl[REG_ADDE_CALORIES+1]				= REG_READ_ONLY;
		p_reg->r_w_ctl[REG_ADDE_DISTANCE]					= REG_READ_ONLY;
		p_reg->r_w_ctl[REG_ADDE_DISTANCE+1]				= REG_READ_ONLY;
		p_reg->r_w_ctl[REG_ADDE_DISTANCE+2]				= REG_READ_ONLY;
		p_reg->r_w_ctl[REG_ADDE_DISTANCE+3]				= REG_READ_ONLY;
		p_reg->r_w_ctl[REG_ADDE_REST_TIME]				= REG_READ_ONLY;
		p_reg->r_w_ctl[REG_ADDE_REST_TIME+1]			= REG_READ_ONLY;
		p_reg->r_w_ctl[REG_ADDE_REST_TIME+2]			= REG_READ_ONLY;
		p_reg->r_w_ctl[REG_ADDE_REST_TIME+3]			= REG_READ_ONLY;
		p_reg->r_w_ctl[REG_ADDE_WALKING_TIME]			= REG_READ_ONLY;
		p_reg->r_w_ctl[REG_ADDE_WALKING_TIME+1]		= REG_READ_ONLY;
		p_reg->r_w_ctl[REG_ADDE_WALKING_TIME+2]		= REG_READ_ONLY;
		p_reg->r_w_ctl[REG_ADDE_WALKING_TIME+3]		= REG_READ_ONLY;
		p_reg->r_w_ctl[REG_ADDE_JOGGING_TIME]			= REG_READ_ONLY;
		p_reg->r_w_ctl[REG_ADDE_JOGGING_TIME+1]		= REG_READ_ONLY;
		p_reg->r_w_ctl[REG_ADDE_JOGGING_TIME+2]		= REG_READ_ONLY;
		p_reg->r_w_ctl[REG_ADDE_JOGGING_TIME+3]		= REG_READ_ONLY;
		p_reg->r_w_ctl[REG_ADDE_RUNNING_TIME]     = REG_READ_ONLY;
		p_reg->r_w_ctl[REG_ADDE_RUNNING_TIME+1]   = REG_READ_ONLY;
		p_reg->r_w_ctl[REG_ADDE_RUNNING_TIME+2]   = REG_READ_ONLY;
		p_reg->r_w_ctl[REG_ADDE_RUNNING_TIME+3]   = REG_READ_ONLY;
		p_reg->r_w_ctl[REG_ADDE_CURRENT_ACTIVITY] = REG_READ_ONLY;
		p_reg->r_w_ctl[REG_ADDE_HEART_RATE]				= REG_READ_ONLY;
		p_reg->r_w_ctl[REG_ADDE_SPO2]							= REG_READ_ONLY;
		p_reg->r_w_ctl[REG_ADDE_TEMPERATURE]		  = REG_READ_ONLY;
		p_reg->r_w_ctl[REG_ADDE_TEMPERATURE+1]		= REG_READ_ONLY;
		//End function.
}

uint8_t a2_read_reg(a2_reg_t *p_reg, uint8_t reg_addr, uint16_t data_len, uint8_t p_data[])
{
		uint16_t k;
	
		if(reg_addr > LAST_ADDR_REG && data_len > LAST_ADDR_REG)
		{
				return LOG_UNSUCCESS;
		}
		
		for(k=0;k<data_len;k++)
		{
				if(reg_addr+k>LAST_ADDR_REG)
				{
						return LOG_UNSUCCESS;
				}
				if(p_reg->r_w_ctl[reg_addr+k] == REG_READ_ONLY || p_reg->r_w_ctl[reg_addr+k] == REG_READ_WRITE)
				{
						p_data[k] = *p_reg->reg[reg_addr+k];
				}
		}
		
		return LOG_SUCCESS;
}

uint8_t a2_write_reg(a2_reg_t *p_reg, uint8_t reg_addr, uint16_t data_len, uint8_t p_data[])
{
	  uint16_t k;
	
		if(reg_addr > LAST_ADDR_REG && data_len > LAST_ADDR_REG)
		{
				return LOG_UNSUCCESS;
		}
	
		for(k=0;k<data_len;k++)
		{
				if(reg_addr+k>LAST_ADDR_REG)
				{
						return LOG_UNSUCCESS;
				}
				if(p_reg->r_w_ctl[reg_addr+k] == REG_READ_ONLY || p_reg->r_w_ctl[reg_addr+k] == REG_READ_WRITE)
				{
						*p_reg->reg[reg_addr+k] = p_data[k];
				}
		}
		
		return LOG_SUCCESS;
}


uint8_t a2_write_queue_config(logging_dev_t *dev, queue_type_t *p_queue)
{
		uint32_t size_of_queue_setting = sizeof(*p_queue);
		uint8_t *p_queue_in_byte = (uint8_t*)p_queue;
		uint16_t crc = 0;
	
		for(uint32_t k=0; k < size_of_queue_setting - sizeof(p_queue->crc16); k++ )
		{
				crc ^= *p_queue_in_byte;
				p_queue_in_byte++;
		}
		p_queue->crc16 = crc;
	
		flash_write(QUERY_PAGE_ADDRESS, (uint8_t*)p_queue, sizeof(*p_queue) );

		return 0;
}

uint8_t a2_read_queue_config(logging_dev_t *dev, queue_type_t *p_queue)
{
	  flash_read(QUERY_PAGE_ADDRESS, (uint8_t*)p_queue, sizeof(*p_queue) );

		return 0;
}












