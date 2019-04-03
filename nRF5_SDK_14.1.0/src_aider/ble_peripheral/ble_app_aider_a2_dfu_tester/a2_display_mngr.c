#include "a2_display_mngr.h"

#define DISPLAY_TIME_OUT				5   // Seconday
#define ADC_BATT_BITPERLEVEL 		0.44324324324324


void display_task(display_page_t *page, a2_info_t *p_a2_info, nrf_drv_a2_i2c_0_comm_t *p_comm, oled_ssd1306_dev_t *oled_dev)
{
		static uint8_t display_state = DISPLAY_WAIT_STATE;
		static uint32_t old_unix_time;
		static uint8_t old_page = DISPLAY_WAIT_STATE;
		static uint32_t counter_display_off;
		
		if(*page != old_page )
		{
				display_state = *page; 		
				old_page = *page;
		}
			
		
		switch(display_state)
		{
			case DISPLAY_WAIT_STATE:
					//Don't implement.
					
					break;	
			case DISPLAY_COUNDOWN_STATE:
					if(p_a2_info->unix_time > (counter_display_off+old_unix_time) )
					{
							display_state = DISPLAY_OFF_STATE;
					}
					break;
			case DISPLAY_OFF_STATE:
					nrf_drv_a2_oled_shutdown();
					nrf_drv_a2_i2c_0_deinit(p_comm);
					display_state = DISPLAY_WAIT_STATE;
					old_page 			= DISPLAY_WAIT_STATE;
					*page = DISPLAY_WAIT_STATE;
					break;
			case TIME_PAGE:
					nrf_drv_a2_i2c_0_init(p_comm);
					nrf_drv_a2_oled_init(p_comm, oled_dev);		
					display_date_time(p_a2_info);
					counter_display_off = DISPLAY_TIME_OUT;
					old_unix_time = p_a2_info->unix_time;
					display_state = DISPLAY_COUNDOWN_STATE;
					break;
			
		}
}

void display_date_time(a2_info_t *p_a2_info)
{		
		char buffer_display[100];
	
		ssd1306_clear_display();
		ssd1306_set_textsize(4);
		ssd1306_set_textcolor(WHITE);
		ssd1306_set_cursor(6,15);
		sprintf(buffer_display,"%02d:",p_a2_info->time_h);
		ssd1306_putstring(buffer_display);
		sprintf(buffer_display,"%02d",p_a2_info->time_m);
		ssd1306_putstring(buffer_display);
		ssd1306_set_cursor(0,55);
		ssd1306_set_textsize(1);
	
		uint16_t batt_voltage = (uint16_t)(ADC_BATT_BITPERLEVEL*p_a2_info->batter_value);
	
	  sprintf(buffer_display,"B:%d.%02d,S:%d",batt_voltage/100,batt_voltage%100, p_a2_info->step_count);
		ssd1306_putstring(buffer_display);
		ssd1306_display();			

}


