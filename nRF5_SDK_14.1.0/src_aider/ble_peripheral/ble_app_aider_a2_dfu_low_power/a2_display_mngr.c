#include "a2_display_mngr.h"
#include "a2_icon.h"
#include "nrf_delay.h"

static const uint8_t daysInMonth[] = { 31,28,31,30,31,30,31,31,30,31,30,31 };


void display_battery_full(void)
{
		ssd1306_clear_display();
		ssd1306_set_textsize(2);
		ssd1306_set_textcolor(WHITE);
		ssd1306_set_cursor(20,15);
		ssd1306_putstring(" BATTERY");
	  ssd1306_set_cursor(20,35);
		ssd1306_putstring("  FULL  ");
		ssd1306_display();
}


void draw_battery(void)
{
		static uint8_t battery_level_state = BATTERY_LEVEL_0;
		char buffer_display[10];
		
		if(m_a2_info.usb_pin_status == 1)
		{
				battery_level_state++;
				if(battery_level_state > BATTERY_FULL)
				{
						battery_level_state = BATTERY_LEVEL_0;
				}
				if(m_display_page == DISPLAY_TIME_PAGE)
				{
						sprintf(buffer_display,"%d%%  ", m_a2_info.batter_value);
						ssd1306_drawRightString(buffer_display,108,-3,2);
				}
				
		}else{
				if(m_a2_info.batter_value > 95)
				{
						battery_level_state = BATTERY_FULL;
				}else if(m_a2_info.batter_value > 75){
						battery_level_state = BATTERY_LEVEL_4;
				}else if(m_a2_info.batter_value > 55){
						battery_level_state = BATTERY_LEVEL_3;
				}else if(m_a2_info.batter_value > 35){
						battery_level_state = BATTERY_LEVEL_2;
				}else if(m_a2_info.batter_value > 15){
						battery_level_state = BATTERY_LEVEL_1;
				}else{
						battery_level_state = BATTERY_LEVEL_0;
				}
		}		
			
		switch(battery_level_state)
		{
			case BATTERY_LEVEL_0:
				//ssd1306_fill_rect(114,2,15,6,WHITE);	
				break;
			case BATTERY_LEVEL_1:
				ssd1306_fill_rect(124,2,15,6,WHITE); // Level 1
				break;
			case BATTERY_LEVEL_2:
				ssd1306_fill_rect(121,2,15,6,WHITE); // Level 2
				break;
			case BATTERY_LEVEL_3:
				ssd1306_fill_rect(118,2,15,6,WHITE); // Level 3
				break;
			case BATTERY_LEVEL_4:
				ssd1306_fill_rect(115,2,15,6,WHITE); // Level 4
				break;
			case BATTERY_FULL:
				ssd1306_fill_rect(112,2,15,6,WHITE); //FULL		
				break;
		}
		
		ssd1306_draw_rect(110,0,18,10,WHITE);
	  ssd1306_draw_rect(111,1,16,8,WHITE);
		ssd1306_draw_fast_vline(109,3,4, WHITE);
		ssd1306_draw_fast_vline(108,3,4, WHITE);
}

void draw_bt_icon(void)
{
		ssd1306_draw_bitmap(0,0,ble_icon,13,13,WHITE);
}

void display_time(void)
{
		char buffer_display[100];
		int str_len;
	  char day[4] = "";
	  char mount[4] = "";
	
		ssd1306_clear_display();
		display_mini_time();	
		
		ssd1306_set_textcolor(WHITE);
		str_len = sprintf(buffer_display,"%02d:%02d",m_a2_info.time_h, m_a2_info.time_m);
		ssd1306_drawRightStringLen(buffer_display, str_len, 128,15, 6);
		ssd1306_set_cursor(33,57);
	 
	  
		switch(dayOfTheWeek(m_a2_info)){
				case 1:
						memcpy(day, "Mon", 3);
				break;
				case 2:
						memcpy(day, "Tue", 3);
				break;
				case 3:
						memcpy(day, "Wed", 4);
				break;
				case 4:
						memcpy(day, "Thu", 3);
				break;
				case 5:
						memcpy(day, "Fri", 3);
				break;
				case 6:
						memcpy(day, "Sat", 3);	
				break;
				case 7:
						memcpy(day, "Son", 3);
				break;

		}
		switch(m_a2_info.date_m){
				case 1:
						memcpy(mount, "Jan", 3);
				break;
				case 2:
						memcpy(mount, "Feb", 3);
				break;
				case 3:
						memcpy(mount, "Mar", 3);
				break;
				case 4:
						memcpy(mount, "Apr", 3);
				break;	
				case 5:
						memcpy(mount, "May", 3);
				break;
				case 6:
						memcpy(mount, "Jun", 3);
				break;	
				case 7:
						memcpy(mount, "Jul", 3);
				break;		
				case 8:
						memcpy(mount, "Aug", 3);
				break;	
				case 9:
						memcpy(mount, "Sep", 3);
				break;	
				case 10:
						memcpy(mount, "Oct", 3);
				break;
				case 11:
						memcpy(mount, "Nov", 3);
				break;			
				case 12:
						memcpy(mount, "Dec", 3);
				break;

		}
		
		sprintf(buffer_display, "%s %d %s", day, m_a2_info.date_d, mount);
		ssd1306_putstring(buffer_display);
		draw_battery();
		draw_bt_icon();
		ssd1306_display();			
}

void display_step_count(void)
{
		char buffer_display[100];
		int str_len;
	
		ssd1306_clear_display();
		display_mini_time();
		
		if(m_a2_info.step_count < 9999){
				 sprintf(buffer_display,"%d",m_a2_info.step_count);
				 ssd1306_drawCentreString(buffer_display,65,16,6);
		}else{
				sprintf(buffer_display,"%d",m_a2_info.step_count); 
				ssd1306_drawCentreString(buffer_display,65,22,4);
		}
		str_len = sprintf(buffer_display,"steps");
		ssd1306_drawRightStringLen(buffer_display, str_len, 83,50, 2);
		ssd1306_draw_triangle(0,34,7, 27, 7, 41, WHITE);
		ssd1306_draw_triangle(127,34,120, 27, 120, 41, WHITE);
		draw_battery();
		draw_bt_icon();
		ssd1306_display();			
}

void display_mini_time()
{
		static uint8_t mes_hr_display_count;
		char buffer_display[100];
	
		ssd1306_set_textcolor(WHITE);
		ssd1306_set_textsize(1);
		if(m_a2_sensor_config.sem_get_heart_rate == 1 && m_a2_info.usb_pin_status != 1)
		{
				ssd1306_set_cursor(45,2);
				if(mes_hr_display_count == 0)
				{
						sprintf((char*)buffer_display,"HR Mes");
						ssd1306_putstring(buffer_display);
				}else if(mes_hr_display_count == 1){
						sprintf((char*)buffer_display,"HR Mes.");
						ssd1306_putstring(buffer_display);
				}else if(mes_hr_display_count == 2){
						sprintf((char*)buffer_display,"HR Mes..");
						ssd1306_putstring(buffer_display);
				}else{
						sprintf((char*)buffer_display,"HR Mes...");
						ssd1306_putstring(buffer_display);
				}	
				mes_hr_display_count++;
				if(mes_hr_display_count>3)
				{
						mes_hr_display_count = 0;
				}
		}else if(m_display_page != DISPLAY_TIME_PAGE){
				ssd1306_set_cursor(52,2);
				sprintf(buffer_display,"%02d:%02d",m_a2_info.time_h, m_a2_info.time_m);
				ssd1306_putstring(buffer_display);
		}
}

void display_heart_reate(uint8_t state_hr)
{
		char buffer_display[100];
		int str_len;
	
		ssd1306_clear_display();
		display_mini_time();
		switch(state_hr)
		{
			case DISPlAY_SHOW_SYMBOL_VALUE:
					ssd1306_draw_bitmap(20,15,heart_icon,35,35,WHITE);
					sprintf(buffer_display,"--");
					ssd1306_drawRightString(buffer_display,118,13,6);
				break;
			case DISPlAY_SHOW_HR_VALUE:
					if(m_a2_info.heart_rate <100){
							ssd1306_draw_bitmap(20,15,heart_icon,35,35,WHITE);
							sprintf(buffer_display,"%d",m_a2_info.heart_rate);
							ssd1306_drawRightString(buffer_display,118,13,6);
					}else{
							ssd1306_draw_bitmap(9,17,heart_icon,35,35,WHITE);
							sprintf(buffer_display,"%d",m_a2_info.heart_rate);
							ssd1306_drawRightString(buffer_display,121,13,6);
					}
				break;
			case DISPlAY_DO_NOT_SHOW_HR_VALUE:
					if(m_a2_info.heart_rate <100){
							ssd1306_draw_bitmap(20,15,heart_icon,35,35,WHITE);
					}else{
							ssd1306_draw_bitmap(9,17,heart_icon,35,35,WHITE);
					}						
				break;
		}
		
		str_len = sprintf(buffer_display,"BPM");
		ssd1306_drawRightStringLen(buffer_display, str_len, 115,49, 2);
		ssd1306_draw_triangle(0,34,7, 27, 7, 41, WHITE);
		ssd1306_draw_triangle(127,34,120, 27, 120, 41, WHITE);
		draw_battery();
		draw_bt_icon();
		ssd1306_display();
    		
}


void display_o2(void)
{
		char buffer_display[100];
		int str_len;
	
		ssd1306_clear_display();
		display_mini_time();		
		ssd1306_draw_bitmap(17,15,spo2_icon,42,42,WHITE);
		sprintf(buffer_display,"%d", m_a2_info.pulse_oximeter );
		ssd1306_drawRightString(buffer_display,120,13,6);
		str_len = sprintf(buffer_display,"%%");
		ssd1306_drawRightStringLen(buffer_display, str_len, 119,49, 2);
		ssd1306_draw_triangle(0,34,7, 27, 7, 41, WHITE);
		ssd1306_draw_triangle(127,34,120, 27, 120, 41, WHITE);
		draw_battery();
		draw_bt_icon();
		ssd1306_display();
}


void display_temperature(void)
{
		char buffer_display[100];
		int str_len;
		uint32_t pox;
	
		ssd1306_clear_display();
		display_mini_time();
		ssd1306_draw_bitmap(14,14,temperature_icon,18,37,WHITE);
		sprintf(buffer_display,"%d", (m_a2_info.temperature%100)/10 );
		pox = ssd1306_drawRightString(buffer_display,120,13,6);
		sprintf(buffer_display,"%d",m_a2_info.temperature/100);
		ssd1306_drawRightString(buffer_display,127-pox-10,13,6);
		ssd1306_fill_rect(90,45,4,4,WHITE);
		str_len = sprintf(buffer_display,"celsius");
		ssd1306_drawRightStringLen(buffer_display, str_len, 119,49, 2);
		ssd1306_draw_triangle(0,34,7, 27, 7, 41, WHITE);
		ssd1306_draw_triangle(127,34,120, 27, 120, 41, WHITE);
		draw_battery();
		draw_bt_icon();
		ssd1306_display();		
}

void display_distanc_cal(void)
{
		char buffer_display[100];
		int str_len;
		uint32_t pox;
		
		ssd1306_clear_display();
		display_mini_time();
		ssd1306_draw_bitmap(13,20,distance_icon,32,40,WHITE);
		if(m_a2_info.distance > 9999)
		{
				str_len = sprintf(buffer_display,"km");
				pox = ssd1306_drawRightStringLen(buffer_display, str_len, 115,27, 2);			
				str_len = sprintf(buffer_display,"%d.%d", m_a2_info.distance/1000, (m_a2_info.distance%1000)/100);
				ssd1306_drawRightStringLen(buffer_display, str_len, 127-pox-15,20, 4);
		}
		else if(m_a2_info.distance > 999)
		{
				str_len = sprintf(buffer_display,"km");
				pox = ssd1306_drawRightStringLen(buffer_display, str_len, 115,27, 2);			
				str_len = sprintf(buffer_display,"%d.%02d", m_a2_info.distance/1000, (m_a2_info.distance%1000)/10);
				ssd1306_drawRightStringLen(buffer_display, str_len, 127-pox-15,20, 4);
		}else{
				str_len = sprintf(buffer_display,"m");
				pox = ssd1306_drawRightStringLen(buffer_display, str_len, 115,27, 2);
				str_len = sprintf(buffer_display,"%d", m_a2_info.distance);
				ssd1306_drawRightStringLen(buffer_display, str_len, 127-pox-15,20, 4);
		}
		
		if(m_a2_info.calories > 9999)
		{
				str_len = sprintf(buffer_display,"Cal");
				pox = ssd1306_drawRightStringLen(buffer_display, str_len, 126,47, 2);

				str_len = sprintf(buffer_display,"%d.%d",m_a2_info.calories/1000, (m_a2_info.calories%1000)/100 );
				ssd1306_drawRightStringLen(buffer_display, str_len, 127-pox-3,40, 4);
		}else if(m_a2_info.calories > 999)
		{
				str_len = sprintf(buffer_display,"Cal");
				pox = ssd1306_drawRightStringLen(buffer_display, str_len, 120,47, 2);

				str_len = sprintf(buffer_display,"%d.%02d",m_a2_info.calories/1000, (m_a2_info.calories%1000)/10 );
				ssd1306_drawRightStringLen(buffer_display, str_len, 127-pox-9,40, 4);
		}else{
				str_len = sprintf(buffer_display,"cal");
				pox = ssd1306_drawRightStringLen(buffer_display, str_len, 125,47, 2);

				str_len = sprintf(buffer_display,"%d",m_a2_info.calories);
				ssd1306_drawRightStringLen(buffer_display, str_len, 127-pox-4,40, 4);
		}

		ssd1306_draw_triangle(0,34,7, 27, 7, 41, WHITE);
		ssd1306_draw_triangle(127,34,120, 27, 120, 41, WHITE);
		draw_battery();
		draw_bt_icon();
		ssd1306_display();	
}

void start_scroll(uint8_t bt)
{
		if(bt == LEFT_BT_PUSH)
		{
				ssd1306_start_scroll_left(0,0x0F);	
				//nrf_delay_ms(300);
		}else if(bt == RIGHT_BT_PUSH)
		{
				ssd1306_start_scroll_right(0,0x0F);	
				//nrf_delay_ms(300);
		}
}

void stop_scroll(void)
{
		ssd1306_stop_scroll();
}

void show_display(void)
{
		switch(m_display_page)
		{
			case DISPLAY_TIME_PAGE:
				display_time();
				break;
			case DISPLAY_STEP_COUNT_PAGE:
				display_step_count();
				break;
			case DISPLAY_DISTANCE_CAL_PAGE:
				display_distanc_cal();;
				break;
			case DISPLAY_TEMPERATURE_PAGE:
				display_temperature();
				break;
			case DISPLAY_O2_PAGE:
				display_o2();
				break;
			case DISPLAY_HEART_RATE_PAGE:
				display_heart_reate(DISPlAY_SHOW_HR_VALUE);
				break;
			case DISPLAY_BATTERY_FULL:
				display_battery_full();
				NRF_LOG_INFO("FULL BATT SHOW");
				NRF_LOG_FLUSH();
	
				break;
			case DISPLAY_COUNT_DOWN_ALERT_PAGE:
				NRF_LOG_INFO("Display Count Down Page");
				NRF_LOG_FLUSH();	
				break;
			case DISPLAY_BT_SOS_PAGE:
				NRF_LOG_INFO("Display SOS Page");
				NRF_LOG_FLUSH();	
				break;
			default:
				NRF_LOG_INFO("Not found page %d", m_display_page);
				NRF_LOG_FLUSH();
				break;
		}
}


// number of days since 2000/01/01, valid for 2001..2099
uint16_t date2days(uint16_t y, uint8_t m, uint8_t d) 
{
    if (y >= 2000)
        y -= 2000;
    uint16_t days = d;
    for (uint8_t i = 1; i < m; ++i)
        days += daysInMonth[i - 1];
    if (m > 2 && y % 4 == 0)
        ++days;
    return days + 365 * y + (y + 3) / 4 - 1;
}


void unixtimestamp_to_date_time(uint32_t unixtimestamp, ble_date_time_t * date_time)
{
		uint32_t t = 0;
		uint16_t days = 0;
		uint8_t yOff = 0, m = 0, d = 0;
		uint8_t leap;
		uint8_t daysPerMonth = 0;
	    t = unixtimestamp;
			t -= SECONDS_FROM_1970_TO_2000  ;
			date_time->seconds = t % 60;
			t /= 60;
			date_time->minutes = t % 60;
			t /= 60;
			date_time->hours  = t % 24;
			days = t / 24;
	   
		 for (yOff = 0; ; ++yOff) {
			leap = yOff % 4 == 0;
			if (days < 365 + leap)
					break;
			days -= 365 + leap;
		 }
		 for (m = 1; ; ++m) {
			 daysPerMonth = daysInMonth[m - 1];
			 if (leap && m == 2)
					++daysPerMonth;
			 if (days < daysPerMonth)
					break;
			 days -= daysPerMonth;
		 }
		 d = days + 1;
		 
		 date_time->year = yOff + 2000;
		 date_time->month = m;
		 date_time->day = d;
}

uint32_t date_time_to_unixtimestamp(ble_date_time_t date_time)
{
	uint16_t days = 0;
	days = date2days(date_time.year, date_time.month, date_time.day);
	return  (uint32_t)(days*86400L + date_time.hours*3600 + date_time.minutes*60 + date_time.seconds + SECONDS_FROM_1970_TO_2000);

}

uint8_t dayOfTheWeek(a2_info_t info_t)
{    
    uint16_t day = date2days(info_t.date_y, info_t.date_m, info_t.date_d);
    return (day + 6) % 7; // Jan 1, 2000 is a Saturday, i.e. returns 6
}

void display_countdown_alert(uint8_t display_number)
{
		char buffer_display[10];
		const uint8_t c_x = 53;
		const uint8_t c_y = 10;
		const uint8_t f_size = 6;
	
		ssd1306_clear_display();
		ssd1306_draw_rect(0,0,128,64,1);
		
		switch(display_number)
		{
			case 3:
				ssd1306_fill_rect(0,0,128,64,1);
				ssd1306_display();
				ssd1306_set_textcolor_bg(0,1);
				ssd1306_set_textsize(f_size);
				ssd1306_set_cursor(c_x,c_y);
				sprintf(buffer_display,"3");
				break;
			case 2:
				ssd1306_fill_rect(0,0,128-38,64,1);
				ssd1306_display();
			  ssd1306_set_textcolor_bg(0,1);
				ssd1306_set_textsize(f_size);
				ssd1306_set_cursor(c_x,c_y);
				sprintf(buffer_display,"2");
				break;
			case 1:
				ssd1306_fill_rect(0,0,128-90,64,1);
				ssd1306_display();
			  ssd1306_set_textcolor_bg(1,0);
				ssd1306_set_textsize(f_size);
				ssd1306_set_cursor(c_x,c_y);
				sprintf(buffer_display,"1");
				break;
			case 0:
				ssd1306_set_textsize(f_size);
				ssd1306_set_cursor(c_x,c_y);
				sprintf(buffer_display,"0");
				break;
		}
		
		ssd1306_putstring(buffer_display);
		
		ssd1306_display();
}

void display_bt_alert(uint8_t en_invert)
{
		char buffer_display[20];
		const uint8_t p_x = 43;
		const uint8_t p_y = 0;
		const uint8_t c_x = 30;
		const uint8_t c_y = 50;
		
		ssd1306_clear_display();
		ssd1306_draw_bitmap(p_x, p_y, ambulance_icon ,42,45,1);	
		//ssd1306_display();
	  ssd1306_set_textcolor(WHITE);
		ssd1306_set_textsize(1);
		ssd1306_set_cursor(c_x,c_y);
		sprintf(buffer_display,"PUSH AERTT!");
	  ssd1306_putstring(buffer_display);
	
		if(en_invert == 0)
		{	
				ssd1306_invert_display(0);
		}else{
			  ssd1306_invert_display(1);
		}	
		
		ssd1306_display();
}


void display_fall_alert(uint8_t en_invert)
{
		char buffer_display[20];
		const uint8_t p_x = 43;
		const uint8_t p_y = 0;
		const uint8_t c_x = 30;
		const uint8_t c_y = 50;
		
		ssd1306_clear_display();
		ssd1306_draw_bitmap(p_x, p_y, ambulance_icon ,42,45,1);	
		//ssd1306_display();
	  ssd1306_set_textcolor(WHITE);
		ssd1306_set_textsize(1);
		ssd1306_set_cursor(c_x,c_y);
		sprintf(buffer_display,"FALL AERTT!");
	  ssd1306_putstring(buffer_display);
	
		if(en_invert == 0)
		{	
				ssd1306_invert_display(0);
		}else{
			  ssd1306_invert_display(1);
		}	
		
		ssd1306_display();
}


