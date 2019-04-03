#include "pedometer.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "nrf_delay.h"


static void pedometer_cal_step_rate_factor(double step_rate, double *step_factor);
static void pedometer_cal_activity_level(double speed_km_per_h , uint8_t *res_activity);
static void pedometer_cal_metabolic_factor(double step_rate, double *meta_factor);	
	
void pedometer_setup(pedometer_data_t *p_pedo_data, uint8_t gender, uint16_t weight, uint16_t height)
{
		NRF_LOG_INFO("gender in flash %d", gender);
		NRF_LOG_PROCESS();
	
		if(gender != MALE && gender != FEMALE)
		{
				p_pedo_data->gender = MALE;
			  p_pedo_data->gender_factor = GENDER_MALE_FACTOR;
		}else{
				p_pedo_data->gender = gender;
				if(p_pedo_data->gender == MALE)
				{
						p_pedo_data->gender_factor = GENDER_MALE_FACTOR;
				}else{
						p_pedo_data->gender_factor = GENDER_FEMALE_FACTOR;
				}
		}
		
		if(weight >= MIN_WEIGHT && weight <= MAX_WEIGHT )
		{
				p_pedo_data->weight = weight;
		}else{
				p_pedo_data->weight = DEFAULT_WEIGHT;
		}
		
		if(height >= MIN_HEIGHT && height <= MAX_HEIGHT)
		{
				p_pedo_data->height = height;
		}else{
				p_pedo_data->height = DEFAULT_HEIGHT;
		}
}


void pedometer_processing(pedometer_data_t *p_pedo_data, uint32_t period_time, uint16_t period_step_count)
{
			// Distance estimation please see datasheet MMA9555L page 100.
			
			p_pedo_data->step_rate = (double)period_step_count/period_time;
	
	
			//NRF_LOG_INFO("gender %d, weight %d, height %d", p_pedo_data->gender, p_pedo_data->weight, p_pedo_data->height);
			//NRF_LOG_PROCESS();
			
			//NRF_LOG_INFO("step %d ", period_step_count);
			//NRF_LOG_PROCESS();
	
		
			//NRF_LOG_INFO("gender factor: " NRF_LOG_FLOAT_MARKER " ", NRF_LOG_FLOAT(p_pedo_data->gender_factor));
			//NRF_LOG_PROCESS();
	
	
			//NRF_LOG_INFO("step rate: " NRF_LOG_FLOAT_MARKER " step/Second", NRF_LOG_FLOAT(p_pedo_data->step_rate));
			//NRF_LOG_PROCESS();
	
	
			p_pedo_data->bsl = p_pedo_data->height*p_pedo_data->gender_factor*1.1;  
			
			//NRF_LOG_INFO("bsl: " NRF_LOG_FLOAT_MARKER " ", NRF_LOG_FLOAT(p_pedo_data->bsl));
			//NRF_LOG_PROCESS();
	
			pedometer_cal_step_rate_factor(p_pedo_data->step_rate, &p_pedo_data->step_rate_factor);	
			p_pedo_data->stride = p_pedo_data->bsl*p_pedo_data->step_rate_factor;
			
			//NRF_LOG_INFO("step factor: " NRF_LOG_FLOAT_MARKER " ", NRF_LOG_FLOAT(p_pedo_data->step_rate_factor));
			//NRF_LOG_PROCESS();
	
			//NRF_LOG_INFO("stride: " NRF_LOG_FLOAT_MARKER " cm", NRF_LOG_FLOAT(p_pedo_data->stride));
			//NRF_LOG_PROCESS();
	
			p_pedo_data->distance = p_pedo_data->stride*period_step_count/100.0;  // value 100 using for convert cm to m
			
			//NRF_LOG_INFO("distance: " NRF_LOG_FLOAT_MARKER " m", NRF_LOG_FLOAT(p_pedo_data->distance));
			//NRF_LOG_PROCESS();
	
			p_pedo_data->speed_m_per_s =  ( p_pedo_data->distance/period_time ) ; // m/S
 			p_pedo_data->speed_km_per_h = p_pedo_data->speed_m_per_s*18/5.0;   // convert m/S to km/h
			
			
			//NRF_LOG_INFO("speed m/S: " NRF_LOG_FLOAT_MARKER " m/S", NRF_LOG_FLOAT(p_pedo_data->speed_m_per_s));
			//NRF_LOG_PROCESS();
	
	
			//NRF_LOG_INFO("speed km/H: " NRF_LOG_FLOAT_MARKER " km/H", NRF_LOG_FLOAT(p_pedo_data->speed_km_per_h));
			//NRF_LOG_PROCESS();
	
			pedometer_cal_activity_level(p_pedo_data->speed_km_per_h, &p_pedo_data->activity_level);
			
			//NRF_LOG_INFO("Activity %d", p_pedo_data->activity_level);
			//NRF_LOG_PROCESS();
	
			pedometer_cal_metabolic_factor(p_pedo_data->step_rate, &p_pedo_data->metabolic_factor);
			
			p_pedo_data->calories = period_step_count*p_pedo_data->metabolic_factor*0.00029*p_pedo_data->weight/p_pedo_data->step_rate;

			//NRF_LOG_INFO("calories fac: " NRF_LOG_FLOAT_MARKER " cal", NRF_LOG_FLOAT(p_pedo_data->metabolic_factor));
			//NRF_LOG_PROCESS();
	
	
			//NRF_LOG_INFO("calories: " NRF_LOG_FLOAT_MARKER " cal", NRF_LOG_FLOAT(p_pedo_data->calories));
			//NRF_LOG_PROCESS();
	

			//NRF_LOG_INFO("\n");
			//NRF_LOG_PROCESS();
	
	
}

static void pedometer_cal_step_rate_factor(double step_rate, double *step_factor)
{
		if(step_rate < 1.6)
		{
				*step_factor = STEP_RATE_FACTOR_VERY_SLOW;
		}else if(step_rate >= 1.6 && step_rate < 1.8 )
		{
				*step_factor = STEP_RATE_FACTOR_SLOW;
		}else if(step_rate >= 1.8 && step_rate < 2.35 )
		{
				*step_factor = STEP_RATE_FACTOR_NORMAL;
		}else if(step_rate >= 2.35 && step_rate < 2.8 )
		{
				*step_factor = STEP_RATE_FACTOR_FAST;
		}else if(step_rate >= 2.8 )
		{
				*step_factor = STEP_RATE_FACTOR_VERY_FAST;
		}else{
				//Not need implement
		}
}

static void pedometer_cal_activity_level(double speed_km_per_h , uint8_t *res_activity)
{
		if(speed_km_per_h >= 10.5)
		{
				*res_activity = RUNNING;
		}else if(speed_km_per_h >= 6.5 && speed_km_per_h < 10.5)
		{
		    *res_activity = JOGGING;  
		}else if(speed_km_per_h >= 1 && speed_km_per_h < 6.5)
		{
		    *res_activity = WALKING;
		}else if(speed_km_per_h < 1)
		{
			  *res_activity = REST; 		
		}else{
				//Not need implement
		}
}

static void pedometer_cal_metabolic_factor(double step_rate, double *meta_factor)
{
		if(step_rate < 1.6)
		{
				*meta_factor = METABOLIC_FACTOR_VERY_SLOW;
		}else if(step_rate >= 1.6 && step_rate < 1.8 )
		{
				*meta_factor = METABOLIC_FACTOR_SLOW;
		}else if(step_rate >= 1.8 && step_rate < 2.35 )
		{
				*meta_factor = METABOLIC_FACTOR_NORMAL;
		}else if(step_rate >= 2.35 && step_rate < 2.8 )
		{
				*meta_factor = METABOLIC_FACTOR_FAST;
		}else if(step_rate >= 2.8 )
		{
				*meta_factor = METABOLIC_FACTOR_VERY_FAST;
		}else{
				//Not need implement
		}
}













