#include "heart_rate_cal.h"
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_delay.h"

float Average(const float chumk[],uint32_t len);
float Std(const float chumk[],uint32_t len);


int32_t ir_dc_remover(int32_t input)
{
		const double alpha = 0.95;
	  static double dcw = 0;
		
		double olddcw = dcw;
		dcw = input+ (alpha*dcw);
	
		return (int32_t)(dcw-olddcw);
}

float ir_dc_removerf(int32_t input)
{
		const float alpha = 0.95;
	  static float dcw = 0;
		
		float olddcw = dcw;
		dcw = input+ (alpha*dcw);
	
		return (dcw-olddcw);
}

int32_t ir_dc_remover2(int32_t input)
{
		const double alpha = 0.95;
	  static double dcw = 0;
		
		double olddcw = dcw;
		dcw = input+ (alpha*dcw);
	
		return (int32_t)(dcw-olddcw);
}


int32_t red_dc_remover(int32_t input)
{
		const double alpha = 0.95;
	  static double dcw = 0;
		
		double olddcw = dcw;
		dcw = input+ (alpha*dcw);
	
		return (int32_t)(dcw-olddcw);
}

/************************************************************************
* http://www.schwietering.com/jayduino/filtuino/
* Low pass butterworth filter order=1 alpha1=0.1
* Fs=400Hz, Fc=6Hz
************************************************************************/
uint32_t btw_fileter_400Hz_order2(btw_data_t *p_btw_data, const uint32_t sample)
{
    uint32_t output_filter = 0;
    p_btw_data->data[0] = p_btw_data->data[1];
    p_btw_data->data[1] = p_btw_data->data[2];

    p_btw_data->data[2] = (5.371697748120518590e-4 * sample) + (-0.93552890497917862156 * p_btw_data->data[0])+(1.93338022587993041412 * p_btw_data->data[1]);

    output_filter = (p_btw_data->data[0] + p_btw_data->data[2]) + (2*p_btw_data->data[1]);

    return output_filter;
}


/************************************************************************
* http://www.schwietering.com/jayduino/filtuino/
* Low pass butterworth filter order=1 alpha1=0.1
* Fs=400Hz, Fc=6Hz
************************************************************************/
uint32_t btw_fileter_400Hz_cf_4Hz_order2(btw_data_t *p_btw_data, const uint32_t sample)
{
    uint32_t output_filter = 0;
    p_btw_data->data[0] = p_btw_data->data[1];
    p_btw_data->data[1] = p_btw_data->data[2];

    p_btw_data->data[2] = (9.446918438401618072e-4  * sample) + (-0.91497583480143362955  * p_btw_data->data[0])+(1.91119706742607298189  * p_btw_data->data[1]);

    output_filter = (p_btw_data->data[0] + p_btw_data->data[2]) + (2*p_btw_data->data[1]);

    return output_filter;
}

/************************************************************************
* http://www.schwietering.com/jayduino/filtuino/
* Low pass butterworth filter order=1 alpha1=0.1
* Fs=100Hz, Fc=1Hz
************************************************************************/

int32_t btw_fileter_100Hz_cf_1Hz_order2(btw_data_t *p_btw_data, const int32_t sample)
{
		int32_t output_filter = 0;
    p_btw_data->data[0] = p_btw_data->data[1];
    p_btw_data->data[1] = p_btw_data->data[2];

    p_btw_data->data[2] = (9.446918438401618072e-4  * sample) + (-0.91497583480143362955  * p_btw_data->data[0])+(1.91119706742607298189  * p_btw_data->data[1]);

    output_filter = (p_btw_data->data[0] + p_btw_data->data[2]) + (2*p_btw_data->data[1]);

    return output_filter;
}

/************************************************************************
*
*
*
************************************************************************/

uint8_t ZScore(const float samples[], uint32_t len,uint32_t lag, double threshold, double influence, int8_t signals[])
{
		float filteredY[800];
		float avgFilter[800];
		float stdFilter[800];

    for(uint32_t i=0;i<=lag;i++)
    {
        filteredY[i] = samples[i];
    }

    avgFilter[lag] = Average(&samples[0],lag);
    stdFilter[lag] = Std(&samples[0],lag);

    for(uint32_t i = lag+1; i < len; i++ ){
  
        float f = samples[i];
        abs( (samples[i]-avgFilter[i-1]) );
        threshold*stdFilter[i-1];
			
				if( (float)(abs((int32_t)(samples[i]-avgFilter[i-1]))) > threshold*(float)(stdFilter[i-1]) ) 
        {
            if( samples[i] > avgFilter[i-1] )
            {
                signals[i] = 1;                  
            }else{
                signals[i] = -1;
            }   
            filteredY[i] =  (uint32_t)( (influence*f) + ( (1-influence)*(float)filteredY[i-1] ) );
        }else{
            
            signals[i] = 0;
            filteredY[i] = samples[i];  
        }

        avgFilter[i] = Average( &filteredY[i-lag], lag );
        stdFilter[i] = Std( &filteredY[i-lag], lag );
    }    

    return 0;
}


/************************************************************************
*
*
*/
float Average(const float chumk[],uint32_t len)
{
    uint32_t data_size = len;
    float sum = 0;
    for(uint32_t i=0;i<data_size;i++)
    {
        float sample = chumk[i]; 
        sum += sample;    
    }

    return sum/(float)data_size;
}

/************************************************************************
*
*
*/
float Std(const float chumk[],uint32_t len)
{
    float mean = 0;
    float sum = 0;
    float standardDeviation = 0.0;

    for(uint32_t i=0;i<len;i++)
    {
        float sample = chumk[i];
        sum += (float)sample;    
    }
    mean = (float)sum/len;
    for(uint32_t j=0; j<len; j++)
    {
        float dif = ((float)chumk[j]-(float)mean);    
        standardDeviation = standardDeviation +  pow(dif, 2);  
    }
    standardDeviation = sqrt(standardDeviation/len);

    return (float)(standardDeviation);
}


uint8_t heart_rate_cal(zscore_cfg_t zscore_cfg, int8_t signals[], uint32_t len, double *output_HR)
{
    double heart_rate_avr = 0.0;
    const uint8_t min_threshold = 8;
    const uint32_t min_theshold_hold = zscore_cfg.lag - 20;
    uint8_t detect_counter = 0;
    uint32_t hold_coutdown = 0;
    uint32_t pre_edge_index = 0;
    uint32_t cur_edge_index = 0;
    uint32_t begin_edge_index =0;
    uint32_t end_edge_index =0;
    uint16_t heart_rate_avr_counter = 0; 
    static HR_detect_state_t state; 
		state = FINDING_EDGE_STATE;

    for(uint32_t i=0;i<len;i++)
    {
        switch(state)
        {
            case FINDING_EDGE_STATE:
                if( signals[i] == -1)
                {
                    state = CHECKING_CONTINUES_STATE;
                    detect_counter++;
                    begin_edge_index = i;
                }else{
                    state = FINDING_EDGE_STATE;
                }
                break;
            case CHECKING_CONTINUES_STATE:
                if( signals[i] == 0 && detect_counter < min_threshold)
                {
                    state = FINDING_EDGE_STATE;
                }else if( signals[i] == 0 && detect_counter >= min_threshold)
                {
                    state = DETECTED_EDGE_STATE;
                    end_edge_index = i-1;
                    cur_edge_index = begin_edge_index + (end_edge_index-begin_edge_index)/2;
                }else{
                    detect_counter++;
                }
								
								
                
                break;
            case DETECTED_EDGE_STATE:
                //printf("Found edge,%u,", i);
                if(pre_edge_index != 0)
                {
                    double heart_rate = 60/((cur_edge_index-pre_edge_index)*0.0025);
                    heart_rate_avr += heart_rate;
                    heart_rate_avr_counter++;
                    //printf("%f,\n", heart_rate);
                }
                pre_edge_index = cur_edge_index;
                hold_coutdown = min_theshold_hold;
                state = HOLD_EDGE_STATE;
                break;
            case HOLD_EDGE_STATE:
                hold_coutdown--;
                if(hold_coutdown <= 0)
                {
                    state = FINDING_EDGE_STATE;    
                }
                break;
        }
    }

    *output_HR = heart_rate_avr/heart_rate_avr_counter;

    return (uint8_t)(heart_rate_avr/heart_rate_avr_counter);
}

uint8_t heart_rate_cal_v2(float hr_data[], uint32_t len, int8_t signals[], hr_data_t *output_hr)
{
		uint32_t plus_index = 0;
		uint32_t minus_index = 0;
		uint16_t peak_index[20];
		uint8_t peak_counter = 0;
		float max_value = -127.0;
		uint32_t kk =0;
		uint32_t avr_index = 0;
		
		uint32_t prv_avr_index = 0;
		float hr = 0;	
		uint8_t hr_buf[20];
		uint8_t hr_buf_count = 0;
		
		peak_search_state_t m_peak_search_state = HR_FIND_PLUS;
		
	
		ZScore( hr_data, len, 5, 2.7, 0.3, signals);
	
		for(uint32_t jj=0;jj<len;jj++)
		{
			
				switch(m_peak_search_state)
				{	
					case HR_FIND_PLUS:
						if(signals[jj] == 1)
						{
								m_peak_search_state = HR_FIND_MINUS;
								plus_index = jj;
								NRF_LOG_RAW_INFO("\n\rPlus Signal %ld, %ld", signals[jj] , jj);
						}
						break;
					case HR_FIND_MINUS:
						if(signals[jj] == -1)
						{
								m_peak_search_state = HR_FIND_PEAK_INDEX;
								minus_index = jj;
								NRF_LOG_RAW_INFO("\n\rMinus Signal %ld, %ld", signals[jj] , jj);
						}
						break;
					case HR_FIND_PEAK_INDEX:
						max_value = -127.0;
					
						NRF_LOG_RAW_INFO("\n\rplus_index %d, minus_index %d, ", plus_index, minus_index);
						NRF_LOG_FLUSH();
						//nrf_delay_ms(10);

						for( kk=plus_index;kk<minus_index;kk++)
						{
								if( hr_data[kk]  > max_value)
								{
										peak_counter = 0;
										max_value =  hr_data[kk];
										peak_index[peak_counter] = kk;
										peak_counter++;
								}else if(hr_data[kk] == max_value && peak_counter < sizeof(peak_index))
								{
										peak_index[peak_counter] = kk;
										peak_counter++;
								}
						}
						
						if(peak_counter>0)
						{
								NRF_LOG_RAW_INFO("Peak Index, ");
								NRF_LOG_FLUSH();
								//nrf_delay_ms(10);
								for(uint32_t i = 0; i<peak_counter;i++)
								{
										NRF_LOG_RAW_INFO("%ld,", peak_index[i]);
										NRF_LOG_FLUSH();
								}
								if(peak_counter > 1)
								{
										avr_index =  ( peak_index[0] + peak_index[peak_counter-1]) / 2;
										NRF_LOG_RAW_INFO(", AVR Peak Index, %ld" , avr_index );
										NRF_LOG_FLUSH();
								}else{
										avr_index =  peak_index[0];
										NRF_LOG_RAW_INFO(", AVR Peak Index, %ld" , avr_index );
										NRF_LOG_FLUSH();
								}
								
								if(prv_avr_index != 0)
								{
										 //hr = 6000.0/(avr_index - prv_avr_index);		
									   hr = 4080.0/(avr_index - prv_avr_index);		
								}
								
								prv_avr_index = avr_index;		
								
								NRF_LOG_RAW_INFO(", HR %ld" , (uint16_t)hr );
								NRF_LOG_FLUSH();
								
								if((uint16_t)hr < 30 || (uint16_t)hr > 180  )
								{
										NRF_LOG_RAW_INFO(", Reject out of 30-180." , (uint16_t)hr );
										NRF_LOG_FLUSH();
								}else{
										hr_buf[hr_buf_count] = hr;
										hr_buf_count++;
								}
								
						}
						m_peak_search_state = HR_FIND_PLUS;
						break;
				}	
		}
		
		NRF_LOG_INFO("HR>>>");
		
		for (int i = 0; i < hr_buf_count; i++)                     //Loop for ascending ordering
		{
			for (int j = 0; j < hr_buf_count; j++)             //Loop for comparing other values
			{
				if (hr_buf[j] > hr_buf[i])                //Comparing other array elements
				{
					uint8_t tmp = hr_buf[i];         //Using temporary variable for storing last value
					hr_buf[i] = hr_buf[j];            //replacing value
					hr_buf[j] = tmp;             //storing last value
				}  
			}
		}
		
				float mean = 0;
				float sum = 0;
				float standardDeviation = 0.0;

		
		if(hr_buf_count>1)
		{
		
				for(uint32_t i=0;i<hr_buf_count;i++)
				{
						float sample = hr_buf[i];
						sum += (float)sample;   
						output_hr->hr_data[i] = (uint8_t)hr_buf[i];	
				}
				mean = (float)sum/hr_buf_count;
				for(uint32_t j=0; j<hr_buf_count; j++)
				{
						float dif = ((float)hr_buf[j]-(float)mean);    
						standardDeviation = standardDeviation +  pow(dif, 2);  
				}
				standardDeviation = sqrt(standardDeviation/hr_buf_count);
				
				NRF_LOG_INFO("HR Mean %ld ", (uint16_t)mean);
				NRF_LOG_FLUSH();
				
				
				NRF_LOG_INFO("HR SD " NRF_LOG_FLOAT_MARKER  " ", NRF_LOG_FLOAT(standardDeviation));
				NRF_LOG_FLUSH();
				
				output_hr->hr_avr = (uint16_t)mean;
				output_hr->hr_sd = standardDeviation;
				output_hr->hr_len = hr_buf_count;		
		}else{
				output_hr->hr_avr = hr_buf[0];
		}
	  
		
		
		uint16_t sum_of_div = 0;
		
		for(uint32_t zz=0;zz<hr_buf_count;zz++)
		{
				NRF_LOG_RAW_INFO("%d," , hr_buf[zz] );
				NRF_LOG_FLUSH();
				if(zz>0)
				{
						sum_of_div += abs(hr_buf[zz] - hr_buf[zz-1]);
				}
		}
		
		NRF_LOG_RAW_INFO(" sum div %d, %d" , sum_of_div, sum_of_div/(hr_buf_count-1) );
		NRF_LOG_FLUSH();
		
//		for(uint32_t zz=0;zz<hr_buf_count;zz++)
//		{
//				if( abs( hr_buf[hr_buf_count>>2] - hr_buf[zz] ) < 15 )
//				{
//						hr_buf2[hr_buf_count2] = hr_buf[zz];
//						hr_buf_count2++;
//				}
//		}

//		NRF_LOG_INFO("HR final>>>");
//		uint16_t sum_hr = 0;
//		
//		for(uint32_t zz=0;zz<hr_buf_count2;zz++)
//		{
//				NRF_LOG_RAW_INFO("%d," , hr_buf2[zz] );
//				NRF_LOG_FLUSH();
//				sum_hr += hr_buf2[zz];
//		}

		NRF_LOG_INFO("\n\rHR  >>> %d", mean );
		NRF_LOG_FLUSH();
				
//		*output_hr = mean;
		
		return 1;
}



uint8_t heart_rate_cal_v3(float hr_data[], uint32_t len, int8_t signals[], hr_data_t *output_hr)
{
		float max_value = -10000.0;
		uint16_t peak_index[100];
		uint16_t peak_counter = 0;
		uint8_t buffer_hr = 0;
	
		peak_search_state_t m_peak_search_state = HR_FIND_PLUS;
	
		ZScore( hr_data, len, 5, 2.7, 0.3, signals);
	
		output_hr->hr_len = 0;
	
		for(uint32_t i=0;i<len;i++)
		{
				switch(m_peak_search_state)
				{
					case HR_FIND_PLUS:
						if(signals[i] == 1)
						{
								m_peak_search_state = HR_FIND_MINUS;
								max_value = -10000.0;
								
						}					
						break;
					case HR_FIND_MINUS:
						if(hr_data[i] >= max_value)
						{
								max_value = hr_data[i];
								peak_index[peak_counter] = i;
						}	
						if(signals[i] == -1)
						{
								m_peak_search_state = HR_FIND_PEAK_INDEX;
						}
						break;
					case HR_FIND_PEAK_INDEX:
						if(peak_counter > 0)
						{
								buffer_hr = 7080/(peak_index[peak_counter] - peak_index[peak_counter-1]);
								if(buffer_hr > MIN_HR_VALUE && buffer_hr < MAX_HR_VALUE)
								{
										output_hr->hr_data[output_hr->hr_len] = buffer_hr ;
										output_hr->hr_len++;
								}
						}
						if(peak_counter<sizeof(peak_index))
						{
								peak_counter++;
						}
						m_peak_search_state = HR_FIND_PLUS;
						break;
				}
		}
		
//#ifdef DEBUG_HR					
		
		if(peak_counter>0)
		{
				NRF_LOG_INFO("Peak Index,");
				NRF_LOG_FLUSH();
				for(uint32_t i=0;i<peak_counter;i++)
				{
						NRF_LOG_RAW_INFO(",%d",peak_index[i]);
						NRF_LOG_FLUSH();
						nrf_delay_ms(10);	
				}
				NRF_LOG_RAW_INFO("\n\r,END Peak Index,");
				NRF_LOG_FLUSH();
		}
		
		float mean = 0;
		float sum = 0;
		float standardDeviation = 0.0;

		
		if(output_hr->hr_len>0)
		{
				NRF_LOG_INFO("HR Index,");
				NRF_LOG_FLUSH();
				
				for(uint32_t i=0;i<output_hr->hr_len;i++)
				{
						float sample = output_hr->hr_data[i];
						sum += (float)sample;
						NRF_LOG_RAW_INFO(",%d", output_hr->hr_data[i]);
						NRF_LOG_FLUSH();
				}
				mean = (float)sum/output_hr->hr_len;
				for(uint32_t j=0; j<output_hr->hr_len; j++)
				{
						float dif = ((float)output_hr->hr_data[j]-(float)mean);    
						standardDeviation = standardDeviation +  pow(dif, 2);  
				}
				standardDeviation = sqrt(standardDeviation/output_hr->hr_len);
				
				NRF_LOG_INFO("HR Mean %ld ", (uint16_t)mean);
				NRF_LOG_FLUSH();
				
				
				NRF_LOG_INFO("HR SD " NRF_LOG_FLOAT_MARKER  " ", NRF_LOG_FLOAT(standardDeviation));
				NRF_LOG_FLUSH();
				
				output_hr->hr_avr = (uint16_t)mean;
				output_hr->hr_sd  = standardDeviation;
		}else{
				output_hr->hr_avr = output_hr->hr_data[0];
				output_hr->hr_sd = 0;
				
		}
	
//#endif
		
		return 1;
}


uint8_t spo2_cal(float hr_data[], uint32_t ir_data[], uint32_t red_data[] ,uint32_t len, int8_t signals[],  spo2_value_t *output_spo2)
{
		uint32_t max_ir_value = 0;
		uint32_t max_red_value = 0;
		uint32_t min_ir_value = 300000;
		uint32_t min_red_value = 300000;
	
		uint32_t peak_max_ir_index[100];
		uint32_t peak_max_red_index[100];
		uint32_t peak_min_ir_index[100];
		uint32_t peak_min_red_index[100];
	
		uint16_t max_ir_index[100];
		uint16_t min_ir_index[100];
		uint16_t max_red_index[100];
		uint16_t min_red_index[100];
	
	
		uint16_t peak_counter = 0;
		uint8_t buffer_hr = 0;

		double ac_red =0;
		double dc_red = 0;
		double ac_ir = 0;
		double dc_ir = 0;
		double r_spo2 = 0;
		double spo2_value = 0;
		//double spo2_value_17 = 0;
		double spo2_value_15 = 0;
		double spo2_value_13 = 0;
		double spo2_value_11 = 0;
		double spo2_value_09 = 0;
		
		peak_search_state_t m_peak_search_state = HR_FIND_PLUS;
	
		ZScore( hr_data, len, 5, 2.7, 0.3, signals);
	
	
		for(uint32_t i=0;i<len;i++)
		{
				switch(m_peak_search_state)
				{
					case HR_FIND_PLUS:
						
						if(ir_data[i] < min_ir_value && peak_counter > 0)
						{
								min_ir_value = ir_data[i];
								peak_min_ir_index[peak_counter] = ir_data[i] ;
								min_ir_index[peak_counter] = i;
						}
					
						if(red_data[i] < min_red_value && peak_counter > 0)
						{
								min_red_value = red_data[i];
								peak_min_red_index[peak_counter] = red_data[i];
								min_red_index[peak_counter] = i;
						}	
							
						if(signals[i] == 1)
						{
								m_peak_search_state = HR_FIND_MINUS;
								max_ir_value = 0;
								max_red_value = 0;
								if(peak_counter<sizeof(peak_max_ir_index))
								{
										peak_counter++;
								}
						}					
						break;
					case HR_FIND_MINUS:
						if(ir_data[i] >= max_ir_value)
						{
								max_ir_value = ir_data[i];
								peak_max_ir_index[peak_counter] = ir_data[i];
								max_ir_index[peak_counter] = i;
						}
							
						if(red_data[i] >= max_red_value)
						{
								max_red_value = red_data[i];
								peak_max_red_index[peak_counter] = red_data[i];
								max_red_index[peak_counter] = i;
						}	
												
						if(signals[i] == -1)
						{
								m_peak_search_state = HR_FIND_PEAK_INDEX;
								min_ir_value = 300000;
								min_red_value = 300000;
						}
						break;
					case HR_FIND_PEAK_INDEX:
						
						m_peak_search_state = HR_FIND_PLUS;
						break;
				}
		}
					
		
		if(peak_counter>0)
		{
				//NRF_LOG_INFO("SPO2 Peak Index,");
				//NRF_LOG_FLUSH();
				output_spo2->len = 0;
			  double sum_spo2 = 0;
				
				for(uint32_t i=0;i<peak_counter;i++)
				{
						
						
						double R = ((peak_max_red_index[i]-peak_min_red_index[i])/ (double)peak_min_red_index[i])/((peak_max_ir_index[i]-peak_min_ir_index[i])/(double)peak_min_ir_index[i] );
						
						if(i>=2)
						{
								dc_red = ((((double)peak_min_red_index[i]-peak_min_red_index[i-1])*((double)max_red_index[i]-min_red_index[i-1])/(double)(min_red_index[i]-min_red_index[i-1]))+(peak_min_red_index[i-1]));
								ac_red = (peak_max_red_index[i]) - dc_red;
								dc_ir  = ((((double)peak_min_ir_index[i]-peak_min_ir_index[i-1])*((double)max_ir_index[i]-min_ir_index[i-1])/(double)(min_ir_index[i]-min_ir_index[i-1]))+(peak_min_ir_index[i-1]));
								ac_ir  = (peak_max_ir_index[i]) - dc_ir;
									
								r_spo2 = (ac_red/dc_red)/(ac_ir/dc_ir);
								
								spo2_value = 104-17*r_spo2;
								
								spo2_value_15 = 104-15*r_spo2;
								spo2_value_13 = 104-13*r_spo2;
								spo2_value_11 = 104-11*r_spo2;
								spo2_value_09 = 104-4*r_spo2;
								
								if(spo2_value_09 > 94 && spo2_value_09 <= 100)
								{
										output_spo2->data[output_spo2->len] = spo2_value_09;
										output_spo2->len++;	
										sum_spo2 += spo2_value_09; 
								}
						}else{
								spo2_value = 0;
						}
					
						NRF_LOG_RAW_INFO("\n\r,%d,%ld,%ld,%ld,%d,", peak_max_red_index[i], peak_min_red_index[i] ,peak_max_ir_index[i], peak_min_ir_index[i], (uint16_t)(104-17*R)  );
						NRF_LOG_FLUSH();
						nrf_delay_ms(5);
						NRF_LOG_RAW_INFO(",%u,%u,%u,%u", max_red_index[i], min_red_index[i] ,max_ir_index[i], min_ir_index[i] );
						NRF_LOG_FLUSH();
						nrf_delay_ms(5);
						NRF_LOG_RAW_INFO("," NRF_LOG_FLOAT_MARKER, NRF_LOG_FLOAT(spo2_value));
						NRF_LOG_FLUSH();
						nrf_delay_ms(5);
						NRF_LOG_RAW_INFO("," NRF_LOG_FLOAT_MARKER, NRF_LOG_FLOAT(spo2_value_15));
						NRF_LOG_FLUSH();
						nrf_delay_ms(5);
						NRF_LOG_RAW_INFO("," NRF_LOG_FLOAT_MARKER, NRF_LOG_FLOAT(spo2_value_13));
						NRF_LOG_FLUSH();		
						nrf_delay_ms(5);
						NRF_LOG_RAW_INFO("," NRF_LOG_FLOAT_MARKER, NRF_LOG_FLOAT(spo2_value_11));
						NRF_LOG_FLUSH();		
						nrf_delay_ms(5);			
						NRF_LOG_RAW_INFO("," NRF_LOG_FLOAT_MARKER, NRF_LOG_FLOAT(spo2_value_09));
						NRF_LOG_FLUSH();		
						nrf_delay_ms(5);						
				}
				output_spo2->avr = sum_spo2/output_spo2->len;
				
				NRF_LOG_RAW_INFO("\n\r,END, , , , , , , , , , , , , , , , , , ," NRF_LOG_FLOAT_MARKER "\n\r", NRF_LOG_FLOAT(sum_spo2/output_spo2->len));
				NRF_LOG_FLUSH();
		}
		
			
		return 0;
}


uint8_t checkForBeat(int32_t sample,uint32_t index, uint32_t delta_time, uint32_t *beatPeriod)
{
		static uint8_t state = BEATDETECTOR_STATE_INIT;
		static int32_t old_sample = 0;
		static uint32_t count = 0;
		static uint32_t tsNewBeat = 0;
		uint8_t beatDetected = 0;
	
		*beatPeriod = 0;
		uint32_t millis = index*delta_time;
	
		switch(state)
		{
			case BEATDETECTOR_STATE_INIT:
				if(millis > BEATDETECTOR_INIT_HOLDOFF)
				{
						state = BEATDETECTOR_STATE_FIND_INCREASING;
				}
				break;
			case BEATDETECTOR_STATE_FIND_INCREASING:
				if(old_sample <= sample)
				{
						count = count+1;
        }else if( (old_sample > sample) && (count > BEATDETECTOR_FIND_EAGE_COUNT) ){
						count = 0;
						state = BEATDETECTOR_STATE_FIND_DECREASING;
						tsNewBeat = millis; 
				}else{
						count = 0;
				}
				break;
			case BEATDETECTOR_STATE_FIND_DECREASING:
				if(old_sample >= sample)
				{
						count = count+1;
				}else{ 
            count = 0;
            state = BEATDETECTOR_STATE_FIND_INCREASING;          
				} 
				
				if(count > BEATDETECTOR_FIND_EAGE_COUNT)
				{
						count = 0;
						state = BEATDETECTOR_STATE_MAYBE_DETECTED; 
				}
				break;
			case BEATDETECTOR_STATE_MAYBE_DETECTED:
						
			 
            state = BEATDETECTOR_STATE_MASKING;
			
						if(*beatPeriod > 330 && *beatPeriod < 1500)
						{
								beatDetected = 1;
						}else{
								beatDetected = 0;
						}
			
				break;
			case BEATDETECTOR_STATE_MASKING:
				if(millis - tsNewBeat > BEATDETECTOR_MASKING_HOLDOFF)
				{
						state = BEATDETECTOR_STATE_FIND_INCREASING;
				}
				break;
		}	

		old_sample = sample;
		
		return beatDetected;
}
















