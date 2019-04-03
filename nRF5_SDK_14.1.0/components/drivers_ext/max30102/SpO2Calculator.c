#include <math.h>

#include "SpO2Calculator.h"



#include "nrf_log.h"
#include "nrf_log_ctrl.h"


// SaO2 Look-up Table
// http://www.ti.com/lit/an/slaa274b/slaa274b.pdf
const uint8_t spO2LUT[43] = {100,100,100,100,99,99,99,99,99,99,98,98,98,98,
                                             98,97,97,97,97,97,97,96,96,96,96,96,96,95,95,
                                             95,95,95,95,94,94,94,94,94,93,93,93,93,93};

																						 
void spO2Calculator_init(spo2_data_t *p_spo2_data)
{
		p_spo2_data->samplesRecorded = 0;
		p_spo2_data->irACValueSqSum = 0.0;
		p_spo2_data->redACValueSqSum = 0.0;
		p_spo2_data->beatsDetectedNum = 0;
		p_spo2_data->spO2 = 0;
}	

uint8_t spO2Calculator_update(spo2_data_t *p_spo2_data, float irACValue, float redACValue, uint8_t beatDetected)
{
		p_spo2_data->irACValueSqSum += irACValue * irACValue;
    p_spo2_data->redACValueSqSum += redACValue * redACValue;
    p_spo2_data->samplesRecorded = p_spo2_data->samplesRecorded + 1;

    if (beatDetected) {
        p_spo2_data->beatsDetectedNum = p_spo2_data->beatsDetectedNum + 1;
        if (p_spo2_data->beatsDetectedNum == CALCULATE_EVERY_N_BEATS) {
            float acSqRatio = 100.0 * log(p_spo2_data->redACValueSqSum/p_spo2_data->samplesRecorded) / log(p_spo2_data->irACValueSqSum/p_spo2_data->samplesRecorded);
            uint8_t index = 0;

            if (acSqRatio > 66) {
                index = (uint8_t)acSqRatio - 66;
            } else if (acSqRatio > 50) {
                index = (uint8_t)acSqRatio - 50;
            }
            spO2Calculator_reset(p_spo2_data);

            p_spo2_data->spO2 = spO2LUT[index];
						
						return 1;
        }
    }
		
		return 0;
}	

void spO2Calculator_reset(spo2_data_t *p_spo2_data)
{
		p_spo2_data->samplesRecorded = 0;
		p_spo2_data->irACValueSqSum = 0.0;
		p_spo2_data->redACValueSqSum = 0.0;
		p_spo2_data->beatsDetectedNum = 0;
		p_spo2_data->spO2 = 0;
}


//uint8_t SPO2_Calulator(float hr_data[], uint32_t len, int8_t signals[], hr_data_t *output_hr)
//{
//		float max_value = -10000.0;
//		uint16_t peak_index[100];
//		uint16_t peak_counter = 0;
//		uint8_t buffer_hr = 0;
//	
//		peak_search_state_t m_peak_search_state = HR_FIND_PLUS;
//	
//		ZScore( hr_data, len, 5, 2.7, 0.3, signals);
//	
//		output_hr->hr_len = 0;
//	
//		for(uint32_t i=0;i<len;i++)
//		{
//				switch(m_peak_search_state)
//				{
//					case HR_FIND_PLUS:
//						if(signals[i] == 1)
//						{
//								m_peak_search_state = HR_FIND_MINUS;
//								max_value = -10000.0;
//								
//						}					
//						break;
//					case HR_FIND_MINUS:
//						if(hr_data[i] >= max_value)
//						{
//								max_value = hr_data[i];
//								peak_index[peak_counter] = i;
//						}	
//						if(signals[i] == -1)
//						{
//								m_peak_search_state = HR_FIND_PEAK_INDEX;
//						}
//						break;
//					case HR_FIND_PEAK_INDEX:
//						if(peak_counter > 0)
//						{
//								buffer_hr = 7080/(peak_index[peak_counter] - peak_index[peak_counter-1]);
//								if(buffer_hr > MIN_HR_VALUE && buffer_hr < MAX_HR_VALUE)
//								{
//										output_hr->hr_data[output_hr->hr_len] = buffer_hr ;
//										output_hr->hr_len++;
//								}
//						}
//						if(peak_counter<sizeof(peak_index))
//						{
//								peak_counter++;
//						}
//						m_peak_search_state = HR_FIND_PLUS;
//						break;
//				}
//		}
//				
//		
//		if(peak_counter>0)
//		{
//				NRF_LOG_INFO("Peak Index,");
//				NRF_LOG_FLUSH();
//				for(uint32_t i=0;i<peak_counter;i++)
//				{
//						NRF_LOG_RAW_INFO(",%d",peak_index[i]);
//						NRF_LOG_FLUSH();
//						nrf_delay_ms(10);	
//				}
//				NRF_LOG_RAW_INFO("\n\r,END Peak Index,");
//				NRF_LOG_FLUSH();
//		}
//		
//		float mean = 0;
//		float sum = 0;
//		float standardDeviation = 0.0;

//		
//		if(output_hr->hr_len>0)
//		{
//				NRF_LOG_INFO("HR Index,");
//				NRF_LOG_FLUSH();
//				
//				for(uint32_t i=0;i<output_hr->hr_len;i++)
//				{
//						float sample = output_hr->hr_data[i];
//						sum += (float)sample;
//						NRF_LOG_RAW_INFO(",%d", output_hr->hr_data[i]);
//						NRF_LOG_FLUSH();
//				}
//				mean = (float)sum/output_hr->hr_len;
//				for(uint32_t j=0; j<output_hr->hr_len; j++)
//				{
//						float dif = ((float)output_hr->hr_data[j]-(float)mean);    
//						standardDeviation = standardDeviation +  pow(dif, 2);  
//				}
//				standardDeviation = sqrt(standardDeviation/output_hr->hr_len);
//				
//				NRF_LOG_INFO("HR Mean %ld ", (uint16_t)mean);
//				NRF_LOG_FLUSH();
//				
//				
//				NRF_LOG_INFO("HR SD " NRF_LOG_FLOAT_MARKER  " ", NRF_LOG_FLOAT(standardDeviation));
//				NRF_LOG_FLUSH();
//				
//				output_hr->hr_avr = (uint16_t)mean;
//				output_hr->hr_sd  = standardDeviation;
//		}else{
//				output_hr->hr_avr = output_hr->hr_data[0];
//				output_hr->hr_sd = 0;
//				
//		}
//		
//		return 1;
//}
