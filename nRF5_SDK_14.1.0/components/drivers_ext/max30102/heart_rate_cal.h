#ifndef __HEART_RATE_CAL_H__
#define __HEART_RATE_CAL_H__

#include <stdint.h>
#include <stdio.h>


#define MAX_HR_VALUE										150
#define MIN_HR_VALUE										40


#define BEATDETECTOR_INIT_HOLDOFF                5000   // in ms, how long to wait before counting
#define BEATDETECTOR_MASKING_HOLDOFF             200     // in ms, non-retriggerable window after beat detection
#define BEATDETECTOR_FIND_EAGE_COUNT             10
		
typedef enum {
		BEATDETECTOR_STATE_INIT,
    BEATDETECTOR_STATE_FIND_INCREASING,
    BEATDETECTOR_STATE_FIND_DECREASING,
    BEATDETECTOR_STATE_MAYBE_DETECTED,
    BEATDETECTOR_STATE_MASKING,
}beatdetector_state_t;	
		
typedef enum {
    FINDING_EDGE_STATE,
    CHECKING_CONTINUES_STATE,
    DETECTED_EDGE_STATE,
    HOLD_EDGE_STATE,
}HR_detect_state_t;

typedef struct{
		uint8_t hr_avr;
		float hr_sd;
		uint8_t hr_data[40];
		uint8_t hr_len;
}hr_data_t;

typedef struct{
		uint8_t avr;
		float sd;
		double data[40];
		uint8_t len;
}spo2_value_t;


typedef struct{
    double data[3];
}btw_data_t;

typedef struct{
    uint32_t lag;
    double threshold;
    double influence;
}zscore_cfg_t;

typedef enum{
	HR_FIND_PLUS,
	HR_FIND_MINUS,
	HR_FIND_PEAK_INDEX,
}peak_search_state_t;

int32_t ir_dc_remover(int32_t input);
int32_t red_dc_remover(int32_t input);
int32_t ir_dc_remover2(int32_t input);


float ir_dc_removerf(int32_t input);

uint32_t btw_fileter_400Hz_order2(btw_data_t *p_btw_data, const uint32_t sample);
uint32_t btw_fileter_400Hz_cf_4Hz_order2(btw_data_t *p_btw_data, const uint32_t sample);
int32_t btw_fileter_100Hz_cf_1Hz_order2(btw_data_t *p_btw_data, const int32_t sample);

uint8_t ZScore(const float samples[], uint32_t len,uint32_t lag, double threshold, double influence, int8_t signals[]);
float Average(const float chumk[],uint32_t len);
float Std(const float chumk[],uint32_t len);
uint8_t heart_rate_cal(zscore_cfg_t zscore_cfg, int8_t signals[], uint32_t len, double *output_HR);
uint8_t heart_rate_cal_v2(float hr_data[], uint32_t len, int8_t signals[], hr_data_t *output_hr);
uint8_t heart_rate_cal_v3(float hr_data[], uint32_t len, int8_t signals[], hr_data_t *output_hr);


uint8_t spo2_cal(float hr_data[], uint32_t ir_data[], uint32_t red_data[] ,uint32_t len, int8_t signals[], spo2_value_t *output_spo2);
uint8_t checkForBeat(int32_t sample,uint32_t index, uint32_t delta_time, uint32_t *beatPeriod);

#endif



