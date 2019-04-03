#ifndef __PEDOMETER_H__
#define __PEDOMETER_H__

#include <stdint.h>
#include "a2_typedef.h"


#define DEGAULT_GENDER		1   // 1 MALE, 2 FEMALE  
#define DEFAULT_WEIGHT		60  // 60KG
#define DEFAULT_HEIGHT		170 // cm

#define MIN_WEIGHT				20  // 20  KG
#define MIN_HEIGHT				100 // 100 cm
#define MAX_WEIGHT				200 // 200 KG
#define MAX_HEIGHT				240 // 240 cm

#define GENDER_MALE_FACTOR						0.415
#define GENDER_FEMALE_FACTOR					0.413


#define STEP_RATE_FACTOR_VERY_SLOW		0.88   // (Steps/sec) when S < 1.6
#define STEP_RATE_FACTOR_SLOW					0.95   // (Steps/sec) when 1.6  <= S < 1.8
#define STEP_RATE_FACTOR_NORMAL       1.00   // (Steps/sec) when 1.8  <= S < 2.35
#define STEP_RATE_FACTOR_FAST         1.30   // (Steps/sec) when 2.35 <= S < 2.8
#define STEP_RATE_FACTOR_VERY_FAST    2.30   // (Steps/sec) when S >= 2.8


#define METABOLIC_FACTOR_VERY_SLOW		2.0   // (Steps/sec) when S < 1.6
#define METABOLIC_FACTOR_SLOW					2.5   // (Steps/sec) when 1.6  <= S < 1.8
#define METABOLIC_FACTOR_NORMAL       3.8   // (Steps/sec) when 1.8  <= S < 2.35
#define METABOLIC_FACTOR_FAST         8.0   // (Steps/sec) when 2.35 <= S < 2.8
#define METABOLIC_FACTOR_VERY_FAST    12.5   // (Steps/sec) when S >= 2.8


typedef enum
{	
	REST,     // 0
	WALKING,  // 1
	JOGGING,  // 2
	RUNNING,  // 3
}activity_level_t;
	
typedef struct{
	  uint8_t gender;
	  uint16_t weight;
		uint16_t height;
		double gender_factor;
	  double step_rate;
		double stride;
		double bsl;
		double step_rate_factor;
		double distance;
		uint32_t time;
		double speed_m_per_s;
	  double speed_km_per_h;
		uint8_t activity_level;
		double metabolic_factor;
		double calories;
}pedometer_data_t;

/**
 * @brief Function for initial pedemeter data
 *
 * @param[in]   p_pedo_data       pointer struct for return pedometer data.
 */
void pedometer_setup(pedometer_data_t *p_pedo_data, uint8_t gender, uint16_t weight, uint16_t height);

/**
 * @brief Function for calulation pedometer data
 *
 * @param[in]   p_pedo_data       pointer struct for return pedometer data.
 */
void pedometer_processing(pedometer_data_t *p_pedo_data, uint32_t period_time, uint16_t period_step_count);

#endif
