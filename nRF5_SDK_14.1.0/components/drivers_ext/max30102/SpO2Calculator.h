#ifndef __SPO2CALCULATOR_H__
#define __SPO2CALCULATOR_H__

#include <stdint.h>
#include <stdio.h>

#define CALCULATE_EVERY_N_BEATS         3

typedef struct
{
		uint32_t samplesRecorded;
    float irACValueSqSum;
    float redACValueSqSum;
    uint8_t beatsDetectedNum;
    uint8_t spO2;
}spo2_data_t;


void spO2Calculator_init(spo2_data_t *p_spo2_data);
uint8_t spO2Calculator_update(spo2_data_t *p_spo2_data, float irACValue, float redACValue, uint8_t beatDetected);
void spO2Calculator_reset(spo2_data_t *p_spo2_data);


#endif


