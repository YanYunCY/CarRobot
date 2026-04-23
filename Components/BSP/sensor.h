#ifndef __SENSOR_H
#define __SENSOR_H

#include "stdint.h"

extern uint16_t adc_value_adc1[4];
extern uint16_t adc_value_adc2[4];

void sensor_init(void);
float get_distance(uint8_t ch);

#endif