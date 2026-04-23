#include "sensor.h"
#include "adc.h"

uint16_t adc_value_adc1[4];
uint16_t adc_value_adc2[4];

#define FILTER_N 10

uint16_t adc_buf[8][FILTER_N];
float last_distance[8] = {0};

uint8_t adc_index = 0;

volatile uint8_t adc1_done = 0;
volatile uint8_t adc2_done = 0;

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	// ADC1 当前4通道
    if(hadc->Instance == ADC1)
    {
        for(int i = 0; i < 4; i++)
        {
            adc_buf[i][adc_index] = adc_value_adc1[i];
        }

        adc1_done = 1;
    }

    // ADC2 当前4通道
    else if(hadc->Instance == ADC2)
    {
        for(int i = 0; i < 4; i++)
        {
            adc_buf[i + 4][adc_index] = adc_value_adc2[i];
        }

        adc2_done = 1;
    }

    // 同步完成
    if(adc1_done && adc2_done)
    {
        adc1_done = 0;
        adc2_done = 0;

        adc_index++;
        if(adc_index >= FILTER_N) adc_index = 0;
    }
}

float get_distance(uint8_t channel)
{
    uint32_t sum = 0;

    // 求平均滤波
    for(int i = 0; i < FILTER_N; i++)
    {
        sum += adc_buf[channel][i];
    }

    uint16_t adc_avg = sum / FILTER_N;

    // 转换电压
    float voltage = adc_avg * 3.3f / 4096.0f;

    // 异常保护防止除零
    if(voltage < 0.4f) voltage = 0.4f;

    // 转换距离
    //float distance = 27.86f / (voltage - 0.1f);
	float distance = (35.247f / voltage) - 4.421;
		
	// 滤波
	distance = 0.7f * last_distance[channel] + 0.3f * distance;
    last_distance[channel] = distance;

    // 限定距离为10~80cm
    if(distance > 80.0f) distance = 80.0f;
    if(distance < 5.0f) distance = 5.0f;
		
    return distance;
}