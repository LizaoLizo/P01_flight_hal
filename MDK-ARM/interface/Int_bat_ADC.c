#include "Int_bat_ADC.h"

void Int_bat_ADC_Init(void)
{
    HAL_GPIO_WritePin(BAT_ADC_EN_GPIO_Port, BAT_ADC_EN_Pin, GPIO_PIN_RESET);

    HAL_ADC_Start(&hadc1);
}

float Int_bat_ADC_Read(void)
{
    uint32_t adc_value = HAL_ADC_GetValue(&hadc1);

    float voltage =
        ((float)adc_value * 3.3f / 4095.0f) * 2.0f;

    return voltage;
}