/*
 * adc.h
 *
 *  Created on: Mar 5, 2023
 *      Author: harryjjacobs
 */

#ifndef INC_VEHICLE_ADC_H_
#define INC_VEHICLE_ADC_H_

#include "stm32g4xx_hal.h"

class ADC {
public:
  ADC(ADC_HandleTypeDef &hadc) :
      hadc(hadc) {
  }

  void init() {
    // https://wiki.st.com/stm32mcu/wiki/Getting_started_with_ADC
    HAL_ADCEx_Calibration_Start(&hadc, ADC_SINGLE_ENDED);
    HAL_ADC_Start_IT(&hadc);
    //  HAL_ADC_Start_DMA(&hadc, (uint32_t*) &value_adc, 1);
  }

  uint32_t poll() {
//    HAL_ADC_Start(&hadc);
    HAL_ADC_PollForConversion(&hadc, 10);
    value_adc = HAL_ADC_GetValue(&hadc);
    //  printf("DMA: %d \t Poll: %d\n",(int) ADC_buf, (int)ADC_poll);
//    HAL_ADC_Stop(&hadc);
    return value_adc;
  }

  uint32_t value() {
    return value_adc;
  }

private:
  uint32_t value_adc;
  ADC_HandleTypeDef &hadc;
};

#endif /* INC_VEHICLE_ADC_H_ */
