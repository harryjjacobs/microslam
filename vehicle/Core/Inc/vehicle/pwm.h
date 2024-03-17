/*
 * pwm.h
 *
 *  Created on: Mar 5, 2023
 *      Author: harryjjacobs
 */

#ifndef INC_PWM_H_
#define INC_PWM_H_

#include "stm32g4xx_hal.h"

class PWM {
public:
  PWM(TIM_HandleTypeDef &htim, int channel) :
      htim(htim), channel(channel) {
  }

  void init() {
    HAL_TIM_PWM_Start(&htim, channel);

    // PWM calculations:
    // freq = clock_freq / ARR (Auto Load Register (counter period))
    // duty % = CCR (capture compare register) / ARR
    // clock_freq = 1MHz
    // ARR = 100 - 1
    // PWM freq = 10KHz

    // Therefore: 50% duty cycle = ccr / ARR = ccr / 100
    //   ccr = 50 for 50% duty cycle @ 10 KHz

    // set duty cycle
    htim.Instance->CCR1 = 50;
    // set duty cycle for pwm pin that triggers ADC
    htim.Instance->CCR2 = 50;
  }

private:
  TIM_HandleTypeDef &htim;
  int channel;
};

//void pwm_init(TIM_HandleTypeDef &htim, int channel);

#endif /* INC_PWM_H_ */
