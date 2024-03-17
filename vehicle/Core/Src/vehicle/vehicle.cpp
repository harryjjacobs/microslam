/*
 * vehicle.cpp
 *
 *  Created on: Mar 5, 2023
 *      Author: harryjjacobs
 */

/*
 * Helpful links I used:
 * https://community.st.com/s/article/using-timers-to-trigger-adc-conversions-periodically
 */

#include <stdio.h>

#include "vehicle/vehicle.h"

#include "vehicle/pwm.h"
#include "vehicle/adc.h"

//PWM motor1_pwm(htim1, TIM_CHANNEL_1);
ADC motor1_back_emf_adc(hadc2);

void vehicle_init() {
//  motor1_pwm.init();

  motor1_back_emf_adc.init();

  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  // PWM to trigger ADC
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);

  // PWM calculations:
  // freq = clock_freq / ARR (Auto Load Register (counter period))
  // duty % = CCR (capture compare register) / ARR
  // clock_freq = 1MHz
  // ARR = 100 - 1
  // PWM freq = 10KHz

  // Therefore: 50% duty cycle = ccr / ARR = ccr / 100
  //   ccr = 50 for 50% duty cycle @ 10 KHz

  // set duty cycle
  htim1.Instance->CCR1 = 100;
  // set duty cycle for pwm pin that triggers ADC
  htim1.Instance->CCR2 = 100;

  HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
}

#define SAMPLES 1000

void vehicle_update() {
  uint32_t adc = motor1_back_emf_adc.poll();
//  data[i] = adc;
  HAL_Delay(200);
u}
