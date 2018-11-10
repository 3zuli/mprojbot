/*
 * MotorControl.h
 *
 *  Created on: 3. 11. 2018
 *      Author: Adam
 */

#ifndef MOTORCONTROL_H_
#define MOTORCONTROL_H_

#include "inttypes.h"
#include "stm32f4xx_hal.h"

#include "main.h"

#define MOTOR_L 0
#define MOTOR_R 1
#define DIR_FWD 0
#define DIR_REV 1

#define PWM_MAX 4096

extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim5;

void initMotors();
void initEncoders();

void setMotor(uint8_t motor, float speed);
void setMotorPWM(uint8_t motor, uint8_t direction, uint32_t pwm);

#endif /* MOTORCONTROL_H_ */
