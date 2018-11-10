/*
 * MotorControl.cpp
 *
 *  Created on: 3. 11. 2018
 *      Author: Adam
 */

#include "MotorControl.h"

void initMotors(){

}

void initEncoders(){
	TIM4->CNT = 0;
	TIM5->CNT = 0;
	HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&htim5, TIM_CHANNEL_ALL);
}

void setMotor(uint8_t motor, float speed){
	if (speed > 1.0f)
		speed = 1.0f;
	else if (speed < -1.0f)
		speed = -1.0f;

	if (speed >= 0){
		setMotorPWM(motor, DIR_FWD, speed*PWM_MAX);
	}
	else {
		setMotorPWM(motor, DIR_REV, -1*speed*PWM_MAX);
	}
}

/*
 * MX1508 control table
 * https://forums.parallax.com/discussion/168432/h-bridges-of-madison-county-77-cents
 * 			IN1		IN2
 *  FWD	   1/PWM     0
 *  REV		 0     1/PWM
 * IDLE      0       0
 * BRAKE     1       1
 */
void setMotorPWM(uint8_t motor, uint8_t direction, uint32_t pwm){
	if (pwm > PWM_MAX)
		pwm = PWM_MAX;

	TIM_OC_InitTypeDef sConfigA, sConfigB;
	sConfigA.OCMode = TIM_OCMODE_PWM1;
	sConfigA.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigA.OCFastMode = TIM_OCFAST_DISABLE;
	sConfigB.OCMode = TIM_OCMODE_PWM1;
	sConfigB.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigB.OCFastMode = TIM_OCFAST_DISABLE;

	if (pwm == 0) {
		sConfigA.Pulse = 0;
		sConfigB.Pulse = 0;
	}
	else if (direction == DIR_FWD) {
		sConfigA.Pulse = PWM_MAX-pwm;
		sConfigB.Pulse = PWM_MAX;
	}
	else if (direction == DIR_REV) {
		sConfigA.Pulse = PWM_MAX;
		sConfigB.Pulse = PWM_MAX-pwm;
	}

	uint32_t channelA = TIM_CHANNEL_1;
	uint32_t channelB = TIM_CHANNEL_3;
	if (motor == MOTOR_L){
		channelA = TIM_CHANNEL_1;
		channelB = TIM_CHANNEL_3;
	}
	else {
		// Right motor is reversed
		channelA = TIM_CHANNEL_4;
		channelB = TIM_CHANNEL_2;
	}

	if (   HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigA, channelA) != HAL_OK
		|| HAL_TIM_PWM_Start(&htim3, channelA) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	if (   HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigB, channelB) != HAL_OK
			|| HAL_TIM_PWM_Start(&htim3, channelB) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

}

