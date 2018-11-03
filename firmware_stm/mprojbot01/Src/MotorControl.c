/*
 * MotorControl.cpp
 *
 *  Created on: 3. 11. 2018
 *      Author: Adam
 */

#include "MotorControl.h"

void initMotors(){
	if (HAL_TIM_Base_Start(&htim3) != HAL_OK)	{
		_Error_Handler(__FILE__, __LINE__);
	}
	if (HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1 | TIM_CHANNEL_2) != HAL_OK)	{
		_Error_Handler(__FILE__, __LINE__);
	}
}

void setMotor(uint8_t motor, float speed){
	if (speed > 1.0f)
		speed = 1.0f;
	else if (speed < -1.0f)
		speed = -1.0f;

	if (speed >= 0){
		setMotorPWM(motor, DIR_FWD, speed*UINT16_MAX);
	}
	else {
		setMotorPWM(motor, DIR_REV, -1*speed*UINT16_MAX);
	}
}

void setMotorPWM(uint8_t motor, uint8_t direction, uint32_t pwm){
	if (pwm > UINT16_MAX)
		pwm = UINT16_MAX;

	TIM_OC_InitTypeDef sConfigOC;
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;

	if (direction == DIR_FWD){
		sConfigOC.Pulse = pwm;
		if (motor == MOTOR_L)
			HAL_GPIO_WritePin(M1_DIR_GPIO_Port, M1_DIR_Pin, GPIO_PIN_SET);
		else
			HAL_GPIO_WritePin(M2_DIR_GPIO_Port, M2_DIR_Pin, GPIO_PIN_SET);
	}
	else {
		sConfigOC.Pulse = 65535 - pwm;
		if (motor == MOTOR_L)
			HAL_GPIO_WritePin(M1_DIR_GPIO_Port, M1_DIR_Pin, GPIO_PIN_RESET);
		else
			HAL_GPIO_WritePin(M2_DIR_GPIO_Port, M2_DIR_Pin, GPIO_PIN_RESET);
	}

	uint32_t channel = TIM_CHANNEL_1;
	if (motor == MOTOR_L)
		channel = TIM_CHANNEL_1;
	else
		channel = TIM_CHANNEL_2;

	if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, channel) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}
}

