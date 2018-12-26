/*
 * ultrasonic.c
 *
 *  Created on: 26. 12. 2018
 *      Author: Adam
 */

#include "ultrasonic.h"

void ultrasonicInit(){
	t_lastUSMeasure = 0;
	_usMeasurePhase = IDLE;
	_ultrasonicDistance = -1;
	HAL_TIM_Base_Start(&htim2);
}

void ultrasonicStartMeasure(){
	uint32_t t = HAL_GetTick();
	if (t-t_lastUSMeasure < USminMeasurePeriod)
		return;

	if(_usMeasurePhase == IDLE){
		t_lastUSMeasure = t;
		HAL_GPIO_WritePin(US_TRIGGER_GPIO_Port, US_TRIGGER_Pin, 1);
		for(TIM2->CNT = 0; TIM2->CNT < 100;); // delay 1us
		HAL_GPIO_WritePin(US_TRIGGER_GPIO_Port, US_TRIGGER_Pin, 0);
		_usMeasurePhase = WAITING_START;
	}
}

float ultrasonicGetDist(){
	return _ultrasonicDistance;
}

void ultrasonicIRQHandler(uint16_t GPIO_Pin){
	/* EXTI line interrupt detected */
	if(!__HAL_GPIO_EXTI_GET_FLAG(GPIO_Pin)) {
		return;
	}

	if(_usMeasurePhase == WAITING_START){
		TIM2->CNT = 0;
		_usMeasurePhase = WAITING_STOP;
	}
	else if(_usMeasurePhase == WAITING_STOP){
		uint32_t tDist = TIM2->CNT;
		_ultrasonicDistance = (float)tDist/58000.0; //10*5800
		if(_ultrasonicDistance > 4.0)
			_ultrasonicDistance = -1; // Large values are invalid
		_usMeasurePhase = IDLE;
	}
}
