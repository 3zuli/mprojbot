/*
 * ultrasonic.h
 *
 * HAL Library for HC-SR05 ultrasonic distance sensor
 * Uses TIM2 in time base mode
 *
 * Requirements:
 *   TIM2 uses the 100MHz main clock as source
 *     htim2.Instance = TIM2;
 *     htim2.Init.Prescaler = 9;
 *     htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
 *     htim2.Init.Period = 0xFFFFFFFF;
 *     htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
 *
 *  Created on: 26. 12. 2018
 *      Author: Adam
 */

#ifndef ULTRASONIC_H_
#define ULTRASONIC_H_

#include "main.h"
#include "stm32f4xx_hal.h"
#include "inttypes.h"

extern TIM_HandleTypeDef htim2;

float _ultrasonicDistance;
static const uint32_t USminMeasurePeriod = 60; // ms
static const uint32_t USfailPeriod = 30; // ms
uint32_t t_lastUSMeasure;

typedef enum {IDLE=0, WAITING_START, WAITING_STOP} US_PHASE;
US_PHASE _usMeasurePhase;

// Init
void ultrasonicInit();

// Call this from main loop to start a measurement
// This function limits the measurement rate to max. 1 measurement every 60ms
void ultrasonicStartMeasure();

// Get last measured distance in meters, -1 if measurement is invalid
float ultrasonicGetDist();

void ultrasonicIRQHandler(uint16_t GPIO_Pin);



#endif /* ULTRASONIC_H_ */
