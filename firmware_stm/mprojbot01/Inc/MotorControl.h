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

//#include "main.h"
#include "math.h"
#include "uart.h"

#define MOTOR_L 0
#define MOTOR_R 1
#define DIR_FWD 0
#define DIR_REV 1

#define PWM_MAX 4096

extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim5;

volatile uint32_t encL;
volatile uint32_t encR;
volatile uint32_t encLPrev;
volatile uint32_t encRPrev;

static float motorKp = 0.2;
static float motorKi = 0.025;
static float motorKd = 0.0;
static float enc2omega = (2*M_PI)/(4*8*100);
float motorIntegratorL;
float motorIntegratorR;
float motorSetpointL; // motor setpoint [rad/s]
float motorSetpointR;
float motorEL;
float motorER;
float motorEPrevL;
float motorEPrevR;
uint32_t motorCtrlLastUpdate;
uint32_t motorCtrlCounter;


void initMotors();
void initEncoders();

void setMotor(uint8_t motor, float speed);
void setMotorPWM(uint8_t motor, uint8_t direction, uint32_t pwm);

void motorCtrlInit();
void motorCtrlUpdate();
void motorCtrlSetpoint(float omega_l, float omega_r);

#endif /* MOTORCONTROL_H_ */
