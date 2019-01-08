/*
 * MotorControl.h
 *
 *  Created on: 3. 11. 2018
 *      Author: Adam
 */

#ifndef MOTORCONTROL_H_
#define MOTORCONTROL_H_

#include "inttypes.h"
#include "stdbool.h"
#include "stm32f4xx_hal.h"

//#include "main.h"
#include "math.h"
#include "uart.h"
#include "commands.h"

#define MOTOR_L 0
#define MOTOR_R 1
#define DIR_FWD 0
#define DIR_REV 1

#define PWM_MAX 4096
#define WHEEL_OMEGA_MAX 20.0f
#define CMDVEL_VX_MAX 0.3f
#define CMDVEL_OMEGA_MAX 3.5

extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim5;

volatile uint32_t encL;
volatile uint32_t encR;
volatile uint32_t encLPrev;
volatile uint32_t encRPrev;

// Control
static int motorCtrlRateDt = 1000/250; // ms / Hz = ms
static float motorKp = 0.3;  //0.2
static float motorKi = 0.55; //0.025
static float motorKd = 0.0005; // 0
static float motorIntegratorMax = 1;
// 7cpr encoder * 4x sampling = 28 pulses / motor rev * 100:1 gear
static float wheelCPR = (4*7*100);
static float enc2omega = (2*M_PI)/(4*7*100);
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
static bool motorsEnabled = false;

// Odom
static float wheel_R = 0.0341/2.0; // wheel radius [m]
static float wheel_d = 0.09425;  //wheel distance [m]
float odomX;
float odomY;
float odomRot;


void initMotors();
void initEncoders();
void initOdom();

void enableMotors(bool enable);
void setMotor(uint8_t motor, float speed);
void setMotorPWM(uint8_t motor, uint8_t direction, uint32_t pwm);

void motorCtrlInit();
void motorCtrlUpdate();
void motorCtrlSetpoint(float omega_l, float omega_r);
void motorCtrlSetpointCmdvel(float v_x, float omega_z);

void setOdom(float oX, float oY, float oRot);
void odomUpdate(float wl, float wr, float dt);

float clamp(float value, float limit);

#endif /* MOTORCONTROL_H_ */
