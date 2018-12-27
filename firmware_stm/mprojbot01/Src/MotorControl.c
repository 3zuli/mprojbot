/*
 * MotorControl.cpp
 *
 *  Created on: 3. 11. 2018
 *      Author: Adam
 */

#include "MotorControl.h"

//float motorKp = 1.0;
//float motorKi = 0.0;
//float motorKd = 0.0;

void initMotors(){

}

void initEncoders(){
	TIM4->CNT = 0;
	TIM5->CNT = 0;
	HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&htim5, TIM_CHANNEL_ALL);
}

void initOdom(){
	setOdom(0, 0, 0);
}

void enableMotors(bool enable){
	motorsEnabled = enable;
}

void setMotor(uint8_t motor, float speed){
	// speed <-1;1> = <-20;20> rad/s
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

void motorCtrlInit(){
	motorsEnabled = false;
	encL = TIM4->CNT;
	encR = TIM5->CNT;
	encLPrev = encL;
	encRPrev = encR;
	motorIntegratorL = 0;
	motorIntegratorR = 0;
	motorSetpointL = 0;
	motorSetpointR = 0;
	motorEL = 0;
	motorER = 0;
	motorEPrevL = 0;
	motorEPrevR = 0;
	motorCtrlLastUpdate = HAL_GetTick();
	motorCtrlCounter = 0;
}

void motorCtrlUpdate(){
	uint32_t t = HAL_GetTick();
	// Rate limit
	if((t - motorCtrlLastUpdate)<motorCtrlRateDt){
		return;
	}

	// Calculate & limit delta time
	float dt = (t - motorCtrlLastUpdate)*0.001;
	if(dt>0.1)
		dt = 0.1;
	else if(dt<0.001) // avoid div 0
		dt = 0.001;

	encLPrev = encL;
	encRPrev = encR;
	encL = TIM4->CNT;
	encR = TIM5->CNT;
	int32_t dencL = encLPrev - encL;
	int32_t dencR = encRPrev - encR;
	if(dencR < -40000)  // Handle encoder counter overflow
		dencR = 65535 + dencR;
	else if(dencR > 40000)
		dencR = 65535 - dencR;
	if(dencL < -40000)
		dencL = 65535 + dencL;
	else if(dencL > 40000)
		dencL = 65535 - dencL;

	volatile float speedL = -1.0*(float)dencL*enc2omega/dt;
	volatile float speedR =  1.0*(float)dencR*enc2omega/dt;

	odomUpdate(speedL, speedR, dt);

	motorEPrevL = motorEL;
	motorEPrevR = motorER;
	// Error
	motorEL = motorSetpointL - speedL;
	motorER = motorSetpointR - speedR;
	// Derivative
	float dEL = (motorEPrevL - motorEL)/dt;
	float dER = (motorEPrevR - motorER)/dt;
	// Integral + integrator clamping
	motorIntegratorL += motorKi * motorEL * dt;
	motorIntegratorR += motorKi * motorER * dt;
	if(motorIntegratorL > motorIntegratorMax)
		motorIntegratorL = motorIntegratorMax;
	else if(motorIntegratorL < -motorIntegratorMax)
		motorIntegratorL = -motorIntegratorMax;
	if(motorIntegratorR > motorIntegratorMax)
		motorIntegratorR = motorIntegratorMax;
	else if(motorIntegratorR < -motorIntegratorMax)
		motorIntegratorR = -motorIntegratorMax;
	// PID magic
	// Calculate control effort u=<-1;1>
	float uL = motorKp*motorEL + motorIntegratorL + motorKd*dEL;
	float uR = motorKp*motorER + motorIntegratorR + motorKd*dER;

	if(motorsEnabled){
		setMotor(MOTOR_L, uL);
		setMotor(MOTOR_R, uR);
	}
	else{
		setMotor(MOTOR_L, 0);
		setMotor(MOTOR_R, 0);
	}

//	if((motorCtrlCounter++) % 10 == 0){
//		uartPrintf("%.4f %.4f %.4f %.3f %.3f\r\n", dt, motorEL, motorER, uL, uR);
//	}
//	uartPrintf("%ld %ld\r\n", dencL, dencR);
	if((motorCtrlCounter++) % 5 == 0){
	//	uartPrintf("%.4f %.4f %.3f\r\n", dt, motorER, uR);
//		uartPrintf("%ld %ld\r\n", encL, encR);
//		uartPrintf("%ld %ld\r\n", dencL, dencR);
//		uartPrintf("%.4f %.4f %.4f %.4f %.3f\r\n", dt, motorER, dER, motorIntegratorR, uR);
//		uartPrintf("%.4f %.4f %.4f %.3f\r\n", speedR, motorER, motorIntegratorR, uR);
//		uartPrintf("%.4f %.4f %.4f %.4f %.4f\r\n", speedL, speedR, odomX, odomY, odomRot);
		uartPrintf("%.4f %.4f %.4f\r\n", odomX, odomY, odomRot);
	}
	motorCtrlLastUpdate = t;
}

void motorCtrlSetpoint(float omega_l, float omega_r){
	motorSetpointL = omega_l*-1;
	motorSetpointR = omega_r;
}

void setOdom(float oX, float oY, float oRot){
	odomX = oX;
	odomY = oY;
	odomRot = oRot;
}

void odomUpdate(float wl, float wr, float dt){
	float v_k = (wl+wr)*wheel_R;
	float odomRotPrev = odomRot;
	odomRot += (wr-wl)*(wheel_R/wheel_d)*dt;
	// TODO: Yaw clamping to +-PI


	if(wl==wr){
		odomX += v_k*cos(odomRot)*dt;
		odomY += v_k*sin(odomRot)*dt;
	}
	else{
		float vl = wl*wheel_R;
		float vr = wr*wheel_R;
		float frac = (wheel_d/2)*((vr+vl)/(vr-vl));
		odomX -= frac*(cos(odomRot)-cos(odomRotPrev));
		odomY += frac*(sin(odomRot)-sin(odomRotPrev));
	}
}
