/*
 * commands.c
 *
 *  Created on: 7. 1. 2019
 *      Author: Adam
 */

#include "commands.h"

void commandInit(){
	t_last_cmdvel = HAL_GetTick();
}

bool updateSerialRxCommand() {
	if(uartLineAvailable()) {
		// So far we only have the cmdvel command
		// Later you can check for other message types e.g.:
		// if(UART2LineBuff[0]=='X') {
		//   _someHandlerFunction();
		// }
		// else {
		return _onCmdvelCommand();
		// }
	}
	else {
		return false;
	}
}

bool _onCmdvelCommand() {
	float tmp_vx = 0.0;
	float tmp_omega = 0.0;
	if(2 == sscanf(UART2LineBuff, "%f,%f", &tmp_vx, &tmp_omega)) {
		cmd_v_x = tmp_vx;
		cmd_omega_z = tmp_omega;
		t_last_cmdvel = HAL_GetTick();
//		uartPrintf("cmdvel %f %f\r\n", cmd_v_x, cmd_omega_z);
		return true;
	}
	else {
		return false;
	}
}

void getCmdvelCommand(float *v_x_dest, float *omega_z_dest) {
	*v_x_dest = cmd_v_x;
	*omega_z_dest = cmd_omega_z;
}

uint32_t getLastCmdvelTime() {
	return t_last_cmdvel;
}

void cmdvelWatchdogUpdate(){
	if(HAL_GetTick() - t_last_cmdvel < CMDVEL_COMMAND_TIMEOUT) {
		enableMotors(true);
	}
	else {
		enableMotors(false);
	}
}
