/*
 * commands.h
 *
 *  Created on: 7. 1. 2019
 *      Author: Adam
 */

#ifndef COMMANDS_H_
#define COMMANDS_H_

#include <stdbool.h>
#include <string.h>
#include <inttypes.h>

#include "main.h"
#include "stm32f4xx_hal.h"

#include "uart.h"

// cmdvel stuff
#define CMDVEL_COMMAND_TIMEOUT (uint32_t)(500) // [ms]
static float cmd_v_x = 0.0;
static float cmd_omega_z = 0.0;
uint32_t t_last_cmdvel;

void commandInit();

bool updateSerialRxCommand(); // Call this periodically (from main loop)

bool _onCmdvelCommand(); // Internal cmdvel command handler
void getCmdvelCommand(float *v_x_dest, float *omega_z_dest);
uint32_t getLastCmdvelTime();
void cmdvelWatchdogUpdate();



#endif /* COMMANDS_H_ */
