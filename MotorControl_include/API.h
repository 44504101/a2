/*
 * API.h
 *
 *  Created on: 2025Äê5ÔÂ7ÈÕ
 *      Author: HP
 */

#ifndef MOTORCONTROL_INCLUDE_API_H_
#define MOTORCONTROL_INCLUDE_API_H_

#include "Motorcontrol_include.h"
#include "Main_PMSM.h"

void Motor_Start(void);
void Motor_Stop(void);
void Motor_SetSpeed(uint16_t rpm);
void Motor_Set_Run_Trun(uint16_t truns);


#endif /* MOTORCONTROL_INCLUDE_API_H_ */
