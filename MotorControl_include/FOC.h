/*
 * FOC.h
 *
 *  Created on: 2024Äê11ÔÂ20ÈÕ
 *      Author: HP
 */

#ifndef MOTORCONTROL_INCLUDE_FOC_H_
#define MOTORCONTROL_INCLUDE_FOC_H_
typedef struct{
                float32_t Desired_Speed_RPM;
}MOTOR_Vars_t;

typedef enum{
	           MODE_FORWARD=1,
			   MODE_REVERSE,
			   MODE_FOEWARD_REVERSE,
			   MODE_HOMING
}MOTORMODE;
extern MOTORMODE currentMode;
#define MOTOR_Vars_t_DEAFAULT { 0,\
}
extern MOTOR_Vars_t gMotorVars;

#endif /* MOTORCONTROL_INCLUDE_FOC_H_ */
