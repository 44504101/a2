//############################################################
// FILE:  Main_PMSM_Resolver.h
// Created on:
// Author: XQ
// summary: Header file  and definition
//############################################################

#ifndef MotorControl_ISR_H
#define MotorControl_ISR_H


#include "Motorcontrol_include.h"
#include"RS485.h"
#define LEVEL1  1      		// Module check out (do not connect the motors)
#define LEVEL2  2           // Verify ADC, park/clarke, calibrate the offset
#define LEVEL3  3           // Verify closed current(torque) loop, QEP and speed meas.
#define LEVEL4  4           // Verify close speed loop and speed PID
#define LEVEL5  5
#define BUILDLEVEL LEVEL4
void  Svpwm_Outpwm(void);
interrupt void MainISR(void);
interrupt void OffsetISR(void);
interrupt void Scib_rx_ISR(void);
void SwitchOffPWMAndSetRunningMotorModeToStop(void);

extern volatile bool brake_hold;
extern volatile bool pos_protect;
extern volatile bool over_current;
extern volatile bool sys_reset;
extern volatile bool motor_run;
extern volatile bool dir_fwd;
static inline uint8_t build_status_byte(void);
#endif  // end of Main_PMSM_Resolver.h definition

//===========================================================================
// End of file.
//===========================================================================
