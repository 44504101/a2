/*
 * API.c
 *
 *  Created on: 2025Äê5ÔÂ7ÈÕ
 *      Author: HP
 */
#include"API.h"
extern MOTOR_Vars_t gMotorVars;
extern RESOLVER resolver1;
void Motor_Start(){
     EnableFlag = true;
}
void Motor_Stop(){
     EnableFlag = false;
}

void Motor_SetSpeed(uint16_t rpm){
    gMotorVars.Desired_Speed_RPM = (float32_t)rpm;
}

void Motor_Set_Run_Trun(uint16_t truns){
    resolver1.TargetRotation = (float32_t)truns*29;
}
