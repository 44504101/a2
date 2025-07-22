//############################################################
// FILE:  Main_PMSM.h
// Created on: 2017年12月10日
// Author: XQ
// summary: Header file  and definition
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//版权所有，盗版必究
//DSP28335旋变PMSM电机控制与开发板
//硕历电子
//网址: https://shuolidianzi.taobao.com
//修改日期:2017/9/8
//版本：V16_3
//Author-QQ: 616264123
//电机控制QQ群：736369775
//############################################################

#ifndef Main_PMSM_H
#define Main_PMSM_H

#include "stdbool.h"
#include "Main_PMSM.h"
#include "Motorcontrol_include.h"
#include "Task_manager.h"
#include "User_CAN.h"
#include"f2833xpwm.h"
#include"f2833xpwmdac.h"
#include"RS485.h"
/*#define MOTOR_Vars_INIT {false,\
	                     false,\
	                     0,\
	                     0,\
	                     0,\
	                     0.0,\
	                     0}

typedef struct
{
	bool Flag_Startcircle;
	bool Is_Control_enable;
	Uint16 Revolutions_Ref;
	Uint16 Revolutions;
	//Uint16 Previous_PM_angle;
	Uint16 PM_angle;
    float Over_flowcount;
    float32_t Desired_Speed_RPM;
}MOTOR_Vars_t;
extern MOTOR_Vars_t gMotorVars;*/
extern volatile Uint16 EnableFlag;
//volatile Uint16 EnableFlag = false;
#endif  // end of Main_PMSM_Resolver.h definition

//===========================================================================
// End of file.
//===========================================================================
