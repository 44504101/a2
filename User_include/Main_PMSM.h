//############################################################
// FILE:  Main_PMSM.h
// Created on: 2017��12��10��
// Author: XQ
// summary: Header file  and definition
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//��Ȩ���У�����ؾ�
//DSP28335����PMSM��������뿪����
//˶������
//��ַ: https://shuolidianzi.taobao.com
//�޸�����:2017/9/8
//�汾��V16_3
//Author-QQ: 616264123
//�������QQȺ��736369775
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
