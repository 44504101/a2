//############################################################
// FILE:  Task_manager_H
// Created on: 2018年1月2日
// Author: XY
// summary: Header file  and definition
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//版权所有，盗版必究
//DSP28335旋变PMSM电机控制与开发板
//硕历电子
//网址: https://shuolidianzi.taobao.com
//修改日期:2017/7/5
//版本：V17.2-1
//Author-QQ: 616264123
//电机控制QQ群：736369775
//############################################################


#ifndef _Task_manager_H_
#define _Task_manager_H_


typedef  void ( *FTimer_P) ( void );

typedef struct {
	    Uint16	 Task_Period;  			//time out for calling function
	    Uint16 	 Task_Count;
	    FTimer_P Task_Function;		//Send function defines in application
	   }TaskTime;

void Timer_Task_Count(void);
void Execute_Task_List_RUN(void);
void Task_Manage_List_Init(void);
void HF_Data(void);
void Task_LED(void);
void Deal_Fault(void);
void CAN_Recovery_Fault(void);
void task_send_Rece(void);

#endif  //Task_manager_H
