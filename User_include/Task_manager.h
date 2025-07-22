//############################################################
// FILE:  Task_manager_H
// Created on: 2018��1��2��
// Author: XY
// summary: Header file  and definition
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//��Ȩ���У�����ؾ�
//DSP28335����PMSM��������뿪����
//˶������
//��ַ: https://shuolidianzi.taobao.com
//�޸�����:2017/7/5
//�汾��V17.2-1
//Author-QQ: 616264123
//�������QQȺ��736369775
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
