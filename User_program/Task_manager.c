//############################################################
// FILE:  Task_manager.c
// Created on: 2017年12月10日
// Author: XQ
// summary: Task_manager
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//版权所有，盗版必究
//DSP28335旋变PMSM电机控制与开发板
//硕历电子
//网址: https://shuolidianzi.taobao.com
//Author-QQ: 616264123
//电机控制QQ群：736369775
//############################################################

#include "Main_PMSM.h"

#define Task_Num   4   //task +1

#define HFPeriod_COUNT    10    // 1ms
#define CANPeriod_COUNT   200    //20ms
#define PPeriod_COUNT     1000   //100ms
#define LEDPeriod_COUNT   5000   //500ms

TaskTime  TasksPare[Task_Num];

void Timer_Task_Count(void)
{
 Uint16 Task_Count=0;

 for(Task_Count=0;Task_Count<Task_Num;Task_Count++)    //TASK_NUM=5
 {
  if((TasksPare[Task_Count].Task_Count<TasksPare[Task_Count].Task_Period)&&(TasksPare[Task_Count].Task_Period>0))
  {
	  TasksPare[Task_Count].Task_Count++;
  }
 }
}

void Execute_Task_List_RUN(void)
{
 Uint16 Task_Count=0;

 for(Task_Count=0;Task_Count < Task_Num;Task_Count++)
 {
  if((TasksPare[Task_Count].Task_Count>=TasksPare[Task_Count].Task_Period)&&(TasksPare[Task_Count].Task_Period > 0))
  {
	  TasksPare[Task_Count].Task_Function();
	  TasksPare[Task_Count].Task_Count=0;
  }
 }
}


void HF_Data(void) // 高频数据发送  预留电压电流等参数发送查询问题
{

}


void Ckeck_BoMa(void) // 拨码开关等于1:时候高电平是电位器控制模式 ；0：CAN通讯发送指令数据
{
  CAN_ControlPara.BoMa_Control_Flag =  GPIO_BMKG;  // // 拨码开关等于1:时候高电平是电位器控制模式 ；0：CAN通讯发送指令数据

}

void Deal_Fault(void)
{
  Ckeck_BoMa( );
}

void Task_LED(void)  //  最小系统个500ms的LED的闪烁
{
	LED2_REV;   //  500ms的LED的闪烁
	if( AD2SXBPare.AD2S_FaultDOSLOT==3)  // 旋变DOSLOT无故障时 ，系统板上第一个LED亮
	LED1_CLR;
	else
	LED1_REV;   // 旋变DOSLOT故障时，系统板上第一个LED闪烁
}

void task_send_Rece(void)
{
    ECANB_MB1_send();
    ECANB_MB2_send();
    ECANB_MB3_send();
    ECANB_MB4_send();
    ECANB_MB5_send();
    CAN_Rcv1( );   // CAN 5组数据从上位机接收
    CAN_Rcv2( );

    StartControl_Mode();
    SCI_RS232TX_sen( ); // 串口测试
 // TOGGLEIO_P6_0;
}


void Task_Manage_List_Init(void)
{
	TasksPare[0].Task_Period=HFPeriod_COUNT; //PERIOD_COUNT=20;2ms
	TasksPare[0].Task_Count=2;
	TasksPare[0].Task_Function=HF_Data;

	TasksPare[1].Task_Period=CANPeriod_COUNT; //20ms
	TasksPare[1].Task_Count=105;
	TasksPare[1].Task_Function=task_send_Rece; // CAN故障处理

	TasksPare[2].Task_Period=PPeriod_COUNT; //100ms
	TasksPare[2].Task_Count=531;
	TasksPare[2].Task_Function=Deal_Fault ;// 通讯接收和发送;

	TasksPare[3].Task_Period = LEDPeriod_COUNT;//500ms
	TasksPare[3].Task_Count=1110;
	TasksPare[3].Task_Function = Task_LED;
}


// USER CODE END
