//############################################################
// FILE:  Task_manager.c
// Created on: 2017��12��10��
// Author: XQ
// summary: Task_manager
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//��Ȩ���У�����ؾ�
//DSP28335����PMSM��������뿪����
//˶������
//��ַ: https://shuolidianzi.taobao.com
//Author-QQ: 616264123
//�������QQȺ��736369775
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


void HF_Data(void) // ��Ƶ���ݷ���  Ԥ����ѹ�����Ȳ������Ͳ�ѯ����
{

}


void Ckeck_BoMa(void) // ���뿪�ص���1:ʱ��ߵ�ƽ�ǵ�λ������ģʽ ��0��CANͨѶ����ָ������
{
  CAN_ControlPara.BoMa_Control_Flag =  GPIO_BMKG;  // // ���뿪�ص���1:ʱ��ߵ�ƽ�ǵ�λ������ģʽ ��0��CANͨѶ����ָ������

}

void Deal_Fault(void)
{
  Ckeck_BoMa( );
}

void Task_LED(void)  //  ��Сϵͳ��500ms��LED����˸
{
	LED2_REV;   //  500ms��LED����˸
	if( AD2SXBPare.AD2S_FaultDOSLOT==3)  // ����DOSLOT�޹���ʱ ��ϵͳ���ϵ�һ��LED��
	LED1_CLR;
	else
	LED1_REV;   // ����DOSLOT����ʱ��ϵͳ���ϵ�һ��LED��˸
}

void task_send_Rece(void)
{
    ECANB_MB1_send();
    ECANB_MB2_send();
    ECANB_MB3_send();
    ECANB_MB4_send();
    ECANB_MB5_send();
    CAN_Rcv1( );   // CAN 5�����ݴ���λ������
    CAN_Rcv2( );

    StartControl_Mode();
    SCI_RS232TX_sen( ); // ���ڲ���
 // TOGGLEIO_P6_0;
}


void Task_Manage_List_Init(void)
{
	TasksPare[0].Task_Period=HFPeriod_COUNT; //PERIOD_COUNT=20;2ms
	TasksPare[0].Task_Count=2;
	TasksPare[0].Task_Function=HF_Data;

	TasksPare[1].Task_Period=CANPeriod_COUNT; //20ms
	TasksPare[1].Task_Count=105;
	TasksPare[1].Task_Function=task_send_Rece; // CAN���ϴ���

	TasksPare[2].Task_Period=PPeriod_COUNT; //100ms
	TasksPare[2].Task_Count=531;
	TasksPare[2].Task_Function=Deal_Fault ;// ͨѶ���պͷ���;

	TasksPare[3].Task_Period = LEDPeriod_COUNT;//500ms
	TasksPare[3].Task_Count=1110;
	TasksPare[3].Task_Function = Task_LED;
}


// USER CODE END
