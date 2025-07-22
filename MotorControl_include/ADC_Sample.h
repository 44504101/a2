//############################################################
// FILE:  ADC_Sample.h
// Created on: 2016年8月5日
// Author: XQ
// summary: Header file  and definition
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//版权所有，盗版必究
//DSP28335旋变PMSM电机控制与开发板
//硕历电子
//网址: https://shuolidianzi.taobao.com
//Author-QQ: 616264123
//电机控制QQ群：736369775
//############################################################

#ifndef _ADC_Sample_H
#define _ADC_Sample_H

typedef struct {
	     Uint16   ADC_test1 ;     // 母线电流 DC Bus  Current
	     Uint16   ADC_test2 ;     // 母线电流 DC Bus  Current
	     Uint16   BUS_Curr ;     // 母线电流 DC Bus  Current
	     Uint16   PhaseU_Curr;   // U相电流 Phase U Current
	     Uint16   PhaseV_Curr;   // V相电流Phase V Current
	     Uint16   PhaseW_Curr;   // V相电流Phase W Current
	     Uint16   BUS_Voltage ;  //母线电压DC Bus  Voltage
	     Uint16   RP_speed_Voltage ;   // 电位器电压 RP1_Voltage
	     Uint16   OffsetBUS_Curr ;     // 母线电流偏执值 DC Bus  Current
	     Uint16   OffsetPhaseU_Curr;   // U相电流偏执值  Phase U Current
	     Uint16   OffsetPhaseV_Curr;   // V相电流偏执值 Phase V Current
	     Uint16   OffsetPhaseW_Curr;   // V相电流偏执值 Phase W Current
       }ADCSamp;

#define  ADCSamp_DEFAULTS  {0,0,0,0,0,0,0,0,0,0,0,0}   // 初始化参数

typedef struct {
  float32   BUS_Curr ;     // 母线电流 DC Bus  Current
  float32   PhaseU_Curr;   // U相电流 Phase U Current
  float32   PhaseV_Curr;   // V相电流Phase V Current
  float32   PhaseW_Curr;   // V相电流Phase W Current
  float32   BUS_Voltage ;  //母线电压DC Bus  Voltage
  }Volt_Curr;

#define  Volt_Curr_DEFAULTS  {0,0,0,0,0}   // 初始化参数

extern Volt_Curr Volt_CurrPara;
extern ADCSamp   ADCSampPara;

void ADC_Sample(void);
void ADC_Sample_deal(void);
void Offset_CurrentReading(void);
void DWQ_Control_AD(void);
#endif /* ADC_Sample */
//===========================================================================
// End of file.
//===========================================================================
