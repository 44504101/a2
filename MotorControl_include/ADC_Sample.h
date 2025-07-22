//############################################################
// FILE:  ADC_Sample.h
// Created on: 2016��8��5��
// Author: XQ
// summary: Header file  and definition
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//��Ȩ���У�����ؾ�
//DSP28335����PMSM��������뿪����
//˶������
//��ַ: https://shuolidianzi.taobao.com
//Author-QQ: 616264123
//�������QQȺ��736369775
//############################################################

#ifndef _ADC_Sample_H
#define _ADC_Sample_H

typedef struct {
	     Uint16   ADC_test1 ;     // ĸ�ߵ��� DC Bus  Current
	     Uint16   ADC_test2 ;     // ĸ�ߵ��� DC Bus  Current
	     Uint16   BUS_Curr ;     // ĸ�ߵ��� DC Bus  Current
	     Uint16   PhaseU_Curr;   // U����� Phase U Current
	     Uint16   PhaseV_Curr;   // V�����Phase V Current
	     Uint16   PhaseW_Curr;   // V�����Phase W Current
	     Uint16   BUS_Voltage ;  //ĸ�ߵ�ѹDC Bus  Voltage
	     Uint16   RP_speed_Voltage ;   // ��λ����ѹ RP1_Voltage
	     Uint16   OffsetBUS_Curr ;     // ĸ�ߵ���ƫִֵ DC Bus  Current
	     Uint16   OffsetPhaseU_Curr;   // U�����ƫִֵ  Phase U Current
	     Uint16   OffsetPhaseV_Curr;   // V�����ƫִֵ Phase V Current
	     Uint16   OffsetPhaseW_Curr;   // V�����ƫִֵ Phase W Current
       }ADCSamp;

#define  ADCSamp_DEFAULTS  {0,0,0,0,0,0,0,0,0,0,0,0}   // ��ʼ������

typedef struct {
  float32   BUS_Curr ;     // ĸ�ߵ��� DC Bus  Current
  float32   PhaseU_Curr;   // U����� Phase U Current
  float32   PhaseV_Curr;   // V�����Phase V Current
  float32   PhaseW_Curr;   // V�����Phase W Current
  float32   BUS_Voltage ;  //ĸ�ߵ�ѹDC Bus  Voltage
  }Volt_Curr;

#define  Volt_Curr_DEFAULTS  {0,0,0,0,0}   // ��ʼ������

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
