//############################################################
// FILE:  ADC_int.h
// Created on: 2017��12��10��
// Author: XQ
// summary: Header file  and definition
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//��Ȩ���У�����ؾ�
//DSP28335����ͬ��������ƿ�����
//˶������
//��ַ: https://shuolidianzi.taobao.com
//Author-QQ: 616264123
//�������QQȺ��736369775
//############################################################

#ifndef ADC_int_H
#define ADC_int_H
#include "DSP2833x_Project.h"


extern void ADC_cal(); // У׼ADC��ת�����
void   ADC_MACRO_INIT( Uint16  *ChSel_X,  Uint16 *Trigsel_X,  Uint16  *ACQPS_X); // ADC��ͨ���ͽ���Ĵ�������
void   ADC_SOC_int(void ); //ADC��ʼ������
#endif  // end of ADC_int_H definition

//===========================================================================
// End of file.
//===========================================================================
