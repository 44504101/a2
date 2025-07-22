//############################################################
// FILE:  ADC_int.h
// Created on: 2017年12月10日
// Author: XQ
// summary: Header file  and definition
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//版权所有，盗版必究
//DSP28335永磁同步电机控制开发板
//硕历电子
//网址: https://shuolidianzi.taobao.com
//Author-QQ: 616264123
//电机控制QQ群：736369775
//############################################################

#ifndef ADC_int_H
#define ADC_int_H
#include "DSP2833x_Project.h"


extern void ADC_cal(); // 校准ADC的转换结果
void   ADC_MACRO_INIT( Uint16  *ChSel_X,  Uint16 *Trigsel_X,  Uint16  *ACQPS_X); // ADC的通道和结果寄存器配置
void   ADC_SOC_int(void ); //ADC初始化函数
#endif  // end of ADC_int_H definition

//===========================================================================
// End of file.
//===========================================================================
