// FILE: CpuTimer.h
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

#include "DSP2833x_Project.h"

#ifndef CpuTimer_H
#define CpuTimer_H

void InitCpuTimer0(void);
interrupt void cpu_timer0_isr(void);
#endif  // end of CpuTimer definition

//===========================================================================
// End of file.
//===========================================================================
