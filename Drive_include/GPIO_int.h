// FILE:  GPIO_int.h
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

#ifndef GPIO_int_H
#define GPIO_int_H
#define LED1_CLR  GpioDataRegs.GPACLEAR.bit.GPIO29=1
#define LED1_SET  GpioDataRegs.GPASET.bit.GPIO29=1
#define LED1_REV  GpioDataRegs.GPATOGGLE.bit.GPIO29=1
#define LED2_CLR  GpioDataRegs.GPACLEAR.bit.GPIO30=1
#define LED2_SET  GpioDataRegs.GPASET.bit.GPIO30=1
#define LED2_REV  GpioDataRegs.GPATOGGLE.bit.GPIO30=1

#define GPIO_BMKG   GpioDataRegs.GPBDAT.bit.GPIO33


void Init_Gpio_LED(void);
void Init_3MotorGpio(void);
void Init_DACEPWMGpio(void);
void InitScirs485bGpio(void);
void Init_ECanbGpio(void);
void Init_SPIGPIO(void);
void WANGLEI(void);
#endif  // end of GPIO_int_H definition

//===========================================================================
// End of file.
//===========================================================================
