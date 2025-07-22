//############################################################
// FILE:  GPIO_int.c
// Created on: 2017年12月10日
// Author: XQ
// summary: GPIO_int
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//版权所有，盗版必究
//DSP28335永磁同步电机控制开发板
//硕历电子
//网址: https://shuolidianzi.taobao.com
//Author-QQ: 616264123
//电机控制QQ群：736369775
//############################################################

#include "Drive_include.h"

 void Init_Gpio_LED(void)
 {
	 EALLOW;

	 // GPIO0      CPU运行灯     //  1 S闪烁一次
   GpioCtrlRegs.GPAMUX2.bit.GPIO29 = 0;  									// GPIO11 = GPIO
    GpioCtrlRegs.GPADIR.bit.GPIO29 = 1;      // GPIO11 = output
    //  GPIO1     系统故障灯
    GpioCtrlRegs.GPAMUX2.bit.GPIO30 = 0;  									// GPIO11 = GPIO
    GpioCtrlRegs.GPADIR.bit.GPIO30 = 1;      // GPIO11 = output
/*	    GpioCtrlRegs.GPCMUX2.bit.GPIO82 = 0;  									// GPIO11 = GPIO
	     GpioCtrlRegs.GPCDIR.bit.GPIO83 = 1;      // GPIO11 = output
	     //  GPIO1     系统故障灯
	     GpioCtrlRegs.GPCMUX2.bit.GPIO82 = 0;  									// GPIO11 = GPIO
	     GpioCtrlRegs.GPCDIR.bit.GPIO83 = 1;*/
    EDIS;
  }

void Init_3MotorGpio(void)
{
   EALLOW;
   GpioCtrlRegs.GPAMUX1.bit.GPIO0 = 1;
   GpioCtrlRegs.GPAMUX1.bit.GPIO1 = 1;
   GpioCtrlRegs.GPAMUX1.bit.GPIO2 = 1;
   GpioCtrlRegs.GPAMUX1.bit.GPIO3 = 1;
   GpioCtrlRegs.GPAMUX1.bit.GPIO4 = 1;
   GpioCtrlRegs.GPAMUX1.bit.GPIO5 = 1;

  // GpioCtrlRegs.GPAMUX1.bit.GPIO12 =1;	// 0=GPIO,  1=TZ1,  2=SCITX-A,  3=SPISIMO-B
  // GpioCtrlRegs.GPADIR.bit.GPIO12 = 0;	// 1=OUTput,  0=INput    DB9

   EDIS;
}

void Init_DACEPWMGpio(void)
{
   EALLOW;
   GpioCtrlRegs.GPAMUX1.bit.GPIO6 = 1;
   GpioCtrlRegs.GPAMUX1.bit.GPIO7 = 1;
   EDIS;
}

void Init_ECanbGpio(void)
{
   EALLOW;
   GpioCtrlRegs.GPAPUD.bit.GPIO20 = 0;   // Enable pull-up for GPIO20 (CANTXB)
   GpioCtrlRegs.GPAPUD.bit.GPIO21 = 0;   // Enable pull-up for GPIO21 (CANRXB)
   GpioCtrlRegs.GPAQSEL2.bit.GPIO21 = 3; // Asynch qual for GPIO21 (CANRXB)
   GpioCtrlRegs.GPAMUX2.bit.GPIO20 = 3;  // Configure GPIO20 for CANTXB operation
   GpioCtrlRegs.GPAMUX2.bit.GPIO21 = 3;  // Configure GPIO21 for CANRXB operation
   EDIS;
}

// 67，68,69,70,。。。。77，78   12个
// 34 35 36 37   时序引脚 RD CS等

void Init_Gpio_AD2S1205(void)
{
	 EALLOW;
	GpioCtrlRegs.GPBMUX1.bit.GPIO35 = 0;
	GpioCtrlRegs.GPBDIR.bit.GPIO35 = 0;  // GPIO67 = inputIO
	GpioCtrlRegs.GPBMUX1.bit.GPIO34 = 0;
	GpioCtrlRegs.GPBDIR.bit.GPIO34 = 0;  // GPIO67 = inputIO


	 // GPIO34
    GpioCtrlRegs.GPAMUX2.bit.GPIO28 = 0;
    GpioCtrlRegs.GPADIR.bit.GPIO28 = 1;      // GPIO34 = output
    //  GPIO35
    GpioCtrlRegs.GPCMUX1.bit.GPIO79 = 0;
    GpioCtrlRegs.GPCDIR.bit.GPIO79 = 1;      // GPIO35 = output
	 // GPIO36
   GpioCtrlRegs.GPBMUX1.bit.GPIO36 = 0;
   GpioCtrlRegs.GPBDIR.bit.GPIO36 = 1;      // GPIO36 = output
    //  GPIO37
   GpioCtrlRegs.GPBMUX1.bit.GPIO38 = 0;
   GpioCtrlRegs.GPBDIR.bit.GPIO38 = 1;      // GPIO37 = output

	// 并口数据读取DB0--DB11
   GpioCtrlRegs.GPCPUD.bit.GPIO67 = 1u;	// pull-up disabled
   	GpioCtrlRegs.GPCMUX1.bit.GPIO67 = 3u;	// XD12 - resolver data bus
   	GpioCtrlRegs.GPCDIR.bit.GPIO67 = 0u;	// input (don't care as pin is peripheral)

   	// GPIO68
   	GpioCtrlRegs.GPCPUD.bit.GPIO68 = 1u;	// pull-up disabled
   	GpioCtrlRegs.GPCMUX1.bit.GPIO68 = 3u;	// XD11 - resolver data bus
   	GpioCtrlRegs.GPCDIR.bit.GPIO68 = 0u;	// input (don't care as pin is peripheral)

   	// GPIO69
   	GpioCtrlRegs.GPCPUD.bit.GPIO69 = 1u;	// pull-up disabled
   	GpioCtrlRegs.GPCMUX1.bit.GPIO69 = 3u;	// XD10 - resolver data bus
   	GpioCtrlRegs.GPCDIR.bit.GPIO69 = 0u;	// input (don't care as pin is peripheral)

   	// GPIO70
   	GpioCtrlRegs.GPCPUD.bit.GPIO70 = 1u;	// pull-up disabled
   	GpioCtrlRegs.GPCMUX1.bit.GPIO70 = 3u;	// XD9 - resolver data bus
   	GpioCtrlRegs.GPCDIR.bit.GPIO70 = 0u;	// input (don't care as pin is peripheral)

   	// GPIO71
   	GpioCtrlRegs.GPCPUD.bit.GPIO71 = 1u;	// pull-up disabled
   	GpioCtrlRegs.GPCMUX1.bit.GPIO71 = 3u;	// XD8 - resolver data bus
   	GpioCtrlRegs.GPCDIR.bit.GPIO71 = 0u;	// input (don't care as pin is peripheral)

   	// GPIO72
   	GpioCtrlRegs.GPCPUD.bit.GPIO72 = 1u;	// pull-up disabled
   	GpioCtrlRegs.GPCMUX1.bit.GPIO72 = 3u;	// XD7 - resolver data bus
   	GpioCtrlRegs.GPCDIR.bit.GPIO72 = 0u;	// input (don't care as pin is peripheral)

   	// GPIO73
   	GpioCtrlRegs.GPCPUD.bit.GPIO73 = 1u;	// pull-up disabled
   	GpioCtrlRegs.GPCMUX1.bit.GPIO73 = 3u;	// XD6 - resolver data bus
   	GpioCtrlRegs.GPCDIR.bit.GPIO73 = 0u;	// input (don't care as pin is peripheral)

   	// GPIO74
   	GpioCtrlRegs.GPCPUD.bit.GPIO74 = 1u;	// pull-up disabled
   	GpioCtrlRegs.GPCMUX1.bit.GPIO74 = 3u;	// XD5 - resolver data bus
   	GpioCtrlRegs.GPCDIR.bit.GPIO74 = 0u;	// input (don't care as pin is peripheral)

   	// GPIO75
   	GpioCtrlRegs.GPCPUD.bit.GPIO75 = 1u;	// pull-up disabled
   	GpioCtrlRegs.GPCMUX1.bit.GPIO75 = 3u;	// XD4 - resolver data bus
   	GpioCtrlRegs.GPCDIR.bit.GPIO75 = 0u;	// input (don't care as pin is peripheral)

   	// GPIO76
   	GpioCtrlRegs.GPCPUD.bit.GPIO76 = 1u;	// pull-up disabled
   	GpioCtrlRegs.GPCMUX1.bit.GPIO76 = 3u;	// XD3 - resolver data bus
   	GpioCtrlRegs.GPCDIR.bit.GPIO76 = 0u;	// input (don't care as pin is peripheral)

   	// GPIO77
   	GpioCtrlRegs.GPCPUD.bit.GPIO77 = 1u;	// pull-up disabled
   	GpioCtrlRegs.GPCMUX1.bit.GPIO77 = 3u;	// XD2 - resolver data bus
   	GpioCtrlRegs.GPCDIR.bit.GPIO77 = 0u;	// input (don't care as pin is peripheral)

   	// GPIO78
   	GpioCtrlRegs.GPCPUD.bit.GPIO78 = 1u;	// pull-up disabled
   	GpioCtrlRegs.GPCMUX1.bit.GPIO78 = 3u;	// XD1 - resolver data bus
   	GpioCtrlRegs.GPCDIR.bit.GPIO78 = 0u;	// input (don't care as pin is peripheral)


   	//GPIO(41or39)
   	GpioCtrlRegs.GPBPUD.bit.GPIO41 = 1u;
   	GpioCtrlRegs.GPBDIR.bit.GPIO41 = 0u;
   	GpioCtrlRegs.GPBMUX1.bit.GPIO41 = 0u;


   EDIS;
}

void Init_OneBMKGGpio(void)
{
   EALLOW;
   GpioCtrlRegs.GPBMUX1.bit.GPIO33 = 0;
   GpioCtrlRegs.GPBDIR.bit.GPIO33 = 0;  // GPIO67 = inputIO

   EDIS;
}


void InitScirs485bGpio(void)
{
   EALLOW;
   // GPIO24
   GpioDataRegs.GPASET.bit.GPIO24 = 1u;     // bit set (receiver disabled)
   GpioCtrlRegs.GPAPUD.bit.GPIO24 = 1u;     // pull-up disabled
   GpioCtrlRegs.GPAMUX2.bit.GPIO24 = 0u;    // GPIO - RS485 #RE (EPWM2B used but is internal signal)
   GpioCtrlRegs.GPADIR.bit.GPIO24 = 1u;     // output

   // GPIO25
   GpioDataRegs.GPACLEAR.bit.GPIO25 = 1u;   // bit clear (transmitter disabled)
   GpioCtrlRegs.GPAPUD.bit.GPIO25 = 1u;     // pull-up disabled
   GpioCtrlRegs.GPAMUX2.bit.GPIO25 = 0u;    // GPIO - RS485 DE
   GpioCtrlRegs.GPADIR.bit.GPIO25 = 1u;     // output

   // GPIO22
   GpioCtrlRegs.GPAPUD.bit.GPIO22 = 0u;    // pull-up enabled
   GpioCtrlRegs.GPAMUX2.bit.GPIO22 = 3u;   // SCITXDB - RS485 DI (output)
   GpioCtrlRegs.GPADIR.bit.GPIO22 = 0u;    // input (don't care as pin is peripheral)

   // GPIO23
       GpioCtrlRegs.GPAPUD.bit.GPIO23 = 0u;    // pull-up enabled
       GpioCtrlRegs.GPAMUX2.bit.GPIO23 = 3u;   // SCIRXDB - RS485 RO (input)
       GpioCtrlRegs.GPADIR.bit.GPIO23 = 0u;    // input

    EDIS;
}
void WANGLEI(void){
	GpioCtrlRegs.GPCMUX2.bit.GPIO82 = 0;
	GpioCtrlRegs.GPCDIR.bit.GPIO82 = 1;
	GpioDataRegs.GPCSET.bit.GPIO82=1;
	GpioCtrlRegs.GPCMUX2.bit.GPIO83 = 0;
	GpioCtrlRegs.GPCDIR.bit.GPIO83 = 1;
	GpioDataRegs.GPCSET.bit.GPIO83=1;
}
/*void Init_SPIGPIO(void)
{
	EALLOW;
	GpioCtrlRegs.GPBPUD.bit.
}*/
//===========================================================================
// No more.
//===========================================================================
