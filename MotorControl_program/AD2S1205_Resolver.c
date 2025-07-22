//############################################################
// FILE: AD2S1205_Resolver.c
// Created on: 2017年12月10日
// Author: XQ
// summary: AD2S1205_Resolver
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//版权所有，盗版必究
//DSP28335旋变PMSM电机控制与开发板
//硕历电子
//网址: https://shuolidianzi.taobao.com
//Author-QQ: 616264123
//电机控制QQ群：736369775
//############################################################

#include "Motorcontrol_include.h"

AD2SXB      AD2SXBPare=AD2SXB_DEFAULTS;
bool IsOverCurrent() {
     _iq current = pi_iq.Out;  // 获取当前电机电流，需要根据您的硬件实现
    return current >= CurrentThreshold;
}
void AD2S1205_Init(void)
{
	AD2SXBPare.init_Angle=442.2;  // 电机零位
	 // 旋变安装时候与电机反电动势过零点存在绝对相差角度是零位值
	 Delay_ms(5);     // 硬件电平SOE高电平并行，FS12高电平，励磁10KHZ
	 XB_NRST_OFF;
	 SAMPLE_ON;
	 DELAY_US(100);  // 超过10us的上电复位
	 XB_NRST_ON;
	 Delay_ms(30);    //超过20ms的电路稳定时间
	 SAMPLE_OFF;
	 Delay_ms(2) ;
	 SAMPLE_ON;
}

Uint16 Read_IO_interface(void)
{
	Uint16 AD2S_data=0;   // 旋转变压器的并口数据12位读取IO 非连续的IO读取状态
    AD2S_data=DB_0+(DB_1<<1)+(DB_2<<2)+(DB_3<<3)+(DB_4<<4)+(DB_5<<5)
    +(DB_6<<6)+(DB_7<<7)+(DB_8<<8)+(DB_9<<9)+(DB_10<<10)+(DB_11<<11);	  // 10US
	return  AD2S_data;
}

void AD2S1205_Read(void)
{
	volatile Uint8 AD2Si=0;
	Uint8 Fault_DOS=0,Fault_LOT=0;

	 SAMPLE_OFF;
	 Delay_100ns(6);  // 6/8.192M+20ns=500ns
	 CS_RD_OFF;
	 Delay_100ns(1);   // 100ns
	 RDVEL_ON;
	 AD2Si=1; AD2Si=2; // 停顿延时2个时钟
	 Delay_100ns(1);   // 100n
	 AD2SXBPare.AD2S_angle =Read_IO_interface();  // 读取位置信号
     //AD2SXBPare.AD2S_angle_du = AD2SXBPare.AD2S_angle*0.0879;
     //AD2SXBPare.AD2S_circle += 1;
	   // 旋变安装时候与电机反电动势过零点存在绝对相差角度是零位值
	 AD2SXBPare.PM_angle =(Uint16)(AD2SXBPare.AD2S_angle*Poles_Ratio_Coeff-AD2SXBPare.init_Angle)&(AD2SXB_12Bit);

	 Delay_100ns(1);   // 100ns
	 RDVEL_OFF;
	 AD2Si=1; AD2Si=2;  // 停顿延时2个时钟
	 Delay_100ns(1);   // 100n
	 CS_RD_ON;
	 AD2Si=1; AD2Si=2;  // 停顿延时2个时钟
	 Delay_100ns(1);   // 100n
	 CS_RD_OFF;
	 Delay_100ns(1);   // 100ns
	 AD2SXBPare.AD2S_Spd_RD =Read_IO_interface(); // 读取旋变速度信号值
	 // 信号数据高第12位是方向信号 0是正转 1是反转负的 所以将数据左移位后，取有符号16数据 即把方向符号取出，然后在右移动4位
	 AD2SXBPare.AD2S_Spd=((int16)(AD2SXBPare.AD2S_Spd_RD<<4))>>4;  //  Poles_Ratio_Coeff
	 AD2SXBPare.AD2S_SpdRPM=(int16)(AD2SXBPare.AD2S_Spd*((float32)(625/64)));
	 //AD2SXBPare.AD2S_SpdRPM=(int16)(AD2SXBPare.AD2S_Spd*((float32)(60*1000/2048)));
	 // 电机极对数和旋变极对数的系数计算    AD2SXBPare.AD2S_Spd*60*1000/2048/3  3= 旋转变压器极对数
	 AD2SXBPare.AD2S_SpdRPMH=AD2SXBPare.AD2S_SpdRPMH*GM_Low_Lass_A + AD2SXBPare.AD2S_SpdRPM*GM_Low_Lass_B;  // 转速一节低通滤波

	 //AD2SXBPare.AD2S_SpdRPMH=SpeedRpm_GXieLv.XieLv_Y+3;  //  测试通讯
     // 读取旋变DOS和LOT故障，进行保护处理
	 Fault_DOS=GPIO_DOS;
	 Fault_LOT=GPIO_LOT;

	 AD2SXBPare.AD2S_FaultDOSLOT=(Uint8)((Fault_DOS<<1)+Fault_LOT); // 将故障数据处理为2位数据
	 //AD2SXBPare.AD2S_FaultDOSLOT=Fault_DOS;

	 if(AD2SXBPare.AD2S_SpdRPMH>0.1)   // 判断正反转
	 AD2SXBPare.Move_State=1;
	 else if(AD2SXBPare.AD2S_SpdRPMH<-0.1)
	 AD2SXBPare.Move_State=2;
	 else
	 AD2SXBPare.Move_State=0;  // 不在正负10RPM内就认为停机

	 AD2SXBPare.Abs_SpdRPM=Abs(AD2SXBPare.AD2S_SpdRPMH);

	 CS_RD_ON;
	 RDVEL_ON;
	 SAMPLE_ON;
}


//===========================================================================
// No more.
//===========================================================================
