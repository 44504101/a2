//############################################################
// FILE:  MotorC_parameter.h
// Created on: 2017年12月10日
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

#ifndef _MotorC_parameter_H
#define _MotorC_parameter_H
#define PI 3.14159265358979
#define POLES  	8						// Number of poles
#define CPU_Freq      (90000000)
#define Delay_1Bns    (CPU_Freq/10000000+5)
#define Delay_1us     (CPU_Freq/1000000+5)
#define Delay_1ms     (CPU_Freq/1000000*1000+5)

#define IQAngle_Range  0xFFF//   12位精度位置

#define Phase_Curr_Sensor_Range  50//相电流传感器量程     对电流环调试有利
#define BUS_Curr_Sensor_Range    50//母线电流传感器量程     此硬件是12.5A，软件放大4倍电流  对电流环调试有利


#define BUS_Voltage_Add_Coeff   0.5    //母线电压加法系数
#define BUS_Voltage_Mult_Coeff  0.015 //母线电压乘法系数

#define  ANGLE_120              1365   // 120/360*4096
#define  Motor_MAX_Trq_NM       20// 电机最大扭矩 1NM 限制扭矩
#define  Motor_MAX_Speed_RPM    3500  //rpm 限制转速
//#define  Motor_MAX_Speed_RPM    3000  //rpm 限制转速
#define  Motor_Over_Speed_RPM   3000  //rpm 限制转速

#define  AD2SXB_12Bit              0XFFF

#define  Protect_Low_Voltage_V     16.0 //低压保护
#define  Standard_Work_Voltage_V   24.0	//标准工作电压   24V 放大10倍
#define  Protect_High_Voltage_V    32.0 //高保护

#define Motor_Poles         4		//电机极对数
#define Resolver_Poles      1      //旋变极对数
#define Poles_Ratio_Coeff   4       //极对数比系数

#define Motor_Rs 			0.46035	  //相电阻 欧姆
#define Motor_Ld          	0.00077905  //D轴电感
#define Motor_Lq         	0.00077905  //Q轴电感
#define Motor_Flux       	0.2077987	  //磁链
#define BASE_FREQ      	233.33          // Base electrical frequency (Hz)
#define Sqrt_OF_3       0.57735
#define Svpwm_Km       (Standard_Work_Voltage_V*Sqrt_OF_3)
#define Vdc_s          (Standard_Work_Voltage_V*Sqrt_OF_3*0.96)
#define PI_MAX_Ud      (Standard_Work_Voltage_V*Sqrt_OF_3*0.5)   // 13.5
#define PI_MAX_Uq      (Standard_Work_Voltage_V*Sqrt_OF_3*0.96)

#define   Control_Mode_Stop      0     //停机模式
#define   Control_Mode_Speed     1     //速度控制模式 正反转
#define   Control_Mode_Torq      2     //扭矩控制模式
#define   Control_Mode_VF        3     //VF控制模式
#define   Control_Mode_Vdq       4     //电压控制模式
#define   Control_Mode_Idq       5     //电流控制模式



#define   Speed_Grad_Timer 	    100		//定义速度的梯度时间
#define   Speed_Grad_RPM    	5		//定义速度的梯度值

#define   Trq_Grad_Timer 		100		//定义扭矩的梯度时间
#define   Trq_Grad_0D1NM    	2		//定义扭矩的梯度值

#define   Idq_Grad_Timer 		50		//定义电流的梯度时间
#define   Idq_Grad_0D1A      	0.05	//定义电流的梯度值

#define   Vdq_Grad_Timer 		50		//定义电压的梯度时间
#define   Vdq_Grad_0D1V      	0.1		//定义电压的梯度值

#define   VF_q_Grad_Timer 		50		//定义电流的梯度时间
#define   VF_q_Grad_0D1V      	0.1 	//定义电流的梯度值

#define   VF_F_Grad_Timer 		50		//定义电压的梯度时间
#define   VF_F_Grad_0D1HZ      	0.1		//定义电压的梯度值

#define   VF_F_Min 		        2		//定义频率的最小值
#define   VF_F_Max      	    50		//定义频率最大值

#define   FW_Grad_Timer 		20		//定义弱磁的执行时间

#define   GM_Low_Lass_A 		0.218	//通用低通滤波系数A
#define   GM_Low_Lass_B    	    0.782	//通用低通滤波系数B

#define   MinSpeed_Grad         10// 最小划算转速分段

#define   Stall_rpm_min         50
#define   Stall_rpm_max         90

#define   Stall_Trq_min         110
#define   Stall_Trq_max         125

#define   PHASE_Curr_max        40   // 相线电流
#define   BUS_Curr_max          20   // 20A的母线电流

#define   OVER_rpm_max          3300

#endif /* Motor_parameter_H*/
//===========================================================================
// End of file.
//===========================================================================
