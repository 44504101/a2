//############################################################
// FILE: Common_Math.h
// Created on: 2017年12月10日
// Author: XQ
// summary:    Header file  and definition
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//版权所有，盗版必究
//DSP28335旋变PMSM电机控制与开发板
//硕历电子
//网址: https://shuolidianzi.taobao.com
//Author-QQ: 616264123
//电机控制QQ群：736369775
//############################################################

#ifndef _Common_Math_H_
#define _Common_Math_H_

#define Abs(A)    ((A>=0)?A:-A)  // 绝对值函数
#define Min(A,B)  ((A<=B)?A:B)   // 求最小函数
#define Max(A,B)  ((A>=B)?A:B)   // 求最大函数


typedef struct
{
	Uint16  table_Angle;
	float32 table_Sin;
	float32 table_Cos;
}Ang_SinCos, *p_Ang_SinCos;

#define  Ang_SinCos_DEFAULTS    {0,0.0,0.0}

typedef struct 	{
	        float32  Alpha; 	//二相静止坐标系 Alpha 轴
	        float32  Beta;		//二相静止坐标系 Beta 轴
	        float32  IQTan;		//IQ格式正切 45度正切是1，IQ的格式是
  			int16  IQAngle;	//IQ格式角度值 0---65536 == 0---360度
         } IQAtan , *p_IQAtan;

#define IQAtan_DEFAULTS  {0,0,0,0}  // 初始化参数

typedef struct
 {  //指令参数的斜率处理
    float32  XieLv_X;   // 指令参数斜率输入变量x
	float32  XieLv_Y;
	float32  XieLv_Grad;
	Uint8    Timer_Count;
	Uint8    Grad_Timer;
 }GXieLv, *p_GXieLv;

 #define  GXieLv_DEFAULTS    {0.0,0.0,0.0,0}


void LookUp_CosSin(void);
float32  Limit_Sat( float32 Uint,float32 U_max, float32 U_min); //限制赋值函数
Uint32   IQSqrt(Uint32  M);
void Atan_Cale(p_IQAtan pV);  // 求取求反正弦函数
void SinCos_Table(p_Ang_SinCos PV);
void Grad_XieLv(p_GXieLv pV);
void Delay_ms(Uint32 t);
void Delay_us(Uint32 t);
void Delay_100ns(Uint32 t); // Delay_us

#endif /* SIN_COS_TABLE1_H_ */
