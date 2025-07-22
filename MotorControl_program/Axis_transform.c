//############################################################
// FILE:  Axis_transform.c
// Created on: 2017年12月10日
// Author: XQ
// summary: ADC_int
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//版权所有，盗版必究
//DSP28335旋变PMSM电机控制与开发板
//硕历电子
//网址: https://shuolidianzi.taobao.com
//Author-QQ: 616264123
//电机控制QQ群：736369775
//############################################################

#include "Motorcontrol_include.h"

//Alpha = Iu
//Beta = (√3/3*Iu + 2*√3/3*Iv)
void  CLARKE_Cale(p_CLARKE  pV)  // 2/3系数 等幅值变换
{
	pV->Alpha = pV->As;
	pV->Beta =  (pV->As +pV->Bs*2)*0.577350269;
}

// Parking Id,Iq
// Id = Ialpha*cos+Ibeta*sin
// Iq = Ibeta*cos-Ialpha*sin

void  PARK_Cale(p_PARK pV)
{
	pV->Ds =  pV->Alpha*pV->Cosine + pV->Beta*pV->Sine ;
    pV->Qs =  pV->Beta*pV->Cosine  - pV->Alpha*pV->Sine ;
}

//IParking Ia,Ib
// Ialpha = Id*cos-Iq*sin
// Ibeta = Iq*cos+Id*sin

/*void  IPARK_MACRO(v)   // pV->
{
	v.Alpha = _IQmpy(v.Ds,v.Cosine) - _IQmpy(v.Qs,v.Sine);
	v.Beta  = _IQmpy(v.Qs,v.Cosine) + _IQmpy(v.Ds,v.Sine);
}*/


//===========================================================================
// No more.
//===========================================================================

