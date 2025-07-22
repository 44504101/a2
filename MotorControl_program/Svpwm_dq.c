//############################################################
// FILE:  Svpwm_dq.c
// Created on: 2017年12月10日
// Author: XQ
// summary: Svpwm_dq
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//版权所有，盗版必究
//DSP28335旋变PMSM电机控制与开发板
//硕历电子
//网址: https://shuolidianzi.taobao.com
//Author-QQ: 616264123
//电机控制QQ群：736369775
//############################################################

#include "Motorcontrol_include.h"

// SVPWM是7段式矢量调试，扇区与输出电压占空比经过简化后为对称输出，1和4，2和5，3和6对称
// 文档给出是按照7段式SVPWM一点点按照矢量调制原理推到，中间推到有Uabc的三相电压
// T1和T1作用时间，Txyz时间最终等效Tabc三相输入占空比,
// Udc/√3=1，  单位1
// 调制利用率等于(1-0.04)=0.96=96%
// Ualpha和Ubeta是和uq ud等幅值变换

/*void  SVPWM_MACRO(p_SVPWM  pV)
{
    pV->tmp1= pV->Ubeta;   // 相当于二相静止坐标--到三相静止变换出Uabc
	pV->tmp2= pV->Ubeta*0.5 + pV->Ualpha*0.8660254 ;
    pV->tmp3= pV->tmp2 - pV->tmp1;   // 三相逆变换和网上公式极性不同，
    //本方法的SVPWM的扇区判断不同1--6/对应0--360，均匀分布
	pV->VecSector=3;   // 根据三相电压符号计算矢量扇区
	pV->VecSector=(pV->tmp2> 0)?( pV->VecSector-1):pV->VecSector;
	pV->VecSector=(pV->tmp3> 0)?( pV->VecSector-1):pV->VecSector;
	pV->VecSector=(pV->tmp1< 0)?(7-pV->VecSector) :pV->VecSector;

	if     (pV->VecSector==1 || pV->VecSector==4)   // 根据矢量扇区计算矢量占空比Tabc
   {
		pV->Ta= pV->tmp2;
		pV->Tb= pV->tmp1-pV->tmp3;
		pV->Tc=-pV->tmp2;
   }

    else if(pV->VecSector==2 || pV->VecSector==5)
   {
    	pV->Ta= pV->tmp3+pV->tmp2;
		pV->Tb= pV->tmp1;
		pV->Tc=-pV->tmp1;
   }
    else if(pV->VecSector==3 || pV->VecSector==6)
   {
    	pV->Ta= pV->tmp3;
      	pV->Tb=-pV->tmp3;
      	pV->Tc=-(pV->tmp1+pV->tmp2);
   }
   else     //异常状态下的判断出的扇区 0---7或者其他就执行0电压矢量
   {
		pV->Ta=0;
		pV->Tb=0;
		pV->Tc=0;
   }

	// Tabc是在浮点数据正负  ， 电压，pV->Ta/Svpwm_Km是标幺值计算，将Tabc计算成-1---1
	// 将占空比调节为-1---1   ，再讲PWM的半周期占空比值提出    (-1---1)*50%+50%=0---100%
	pV->SVPTa=(pV->Ta/Svpwm_Km)*PWM_HalfPerMax+PWM_HalfPerMax;  // 计算
	pV->SVPTb=(pV->Tb/Svpwm_Km)*PWM_HalfPerMax+PWM_HalfPerMax;
	pV->SVPTc=(pV->Tc/Svpwm_Km)*PWM_HalfPerMax+PWM_HalfPerMax;
}*/

//===========================================================================
// No more.
//===========================================================================
