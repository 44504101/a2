 //############################################################
// FILE:  PI_Cale.c
// Created on: 2017年12月10日
// Author: XQ
// summary: PI_Cale
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//版权所有，盗版必究
//DSP28335旋变PMSM电机控制与开发板
//硕历电子
//网址: https://shuolidianzi.taobao.com
//Author-QQ: 616264123
//电机控制QQ群：736369775
//############################################################

#include "Motorcontrol_include.h"

PI_Control   pi_spd = PI_CONTROLLER_DEFAULTS ;
PI_Control   pi_id  = PI_CONTROLLER_DEFAULTS ;
PI_Control   pi_iq  = PI_CONTROLLER_DEFAULTS ;
PI_Control   pi_FW  = PI_CONTROLLER_DEFAULTS ;

/*
void  PI_Controller(p_PI_Control  pV)
{
     proportional term
	pV->up = pV->Ref - pV->Fbk; // 偏差计算

	 integral term
	pV->ui = (pV->Out == pV->v1)?(pV->Ki*pV->up+ pV->i1): pV->i1;  // 抗积分饱和，锁定PI饱和时的积分值
	pV->i1 = pV->ui;  // 当out等于v1时输出，没有达到最大值，没有限制输出，当不等于的时候就是溢出最大，消除积分累加项

	 control output
	pV->v1 = pV->Kp*pV->up  + pV->ui;  // 比例量角积分量
	pV->Out= Limit_Sat(pV->v1, pV->Umax, pV->Umin); // 限制PI输出，超出最大值
}
*/



void  PI_Pare_init(void )
{
	  pi_spd.Kp=0.012;//0.012
	  pi_spd.Ki=0.0028;  // 0.0001*10 / 0.2   T*SpeedLoopPrescaler/0.2  0.00853928209901472  2.26849872269596
	  pi_spd.Umax =100;  // 扭矩 100
	  pi_spd.Umin =-100;  //-100

	  pi_id.Kp=0.005;  //0.05
	  pi_id.Ki=0.0012;  //
	  pi_id.Umax =PI_MAX_Ud;
	//pi_id.Umax =5;
	  pi_id.Umin =-PI_MAX_Ud;
			 // pi_id.Umax =-5;

	  pi_iq.Kp=0.05;
	  pi_iq.Ki=0.0012;
	  pi_iq.Umax = PI_MAX_Uq;
	  //pi_iq.Umax = 10;
	 pi_iq.Umin = -PI_MAX_Uq;  //-PI_MAX_Uq
	  //pi_iq.Umin = -10;
}


//===========================================================================
// No more.
//===========================================================================

