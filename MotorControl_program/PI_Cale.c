 //############################################################
// FILE:  PI_Cale.c
// Created on: 2017��12��10��
// Author: XQ
// summary: PI_Cale
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//��Ȩ���У�����ؾ�
//DSP28335����PMSM��������뿪����
//˶������
//��ַ: https://shuolidianzi.taobao.com
//Author-QQ: 616264123
//�������QQȺ��736369775
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
	pV->up = pV->Ref - pV->Fbk; // ƫ�����

	 integral term
	pV->ui = (pV->Out == pV->v1)?(pV->Ki*pV->up+ pV->i1): pV->i1;  // �����ֱ��ͣ�����PI����ʱ�Ļ���ֵ
	pV->i1 = pV->ui;  // ��out����v1ʱ�����û�дﵽ���ֵ��û������������������ڵ�ʱ��������������������ۼ���

	 control output
	pV->v1 = pV->Kp*pV->up  + pV->ui;  // �������ǻ�����
	pV->Out= Limit_Sat(pV->v1, pV->Umax, pV->Umin); // ����PI������������ֵ
}
*/



void  PI_Pare_init(void )
{
	  pi_spd.Kp=0.012;//0.012
	  pi_spd.Ki=0.0028;  // 0.0001*10 / 0.2   T*SpeedLoopPrescaler/0.2  0.00853928209901472  2.26849872269596
	  pi_spd.Umax =100;  // Ť�� 100
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

