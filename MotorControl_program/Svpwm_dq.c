//############################################################
// FILE:  Svpwm_dq.c
// Created on: 2017��12��10��
// Author: XQ
// summary: Svpwm_dq
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//��Ȩ���У�����ؾ�
//DSP28335����PMSM��������뿪����
//˶������
//��ַ: https://shuolidianzi.taobao.com
//Author-QQ: 616264123
//�������QQȺ��736369775
//############################################################

#include "Motorcontrol_include.h"

// SVPWM��7��ʽʸ�����ԣ������������ѹռ�ձȾ����򻯺�Ϊ�Գ������1��4��2��5��3��6�Գ�
// �ĵ������ǰ���7��ʽSVPWMһ��㰴��ʸ������ԭ���Ƶ����м��Ƶ���Uabc�������ѹ
// T1��T1����ʱ�䣬Txyzʱ�����յ�ЧTabc��������ռ�ձ�,
// Udc/��3=1��  ��λ1
// ���������ʵ���(1-0.04)=0.96=96%
// Ualpha��Ubeta�Ǻ�uq ud�ȷ�ֵ�任

/*void  SVPWM_MACRO(p_SVPWM  pV)
{
    pV->tmp1= pV->Ubeta;   // �൱�ڶ��ྲֹ����--�����ྲֹ�任��Uabc
	pV->tmp2= pV->Ubeta*0.5 + pV->Ualpha*0.8660254 ;
    pV->tmp3= pV->tmp2 - pV->tmp1;   // ������任�����Ϲ�ʽ���Բ�ͬ��
    //��������SVPWM�������жϲ�ͬ1--6/��Ӧ0--360�����ȷֲ�
	pV->VecSector=3;   // ���������ѹ���ż���ʸ������
	pV->VecSector=(pV->tmp2> 0)?( pV->VecSector-1):pV->VecSector;
	pV->VecSector=(pV->tmp3> 0)?( pV->VecSector-1):pV->VecSector;
	pV->VecSector=(pV->tmp1< 0)?(7-pV->VecSector) :pV->VecSector;

	if     (pV->VecSector==1 || pV->VecSector==4)   // ����ʸ����������ʸ��ռ�ձ�Tabc
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
   else     //�쳣״̬�µ��жϳ������� 0---7����������ִ��0��ѹʸ��
   {
		pV->Ta=0;
		pV->Tb=0;
		pV->Tc=0;
   }

	// Tabc���ڸ�����������  �� ��ѹ��pV->Ta/Svpwm_Km�Ǳ���ֵ���㣬��Tabc�����-1---1
	// ��ռ�ձȵ���Ϊ-1---1   ���ٽ�PWM�İ�����ռ�ձ�ֵ���    (-1---1)*50%+50%=0---100%
	pV->SVPTa=(pV->Ta/Svpwm_Km)*PWM_HalfPerMax+PWM_HalfPerMax;  // ����
	pV->SVPTb=(pV->Tb/Svpwm_Km)*PWM_HalfPerMax+PWM_HalfPerMax;
	pV->SVPTc=(pV->Tc/Svpwm_Km)*PWM_HalfPerMax+PWM_HalfPerMax;
}*/

//===========================================================================
// No more.
//===========================================================================
