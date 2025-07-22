//############################################################
// FILE:  Axis_transform.c
// Created on: 2017��12��10��
// Author: XQ
// summary: ADC_int
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//��Ȩ���У�����ؾ�
//DSP28335����PMSM��������뿪����
//˶������
//��ַ: https://shuolidianzi.taobao.com
//Author-QQ: 616264123
//�������QQȺ��736369775
//############################################################

#include "Motorcontrol_include.h"

//Alpha = Iu
//Beta = (��3/3*Iu + 2*��3/3*Iv)
void  CLARKE_Cale(p_CLARKE  pV)  // 2/3ϵ�� �ȷ�ֵ�任
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

