//############################################################
// FILE:  Axis_transform.h
// Created on: 2017��12��10��
// Author: XQ
// summary: Header file  and definition
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//��Ȩ���У�����ؾ�
//DSP28335����ͬ��������ƿ�����
//˶������
//��ַ: https://shuolidianzi.taobao.com
//Author-QQ: 616264123
//�������QQȺ��736369775
//############################################################

#ifndef Axis_transform_H
#define Axis_transform_H

#include "DSP2833x_Project.h"
#include"offset.h"

typedef struct {
            _iq  As;        //  �������A
            _iq  Bs;          //  �������B
            _iq  Cs;          //  �������C
            _iq Alpha;       //  ���ྲֹ����ϵ Alpha ��
            _iq  Beta;
            _iq abc;
            _iq bbc;
            } CLARKE ,*p_CLARKE ;

#define  CLARKE_DEFAULTS {0,0,0,0,0,0,0}  // ��ʼ������
extern CLARKE clark1;
typedef struct {
            _iq Alpha;     //  ���ྲֹ����ϵ Alpha ��
            _iq  Beta;        //  ���ྲֹ����ϵ Beta ��
            _iq   Angle;       //  ����ż�λ�ýǶ�0---65536����0---360��
            _iq  Ds;          //  ���������ת����ϵ�µ�d�����
            _iq  Qs;          //  ���������ת����ϵ�µ�q�����
            _iq  Sine;    //  ���Ҳ�����-32768---32767  -1��1
            _iq  Cosine;  //  ���Ҳ�����-32768---32767  -1��1
            } PARK , *p_PARK ;

#define  PARK_DEFAULTS {0,0,0,0,0,0,0}  // ��ʼ������


typedef struct {
            _iq  Alpha;         // ���ྲֹ����ϵ Alpha ��
            _iq   Beta;          // ���ྲֹ����ϵ Beta ��
            _iq   Angle;         // ����ż�λ�ýǶ�0---65536����0---360��
            _iq   Ds;            //  ���������ת����ϵ�µ�d�����
            _iq   Qs;            //  ���������ת����ϵ�µ�q�����
            _iq   Sine;        //  ���Ҳ�����-32768---32767  -1��1
            _iq   Cosine;        //  ���Ҳ�����-32768---32767  -1��1
            }IPARK , *p_IPARK;

#define  IPARK_DEFAULTS {0,0,0,0,0,0,0}  // ��ʼ������


void  CLARKE_Cale(p_CLARKE  pV); // ���ൽ����任 �����˱任
void  PARK_Cale(p_PARK pV) ;   // ���ൽ����任 �¿˱任
#define IPARK_MACRO(v)                    \
		v.Alpha = _IQmpy(v.Ds,v.Cosine) - _IQmpy(v.Qs,v.Sine);		\
		v.Beta  = _IQmpy(v.Qs,v.Cosine) + _IQmpy(v.Ds,v.Sine);

#define CLARKE_MACRO(v)											\
																\
v.Alpha = v.As;													\
v.Beta = _IQmpy((v.As +_IQmpy2(v.Bs)),_IQ(0.57735026918963));	\


#define PARK_MACRO(v)                                           \
		v.Ds = _IQmpy(v.Alpha,v.Cosine) + _IQmpy(v.Beta,v.Sine);	\
	    v.Qs = _IQmpy(v.Beta,v.Cosine) - _IQmpy(v.Alpha,v.Sine);




#endif /* Axis_transform*/
//===========================================================================
// End of file.
//===========================================================================
