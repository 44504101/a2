//############################################################
// FILE:  PI_Cale.h
// Created on: 2017��12��10��
// Author: XQ
// summary: Header file  and definition
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//��Ȩ���У�����ؾ�
//DSP28335����PMSM��������뿪����
//˶������
//��ַ: https://shuolidianzi.taobao.com
//Author-QQ: 616264123
//�������QQȺ��736369775
//############################################################

#ifndef  PI_Cale_H
#define  PI_Cale_H

#include "DSP2833x_Project.h"
#include"IQmathLib.h"
typedef struct {
	_iq  Ref;       // PI���Ƶĸ�������
	    _iq  Fbk;       // PI���Ƶķ�������
	    _iq  Out;       // PI���Ƶ��������
	    _iq  Kp;        // ��ǰ��������
	    _iq  Ki;        // ��ǰ��������
	    _iq  Kp_base;   // ������������
	    _iq  Ki_base;   // ������������
	    _iq  Kp_max;    // �����������ֵ
	    _iq  Kp_min;    // ����������Сֵ
	    _iq  Ki_max;    // �����������ֵ
	    _iq  Ki_min;    // ����������Сֵ
	    _iq  Kaw;       // �����ֱ�������
	    _iq  SatErr;    // �������
	    _iq  Umax;      // �������޷�
	    _iq  Umin;      // �����С�޷�
	    _iq  up;        // �����������
	    _iq  ui;        // �����������
	    _iq  v1;        // ���������ǰ��ֵ
	    _iq  i1;        // ��������״̬����ʷ����ֵ��
	    _iq  w1;        // ���ͼ�¼��u(k-1) - v(k-1)��
	    _iq  up_term;   // ���������
	    _iq  e_prev;    // ǰһ�������ڼ������仯�ʣ�
       } PI_Control;

#define PI_CONTROLLER_DEFAULTS {		\
		    _IQ(0.0),      /* Ref       */ \
		    _IQ(0.0),      /* Fbk       */ \
		    _IQ(0.0),      /* Out       */ \
		    _IQ(1.0),      /* Kp        */ \
		    _IQ(0.0),      /* Ki        */ \
		    _IQ(1.0),      /* Kp_base   */ \
		    _IQ(0.0),      /* Ki_base   */ \
		    _IQ(5.0),      /* Kp_max    */ \
		    _IQ(0.0),      /* Kp_min    */ \
		    _IQ(1.0),      /* Ki_max    */ \
		    _IQ(0.0),      /* Ki_min    */ \
		    _IQ(0.0),      /* Kaw       */ \
		    _IQ(0.0),      /* SatErr    */ \
		    _IQ(1.0),      /* Umax      */ \
		    _IQ(-1.0),     /* Umin      */ \
		    _IQ(0.0),      /* up        */ \
		    _IQ(0.0),      /* ui        */ \
		    _IQ(0.0),      /* v1        */ \
		    _IQ(0.0),      /* i1        */ \
		    _IQ(0.0),      /* w1        */ \
		    _IQ(0.0),      /* up_term   */ \
		    _IQ(0.0)       /* e_prev    */ \
              			  }

extern  PI_Control   pi_spd ;
extern  PI_Control   pi_id ;
extern  PI_Control   pi_iq ;
extern  PI_Control   pi_FW ;

//void  PI_Controller(p_PI_Control  pV);  //PI�����㷨����
void  PI_Pare_init(void );    //PI���Ʋ�����ʼ��

/*#define PI_MACRO(v)                                                      \
		v.up = v.Ref - v.Fbk;                                          \
		v.ui = (v.Out == v.v1)?(_IQmpy(v.Ki, v.up)+ v.i1) : v.i1;	\

		v.i1 = v.ui;												\
		v.v1 = _IQmpy(v.Kp, (v.up + v.ui));							\
		v.Out= _IQsat(v.v1, v.Umax, v.Umin);						\*/
static void FuzzyAdaptivePI(PI_Control *v)
{
    // �����������仯��
    _iq e = v->Ref - v->Fbk;          // ��ǰ���
    _iq de = e - v->e_prev;           // ���仯��
    v->e_prev = e;                    // ����ǰһ�����

    // ģ�������̣�����ʵ�������������Ⱥ�����
    // �����ṩһ���򻯵�ʾ��

    int e_level = 0;   // ���ȼ�
    int de_level = 0;  // ���仯�ʵȼ�

    // �������ȼ���ʾ����
    if(e > _IQ(0.1)) e_level = 2;
    else if(e > _IQ(0.05)) e_level = 1;
    else if(e > _IQ(-0.05)) e_level = 0;
    else if(e > _IQ(-0.1)) e_level = -1;
    else e_level = -2;

    // �������仯�ʵȼ���ʾ����
    if(de > _IQ(0.1)) de_level = 2;
    else if(de > _IQ(0.05)) de_level = 1;
    else if(de > _IQ(-0.05)) de_level = 0;
    else if(de > _IQ(-0.1)) de_level = -1;
    else de_level = -2;

    // ģ���������Ҫ����ʵ��ϵͳ��ƣ�
    _iq delta_Kp = _IQ(0.0);
    _iq delta_Ki = _IQ(0.0);

    // �򻯵�ģ������ʾ��
    if(e_level == 2)
    {
        delta_Kp = _IQ(0.1);
        delta_Ki = _IQ(-0.01);
    }
    else if(e_level == 1)
    {
        delta_Kp = _IQ(0.05);
        delta_Ki = _IQ(0.0);
    }
    else if(e_level == 0)
    {
        delta_Kp = _IQ(0.0);
        delta_Ki = _IQ(0.0);
    }
    else if(e_level == -1)
    {
        delta_Kp = _IQ(-0.05);
        delta_Ki = _IQ(0.0);
    }
    else // e_level == -2
    {
        delta_Kp = _IQ(-0.1);
        delta_Ki = _IQ(-0.01);
    }

    // ��������������
    v->Kp = v->Kp_base + delta_Kp;
    v->Ki = v->Ki_base + delta_Ki;

    // �����޷�
    if(v->Kp > v->Kp_max) v->Kp = v->Kp_max;
    if(v->Kp < v->Kp_min) v->Kp = v->Kp_min;
    if(v->Ki > v->Ki_max) v->Ki = v->Ki_max;
    if(v->Ki < v->Ki_min) v->Ki = v->Ki_min;
}
#define PI_MACRO(v)                                                          \
/* ����ģ������Ӧ���� */                                                 \
    FuzzyAdaptivePI(&v);                                                     \
    /* ������� */                                                           \
    v.up = v.Ref - v.Fbk;                                                    \
    /* ���������� */                                                       \
    v.up_term = _IQmpy(v.Kp, v.up);                                          \
    /* ���»�������뷴���ֻ��� */                                          \
    v.SatErr = v.Out - v.v1;                                                 \
    v.ui = v.ui + _IQmpy(v.Ki, v.up) + _IQmpy(v.Kaw, v.SatErr);              \
    /* �洢��������״̬ */                                                   \
    v.i1 = v.ui;                                                             \
    /* ������������ǰ��ֵ */                                                \
    v.v1 = v.up_term + v.ui;                                                 \
    /* ����������޷� */                                                     \
    v.Out = _IQsat(v.v1, v.Umax, v.Umin);


#endif /* PI_Cale*/
