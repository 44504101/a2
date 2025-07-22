//############################################################
// FILE:  Motorcontrol_include.h
// Created on: 2017��12��10��
// Author: XQ
// summary: Header file  and definition
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//��Ȩ���У�����ؾ�
//TC1782��������뿪����
//˶������
//��ַ: https://shuolidianzi.taobao.com
//Author-QQ: 616264123
//�������QQȺ��736369775
//############################################################

#ifndef _Motorcontrol_include_H
#define _Motorcontrol_include_H

#include "IQmathLib.h"
#include "Drive_include.h"
#include "AD2S1205_Resolver.h"
#include "MotorC_parameter.h"
#include "Common_Math.h"
#include "Axis_transform.h"
#include "Svpwm_dq.h"
#include "PI_Cale.h"
#include "MotorControl_ISR.h"
#include "ADC_Sample.h"
#include "IdIqMap.h"
#include "FOC_Control.h"
#include "rmp_cntl.h"
#include "rampgen.h"
#include "volt_calc.h"
#include"dlog4ch-HVPM_Sensored.h"
#include"f2833xpwm.h"
#include"f2833xpwmdac.h"
#include"f2833xileg_vdc.h"
#include"offset.h"
#include"speed_fr.h"
#include"smopos_const.h"
#include"smopos.h"
#include"speed_est.h"
#include"FOC.h"
#include"RS485.h"
#include"protocol.h"

typedef struct{
    uint16_t BusVolat;//ĸ�ߵ�ѹ
    float32_t Current;
    uint16_t SpeedRpm;
    uint16_t ABsPos_cnt;
    uint16_t NowPos_cnt;
    uint8_t  StatusByte;
    float32_t adc_raw;
}MOTOR_STATUS_t;

#define MOTOR_STATUS_t_DEAFAULT {0,        \
                          0,        \
                          0,        \
                          0,   \
                          0,        \
                         0,   \
                         0,\
                         }
extern  MOTOR_STATUS_t motorSt;
extern int16 DlogCh1;
extern int16 DlogCh2;
extern int16 DlogCh3;
extern int16 DlogCh4;

extern float32 T;
extern Uint16 SpeedLoopCount;
extern Uint16 SpeedLoopPrescaler;
#define SYSTEM_FREQUENCY 150
#define ISR_FREQUENCY 10
#endif /* Motorcontrol_include*/
//===========================================================================
// End of file.
//===========================================================================
