// FILE: CpuTimer.h
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

#include "DSP2833x_Project.h"

#ifndef CpuTimer_H
#define CpuTimer_H

void InitCpuTimer0(void);
interrupt void cpu_timer0_isr(void);
#endif  // end of CpuTimer definition

//===========================================================================
// End of file.
//===========================================================================
