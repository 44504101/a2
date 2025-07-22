//############################################################
// FILE:  MotorC_parameter.h
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

#ifndef _MotorC_parameter_H
#define _MotorC_parameter_H
#define PI 3.14159265358979
#define POLES  	8						// Number of poles
#define CPU_Freq      (90000000)
#define Delay_1Bns    (CPU_Freq/10000000+5)
#define Delay_1us     (CPU_Freq/1000000+5)
#define Delay_1ms     (CPU_Freq/1000000*1000+5)

#define IQAngle_Range  0xFFF//   12λ����λ��

#define Phase_Curr_Sensor_Range  50//���������������     �Ե�������������
#define BUS_Curr_Sensor_Range    50//ĸ�ߵ�������������     ��Ӳ����12.5A������Ŵ�4������  �Ե�������������


#define BUS_Voltage_Add_Coeff   0.5    //ĸ�ߵ�ѹ�ӷ�ϵ��
#define BUS_Voltage_Mult_Coeff  0.015 //ĸ�ߵ�ѹ�˷�ϵ��

#define  ANGLE_120              1365   // 120/360*4096
#define  Motor_MAX_Trq_NM       20// ������Ť�� 1NM ����Ť��
#define  Motor_MAX_Speed_RPM    3500  //rpm ����ת��
//#define  Motor_MAX_Speed_RPM    3000  //rpm ����ת��
#define  Motor_Over_Speed_RPM   3000  //rpm ����ת��

#define  AD2SXB_12Bit              0XFFF

#define  Protect_Low_Voltage_V     16.0 //��ѹ����
#define  Standard_Work_Voltage_V   24.0	//��׼������ѹ   24V �Ŵ�10��
#define  Protect_High_Voltage_V    32.0 //�߱���

#define Motor_Poles         4		//���������
#define Resolver_Poles      1      //���伫����
#define Poles_Ratio_Coeff   4       //��������ϵ��

#define Motor_Rs 			0.46035	  //����� ŷķ
#define Motor_Ld          	0.00077905  //D����
#define Motor_Lq         	0.00077905  //Q����
#define Motor_Flux       	0.2077987	  //����
#define BASE_FREQ      	233.33          // Base electrical frequency (Hz)
#define Sqrt_OF_3       0.57735
#define Svpwm_Km       (Standard_Work_Voltage_V*Sqrt_OF_3)
#define Vdc_s          (Standard_Work_Voltage_V*Sqrt_OF_3*0.96)
#define PI_MAX_Ud      (Standard_Work_Voltage_V*Sqrt_OF_3*0.5)   // 13.5
#define PI_MAX_Uq      (Standard_Work_Voltage_V*Sqrt_OF_3*0.96)

#define   Control_Mode_Stop      0     //ͣ��ģʽ
#define   Control_Mode_Speed     1     //�ٶȿ���ģʽ ����ת
#define   Control_Mode_Torq      2     //Ť�ؿ���ģʽ
#define   Control_Mode_VF        3     //VF����ģʽ
#define   Control_Mode_Vdq       4     //��ѹ����ģʽ
#define   Control_Mode_Idq       5     //��������ģʽ



#define   Speed_Grad_Timer 	    100		//�����ٶȵ��ݶ�ʱ��
#define   Speed_Grad_RPM    	5		//�����ٶȵ��ݶ�ֵ

#define   Trq_Grad_Timer 		100		//����Ť�ص��ݶ�ʱ��
#define   Trq_Grad_0D1NM    	2		//����Ť�ص��ݶ�ֵ

#define   Idq_Grad_Timer 		50		//����������ݶ�ʱ��
#define   Idq_Grad_0D1A      	0.05	//����������ݶ�ֵ

#define   Vdq_Grad_Timer 		50		//�����ѹ���ݶ�ʱ��
#define   Vdq_Grad_0D1V      	0.1		//�����ѹ���ݶ�ֵ

#define   VF_q_Grad_Timer 		50		//����������ݶ�ʱ��
#define   VF_q_Grad_0D1V      	0.1 	//����������ݶ�ֵ

#define   VF_F_Grad_Timer 		50		//�����ѹ���ݶ�ʱ��
#define   VF_F_Grad_0D1HZ      	0.1		//�����ѹ���ݶ�ֵ

#define   VF_F_Min 		        2		//����Ƶ�ʵ���Сֵ
#define   VF_F_Max      	    50		//����Ƶ�����ֵ

#define   FW_Grad_Timer 		20		//�������ŵ�ִ��ʱ��

#define   GM_Low_Lass_A 		0.218	//ͨ�õ�ͨ�˲�ϵ��A
#define   GM_Low_Lass_B    	    0.782	//ͨ�õ�ͨ�˲�ϵ��B

#define   MinSpeed_Grad         10// ��С����ת�ٷֶ�

#define   Stall_rpm_min         50
#define   Stall_rpm_max         90

#define   Stall_Trq_min         110
#define   Stall_Trq_max         125

#define   PHASE_Curr_max        40   // ���ߵ���
#define   BUS_Curr_max          20   // 20A��ĸ�ߵ���

#define   OVER_rpm_max          3300

#endif /* Motor_parameter_H*/
//===========================================================================
// End of file.
//===========================================================================
