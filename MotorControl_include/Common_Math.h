//############################################################
// FILE: Common_Math.h
// Created on: 2017��12��10��
// Author: XQ
// summary:    Header file  and definition
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//��Ȩ���У�����ؾ�
//DSP28335����PMSM��������뿪����
//˶������
//��ַ: https://shuolidianzi.taobao.com
//Author-QQ: 616264123
//�������QQȺ��736369775
//############################################################

#ifndef _Common_Math_H_
#define _Common_Math_H_

#define Abs(A)    ((A>=0)?A:-A)  // ����ֵ����
#define Min(A,B)  ((A<=B)?A:B)   // ����С����
#define Max(A,B)  ((A>=B)?A:B)   // �������


typedef struct
{
	Uint16  table_Angle;
	float32 table_Sin;
	float32 table_Cos;
}Ang_SinCos, *p_Ang_SinCos;

#define  Ang_SinCos_DEFAULTS    {0,0.0,0.0}

typedef struct 	{
	        float32  Alpha; 	//���ྲֹ����ϵ Alpha ��
	        float32  Beta;		//���ྲֹ����ϵ Beta ��
	        float32  IQTan;		//IQ��ʽ���� 45��������1��IQ�ĸ�ʽ��
  			int16  IQAngle;	//IQ��ʽ�Ƕ�ֵ 0---65536 == 0---360��
         } IQAtan , *p_IQAtan;

#define IQAtan_DEFAULTS  {0,0,0,0}  // ��ʼ������

typedef struct
 {  //ָ�������б�ʴ���
    float32  XieLv_X;   // ָ�����б���������x
	float32  XieLv_Y;
	float32  XieLv_Grad;
	Uint8    Timer_Count;
	Uint8    Grad_Timer;
 }GXieLv, *p_GXieLv;

 #define  GXieLv_DEFAULTS    {0.0,0.0,0.0,0}


void LookUp_CosSin(void);
float32  Limit_Sat( float32 Uint,float32 U_max, float32 U_min); //���Ƹ�ֵ����
Uint32   IQSqrt(Uint32  M);
void Atan_Cale(p_IQAtan pV);  // ��ȡ�����Һ���
void SinCos_Table(p_Ang_SinCos PV);
void Grad_XieLv(p_GXieLv pV);
void Delay_ms(Uint32 t);
void Delay_us(Uint32 t);
void Delay_100ns(Uint32 t); // Delay_us

#endif /* SIN_COS_TABLE1_H_ */
