//############################################################
// FILE: IdIqMap.h
// Created on: 2018��01��20��
// Author: XQ
//summary:   Header file  and definition
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//TC1782��������뿪����
//˶������
//��ַ: https://shuolidianzi.taobao.com
//Ӣ����MCU��������QQȺ��  643127024
//############################################################

#ifndef _IDIAMAP_H_ 
#define _IDIAMAP_H_ 

#define Motor_MAX_Trq_Count ( 100+1) 

#define Motor_MAX_Speed_Count 40

extern const float32 Id_Map[Motor_MAX_Speed_Count][Motor_MAX_Trq_Count];
extern const float32 Iq_Map[Motor_MAX_Speed_Count][Motor_MAX_Trq_Count];

#endif /* idqmap*/
