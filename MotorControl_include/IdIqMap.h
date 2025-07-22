//############################################################
// FILE: IdIqMap.h
// Created on: 2018年01月20日
// Author: XQ
//summary:   Header file  and definition
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//TC1782电机控制与开发板
//硕历电子
//网址: https://shuolidianzi.taobao.com
//英飞凌MCU与电机控制QQ群：  643127024
//############################################################

#ifndef _IDIAMAP_H_ 
#define _IDIAMAP_H_ 

#define Motor_MAX_Trq_Count ( 100+1) 

#define Motor_MAX_Speed_Count 40

extern const float32 Id_Map[Motor_MAX_Speed_Count][Motor_MAX_Trq_Count];
extern const float32 Iq_Map[Motor_MAX_Speed_Count][Motor_MAX_Trq_Count];

#endif /* idqmap*/
