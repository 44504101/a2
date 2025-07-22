//############################################################
// FILE:  User_CAN_H
// Created on: 2018年1月2日
// Author: XY
// summary:User_CAN.h  Header file  and definition
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//版权所有，盗版必究
//DSP28335旋变PMSM电机控制与开发板
//硕历电子
//网址: https://shuolidianzi.taobao.com
//修改日期:2017/7/5
//版本：V17.2-1
//Author-QQ: 616264123
//电机控制QQ群：736369775
//############################################################

#ifndef _User_CAN_H_
#define _User_CAN_H_

typedef struct {
	    Uint8	 BoMa_Control_Flag;  //拨码开关控制选择标志位
	    Uint8    drive_car;    //电位器控制的启动标志
	    Uint8    olddrive_car;
	    Uint8    Control_Mode;        // 通讯控制模式
	    Uint8    Fault_Recover;         // 控制  故障恢复
	    float32  CANSpeed_RPM;
	    float32  CANTorq_Nm;
	    float32    SpeedRPM;	         //速度控制模式速度输入
	 	float32  Trq;                //扭矩控制模式扭矩输入
	 	float32  VF_freq;            //VF控制模式频率输入
	    float32  VF_Vq;	     	     //VF控制模式电压输入
	 	float32  V_d;                //Vdq控制模式电压Vd输入
	    float32  V_q;                //Vdq控制模式电压Vq输入
	    float32  I_d_cmd;                //Idq控制模式电压Id输入
	    float32  I_q_cmd;                //Idq控制模式电压Iq输入
	   }CAN_Control_Order;

#define CAN_Control_Order_DEFAULTS  {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}  // 初始化参数


extern CAN_Control_Order  CAN_ControlPara;

void mailbox_send(Uint16 MBXnbS,Uint32 CANID, Uint8 *CAN_Buff);
void ECANB_MB1_send(void);
void ECANB_MB2_send(void);
void ECANB_MB3_send(void);
void ECANB_MB4_send(void);
void ECANB_MB5_send(void);
void CAN_Rcv1(void);
void CAN_Rcv2(void);

void CAN_ControlMotor_CMD(void); //
#endif  //Task_manager_H
