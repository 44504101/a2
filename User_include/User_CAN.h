//############################################################
// FILE:  User_CAN_H
// Created on: 2018��1��2��
// Author: XY
// summary:User_CAN.h  Header file  and definition
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//��Ȩ���У�����ؾ�
//DSP28335����PMSM��������뿪����
//˶������
//��ַ: https://shuolidianzi.taobao.com
//�޸�����:2017/7/5
//�汾��V17.2-1
//Author-QQ: 616264123
//�������QQȺ��736369775
//############################################################

#ifndef _User_CAN_H_
#define _User_CAN_H_

typedef struct {
	    Uint8	 BoMa_Control_Flag;  //���뿪�ؿ���ѡ���־λ
	    Uint8    drive_car;    //��λ�����Ƶ�������־
	    Uint8    olddrive_car;
	    Uint8    Control_Mode;        // ͨѶ����ģʽ
	    Uint8    Fault_Recover;         // ����  ���ϻָ�
	    float32  CANSpeed_RPM;
	    float32  CANTorq_Nm;
	    float32    SpeedRPM;	         //�ٶȿ���ģʽ�ٶ�����
	 	float32  Trq;                //Ť�ؿ���ģʽŤ������
	 	float32  VF_freq;            //VF����ģʽƵ������
	    float32  VF_Vq;	     	     //VF����ģʽ��ѹ����
	 	float32  V_d;                //Vdq����ģʽ��ѹVd����
	    float32  V_q;                //Vdq����ģʽ��ѹVq����
	    float32  I_d_cmd;                //Idq����ģʽ��ѹId����
	    float32  I_q_cmd;                //Idq����ģʽ��ѹIq����
	   }CAN_Control_Order;

#define CAN_Control_Order_DEFAULTS  {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}  // ��ʼ������


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
