//############################################################
// FILE:  User_CAN.c
// Created on: 2018年1月2日
// Author: XY
// summary:User_CAN.c
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//版权所有，盗版必究
//DSP28335旋变PMSM电机控制与开发板
//硕历电子
//网址: https://shuolidianzi.taobao.com
//修改日期:2018/1/5
//版本：V18.1
//Author-QQ: 616264123
//电机控制QQ群：736369775
//############################################################
 
#include "Main_PMSM.h"

CAN_Control_Order  CAN_ControlPara=CAN_Control_Order_DEFAULTS;

#define  MB1_CANID    0x04000000   // identifier 11-bit:   发送 ：ID   0x100
#define  MB2_CANID    0x04040000   // identifier 11-bit:   发送 ：ID   0x101
#define  MB3_CANID    0x04080000   // identifier 11-bit:   发送 ：ID   0x102
#define  MB4_CANID    0x040C0000   // identifier 11-bit:   发送 ：ID   0x103
#define  MB5_CANID    0x04100000   // identifier 11-bit:   发送 ：ID   0x104

void mailbox_send(Uint16 MBXnbS,Uint32 CANID, Uint8 *CAN_Buff)   //波特率500K 标准帧  只发送0x568帧 做了屏蔽位
{
      volatile struct MBOX *Mailboxsend;
      Uint32  data_L32=0,data_H32=0;

      Mailboxsend = &ECanbMboxes.MBOX0 +MBXnbS;
      data_L32= (Uint32)(CAN_Buff[0]*16777216+(CAN_Buff[1]*65536)+(CAN_Buff[2]*256)+(CAN_Buff[3])); //
      data_H32= (Uint32)(CAN_Buff[4]*16777216+(CAN_Buff[5]*65536)+(CAN_Buff[6]*256)+(CAN_Buff[7]));
      Mailboxsend->MDL.all= data_L32; //
      Mailboxsend->MDH.all= data_H32;
      Mailboxsend->MSGID.all=CANID;// 0X15A00000  0x568
}

void ECANB_MB1_send(void)
{
    Uint8  data_buff[8]={0};  // volatile
	Uint16 data_arrange=0;   //  AD2SXBPare.AD2S_FaultDOSLOT

	ECanbRegs.CANTA.bit.TA1 = 0;  // Clear all TAn

	if((CAN_ControlPara.Control_Mode==1)||(CAN_ControlPara.Control_Mode==2))
	data_arrange= pi_spd.Ref;
    else
	data_arrange= ADCSampPara.RP_speed_Voltage;
	data_buff[0]=(Uint8)(data_arrange&0x00FF);
	data_buff[1]=(Uint8)((data_arrange>>8)&0x00FF);

	data_buff[2]=(Uint8)(ADCSampPara.OffsetBUS_Curr&0x00FF);  //
	data_buff[3]=(Uint8)((ADCSampPara.OffsetBUS_Curr>>8)&0x00FF);  //
	data_buff[4]=(Uint8)(ADCSampPara.OffsetPhaseU_Curr&0x00FF);
	data_buff[5]=(Uint8)((ADCSampPara.OffsetPhaseU_Curr>>8)&0x00FF);
	data_buff[6]=(Uint8)(ADCSampPara.OffsetPhaseV_Curr&0x00FF);
	data_buff[7]=(Uint8)((ADCSampPara.OffsetPhaseV_Curr>>8)&0x00FF);

	mailbox_send(1, MB1_CANID, data_buff);
	DSP28x_usDelay(1);  // 延时等待1us
	ECanbRegs.CANTRS.bit.TRS1 = 1;  // Set TRS for all transmit mailboxes
}

void ECANB_MB2_send(void)
{
    Uint8  data_buff[8]={0};  // volatile
	int16 data_arrange=0;   //  AD2SXBPare.AD2S_FaultDOSLOT

	ECanbRegs.CANTA.bit.TA2 = 0;  // Clear all TAn
	data_arrange =(int16)(Volt_CurrPara.BUS_Curr*10);
	data_buff[0]=(Uint8)(data_arrange&0x00FF);
	data_buff[1]=(Uint8)((data_arrange>>8)&0x00FF);

	data_arrange= (int16)(Volt_CurrPara.PhaseU_Curr*10);
	data_buff[2]=(Uint8)(data_arrange&0x00FF);
	data_buff[3]=(Uint8)((data_arrange>>8)&0x00FF);

	data_arrange= (int16)(Volt_CurrPara.PhaseV_Curr*10);
	data_buff[4]=(Uint8)(data_arrange&0x00FF);
	data_buff[5]=(Uint8)((data_arrange>>8)&0x00FF);

	data_arrange= (int16)(Volt_CurrPara.PhaseW_Curr*10);
	data_buff[6]=(Uint8)(data_arrange&0x00FF);
	data_buff[7]=(Uint8)((data_arrange>>8)&0x00FF);

	mailbox_send(2, MB2_CANID, data_buff);
	DSP28x_usDelay(1);    //延时等待1us
	ECanbRegs.CANTRS.bit.TRS2 = 1;  // Set TRS for all transmit mailboxes
}

void ECANB_MB3_send(void)
{
    Uint8  data_buff[8]={0};  // volatile
	int16 data_arrange=0;   //  AD2SXBPare.AD2S_FaultDOSLOT

	ECanbRegs.CANTA.bit.TA3 = 0;  // Clear all TAn
	data_arrange = (Uint16)(Volt_CurrPara.BUS_Voltage*10);
	data_buff[0]=(Uint8)(data_arrange&0x00FF);
	data_buff[1]=(Uint8)((data_arrange>>8)&0x00FF);

	data_arrange= (int16)(Control_FB_Para.SpeedRPM);  // AD2S_Spd  Control_FB_Para.SpeedRPM  AD2SXBPare.Abs_SpdRPM
	data_buff[2]=(Uint8)(data_arrange&0x00FF);
	data_buff[3]=(Uint8)((data_arrange>>8)&0x00FF);

	data_arrange= (Uint16)(AD2SXBPare.PM_angle);    //ipark1.Angle  Control_FB_Para.PM_Angle
	data_buff[4]=(Uint8)(data_arrange&0x00FF);
	data_buff[5]=(Uint8)((data_arrange>>8)&0x00FF);

	data_buff[6]=(Uint8)(Control_FB_Para.Fault_DTC&0x00FF);
	data_buff[7]=(Uint8)( CAN_ControlPara.Control_Mode&0x00FF);  // CAN_ControlPara.RP_Control_Flag

	mailbox_send(3, MB3_CANID, data_buff);
	DSP28x_usDelay(1);    //延时等待1us
	ECanbRegs.CANTRS.bit.TRS3 = 1;  // Set TRS for all transmit mailboxes
}

void ECANB_MB4_send(void)
{
    Uint8  data_buff[8]={0};  // volatile
	int16 data_arrange=0;   //  AD2SXBPare.AD2S_FaultDOSLOT

	ECanbRegs.CANTA.bit.TA4 = 0;  // Clear all TAn
	data_arrange = (int16)(Control_FB_Para.Trq);
	data_buff[0]=(Uint8)(data_arrange&0x00FF);
	data_buff[1]=(Uint8)((data_arrange>>8)&0x00FF);

	data_arrange= (int16)(Control_FB_Para.VF_Freq*10);
	data_buff[2]=(Uint8)(data_arrange&0x00FF);
	data_buff[3]=(Uint8)((data_arrange>>8)&0x00FF);

	data_arrange= (int16)(Control_FB_Para.V_d*10);
	data_buff[4]=(Uint8)(data_arrange&0x00FF);
	data_buff[5]=(Uint8)((data_arrange>>8)&0x00FF);

	data_arrange= (int16)(Control_FB_Para.V_q*10);
	data_buff[6]=(Uint8)(data_arrange&0x00FF);
	data_buff[7]=(Uint8)((data_arrange>>8)&0x00FF);

	mailbox_send(4, MB4_CANID, data_buff);
	DSP28x_usDelay(1);    //延时等待1us
	ECanbRegs.CANTRS.bit.TRS4 = 1;  // Set TRS for all transmit mailboxes
}

void ECANB_MB5_send(void)
{
    Uint8  data_buff[8]={0};  // volatile
	int16 data_arrange=0;   //  AD2SXBPare.AD2S_FaultDOSLOT

	ECanbRegs.CANTA.bit.TA5 = 0;  // Clear all TAn
	data_arrange = (int16)(Control_FB_Para.I_d_fb*10);
	data_buff[0]=(Uint8)(data_arrange&0x00FF);
	data_buff[1]=(Uint8)((data_arrange>>8)&0x00FF);

	data_arrange= (int16)(Control_FB_Para.I_q_fb*10);
	data_buff[2]=(Uint8)(data_arrange&0x00FF);
	data_buff[3]=(Uint8)((data_arrange>>8)&0x00FF);

	data_arrange= (int16)(Control_FB_Para.I_d*10);
	data_buff[4]=(Uint8)(data_arrange&0x00FF);
	data_buff[5]=(Uint8)((data_arrange>>8)&0x00FF);

	data_arrange= (int16)(Control_FB_Para.I_q*10);
	data_buff[6]=(Uint8)(data_arrange&0x00FF);
	data_buff[7]=(Uint8)((data_arrange>>8)&0x00FF);

	mailbox_send(5, MB5_CANID, data_buff);
	DSP28x_usDelay(1);    //延时等待1us
	ECanbRegs.CANTRS.bit.TRS5 = 1;  // Set TRS for all transmit mailboxes
}



void CAN_Rcv1(void)//
{
    Uint8   Rec_Buff[8]={0};
    Uint32  TestMbox1=0,TestMbox2=0;

    if(ECanbRegs.CANRMP.bit.RMP11==0)
    	return;
    else if( ECanbMboxes.MBOX11.MSGID.all == 0x45400000) // ID=0X150
    {
    // DSP28x_usDelay(1);  // 延时等待1us
 	 ECanbRegs.CANRMP.bit.RMP11= 1;  //
 	 TestMbox1 = ECanbMboxes.MBOX11.MDL.all;
 	 TestMbox2 = ECanbMboxes.MBOX11.MDH.all;

 	 Rec_Buff[0]=(Uint8)((TestMbox1>>24)&0x00FF);
 	 Rec_Buff[1]=(Uint8)((TestMbox1>>16)&0x00FF);
 	 Rec_Buff[2]=(Uint8)((TestMbox1>>8)&0x00FF);
 	 Rec_Buff[3]=(Uint8)((TestMbox1)&0x00FF);
 	 Rec_Buff[4]=(Uint8)((TestMbox2>>24)&0x00FF);
 	 Rec_Buff[5]=(Uint8)((TestMbox2>>16)&0x00FF);
 	 Rec_Buff[6]=(Uint8)((TestMbox2>>8)&0x00FF);
 	 Rec_Buff[7]=(Uint8)((TestMbox2)&0x00FF);

	 CAN_ControlPara.Control_Mode = Rec_Buff[0] ;
	 CAN_ControlPara.Fault_Recover = Rec_Buff[1] ;

	 CAN_ControlPara.CANSpeed_RPM   = (int16)(Rec_Buff[2]+ Rec_Buff[3]*256);
	 CAN_ControlPara.CANTorq_Nm   = (int16)(Rec_Buff[4]+ Rec_Buff[5]*256);

    }
}

void CAN_ControlMotor_CMD(void)//
{
	switch(CAN_ControlPara.Control_Mode)
	{
	case Control_Mode_Speed:
		CAN_ControlPara.SpeedRPM= CAN_ControlPara.CANSpeed_RPM;	  //速度控制模式速度输入
		if( CAN_ControlPara.SpeedRPM <-Motor_MAX_Speed_RPM )
			CAN_ControlPara.SpeedRPM =-Motor_MAX_Speed_RPM;
		else if( CAN_ControlPara.SpeedRPM>Motor_MAX_Speed_RPM)
			CAN_ControlPara.SpeedRPM = Motor_MAX_Speed_RPM ;
		break;
	case Control_Mode_Torq: // Control_Mode_Trq
		CAN_ControlPara.Trq=(float32)(CAN_ControlPara.CANTorq_Nm); //扭矩控制模式扭矩输入   // 此扭矩与真实扭矩放大100倍
		if( CAN_ControlPara.Trq <-Motor_MAX_Trq_NM )
			CAN_ControlPara.Trq =-Motor_MAX_Trq_NM;
		else if( CAN_ControlPara.Trq>Motor_MAX_Trq_NM)
			CAN_ControlPara.Trq = Motor_MAX_Trq_NM ;
		break;

		//VF Vdq Idq三种控制模式的CAN控制由客户自行开发 ，把上位机Labview的控制模式打开启用
	/*
	case Control_Mode_VF:                               //VF控制模式扭矩输入
		CAN_ControlPara.VF_freq =(float32)(Order_Param1*0.1);
		CAN_ControlPara.VF_Vq  = (float32)(Order_Param2*0.1);
		if(CAN_ControlPara.VF_freq>50) //限制最大频率速度   50HZ
		CAN_ControlPara.VF_freq=50;
		if(CAN_ControlPara.VF_Vq>5) // 限制最大电压Vq
		CAN_ControlPara.VF_Vq=5;
		break;
	case Control_Mode_Vdq:
		CAN_ControlPara.V_d = -(float32)(Order_Param1*0.1);
		CAN_ControlPara.V_q = (float32)(Order_Param2*0.1);
		if(CAN_ControlPara.V_q>6.5)
		CAN_ControlPara.V_q=6.5;
		break;
	case Control_Mode_Idq:
		CAN_ControlPara.I_d_cmd = -(float32)(Order_Param1*0.1);
		CAN_ControlPara.I_q_cmd = (float32)(Order_Param2*0.1);
		if( CAN_ControlPara.I_q_cmd <-0.9 )  // 限制最大最小Idq电流
			CAN_ControlPara.I_q_cmd =-0.9;
		else if( CAN_ControlPara.I_q_cmd>0)
			CAN_ControlPara.I_q_cmd = 0;
		if( CAN_ControlPara.I_q_cmd <-2.5 )  // 限制最大最小电流
			CAN_ControlPara.I_q_cmd =-2.5;
		else if( CAN_ControlPara.I_q_cmd>2.5)
			CAN_ControlPara.I_q_cmd = -2.5;
		break;
	*/
	}
}


void CAN_Rcv2(void)//
{
    Uint32  TestMbox1=0,TestMbox2=0;
    volatile  Uint8   Rec_Buff[8]={0};
	if(ECanbRegs.CANRMP.bit.RMP12==0)
    	return;
    else if( ECanbMboxes.MBOX12.MSGID.all == 0x45400000)
    {
 	 ECanbRegs.CANRMP.bit.RMP12= 1;  //
	 TestMbox1 = ECanbMboxes.MBOX12.MDL.all;
	 TestMbox2 = ECanbMboxes.MBOX12.MDH.all;

 	 Rec_Buff[0]=(Uint8)((TestMbox1>>24)&0x00FF);
 	 Rec_Buff[1]=(Uint8)((TestMbox1>>16)&0x00FF);
 	 Rec_Buff[2]=(Uint8)((TestMbox1>>8)&0x00FF);
 	 Rec_Buff[3]=(Uint8)((TestMbox1)&0x00FF);
 	 Rec_Buff[4]=(Uint8)((TestMbox2>>24)&0x00FF);
 	 Rec_Buff[5]=(Uint8)((TestMbox2>>16)&0x00FF);
 	 Rec_Buff[6]=(Uint8)((TestMbox2>>8)&0x00FF);
 	 Rec_Buff[7]=(Uint8)((TestMbox2)&0x00FF);

    }
}



// USER CODE END
