//############################################################
// FILE: AD2S1205_Resolver.c
// Created on: 2017��12��10��
// Author: XQ
// summary: AD2S1205_Resolver
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//��Ȩ���У�����ؾ�
//DSP28335����PMSM��������뿪����
//˶������
//��ַ: https://shuolidianzi.taobao.com
//Author-QQ: 616264123
//�������QQȺ��736369775
//############################################################

#include "Motorcontrol_include.h"

AD2SXB      AD2SXBPare=AD2SXB_DEFAULTS;
bool IsOverCurrent() {
     _iq current = pi_iq.Out;  // ��ȡ��ǰ�����������Ҫ��������Ӳ��ʵ��
    return current >= CurrentThreshold;
}
void AD2S1205_Init(void)
{
	AD2SXBPare.init_Angle=442.2;  // �����λ
	 // ���䰲װʱ���������綯�ƹ������ھ������Ƕ�����λֵ
	 Delay_ms(5);     // Ӳ����ƽSOE�ߵ�ƽ���У�FS12�ߵ�ƽ������10KHZ
	 XB_NRST_OFF;
	 SAMPLE_ON;
	 DELAY_US(100);  // ����10us���ϵ縴λ
	 XB_NRST_ON;
	 Delay_ms(30);    //����20ms�ĵ�·�ȶ�ʱ��
	 SAMPLE_OFF;
	 Delay_ms(2) ;
	 SAMPLE_ON;
}

Uint16 Read_IO_interface(void)
{
	Uint16 AD2S_data=0;   // ��ת��ѹ���Ĳ�������12λ��ȡIO ��������IO��ȡ״̬
    AD2S_data=DB_0+(DB_1<<1)+(DB_2<<2)+(DB_3<<3)+(DB_4<<4)+(DB_5<<5)
    +(DB_6<<6)+(DB_7<<7)+(DB_8<<8)+(DB_9<<9)+(DB_10<<10)+(DB_11<<11);	  // 10US
	return  AD2S_data;
}

void AD2S1205_Read(void)
{
	volatile Uint8 AD2Si=0;
	Uint8 Fault_DOS=0,Fault_LOT=0;

	 SAMPLE_OFF;
	 Delay_100ns(6);  // 6/8.192M+20ns=500ns
	 CS_RD_OFF;
	 Delay_100ns(1);   // 100ns
	 RDVEL_ON;
	 AD2Si=1; AD2Si=2; // ͣ����ʱ2��ʱ��
	 Delay_100ns(1);   // 100n
	 AD2SXBPare.AD2S_angle =Read_IO_interface();  // ��ȡλ���ź�
     //AD2SXBPare.AD2S_angle_du = AD2SXBPare.AD2S_angle*0.0879;
     //AD2SXBPare.AD2S_circle += 1;
	   // ���䰲װʱ���������綯�ƹ������ھ������Ƕ�����λֵ
	 AD2SXBPare.PM_angle =(Uint16)(AD2SXBPare.AD2S_angle*Poles_Ratio_Coeff-AD2SXBPare.init_Angle)&(AD2SXB_12Bit);

	 Delay_100ns(1);   // 100ns
	 RDVEL_OFF;
	 AD2Si=1; AD2Si=2;  // ͣ����ʱ2��ʱ��
	 Delay_100ns(1);   // 100n
	 CS_RD_ON;
	 AD2Si=1; AD2Si=2;  // ͣ����ʱ2��ʱ��
	 Delay_100ns(1);   // 100n
	 CS_RD_OFF;
	 Delay_100ns(1);   // 100ns
	 AD2SXBPare.AD2S_Spd_RD =Read_IO_interface(); // ��ȡ�����ٶ��ź�ֵ
	 // �ź����ݸߵ�12λ�Ƿ����ź� 0����ת 1�Ƿ�ת���� ���Խ���������λ��ȡ�з���16���� ���ѷ������ȡ����Ȼ�������ƶ�4λ
	 AD2SXBPare.AD2S_Spd=((int16)(AD2SXBPare.AD2S_Spd_RD<<4))>>4;  //  Poles_Ratio_Coeff
	 AD2SXBPare.AD2S_SpdRPM=(int16)(AD2SXBPare.AD2S_Spd*((float32)(625/64)));
	 //AD2SXBPare.AD2S_SpdRPM=(int16)(AD2SXBPare.AD2S_Spd*((float32)(60*1000/2048)));
	 // ��������������伫������ϵ������    AD2SXBPare.AD2S_Spd*60*1000/2048/3  3= ��ת��ѹ��������
	 AD2SXBPare.AD2S_SpdRPMH=AD2SXBPare.AD2S_SpdRPMH*GM_Low_Lass_A + AD2SXBPare.AD2S_SpdRPM*GM_Low_Lass_B;  // ת��һ�ڵ�ͨ�˲�

	 //AD2SXBPare.AD2S_SpdRPMH=SpeedRpm_GXieLv.XieLv_Y+3;  //  ����ͨѶ
     // ��ȡ����DOS��LOT���ϣ����б�������
	 Fault_DOS=GPIO_DOS;
	 Fault_LOT=GPIO_LOT;

	 AD2SXBPare.AD2S_FaultDOSLOT=(Uint8)((Fault_DOS<<1)+Fault_LOT); // ���������ݴ���Ϊ2λ����
	 //AD2SXBPare.AD2S_FaultDOSLOT=Fault_DOS;

	 if(AD2SXBPare.AD2S_SpdRPMH>0.1)   // �ж�����ת
	 AD2SXBPare.Move_State=1;
	 else if(AD2SXBPare.AD2S_SpdRPMH<-0.1)
	 AD2SXBPare.Move_State=2;
	 else
	 AD2SXBPare.Move_State=0;  // ��������10RPM�ھ���Ϊͣ��

	 AD2SXBPare.Abs_SpdRPM=Abs(AD2SXBPare.AD2S_SpdRPMH);

	 CS_RD_ON;
	 RDVEL_ON;
	 SAMPLE_ON;
}


//===========================================================================
// No more.
//===========================================================================
