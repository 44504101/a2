//############################################################
// FILE:  ADC_Sample.c
// Created on: 2016��5��18��
// Author: XQ
// summary: ADC_Sample
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//��Ȩ���У�����ؾ�
//DSP28335����PMSM��������뿪����
//˶������
//��ַ: https://shuolidianzi.taobao.com
//Author-QQ: 616264123
//�������QQȺ��736369775
//############################################################

#include "Motorcontrol_include.h"
#include "User_CAN.h"

ADCSamp      ADCSampPara=ADCSamp_DEFAULTS;
Volt_Curr    Volt_CurrPara=Volt_Curr_DEFAULTS;


void  ADC_Sample(void)  // ĸ�ߵ�ѹ��ĸ�ߵ�����������ɼ�
{
	ADCSampPara.BUS_Curr = AdcMirror.ADCRESULT1;  // ĸ�ߵ������������ĸ�ߵ�ѹ����λ��

	ADCSampPara.PhaseU_Curr =  AdcMirror.ADCRESULT3;
	ADCSampPara.PhaseV_Curr = AdcMirror.ADCRESULT2; // V��W�������ȡ��ͬ
	ADCSampPara.PhaseW_Curr =  AdcMirror.ADCRESULT2;

	ADCSampPara.BUS_Voltage = AdcMirror.ADCRESULT4;
	ADCSampPara.RP_speed_Voltage = AdcMirror.ADCRESULT5;
}


void ADC_Sample_deal(void)  //   ������ɼ������ĸ�ߵ�����ѹ�� �������ĸ�ߵ���������ϵ��������ʵ��ֵ
{
  float32 BUS_Curr=0,PhaseU_Curr=0,PhaseV_Curr=0,PhaseW_Curr=0,BUS_Voltage=0;
  // DSP��0--3V ��Ӧ12λAD4096
  BUS_Curr = (float32)((int16)(ADCSampPara.BUS_Curr- ADCSampPara.OffsetBUS_Curr)/4096.0*1.5*BUS_Curr_Sensor_Range);
  PhaseU_Curr = (float32)((int16)(ADCSampPara.PhaseU_Curr- ADCSampPara.OffsetPhaseU_Curr)/4096.0*1.5*Phase_Curr_Sensor_Range);
  PhaseV_Curr = (float32)((ADCSampPara.PhaseV_Curr- ADCSampPara.OffsetPhaseV_Curr)/4096.0);
  BUS_Voltage=  ADCSampPara.BUS_Voltage*BUS_Voltage_Mult_Coeff +BUS_Voltage_Add_Coeff ;
  PhaseW_Curr= -(PhaseU_Curr+ PhaseV_Curr);  // �����Iu Iv  Iw�͵���0

  //  һ�����ֵ�ͨ�˲���     ���� 0.278+0.722=1     ĸ�ߵ�ѹ�͵���������ͬ�˲�ϵ�� ��ϵ����ֹƵ�ʱȽϵ�
  // https://wenku.baidu.com/view/2d022b6b7375a417866f8f7e.html?from=search
  Volt_CurrPara.BUS_Curr=Volt_CurrPara.BUS_Curr*0.95+ BUS_Curr*0.05;   // ĸ���˲����˲�
  Volt_CurrPara.PhaseU_Curr=PhaseU_Curr;
  Volt_CurrPara.PhaseV_Curr=PhaseV_Curr;
  Volt_CurrPara.PhaseW_Curr=PhaseW_Curr;
  Volt_CurrPara.BUS_Voltage = Volt_CurrPara.BUS_Voltage*0.95+ BUS_Voltage*0.05;   // �˲�

}


void Offset_CurrentReading(void)  // �������ĸ�ߵ���ƫִ��ѹ��ȡ   ��У׼��Ư
{
	static Uint16 ADC_PhaseU_Curr[64];
	static Uint16 ADC_PhaseV_Curr[64];
	static Uint16 ADC_BUS_Curr[64];
	static Uint8 i=0;
  // ��ȡ�洢64������
	ADC_PhaseU_Curr[i] =  ADCSampPara.PhaseU_Curr;
	ADC_PhaseV_Curr[i] =  ADCSampPara.PhaseV_Curr;// û��V�����������V������ĳ�W�� 20160128
	ADC_BUS_Curr[i] =  ADCSampPara.BUS_Curr;

	i++;
	if(i>=64)
	i=0;

	if(CAN_ControlPara.Control_Mode ==0)//�Զ�����  �Ѹ�������64�����������ƽ��
	{
		Uint32 sum_U=0;
		Uint32 sum_V=0;
		Uint32 sum_BUS=0;
		Uint16 i;
		for(i=0; i < 64; i++)
		{
			sum_U += ADC_PhaseU_Curr[i];   // ĸ�ߵ������������ͼ���
			sum_V += ADC_PhaseV_Curr[i];
			sum_BUS += ADC_BUS_Curr[i];
		}
	     ADCSampPara.OffsetPhaseU_Curr = sum_U /64;
	     ADCSampPara.OffsetPhaseV_Curr = sum_V /64;
	     ADCSampPara.OffsetBUS_Curr = sum_BUS /64;
	}
}


void DWQ_Control_AD(void)  // ��ת��λ������ ����0--4095 ͨ��ϵ��������ͬģʽ�µĸ���������ת��Ť�ص�����ѹ��
{
	// ��λ������ת�٣�Ť�أ�VF��ѹƵ�ʣ���ѹVdq������Idq
	switch(CAN_ControlPara.Control_Mode)
	{
	case Control_Mode_Speed:
		CAN_ControlPara.SpeedRPM=(ADCSampPara.RP_speed_Voltage*0.6-40); //   4095*0.5-50  2000RPM;	  //�ٶȿ���ģʽ�ٶ�����
		// ����ת ��AD�����Ӹ���  CAN_ControlPara.SpeedRPM=-(ADCSampPara.RP_speed_Voltage*0.5-50);
		if( CAN_ControlPara.SpeedRPM <-Motor_MAX_Speed_RPM ) // ���������Сת��
			CAN_ControlPara.SpeedRPM =-Motor_MAX_Speed_RPM;
		else if( CAN_ControlPara.SpeedRPM>Motor_MAX_Speed_RPM)
			CAN_ControlPara.SpeedRPM = Motor_MAX_Speed_RPM ;
		break;
	case Control_Mode_Torq: // Control_Mode_Trq
		CAN_ControlPara.Trq= (ADCSampPara.RP_speed_Voltage*0.6-40)*0.05; // 2300*0.05 Լ100Nm
		// ����ת ��AD�����Ӹ���  CAN_ControlPara.SpeedRPM= -(ADCSampPara.RP_speed_Voltage*0.6-40)*0.05;  // ��Ť�ط�ת
		if( CAN_ControlPara.Trq <-Motor_MAX_Trq_NM )  // ���������СŤ��
			CAN_ControlPara.Trq =-Motor_MAX_Trq_NM;
		else if( CAN_ControlPara.Trq>Motor_MAX_Trq_NM)
			CAN_ControlPara.Trq = Motor_MAX_Trq_NM ;
		break;

	case Control_Mode_VF:                               //VF����ģʽŤ������
		CAN_ControlPara.VF_freq =(float32) (ADCSampPara.RP_speed_Voltage*0.6-40)*0.03; //
		CAN_ControlPara.VF_Vq  = (float32)(ADCSampPara.RP_speed_Voltage*0.6-40)*0.002;
		if(CAN_ControlPara.VF_freq>30) //�������Ƶ���ٶ�   50HZ
		CAN_ControlPara.VF_freq=30;
		if(CAN_ControlPara.VF_Vq>5) // ��������ѹVq
		CAN_ControlPara.VF_Vq=5;
		break;
	case Control_Mode_Vdq:         // ��ѹ����Us������6.5V
		CAN_ControlPara.V_d = 0;
		CAN_ControlPara.V_q = (ADCSampPara.RP_speed_Voltage*0.6-40)*0.0035;
		if(CAN_ControlPara.V_q>6.5)
		CAN_ControlPara.V_q=6.5;
		break;
	case Control_Mode_Idq:      // ����ģʽ��Ҫ���Ƶ���ָ����ƣ�������Ť�ؾʹ�ת�پͻ�ܸߣ��ᳬ��
		CAN_ControlPara.I_d_cmd = -(ADCSampPara.RP_speed_Voltage*0.6-40)*0.0008;  //��һ�����������-id
		CAN_ControlPara.I_q_cmd = (ADCSampPara.RP_speed_Voltage*0.6-40)*0.0015; //2300RPM   0--2.3A;
		if( CAN_ControlPara.I_d_cmd <-0.9 )  // ���������СIdq����
			CAN_ControlPara.I_d_cmd =-0.9;
		else if( CAN_ControlPara.I_d_cmd>0)
			CAN_ControlPara.I_d_cmd = 0;
		if( CAN_ControlPara.I_q_cmd >2.0 )  // ���������С����
			CAN_ControlPara.I_q_cmd =2.0;
		else if( CAN_ControlPara.I_q_cmd<-2.0)
			CAN_ControlPara.I_q_cmd = -2.0;
		break;
 }
}

//===========================================================================
// No more.
//===========================================================================
