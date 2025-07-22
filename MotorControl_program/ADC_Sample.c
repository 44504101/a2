//############################################################
// FILE:  ADC_Sample.c
// Created on: 2016年5月18日
// Author: XQ
// summary: ADC_Sample
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//版权所有，盗版必究
//DSP28335旋变PMSM电机控制与开发板
//硕历电子
//网址: https://shuolidianzi.taobao.com
//Author-QQ: 616264123
//电机控制QQ群：736369775
//############################################################

#include "Motorcontrol_include.h"
#include "User_CAN.h"

ADCSamp      ADCSampPara=ADCSamp_DEFAULTS;
Volt_Curr    Volt_CurrPara=Volt_Curr_DEFAULTS;


void  ADC_Sample(void)  // 母线电压和母线电流，相电流采集
{
	ADCSampPara.BUS_Curr = AdcMirror.ADCRESULT1;  // 母线电流，相电流，母线电压，电位器

	ADCSampPara.PhaseU_Curr =  AdcMirror.ADCRESULT3;
	ADCSampPara.PhaseV_Curr = AdcMirror.ADCRESULT2; // V和W相电流读取相同
	ADCSampPara.PhaseW_Curr =  AdcMirror.ADCRESULT2;

	ADCSampPara.BUS_Voltage = AdcMirror.ADCRESULT4;
	ADCSampPara.RP_speed_Voltage = AdcMirror.ADCRESULT5;
}


void ADC_Sample_deal(void)  //   相电流采集处理和母线电流电压等 相电流和母线电流传感器系数，计算实际值
{
  float32 BUS_Curr=0,PhaseU_Curr=0,PhaseV_Curr=0,PhaseW_Curr=0,BUS_Voltage=0;
  // DSP是0--3V 对应12位AD4096
  BUS_Curr = (float32)((int16)(ADCSampPara.BUS_Curr- ADCSampPara.OffsetBUS_Curr)/4096.0*1.5*BUS_Curr_Sensor_Range);
  PhaseU_Curr = (float32)((int16)(ADCSampPara.PhaseU_Curr- ADCSampPara.OffsetPhaseU_Curr)/4096.0*1.5*Phase_Curr_Sensor_Range);
  PhaseV_Curr = (float32)((ADCSampPara.PhaseV_Curr- ADCSampPara.OffsetPhaseV_Curr)/4096.0);
  BUS_Voltage=  ADCSampPara.BUS_Voltage*BUS_Voltage_Mult_Coeff +BUS_Voltage_Add_Coeff ;
  PhaseW_Curr= -(PhaseU_Curr+ PhaseV_Curr);  // 相电流Iu Iv  Iw和等于0

  //  一阶数字低通滤波器     假设 0.278+0.722=1     母线电压和电流采用相同滤波系数 此系数截止频率比较低
  // https://wenku.baidu.com/view/2d022b6b7375a417866f8f7e.html?from=search
  Volt_CurrPara.BUS_Curr=Volt_CurrPara.BUS_Curr*0.95+ BUS_Curr*0.05;   // 母线滤波大滤波
  Volt_CurrPara.PhaseU_Curr=PhaseU_Curr;
  Volt_CurrPara.PhaseV_Curr=PhaseV_Curr;
  Volt_CurrPara.PhaseW_Curr=PhaseW_Curr;
  Volt_CurrPara.BUS_Voltage = Volt_CurrPara.BUS_Voltage*0.95+ BUS_Voltage*0.05;   // 滤波

}


void Offset_CurrentReading(void)  // 相电流和母线电流偏执电压读取   ，校准零漂
{
	static Uint16 ADC_PhaseU_Curr[64];
	static Uint16 ADC_PhaseV_Curr[64];
	static Uint16 ADC_BUS_Curr[64];
	static Uint8 i=0;
  // 读取存储64个数据
	ADC_PhaseU_Curr[i] =  ADCSampPara.PhaseU_Curr;
	ADC_PhaseV_Curr[i] =  ADCSampPara.PhaseV_Curr;// 没有V相电流采样，V相电流改成W相 20160128
	ADC_BUS_Curr[i] =  ADCSampPara.BUS_Curr;

	i++;
	if(i>=64)
	i=0;

	if(CAN_ControlPara.Control_Mode ==0)//自动调零  把各参数的64个数据求和算平局
	{
		Uint32 sum_U=0;
		Uint32 sum_V=0;
		Uint32 sum_BUS=0;
		Uint16 i;
		for(i=0; i < 64; i++)
		{
			sum_U += ADC_PhaseU_Curr[i];   // 母线电流和相电流求和计算
			sum_V += ADC_PhaseV_Curr[i];
			sum_BUS += ADC_BUS_Curr[i];
		}
	     ADCSampPara.OffsetPhaseU_Curr = sum_U /64;
	     ADCSampPara.OffsetPhaseV_Curr = sum_V /64;
	     ADCSampPara.OffsetBUS_Curr = sum_BUS /64;
	}
}


void DWQ_Control_AD(void)  // 旋转电位器控制 采样0--4095 通过系数调整不同模式下的给定参数，转速扭矩电流电压等
{
	// 电位器给定转速，扭矩，VF电压频率，电压Vdq，电流Idq
	switch(CAN_ControlPara.Control_Mode)
	{
	case Control_Mode_Speed:
		CAN_ControlPara.SpeedRPM=(ADCSampPara.RP_speed_Voltage*0.6-40); //   4095*0.5-50  2000RPM;	  //速度控制模式速度输入
		// 若反转 把AD参数加负号  CAN_ControlPara.SpeedRPM=-(ADCSampPara.RP_speed_Voltage*0.5-50);
		if( CAN_ControlPara.SpeedRPM <-Motor_MAX_Speed_RPM ) // 限制最大最小转速
			CAN_ControlPara.SpeedRPM =-Motor_MAX_Speed_RPM;
		else if( CAN_ControlPara.SpeedRPM>Motor_MAX_Speed_RPM)
			CAN_ControlPara.SpeedRPM = Motor_MAX_Speed_RPM ;
		break;
	case Control_Mode_Torq: // Control_Mode_Trq
		CAN_ControlPara.Trq= (ADCSampPara.RP_speed_Voltage*0.6-40)*0.05; // 2300*0.05 约100Nm
		// 若反转 把AD参数加负号  CAN_ControlPara.SpeedRPM= -(ADCSampPara.RP_speed_Voltage*0.6-40)*0.05;  // 负扭矩反转
		if( CAN_ControlPara.Trq <-Motor_MAX_Trq_NM )  // 限制最大最小扭矩
			CAN_ControlPara.Trq =-Motor_MAX_Trq_NM;
		else if( CAN_ControlPara.Trq>Motor_MAX_Trq_NM)
			CAN_ControlPara.Trq = Motor_MAX_Trq_NM ;
		break;

	case Control_Mode_VF:                               //VF控制模式扭矩输入
		CAN_ControlPara.VF_freq =(float32) (ADCSampPara.RP_speed_Voltage*0.6-40)*0.03; //
		CAN_ControlPara.VF_Vq  = (float32)(ADCSampPara.RP_speed_Voltage*0.6-40)*0.002;
		if(CAN_ControlPara.VF_freq>30) //限制最大频率速度   50HZ
		CAN_ControlPara.VF_freq=30;
		if(CAN_ControlPara.VF_Vq>5) // 限制最大电压Vq
		CAN_ControlPara.VF_Vq=5;
		break;
	case Control_Mode_Vdq:         // 电压控制Us不超过6.5V
		CAN_ControlPara.V_d = 0;
		CAN_ControlPara.V_q = (ADCSampPara.RP_speed_Voltage*0.6-40)*0.0035;
		if(CAN_ControlPara.V_q>6.5)
		CAN_ControlPara.V_q=6.5;
		break;
	case Control_Mode_Idq:      // 电流模式需要限制电流指令控制，电流大扭矩就大，转速就会很高，会超速
		CAN_ControlPara.I_d_cmd = -(ADCSampPara.RP_speed_Voltage*0.6-40)*0.0008;  //加一点比例的弱磁-id
		CAN_ControlPara.I_q_cmd = (ADCSampPara.RP_speed_Voltage*0.6-40)*0.0015; //2300RPM   0--2.3A;
		if( CAN_ControlPara.I_d_cmd <-0.9 )  // 限制最大最小Idq电流
			CAN_ControlPara.I_d_cmd =-0.9;
		else if( CAN_ControlPara.I_d_cmd>0)
			CAN_ControlPara.I_d_cmd = 0;
		if( CAN_ControlPara.I_q_cmd >2.0 )  // 限制最大最小电流
			CAN_ControlPara.I_q_cmd =2.0;
		else if( CAN_ControlPara.I_q_cmd<-2.0)
			CAN_ControlPara.I_q_cmd = -2.0;
		break;
 }
}

//===========================================================================
// No more.
//===========================================================================
