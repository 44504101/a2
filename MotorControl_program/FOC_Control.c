//############################################################
// FILE:  FOC_Control.c
// Created on: 2017��5��18��
// Author: XQ
// summary:  FOC_Control.c
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

Control_FB   Control_FB_Para=Control_FB_DEFAULTS;
CESHIIIII     CESHIIIII565=CESHIIIII_DEFAULTS;
GXieLv       SpeedRpm_GXieLv=GXieLv_DEFAULTS;
GXieLv       Trq_0D1NM_GXieLv=GXieLv_DEFAULTS;
GXieLv       VF_Freq_GXieLv=GXieLv_DEFAULTS;
GXieLv       VF_Vq_GXieLv=GXieLv_DEFAULTS;
GXieLv       V_d_GXieLv=GXieLv_DEFAULTS;
GXieLv       V_q_GXieLv=GXieLv_DEFAULTS;
GXieLv       I_d_GXieLv=GXieLv_DEFAULTS;
GXieLv       I_q_GXieLv=GXieLv_DEFAULTS;
CLARKE       clark1=CLARKE_DEFAULTS;
PARK         park1=PARK_DEFAULTS;
Ang_SinCos   Park_SinCos=Ang_SinCos_DEFAULTS;
Ang_SinCos   IPark_SinCos=Ang_SinCos_DEFAULTS;
IPARK        ipark1=IPARK_DEFAULTS;
SVGEN        svpwm1=SVGEN_DEFAULTS;


Uint8     Spd_Count=0,FW_Count=0;    // �ٶȻ�ִ�����ڼ���
float32   FW_Curr_d=0,V_d=0,V_q=0, VF_Angle=0;    // ��ѹVs����PI�����d�����ŵ���
Uint16  VF_AngleJZ=0,Park_Angle=0;

// CAN_ControlPara.Control_Mode=3;  �޸Ĵ˱������޸Ŀ���ģʽ 1 2  3  4
//  ����ģʽ ����ѡ��  1Ϊ�ٶȻ���ת   2Ϊ�ٶȻ���ת      3ΪVF����    4Ϊ��ѹ����


void  UVW_Axis_dq(void)   // ���������dq������仯
{
	 clark1.As  =Volt_CurrPara.PhaseU_Curr;   // �����Ķ�������������CLARKE�仯
	 clark1.Bs = Volt_CurrPara.PhaseV_Curr;
	 CLARKE_Cale((p_CLARKE)&clark1); //  Clarke�任���� �ȷ�ֵ�任

	 Park_Angle=AD2SXBPare.PM_angle;
	 park1.Angle=Park_Angle;

	 Park_SinCos.table_Angle= park1.Angle;  // ��ȡ����Ķ�Ӧ������ż�λ��
	 SinCos_Table((p_Ang_SinCos)&Park_SinCos);   // ����λ�ö�ȡ������ֵ
	 park1.Sine =Park_SinCos.table_Sin;
	 park1.Cosine =Park_SinCos.table_Cos;  //��ز����������PARK�仯�õ�dq�����
	 park1.Alpha =clark1.Alpha;
	 park1.Beta=clark1.Beta;
	 PARK_Cale((p_PARK)&park1);
}

void Speed_FOC(void) // �ٶȿ���ģʽ
{
	 SpeedRpm_GXieLv.XieLv_Grad=Speed_Grad_RPM;  // �ٶ�������ݶȼ���
	 SpeedRpm_GXieLv.Grad_Timer=Speed_Grad_Timer;// 2ms�ٶȻ�����
	 SpeedRpm_GXieLv.Timer_Count++;
	 if(SpeedRpm_GXieLv.Timer_Count>SpeedRpm_GXieLv.Grad_Timer)
	 {
	  SpeedRpm_GXieLv.Timer_Count=0;
	  Grad_XieLv((p_GXieLv)&SpeedRpm_GXieLv);
	  pi_spd.Ref =SpeedRpm_GXieLv.XieLv_Y;   // �ٶȻ�����ֵ
	  pi_spd.Fbk =AD2SXBPare.AD2S_SpdRPMH;

	  //PI_Controller((p_PI_Control)&pi_spd);  // �ٶȻ�PI����
	  //pi_spd.OutF=pi_spd.OutF*GM_Low_Lass_A+pi_spd.Out*GM_Low_Lass_B; //��·�˲������
	 }
	 //Trq_0D1NM_GXieLv.XieLv_X=pi_spd.OutF;
}

void FW_PI(void)    // ���ŵ��Ժ���   // pi_FW
{
	FW_Count++;
	if(FW_Count>FW_Grad_Timer )
	{
		FW_Count=0;
	  pi_FW.Ref = Vdc_s;
	  pi_FW.Fbk = Control_FB_Para.V_s;  //����VS
	  //PI_Controller((p_PI_Control)&pi_FW);
	  //pi_FW.OutF=pi_FW.OutF*GM_Low_Lass_A+pi_FW.Out*GM_Low_Lass_B;
	  //FW_Curr_d=pi_FW.OutF;
	}
}


void Trq_FOC(void)    //Ť�ؿ���ģʽ  // pi_FW
{
	Uint16  MinSpeed_Section;
	Uint16  Speed_Index=0;
	Uint16  Abs_FOC_Trq=0;
	Uint16  FOC_Trq_Index=0;
	Trq_0D1NM_GXieLv.XieLv_Grad=Trq_Grad_0D1NM;  //�����ٶȵ��ݶ�ֵ
	Trq_0D1NM_GXieLv.Grad_Timer=Trq_Grad_Timer;  //�����ٶȵ��ݶ�ʱ��
	Trq_0D1NM_GXieLv.Timer_Count++;
	if(Trq_0D1NM_GXieLv.Timer_Count>Trq_0D1NM_GXieLv.Grad_Timer)
	{
	//float32 Id_Table_CMD=0,Iq_Table_CMD=0;

	Trq_0D1NM_GXieLv.Timer_Count=0;
	Grad_XieLv((p_GXieLv)&Trq_0D1NM_GXieLv);
	MinSpeed_Section=(Uint16)(AD2SXBPare.Abs_SpdRPM/MinSpeed_Grad);
	Speed_Index=(Uint16)( MinSpeed_Section/MinSpeed_Grad);
	Uint16 Speed_Index_Offset=(Uint16)( MinSpeed_Section%MinSpeed_Grad);

	Abs_FOC_Trq  =(Uint16)(Abs(Trq_0D1NM_GXieLv.XieLv_Y));
	FOC_Trq_Index = Abs_FOC_Trq;

	if(Speed_Index > Motor_MAX_Speed_Count-2)
	Speed_Index=Motor_MAX_Speed_Count-2;
		//Speed_Index=8;
	if(FOC_Trq_Index> Motor_MAX_Trq_Count-2)
	FOC_Trq_Index= Motor_MAX_Trq_Count-2;
		//FOC_Trq_Index= 24;
	if(Speed_Index> Motor_MAX_Speed_Count-2)
	Speed_Index= Motor_MAX_Speed_Count-2;
		//Speed_Index=8;

	//Id���
	CESHIIIII565.Id_Value =(float32)( Id_Map[Speed_Index][FOC_Trq_Index]);
	CESHIIIII565.Next_Id_Value=(float32)( Id_Map[Speed_Index+1][FOC_Trq_Index]);
	CESHIIIII565.Id_Table_CMD =(float32)(CESHIIIII565.Id_Value+(float32)((CESHIIIII565.Next_Id_Value-CESHIIIII565.Id_Value)*Speed_Index_Offset/MinSpeed_Grad));

	CESHIIIII565.Iq_Value= (float32)(Iq_Map[Speed_Index][FOC_Trq_Index]);
	CESHIIIII565.Next_Iq_Value =(float32) (Iq_Map[Speed_Index+1][FOC_Trq_Index]);
	CESHIIIII565.Iq_Table_CMD =(float32)(CESHIIIII565.Iq_Value+(float32)((CESHIIIII565.Next_Iq_Value-CESHIIIII565.Iq_Value)*Speed_Index_Offset/MinSpeed_Grad));

	 FW_Curr_d=0;
	 I_d_GXieLv.XieLv_X = CESHIIIII565.Id_Table_CMD+FW_Curr_d;

	 if(Trq_0D1NM_GXieLv.XieLv_Y>=0)
	 I_q_GXieLv.XieLv_X = CESHIIIII565.Iq_Table_CMD ;
	 else
	 I_q_GXieLv.XieLv_X = -CESHIIIII565.Iq_Table_CMD ;
	}
}

void VF_Control(void)    //VF����ģʽ
{
	float32 PWM_Step_Angle	=0;
	VF_Freq_GXieLv.XieLv_Grad=VF_F_Grad_0D1HZ ;  //�����ٶȵ��ݶ�ֵ
	VF_Freq_GXieLv.Grad_Timer=VF_F_Grad_Timer;  //�����ٶȵ��ݶ�ʱ��
	VF_Vq_GXieLv.XieLv_Grad=VF_q_Grad_0D1V;  //�����ٶȵ��ݶ�ֵ
	VF_Vq_GXieLv.Grad_Timer=VF_q_Grad_Timer;  //�����ٶȵ��ݶ�ʱ��

	VF_Freq_GXieLv.Timer_Count++;
	if(VF_Freq_GXieLv.Timer_Count>VF_Freq_GXieLv.Grad_Timer)
	{
	 VF_Freq_GXieLv.Timer_Count=0;
	 Grad_XieLv((p_GXieLv)&VF_Freq_GXieLv);
	 Grad_XieLv((p_GXieLv)&VF_Vq_GXieLv);
	}
     float32 VF_Freq_EX=Limit_Sat(VF_Freq_GXieLv.XieLv_Y,VF_F_Max,VF_F_Min );
	//PWM_Step_Angle =(float32)(4096/((float32)(PWM_FREQ/VF_Freq_EX)));
   // VF_Angle+=PWM_Step_Angle;
    if(VF_Angle>=4096)
    VF_Angle-=4096;
    else if(VF_Angle<0)
    VF_Angle+=4096;
    VF_AngleJZ =(((Uint16)VF_Angle)&AD2SXB_12Bit);

	V_d=0;
	V_q=VF_Vq_GXieLv.XieLv_Y;
}

void Vdq_FOC(void)    //��ѹ����ģʽ
{
	float32 Us_Limit=0;
	V_d_GXieLv.XieLv_Grad=Vdq_Grad_0D1V;  //�����ٶȵ��ݶ�ֵ
	V_d_GXieLv.Grad_Timer=Vdq_Grad_Timer;  //�����ٶȵ��ݶ�ʱ��
	V_q_GXieLv.XieLv_Grad=Vdq_Grad_0D1V;  //�����ٶȵ��ݶ�ֵ
	V_q_GXieLv.Grad_Timer=Vdq_Grad_Timer;  //�����ٶȵ��ݶ�ʱ��
	V_d_GXieLv.Timer_Count++;
	if(V_d_GXieLv.Timer_Count>V_d_GXieLv.Grad_Timer)
	{
	   V_d_GXieLv.Timer_Count=0;
	   Grad_XieLv((p_GXieLv)&V_d_GXieLv);
       Grad_XieLv((p_GXieLv)&V_q_GXieLv);
	}

	V_d=V_d_GXieLv.XieLv_Y;
	Us_Limit=(IQSqrt(Vdc_s*Vdc_s*100-V_d*V_d*100))*0.1;
	V_q=V_q_GXieLv.XieLv_Y;
	V_q=Min(Us_Limit,V_q);
}

void Idq_FOC(void)    //��������ģʽ
{
	 float32 Us_Limit=0;

	  pi_id.Ref = I_d_GXieLv.XieLv_Y;
	  pi_id.Fbk = park1.Ds ;
	  //PI_Controller((p_PI_Control)&pi_id);
	  //pi_id.OutF=pi_id.OutF*GM_Low_Lass_A+pi_id.Out*GM_Low_Lass_B;
	  //V_d=pi_id.OutF;

      Us_Limit=(IQSqrt(Vdc_s*Vdc_s*100-V_d*V_d*100))*0.1;
	  pi_iq.Umax=Us_Limit;
	  pi_iq.Umin=-Us_Limit;

	  pi_iq.Ref = I_q_GXieLv.XieLv_Y;
	  pi_iq.Fbk = park1.Qs;
	  //PI_Controller((p_PI_Control)&pi_iq);
	  //pi_iq.OutF=pi_iq.OutF*GM_Low_Lass_A+pi_iq.Out*GM_Low_Lass_B;
	  //V_q=pi_iq.OutF;
}

void Control_Mode_CmdPara(void)    // ����ģʽ ����
 {
	switch(CAN_ControlPara.Control_Mode)
	{
		case Control_Mode_Speed:  // 1
			 SpeedRpm_GXieLv.XieLv_X= CAN_ControlPara.SpeedRPM;   //
			 Speed_FOC();  //�ٶȿ���ģʽ
			 //Idq_FOC();
			 Trq_FOC();
			 I_d_GXieLv.XieLv_Y=I_d_GXieLv.XieLv_X;
			 I_q_GXieLv.XieLv_Y=I_q_GXieLv.XieLv_X;
			 Idq_FOC();
			 break;
		case Control_Mode_Torq:   //2   Ť�ؿ���
			 Trq_0D1NM_GXieLv.XieLv_X=CAN_ControlPara.Trq;
			 Trq_FOC();  // ���TrqŤ�ز������ջ�
             // Ť��Trq  ���    I_d_GXieLv.XieLv_X ;
			 // Ť��Trq  ���    I_q_GXieLv.XieLv_X ;
			 I_q_GXieLv.XieLv_Grad=  Idq_Grad_0D1A ;  //0.05A �ٶ�������ݶȼ���
			 I_q_GXieLv.Grad_Timer=  Idq_Grad_Timer;// 2ms�ٶȻ�����
			 I_q_GXieLv.Timer_Count++;
			 if(I_q_GXieLv.Timer_Count>I_q_GXieLv.Grad_Timer)
			 {
			   I_q_GXieLv.Timer_Count=0;
			   Grad_XieLv((p_GXieLv)&I_q_GXieLv);
			   Grad_XieLv((p_GXieLv)&I_d_GXieLv);
			   // I_q_GXieLv.XieLv_Y  �������Ƹ���ֵ
			   // I_d_GXieLv.XieLv_Y  �������Ƹ���ֵ
			 }
			 Idq_FOC();  // ���idq�����ջ�
			break;
		case Control_Mode_VF:    // 3
			 VF_Freq_GXieLv.XieLv_X =CAN_ControlPara.VF_Vq;   // 35HZ
			 VF_Vq_GXieLv.XieLv_X = CAN_ControlPara.VF_Vq;   // 6.5v
			 VF_Control();
			break;
		case Control_Mode_Vdq:   // 4
		     V_d_GXieLv.XieLv_X = 0;
		     V_q_GXieLv.XieLv_X =  CAN_ControlPara.V_q;   //  SpeedRpm_GXieLv.XieLv_X*0.0038;  // 13.5v
			 Vdq_FOC();
			break;
		case Control_Mode_Idq:   // 5   ��֧�ֵ�λ������,���е���  ֱ�Ӹ������ᵼ�´�Ť�طɳ�  ������id=0����
			 I_q_GXieLv.XieLv_X= CAN_ControlPara.I_q_cmd ;    //
			 I_d_GXieLv.XieLv_X = CAN_ControlPara.I_d_cmd;   //
			 I_q_GXieLv.XieLv_Grad=  Idq_Grad_0D1A ;  //0.05A �ٶ�������ݶȼ���
			 I_q_GXieLv.Grad_Timer=  Idq_Grad_Timer;// 2ms�ٶȻ�����
			 I_q_GXieLv.Timer_Count++;
			 if(I_q_GXieLv.Timer_Count>I_q_GXieLv.Grad_Timer)
			 {
			   I_q_GXieLv.Timer_Count=0;
			   Grad_XieLv((p_GXieLv)&I_q_GXieLv);
			   Grad_XieLv((p_GXieLv)&I_d_GXieLv);
			   // I_q_GXieLv.XieLv_Y  �������Ƹ���ֵ
			   // I_d_GXieLv.XieLv_Y  �������Ƹ���ֵ
			 }
			 Idq_FOC();
			break;
	}
 }


void  FOC_Svpwm_dq(void)   // dq���ѹ���룬��Park�任�����SVPWM
{
	ipark1.Ds=V_d;  //
	ipark1.Qs=V_q;
	if(CAN_ControlPara.Control_Mode==Control_Mode_VF)  // VF����ģʽ��ʱ�򣬵���Ƕ��ԼӼ��Ƕ�λ��
	ipark1.Angle= VF_AngleJZ;
	else
	ipark1.Angle=AD2SXBPare.PM_angle;     // ��VF����ʱ������λ���ź�λ�ô�������Ϊ��Ƕ�
	IPark_SinCos.table_Angle=ipark1.Angle;
	SinCos_Table((p_Ang_SinCos)&IPark_SinCos);   // ��PARK�仯�������Ҳ��
	ipark1.Sine =IPark_SinCos.table_Sin;
	ipark1.Cosine =IPark_SinCos.table_Cos;
	//IPARK_Cale((p_IPARK)&ipark1);    // ��Park�任
	svpwm1.Ualpha=ipark1.Alpha;
	svpwm1.Ubeta=ipark1.Beta;
	//SVPWM_Cale((p_SVPWM)&svpwm1);   // ��Alpha��Beta��ѹ�������SVPWM��ռ�ձ�

}

void Stop_Motor(void)   // ͣ������  ��ʼ�������Ʋ������
{
	// ����ָ������ٶȻ�����PID���ƹ��̱���
  Control_FB_Para.Motor_State=0;
  //STOP_CAR( );
  SpeedRpm_GXieLv.XieLv_X=0;
  SpeedRpm_GXieLv.XieLv_Y=0;

  Trq_0D1NM_GXieLv.XieLv_X=0;
  Trq_0D1NM_GXieLv.XieLv_Y=0;


  VF_Freq_GXieLv.XieLv_X =0;
  VF_Vq_GXieLv.XieLv_X = 0;
  VF_Freq_GXieLv.XieLv_Y =0;
  VF_Vq_GXieLv.XieLv_Y = 0;


  V_d_GXieLv.XieLv_X = 0;
  V_q_GXieLv.XieLv_X = 0;
  I_d_GXieLv.XieLv_X = 0;
  I_q_GXieLv.XieLv_X = 0;
  V_d_GXieLv.XieLv_Y = 0;
  V_q_GXieLv.XieLv_Y = 0;
  I_d_GXieLv.XieLv_Y = 0;
  I_q_GXieLv.XieLv_Y = 0;

  pi_spd.Ref=0;
  pi_spd.Fbk=0;
  pi_spd.v1=0;
  pi_spd.Out=0;
  //pi_spd.OutF=0;
  pi_spd.i1=0;
  pi_spd.ui=0;

  pi_id.Ref=0;
  pi_id.Fbk=0;
  pi_id.v1=0;
  pi_id.Out=0;
  //pi_id.OutF=0;
  pi_id.i1=0;
  pi_id.ui=0;

  pi_iq.Ref=0;
  pi_iq.Fbk=0;
  pi_iq.v1=0;
  pi_iq.Out=0;
  //pi_iq.OutF=0;
  pi_iq.i1=0;
  pi_iq.ui=0;

  pi_FW.Ref=0;
  pi_FW.Fbk=0;
  pi_FW.v1=0;
  pi_FW.Out=0;
  //pi_FW.OutF=0;
  pi_FW.i1=0;
  pi_FW.ui=0;
  V_d=0;
  V_q=0;
}

void  FOC_Control_Select(void)   //
{
    // ��λ�����ٶȻ�����
	 if( CAN_ControlPara.drive_car==1 )
	 {
		 Control_Mode_CmdPara();  // 1,2,3,4,5�������ģʽԲ�κ���
	 }
	 else     // ͣ�����д���
	 {
	   Stop_Motor();     // ͣ���������ݺ�ָ��
	   Offset_CurrentReading();  // ƫִ��������
	 }
}

void  StartControl_Mode(void)   //  ��ʼ����ģʽ����
{
  if(CAN_ControlPara.BoMa_Control_Flag==0x01)   // ���뿪�ص���1:ʱ��ߵ�ƽ�ǵ�λ������ģʽ ��0��CANͨѶ����ָ������
  {
	  if(ADCSampPara.RP_speed_Voltage>100)
	  {
	  CAN_ControlPara.Control_Mode=1;  //  ����ģʽ ����ѡ��  1Ϊ�ٶȻ���ת   2�ٶȻ���ת   3ΪVF����    4ΪVdq��ѹ����   5Idq��������   6TrqŤ�ؿ���
	  CAN_ControlPara.drive_car=1;
	   DWQ_Control_AD();  //  ��λ������ģʽ�µĲ�����������
    	if(CAN_ControlPara.Control_Mode==3)  //VF����ʱ�򲻽�����ת��ѹ�������ź��ж�
		{
		  CAN_ControlPara.drive_car=1;
		}
		else              // �������Ʒ�ʽ��Ҫ��ת��ѹ��λ�ý��� ����  ������LOTDOS�жϹ���
		{
		if(Control_FB_Para.Fault_DTC==0)     // AD2SXBPare.AD2S_FaultDOSLOT
		 CAN_ControlPara.drive_car=1;
		else
		 CAN_ControlPara.drive_car=0;
		}
	  }
	  else
	  {
		 CAN_ControlPara.drive_car=0;
		 CAN_ControlPara.Control_Mode=0;
	  }
  }
  else  if( CAN_ControlPara.BoMa_Control_Flag==0x00 )  // 0��CANͨѶ����ָ������
  {
	  if((CAN_ControlPara.Control_Mode!=0) &&(Control_FB_Para.Fault_DTC==0))   //  ����ģʽ ����ѡ��  1Ϊ�ٶȻ���ת   2�ٶȻ���ת       4ΪVdq��ѹ����
	 {
		 // �������ģʽ��Ϊ����޹���ʱ�� ����ͨPWM
		CAN_ControlPara.drive_car=1;   // ʹ�ܿ�ʼ
	    CAN_ControlMotor_CMD();  //  ͨѶ���ƣ�ֻ��ת�ٺ�Ť��ģʽ ������ת�ٺ�ָ�����
	 }
	 else  // �ع���
	 {
	   CAN_ControlPara.drive_car=0;
	   CAN_ControlPara.Control_Mode=0;
	 }
  }
  else
  {
	CAN_ControlPara.drive_car=0;
	CAN_ControlPara.Control_Mode=0;
  }


  if ((CAN_ControlPara.olddrive_car==0) &&( CAN_ControlPara.drive_car==1 ))
  {
	   //START_CAR();
  }
  else if ((CAN_ControlPara.olddrive_car==0) &&( CAN_ControlPara.drive_car==0))
  {
	   Stop_Motor();
  }

  CAN_ControlPara.olddrive_car=CAN_ControlPara.drive_car;
}



void  FOC_Control_TestPara(void)  // ����ز���CAN������λ��  �ٶ� ��Ƶ�ʡ� dq���������ѹ���Ƕ�λ�õ�
{
	// �������
   Control_FB_Para.SpeedRPM = AD2SXBPare.AD2S_SpdRPMH;
   Control_FB_Para.VF_Freq = VF_Freq_GXieLv.XieLv_Y;
   Control_FB_Para.Trq = Trq_0D1NM_GXieLv.XieLv_Y ;
   Control_FB_Para.V_s= (IQSqrt(V_q*V_q*100+V_d*V_d*100))*0.1;
   Control_FB_Para.V_d = V_d;
   Control_FB_Para.V_q = V_q;
   Control_FB_Para.I_d_fb = park1.Ds ;
   Control_FB_Para.I_q_fb = park1.Qs ;
   Control_FB_Para.I_d = I_d_GXieLv.XieLv_Y;
   Control_FB_Para.I_q = I_q_GXieLv.XieLv_Y;
   Control_FB_Para.PM_Angle= ipark1.Angle;
   Control_FB_Para.Motor_State=0;
}

//===========================================================================
// No more.
//===========================================================================
