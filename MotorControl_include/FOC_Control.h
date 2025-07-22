//############################################################
// FILE: FOC_Control.h
// Created on: 2017��5��2��
// Author: XQ
// summary: Header file  and definition
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//��Ȩ���У�����ؾ�
//DSP28335����PMSM��������뿪����
//˶������
//��ַ: https://shuolidianzi.taobao.com
//Author-QQ: 616264123
//�������QQȺ��736369775
//############################################################

#ifndef  _FOC_Control_H
#define  _FOC_Control_H

typedef struct {
	    float32  VF_Freq;	         //�ٶȿ���
	 	int16    SpeedRPM;	         //�ٶȿ���
	 	float32  Trq;                //Ť�ؿ���
	 	float32  V_d;                //Vdq����
	    float32  V_q;                //Vdq����
	    float32  V_s;                //���ѹVs
	    float32  I_d_fb;             //Idq����
	    float32  I_q_fb;             //Idq����
	    float32  I_d;                //Idq����
	    float32  I_q;                //Idq����
	    Uint16   PM_Angle;      // ����Ƕ�
	    Uint8    Motor_State;     // ���״̬
	    Uint8    Fault_DTC;
      }Control_FB;

#define Control_FB_DEFAULTS  {0,0,0,0,0,0,0,0,0,0,0,0,0}  // ��ʼ������

  typedef struct {
    	  float32 Id_Value;
    	  float32 Next_Id_Value;
    	  float32 Id_Table_CMD;
    	  float32 Iq_Value;
    	  float32 Iq_Table_CMD;
    	  float32 Next_Iq_Value;
      }CESHIIIII;

#define CESHIIIII_DEFAULTS  {0,0,0,0,0,0}


extern GXieLv    SpeedRpm_GXieLv;
extern GXieLv    Trq_0D1NM_GXieLv;
extern GXieLv    VF_Freq_GXieLv;
extern GXieLv    VF_Vq_GXieLv;
extern GXieLv    V_d_GXieLv;
extern GXieLv    V_q_GXieLv;
extern GXieLv    I_d_GXieLv;
extern GXieLv    I_q_GXieLv;
extern Control_FB   Control_FB_Para;
extern CLARKE       clark1;
extern PARK         park1;
extern IPARK        ipark1;
extern SVGEN        svpwm1;
extern CESHIIIII    CESHIIIII565;

extern float32   VF_Angle;
extern Uint16   VF_AngleJZ;

void FOC_Control_Select(void); // FOC����ģʽѡ��
void UVW_Axis_dq(void);   // ���������dq������仯
void FOC_Svpwm_dq(void);  // FOC��ʸ���仯
void Speed_FOC(void); // �ٶȿ���ģʽ
void Trq_FOC(void);   //Ť�ؿ���ģʽ
void VF_Control(void);    //VF����ģʽ
void Vdq_FOC(void);    //��ѹ����ģʽ
void Idq_FOC(void);    //��������ģʽ
void FW_PI(void);      //��ѹ���ŷ�PI
void Stop_Motor(void);   // ͣ������
void FOC_Control_TestPara(void);  // FOC���ƵĲ��Բ���
void Control_Mode_CmdPara(void);    // ���п���ģʽ
void  StartControl_Mode(void);  // ��ʼ����ģʽ����
#endif /* FOC_Control*/
