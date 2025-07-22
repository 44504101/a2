//############################################################
// FILE: FOC_Control.h
// Created on: 2017年5月2日
// Author: XQ
// summary: Header file  and definition
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//版权所有，盗版必究
//DSP28335旋变PMSM电机控制与开发板
//硕历电子
//网址: https://shuolidianzi.taobao.com
//Author-QQ: 616264123
//电机控制QQ群：736369775
//############################################################

#ifndef  _FOC_Control_H
#define  _FOC_Control_H

typedef struct {
	    float32  VF_Freq;	         //速度控制
	 	int16    SpeedRPM;	         //速度控制
	 	float32  Trq;                //扭矩控制
	 	float32  V_d;                //Vdq控制
	    float32  V_q;                //Vdq控制
	    float32  V_s;                //相电压Vs
	    float32  I_d_fb;             //Idq控制
	    float32  I_q_fb;             //Idq控制
	    float32  I_d;                //Idq控制
	    float32  I_q;                //Idq控制
	    Uint16   PM_Angle;      // 旋变角度
	    Uint8    Motor_State;     // 电机状态
	    Uint8    Fault_DTC;
      }Control_FB;

#define Control_FB_DEFAULTS  {0,0,0,0,0,0,0,0,0,0,0,0,0}  // 初始化参数

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

void FOC_Control_Select(void); // FOC控制模式选择
void UVW_Axis_dq(void);   // 三相电流到dq轴电流变化
void FOC_Svpwm_dq(void);  // FOC的矢量变化
void Speed_FOC(void); // 速度控制模式
void Trq_FOC(void);   //扭矩控制模式
void VF_Control(void);    //VF控制模式
void Vdq_FOC(void);    //电压控制模式
void Idq_FOC(void);    //电流控制模式
void FW_PI(void);      //电压弱磁法PI
void Stop_Motor(void);   // 停机函数
void FOC_Control_TestPara(void);  // FOC控制的测试参数
void Control_Mode_CmdPara(void);    // 运行控制模式
void  StartControl_Mode(void);  // 开始控制模式函数
#endif /* FOC_Control*/
