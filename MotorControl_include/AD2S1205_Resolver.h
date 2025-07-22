//############################################################
// FILE:  AD2S1205_Resolver.h
// Created on: 2017年12月10日
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

#ifndef AD2S1205_Resolver_H
#define AD2S1205_Resolver_H
#include "DSP2833x_Project.h"
#include "IQmathLib.h"
#include"offset.h"
#include"FOC.h"
#include <stdbool.h>
#include"Axis_transform.h"
#include"PI_Cale.h"
#define RDVEL_ON     GpioDataRegs.GPBSET.bit.GPIO38=1
#define RDVEL_OFF    GpioDataRegs.GPBCLEAR.bit.GPIO38=1
#define SAMPLE_ON    GpioDataRegs.GPCSET.bit.GPIO79=1
#define SAMPLE_OFF   GpioDataRegs.GPCCLEAR.bit.GPIO79=1
#define CS_RD_ON     GpioDataRegs.GPASET.bit.GPIO28=1
#define CS_RD_OFF    GpioDataRegs.GPACLEAR.bit.GPIO28=1
#define XB_NRST_ON   GpioDataRegs.GPBSET.bit.GPIO36=1
#define XB_NRST_OFF  GpioDataRegs.GPBCLEAR.bit.GPIO36=1

#define GPIO_DOS    GpioDataRegs.GPBDAT.bit.GPIO35
#define GPIO_LOT    GpioDataRegs.GPBDAT.bit.GPIO34

#define DB_0    GpioDataRegs.GPCDAT.bit.GPIO68
#define DB_1    GpioDataRegs.GPCDAT.bit.GPIO67
#define DB_2    GpioDataRegs.GPCDAT.bit.GPIO70
#define DB_3    GpioDataRegs.GPCDAT.bit.GPIO69
#define DB_4    GpioDataRegs.GPCDAT.bit.GPIO72
#define DB_5    GpioDataRegs.GPCDAT.bit.GPIO71
#define DB_6    GpioDataRegs.GPCDAT.bit.GPIO74
#define DB_7    GpioDataRegs.GPCDAT.bit.GPIO73
#define DB_8    GpioDataRegs.GPCDAT.bit.GPIO76
#define DB_9    GpioDataRegs.GPCDAT.bit.GPIO75
#define DB_10   GpioDataRegs.GPCDAT.bit.GPIO78
#define DB_11   GpioDataRegs.GPCDAT.bit.GPIO77

typedef struct {
	Uint16      AD2S_angle;        //电机旋转角度
	float32      AD2S_angle_du;
	float32      AD2S_circle;
	Uint16      PM_angle;          //电机旋转电角度
	Uint16	    PM_Poles;          //电机极对数
	Uint16	    XB_Poles;          //旋变极对数
	Uint16	    PM_XB_Knum;        //电机/旋变极对数比
	Uint16      AD2S_Spd_RD;         //电机旋变速度
	int16       AD2S_Spd;         //电机旋转速度
	Uint16	    init_Angle;       // 电机零位角
	int16       AD2S_SpdRPM;       //电机旋转速度
	int16       AD2S_SpdRPMH;      //历史电机旋转速度
	Uint16      Abs_SpdRPM;        //历史电机旋转速度
	Uint16      Move_State;        //电机旋转状态
	Uint8       AD2S_FaultDOSLOT;  // 旋转变压器的故障 LOT DOS
}AD2SXB;

typedef struct{
	float32_t tempresolver;
	float32_t TempResolver;
	float32_t UncorrectedPosition_degrees;
	float32_t mResolverOffset_degrees;
	float32_t mNumberOfPoles;
	_iq ElecTheta;
	float32_t PreviousResolver;
	float32_t RotationCount;
	Uint16 PreviousDirection;
	Uint16 Direction;
	float32_t MechTheta;
	float32_t delta_theta;
	float32_t TargetRotation;
	float32_t AccumulatedMechTheta;
	float32_t  DiverseCount;
	float32_t RelativeCount;
	float32_t Count;
}RESOLVER;

#define  AD2SXB_DEFAULTS {0,0,0.0,0,0,0,0,0,0,0,0,0,0}       // 初始化参数
#define RESOLVER_DEAFAULTS { 0,\
							0,\
							0,\
							3.3f,\
							4.0f,\
							0,\
							0,\
							0,\
							0,\
							0,\
							0,\
							0,\
							0,\
							0,\
							0,\
							0,\
							0,\
}

static _iq prevAs =0;
extern AD2SXB      AD2SXBPare;
void Init_Gpio_AD2S1205(void);
void AD2S1205_Init(void);
Uint16 Read_IO_interface(void);
void AD2S1205_Read(void);
extern bool MotionInProgress;         // 运动进行标志
extern bool ReverseBack;          // 是否需要反转回起始位置
extern bool ReturnToZero;            // 返回零点标志
extern bool OverCurrentDetected;
extern  _iq CurrentThreshold;
bool IsOverCurrent();
static inline void UpdateRotationCount(RESOLVER* v, float positive_threshold, float negative_threshold) {
    v->delta_theta = v->MechTheta - v->PreviousResolver;
    if (v->delta_theta > 180.0f) {
        v->delta_theta -= 360.0f;
    } else if (v->delta_theta < -180.0f) {
        v->delta_theta += 360.0f;
    }
    if (v->delta_theta >= positive_threshold) {
        v->Direction = 1;
    } else if (v->delta_theta <= negative_threshold) {
        v->Direction = 0;
    } else {
        v->Direction = v->PreviousDirection;
    }
       v->AccumulatedMechTheta += v->delta_theta;
       if(v->AccumulatedMechTheta >= 360.0f){
    	   v->RotationCount +=1;
    	   v->AccumulatedMechTheta -=360.0f;
       }else if(v->AccumulatedMechTheta <= -360.0f){
    	   v->RotationCount +=1;
    	   v->RelativeCount -=1;
    	   v->AccumulatedMechTheta +=360.0f;
       }
       v->DiverseCount = v->RotationCount/29.0f;
       v->PreviousDirection = v->Direction;
       v->PreviousResolver = v->MechTheta;
}

static inline void MotionControl(RESOLVER* v){
    float32_t a =0;
	if (MotionInProgress) {
	        switch (currentMode) {
	            case MODE_FORWARD:
	                // 模式 1：电机正转指定圈数
	                if (v->RotationCount >= v->TargetRotation) {
	                    gMotorVars.Desired_Speed_RPM = 0.0f;
	                    MotionInProgress = false;
	                }
	                break;

	            case MODE_REVERSE:
	                // 模式 2：电机反转指定圈数
	                if (v->RotationCount <= -v->TargetRotation) {
	                    gMotorVars.Desired_Speed_RPM = 0.0f;
	                    MotionInProgress = false;
	                }
	                break;

	            case MODE_FOEWARD_REVERSE:
	                if (v->Count < 4)
	                {
	                    if (!ReverseBack)                    // -------- 正转阶段 --------
	                    {
	                        if (v->RelativeCount >= v->TargetRotation)
	                        {
	                            ReverseBack = true;          // 进入反转
	                            gMotorVars.Desired_Speed_RPM = -fabsf(gMotorVars.Desired_Speed_RPM);
	                        }
	                    }
	                    else                                 // -------- 反转阶段 --------
	                    {
	                        if (v->RelativeCount <= 0)
	                                    {
	                            v->Count++;                   /* 往返 +1 */

	                                        /* ===== 已完成全部往返？立刻停机！ ===== */
	                                        if (v->Count >= 4)
	                                        {
	                                            /* ---------- 统一停机例程 ---------- */
	                                            gMotorVars.Desired_Speed_RPM = 0.0f;
	                                            MotionInProgress   = false;
	                                            ReverseBack        = false;
	                                            v->RelativeCount = 0;
	                                            break;                       /* 本周期不再下发 PWM */
	                                        }

	                                        /* ---------- 继续下一次正转 ---------- */
	                                        ReverseBack = false;
	                                        gMotorVars.Desired_Speed_RPM = fabsf(gMotorVars.Desired_Speed_RPM);
	                                        v->RelativeCount = 0;         /* 清零计数，消除误差 */
	                                    }
	                                }
	                    }

	                else                                     // -------- 运动结束 --------
	                {
	                    gMotorVars.Desired_Speed_RPM = 0.0f;
	                    MotionInProgress = false;
	                    ReverseBack = false;
	                }
	                break;

	            case MODE_HOMING:
	            {
	            	_iq spike_threshold = _IQ(0.027);
	            	if(!OverCurrentDetected){
	            		if((clark1.As - prevAs) > spike_threshold){
	            			OverCurrentDetected = true;
	            			gMotorVars.Desired_Speed_RPM = 300.0f;
	            			v->RotationCount = 0;
	            			v->AccumulatedMechTheta = 0;
	            			v->TargetRotation = 18*29;
	            		}
	            	}else{
	            		if(v->RotationCount >= v->TargetRotation){
	            			gMotorVars.Desired_Speed_RPM = 0.0f;
	            			MotionInProgress = false;
	            			OverCurrentDetected = false;
	            		}
	            	}
	            	prevAs = clark1.As;
	            }
                 break;
	            default:
	                // 未知模式，停止电机
	                gMotorVars.Desired_Speed_RPM = 0.0f;
	                MotionInProgress = false;
	                break;
	        }
	    }
	}


    /*	if((v->PreviousResolver>270.0f && v->MechTheta<90.0f) || (v->PreviousResolver<90.0f && v->MechTheta>270.0f)){
    		v->RotationCount += 1;
    		v->DiverseCount=v->RotationCount/29.0f;
    		 if (v->AccumulatedMechTheta >= 360.0f) {
    		        v->RotationCount += 1;  // 不考虑方向
    		        v->AccumulatedMechTheta -= 360.0f;  // 减去一圈的角度
    		    }*/


/*loat GetMotorSpeed(void)
{
    return gMotorVars.Desired_Speed_RPM;
}
static inline void SetMotorSpeed(RESOLVER*v,MOTOR_Vars_t*b){
	if(abs(v->RotationCount) >= v->TargetRotation+1){
		b->Desired_Speed_RPM = 0;
	}
}
*/

#endif /* AD2S1205_Resolver_H*/
//===========================================================================
// End of file.
//===========================================================================
