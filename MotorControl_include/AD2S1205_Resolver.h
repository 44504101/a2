//############################################################
// FILE:  AD2S1205_Resolver.h
// Created on: 2017��12��10��
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
	Uint16      AD2S_angle;        //�����ת�Ƕ�
	float32      AD2S_angle_du;
	float32      AD2S_circle;
	Uint16      PM_angle;          //�����ת��Ƕ�
	Uint16	    PM_Poles;          //���������
	Uint16	    XB_Poles;          //���伫����
	Uint16	    PM_XB_Knum;        //���/���伫������
	Uint16      AD2S_Spd_RD;         //��������ٶ�
	int16       AD2S_Spd;         //�����ת�ٶ�
	Uint16	    init_Angle;       // �����λ��
	int16       AD2S_SpdRPM;       //�����ת�ٶ�
	int16       AD2S_SpdRPMH;      //��ʷ�����ת�ٶ�
	Uint16      Abs_SpdRPM;        //��ʷ�����ת�ٶ�
	Uint16      Move_State;        //�����ת״̬
	Uint8       AD2S_FaultDOSLOT;  // ��ת��ѹ���Ĺ��� LOT DOS
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

#define  AD2SXB_DEFAULTS {0,0,0.0,0,0,0,0,0,0,0,0,0,0}       // ��ʼ������
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
extern bool MotionInProgress;         // �˶����б�־
extern bool ReverseBack;          // �Ƿ���Ҫ��ת����ʼλ��
extern bool ReturnToZero;            // ��������־
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
	                // ģʽ 1�������תָ��Ȧ��
	                if (v->RotationCount >= v->TargetRotation) {
	                    gMotorVars.Desired_Speed_RPM = 0.0f;
	                    MotionInProgress = false;
	                }
	                break;

	            case MODE_REVERSE:
	                // ģʽ 2�������תָ��Ȧ��
	                if (v->RotationCount <= -v->TargetRotation) {
	                    gMotorVars.Desired_Speed_RPM = 0.0f;
	                    MotionInProgress = false;
	                }
	                break;

	            case MODE_FOEWARD_REVERSE:
	                if (v->Count < 4)
	                {
	                    if (!ReverseBack)                    // -------- ��ת�׶� --------
	                    {
	                        if (v->RelativeCount >= v->TargetRotation)
	                        {
	                            ReverseBack = true;          // ���뷴ת
	                            gMotorVars.Desired_Speed_RPM = -fabsf(gMotorVars.Desired_Speed_RPM);
	                        }
	                    }
	                    else                                 // -------- ��ת�׶� --------
	                    {
	                        if (v->RelativeCount <= 0)
	                                    {
	                            v->Count++;                   /* ���� +1 */

	                                        /* ===== �����ȫ������������ͣ���� ===== */
	                                        if (v->Count >= 4)
	                                        {
	                                            /* ---------- ͳһͣ������ ---------- */
	                                            gMotorVars.Desired_Speed_RPM = 0.0f;
	                                            MotionInProgress   = false;
	                                            ReverseBack        = false;
	                                            v->RelativeCount = 0;
	                                            break;                       /* �����ڲ����·� PWM */
	                                        }

	                                        /* ---------- ������һ����ת ---------- */
	                                        ReverseBack = false;
	                                        gMotorVars.Desired_Speed_RPM = fabsf(gMotorVars.Desired_Speed_RPM);
	                                        v->RelativeCount = 0;         /* ���������������� */
	                                    }
	                                }
	                    }

	                else                                     // -------- �˶����� --------
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
	                // δ֪ģʽ��ֹͣ���
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
    		        v->RotationCount += 1;  // �����Ƿ���
    		        v->AccumulatedMechTheta -= 360.0f;  // ��ȥһȦ�ĽǶ�
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
