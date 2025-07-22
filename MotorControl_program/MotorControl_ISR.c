//############################################################
// FILE:  MotorControl_ISR.c
// Created on: 2017年12月10日
// Author: XQ
// summary: PMSM  control  with Resolver
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//版权所有，盗版必究
//DSP28335旋变PMSM电机控制与开发板
//硕历电子
//网址: https://shuolidianzi.taobao.com
//修改日期:2017/12/18
//版本：V17_1
//Author-QQ: 616264123
//电机控制QQ群：736369775
//############################################################
#include "IQmathLib.h"
#include "Motorcontrol_include.h"
#include "Task_manager.h"
#include <math.h>
#include"Main_PMSM.h"
#define RESOLVER_POSITION           (360.0f / 4096.0f)
bool ReverseBack = false;
bool MotionInProgress =true;
bool OverCurrentDetected = false;
MOTORMODE currentMode = MODE_FORWARD;
_iq CurrentThreshold = _IQ(0.15);
float32_t positive_threshold = 0.5f; // 正向阈值
float32_t negative_threshold = -0.5f; // 反向阈值
Uint32 IsrTicker=0;
Uint16 lsw=0;
_iq VdTesting = _IQ(0.0);
_iq VqTesting = _IQ(0.1);
_iq IdRef = 0;
_iq IqRef = _IQ(0.3);
_iq SpeedRef;
_iq offsetA = 0;
_iq offsetB = 0;
_iq adcResult2IQ;
_iq adcResult3IQ;
_iq K1=_IQ(0.998);		//Offset filter coefficient K1: 0.05/(T+0.05);
_iq K2=_IQ(0.001999);	//Offset filter coefficient K2: T/(T+0.05);
RMPCNTL rc1 = RMPCNTL_DEAFAULT;
RAMPGEN rg1 = RAMPGEN_DEAFAULT;
PHASEVOLTAGE volt1 = PHASEVOLTAGE_DEAFAULTS;
MotorOffsetCurent_t mMotorOffsets = MOTOR_OFFSET_DEAFAULT;
RESOLVER resolver1 = RESOLVER_DEAFAULTS;
MOTOR_Vars_t gMotorVars = MOTOR_Vars_t_DEAFAULT;

volatile bool brake_hold   = false;
volatile bool pos_protect  = false;
volatile bool over_current = false;
volatile bool sys_reset    = false;
volatile bool motor_run    = false;
volatile bool dir_fwd      = true;   /* 默认正转 */

MOTOR_STATUS_t motorSt = MOTOR_STATUS_t_DEAFAULT;
interrupt void OffsetISR(void)
{
// Verifying the ISR
    IsrTicker++;
    //SwitchOffPWMAndSetRunningMotorModeToStop();
// DC offset measurement for ADC
   if (IsrTicker>=5000)
    	{
	  adcResult2IQ = _IQ(AdcMirror.ADCRESULT2 * 0.00024414);
	   adcResult3IQ = _IQ(AdcMirror.ADCRESULT3 * 0.00024414);
	   offsetA = _IQmpy(K1, offsetA) + _IQmpy(K2, adcResult3IQ);
	   offsetB = _IQmpy(K1, offsetB) + _IQmpy(K2, adcResult2IQ);			//Phase B offset
    }
	if (IsrTicker > 20000)
	{
		EALLOW;
		PieVectTable.EPWM1_INT = &MainISR;
		EDIS;
	}
// Enable more interrupts from this timer
	EPwm1Regs.ETCLR.bit.INT = 1;
// Acknowledge interrupt to recieve more interrupts from PIE group 3
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;
}

interrupt void MainISR(void)
{

	IsrTicker++;

#if(BUILDLEVEL==LEVEL1)

  rc1.TargetValue = _IQ(0.15);
  RC_MACRO(rc1);
  rg1.StepAngleMax=_IQ(BASE_FREQ*T);
  rg1.Freq = rc1.SetPointValue;
  RG_MACRO(rg1);

  ipark1.Ds = VdTesting;
  ipark1.Qs = VqTesting;

  ipark1.Sine = _IQsinPU(rg1.Out);
  ipark1.Cosine = _IQcosPU(rg1.Out);
  IPARK_MACRO(ipark1);

  svpwm1.Ualpha = ipark1.Alpha;
  svpwm1.Ubeta = ipark1.Beta;
  SVGENDQ_MACRO(svpwm1);

  pwm1.MfuncC1 = svpwm1.Ta;
  pwm1.MfuncC2 = svpwm1.Tb;
  pwm1.MfuncC3 = svpwm1.Tc;
  PWM_MACRO(1,2,3,pwm1);

  pwmdac1.MfuncC1 = rg1.Out;
  pwmdac1.MfuncC2 = svpwm1.Tb - svpwm1.Tc;
  PWMDAC_MACRO(4,pwmdac1);


      DlogCh1 = (int16)_IQtoIQ15(pwm1.shuchu);
      DlogCh2 = (int16)_IQtoIQ15(rg1.Out);
      DlogCh3 = (int16)_IQtoIQ15(svpwm1.Tc);
      DlogCh4 = (int16)_IQtoIQ15(svpwm1.Tb-svpwm1.Tc);
      dlog.update(&dlog);
      FOC_Control_TestPara( );// 将相关参数CAN发送上位机  速度 。频率。 dq轴电流。电压。角度位置等

      EPwm1Regs.ETCLR.bit.INT = 1; // 使能中断
      PieCtrlRegs.PIEACK.all = PIEACK_GROUP3; //开中断向量

#endif
#if(BUILDLEVEL==LEVEL2)
      rc1.TargetValue = _IQ(0.1);
      RC_MACRO(rc1);

      rg1.StepAngleMax=_IQ(BASE_FREQ*T);
      rg1.Freq = rc1.SetPointValue;
      RG_MACRO(rg1);


         ADC_Sample( );
         ADC_Sample_deal( );

         _iq scaledFactor = _IQ(2 * 0.909);
         //_iq temp = _IQmpy(_IQ(AdcMirror.ADCRESULT3), _IQ(0.00024414));
         clark1.abc=_IQ((float32_t)AdcMirror.ADCRESULT3/4096)-offsetA; // Phase A curr.
         clark1.As = _IQmpy(clark1.abc,scaledFactor);
         clark1.bbc=_IQ((float32_t)AdcMirror.ADCRESULT2/4096)-offsetB;
         clark1.Bs=_IQmpy(clark1.bbc,scaledFactor);
      CLARKE_MACRO(clark1);

      park1.Alpha = clark1.Alpha;
      park1.Beta =  clark1.Beta;
      park1.Angle = rg1.Out;
      park1.Sine = _IQsinPU(park1.Angle);
      park1.Cosine = _IQcosPU(park1.Angle);
      PARK_MACRO(park1);

      ipark1.Ds = VdTesting;
      ipark1.Qs = VqTesting;

      ipark1.Sine = park1.Sine;
      ipark1.Cosine = park1.Cosine;
      IPARK_MACRO(ipark1);

      /*volt1.DcBusVolt = ((AdcMirror.ADCRESULT1)*0.00024414)*0.909;
      volt1.MfuncV1 = svpwm1.Ta;
      volt1.MfuncV2 = svpwm1.Tb;
      volt1.MfuncV3 = svpwm1.Tc;
      PHASEVOLTAGE_MACRO(volt1);*/

      svpwm1.Ualpha = ipark1.Alpha;
      svpwm1.Ubeta = ipark1.Beta;
      SVGENDQ_MACRO(svpwm1);


            float DcBusVoltage_Float = (AdcMirror.ADCRESULT4 / 4096.0) * 3.3;
            volt1.DcBusVolt = _IQ24(DcBusVoltage_Float);
            /*ADC_Sample( );
            ADC_Sample_deal( );*/
            volt1.MfuncV1 = svpwm1.Ta;
            volt1.MfuncV2 = svpwm1.Tb;
            volt1.MfuncV3 = svpwm1.Tc;
            PHASEVOLTAGE_MACRO(volt1);

            motorSt.BusVolat=volt1.DcBusVolt;
            motorSt.Current = (AdcMirror.ADCRESULT1/4096.0)*3.3;

      pwm1.MfuncC1 = svpwm1.Ta;
      pwm1.MfuncC2 = svpwm1.Tb;
      pwm1.MfuncC3 = svpwm1.Tc;
      PWM_MACRO(1,2,3,pwm1);

      pwmdac1.MfuncC1 =clark1.Alpha;
      pwmdac1.MfuncC2 =clark1.Beta;
      PWMDAC_MACRO(4,pwmdac1);

      //DlogCh1 = (int16)(Volt_CurrPara.PhaseU_Curr);
     // DlogCh2 = (int16)(Volt_CurrPara.PhaseV_Curr);
          DlogCh1 = (int16)_IQtoIQ15(svpwm1.Ta);
          DlogCh2 = (int16)_IQtoIQ15(svpwm1.Tb-svpwm1.Tc);
          DlogCh3 = (int16)_IQtoIQ15(volt1.VphaseB);
          DlogCh4 = (int16)_IQtoIQ15(volt1.VphaseA);
          dlog.update(&dlog);
          FOC_Control_TestPara( );// 将相关参数CAN发送上位机  速度 。频率。 dq轴电流。电压。角度位置等

          EPwm1Regs.ETCLR.bit.INT = 1; // 使能中断
          PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;
#endif
#if(BUILDLEVEL==LEVEL3)
          float32_t Desired_Speed_RPM = 300.0f;

          // 计算对应的电气频率
          float Electrical_Frequency = (Desired_Speed_RPM * POLES) / 60.0f;

          // 计算SpeedRef
          SpeedRef = _IQ(Electrical_Frequency / BASE_FREQ);
          if(lsw==0){
        	  rc1.TargetValue = 0;
          }
          else{
        	  rc1.TargetValue = SpeedRef;
          }
          RC_MACRO(rc1);


          rg1.StepAngleMax=_IQ(BASE_FREQ*T);
          rg1.Freq = rc1.SetPointValue;
          RG_MACRO(rg1);


          _iq scaledFactor = _IQ(2 * 0.909);
          clark1.abc=_IQ((float32_t)AdcMirror.ADCRESULT3/4096)-offsetA; // Phase A curr.
          clark1.As = _IQmpy(clark1.abc,scaledFactor);
          clark1.bbc=_IQ((float32_t)AdcMirror.ADCRESULT2/4096)-offsetB;
          clark1.Bs=_IQmpy(clark1.bbc,scaledFactor);
          CLARKE_MACRO(clark1);


          park1.Alpha = clark1.Alpha;
          park1.Beta =  clark1.Beta;
          if(lsw==0){
        	  park1.Angle = 0;
          }
          else if(lsw==1){
        	  park1.Angle = rg1.Out;
          }
          park1.Sine = _IQsinPU(park1.Angle);
          park1.Cosine = _IQcosPU(park1.Angle);
          PARK_MACRO(park1);

          if(lsw==0){
        	  pi_iq.Ref = 0;
          }
          else if(lsw==1){
        	  pi_iq.Ref = IqRef;
          }
          pi_iq.Fbk = park1.Qs;
          PI_MACRO(pi_iq);


          if(lsw==0){
        	  pi_id.Ref = _IQ(0.05);
          }
          else{
        	  pi_id.Ref = IdRef;
          }
          pi_id.Fbk = park1.Ds;
          PI_MACRO(pi_id);


          ipark1.Ds = pi_id.Out;
          ipark1.Qs = pi_iq.Out;

          ipark1.Sine = park1.Sine;
          ipark1.Cosine = park1.Cosine;
          IPARK_MACRO(ipark1);

           volatile Uint8 AD2Si=0;
          	 SAMPLE_OFF;
          	 Delay_100ns(6);  // 6/8.192M+20ns=500ns
          	 CS_RD_OFF;
          	 Delay_100ns(1);   // 100ns
          	 RDVEL_ON;
          	 AD2Si=1; AD2Si=2; // 停顿延时2个时钟
          	 Delay_100ns(1);   // 100n
          	 resolver1.tempresolver = (float32_t)Read_IO_interface();
          	 resolver1.TempResolver = resolver1.tempresolver*RESOLVER_POSITION;
             resolver1.UncorrectedPosition_degrees = resolver1.TempResolver;
             resolver1.TempResolver = resolver1.TempResolver - resolver1.mResolverOffset_degrees;
             if(resolver1.TempResolver<0.0f){
            	 resolver1.TempResolver +=360.0f;
             }
             resolver1.MechTheta = resolver1.TempResolver;
             resolver1.MechTheta = fmod(resolver1.MechTheta,360.0f);
             resolver1.TempResolver = resolver1.TempResolver *resolver1.mNumberOfPoles;
             resolver1.TempResolver = fmod(resolver1.TempResolver,360.0);
             resolver1.ElecTheta = _IQ(resolver1.TempResolver*(1.0f / 360.0f));
             resolver1.angle_difference = resolver1.MechTheta - resolver1.previous_angle;
            if (resolver1.angle_difference > 180.0f) {
            	 resolver1.angle_difference -= 360.0f;
                 } else if (resolver1.angle_difference < -180.0f) {
                	 resolver1.angle_difference += 360.0f;
                 }

                 // 判断旋转方向
                 if (resolver1.angle_difference > threshold) {
                     direction = 1;    // 正向旋转
                 } else if (resolver1.angle_difference < -threshold) {
                     direction = 0;   // 反向旋转
                 } else {
                     direction = 2;    // 错误
                 }

                 // 将方向存储到 resolver1.Direction
                 resolver1.Direction = direction;
                 resolver1.previous_angle =resolver1.MechTheta;
             CS_RD_ON;
             RDVEL_ON;
             SAMPLE_ON;
             //resolver1.Direction = GpioDataRegs.GPBDAT.bit.GPIO41;//如果采用飞线将DIR接到GPIO41

             speed1.ElecTheta = resolver1.ElecTheta;
             speed1.Direction = (int32)(resolver1.Direction);
             SPEED_FR_MACRO(speed1);

           svpwm1.Ualpha = ipark1.Alpha;
           svpwm1.Ubeta = ipark1.Beta;
           SVGENDQ_MACRO(svpwm1);


           float DcBusVoltage_Float = (AdcMirror.ADCRESULT4 / 4096.0) * 3.3;//(AdcMirror.ADCRESULT4 / 4096.0) * 3.3
           volt1.DcBusVolt = _IQ24(DcBusVoltage_Float);
           volt1.MfuncV1 = svpwm1.Ta;
           volt1.MfuncV2 = svpwm1.Tb;
           volt1.MfuncV3 = svpwm1.Tc;
           PHASEVOLTAGE_MACRO(volt1);
           smo1.Ialpha = clark1.Alpha;
                      	smo1.Ibeta  = clark1.Beta;
                       smo1.Valpha = volt1.Valpha;
                       smo1.Vbeta  = volt1.Vbeta;
                    	SMO_MACRO(smo1)

                        speed3.EstimatedTheta = smo1.Theta;
                        SE_MACRO(speed3)
           pwm1.MfuncC1 = svpwm1.Ta;
           pwm1.MfuncC2 = svpwm1.Tb;
           pwm1.MfuncC3 = svpwm1.Tc;
           PWM_MACRO(1,2,3,pwm1);

          pwmdac1.MfuncC1 =resolver1.ElecTheta;
          pwmdac1.MfuncC2 =smo1.Theta;
          PWMDAC_MACRO(4,pwmdac1);

          DlogCh1 = (int16)_IQtoIQ15(pi_iq.Out);
          DlogCh2 = (int16)_IQtoIQ15(resolver1.ElecTheta);
          DlogCh3 = (int16)_IQtoIQ15(volt1.VphaseB);
          DlogCh4 = (int16)_IQtoIQ15(volt1.VphaseA);
          dlog.update(&dlog);


           FOC_Control_TestPara( );// 将相关参数CAN发送上位机  速度 。频率。 dq轴电流。电压。角度位置等

          EPwm1Regs.ETCLR.bit.INT = 1; // 使能中断
         PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;
#endif
#if(BUILDLEVEL==LEVEL4)
         //gMotorVars.Desired_Speed_RPM = 100.0f;
         motor_run=EnableFlag;
         // 计算对应的电气频率
         float Electrical_Frequency = (gMotorVars.Desired_Speed_RPM * 4) / 60.0f;

         // 计算SpeedRef
         SpeedRef = _IQ(Electrical_Frequency / BASE_FREQ);
         if(lsw==0)rc1.TargetValue = 0;
             else rc1.TargetValue = SpeedRef;
         	RC_MACRO(rc1)

         // ------------------------------------------------------------------------------
         //  Connect inputs of the RAMP GEN module and call the ramp generator macro
         // ------------------------------------------------------------------------------
         rg1.StepAngleMax=_IQ(BASE_FREQ*T);
         rg1.Freq = rc1.SetPointValue;
         	RG_MACRO(rg1)

         // ------------------------------------------------------------------------------
         //  Measure phase currents, subtract the offset and normalize from (-0.5,+0.5) to (-1,+1).
         //	Connect inputs of the CLARKE module and call the clarke transformation macro
         // ------------------------------------------------------------------------------
         _iq scaledFactor = _IQ(2 * 0.909);
         clark1.abc=_IQ((float32_t)AdcMirror.ADCRESULT3/4096)-offsetA; // Phase A curr.
         clark1.As = _IQmpy(clark1.abc,scaledFactor);
         clark1.bbc=_IQ((float32_t)AdcMirror.ADCRESULT2/4096)-offsetB;
         clark1.Bs=_IQmpy(clark1.bbc,scaledFactor);
         CLARKE_MACRO(clark1);

         // ------------------------------------------------------------------------------
         //  Connect inputs of the PARK module and call the park trans. macro
         // ------------------------------------------------------------------------------
         	park1.Alpha = clark1.Alpha;
         	park1.Beta = clark1.Beta;

         	if(lsw==0) park1.Angle = 0;
         	else if(lsw==1) park1.Angle = rg1.Out;
         	else park1.Angle = resolver1.ElecTheta;

         	park1.Sine = _IQsinPU(park1.Angle);
         	park1.Cosine = _IQcosPU(park1.Angle);

         	PARK_MACRO(park1)

         // ------------------------------------------------------------------------------
         //    Connect inputs of the PI module and call the PID speed controller macro
         // ------------------------------------------------------------------------------
            if (SpeedLoopCount==SpeedLoopPrescaler)
              {
               pi_spd.Ref = rc1.SetPointValue;
               pi_spd.Fbk = speed1.Speed;
         	  PI_MACRO(pi_spd);
               SpeedLoopCount=1;
              }
         	else SpeedLoopCount++;

         	if(lsw!=2)	{pi_spd.ui=0; pi_spd.i1=0;}

         // ------------------------------------------------------------------------------
         //    Connect inputs of the PI module and call the PID IQ controller macro
         // ------------------------------------------------------------------------------
         	if(lsw==0) pi_iq.Ref = 0;
             else if(lsw==1) pi_iq.Ref = IqRef;
             else pi_iq.Ref = pi_spd.Out;
         	pi_iq.Fbk = park1.Qs;
         	PI_MACRO(pi_iq)

         // ------------------------------------------------------------------------------
         //    Connect inputs of the PI module and call the PID ID controller macro
         // ------------------------------------------------------------------------------
         	if(lsw==0) pi_id.Ref = _IQ(0.05);
             else pi_id.Ref = IdRef;
         	pi_id.Fbk = park1.Ds;
         	PI_MACRO(pi_id)

         // ------------------------------------------------------------------------------
         //  Connect inputs of the INV_PARK module and call the inverse park trans. macro
         // ------------------------------------------------------------------------------
            ipark1.Ds = pi_id.Out;
            ipark1.Qs = pi_iq.Out;
         	ipark1.Sine=park1.Sine;
            ipark1.Cosine=park1.Cosine;
         	IPARK_MACRO(ipark1)

             volatile Uint8 AD2Si=0;
         	 SAMPLE_OFF;
         	 Delay_100ns(6);  // 6/8.192M+20ns=500ns
         	 CS_RD_OFF;
         	 Delay_100ns(1);   // 100ns
         	 RDVEL_ON;
         	 AD2Si=1; AD2Si=2; // 停顿延时2个时钟
         	 Delay_100ns(1);   // 100n
         	 resolver1.tempresolver = (float32_t)Read_IO_interface();
         	 resolver1.TempResolver = resolver1.tempresolver*RESOLVER_POSITION;
            resolver1.UncorrectedPosition_degrees = resolver1.TempResolver;
            resolver1.TempResolver = resolver1.TempResolver - resolver1.mResolverOffset_degrees;
            if(resolver1.TempResolver<0.0f){
           	 resolver1.TempResolver +=360.0f;
            }
            resolver1.MechTheta = resolver1.TempResolver;
            resolver1.MechTheta = fmod(resolver1.MechTheta,360.0f);
            resolver1.TempResolver = resolver1.TempResolver *resolver1.mNumberOfPoles;
            resolver1.TempResolver = fmod(resolver1.TempResolver,360.0);
            resolver1.ElecTheta = _IQ(resolver1.TempResolver*(1.0f / 360.0f));

            UpdateRotationCount(&resolver1, positive_threshold, negative_threshold);
            MotionControl(&resolver1);

            //ControlMotor(&resolver1,&gMotorVars);

            CS_RD_ON;
            RDVEL_ON;
            SAMPLE_ON;
            //resolver1.Direction = GpioDataRegs.GPBDAT.bit.GPIO41;//如果采用飞线将DIR接到GPIO41

            speed1.ElecTheta = resolver1.ElecTheta;
            speed1.Direction = (int32)(resolver1.Direction);
            dir_fwd = (speed1.Direction != 0);
            SPEED_FR_MACRO(speed1);

          svpwm1.Ualpha = ipark1.Alpha;
          svpwm1.Ubeta = ipark1.Beta;
          SVGENDQ_MACRO(svpwm1);


          float DcBusVoltage_Float = (AdcMirror.ADCRESULT4 / 4096.0) * 3.3;//(AdcMirror.ADCRESULT4 / 4096.0) * 3.3
          volt1.DcBusVolt = _IQ24(DcBusVoltage_Float);
          volt1.MfuncV1 = svpwm1.Ta;
          volt1.MfuncV2 = svpwm1.Tb;
          volt1.MfuncV3 = svpwm1.Tc;
          PHASEVOLTAGE_MACRO(volt1);

          pwm1.MfuncC1 = svpwm1.Ta;
          pwm1.MfuncC2 = svpwm1.Tb;
          pwm1.MfuncC3 = svpwm1.Tc;
          PWM_MACRO(1,2,3,pwm1);

         pwmdac1.MfuncC1 =clark1.As;
         pwmdac1.MfuncC2 =rg1.Out;
         PWMDAC_MACRO(4,pwmdac1);

         DlogCh1 = (int16)_IQtoIQ15(pi_iq.Out);
         DlogCh2 = (int16)_IQtoIQ15(resolver1.ElecTheta);
         DlogCh3 = (int16)_IQtoIQ15(volt1.VphaseB);
         DlogCh4 = (int16)_IQtoIQ15(volt1.VphaseA);

         motorSt.BusVolat=volt1.DcBusVolt;
         motorSt.adc_raw= (float32_t)(((AdcMirror.ADCRESULT1/4096.0)*3.3)-1.7);
         motorSt.Current = _IQ((float32_t)10.0f*motorSt.adc_raw);
         motorSt.SpeedRpm = speed1.SpeedRpm;
         motorSt.NowPos_cnt = resolver1.DiverseCount;
         motorSt.ABsPos_cnt = resolver1.RotationCount/29;
         motorSt.StatusByte = build_status_byte();

        dlog.update(&dlog);
        EPwm1Regs.ETCLR.bit.INT = 1; // 使能中断
        PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;

#endif
}
interrupt void Scib_rx_ISR(void){
   uint8_t ch = ScibRegs.SCIRXBUF.all & 0xFF;
   Protocol_ProcessRx(ch);
   PieCtrlRegs.PIEACK.all |= PIEACK_GROUP9;
}

void SwitchOffPWMAndSetRunningMotorModeToStop(void)
{
      EPwm1Regs.CMPA.half.CMPA = 0u;
      EPwm2Regs.CMPA.half.CMPA = 0u;
      EPwm3Regs.CMPA.half.CMPA = 0u;

}

static inline uint8_t build_status_byte(void)
{
    uint8_t v = 0;
    if (brake_hold)      v |= (1<<7);
    if (pos_protect)     v |= (1<<5);
    if (over_current)    v |= (1<<4);
    if (sys_reset)       v |= (1<<2);
    if (motor_run)       v |= (1<<1);
    if (dir_fwd)         v |= (1<<0);
    return v;
}



//===========================================================================
// No more.
//===========================================================================

