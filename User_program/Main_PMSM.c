//############################################################
// FILE:  Main_PMSM.c
// Created on: 2017��12��10��
// Author: XQ
// summary: PMSM  control  with Resolver
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//��Ȩ���У�����ؾ�
//DSP28335����PMSM��������뿪����
//˶������
//��ַ: https://shuolidianzi.taobao.com
//�޸�����:2017/12/18
//�汾��V17_1
//Author-QQ: 616264123
//�������QQȺ��736369775
//############################################################
#include "Main_PMSM.h"
DLOG_4CH dlog = DLOG_4CH_DEFAULTS;
int16 DlogCh1 = 0;
int16 DlogCh2 = 0;
int16 DlogCh3 = 0;
int16 DlogCh4 = 0;
volatile Uint16 EnableFlag = false;
volatile uint16_t tick = 0;
Uint16 SpeedLoopCount = 1;
Uint16 SpeedLoopPrescaler = 10;
Uint16 BackTicker = 0;
volatile struct EPWM_REGS *ePWM[] =
 				  { &EPwm1Regs,			//intentional: (ePWM[0] not used)
 						  	&EPwm1Regs,
 							&EPwm2Regs,
 							&EPwm3Regs,
 							&EPwm4Regs,
 							&EPwm5Regs,
 							&EPwm6Regs,
 							&EPwm7Regs,
				  };


// Used to indirectly access eQEP module
volatile struct EQEP_REGS *eQEP[] =
 				  { &EQep1Regs,
 				  	&EQep1Regs,
					&EQep2Regs,
				  };

// Used to indirectly access eQEP module
volatile struct ECAP_REGS *eCAP[] =
 				  { &ECap1Regs,
 				  	&ECap1Regs,
					&ECap2Regs,
					&ECap3Regs,
					&ECap4Regs,
					&ECap5Regs,
					&ECap6Regs,
				  };
float32 T = 0.001/ISR_FREQUENCY;
PWMGEN pwm1 = PWMGEN_DEFAULTS;
PWMDAC pwmdac1 = PWMDAC_DEFAULTS;
SPEED_MEAS speed1 = SPEED_MEAS_DEAFAULTS;
// Instance a position estimator
// Instance a sliding-mode position observer constant Module
SMOPOS_CONST smo1_const = SMOPOS_CONST_DEFAULTS;
SMOPOS smo1 = SMOPOS_DEFAULTS;
SPEED_ESTIMATION speed3 = SPEED_ESTIMATION_DEFAULTS;
void main(void)
{
   InitSysCtrl();   // ��ʼ�� ϵͳ, ���໷ , ���Ź���
   InitPieCtrl();   //�ж����������
   InitPieVectTable(); //�ж�������

   //Initalize GPIO:
   Init_Gpio_LED();   // LED��GPIO��ʼ��
   Init_3MotorGpio(); // �������6PWM��GPIO��ʼ��
   Init_DACEPWMGpio();// DAC��PWM��GPIO��ʼ��
   InitScirs485bGpio();
   Init_ECanbGpio( );  //CAN��GPIO��ʼ��
   Init_Gpio_AD2S1205();

   MemCopy(&RamfuncsLoadStart, &RamfuncsLoadEnd, &RamfuncsRunStart);  // ramfuncs
   InitFlash();

   AD2S1205_Init();
   SCI_Open(SCI_B);
   (void)SCI_BaudRateSet(SCI_B, MASTER_CLOCK_FREQUENCY, (uint32_t)57600u);

while(EnableFlag==false){
	BackTicker++;
}
   //EPWM3_int() ;  // �������6PWM�ĳ�ʼ��
   pwm1.PeriodMax = SYSTEM_FREQUENCY*1000000*T/2; // Prescaler X1 (T1), ISR period = T x 1
   pwm1.HalfPerMax=pwm1.PeriodMax/2;
   pwm1.Deadband  = 2.0*SYSTEM_FREQUENCY;     	    // 120 counts -> 2.0 usec for TBCLK = SYSCLK/1
   PWM_INIT_MACRO(1,2,3,pwm1);

   //PWMDAC_int();  // DAC��PWM�ĳ�ʼ��
    pwmdac1.PeriodMax=500;		   	// @60Mhz, 1500->20kHz, 1000-> 30kHz, 500->60kHz
   	pwmdac1.HalfPerMax=pwmdac1.PeriodMax/2;
   	PWMDAC_INIT_MACRO(4,pwmdac1) 	// PWM 6A,6B
   	//PWMDAC_INIT_MACRO(7,pwmdac1) 	// PWM 7A,7B

    pi_spd.Kp_base=_IQ(1.3);
	pi_spd.Ki_base=_IQ(T*SpeedLoopPrescaler/0.2);
	pi_spd.Kp = pi_spd.Kp_base;
	pi_spd.Ki = pi_spd.Ki_base;
	pi_spd.Kaw=_IQ(T/0.04);
	pi_spd.Umax =_IQ(0.95);
	pi_spd.Umin =_IQ(-0.95);

   //STOP_CAR();    // ͣ������ƹ�PWM����
   ADC_SOC_int( ); //ADC��ʼ������
	pi_id.Kp_base=_IQ(1.05);
	pi_id.Ki_base=_IQ(T/0.04);
	pi_id.Kp = pi_id.Kp_base;
	pi_id.Ki = pi_id.Ki_base;
	pi_id.Kaw=_IQ(T/0.06);
	pi_id.Umax =_IQ(1.0);
	pi_id.Umin =_IQ(-1.0);

	pi_iq.Kp_base = _IQ(1.05);  // ��������ϵͳ���Ե���
	pi_iq.Ki_base = _IQ(T/0.04);  // ��������ϵͳ���Ե���
	pi_iq.Kp = pi_iq.Kp_base;
	pi_iq.Ki = pi_iq.Ki_base;
	pi_iq.Kaw = _IQ(T / 0.06);  // �����ֱ������棬����ϵͳ�������
	pi_iq.Umax = _IQ(1.0);      // ���������������޷�
	pi_iq.Umin = _IQ(-1.0);     // �������������С�޷�

    speed1.K1 = _IQ21(1/(BASE_FREQ*T));
    speed1.K2 = _IQ(1/(1+T*2*PI*5));  // Low-pass cut-off frequency
    speed1.K3 = _IQ(1)-speed1.K2;
    speed1.BaseRpm = 120*(BASE_FREQ/POLES);

   //scib_fifo_init( ); //���ڵĳ�ʼ������
   ECanB_init( );  //CAN�ĳ�ʼ������
   InitCpuTimer0( );  //��ʱ��0�ĵĳ�ʼ������
   Task_Manage_List_Init();

      dlog.iptr1 = &DlogCh1;
      dlog.iptr2 = &DlogCh2;
  	  dlog.iptr3 = &DlogCh3;
      dlog.iptr4 = &DlogCh4;
      dlog.trig_value = 0x1;
      dlog.size = 0x190;
      dlog.prescalar = 5;
      dlog.init(&dlog);

 	EALLOW;	// This is needed to write to EALLOW protected registers
  	PieVectTable.EPWM1_INT = &OffsetISR;
  	PieVectTable.SCIRXINTB = &Scib_rx_ISR;
  	EDIS;

  // Enable PIE group 3 interrupt 1 for EPWM1_INT
      PieCtrlRegs.PIEIER3.bit.INTx1 = 1;
      PieCtrlRegs.PIEIER9.bit.INTx4 = 1;   // Group-9 / INT4 = SCIRXINTB

  // Enable CNT_zero interrupt using EPWM1 Time-base
      EPwm1Regs.ETSEL.bit.INTEN = 1;   // Enable EPWM1INT generation
      EPwm1Regs.ETSEL.bit.INTSEL = 1;  // Enable interrupt CNT_zero event
      EPwm1Regs.ETPS.bit.INTPRD = 1;   // Generate interrupt on the 1st event
  	  EPwm1Regs.ETCLR.bit.INT = 1;     // Enable more interrupts

  // Enable CPU INT3 for EPWM1_INT:
  	IER |= M_INT3|M_INT9;
  	//IER |= M_INT3;
  // Enable global Interrupts and higher priority real-time debug events:
  	EINT;   // Enable Global interrupt INTM
  	ERTM;	// Enable Global realtime interrupt DBGM
    while(1)
   {
    	//AD2SXBPare.AD2S_circle += AD2SXBPare.AD2S_angle_du/360.0f;
     Execute_Task_List_RUN( );
     if (tick >= 100) {        //
         SendMotorStatus();    //
         tick = 0;
     }
   }
}



//===========================================================================
// No more.
//===========================================================================

