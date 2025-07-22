//############################################################
// FILE:  CpuTimer.c
// Created on: 2017年12月10日
// Author: XQ
// summary: CpuTimer
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//版权所有，盗版必究
//DSP28335永磁同步电机控制开发板
//硕历电子
//网址: https://shuolidianzi.taobao.com
//Author-QQ: 616264123
//电机控制QQ群：736369775
//############################################################

#include "Drive_include.h"
#include "Task_manager.h"

void InitCpuTimer0(void)
{
	    EALLOW;
	    // Initialize address pointers to respective timer registers:
	    CpuTimer0.RegsAddr = &CpuTimer0Regs;
	    // Initialize timer period to maximum:
	    CpuTimer0Regs.PRD.all  = 0xFFFFFFFF;
	    // Initialize pre-scale counter to divide by 1 (SYSCLKOUT):
	    CpuTimer0Regs.TPR.all  = 0;
	    CpuTimer0Regs.TPRH.all = 0;
	    // Make sure timer is stopped:
	    CpuTimer0Regs.TCR.bit.TSS = 1;
	    // Reload all counter register with period value:
	    CpuTimer0Regs.TCR.bit.TRB = 1;
	    // Reset interrupt counters:
	    CpuTimer0.InterruptCount = 0;
	    // 150MHz CPU Freq, 1 second Period (in uSeconds)
	    ConfigCpuTimer(&CpuTimer0, 150, 100);  // 100us定时器

	    CpuTimer0Regs.TCR.all = 0x4001; // Use write-only instruction to set TSS bit = 0
	    EDIS;
}

interrupt void cpu_timer0_isr(void)
{
	 EINT;
    Timer_Task_Count( );
   // Acknowledge this interrupt to receive more interrupts from group 1
   PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}



//===========================================================================
// No more.
//===========================================================================
