//############################################################
// FILE:  SCI_RS232.c
// Created on: 2017年12月10日
// Author: XQ
// summary: SCI_RS232
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//版权所有，盗版必究
//DSP28335永磁同步电机控制开发板
//硕历电子
//网址: https://shuolidianzi.taobao.com
//Author-QQ: 616264123
//电机控制QQ群：736369775
//############################################################

#include "Drive_include.h"


void scib_fifo_init(void)
{
   ScibRegs.SCICCR.all =0x0007;
   ScibRegs.SCICTL1.all =0x0003;

   SciaRegs.SCICTL2.bit.TXINTENA =1;
   ScibRegs.SCICTL2.bit.RXBKINTENA =1;
   ScibRegs.SCIHBAUD = 0x0001;
   ScibRegs.SCILBAUD =0xE7 ;  //SCI_PRD 194
   ScibRegs.SCICCR.bit.LOOPBKENA =0;
   ScibRegs.SCIFFTX.all=0xC028;
   ScibRegs.SCIFFRX.all=0x0028;
   ScibRegs.SCIFFCT.all=0x00;

   ScibRegs.SCICTL1.all =0x0023;
   ScibRegs.SCIFFTX.bit.TXFIFOXRESET=1;
   ScibRegs.SCIFFRX.bit.RXFIFORESET=1;

}

void  SCI_RS232TX_sen(void)
 {
        Uint16 i,SCI232_txdr[8];
   	{

   	SCI232_txdr[0] = 0xaa ;  // 打包数据帧头aa
    SCI232_txdr[1] = 0xef ;  //其他位数据用户可以自行根据需求定义
   	SCI232_txdr[2] = 0x14 ;   //
   	SCI232_txdr[3] = 0x78 ;     //
   	SCI232_txdr[4] = 0x96 ;   //
   	SCI232_txdr[5]=  0x98 ;    //
    SCI232_txdr[6] = 0xac ;  //
   	SCI232_txdr[7] = 0xcc ;  // 打包数据帧尾cc

   	    for(i=0; i<8; i++)
   	    {
   	      while (ScibRegs.SCIFFTX.bit.TXFFST != 0){}    // 修改  寄存器 ScibRegs
   	      ScibRegs.SCITXBUF=SCI232_txdr[i];    // Send data
   	    }
   	}
}

//===========================================================================
// No more.
//===========================================================================

