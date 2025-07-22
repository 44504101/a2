//############################################################
// FILE:  ECANB.c
// Created on: 2017��12��10��
// Author: XQ
// summary: ECANB
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//��Ȩ���У�����ؾ�
//DSP28335����ͬ��������ƿ�����
//˶������
//��ַ: https://shuolidianzi.taobao.com
//Author-QQ: 616264123
//�������QQȺ��736369775
//############################################################

#include "Drive_include.h"



// ������500K ��׼֡  ֻ����0x3DE֡  ��������λ 0x3DE 0x12 0x58 0xAA 0x53 0xC4 0xAA 0x62 0xAF 0x08 Standard Data
//������500K ��׼֡  ֻ����0x568֡ ��������λ    0x568 0x60 0x54 0x55 0x2c 0x4b 0xaa 0x40 0x9e 0x08 Standard Data
void ECanB_init(void)
{
	struct ECAN_REGS ECanbShadow;

    EALLOW;
    ECanbShadow.CANTIOC.all = ECanbRegs.CANTIOC.all;
    ECanbShadow.CANTIOC.bit.TXFUNC = 1;
    ECanbRegs.CANTIOC.all = ECanbShadow.CANTIOC.all;

    ECanbShadow.CANRIOC.all = ECanbRegs.CANRIOC.all;
    ECanbShadow.CANRIOC.bit.RXFUNC = 1;
    ECanbRegs.CANRIOC.all = ECanbShadow.CANRIOC.all;

    /* Configure eCAN for HECC mode - (reqd to access mailboxes 16 thru 31) */
    ECanbShadow.CANMC.all = ECanbRegs.CANMC.all;
    ECanbShadow.CANMC.bit.SCB = 1;
    ECanbRegs.CANMC.all = ECanbShadow.CANMC.all;

    // Mailboxs can be written to 16-bits or 32-bits at a time
    // Write to the MSGID field of TRANSMIT mailboxes MBOX0 - 15
    ECanbMboxes.MBOX0.MSGID.all = 0x00000000;
    ECanbMboxes.MBOX1.MSGID.all = 0x00000000;
    ECanbMboxes.MBOX2.MSGID.all = 0x00000000;
    ECanbMboxes.MBOX3.MSGID.all = 0x00000000;
    ECanbMboxes.MBOX4.MSGID.all = 0x00000000;
    ECanbMboxes.MBOX5.MSGID.all = 0x00000000;
    ECanbMboxes.MBOX6.MSGID.all = 0x00000000;
    ECanbMboxes.MBOX7.MSGID.all = 0x00000000;
    ECanbMboxes.MBOX8.MSGID.all = 0x00000000;
    ECanbMboxes.MBOX9.MSGID.all = 0x00000000;
    ECanbMboxes.MBOX10.MSGID.all = 0x00000000;
    ECanbMboxes.MBOX11.MSGID.all = 0x00000000;
    ECanbMboxes.MBOX12.MSGID.all = 0x00000000;
    ECanbMboxes.MBOX13.MSGID.all = 0x00000000;
    ECanbMboxes.MBOX14.MSGID.all = 0x00000000;
    ECanbMboxes.MBOX15.MSGID.all = 0x00000000;
    ECanbMboxes.MBOX16.MSGID.all = 0x00000000;
    ECanbMboxes.MBOX17.MSGID.all = 0x00000000;
    ECanbMboxes.MBOX18.MSGID.all = 0x00000000;
    ECanbMboxes.MBOX19.MSGID.all = 0x00000000;
    ECanbMboxes.MBOX20.MSGID.all = 0x00000000;
    ECanbMboxes.MBOX21.MSGID.all = 0x00000000;
    ECanbMboxes.MBOX22.MSGID.all = 0x00000000;
    ECanbMboxes.MBOX23.MSGID.all = 0x00000000;
    ECanbMboxes.MBOX24.MSGID.all = 0x00000000;
    ECanbMboxes.MBOX25.MSGID.all = 0x00000000;
    ECanbMboxes.MBOX26.MSGID.all = 0x00000000;
    ECanbMboxes.MBOX27.MSGID.all = 0x00000000;
    ECanbMboxes.MBOX28.MSGID.all = 0x00000000;
    ECanbMboxes.MBOX29.MSGID.all = 0x00000000;
    ECanbMboxes.MBOX30.MSGID.all = 0x00000000;
    ECanbMboxes.MBOX31.MSGID.all = 0x00000000;

    // TAn, RMPn, GIFn bits are all zero upon reset and are cleared again
    //  as a matter of precaution.

	ECanbRegs.CANTA.all = 0xFFFFFFFF;   /* Clear all TAn bits */

	ECanbRegs.CANRMP.all = 0xFFFFFFFF;  /* Clear all RMPn bits */

	ECanbRegs.CANGIF0.all = 0xFFFFFFFF; /* Clear all interrupt flag bits */
	ECanbRegs.CANGIF1.all = 0xFFFFFFFF;

    // Request permission to change the configuration registers
    ECanbShadow.CANMC.all = ECanbRegs.CANMC.all;
    ECanbShadow.CANMC.bit.CCR = 1;
    ECanbRegs.CANMC.all = ECanbShadow.CANMC.all;

    // Wait until the CPU has been granted permission to change the
    // configuration registers
    // Wait for CCE bit to be set..
    do
    {
      ECanbShadow.CANES.all = ECanbRegs.CANES.all;
    } while(ECanbShadow.CANES.bit.CCE != 1 );

    ECanbShadow.CANBTC.all =0;

    ECanbShadow.CANBTC.bit.BRPREG = 9;    // (BRPREG + 1) = 10 feeds a 15 MHz CAN clock
    ECanbShadow.CANBTC.bit.TSEG2REG = 5 ; // to the CAN module. (150 / 10 = 15)
    ECanbShadow.CANBTC.bit.TSEG1REG = 7;  // Bit time = 15=5+1+7+1+1

    ECanbShadow.CANBTC.bit.SAM = 1;
    ECanbRegs.CANBTC.all = ECanbShadow.CANBTC.all;

    ECanbShadow.CANMC.all = ECanbRegs.CANMC.all;
    ECanbShadow.CANMC.bit.CCR = 0;
    ECanbRegs.CANMC.all = ECanbShadow.CANMC.all;

    do
    {
      ECanbShadow.CANES.all = ECanbRegs.CANES.all;
    } while(ECanbShadow.CANES.bit.CCE != 0 );

    ECanbRegs.CANME.all = 0;
    EDIS;

    EALLOW;
    //  ������������  ��׼֡ �ͱ�׼����

    ECanbMboxes.MBOX1.MSGID.all = 0x04000000;  // 0x100
    ECanbMboxes.MBOX2.MSGID.all = 0x04040000;  // 0x101
    ECanbMboxes.MBOX3.MSGID.all = 0x04080000;  // 0x102
    ECanbMboxes.MBOX4.MSGID.all = 0x040C0000;  // 0x103
    ECanbMboxes.MBOX5.MSGID.all = 0x04100000;  // 0x104


	// ���� ��������
    ECanbMboxes.MBOX11.MSGID.all = 0x45400000;     //      0X150
    ECanbLAMRegs.LAM11.all=0x8003FFF;  //use the acceptance mask
    ECanbMboxes.MBOX12.MSGID.all = 0x45440000;     //  0X151
    ECanbLAMRegs.LAM12.all=0x8003FFF;  //use the acceptance mask



    // Configure Mailboxes 0-15 as Tx, 16-31 as Rx
    // Since this write is to the entire register (instead of a bit
    // field) a shadow register is not required.
    ECanbRegs.CANMD.all = 0x00001800;  // Configure the mailbox 3  as a receive mailbox   1 ����    0  ����
    ECanbRegs.CANOPC.all=0x00000000;   //

    // Enable all Mailboxes */
    // Since this write is to the entire register (instead of a bit
    // field) a shadow register is not required.
    ECanbRegs.CANME.all = 0x0000FFFF;    //  ��������

    // Specify that 8  zijie  will be sent/received
    ECanbMboxes.MBOX1.MSGCTRL.bit.DLC = 8;
    ECanbMboxes.MBOX2.MSGCTRL.bit.DLC = 8;
    ECanbMboxes.MBOX3.MSGCTRL.bit.DLC = 8;
    ECanbMboxes.MBOX4.MSGCTRL.bit.DLC = 8;
    ECanbMboxes.MBOX5.MSGCTRL.bit.DLC = 8;

    ECanbMboxes.MBOX1.MSGCTRL.bit.RTR=0;  // ���Ʒ������������֡��Զ��֡   RTR=1Զ��֡
    ECanbMboxes.MBOX2.MSGCTRL.bit.RTR=0;  // ���Ʒ������������֡��Զ��֡   RTR=1Զ��֡
    ECanbMboxes.MBOX3.MSGCTRL.bit.RTR=0;  // ���Ʒ������������֡��Զ��֡   RTR=1Զ��֡
    ECanbMboxes.MBOX4.MSGCTRL.bit.RTR=0;  // ���Ʒ������������֡��Զ��֡   RTR=1Զ��֡
    ECanbMboxes.MBOX5.MSGCTRL.bit.RTR=0;  // ���Ʒ������������֡��Զ��֡   RTR=1Զ��֡


    ECanbMboxes.MBOX11.MSGCTRL.bit.DLC = 8;
    ECanbMboxes.MBOX12.MSGCTRL.bit.DLC = 8;
    ECanbMboxes.MBOX11.MSGCTRL.bit.RTR=0;  // ���Ʒ������������֡��Զ��֡   RTR=1Զ��֡
    ECanbMboxes.MBOX12.MSGCTRL.bit.RTR=0;  // ���Ʒ������������֡��Զ��֡   RTR=1Զ��֡


    // Configure the eCAN for self test mode
    // Enable the enhanced features of the eCAN.

    ECanbShadow.CANMC.all = ECanbRegs.CANMC.all;  //
    ECanbShadow.CANMC.bit.STM = 0;                // Configure CAN for normal mode
    ECanbRegs.CANMC.all = ECanbShadow.CANMC.all;  //
    //�����ж�ʹ��
  /*  ECanbRegs.CANMIM.all = 0x00000008; //  �����ж�ʹ��        CANRFPԶ��֡�Ĵ���
   	//�����жϽ�������ECAN0INT
    ECanbRegs.CANMIL.all = 0;
    ECanbRegs.CANGIF0.all = 0xFFFFFFFF;
    ECanbRegs.CANGIF1.all = 0xFFFFFFFF;
   	//ECAN0INT�ж������߱�ʹ��
    ECanbRegs.CANGIM.bit.I0EN = 1;
      // ECanaRegs.CANGIM.bit.I1EN = 1;
    ECanbRegs.CANGIM.all =0X3FF07;
   */
    EDIS;
}


//===========================================================================
// No more.
//===========================================================================

