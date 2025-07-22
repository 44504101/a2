//############################################################
// FILE:   SCI_RS232.h
// Created on: 2017年12月10日
// Author: XQ
// summary: Header file  and definition
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//版权所有，盗版必究
//DSP28335永磁同步电机控制开发板
//硕历电子
//网址: https://shuolidianzi.taobao.com
//Author-QQ: 616264123
//电机控制QQ群：736369775
//############################################################

#include "DSP2833x_Project.h"

#ifndef SCI_RS232_H
#define SCI_RS232_H



void SCI_RS232TX_sen(void);  // RS232 串口发送函数
void scib_fifo_init(void);    // 串口 fifo模式

#endif  // end of SCI_RS232.h definition

//===========================================================================
// End of file.
//===========================================================================
