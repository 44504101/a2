/*
 * RS485.h
 *
 *  Created on: 2025年4月22日
 *      Author: HP
 */

#ifndef DRIVE_INCLUDE_RS485_H_
#define DRIVE_INCLUDE_RS485_H_

#include"stdint.h"
#include"dlog4ch-HVPM_Sensored.h"
#include"stdbool.h"
#define SCI_B_BASE_ADDRESS  0x00007750u     ///< Base address for SCI-B 见手册337页
#define SCI_A_BASE_ADDRESS  0x00007050u

#define SCICCR_OFFSET       0x0000u         ///< Offset from base for SCICCR  0000-7750是SCICCR，不加偏执就是CCR，加1位偏置就是CTL1
#define SCICTL1_OFFSET      0x0001u         ///< Offset from base for SCICTL1
#define SCIHBAUD_OFFSET     0x0002u         ///< Offset from base for SCIHBAUD
#define SCILBAUD_OFFSET     0x0003u         ///< Offset from base for SCILBAUD
#define SCICTL2_OFFSET      0x0004u         ///< Offset from base for SCICTL2

#define SCIFFTX_OFFSET      0x000Au         ///< Offset from base for SCIFFTX
#define SCIFFRX_OFFSET      0x000Bu         ///< Offset from base for SCIFFRX
#define SCIFFCT_OFFSET      0x000Cu         ///< Offset from base for SCIFFCT
#define SCIPRI_OFFSET       0x000Fu         ///< Offset from base for SCIPRI

#define MASTER_CLOCK_FREQUENCY  (uint32_t)37500000u     ///< Master clock frequency.
#define FALSE   (bool_t)(0)                 //lint !e960 Disallowed definition for macro FALSE.
#define TRUE    (bool_t)(1)                 //lint !e960 Disallowed definition for macro TRUE.
typedef unsigned char   uint8_t;        //lint !e1960 Re-use of C++ identifier
typedef int16_t         bool_t;         // Make boolean type native width of processor.
//#define NULL    (void*)0                    //lint !e960 !e9071 Macro reserved by compiler
typedef void(*pTriggerTimerFunction)(void);

extern void (*genericIO_16bitWrite)(const uint32_t address, const uint16_t data);
/// Defining instance of the global function pointer genericIO_16bitWrite.
/// The pointer is initialised to point to IO_16bitWrite_Impl.
//lint -e{956} External variable.  Pointer doesn't change so it's fine.
//void (*genericIO_16bitWrite)(const uint32_t address, const uint16_t data) = IO_16bitWrite_Impl;
typedef enum
{
    SCI_A                   = 0,        ///< Serial port A.
    SCI_B                   = 1,        ///< Serial port B.
    SCI_C                   = 2,        ///< Serial port C.
    SCI_NUMBER_OF_PORTS     = 3         ///< The number of serial ports.
} ESCIModule_t;

void SCI_Open(const ESCIModule_t module);
void RS485_SetTX(void);
void RS485_SetRX(void);
void SCIB_SendByte(uint8_t b);
void SCIB_SendBuffer(const uint8_t *buf, uint16_t len);
bool_t          SCI_BaudRateSet(const ESCIModule_t module,
                                const uint32_t iLspClk_Hz,
                                const uint32_t iBaudRate);
#endif /* DRIVE_INCLUDE_RS485_H_ */
