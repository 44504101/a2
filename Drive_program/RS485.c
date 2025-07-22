/*
 * RS485.c
 *
 *  Created on: 2025年4月22日
 *      Author: HP
 */

#include"RS485.h"
#include "Drive_include.h"
static void         ResetAllSCIRegisters(const uint32_t iBaseAddress);
static uint32_t     SetupSCIBaseAddress(const ESCIModule_t module);
typedef struct
{
    /* ---------- 接收侧 ---------- */
    uint8_t  *p_rxBuffer;           // 指向接收缓存区
    uint16_t  rxOffset;             // 当前已写入缓存区的字节数
    uint16_t  rxMaxLength;          // 接收缓存区最大长度

    pTriggerTimerFunction p_timerTrigger; // 指向“喂狗/超时计时器”触发函数
    bool_t    b_matchRequired;      // 是否需要做字符匹配
    uint8_t   matchCharacter;       // 要匹配的字符（如 '\n'）
    uint16_t  matchCounter;         // 已匹配到的字符个数
    void     *p_receiveSemaphore;   // 用于任务同步的接收信号量

    /* ---------- 发送侧 ---------- */
    const uint8_t *p_txBuffer;      // 指向待发送数据
    uint16_t       txOffset;        // 已发送字节数
    uint16_t       txMessageLength; // 整条消息长度
    void          *p_transmitSemaphore; // 发送完成信号量
} serialPortVars_t;

static serialPortVars_t m_serialPorts[SCI_NUMBER_OF_PORTS];

void SCI_Open(const ESCIModule_t module){
    uint32_t    baseAddress;
    uint16_t    requiredData;

    if (module < SCI_NUMBER_OF_PORTS)
        {
            baseAddress = SetupSCIBaseAddress(module); // 设置SCI的基地址
            ResetAllSCIRegisters(baseAddress);//初始化SCI所有的寄存器，全写0
            m_serialPorts[module].p_rxBuffer          = NULL;
            m_serialPorts[module].rxOffset            = 0u;
            m_serialPorts[module].rxMaxLength         = 0u;
            m_serialPorts[module].p_timerTrigger      = NULL;
            m_serialPorts[module].b_matchRequired     = FALSE;
            m_serialPorts[module].matchCharacter      = 0x00u;
            m_serialPorts[module].matchCounter        = 0u;
            m_serialPorts[module].p_receiveSemaphore  = NULL;
            m_serialPorts[module].p_txBuffer          = NULL;
            m_serialPorts[module].txOffset            = 0u;
            m_serialPorts[module].txMessageLength     = 0u;
            m_serialPorts[module].p_transmitSemaphore = NULL;

            requiredData =
                        ( (uint16_t)0u << 7 ) |     // 0: 1位停止位
                        ( (uint16_t)0u << 6 ) |     // 0: 奇校验
                        ( (uint16_t)0u << 5 ) |     // 0: 奇偶校验关闭
                        ( (uint16_t)0u << 4 ) |     // 0: 自测试模式禁用
                        ( (uint16_t)0u << 3 ) |     // 0: 空闲线模式协议
                        ( (uint16_t)7u << 0 );      // 111: 8位数据 (bits 2:0)
                    genericIO_16bitWrite(( baseAddress + SCICCR_OFFSET ), requiredData); //lint !e835    //SCICCR register表示communication control register

            requiredData =
                        ( (uint16_t)0u << 7 ) |     // 0: 读取为0，写入无效
                        ( (uint16_t)1u << 6 ) |     // 1: 接受错误中断使能
                        ( (uint16_t)1u << 5 ) |     // 1: 重新使能SCI
                        ( (uint16_t)0u << 4 ) |     // 0: 读取为0，写入无效
                        ( (uint16_t)0u << 3 ) |     // 0: 发送特征未选择
                        ( (uint16_t)0u << 2 ) |     // 0: 休眠模式禁用
                        ( (uint16_t)1u << 1 ) |     // 1: 发送器使能
                        ( (uint16_t)1u << 0 );      // 1: 接受到的数据送到 RXREG
                   genericIO_16bitWrite(( baseAddress + SCICTL1_OFFSET ), requiredData);

              requiredData =
                           ( (uint16_t)0u << 7 ) |     // 0: read only bit, writes have no effect
                           ( (uint16_t)0u << 6 ) |     // 0: read only bit, writes have no effect
                           ( (uint16_t)0u << 5 ) |     // 0:
                           ( (uint16_t)0u << 4 ) |     // 0: 保留位
                           ( (uint16_t)0u << 3 ) |     // 0:
                           ( (uint16_t)0u << 2 ) |     // 0:
                           ( (uint16_t)1u << 1 ) |     // 1: RXRDY\BRKDT 中断使能
                           ( (uint16_t)0u << 0 );      // 1: TXRDY 中断使能
                     genericIO_16bitWrite(( baseAddress + SCICTL2_OFFSET ), requiredData);
                     requiredData =
                                 ( (uint16_t)1u << 15 ) |    // 1: SCI FIFO 可以重新发送和接受
                                 ( (uint16_t)1u << 14 ) |    // 1: SCI FIFO 使能
                                 ( (uint16_t)1u << 13 ) |    // 1: 再次使能发送FIFO操作
                                 ( (uint16_t)0u << 12 ) |    // 0: read only bit, writes have no effect
                                 ( (uint16_t)0u << 11 ) |    // 0: read only bit, writes have no effect
                                 ( (uint16_t)0u << 10 ) |    // 0: read only bit, writes have no effect
                                 ( (uint16_t)0u << 9 ) |     // 0: read only bit, writes have no effect
                                 ( (uint16_t)0u << 8 ) |     // 0: read only bit, writes have no effect
                                 ( (uint16_t)0u << 7 ) |     // 0: read only bit, writes have no effect
                                 ( (uint16_t)1u << 6 ) |     // 1: 写人1清除第7位的发送 FIFO中断(TXFFINT)标志
                                 ( (uint16_t)0u << 5 ) |     // 0: TX FIFO interrupt based on match disabled
                                 ( (uint16_t)0u << 0 );      // 00000: TX FIFO interrupt depth = 0 (bits 4:0)
                      genericIO_16bitWrite(( baseAddress + SCIFFTX_OFFSET ), requiredData); // SCIFFTX表示SCI FIFO Transmit register

                      requiredData =
                          ( (uint16_t)0u << 15 ) |    // 0: read only bit, writes have no effect
                          ( (uint16_t)1u << 14 ) |    // 1: 写1清除第 15 位的接收 FIFO 溢出(RXFFOVF)标志
                          ( (uint16_t)1u << 13 ) |    // 1: SCI FIFO receive pointer enabled
                          ( (uint16_t)0u << 12 ) |    // 0: read only bit, writes have no effect
                          ( (uint16_t)0u << 11 ) |    // 0: read only bit, writes have no effect
                          ( (uint16_t)0u << 10 ) |    // 0: read only bit, writes have no effect
                          ( (uint16_t)0u << 9 ) |     // 0: read only bit, writes have no effect
                          ( (uint16_t)0u << 8 ) |     // 0: read only bit, writes have no effect
                          ( (uint16_t)0u << 7 ) |     // 0: read only bit, writes have no effect
                          ( (uint16_t)1u << 6 ) |     // 1: 写1清除第7位的 RXFFINT 标志
                          ( (uint16_t)1u << 5 ) |     // 1: 使能基于 RXFFIVL匹配(小于或等于的接收 FIFO中断
                          ( (uint16_t)1u << 0 );      // 00001: RX FIFO interrupt depth = 1 (bits 4:0)
                      genericIO_16bitWrite(( baseAddress + SCIFFRX_OFFSET ), requiredData);  // SCIFFRX表示SCI FIFO Receive register

                      requiredData =
                          ( (uint16_t)0u << 15 ) |    // 0: read only bit, writes have no effect
                          ( (uint16_t)1u << 14 ) |    // 1: 写1清除第 15位 ABD志
                          ( (uint16_t)0u << 13 ) |    // 0: 禁用自动波特率调整
                          ( (uint16_t)0u << 12 ) |    // 0: reserved, writes have no effect
                          ( (uint16_t)0u << 11 ) |    // 0: reserved, writes have no effect
                          ( (uint16_t)0u << 10 ) |    // 0: reserved, writes have no effect
                          ( (uint16_t)0u << 9 ) |     // 0: reserved, writes have no effect
                          ( (uint16_t)0u << 8 ) |     // 0: reserved, writes have no effect
                          ( (uint16_t)0u << 0 );      // 00000000: FIFO transfer delay = 0 (bits 7:0)
                      genericIO_16bitWrite(( baseAddress + SCIFFCT_OFFSET ), requiredData); // SCIFFCT表示SCI FIFO control register

                      requiredData =
                          ( (uint16_t)0u << 7 ) |     // 0: reserved bit, writes have no effect
                          ( (uint16_t)0u << 6 ) |     // 0: reserved bit, writes have no effect
                          ( (uint16_t)0u << 5 ) |     // 0: reserved bit, writes have no effect
                          ( (uint16_t)0u << 3 ) |     // 00: immediate stop on suspend (bits 4:3)
                          ( (uint16_t)0u << 2 ) |     // 0: reserved bit, writes have no effect
                          ( (uint16_t)0u << 1 ) |     // 0: reserved bit, writes have no effect
                          ( (uint16_t)0u << 0 );      // 0: reserved bit, writes have no effect
                      genericIO_16bitWrite(( baseAddress + SCIPRI_OFFSET ), requiredData); // SCIPRI表示SCI Priority control register
}
}

bool_t SCI_BaudRateSet(const ESCIModule_t module,
                       const uint32_t iLspClk_Hz,
                       const uint32_t iBaudRate)
{
    uint32_t    iResult;
    bool_t      b_validBaudRate = TRUE;
    uint16_t    iData;
    uint32_t    baseAddress;

    // Baud rate divider = (LSPCLK / (BAUD RATE * 8)) - 1
    // (taken from table 2.5 in SPRUFZ5A - SCI reference manual).
    iResult = ( iLspClk_Hz / ( iBaudRate * 8u ) ) - 1u;

    // If result overflows 16 bits then we can't set the baud rate.
    if (iResult > 65535u)
    {
        b_validBaudRate = FALSE;
    }
    else
    {
        baseAddress = SetupSCIBaseAddress(module);

        // Write LSB of baud rate - mask off everything except bottom 8 bits.
        //lint -e{921} Cast to uint16_t as the write function uses 16 bit.
        iData = (uint16_t)( iResult & 0x000000FFu );
        genericIO_16bitWrite(( baseAddress + SCILBAUD_OFFSET ), iData);

        // Write MSB of baud rate - shift down and mask off.
        //lint -e{921} Cast to uint16_t as the write function uses 16 bit.
        iData = (uint16_t)( ( iResult >> 8u ) & 0x000000FFu );
        genericIO_16bitWrite(( baseAddress + SCIHBAUD_OFFSET ), iData);
    }

    return b_validBaudRate;
}





static uint32_t SetupSCIBaseAddress(const ESCIModule_t module)
    {
        uint32_t address;

        switch (module)
        {
            case SCI_B:
                address = SCI_B_BASE_ADDRESS;
                break;
            default:
                address = SCI_A_BASE_ADDRESS;
                break;
        }

        return address;
    }


static void ResetAllSCIRegisters(const uint32_t iBaseAddress)
{
    genericIO_16bitWrite(( iBaseAddress + SCICTL1_OFFSET ), 0u);
    genericIO_16bitWrite(( iBaseAddress + SCICCR_OFFSET ), 0u); //lint !e835
    genericIO_16bitWrite(( iBaseAddress + SCICTL2_OFFSET ), 0u);
    genericIO_16bitWrite(( iBaseAddress + SCIFFTX_OFFSET ), 0u);
    genericIO_16bitWrite(( iBaseAddress + SCIFFRX_OFFSET ), 0u);
    genericIO_16bitWrite(( iBaseAddress + SCIFFCT_OFFSET ), 0u);
    genericIO_16bitWrite(( iBaseAddress + SCIPRI_OFFSET ), 0u);
}


static void IO_16bitWrite_Impl(const uint32_t address, const uint16_t data)
{
    volatile uint16_t *p;

    p  = (uint16_t*)address;        //lint !e511 !e923 !e9078
    *p = data;
}

/// Defining instance of the global function pointer genericIO_16bitWrite.
/// The pointer is initialised to point to IO_16bitWrite_Impl.
//lint -e{956} External variable.  Pointer doesn't change so it's fine.
void (*genericIO_16bitWrite)(const uint32_t address, const uint16_t data) = IO_16bitWrite_Impl;
void RS485_SetTX(void){
    GpioDataRegs.GPASET.bit.GPIO25=1;
}
void RS485_SetRX(void){
    GpioDataRegs.GPACLEAR.bit.GPIO24=1;
}

void SCIB_SendByte(uint8_t b){
    while(ScibRegs.SCICTL2.bit.TXRDY==0);
        ScibRegs.SCITXBUF = b;

}
void SCIB_SendBuffer(const uint8_t *buf, uint16_t len){
    RS485_SetTX();
    uint16_t i;
    for(i=0;i<len;++i){
        SCIB_SendByte(buf[i]);
    }
    while(ScibRegs.SCICTL2.bit.TXEMPTY ==0);
        RS485_SetRX();
}
