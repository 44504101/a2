/*
 * RS485.c
 *
 *  Created on: 2025��4��22��
 *      Author: HP
 */

#include"RS485.h"
#include "Drive_include.h"
static void         ResetAllSCIRegisters(const uint32_t iBaseAddress);
static uint32_t     SetupSCIBaseAddress(const ESCIModule_t module);
typedef struct
{
    /* ---------- ���ղ� ---------- */
    uint8_t  *p_rxBuffer;           // ָ����ջ�����
    uint16_t  rxOffset;             // ��ǰ��д�뻺�������ֽ���
    uint16_t  rxMaxLength;          // ���ջ�������󳤶�

    pTriggerTimerFunction p_timerTrigger; // ָ��ι��/��ʱ��ʱ������������
    bool_t    b_matchRequired;      // �Ƿ���Ҫ���ַ�ƥ��
    uint8_t   matchCharacter;       // Ҫƥ����ַ����� '\n'��
    uint16_t  matchCounter;         // ��ƥ�䵽���ַ�����
    void     *p_receiveSemaphore;   // ��������ͬ���Ľ����ź���

    /* ---------- ���Ͳ� ---------- */
    const uint8_t *p_txBuffer;      // ָ�����������
    uint16_t       txOffset;        // �ѷ����ֽ���
    uint16_t       txMessageLength; // ������Ϣ����
    void          *p_transmitSemaphore; // ��������ź���
} serialPortVars_t;

static serialPortVars_t m_serialPorts[SCI_NUMBER_OF_PORTS];

void SCI_Open(const ESCIModule_t module){
    uint32_t    baseAddress;
    uint16_t    requiredData;

    if (module < SCI_NUMBER_OF_PORTS)
        {
            baseAddress = SetupSCIBaseAddress(module); // ����SCI�Ļ���ַ
            ResetAllSCIRegisters(baseAddress);//��ʼ��SCI���еļĴ�����ȫд0
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
                        ( (uint16_t)0u << 7 ) |     // 0: 1λֹͣλ
                        ( (uint16_t)0u << 6 ) |     // 0: ��У��
                        ( (uint16_t)0u << 5 ) |     // 0: ��żУ��ر�
                        ( (uint16_t)0u << 4 ) |     // 0: �Բ���ģʽ����
                        ( (uint16_t)0u << 3 ) |     // 0: ������ģʽЭ��
                        ( (uint16_t)7u << 0 );      // 111: 8λ���� (bits 2:0)
                    genericIO_16bitWrite(( baseAddress + SCICCR_OFFSET ), requiredData); //lint !e835    //SCICCR register��ʾcommunication control register

            requiredData =
                        ( (uint16_t)0u << 7 ) |     // 0: ��ȡΪ0��д����Ч
                        ( (uint16_t)1u << 6 ) |     // 1: ���ܴ����ж�ʹ��
                        ( (uint16_t)1u << 5 ) |     // 1: ����ʹ��SCI
                        ( (uint16_t)0u << 4 ) |     // 0: ��ȡΪ0��д����Ч
                        ( (uint16_t)0u << 3 ) |     // 0: ��������δѡ��
                        ( (uint16_t)0u << 2 ) |     // 0: ����ģʽ����
                        ( (uint16_t)1u << 1 ) |     // 1: ������ʹ��
                        ( (uint16_t)1u << 0 );      // 1: ���ܵ��������͵� RXREG
                   genericIO_16bitWrite(( baseAddress + SCICTL1_OFFSET ), requiredData);

              requiredData =
                           ( (uint16_t)0u << 7 ) |     // 0: read only bit, writes have no effect
                           ( (uint16_t)0u << 6 ) |     // 0: read only bit, writes have no effect
                           ( (uint16_t)0u << 5 ) |     // 0:
                           ( (uint16_t)0u << 4 ) |     // 0: ����λ
                           ( (uint16_t)0u << 3 ) |     // 0:
                           ( (uint16_t)0u << 2 ) |     // 0:
                           ( (uint16_t)1u << 1 ) |     // 1: RXRDY\BRKDT �ж�ʹ��
                           ( (uint16_t)0u << 0 );      // 1: TXRDY �ж�ʹ��
                     genericIO_16bitWrite(( baseAddress + SCICTL2_OFFSET ), requiredData);
                     requiredData =
                                 ( (uint16_t)1u << 15 ) |    // 1: SCI FIFO �������·��ͺͽ���
                                 ( (uint16_t)1u << 14 ) |    // 1: SCI FIFO ʹ��
                                 ( (uint16_t)1u << 13 ) |    // 1: �ٴ�ʹ�ܷ���FIFO����
                                 ( (uint16_t)0u << 12 ) |    // 0: read only bit, writes have no effect
                                 ( (uint16_t)0u << 11 ) |    // 0: read only bit, writes have no effect
                                 ( (uint16_t)0u << 10 ) |    // 0: read only bit, writes have no effect
                                 ( (uint16_t)0u << 9 ) |     // 0: read only bit, writes have no effect
                                 ( (uint16_t)0u << 8 ) |     // 0: read only bit, writes have no effect
                                 ( (uint16_t)0u << 7 ) |     // 0: read only bit, writes have no effect
                                 ( (uint16_t)1u << 6 ) |     // 1: д��1�����7λ�ķ��� FIFO�ж�(TXFFINT)��־
                                 ( (uint16_t)0u << 5 ) |     // 0: TX FIFO interrupt based on match disabled
                                 ( (uint16_t)0u << 0 );      // 00000: TX FIFO interrupt depth = 0 (bits 4:0)
                      genericIO_16bitWrite(( baseAddress + SCIFFTX_OFFSET ), requiredData); // SCIFFTX��ʾSCI FIFO Transmit register

                      requiredData =
                          ( (uint16_t)0u << 15 ) |    // 0: read only bit, writes have no effect
                          ( (uint16_t)1u << 14 ) |    // 1: д1����� 15 λ�Ľ��� FIFO ���(RXFFOVF)��־
                          ( (uint16_t)1u << 13 ) |    // 1: SCI FIFO receive pointer enabled
                          ( (uint16_t)0u << 12 ) |    // 0: read only bit, writes have no effect
                          ( (uint16_t)0u << 11 ) |    // 0: read only bit, writes have no effect
                          ( (uint16_t)0u << 10 ) |    // 0: read only bit, writes have no effect
                          ( (uint16_t)0u << 9 ) |     // 0: read only bit, writes have no effect
                          ( (uint16_t)0u << 8 ) |     // 0: read only bit, writes have no effect
                          ( (uint16_t)0u << 7 ) |     // 0: read only bit, writes have no effect
                          ( (uint16_t)1u << 6 ) |     // 1: д1�����7λ�� RXFFINT ��־
                          ( (uint16_t)1u << 5 ) |     // 1: ʹ�ܻ��� RXFFIVLƥ��(С�ڻ���ڵĽ��� FIFO�ж�
                          ( (uint16_t)1u << 0 );      // 00001: RX FIFO interrupt depth = 1 (bits 4:0)
                      genericIO_16bitWrite(( baseAddress + SCIFFRX_OFFSET ), requiredData);  // SCIFFRX��ʾSCI FIFO Receive register

                      requiredData =
                          ( (uint16_t)0u << 15 ) |    // 0: read only bit, writes have no effect
                          ( (uint16_t)1u << 14 ) |    // 1: д1����� 15λ ABD־
                          ( (uint16_t)0u << 13 ) |    // 0: �����Զ������ʵ���
                          ( (uint16_t)0u << 12 ) |    // 0: reserved, writes have no effect
                          ( (uint16_t)0u << 11 ) |    // 0: reserved, writes have no effect
                          ( (uint16_t)0u << 10 ) |    // 0: reserved, writes have no effect
                          ( (uint16_t)0u << 9 ) |     // 0: reserved, writes have no effect
                          ( (uint16_t)0u << 8 ) |     // 0: reserved, writes have no effect
                          ( (uint16_t)0u << 0 );      // 00000000: FIFO transfer delay = 0 (bits 7:0)
                      genericIO_16bitWrite(( baseAddress + SCIFFCT_OFFSET ), requiredData); // SCIFFCT��ʾSCI FIFO control register

                      requiredData =
                          ( (uint16_t)0u << 7 ) |     // 0: reserved bit, writes have no effect
                          ( (uint16_t)0u << 6 ) |     // 0: reserved bit, writes have no effect
                          ( (uint16_t)0u << 5 ) |     // 0: reserved bit, writes have no effect
                          ( (uint16_t)0u << 3 ) |     // 00: immediate stop on suspend (bits 4:3)
                          ( (uint16_t)0u << 2 ) |     // 0: reserved bit, writes have no effect
                          ( (uint16_t)0u << 1 ) |     // 0: reserved bit, writes have no effect
                          ( (uint16_t)0u << 0 );      // 0: reserved bit, writes have no effect
                      genericIO_16bitWrite(( baseAddress + SCIPRI_OFFSET ), requiredData); // SCIPRI��ʾSCI Priority control register
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
