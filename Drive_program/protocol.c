/*
 * protocol.c
 *
 *  Created on: 2025年4月29日
 *      Author: HP
 */
#include"protocol.h"
#include"stdint.h"
#include "Motorcontrol_include.h"
typedef enum{
    ST_HEAD,
    ST_ADDR,
    ST_CTRL,
    ST_LEN,
    ST_CCB,
    ST_CMD,
    ST_PAYLOAD,
    ST_CRC,
    ST_TAIL
}RxState;
static uint8_t calc_crc(uint8_t ctrl,uint8_t len,
                       const uint8_t *payload)
{
    uint16_t sum = ctrl+len;
    uint8_t i;
    for(i=0; i<len;++i){
        sum+=payload[i];
    }
    return (uint8_t)sum;
}

void Protocol_Send(uint8_t addr,uint8_t ctrl,uint8_t cmd,
                   const uint8_t *data,uint8_t dlen){
    uint8_t buf[7+1+MAX_DATA_LEN];//head....tail
    uint8_t len=2+dlen;
    uint8_t idx=0;
    buf[idx++]=FRAME_HEAD;
    buf[idx++]=addr;
    buf[idx++]=ctrl;
    buf[idx++]=len;
    buf[idx++]=PROTO_CC;
    buf[idx++]=cmd;
    memcpy(&buf[idx],data,dlen);
    idx+=dlen;
    buf[idx++]=calc_crc(ctrl,len,&buf[4]);
    SCIB_SendBuffer(buf,idx);
}

//Rx状态机
static RxState rxState =ST_HEAD;
static uint8_t rxBuf[MAX_DATA_LEN];//cmd+data
static uint8_t rxLen = 0,rxIdx = 0;
static uint8_t rxAddr,rxCtrl,rxCRC,rxCMD;

extern void Motor_Start(void);
extern void Motor_Stop(void);
extern void Motor_SetDir(uint8_t fwd);
extern void Motor_SetSpeed(uint16_t rpm);
extern void SendMotorStatus(void);
extern void Motor_Set_Run_Trun(uint16_t turns);

static void Frame_Dispatch(uint16_t cmd,const uint8_t *dat,uint8_t len)
{
    switch(cmd)
    {
    case CMD_MOTOR_START:
        Motor_Start();
        break;
    case CMD_MOTOR_STOP:
        Motor_Stop();
        break;
    case CMD_SET_SPEED:
        if(len>=2){
            uint16_t rpm = ((uint16_t)dat[0]<<8)|dat[1];
            Motor_SetSpeed(rpm);
        }
        break;
    case CMD_SET_RUN_TRUN:
        if(len>=2){
            uint16_t turns = ((uint16_t)dat[0]<<8)|dat[1];
            Motor_Set_Run_Trun(turns);
        }
        break;
    }
}
void Protocol_ProcessRx(uint8_t ch)
{
    uint8_t calc = 0;
    switch(rxState)
    {
    case ST_HEAD:
      if(ch==FRAME_HEAD)
        rxState = ST_ADDR;
      break;
    case ST_ADDR:
        rxAddr = ch;
      rxState = ST_CTRL;
      break;
    case ST_CTRL:
      rxCtrl = ch;
      rxState = ST_LEN;
      break;
    case ST_LEN:
      rxLen = ch;
      rxIdx = 0;
      rxState = ST_CCB;
      break;
    case ST_CCB:
        if(ch == PROTO_CC)
            rxState = ST_CMD;
        else
            rxState = ST_HEAD;
        break;
    case ST_CMD:
        rxCMD=ch;
        if(rxLen==1)
            rxState=ST_CRC;
        else
            rxState=ST_PAYLOAD;
        break;
    case ST_PAYLOAD:
      if(rxIdx<MAX_DATA_LEN)
          rxBuf[rxIdx++] = ch;
      if(rxIdx >= rxLen-1)
          rxState = ST_CRC;
      break;
    case ST_CRC:
      rxCRC = ch;
      calc = rxCtrl+rxLen+PROTO_CC+rxCMD;
      uint8_t k;
      for(k=0;k<rxIdx;++k)
          calc+=rxBuf[k];
      if(calc == rxCRC)
          Frame_Dispatch(rxCMD,rxBuf,rxIdx);
      rxState = ST_TAIL;
      break;
    case ST_TAIL:
        rxState = ST_HEAD;
        break;
    }
}

void SendMotorStatus(void){
    uint8_t payload[12];
    payload[0]=(uint8_t)(motorSt.BusVolat >>8);
    payload[1]=(uint8_t)(motorSt.BusVolat & 0xFF);

    //payload[2]=(motorSt.Current >>8);
    //payload[3]=(motorSt.Current & 0xFF);

    payload[4]=(uint8_t)(motorSt.SpeedRpm >>8);
    payload[5]=(uint8_t)(motorSt.SpeedRpm & 0xFF);

    payload[6]=(uint8_t)(motorSt.ABsPos_cnt >>8);
    payload[7]=(uint8_t)(motorSt.ABsPos_cnt & 0xFF);

    payload[8]=(uint8_t)(motorSt.NowPos_cnt >>8);
    payload[9]=(uint8_t)(motorSt.NowPos_cnt & 0xFF);

    payload[10]=0x00;
    payload[11]=motorSt.StatusByte;

    Protocol_Send(ADDR_MOTOR,
                  0x00,
                  CMD_UPLOAD,
                  payload,
                  12);
}
