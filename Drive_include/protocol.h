/*
 * protocol.h
 *
 *  Created on: 2025年4月28日
 *      Author: HP
 */

#ifndef DRIVE_INCLUDE_PROTOCOL_H_
#define DRIVE_INCLUDE_PROTOCOL_H_
#include"RS485.h"
#define FRAME_HEAD 0xAA
#define FRAME_TAIL 0x55
#define PROTO_CC   0xCC
#define ADDR_MOTOR 0x20
#define ADDR_VALUE 0x60
//----电机命令
#define CMD_UPLOAD       0x2000
#define CMD_MOTOR_START  0x2100
#define CMD_MOTOR_STOP   0x2200
#define CMD_MOTOR_FWD    0x2300
#define CMD_MOTOR_REV    0x2400
#define CMD_SET_SPEED    0x2A00
#define CMD_SET_OC_LIMIT 0x2B00
#define CMD_SET_ZERO_I   0x2700
#define CMD_SET_RUN_TRUN 0x2C00
#define CMD_GO_HOME      0x2E00
#define CMD_SET_MAX_TRUN 0x2F00
#define CMD_UNLOCK_BRAKE 0x8000
#define CMD_LOCK_BRAKE   0x8100
#define CMD_PUMP_MANNUAL 0x8200
#define CMD_PUMP_AUTO    0x8300
//---------阀控
#define CMD_VALUE_CTRL   0x0000

#define MAX_DATA_LEN      16

typedef struct{
    uint8_t    head;//0xAA
    uint8_t    addr;//0x20/0x60
    uint8_t    ctrl;//控制字节
    uint8_t    len;
    uint8_t    cmd;
    uint8_t    data[MAX_DATA_LEN];
    uint8_t    crc;//求和
    uint8_t    tail;//0x55
}Frame;
//
void Protocol_Send(uint8_t addr,uint8_t ctrl,uint8_t cmd,
                   const uint8_t *data,uint8_t dlen);
void Protocol_ProcessRx(uint8_t ch);

void SendMotorStatus(void);

#endif /* DRIVE_INCLUDE_PROTOCOL_H_ */
