//############################################################
// FILE:  PI_Cale.h
// Created on: 2017年12月10日
// Author: XQ
// summary: Header file  and definition
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//版权所有，盗版必究
//DSP28335旋变PMSM电机控制与开发板
//硕历电子
//网址: https://shuolidianzi.taobao.com
//Author-QQ: 616264123
//电机控制QQ群：736369775
//############################################################

#ifndef  PI_Cale_H
#define  PI_Cale_H

#include "DSP2833x_Project.h"
#include"IQmathLib.h"
typedef struct {
	_iq  Ref;       // PI控制的给定参数
	    _iq  Fbk;       // PI控制的反馈参数
	    _iq  Out;       // PI控制的输出参数
	    _iq  Kp;        // 当前比例增益
	    _iq  Ki;        // 当前积分增益
	    _iq  Kp_base;   // 基础比例增益
	    _iq  Ki_base;   // 基础积分增益
	    _iq  Kp_max;    // 比例增益最大值
	    _iq  Kp_min;    // 比例增益最小值
	    _iq  Ki_max;    // 积分增益最大值
	    _iq  Ki_min;    // 积分增益最小值
	    _iq  Kaw;       // 抗积分饱和增益
	    _iq  SatErr;    // 饱和误差
	    _iq  Umax;      // 输出最大限幅
	    _iq  Umin;      // 输出最小限幅
	    _iq  up;        // 比例项计算结果
	    _iq  ui;        // 积分项计算结果
	    _iq  v1;        // 控制器输出前的值
	    _iq  i1;        // 积分器的状态（历史积分值）
	    _iq  w1;        // 饱和记录（u(k-1) - v(k-1)）
	    _iq  up_term;   // 比例项输出
	    _iq  e_prev;    // 前一次误差（用于计算误差变化率）
       } PI_Control;

#define PI_CONTROLLER_DEFAULTS {		\
		    _IQ(0.0),      /* Ref       */ \
		    _IQ(0.0),      /* Fbk       */ \
		    _IQ(0.0),      /* Out       */ \
		    _IQ(1.0),      /* Kp        */ \
		    _IQ(0.0),      /* Ki        */ \
		    _IQ(1.0),      /* Kp_base   */ \
		    _IQ(0.0),      /* Ki_base   */ \
		    _IQ(5.0),      /* Kp_max    */ \
		    _IQ(0.0),      /* Kp_min    */ \
		    _IQ(1.0),      /* Ki_max    */ \
		    _IQ(0.0),      /* Ki_min    */ \
		    _IQ(0.0),      /* Kaw       */ \
		    _IQ(0.0),      /* SatErr    */ \
		    _IQ(1.0),      /* Umax      */ \
		    _IQ(-1.0),     /* Umin      */ \
		    _IQ(0.0),      /* up        */ \
		    _IQ(0.0),      /* ui        */ \
		    _IQ(0.0),      /* v1        */ \
		    _IQ(0.0),      /* i1        */ \
		    _IQ(0.0),      /* w1        */ \
		    _IQ(0.0),      /* up_term   */ \
		    _IQ(0.0)       /* e_prev    */ \
              			  }

extern  PI_Control   pi_spd ;
extern  PI_Control   pi_id ;
extern  PI_Control   pi_iq ;
extern  PI_Control   pi_FW ;

//void  PI_Controller(p_PI_Control  pV);  //PI控制算法函数
void  PI_Pare_init(void );    //PI控制参数初始化

/*#define PI_MACRO(v)                                                      \
		v.up = v.Ref - v.Fbk;                                          \
		v.ui = (v.Out == v.v1)?(_IQmpy(v.Ki, v.up)+ v.i1) : v.i1;	\

		v.i1 = v.ui;												\
		v.v1 = _IQmpy(v.Kp, (v.up + v.ui));							\
		v.Out= _IQsat(v.v1, v.Umax, v.Umin);						\*/
static void FuzzyAdaptivePI(PI_Control *v)
{
    // 计算误差和误差变化率
    _iq e = v->Ref - v->Fbk;          // 当前误差
    _iq de = e - v->e_prev;           // 误差变化率
    v->e_prev = e;                    // 更新前一次误差

    // 模糊化过程（根据实际情况设计隶属度函数）
    // 这里提供一个简化的示例

    int e_level = 0;   // 误差等级
    int de_level = 0;  // 误差变化率等级

    // 定义误差等级（示例）
    if(e > _IQ(0.1)) e_level = 2;
    else if(e > _IQ(0.05)) e_level = 1;
    else if(e > _IQ(-0.05)) e_level = 0;
    else if(e > _IQ(-0.1)) e_level = -1;
    else e_level = -2;

    // 定义误差变化率等级（示例）
    if(de > _IQ(0.1)) de_level = 2;
    else if(de > _IQ(0.05)) de_level = 1;
    else if(de > _IQ(-0.05)) de_level = 0;
    else if(de > _IQ(-0.1)) de_level = -1;
    else de_level = -2;

    // 模糊规则表（需要根据实际系统设计）
    _iq delta_Kp = _IQ(0.0);
    _iq delta_Ki = _IQ(0.0);

    // 简化的模糊规则示例
    if(e_level == 2)
    {
        delta_Kp = _IQ(0.1);
        delta_Ki = _IQ(-0.01);
    }
    else if(e_level == 1)
    {
        delta_Kp = _IQ(0.05);
        delta_Ki = _IQ(0.0);
    }
    else if(e_level == 0)
    {
        delta_Kp = _IQ(0.0);
        delta_Ki = _IQ(0.0);
    }
    else if(e_level == -1)
    {
        delta_Kp = _IQ(-0.05);
        delta_Ki = _IQ(0.0);
    }
    else // e_level == -2
    {
        delta_Kp = _IQ(-0.1);
        delta_Ki = _IQ(-0.01);
    }

    // 调整控制器增益
    v->Kp = v->Kp_base + delta_Kp;
    v->Ki = v->Ki_base + delta_Ki;

    // 增益限幅
    if(v->Kp > v->Kp_max) v->Kp = v->Kp_max;
    if(v->Kp < v->Kp_min) v->Kp = v->Kp_min;
    if(v->Ki > v->Ki_max) v->Ki = v->Ki_max;
    if(v->Ki < v->Ki_min) v->Ki = v->Ki_min;
}
#define PI_MACRO(v)                                                          \
/* 调用模糊自适应函数 */                                                 \
    FuzzyAdaptivePI(&v);                                                     \
    /* 计算误差 */                                                           \
    v.up = v.Ref - v.Fbk;                                                    \
    /* 计算比例输出 */                                                       \
    v.up_term = _IQmpy(v.Kp, v.up);                                          \
    /* 更新积分项，加入反积分机制 */                                          \
    v.SatErr = v.Out - v.v1;                                                 \
    v.ui = v.ui + _IQmpy(v.Ki, v.up) + _IQmpy(v.Kaw, v.SatErr);              \
    /* 存储积分器的状态 */                                                   \
    v.i1 = v.ui;                                                             \
    /* 计算控制器输出前的值 */                                                \
    v.v1 = v.up_term + v.ui;                                                 \
    /* 对输出进行限幅 */                                                     \
    v.Out = _IQsat(v.v1, v.Umax, v.Umin);


#endif /* PI_Cale*/
