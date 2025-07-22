//############################################################
// FILE:  Axis_transform.h
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

#ifndef Axis_transform_H
#define Axis_transform_H

#include "DSP2833x_Project.h"
#include"offset.h"

typedef struct {
            _iq  As;        //  三相电流A
            _iq  Bs;          //  三相电流B
            _iq  Cs;          //  三相电流C
            _iq Alpha;       //  二相静止坐标系 Alpha 轴
            _iq  Beta;
            _iq abc;
            _iq bbc;
            } CLARKE ,*p_CLARKE ;

#define  CLARKE_DEFAULTS {0,0,0,0,0,0,0}  // 初始化参数
extern CLARKE clark1;
typedef struct {
            _iq Alpha;     //  二相静止坐标系 Alpha 轴
            _iq  Beta;        //  二相静止坐标系 Beta 轴
            _iq   Angle;       //  电机磁极位置角度0---65536即是0---360度
            _iq  Ds;          //  电机二相旋转坐标系下的d轴电流
            _iq  Qs;          //  电机二相旋转坐标系下的q轴电流
            _iq  Sine;    //  正弦参数，-32768---32767  -1到1
            _iq  Cosine;  //  余弦参数，-32768---32767  -1到1
            } PARK , *p_PARK ;

#define  PARK_DEFAULTS {0,0,0,0,0,0,0}  // 初始化参数


typedef struct {
            _iq  Alpha;         // 二相静止坐标系 Alpha 轴
            _iq   Beta;          // 二相静止坐标系 Beta 轴
            _iq   Angle;         // 电机磁极位置角度0---65536即是0---360度
            _iq   Ds;            //  电机二相旋转坐标系下的d轴电流
            _iq   Qs;            //  电机二相旋转坐标系下的q轴电流
            _iq   Sine;        //  正弦参数，-32768---32767  -1到1
            _iq   Cosine;        //  余弦参数，-32768---32767  -1到1
            }IPARK , *p_IPARK;

#define  IPARK_DEFAULTS {0,0,0,0,0,0,0}  // 初始化参数


void  CLARKE_Cale(p_CLARKE  pV); // 三相到二相变换 克拉克变换
void  PARK_Cale(p_PARK pV) ;   // 二相到二相变换 怕克变换
#define IPARK_MACRO(v)                    \
		v.Alpha = _IQmpy(v.Ds,v.Cosine) - _IQmpy(v.Qs,v.Sine);		\
		v.Beta  = _IQmpy(v.Qs,v.Cosine) + _IQmpy(v.Ds,v.Sine);

#define CLARKE_MACRO(v)											\
																\
v.Alpha = v.As;													\
v.Beta = _IQmpy((v.As +_IQmpy2(v.Bs)),_IQ(0.57735026918963));	\


#define PARK_MACRO(v)                                           \
		v.Ds = _IQmpy(v.Alpha,v.Cosine) + _IQmpy(v.Beta,v.Sine);	\
	    v.Qs = _IQmpy(v.Beta,v.Cosine) - _IQmpy(v.Alpha,v.Sine);




#endif /* Axis_transform*/
//===========================================================================
// End of file.
//===========================================================================
