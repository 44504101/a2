/*
 * rampgen.h
 *
 *  Created on: 2024Äê9ÔÂ12ÈÕ
 *      Author: HP
 */

#ifndef MOTORCONTROL_INCLUDE_RAMPGEN_H_
#define MOTORCONTROL_INCLUDE_RAMPGEN_H_

#include "IQmathLib.h"

typedef struct{
	            _iq Freq;
	            _iq StepAngleMax;
	            _iq Angle;
	            _iq Gain;
	            _iq Out;
	            _iq Offset;
}RAMPGEN;

#define RAMPGEN_DEAFAULT {0,		\
						  0,		\
						  0,		\
						  _IQ(1),	\
						  0,		\
						  _IQ(1), 	\
                         }

/*------------------------------------------------------------------------------
	RAMP(Sawtooh) Generator Macro Definition
------------------------------------------------------------------------------*/

#define RG_MACRO(v)									\
													\
/* Compute the angle rate */						\
	v.Angle += _IQmpy(v.StepAngleMax,v.Freq);		\
													\
/* Saturate the angle rate within (-1,1) */			\
	if (v.Angle>_IQ(1.0))							\
		v.Angle -= _IQ(1.0);						\
	else if (v.Angle<_IQ(-1.0))						\
		v.Angle += _IQ(1.0);						\
		v.Out=v.Angle;

#endif /* MOTORCONTROL_INCLUDE_RAMPGEN_H_ */
