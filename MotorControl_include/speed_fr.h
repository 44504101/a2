/*
 * speed_fr.h
 *
 *  Created on: 2024Äê10ÔÂ22ÈÕ
 *      Author: HP
 */

#ifndef MOTORCONTROL_INCLUDE_SPEED_FR_H_
#define MOTORCONTROL_INCLUDE_SPEED_FR_H_
#include"IQmathLib.h"
typedef struct{
	           _iq ElecTheta;
	           Uint32 Direction;
	           _iq OldElecTheta;
	           _iq Speed;
	           Uint32 BaseRpm;
	           _iq21 K1;
	           _iq K2;
	           _iq K3;
	           float32_t SpeedRpm;
	           _iq Tmp;
	           _iq AngleDiff;
}SPEED_MEAS;
extern SPEED_MEAS speed1;
#define SPEED_MEAS_DEAFAULTS { 0,\
	                           1,\
							   0,\
							   0,\
							   0,\
							   0,\
							   0,\
							   0,\
							   0,\
							   0,\
}




#define SPEED_FR_MACRO(v)											\
/* Differentiator*/													\
/* Synchronous speed computation   */								\
   if ((v.ElecTheta < _IQ(0.9))&(v.ElecTheta > _IQ(0.1)))			\
/* Q21 = Q21*(GLOBAL_Q-GLOBAL_Q)*/									\
		v.Tmp = _IQmpy(v.K1,(v.ElecTheta - v.OldElecTheta));		\
   else v.Tmp = _IQtoIQ21(v.Speed);									\
/* Low-pass filter*/												\
/* Q21 = GLOBAL_Q*Q21 + GLOBAL_Q*Q21*/								\
   	v.Tmp = _IQmpy(v.K2,_IQtoIQ21(v.Speed))+_IQmpy(v.K3,v.Tmp);		\
/* Saturate the output */											\
	v.Tmp=_IQsat(v.Tmp,_IQ21(1),_IQ21(-1));							\
	v.Speed = _IQ21toIQ(v.Tmp);										\
/* Update the electrical angle */									\
    v.OldElecTheta = v.ElecTheta;									\
/* Change motor speed from pu value to rpm value (GLOBAL_Q -> Q0)*/	\
/* Q0 = Q0*GLOBAL_Q => _IQXmpy(), X = GLOBAL_Q*/					\
    v.SpeedRpm = _IQmpy(v.BaseRpm,v.Speed);

/*
#define SPEED_FR_MACRO(v)                                                     \
     Calculate angle difference                                           \
    v.AngleDiff = v.ElecTheta - v.OldElecTheta;                               \
     Handle angle wrap-around                                             \
    if (v.AngleDiff < _IQ(-0.5)) {                                            \
        v.AngleDiff += _IQ(1.0);                                              \
    } else if (v.AngleDiff > _IQ(0.5)) {                                      \
        v.AngleDiff -= _IQ(1.0);                                              \
    }                                                                         \
     Calculate the speed                                                  \
    v.Tmp = _IQmpy(v.K1, v.AngleDiff);                                        \
     Apply low-pass filter                                                \
    v.Speed = _IQmpy(v.K2, v.Speed) + _IQmpy(v.K3, v.Tmp);                    \
     Limit the speed                                                      \
    v.Speed = _IQsat(v.Speed, _IQ(1.0), _IQ(-1.0));                           \
     Update the old electrical angle                                      \
    v.OldElecTheta = v.ElecTheta;                                             \
     Convert speed from pu to RPM                                         \
    v.SpeedRpm = _IQmpy(v.BaseRpm, v.Speed);
*/
#endif /* MOTORCONTROL_INCLUDE_SPEED_FR_H_ */
