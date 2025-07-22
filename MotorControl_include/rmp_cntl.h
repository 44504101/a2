/*
 * rmp_cntl.h
 *
 *  Created on: 2024Äê9ÔÂ11ÈÕ
 *      Author: HP
 */

#ifndef MOTORCONTROL_INCLUDE_RMP_CNTL_H_
#define MOTORCONTROL_INCLUDE_RMP_CNTL_H_

#include"IQmathLib.h"
typedef struct{
	            _iq TargetValue;
	            Uint32 RampDelayMax;
	            _iq RampLowLimit;
	            _iq RampHighLimit;
	            Uint32 RampDelayCount;
	            _iq SetPointValue;
	            Uint32 EqualFlag;
	            _iq Tmp;
}RMPCNTL;

#define RMPCNTL_DEAFAULT {  0,\
	                        5,\
							_IQ(-1),\
							_IQ(1),\
							0,\
							0,\
							0,\
							0,\
}


#define RC_MACRO(v)                                                                     \
        v.Tmp = v.TargetValue - v.SetPointValue;                                        \
     if(_IQabs(v.Tmp)>=_IQ(0.000030518)){                                               \
    	 v.RampDelayCount++;                                                            \
    	 if(v.RampDelayCount >= v.RampDelayMax){                                        \
    		 if(v.SetPointValue <= v.TargetValue ){                                      \
    			 v.SetPointValue += _IQ(0.000030518);                                   \
    		 }                                                                          \
    		 else v.SetPointValue -= _IQ(0.000030518);                                  \
    		 v.SetPointValue = _IQsat(v.SetPointValue, v.RampHighLimit, v.RampLowLimit); \
    		 v.RampDelayCount = 0;                                                       \
    	 }                                                                               \
     }                                                                                   \
     else v.EqualFlag = 0x7FFFFFFF;
#endif /* MOTORCONTROL_INCLUDE_RMP_CNTL_H_ */
