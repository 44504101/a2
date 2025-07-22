/*
 * volt_calc.h
 *
 *  Created on: 2024Äê9ÔÂ24ÈÕ
 *      Author: HP
 */

#ifndef MOTORCONTROL_INCLUDE_VOLT_CALC_H_
#define MOTORCONTROL_INCLUDE_VOLT_CALC_H_

typedef struct{
	              _iq DcBusVolt;
	              _iq MfuncV1;
	              _iq MfuncV2;
	              _iq MfuncV3;
	              Uint16 OutOfPhase;
	              _iq VphaseA;
	              _iq VphaseB;
	              _iq VphaseC;
	              _iq Valpha;
	              _iq Vbeta;
	              _iq temp;
}PHASEVOLTAGE;

#define PHASEVOLTAGE_DEAFAULTS { 0, \
	                           0,\
							   0,\
							   0,\
							   1,\
							   0,\
							   0,\
							   0,\
							   0,\
							   0,\
							   0,\
}


#define ONE_THIRD  _IQ(0.33333333333333)
#define TWO_THIRD  _IQ(0.66666666666667)
#define INV_SQRT3  _IQ(0.57735026918963)

#define PHASEVOLTAGE_MACRO(v)                                                    \
	v.temp = _IQmpy(v.DcBusVolt,ONE_THIRD);                                      \
	v.VphaseA = _IQmpy(v.temp,(_IQmpy2(v.MfuncV1)-v.MfuncV2-v.MfuncV3));         \
	v.VphaseB = _IQmpy(v.temp,(_IQmpy2(v.MfuncV2)-v.MfuncV1-v.MfuncV3));         \
	if(v.OutOfPhase == 0)                                                        \
{                                                                                \
		v.VphaseA = -v.VphaseA;                                                  \
		v.VphaseB = -v.VphaseB;                                                  \
}                                                                                \
	v.Valpha = v.VphaseA;                                                        \
	v.Vbeta = _IQmpy((v.VphaseA + _IQmpy2(v.VphaseB)),INV_SQRT3);                 \


#endif /* MOTORCONTROL_INCLUDE_VOLT_CALC_H_ */
