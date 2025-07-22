/*
 * smopos_const.h
 *
 *  Created on: 2024Äê11ÔÂ15ÈÕ
 *      Author: HP
 */

#ifndef MOTORCONTROL_INCLUDE_SMOPOS_CONST_H_
#define MOTORCONTROL_INCLUDE_SMOPOS_CONST_H_
#include<math.h>
typedef struct 	{ float32  Rs; 				// Input: Stator resistance (ohm)
			      float32  Ls;				// Input: Stator inductance (H)
				  float32  Ib; 				// Input: Base phase current (amp)
				  float32  Vb;				// Input: Base phase voltage (volt)
				  float32  Ts;				// Input: Sampling period in sec
			      float32  Fsmopos;			// Output: constant using in observed current calculation
			      float32  Gsmopos;			// Output: constant using in observed current calculation

				} SMOPOS_CONST;

/*-----------------------------------------------------------------------------
Default initalizer for the SMOPOS_CONST object.
-----------------------------------------------------------------------------*/
#define SMOPOS_CONST_DEFAULTS {0,0,0,0,0,0,0, \
                            }

/*------------------------------------------------------------------------------
Prototypes for the functions in SMOPOS_CONST.C
------------------------------------------------------------------------------*/
extern SMOPOS_CONST smo1_const;
#define SMO_CONST_MACRO(v)								\
														\
	v.Fsmopos = exp((-v.Rs/v.Ls)*(v.Ts));				\
	v.Gsmopos = (v.Vb/v.Ib)*(1/v.Rs)*(1-v.Fsmopos);


#endif /* MOTORCONTROL_INCLUDE_SMOPOS_CONST_H_ */
