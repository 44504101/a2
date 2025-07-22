/*
 * offset.h
 *
 *  Created on: 2024Äê10ÔÂ8ÈÕ
 *      Author: HP
 */

#ifndef MOTORCONTROL_INCLUDE_OFFSET_H_
#define MOTORCONTROL_INCLUDE_OFFSET_H_
#include <stdint.h>
typedef double          float32_t;      // Note use of double to avoid Lint warnings.
typedef long double     float64_t;      //lint !e586 long is deprecated but have to use it here.
typedef struct
{
    uint32_t    RunningCounter;                         ///< Counter for offset calculations.
    uint32_t    RunningCounterLimit;                    ///< Value to count up to.
    uint64_t    PhaseARunningTotal;                     ///< Running total for phase A offset.
    uint64_t    PhaseBRunningTotal;                     ///< Running total for phase B offset.
    uint64_t    IDCRunningTotal;
}MotorOffsetCurent_t;


#define MOTOR_OFFSET_DEAFAULT {0u, 240000u, 0u, 0u, 0u}







#endif /* MOTORCONTROL_INCLUDE_OFFSET_H_ */
