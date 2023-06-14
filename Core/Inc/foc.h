//
// Created by 27825 on 2023/6/11.
//
#ifndef _FOC_H
#define _FOC_H
#include "main.h"

/******************************************************************************/
// sign function
#define _sign(a) ( ( (a) < 0 )  ?  -1   : ( (a) > 0 ) )
#define _round(x) ((x)>=0?(long)((x)+0.5):(long)((x)-0.5))
#define _constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
#define _sqrt2(a) (_sqrtApprox2(a))
#define _isset(a) ( (a) != (NOT_SET) )

// utility defines
#define _2_SQRT3 1.15470053838
#define _SQRT3 1.73205080757
#define _1_SQRT3 0.57735026919
#define _SQRT3_2 0.86602540378
#define _SQRT2 1.41421356237
#define _120_D2R 2.09439510239
#define _PI 3.14159265359
#define _PI_2 1.57079632679
#define _PI_3 1.0471975512
#define _2PI 6.28318530718
#define _3PI_2 4.71238898038
#define _PI_6 0.52359877559
/******************************************************************************/
// dq current structure
typedef struct
{
    float d;
    float q;
} DQCurrent_s;
// phase current structure
typedef struct
{
    float a;
    float b;
    float c;
} PhaseCurrent_s;
// dq voltage structs
typedef struct
{
    float d;
    float q;
} DQVoltage_s;

/******************************************************************************/
/**
 *  Motiron control type
 */
typedef enum
{
    Type_torque,//!< Torque control
    Type_velocity,//!< Velocity motion control
    Type_angle,//!< Position/angle motion control
    Type_velocity_openloop,
    Type_angle_openloop
} MotionControlType;

/**
 *  Motiron control type
 */
typedef enum
{
    Type_voltage, //!< Torque control using voltage
    Type_dc_current, //!< Torque control using DC current (one current magnitude)
    Type_foc_current //!< torque control using dq currents
} TorqueControlType;


float velocityOpenloop(float target_velocity);
float angleOpenloop(float target_angle);
void foc_Init();
#endif //BLDC5_FOC_H
