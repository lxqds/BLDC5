//
// Created by 27825 on 2023/6/11.
//
#include "foc.h"
#include <math.h>
#include "stm32g4xx_hal.h"
#include "tim.h"
#define pi 3.1415926
#define _constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
#define pole_pairs 6
float voltage_limit = 4;
float voltage_power_supply = 12;
float shaft_angle = 0;
unsigned long open_loop_timestamp;
float zero_electric_angle = 0,Ualpha,Ubera=0,Ua=0,Ub=0,Uc=0,dc_a=0,dc_b=0,dc_c=0;

float velocity_limit = 10;


void foc_Init()
{
    HAL_GPIO_WritePin(GPIOA,GPIO_PIN_11,SET);
    HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_3);

    __HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_1,0);
    __HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_2,0);
    __HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_3,0);


}
//电角度求解
float _electricalAngle(float shaft_angle,int _pole_pairs)
{
    return (shaft_angle * _pole_pairs);
}

//归一化角度到（0，2pi）
float  _normallizeAngle(float angle)
{
    float a = fmod(angle,2*pi);//取余归一化
    return a>= 0?a:(a+2*pi);//保证为正
}
//设置PWM输出
void setPwm(float Ua,float Ub,float Uc)
{
    Ua = _constrain(Ua,0.0f,voltage_limit);
    Ub = _constrain(Ub,0.0f,voltage_limit);
    Uc = _constrain(Uc,0.0f,voltage_limit);

    dc_a = _constrain(Ua/voltage_power_supply,0.0f,1.0f);
    dc_b = _constrain(Ub/voltage_power_supply,0.0f,1.0f);
    dc_c = _constrain(Uc/voltage_power_supply,0.0f,1.0f);

    __HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_1,dc_a*1000);
    __HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_2,dc_b*1000);
    __HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_3,dc_c*1000);
}
//从Uq，Ud电角度到输出PWM
void setPhaseVoltage(float Uq,float Ud,float angle_el)
{
    angle_el = _normallizeAngle(angle_el+zero_electric_angle);
    //帕克逆变换
    Ualpha = -Uq*sin(angle_el);
    Ubera = Uq*cos(angle_el);

    //克拉克逆变换
    Ua = Ualpha +voltage_power_supply/2;
    Ub = (sqrt(3)*Ubera-Ualpha)/2 + voltage_power_supply/2;
    Uc = (-Ualpha- sqrt(3)*Ubera)/2 +voltage_power_supply/2;
    setPwm(Ua,Ub,Uc);
}

/*数学公式*/
/***************************************************************************/
// int array instead of float array
// 4x200 points per 360 deg
// 2x storage save (int 2Byte float 4 Byte )
// sin*10000
const int sine_array[200] = {0,79,158,237,316,395,473,552,631,710,789,867,946,1024,1103,1181,1260,1338,1416,1494,1572,1650,1728,1806,1883,1961,2038,2115,2192,2269,2346,2423,2499,2575,2652,2728,2804,2879,2955,3030,3105,3180,3255,3329,3404,3478,3552,3625,3699,3772,3845,3918,3990,4063,4135,4206,4278,4349,4420,4491,4561,4631,4701,4770,4840,4909,4977,5046,5113,5181,5249,5316,5382,5449,5515,5580,5646,5711,5775,5839,5903,5967,6030,6093,6155,6217,6279,6340,6401,6461,6521,6581,6640,6699,6758,6815,6873,6930,6987,7043,7099,7154,7209,7264,7318,7371,7424,7477,7529,7581,7632,7683,7733,7783,7832,7881,7930,7977,8025,8072,8118,8164,8209,8254,8298,8342,8385,8428,8470,8512,8553,8594,8634,8673,8712,8751,8789,8826,8863,8899,8935,8970,9005,9039,9072,9105,9138,9169,9201,9231,9261,9291,9320,9348,9376,9403,9429,9455,9481,9506,9530,9554,9577,9599,9621,9642,9663,9683,9702,9721,9739,9757,9774,9790,9806,9821,9836,9850,9863,9876,9888,9899,9910,9920,9930,9939,9947,9955,9962,9969,9975,9980,9985,9989,9992,9995,9997,9999,10000,10000};

/***************************************************************************/
// function approximating the sine calculation by using fixed size array
// ~40us (float array)
// ~50us (int array)
// precision +-0.005
// it has to receive an angle in between 0 and 2PI
float _sin2(float a){
    if(a < _PI_2){
        //return sine_array[(int)(199.0*( a / (_PI/2.0)))];
        //return sine_array[(int)(126.6873* a)];           // float array optimized
        return 0.0001*sine_array[_round(126.6873* a)];      // int array optimized
    }else if(a < _PI){
        // return sine_array[(int)(199.0*(1.0 - (a-_PI/2.0) / (_PI/2.0)))];
        //return sine_array[398 - (int)(126.6873*a)];          // float array optimized
        return 0.0001*sine_array[398 - _round(126.6873*a)];     // int array optimized
    }else if(a < _3PI_2){
        // return -sine_array[(int)(199.0*((a - _PI) / (_PI/2.0)))];
        //return -sine_array[-398 + (int)(126.6873*a)];           // float array optimized
        return -0.0001*sine_array[-398 + _round(126.6873*a)];      // int array optimized
    } else {
        // return -sine_array[(int)(199.0*(1.0 - (a - 3*_PI/2) / (_PI/2.0)))];
        //return -sine_array[796 - (int)(126.6873*a)];           // float array optimized
        return -0.0001*sine_array[796 - _round(126.6873*a)];      // int array optimized
    }
}
/***************************************************************************/
// function approximating cosine calculation by using fixed size array
// ~55us (float array)
// ~56us (int array)
// precision +-0.005
// it has to receive an angle in between 0 and 2PI
float _cos2(float a){
    float a_sin = a + _PI_2;
    a_sin = a_sin > _2PI ? a_sin - _2PI : a_sin;
    return _sin2(a_sin);
}
/***************************************************************************/
// normalizing radian angle to [0,2PI]
float _normalizeAngle2(float angle){
    float a = fmod(angle, _2PI);
    return a >= 0 ? a : (a + _2PI);
}
/***************************************************************************/
// Electrical angle calculation
float _electricalAngle2(float shaft_angle, int _pole_pairs) {
    return (shaft_angle * _pole_pairs);
}
/***************************************************************************/
// square root approximation function using
// https://reprap.org/forum/read.php?147,219210
// https://en.wikipedia.org/wiki/Fast_inverse_square_root
float _sqrtApprox2(float number) {//low in fat
    long i;
    float y;
    // float x;
    // const float f = 1.5F; // better precision

    // x = number * 0.5F;
    y = number;
    i = * ( long * ) &y;
    i = 0x5f375a86 - ( i >> 1 );
    y = * ( float * ) &i;
    // y = y * ( f - ( x * y * y ) ); // better precision
    return number * y;
}
/***************************************************************************/
void setPhaseVoltage2(float Uq, float Ud, float angle_el) {
    float Uout;
    uint32_t sector;
    float T0, T1, T2;
    float Ta, Tb, Tc;

    if (Ud) // only if Ud and Uq set
    {// _sqrt is an approx of sqrt (3-4% error)
        Uout = _sqrt2(Ud * Ud + Uq * Uq) / voltage_power_supply;
        // angle normalisation in between 0 and 2pi
        // only necessary if using _sin and _cos - approximation functions
        angle_el = _normalizeAngle2(angle_el + atan2(Uq, Ud));
    } else {// only Uq available - no need for atan2 and sqrt
        Uout = Uq / voltage_power_supply;
        // angle normalisation in between 0 and 2pi
        // only necessary if using _sin and _cos - approximation functions
        angle_el = _normalizeAngle2(angle_el + _PI_2);
    }
    if (Uout > 0.577)Uout = 0.577;
    if (Uout < -0.577)Uout = -0.577;

    sector = (angle_el / _PI_3) + 1;
    T1 = _SQRT3 * _sin2(sector * _PI_3 - angle_el) * Uout;
    T2 = _SQRT3 * _sin2(angle_el - (sector - 1.0) * _PI_3) * Uout;
    T0 = 1 - T1 - T2;

    // calculate the duty cycles(times)
    switch (sector) {
        case 1:
            Ta = T1 + T2 + T0 / 2;
            Tb = T2 + T0 / 2;
            Tc = T0 / 2;
            break;
        case 2:
            Ta = T1 + T0 / 2;
            Tb = T1 + T2 + T0 / 2;
            Tc = T0 / 2;
            break;
        case 3:
            Ta = T0 / 2;
            Tb = T1 + T2 + T0 / 2;
            Tc = T2 + T0 / 2;
            break;
        case 4:
            Ta = T0 / 2;
            Tb = T1 + T0 / 2;
            Tc = T1 + T2 + T0 / 2;
            break;
        case 5:
            Ta = T2 + T0 / 2;
            Tb = T0 / 2;
            Tc = T1 + T2 + T0 / 2;
            break;
        case 6:
            Ta = T1 + T2 + T0 / 2;
            Tb = T0 / 2;
            Tc = T1 + T0 / 2;
            break;
        default:  // possible error state
            Ta = 0;
            Tb = 0;
            Tc = 0;
    }

    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, Ta * 1000);
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, Tb * 1000);
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, Tc * 1000);
}


//1ms
float velocityOpenloop(float target_velocity)
{
    unsigned long now_us;
    float Ts =  100*(1e-6f);//10us

    now_us = SysTick->VAL; //_micros();
    if (now_us < open_loop_timestamp)Ts = (float) (open_loop_timestamp - now_us) / 150 * 1e-6;//150分频
    else
        Ts = (float) (0xFFFFFF - now_us + open_loop_timestamp) / 150 * 1e-6;//150分频
    open_loop_timestamp = now_us;  //save timestamp for next call

    shaft_angle = _normallizeAngle(shaft_angle + target_velocity*Ts);//计算所需转动的机械角度，

    if (Ts == 0 || Ts > 0.5) Ts = 1e-3;

    float Uq = voltage_limit;

    setPhaseVoltage2(Uq,0, _electricalAngle(shaft_angle,pole_pairs));

    return Uq;
}

/******************************************************************************/
float angleOpenloop(float target_angle) {
    unsigned long now_us;
    float Ts, Uq;

    now_us = SysTick->VAL; //_micros();
    if (now_us < open_loop_timestamp)Ts = (float) (open_loop_timestamp - now_us) / 150 * 1e-6;
    else
        Ts = (float) (0xFFFFFF - now_us + open_loop_timestamp) / 150 * 1e-6;
    open_loop_timestamp = now_us;  //save timestamp for next call
    // quick fix for strange cases (micros overflow)
    if (Ts == 0 || Ts > 0.5) Ts = 1e-3;

    // calculate the necessary angle to move from current position towards target angle
    // with maximal velocity (velocity_limit)
    if (fabs(target_angle - shaft_angle) > velocity_limit * Ts) {
        shaft_angle += _sign(target_angle - shaft_angle) * velocity_limit * Ts;
        //shaft_velocity = velocity_limit;
    } else {
        shaft_angle = target_angle;
        //shaft_velocity = 0;
    }

    Uq = voltage_limit;
    // set the maximal allowed voltage (voltage_limit) with the necessary angle
    setPhaseVoltage(Uq, 0, _electricalAngle(shaft_angle, pole_pairs));

    return Uq;
}
/******************************************************************************/

