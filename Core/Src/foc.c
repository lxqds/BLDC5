//
// Created by 27825 on 2023/6/11.
//
#include "foc.h"
#include <math.h>
#include "stm32g4xx_hal.h"
#include "tim.h"
#define pi 3.1415926
#define _constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
float voltage_limit = 10;
float voltage_power_supply = 12.6;
float shaft_angle = 0,open_loop_timestamp = 0;
float zero_electric_angle = 0,Ualpha,Ubera=0,Ua=0,Ub=0,Uc=0,dc_a=0,dc_b=0,dc_c=0;

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
float _electricalAngle(float shaft_angle,int pole_pairs)
{
    return (shaft_angle * pole_pairs);
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

void setPhaseVoltage(float Uq,float Ud,float angle_el)
{
    angle_el = _normallizeAngle(angle_el+zero_electric_angle);
    //帕克变换
    Ualpha = -Uq*sin(angle_el);
    Ubera = Uq*cos(angle_el);

    //克拉克变换
    Ua = Ualpha +voltage_power_supply/2;
    Ub = (sqrt(3)*Ubera-Ualpha)/2 + voltage_power_supply/2;
    Uc = (-Ualpha- sqrt(3)*Ubera)/2 +voltage_power_supply/2;
    setPwm(Ua,Ub,Uc);
}


//1ms
float velocityOpenloop(float target_velocity)
{


    float Ts =  1e-3f;//1ms

    shaft_angle = _normallizeAngle(shaft_angle + target_velocity*Ts);//计算所需转动的机械角度，

    float Uq = voltage_limit;

    setPhaseVoltage(Uq,0, _electricalAngle(shaft_angle,7));

    return Uq;
}



