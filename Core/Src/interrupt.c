//
// Created by 27825 on 2023/6/11.
//
#include "interrupt.h"
#include "foc.h"
#include "retarget.h"
#include "tim.h"
#include "AS5047.h"
extern float target;


void interrupt_Init()
{
    HAL_TIM_Base_Start_IT(&htim2);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htimx)
{
    if(htimx ->Instance == TIM2)
    {
        static uint32_t Delay1ms=0;
        Delay1ms++;
        if(Delay1ms==1000)
        {
            printf("hello");
            HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_0);
            Delay1ms=0;
        }

        //angleOpenloop(3.14);
        //velocityOpenloop(10.f);
        /*if(++interval_count % 1000 == 0)
        {
            printf("Angle: %.2f", Get_Angle2());
        }*/

    }

    /*if(htimx ->Instance == TIM3)
    {
        static uint16_t Delay1ms=0;
        Delay1ms++;
        if(Delay1ms==500)
        {
            printf("hello");
            HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_0);
            Delay1ms=0;
        }
        velocityOpenloop(10);
    }*/
}