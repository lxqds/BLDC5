//
// Created by 27825 on 2023/6/11.
//
#include "interrupt.h"
#include "foc.h"
#include "retarget.h"
#include "tim.h"
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
        if(Delay1ms==100000)
        {
            printf("hello");
            HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_0);
            Delay1ms=0;
        }

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