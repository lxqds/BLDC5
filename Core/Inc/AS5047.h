#ifndef __AS5047_H
#define __AS5047_H

#include "main.h"
/*#include "spi.h"*/

#define abs(x) ((x)>0?(x):-(x))//绝对值函数

#define AS5047_CS_L HAL_GPIO_WritePin(GPIOB,GPIO_PIN_11,GPIO_PIN_RESET)
#define AS5047_CS_H HAL_GPIO_WritePin(GPIOB,GPIO_PIN_11,GPIO_PIN_SET)

#define AS5047P_CPR      16384      //14bit
#define _2PI 6.28318530718f

uint16_t AS5047_Get_Raw_Angle(void);      //直接读取寄存器的未经处理
float Get_Angle(void);
void Sensor_lnit(void);
void Get_Angle2();
#endif
