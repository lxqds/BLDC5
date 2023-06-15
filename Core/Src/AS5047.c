#include "AS5047.h"
#include "retarget.h"

uint16_t AS5047P_ADD[3] = {0x3FFF,0x01,0x00};
static uint16_t Raw_angle = 0;
static float angle_data_prev; //上次位置
static float full_rotation_offset; //转过的整圈数
unsigned long velocity_calc_timestamp;
float angle_prev;


/**SPI1 GPIO Configuration
    PB15     ------> SPI1_SCK
    PB14     ------> SPI1_MISO
    PB13     ------> SPI1_MOSI
    PB11     ------> SPI1_CS
    */
#define SPI_SCK_PIN                     GPIO_PIN_15
#define SPI_SCK_GPIO_PORT               GPIOB
#define SPI_MOSI_PIN                    GPIO_PIN_13
#define SPI_MOSI_GPIO_PORT              GPIOB
#define SPI_MISO_PIN                    GPIO_PIN_14
#define SPI_MISO_GPIO_PORT              GPIOB
#define SPI_NSS_PIN                     GPIO_PIN_11
#define SPI_NSS_GPIO_PORT               GPIOB


#define SPI_SCK_GPIO_CLK_ENABLE()       __HAL_RCC_GPIOB_CLK_ENABLE()
#define SPI_MISO_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOB_CLK_ENABLE()
#define SPI_MOSI_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOB_CLK_ENABLE()
#define SPI_NSS_GPIO_CLK_ENABLE()       __HAL_RCC_GPIOB_CLK_ENABLE()

#define MOSI_H  HAL_GPIO_WritePin(SPI_MOSI_GPIO_PORT, SPI_MOSI_PIN, GPIO_PIN_SET)
#define MOSI_L  HAL_GPIO_WritePin(SPI_MOSI_GPIO_PORT, SPI_MOSI_PIN, GPIO_PIN_RESET)
#define SCK_H   HAL_GPIO_WritePin(SPI_SCK_GPIO_PORT, SPI_SCK_PIN, GPIO_PIN_SET)
#define SCK_L   HAL_GPIO_WritePin(SPI_SCK_GPIO_PORT, SPI_SCK_PIN, GPIO_PIN_RESET)
#define MISO    HAL_GPIO_ReadPin(SPI_MISO_GPIO_PORT, SPI_MISO_PIN)
#define NSS_H   HAL_GPIO_WritePin(SPI_NSS_GPIO_PORT, SPI_NSS_PIN, GPIO_PIN_SET)
#define NSS_L   HAL_GPIO_WritePin(SPI_NSS_GPIO_PORT, SPI_NSS_PIN, GPIO_PIN_RESET)


/*
for循环实现延时us
*/
void for_delay_us(uint32_t nus)
{
    uint32_t Delay = nus * 150;//时钟频率
    do
    {
        __NOP();
    }
    while (Delay --);
}
void SPI_Init(void)
{
    /*##-1- Enable peripherals and GPIO Clocks #########################*/
    /* Enable GPIO TX/RX clock */
    SPI_SCK_GPIO_CLK_ENABLE();
    SPI_MISO_GPIO_CLK_ENABLE();
    SPI_MOSI_GPIO_CLK_ENABLE();
    SPI_NSS_GPIO_CLK_ENABLE();


    /*##-2- Configure peripheral GPIO #######################*/
    /* SPI SCK GPIO pin configuration  */
    GPIO_InitTypeDef GPIO_InitStruct;

    GPIO_InitStruct.Pin       = SPI_SCK_PIN;
    GPIO_InitStruct.Mode      = GPIO_MODE_OUTPUT_PP;
    //GPIO_InitStruct.Pull      = GPIO_PULLDOWN;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(SPI_SCK_GPIO_PORT, &GPIO_InitStruct);
    HAL_GPIO_WritePin(SPI_SCK_GPIO_PORT, SPI_SCK_PIN, GPIO_PIN_SET);

    /* SPI MISO GPIO pin configuration  */
    GPIO_InitStruct.Pin = SPI_MISO_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    HAL_GPIO_Init(SPI_MISO_GPIO_PORT, &GPIO_InitStruct);

    /* SPI MOSI GPIO pin configuration  */
    GPIO_InitStruct.Pin = SPI_MOSI_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    HAL_GPIO_Init(SPI_MOSI_GPIO_PORT, &GPIO_InitStruct);
    HAL_GPIO_WritePin(SPI_MOSI_GPIO_PORT, SPI_MOSI_PIN, GPIO_PIN_SET);

    GPIO_InitStruct.Pin = SPI_NSS_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    HAL_GPIO_Init(SPI_NSS_GPIO_PORT, &GPIO_InitStruct);
    HAL_GPIO_WritePin(SPI_NSS_GPIO_PORT, SPI_NSS_PIN, GPIO_PIN_SET);
}


/* CPOL = 0, CPHA = 0, MSB first */
uint8_t SOFT_SPI_RW_MODE0( uint8_t write_dat )
{
    uint8_t i, read_dat;
    for( i = 0; i < 8; i++ )
    {
        if( write_dat & 0x80 )
            MOSI_H;
        else
            MOSI_L;
        write_dat <<= 1;
        for_delay_us(1);
        SCK_H;
        read_dat <<= 1;
        if( MISO )
            read_dat++;
        for_delay_us(1);
        SCK_L;
        __NOP();
    }

    return read_dat;
}


/* CPOL=0，CPHA=1, MSB first */
uint8_t SOFT_SPI_RW_MODE1(uint8_t byte)
{
    uint8_t i,Temp=0;

    for(i=0;i<8;i++)     // 循环8次
    {
        SCK_H;     //拉高时钟
        if(byte&0x80)
        {
            MOSI_H;  //若最到位为高，则输出高
        }
        else
        {
            MOSI_L;   //若最到位为低，则输出低
        }
        byte <<= 1;     // 低一位移位到最高位
        for_delay_us(1);
        SCK_L;     //拉低时钟
        Temp <<= 1;     //数据左移

        if(MISO)
            Temp++;     //若从从机接收到高电平，数据自加一
        for_delay_us(1);

    }
    return (Temp);     //返回数据
}

/* CPOL=1，CPHA=0, MSB first */
uint8_t SOFT_SPI_RW_MODE2(uint8_t byte)
{
    uint8_t i,Temp=0;

    for(i=0;i<8;i++)     // 循环8次
    {
        if(byte&0x80)
        {
            MOSI_H;  //若最到位为高，则输出高
        }
        else
        {
            MOSI_L;   //若最到位为低，则输出低
        }
        byte <<= 1;     // 低一位移位到最高位
        for_delay_us(1);
        SCK_L;     //拉低时钟
        Temp <<= 1;     //数据左移

        if(MISO)
            Temp++;     //若从从机接收到高电平，数据自加一
        for_delay_us(1);
        SCK_H;     //拉高时钟

    }
    return (Temp);     //返回数据
}


/* CPOL = 1, CPHA = 1, MSB first */
uint8_t SOFT_SPI_RW_MODE3( uint8_t write_dat )
{
    uint8_t i, read_dat;
    for( i = 0; i < 8; i++ )
    {
        SCK_L;
        if( write_dat & 0x80 )
            MOSI_H;
        else
            MOSI_L;
        write_dat <<= 1;
        for_delay_us(1);
        SCK_H;
        read_dat <<= 1;
        if( MISO )
            read_dat++;
        for_delay_us(1);
        __NOP();
    }
    return read_dat;
}

uint16_t SPIx_ReadWrite_Byte(uint16_t WriteByte,uint16_t ReadByte)
{
    uint8_t T_high,T_low,R_high,R_low;
    T_high = (WriteByte >> 8) & 0xff; //高8位
    T_low = WriteByte & 0xff; //低8位

    R_high = SOFT_SPI_RW_MODE1(T_high);
    R_low = SOFT_SPI_RW_MODE1(T_low);

    ReadByte = (uint16_t) (R_high << 8) | R_low; //uint16_t是无符号16位
    return ReadByte;
}
uint16_t Read_Register(uint16_t addr)
{
    uint16_t back,temp_back;

    AS5047_CS_L;
    temp_back =  SPIx_ReadWrite_Byte(addr,temp_back);

    AS5047_CS_H;
    HAL_Delay(1);
    AS5047_CS_L;
    back = SPIx_ReadWrite_Byte(0,back);

    AS5047_CS_H;
    HAL_Delay(1);
    return back;
}
uint16_t Write_Register(uint16_t addr,uint16_t data)
{
    uint16_t back,temp_back;

    AS5047_CS_L;
    SPIx_ReadWrite_Byte(addr,temp_back);
    AS5047_CS_H;
    HAL_Delay(10);
    AS5047_CS_L;
    SPIx_ReadWrite_Byte(data,back);
    AS5047_CS_H;
}

float Get_Angle2()
{
    uint16_t a,c;
    float b;
    a=Read_Register(0x3FFC)&0x0fff;     //AGC=>bit11:MAGL,bit10:MAGH,bit7-0:AGC   0x01AD
    b=((float)(Read_Register(0x3FFF)&0x3fff)*360)/16384;   //angle
    c=(uint16_t)b;
   //printf("AGC=%X,Angle=%d\r\n",a,c);
    return b;
}

float getVelocity(void)
{
    unsigned long now_us;
    float Ts, angle_c, vel;

    // calculate sample time
    now_us = SysTick->VAL; //_micros();
    if(now_us<velocity_calc_timestamp)Ts = (float)(velocity_calc_timestamp - now_us)/150*1e-6;
    else
        Ts = (float)(0xFFFFFF - now_us + velocity_calc_timestamp)/150*1e-6;
    // quick fix for strange cases (micros overflow)
    if(Ts == 0 || Ts > 0.5) Ts = 1e-3;

    // current angle
    angle_c = Get_Angle2();
    // velocity calculation
    vel = (angle_c - angle_prev)/Ts;

    // save variables for future pass
    angle_prev = angle_c;
    velocity_calc_timestamp = now_us;
    return vel;
}
void Sensor_lnit(void)
{
    SPI_Init();
    Get_Angle2();
}

