#ifndef __DS18B20_H
#define __DS18B20_H
#include "stm32f1xx_hal.h"

#define DS18B20_BIT         GPIO_PIN_0

#define DS18B20_PORT        GPIOG

#define CLR_DS18B20()       HAL_GPIO_WritePin (DS18B20_PORT,DS18B20_BIT,GPIO_PIN_RESET)

#define SET_DS18B20()       HAL_GPIO_WritePin (DS18B20_PORT,DS18B20_BIT,GPIO_PIN_SET)

#define DS18B20_DQ_IN   HAL_GPIO_ReadPin(DS18B20_PORT, DS18B20_BIT)
void delay_us(uint32_t value);
void delay_ms(__IO uint32_t Delay);
uint8_t DS18B20_Init(void);          //???DS18B20
float DS18B20_Get_Temp(void);   //????
void DS18B20_Start(void);       //??????
void DS18B20_Write_Byte(uint8_t dat);//??????
uint8_t DS18B20_Read_Byte(void);     //??????
uint8_t DS18B20_Read_Bit(void);      //?????
uint8_t DS18B20_Check(void);         //??????DS18b20
void DS18B20_Rst(void);         //??DS18B20   

#endif
