#include "stm32f1xx_hal.h"
#include "ds18b20.h"


void delay_us(uint32_t value)
{
uint32_t i;
i = value * 15;
while(i--);
}


void delay_ms(__IO uint32_t Delay)
{ 
	HAL_Delay(Delay);
}

//??DS18B20
void DS18B20_Rst(void)     
{                 
CLR_DS18B20();    //??DQ
delay_us(750);    //??750us
SET_DS18B20();;   //DQ=1 
delay_us(15);     //15US
}



uint8_t DS18B20_Check(void)        
{   
uint8_t retry=0;

while (DS18B20_DQ_IN&&retry<200)
{
    retry++;
    delay_us(1);
};   
if(retry>=200)return 1;
else retry=0;
while (!DS18B20_DQ_IN&&retry<240)
{
    retry++;
    delay_us(1);
};
if(retry>=240)return 1;     
return 0;
}



uint8_t DS18B20_Read_Bit(void)           // read one bit
{
uint8_t data;

CLR_DS18B20();   //??DQ 
delay_us(2);
SET_DS18B20();;  //DQ=1 
delay_us(12);
if(DS18B20_DQ_IN)data=1;
else data=0;   
delay_us(50);  

return data;
}


//?ds18b20??????
//???:??????
uint8_t DS18B20_Read_Byte(void)    // read one byte
{        
uint8_t i,j,dat;
dat=0;

for (i=1;i<=8;i++) 
{
    j=DS18B20_Read_Bit();
    dat=(j<<7)|(dat>>1);
}                           
return dat;
}

//*???? www.stm32cube.com ??
//??????DS18B20
//dat:??????
void DS18B20_Write_Byte(uint8_t dat)     
{             
uint8_t j;
uint8_t testb;

for (j=1;j<=8;j++) 
{
    testb=dat&0x01;
    dat=dat>>1;
    if (testb) 
    {
        CLR_DS18B20(); //DS18B20_DQ_OUT=0;// Write 1
        delay_us(2);                            
        SET_DS18B20(); //DS18B20_DQ_OUT=1;
        delay_us(60);             
    }
    else 
    {
        CLR_DS18B20(); //DS18B20_DQ_OUT=0;// Write 0
        delay_us(60);             
        SET_DS18B20(); //DS18B20_DQ_OUT=1;
        delay_us(2);                          
    }
}
}


//??????
void DS18B20_Start(void)// ds1820 start convert
{                                          
DS18B20_Rst();     
  DS18B20_Check();   
DS18B20_Write_Byte(0xcc);// skip rom
DS18B20_Write_Byte(0x44);// convert
} 


//???DS18B20?IO?DQ????DS???
//??1:???
//??0:??         
uint8_t DS18B20_Init(void)
{
//??PA2?    
GPIO_InitTypeDef GPIO_InitStruct;

/* GPIO Ports Clock Enable */
//__GPIOF_CLK_ENABLE();  //don't know why

/*Configure GPIO pin : PF0 */
GPIO_InitStruct.Pin = DS18B20_BIT;
GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
GPIO_InitStruct.Pull = GPIO_NOPULL;
GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
HAL_GPIO_Init(DS18B20_PORT, &GPIO_InitStruct);

SET_DS18B20();         //?PA2?????

DS18B20_Rst();
return DS18B20_Check();
} 


//?ds18b20?????
//??:0.1C
//???:??? (-550~1250) 
float DS18B20_Get_Temp(void)
{
    uint8_t temp;
    uint8_t TL,TH;
		int16_t tem;
    DS18B20_Start ();                    // ds1820 start convert
    DS18B20_Rst();
    DS18B20_Check();	 
    DS18B20_Write_Byte(0xcc);// skip rom
    DS18B20_Write_Byte(0xbe);// convert	    
    TL=DS18B20_Read_Byte(); // LSB   
    TH=DS18B20_Read_Byte(); // MSB  
	    	  
    if(TH>7)
    {
        TH=~TH;
        TL=~TL; 
        temp=0;//????  
    }else temp=1;//????	  	  
    tem=TH; //?????
    tem<<=8;    
    tem+=TL;//?????
    tem=(float)tem*0.625;//??     
	if(temp)return tem/10.00; //?????
	else return -tem/10.00;    
}

