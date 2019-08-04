/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"
#include "adc.h"
#include "dac.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include "math.h"
#include "stdio.h"
#include "stdlib.h"
#include "ds18b20.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
	uint8_t aTxStartMessage[] = "\r\n****Connected to STM32 successfully :) ****\r\nPlease set temperature by your phone :\r\n";
	double_t tempr_set;
	double_t tempr_now; //set temperature
	double_t tempr_18b20; //ds18b20
	double_t lightening_number;  //used for choosing value to light
	double count;  //used for light 3min
	#define BUFFERSIZE 10
	uint8_t ReceiveBuff[BUFFERSIZE];
	uint8_t SendBuff[BUFFERSIZE];  //for send message to phone
	
	uint8_t recv_end_flag = 0,Rx_len;
	/* Buffer used for reception */
	uint8_t SettemMessage1[]="\r\n***You have set temperature =  ";
	uint8_t SettemMessage2[]="  ℃	****\r\n";
	
	uint8_t StateMessage1[]="\r\n***Temperature now =   ";
	uint8_t StateMessage2[]="  ℃	****\r\n";
	
	uint8_t ds18b20message[]="DS18B20 Check Failed!\r\n";
//	uint8_t aRxBuffer[10];
	int tag;
	int temp;
	int dac_number;
	
	/*for change x9312*/
	int R_set;  //0-100
	int R_now;  //0-100
	
	/* for ADC*/
	uint32_t ADC_PID[50];
	uint8_t i;
	double ad_pid;
	
	/*for pt100 ADC*/
	uint32_t ADC_PT100[100];
	double ad_pt100_low,ad_pt100_high;
	double pt100;
	
	
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_NVIC_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void uartdamget(void);
#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif 
PUTCHAR_PROTOTYPE
{
        HAL_UART_Transmit(&huart1 , (uint8_t *)&ch, 1, 0xFFFF);
return ch;
}

void ecvt (double_t,uint8_t* );  //used for transfer number to char

void* memset(void*,int,unsigned int);

void Light(uint16_t);
void light_number( double_t n);
void set_dac_value (double_t t,uint32_t channel); //for transfer temperature to dac value and start dac


void init_x9312(void); //for x9312
void set_PWM_R(int set); //change X9312
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */
	tempr_set=75.0; //design for light number
	
	tempr_now=60; //used for show temperature now
	
	count=0;
	lightening_number=tempr_set;  //initial
	
	tag=0;	//design for twlinking
	temp=0;	//design for choose weishu 
	
	R_set=0;
	R_now=0;
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  MX_DAC_Init();
  MX_ADC1_Init();
  MX_ADC3_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();

  /* USER CODE BEGIN 2 */
		HAL_Delay(100);
	
		__HAL_UART_ENABLE_IT(&huart1,UART_IT_IDLE);  //使能串口1idle中断
		
		HAL_UART_Transmit_IT(&huart1, (uint8_t *)aTxStartMessage, sizeof(aTxStartMessage));
		HAL_UART_Receive_DMA(&huart1,(uint8_t*)ReceiveBuff,BUFFERSIZE);
	
		HAL_TIM_Base_Start_IT(&htim2);
		
		HAL_GPIO_WritePin(IO5_GPIO_Port,IO5_Pin,GPIO_PIN_SET);

		set_dac_value (tempr_set,DAC_CHANNEL_2);
		
		HAL_ADC_Start_DMA(&hadc1,(uint32_t*)&ADC_PID,50); //for adc
		HAL_ADC_Start_DMA(&hadc3,(uint32_t*)&ADC_PT100,100); //for pt100
		
		HAL_GPIO_WritePin(Cold_control_GPIO_Port,Cold_control_Pin,GPIO_PIN_RESET); //for cold
		
		/*ds18b20*/
		while(DS18B20_Init()) 
		{
			HAL_UART_Transmit(&huart1,(uint8_t*)ds18b20message,sizeof(ds18b20message),100);
			HAL_Delay (500);
		}
		/*x9312*/
		init_x9312();
		
		tempr_set=75;
		HAL_Delay(200);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
		light_number(lightening_number);
		uartdamget();
		 if(tempr_now<tempr_set)
		{
		set_dac_value (tempr_now,DAC_CHANNEL_1);
		set_PWM_R(R_set+55);
		}
		else 
		{
		set_dac_value (tempr_set,DAC_CHANNEL_1);
			set_PWM_R(0);
		}
		
				
//		set_PWM_R(R_set);
  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV4;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/** NVIC Configuration
*/
static void MX_NVIC_Init(void)
{
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);
  /* USART1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(USART1_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(USART1_IRQn);
  /* DMA1_Channel4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);
  /* TIM2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM2_IRQn);
}

/* USER CODE BEGIN 4 */
void set_dac_value (double_t t,uint32_t channel)
{
	if(t<=110&&t>=40)
	{
	dac_number=(int)(((t-40)*4095)/70);
	HAL_DAC_SetValue(&hdac, channel,DAC_ALIGN_12B_R,dac_number);
	HAL_DAC_Start(&hdac,channel);
	}
	else if(t<=40)
	{
	dac_number=(int)(0);
	HAL_DAC_SetValue(&hdac, channel,DAC_ALIGN_12B_R,dac_number);
	HAL_DAC_Start(&hdac,channel);
	}
	else if(t>=110)
	{
	dac_number=(int)(4095);
	HAL_DAC_SetValue(&hdac, channel,DAC_ALIGN_12B_R,dac_number);
	HAL_DAC_Start(&hdac,channel);
	}
}

void light_number( double_t n)
{
int a,b,c,d;
	n=n*10;
	a=(int)n/1000;
	n=n-a*1000;
	b=(int)n/100;
	n=n-b*100;
	c=(int)n/10;
	d=(int)n-c*10;
	if(tag!=1)
	{
		HAL_GPIO_WritePin(IO6_GPIO_Port ,IO6_Pin,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(IO7_GPIO_Port ,IO7_Pin,GPIO_PIN_RESET);
		Light(a);
	}
	if(tag!=2)
	{
		HAL_GPIO_WritePin(IO6_GPIO_Port ,IO6_Pin,GPIO_PIN_SET);
		HAL_GPIO_WritePin(IO7_GPIO_Port ,IO7_Pin,GPIO_PIN_RESET);
		Light(b);
	}
	if(tag!=3)
	{
		HAL_GPIO_WritePin(IO6_GPIO_Port ,IO6_Pin,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(IO7_GPIO_Port ,IO7_Pin,GPIO_PIN_SET);
		Light(c);
	}
	
	if(tag!=4)
	{
		HAL_GPIO_WritePin(IO6_GPIO_Port ,IO6_Pin,GPIO_PIN_SET);
		HAL_GPIO_WritePin(IO7_GPIO_Port ,IO7_Pin,GPIO_PIN_SET);
		Light(d);
	}
}

void uartdamget(void)
	{
		if(recv_end_flag==1)
		{
			HAL_UART_Transmit(&huart1,(uint8_t*)SettemMessage1,sizeof(SettemMessage1),100);
			tempr_set=atof((char*)ReceiveBuff);  //changed
			
//			if(tempr_now<tempr_set)
		{
			set_dac_value(tempr_set,DAC_CHANNEL_2);//for dac setting voltage
		}
		
			ecvt(tempr_set,(uint8_t*)SendBuff);
			HAL_UART_Transmit(&huart1,(uint8_t*)SendBuff,sizeof(SendBuff),100);
			
			HAL_UART_Transmit(&huart1,(uint8_t*)SettemMessage2,sizeof(SettemMessage2),100);
			memset(SendBuff,0,8);
			
			count=6;  //used for light 3min
			
			memset(ReceiveBuff,0,BUFFERSIZE);
			
		recv_end_flag=0;
		HAL_UART_Receive_DMA(&huart1,(uint8_t*)ReceiveBuff,BUFFERSIZE);

		}
	}
	/*按键部分*/
	void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin==weishu_Pin)
	{
		temp++;
		if(temp>4)
		{
			temp=0;
			count=6;   //used for show set number
		}
	}
	else if(GPIO_Pin==up_Pin&&temp!=0)
	{
		tempr_set=tempr_set+pow(10,3-temp);
	}
	else if(GPIO_Pin==down_Pin&&temp!=0&&tempr_set!=0)
	{
		tempr_set=tempr_set-pow(10,3-temp);
		if(tempr_set<0)
				tempr_set=0;
	}
		/*for setting dac value*/
//		 if(tempr_now<tempr_set)
		{
			set_dac_value(tempr_set,DAC_CHANNEL_2);//for dac setting voltage
		}
}
	void ecvt (double_t a,uint8_t*b )  //used for transfer number to ch
{
	double_t g;
	int c,d,e,f,h;
	g=a*1000;
	c=g/100000;
	g=g-c*100000;
	d=g/10000;
	g=g-d*10000;
	e=g/1000;
	g=g-1000*e;
	f=g/100;
	g=g-100*f;
	h=g/10;
	g=g-h*10;
	b[0]=c+'0';
	b[1]=d+'0';
	b[2]=e+'0';
	b[3]='.';
	b[4]=f+'0';
	b[5]=' ';
	b[6]=' ';
//	b[7]=' ';
//	b[8]=' ';
//	b[9]=0;
	
		if(b[0]=='0')
		{
			b[0]=0;
			if(b[1]=='0')
				b[1]=0;
		}
}

void HAL_TIM_PeriodElapsedCallback (TIM_HandleTypeDef* htim)
		{
			if(htim->Instance==htim2.Instance)
			{
				/* for getting PID value now by adc */
				for(i=0,ad_pid=0;i<50;)
				{
					ad_pid = ad_pid+ADC_PID[i];
					i++;
				}
				ad_pid = ad_pid/50;
				ad_pid=ad_pid*3.3/4096;
				R_set=ad_pid*100/3.24;
				
				if(R_set>100)
					R_set=100;
				else if(R_set<0)
					R_set=0;

				/*for get pt100 value*/
				for(i=0,ad_pt100_low=0,ad_pt100_high=0;i<100;)
				{
						ad_pt100_high += ADC_PT100[i++];
						ad_pt100_low += ADC_PT100[i++];
				}
				ad_pt100_low = ad_pt100_low/50;
				ad_pt100_low=ad_pt100_low*3.3/4096;
				ad_pt100_high = ad_pt100_high/50;
				ad_pt100_high=ad_pt100_high*3.3/4096;
				
//			ecvt((ad_pt100_high-ad_pt100_low)*100,(uint8_t*)SendBuff);
//			HAL_UART_Transmit(&huart1,(uint8_t*)SendBuff,sizeof(SendBuff),100);
//			memset(SendBuff,0,8); //used for clear SendBuff
//				
//			HAL_UART_Transmit(&huart1,(uint8_t*)SettemMessage2,sizeof(SettemMessage2),100);
//			memset(SendBuff,0,8);
//				
				tempr_now=(ad_pt100_high-ad_pt100_low)*65.128-4.263+1;
//				tempr_now=DS18B20_Get_Temp();
				
				
				/* for blink*/
				if(tag==0)
					tag=temp;
				else
					tag=0;
				/*   used for transfer set temperature*/
			ecvt(tempr_set,(uint8_t*)SendBuff);
			HAL_UART_Transmit(&huart1,(uint8_t*)SendBuff,sizeof(SendBuff),100);
			memset(SendBuff,0,8); //used for clear SendBuff
				
				/*   used for transfer now temperature*/
			ecvt(tempr_now,(uint8_t*)SendBuff);
			HAL_UART_Transmit(&huart1,(uint8_t*)SendBuff,sizeof(SendBuff),100);
			memset(SendBuff,0,8); //used for clear SendBuff
				
				/*  used for showing set value */
				if(temp>0&&temp<5)
				{
					lightening_number=tempr_set;
				}
				else if(count>0)
				{
					count=count-1;
					lightening_number=tempr_set;
				}
				else if(count==0)
				{
					lightening_number=tempr_now;
				}
				
				/*ds18b20*/
				tempr_18b20=DS18B20_Get_Temp();
				/*   used for transfer now temperature*/
				ecvt(tempr_18b20,(uint8_t*)SendBuff);
				HAL_UART_Transmit(&huart1,(uint8_t*)SendBuff,sizeof(SendBuff),100);
				memset(SendBuff,0,8); //used for clear SendBuff
				//				
			HAL_UART_Transmit(&huart1,(uint8_t*)SettemMessage2,sizeof(SettemMessage2),100);
			memset(SendBuff,0,8);
			
			if(tempr_now> (tempr_set+1))
			{
			HAL_GPIO_WritePin(Cold_control_GPIO_Port,Cold_control_Pin,GPIO_PIN_SET);
			set_PWM_R(0);
			}
			else if(tempr_now<tempr_set)
			HAL_GPIO_WritePin(Cold_control_GPIO_Port,Cold_control_Pin,GPIO_PIN_RESET);
		
			}
//			else if(htim->Instance==htim4.Instance)
//			{
////			for(i=0,ad1=0;i<50;i++)
////				{
////					ad1=ad1+ADC_Value[i];
////				}
////				ad1=ad1/50;     //get pid voltage
////				R_set=ad1*100/3.2;
//			}
		}
		

	
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler_Debug */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
