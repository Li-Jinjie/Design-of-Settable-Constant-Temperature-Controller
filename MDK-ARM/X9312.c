#include "stm32f1xx_hal.h"
#include "main.h"
#include "stdio.h"
extern int R_now;

void init_x9312(void)
{
	int i;
	i=100;
	HAL_GPIO_WritePin(CS_pwm_GPIO_Port,CS_pwm_Pin,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(UD_pwm_GPIO_Port,UD_pwm_Pin,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(INC_pwm_GPIO_Port,INC_pwm_Pin,GPIO_PIN_SET);
	while(i)
	{
		HAL_GPIO_TogglePin(INC_pwm_GPIO_Port,INC_pwm_Pin);
		HAL_Delay(1);
		HAL_GPIO_TogglePin(INC_pwm_GPIO_Port,INC_pwm_Pin);
		i--;
	}
}	

void set_PWM_R(int set)
{
	double i=0;
	if(R_now>set)
	{
	HAL_GPIO_WritePin(UD_pwm_GPIO_Port,UD_pwm_Pin,GPIO_PIN_RESET);
	i=-1;	
	}
	else if(R_now<set)
	{
	HAL_GPIO_WritePin(UD_pwm_GPIO_Port,UD_pwm_Pin,GPIO_PIN_SET);
	i=1;
	}
	while(R_now!=set)
	{
		HAL_GPIO_TogglePin(INC_pwm_GPIO_Port,INC_pwm_Pin);
		HAL_GPIO_TogglePin(INC_pwm_GPIO_Port,INC_pwm_Pin);
		R_now=R_now+i;
	}
}
