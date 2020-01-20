#include "other_tasks.h"

/* 内部变量------------------------------------------------------------------*/
uint32_t test_i = 0;

int current_adc_get;
float I_value[8];
float I_measure;
float I_SUM;
float V_value;
float Total_power;

extern float total_set;
/* 内部函数原型声明----------------------------------------------------------*/

/* 任务主体部分 -------------------------------------------------------------*/
void testTask(void const * argument)
{
	int16_t angle[4];
  int getbuff = 0;	
	osDelay(200);//延时200ms
	portTickType xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();
	
	for(;;)
	{ 				
		RefreshTaskOutLineTime(testTask_ON);
    test_i++;
		
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOE,GPIO_PIN_14,GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOE,GPIO_PIN_15,GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOF,GPIO_PIN_12,GPIO_PIN_RESET);
		
		uint32_t *buff = uhADCxConvertedValue;
		
		getbuff = 0;
		for(uint8_t i = 0;i < 10;i++)
		{
			getbuff += buff[i];
		}
		V_value =  getbuff * 0.1f * 3.3 / 4096;
		
		I_value[7] = I_value[6];
		I_value[6] = I_value[5];
		I_value[5] = I_value[4];
		I_value[4] = I_value[3];
		I_value[3] = I_value[2];
		I_value[2] = I_value[1];
		I_value[1] = I_value[0];
		I_value[0] = (getbuff * 0.1f * (0.00080566f) - 1.65f) / 0.066f;    //0.066为线性度
		
		I_SUM = 0;
	  for(uint8_t i = 0;i < 8;i++)
		{
			I_SUM += I_value[i];
		}
		I_measure = I_SUM / 8;
		Total_power = I_measure * 24.0 * 1.25;
		  int16_t  *ptr = angle; //初始化指针
			angle[0] = (int16_t)Total_power;
			angle[1] = (int16_t)total_set;
			angle[2] = 80;
			angle[3] = 120;
			/*用虚拟示波器，发送数据*/
			vcan_sendware((uint8_t *)ptr,4 * sizeof(angle[0]));
		  vcan_sendware((uint8_t *)ptr,4 * sizeof(angle[1]));
		  vcan_sendware((uint8_t *)ptr,4 * sizeof(angle[2]));
		  vcan_sendware((uint8_t *)ptr,4 * sizeof(angle[3]));
		
		
		  osDelayUntil(&xLastWakeTime,20);
	}
}
