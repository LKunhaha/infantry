#include "other_tasks.h"

/* �ڲ�����------------------------------------------------------------------*/
uint32_t test_i = 0;
/* �ڲ�����ԭ������----------------------------------------------------------*/

/* �������岿�� -------------------------------------------------------------*/
void testTask(void const * argument)
{
	int16_t angle[4];	
	osDelay(200);//��ʱ200ms
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
//		  int16_t  *ptr = angle; //��ʼ��ָ��
//			angle[0] = 0;
//			angle[1] = 0;
//			angle[2] = 0;
//			angle[3] = 0;
//			/*������ʾ��������������*/
//			vcan_sendware((uint8_t *)ptr,4 * sizeof(angle[0]));
//		  vcan_sendware((uint8_t *)ptr,4 * sizeof(angle[1]));
//		  vcan_sendware((uint8_t *)ptr,4 * sizeof(angle[2]));
//		  vcan_sendware((uint8_t *)ptr,4 * sizeof(angle[3]));
		
		
		  osDelayUntil(&xLastWakeTime,100);
	}
}
