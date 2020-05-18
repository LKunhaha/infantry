#include "other_tasks.h"
#include "cmsis_os.h"
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
//		float pSinVal;
//	float pCosVal;
	for(;;)
	{ 				
		RefreshTaskOutLineTime(testTask_ON);
//		arm_sin_cos_f32(35, &pSinVal, &pCosVal);
//    test_i++;
//		int16_t  *ptr = angle; //��ʼ��ָ��
//		angle[0] = yaw_get.angle;
//		angle[1] = (imu_data.gx);
//		angle[2] = yaw_get.total_angle;
//		angle[3] = 0;
//		/*������ʾ��������������*/
//		vcan_sendware((uint8_t *)ptr,4 * sizeof(angle[0]));
//		vcan_sendware((uint8_t *)ptr,4 * sizeof(angle[1]));
//		vcan_sendware((uint8_t *)ptr,4 * sizeof(angle[2]));
//		vcan_sendware((uint8_t *)ptr,4 * sizeof(angle[3]));
		 HAL_GPIO_WritePin(LED_R_GPIO_Port,LED_R_Pin,GPIO_PIN_SET);	
		 HAL_GPIO_WritePin(LED_G_GPIO_Port,LED_G_Pin,GPIO_PIN_SET);	
		 HAL_GPIO_WritePin(LED_B_GPIO_Port,LED_B_Pin,GPIO_PIN_SET);	
		
		  osDelayUntil(&xLastWakeTime,20);
	}
}
