#include "other_tasks.h"
#include "cmsis_os.h"
/* 内部变量------------------------------------------------------------------*/
uint32_t test_i = 0;
/* 内部函数原型声明----------------------------------------------------------*/

/* 任务主体部分 -------------------------------------------------------------*/
void testTask(void const * argument)
{
	int16_t angle[4];	
	osDelay(200);//延时200ms
	portTickType xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();
//		float pSinVal;
//	float pCosVal;
	for(;;)
	{ 				
		RefreshTaskOutLineTime(testTask_ON);
//		arm_sin_cos_f32(35, &pSinVal, &pCosVal);
    test_i++;
		int16_t  *ptr = angle; //初始化指针
		angle[0] = yaw_get.angle;
		angle[1] = (imu_data.gx);
		angle[2] = yaw_get.total_angle;
		angle[3] = 0;
		/*用虚拟示波器，发送数据*/
		vcan_sendware((uint8_t *)ptr,4 * sizeof(angle[0]));
		vcan_sendware((uint8_t *)ptr,4 * sizeof(angle[1]));
		vcan_sendware((uint8_t *)ptr,4 * sizeof(angle[2]));
		vcan_sendware((uint8_t *)ptr,4 * sizeof(angle[3]));
		
		
		  osDelayUntil(&xLastWakeTime,20);
	}
}
