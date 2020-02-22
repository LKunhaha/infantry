/* ����ͷ�ļ�----------------------------------------------------------------*/
#include "offline_check.h"
#include "SystemState.h"
#include "gpio.h"
#include "Motor_USE_CAN.h"
#include "communication.h "
#include "usart.h"
#include "gimbal_control.h"

/* �ڲ�����------------------------------------------------------------------*/
SystemStateDef Gimbal_Motor;
SystemStateDef JY61;

/* �ڲ�����ԭ������----------------------------------------------------------*/

/* �������岿�� -------------------------------------------------------------*/



void Led_Task(void const * argument)
{
	
	osDelay(100);
	portTickType xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();
	for(;;)
	{
		
		RefreshTaskOutLineTime(LedTask_ON);
		

				if((SystemState.OutLine_Flag & 0x01))//ң��������
				{
						
//					  gimbal_status.remote_mode = 1;
					  printf("remote over");
						osDelayUntil(&xLastWakeTime,200);		  
				}else  
				{
//						gimbal_status.remote_mode=0;
						
				}
				
				if((SystemState.OutLine_Flag & 0x02))
				{
						
					  Gimbal_Motor.Mode = 1;
					  printf("motor_Y over");
						osDelayUntil(&xLastWakeTime,200);
				} else  
				{
						
						Gimbal_Motor.Mode = 0;
				}

				if((SystemState.OutLine_Flag & 0x04))
				{
						
				  	Gimbal_Motor.Mode = 2;
					  printf("motor_P over");
						osDelayUntil(&xLastWakeTime,200);
				} else  
				{
						Gimbal_Motor.Mode = 0;
						
				}
				if((SystemState.OutLine_Flag & 0x08))
				{
						
					  Gimbal_Motor.Mode = 3;
					  printf("motor_B over");
						osDelayUntil(&xLastWakeTime,200);
				} else 
        {	
					Gimbal_Motor.Mode = 0;
					
				}
				if((SystemState.OutLine_Flag & 0x10))
				{
					  JY61.Mode = 1;
					  printf("JY61 over");
						
						osDelayUntil(&xLastWakeTime,200);
					  
				} else  
				{
					JY61.Mode = 0;
					
				}
	  	osDelayUntil(&xLastWakeTime,LED_PERIOD); 
		
	 }
 
}



void Check_Task(void const * argument)
{
	osDelay(100);
	portTickType xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();
	
	for(;;)
	{
	      if((SystemState.task_OutLine_Flag & 0x01))   //����Ӧ��λ�Ƿ���Ϊ1
				{
					printf("testTask GG \n\t");
					osDelayUntil(&xLastWakeTime,100);
				}
				
				if((SystemState.task_OutLine_Flag & 0x02))
				{
						printf("RemoteDataTask GG \n\t");
			      Remote_Disable();
						osDelayUntil(&xLastWakeTime,100);
				} 
				
				if((SystemState.task_OutLine_Flag & 0x04))
				{
						printf("GimbalContrlTask GG \n\t");
					  Cloud_Platform_Motor_Disable(&hcan1);
						osDelayUntil(&xLastWakeTime,100);
				} 
				
				if((SystemState.task_OutLine_Flag & 0x08))
				{
						printf("GunTask GG \n\t");
						osDelayUntil(&xLastWakeTime,100);
				} 
				
				if((SystemState.task_OutLine_Flag & 0x10))
				{
						printf("LedTask GG \n\t");
						osDelayUntil(&xLastWakeTime,100);
				} 


				if((SystemState.task_OutLine_Flag & 0x20))
				{
						printf("vOutLineCheckTask GG \n\t");
						osDelayUntil(&xLastWakeTime,100);
				} 

				osDelayUntil(&xLastWakeTime,Check_PERIOD);
			}
}


//���߼��
void vOutLineCheck_Task(void const *argument)
{

	portTickType xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();
	for(;;)
	{
		RefreshTaskOutLineTime(vOutLineCheckTask_ON);
		
	  limit_check();
		TASK_Check();       //������߼��
		OutLine_Check();    //ģ����߼��
		osDelayUntil(&xLastWakeTime,20);
		
	}
}

