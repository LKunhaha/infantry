/* ����ͷ�ļ�----------------------------------------------------------------*/
#include "data_processing.h"
#include "SystemState.h"
#include "user_lib.h"
#include "main.h"
#include "AX-12A.h"

/* �ڲ�����------------------------------------------------------------------*/
pid_t pid_minipc_yaw = {0};
pid_t pid_minipc_pit = {0};

uint8_t Reset_Mocha = 0;
/* �ڲ�����ԭ������-----------------------------------------------------------*/


void Minipc_Pid_Init()
{
		PID_struct_init(&pid_minipc_yaw, POSITION_PID, 50, 1, 1.0f,	0.00f, 0.0f);  
		PID_struct_init(&pid_minipc_pit, POSITION_PID, 6000, 5000, 0.5f,	0.00f, 0.0f);   
}


/* �������岿�� -------------------------------------------------------------*/
/***************************************************************************************
**
	*	@brief	Data_Pro_task(void const * argument)
	*	@param
	*	@supplement	ң�����ݽ��ռ���������
	*	@retval	
****************************************************************************************/
void Remote_Data_Task(void const * argument)
{
	  uint32_t NotifyValue;

		portTickType xLastWakeTime;
		xLastWakeTime = xTaskGetTickCount();
	
	for(;;)
	{
		NotifyValue = ulTaskNotifyTake(pdTRUE,portMAX_DELAY);  //δ������֪ͨ��������״̬ȥ�ȴ�����֪ͨ
		
    if(NotifyValue == 1)
		{
			RefreshTaskOutLineTime(RemoteDataTask_ON);
			
			Remote_Ctrl();   //���ջ�ԭʼ���ݴ���
			
//			Send_MiniPC_Data(gimbal_status.minipc_color,gimbal_status.minipc_mode,yaw_get.offset_angle,pit_get.offset_angle,0);
			CAN_Send_Remote(&hcan2,RC_Ctl.key.v,RC_Ctl.rc.ch0,RC_Ctl.rc.ch1,RC_Ctl.rc.s1,RC_Ctl.rc.s2);
			
			switch(RC_Ctl.rc.s1)  //��ť
			{
					case 1: //��
					{
						gimbal_status.remote_mode = 0;
						MouseKeyControlProcess();
					}break; 
					case 2: //��
					{
						gimbal_status.remote_mode = 0;
				    RemoteControlProcess();
					}break; 
					case 3: //��
					{
						gimbal_status.remote_mode = 1;
					}break;  
					default :break;
			}
		
		}
		
		osDelayUntil(&xLastWakeTime, REMOTE_PERIOD);
	}
}

/***************************************************************************************
**
	*	@brief	RemoteControlProcess()
	*	@param
	*	@supplement	��ң�������жԽӣ���ң���������ݽ��д���ʵ�ֶԵ��̡���̨����������Ŀ���
	*	@retval	
****************************************************************************************/
void RemoteControlProcess()  
{
		switch(RC_Ctl.rc.s2)
		{
			case 1: //��
			{
				 gimbal_status.gimbal_mode = 1;            //������ģʽ
//				 Set_AX6(2,819,0x3ff);       //��ʱ��Ƕ�   240�� ��
//				gimbal_status.bopan_mode = bopan_Lianfa_mode;
			}break; 
			case 2: //��
			{
				 gimbal_status.gimbal_mode = 2;            //������ģʽ  
//				 Set_AX6(2,819,0x3ff);       //��ʱ��Ƕ�   240�� ��
				gimbal_status.bopan_mode = bopan_Stop;
			   
			}break; 
			case 3: //��
			{
//				 gimbal_status.gimbal_mode = 3;            //С����ģʽ
//				 Set_AX6(2,512,0x3ff);       //��ʱ��Ƕ�   150��
//         gimbal_status.bopan_mode = bopan_danfa_mode;
				
				gimbal_status.gimbal_mode = 4;    //�Ӿ�ģʽ
				gimbal_status.minipc_mode = 1;    //����ģʽ
			}break; 
			default :break;
		}					
        
    if((gimbal_status.gimbal_mode == 1) || (gimbal_status.gimbal_mode == 3))            //������ģʽ
		{
			  yaw_tly_set.expect = yaw_tly_set.expect + (RC_Ctl.rc.ch2 - 0x400) / 10;	
				pit_set.expect = pit_set.expect + (0x400 - RC_Ctl.rc.ch3) / 20;	              //pit_set.expect_remote����ģʽӰ��		
			
		}else if(gimbal_status.gimbal_mode == 2 )       // ������ģʽ  
		{
				yaw_set.expect = yaw_set.expect + (RC_Ctl.rc.ch2 - 0x400) / 10;
		  	pit_set.expect = pit_set.expect + (0x400 - RC_Ctl.rc.ch3) / 20;	 
		}
			
}

/***************************************************************************************
**
	*	@brief	MouseKeyControlProcess()
	*	@param
	*	@supplement	�Լ�������ݽ��д���
	*	@retval	
****************************************************************************************/
void MouseKeyControlProcess()
{
		
    gimbal_status.gimbal_mode = 2;  
                   
}

/***************************************************************************************
**
	*	@brief	hard_brak()
	*	@param
	*	@supplement	����ֹͣ����
	*	@retval	
****************************************************************************************/
void hard_brak()
{

	
}





/***************************************************************************************
**
	*	@brief	MiniPC_Data_task(void const * argument)
	*	@param
	*	@supplement	�Ӿ����ݴ�������
	*	@retval	
****************************************************************************************/
void MiniPC_Data_task(void const * argument)
{
	static uint8_t frist_flag = 0;
  
  static int16_t  frist_angle;//pit�ĽǶ�ƫ��
	minipc_rx.state_flag = 0;
	minipc_rx.angle_pit  = 0;
	minipc_rx.angle_yaw  = 0;
  uint32_t NotifyValue;
	Minipc_Pid_Init();
	osDelay(4000);//��ʱ4000ms���ȴ���̨�������������ת��ſ�ʼ�Ӿ����ݽ���
	portTickType xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
		frist_angle  =  pit_get.total_angle-pit_set.expect_pc;
	for(;;)
	{
	   NotifyValue=ulTaskNotifyTake(pdTRUE,portMAX_DELAY);
    if(NotifyValue==1)
		{
			Get_MiniPC_Data();  //��ȡ�Ӿ���������Ϣ
			
			switch(gimbal_status.minipc_mode)
			{
					case 1 ://����
					{
						if(my_abs(minipc_rx.angle_yaw) > 640 || my_abs(minipc_rx.angle_pit) > 360)  //pc�����쳣
						{
							
						}else
						{
								if(minipc_rx.state_flag == 1)    //�Ӿ���⵽��׼��Ŀ�긽���󣬲��ٵ���λ��
								{
									yaw_set.expect_pc -= minipc_rx.angle_yaw;    //yaw�ұ�Ϊ��
									pit_set.expect_pc -= minipc_rx.angle_pit;    //               
									
								}else if(minipc_rx.state_flag == 2)
								{
									yaw_set.expect_pc -= minipc_rx.angle_yaw;    //yaw�ұ�Ϊ��
									pit_set.expect_pc -= minipc_rx.angle_pit;    // 
									gimbal_status.bopan_mode = bopan_danfa_mode;
								}      
						}
						minipc_rx.angle_yaw = 0;
						minipc_rx.angle_pit = 0; 
					}break ;
					
					case 2 ://���						
					{
						
//						if(my_abs(minipc_rx.angle_yaw)>500 || my_abs(minipc_rx.angle_yaw)>500)  //pc�����쳣
//						{
//						}else
//						{
//								Shoot_status.remote_mode = 1; //������Ĩ����
//								if( frist_flag == 0)
//								{
//										if(shoot.out == 128 )  frist_flag=1;
//								
//										yaw_set.expect_pc = minipc_rx.angle_yaw;
//										pit_set.expect_pc = minipc_rx.angle_pit;
//								}
//							
//								if(minipc_rx.state_flag)//��⵽��ͬװ�׿�ʱ������׼λ��
//								{
//										yaw_set.expect_pc = minipc_rx.angle_yaw;
//										pit_set.expect_pc = minipc_rx.angle_pit;
//								}
//								else if( frist_flag )//�������õ�̫��
//								{
//										yaw_set.expect_pc = minipc_rx.angle_yaw ;               
//										pit_set.expect_pc = minipc_rx.angle_pit ;
//										gimbal_status.bopan_mode = 1;
//								}else     
//										gimbal_status.bopan_mode = 0;
//								
//						 }break;
					 }
					 default : break;
				}
			 
		}

	  osDelayUntil(&xLastWakeTime, MINIPC_PERIOD);
  }
}
