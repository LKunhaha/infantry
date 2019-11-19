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
			
			Send_MiniPC_Data(gimbal_status.minipc_color,gimbal_status.minipc_mode,yaw_get.offset_angle,pit_get.offset_angle,0);
			CAN_Send_Remote(&hcan2,RC_Ctl.key.v,RC_Ctl.rc.ch0,RC_Ctl.rc.ch1,RC_Ctl.rc.s1,RC_Ctl.rc.s2);
			
			RemoteControlProcess();   //ң�������ݴ���
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
				 Set_AX6(2,819,0x3ff);       //��ʱ��Ƕ�   240�� ��
//						RemoteControlProcess();
			}break; 
			case 2: //��
			{
				 Set_AX6(2,819,0x3ff);       //��ʱ��Ƕ�   240�� ��
			   gimbal_status.gimbal_mode = 3;            //������ģʽ  ���̵�����Ϊ������ģʽ
//						MouseKeyControlProcess();
			}break; 
			case 3: //��
			{
				 Set_AX6(2,512,0x3ff);       //��ʱ��Ƕ�   150��
//						hard_brak();
			}break; 
			default :break;
		}					
        
    if(gimbal_status.gimbal_mode == 1 )            //������ģʽ
		{
				pit_set.expect_remote = pit_set.expect_remote +(0x400-RC_Ctl.rc.ch3)/20;	  //pit_set.expect_remote����ģʽӰ��
				yaw_tly_set.expect_remote = yaw_tly_set.expect_remote +(0x400-RC_Ctl.rc.ch2)/20;	
			
		}else if(gimbal_status.gimbal_mode == 3)       // ������ģʽ  ||  ����
		{
				pit_set.expect_remote = pit_set.expect_remote +(0x400-RC_Ctl.rc.ch3)/20;	
				yaw_set.expect = yaw_set.expect +(0x400-RC_Ctl.rc.ch2)/20;
		}

	  if(RC_Ctl.rc.s1 == 1)            //��
		{    
			
				 gimbal_status.bopan_mode = 1;             //����              
//						 HAL_GPIO_WritePin(GPIOG,LASTER_Pin,GPIO_PIN_RESET);//����ر�

		}
		else if(RC_Ctl.rc.s1 == 2)       //��
		{
	//		gimbal_status.gimbal_mode = 3;              //������ģʽ
//                Shoot.mode=3;
//                if(shoot.out == 115) 
//              gimbal_status.bopan_mode=3;    
		
				gimbal_status.bopan_mode = 3;               //����
//           Set_AX6(2,512,0x3ff);    //��ʱ��Ƕ�   150��
		}else                             //��
		{
			  gimbal_status.bopan_mode = 0;                 //ֹͣ
//							HAL_GPIO_WritePin(GPIOG,LASTER_Pin,GPIO_PIN_SET);//���⿪��
//							gimbal_status.bopan_mode = 0;
//							gimbal_status.gimbal_mode = 1;     //������ģʽ
		    gimbal_status.minipc_mode = 0;
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
 
						if(my_abs(minipc_rx.angle_yaw)>640 || my_abs(minipc_rx.angle_pit)>360)  //pc�����쳣
						{
							
						}else
						{
//								      pid_calc(&pid_minipc_yaw,minipc_rx.angle_yaw,0);
//                      pid_calc(&pid_minipc_pit,minipc_rx.angle_pit,0);

//                      yaw_zm_set.expect_pc = (pid_minipc_yaw.pos_out)+tly.final_angle;
//											pit_zm_set.expect_pc = (-pid_minipc_pit.pos_out)+pit_get.total_angle;

//                      yaw_zm_set.expect_pc += (-minipc_rx.angle_yaw);
//											pit_zm_set.expect_pc += (minipc_rx.angle_pit);                   
							
						  	minipc_rx.angle_yaw = 0;
						  	minipc_rx.angle_pit = 0;
//                      
//                      if(yaw_zm_set.expect_pc>500)yaw_zm_set.expect_pc=500;
//                      else if(yaw_zm_set.expect_pc<-500)yaw_zm_set.expect_pc=-500;
//                      
//                      if(pit_zm_set.expect_pc>500)pit_zm_set.expect_pc=500;
//                      else if(yaw_zm_set.expect_pc<-500)yaw_zm_set.expect_pc=-500;
							
							/*  ��׼�Ĳ���ʹ��
							Shoot.mode = 3; //������Ĩ���� ���ò�ͬ���ٶ�
							if( shoot.out == 112 ) gimbal_status.bopan_mode  = 1;
							*/
						}
					}break ;
					
					case 2 ://���						
					{
						
						if(my_abs(minipc_rx.angle_yaw)>500 || my_abs(minipc_rx.angle_yaw)>500)  //pc�����쳣
						{
						}else
						{
								Shoot_status.remote_mode = 1; //������Ĩ����
								if( frist_flag == 0)
								{
										if(shoot.out == 128 )  frist_flag=1;
								
										yaw_set.expect_pc = minipc_rx.angle_yaw;
										pit_set.expect_pc = minipc_rx.angle_pit;
								}
							
								if(minipc_rx.state_flag)//��⵽��ͬװ�׿�ʱ������׼λ��
								{
										yaw_set.expect_pc = minipc_rx.angle_yaw;
										pit_set.expect_pc = minipc_rx.angle_pit;
								}
								else if( frist_flag )//�������õ�̫��
								{
										yaw_set.expect_pc = minipc_rx.angle_yaw ;               
										pit_set.expect_pc = minipc_rx.angle_pit ;
										gimbal_status.bopan_mode = 1;
								}else     
										gimbal_status.bopan_mode = 0;
								
						 }break;
					 }
					 default : break;
				}
			 
		}

	  osDelayUntil(&xLastWakeTime, MINIPC_PERIOD);
  }
}
