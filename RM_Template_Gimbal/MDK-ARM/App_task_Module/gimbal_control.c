/* ����ͷ�ļ�----------------------------------------------------------------*/
#include "gimbal_control.h"
#include "Power_restriction.h"
#include "SystemState.h"
#include "user_lib.h"

/* �ڲ�����------------------------------------------------------------------*/
static  int16_t Yaw_Current_Value = 0;
static  int16_t Pitch_Current_Value = 0;

Pos_Set  yaw_set={0};
Pos_Set  yaw_tly_set={0};
Pos_Set  pit_set={0};
Gimbal_Status_t gimbal_status;

pid_t pid_yaw       = {0};  //yaw��λ�û�
pid_t pid_yaw_spd   = {0};	//yaw���ٶȻ�

pid_t pid_pit       = {0};	//pit��λ�û�
pid_t pid_pit_spd   = {0};	//pit���ٶȻ�

pid_t pid_pit_start = {0};
pid_t pid_pit_start_spd = {0};
pid_t pid_yaw_start = {0};
pid_t pid_yaw_start_spd = {0};

pid_t pid_yaw_jy901 = {0};  //��������� /*Ŀǰֻ����λ�û�*/ 
pid_t pid_yaw_jy901_spd = {0};

pid_t pid_pit_dashen = {0};       //���������
pid_t pid_pit_dashen_spd = {0};

pid_t pid_pit_jy901 = {0};

//zimiao
pid_t pid_yaw_zimiao = {0};        //
pid_t pid_yaw_zimiao_spd = {0};
pid_t pid_pit_zimiao = {0};        //
pid_t pid_pit_zimiao_spd = {0};
/* �ڲ�����ԭ������----------------------------------------------------------*/
/**                                                           //����
	**************************************************************
	** Descriptions: ��̨pid��ʼ��
	** Input:  NULL
	** Output: NULL
	**************************************************************
**/

void gimbal_pid_init(void)   
{
	
	 PID_struct_init(&pid_yaw, POSITION_PID, 550, 350, 1.15f, 0.033f, 0.0f);              //550, 350, 1.15f, 0.033f, 0.0f  
	 PID_struct_init(&pid_yaw_spd, POSITION_PID, 25000, 16000, 60.0f, 2.25f, 0.1f);       //25000, 16000, 60.0f, 2.25f, 0.1f
		 
   PID_struct_init(&pid_pit, POSITION_PID, 650, 400, 1.25f, 0.025f, 0.0f);              //650, 400, 1.25f, 0.025f, 0.0f
   PID_struct_init(&pid_pit_spd, POSITION_PID, 25000, 16000, 45.0f, 0.95f, 0.1f );      //25000, 15000, 45.0f, 0.95f, 0.1f
  
	 PID_struct_init(&pid_yaw_jy901, POSITION_PID, 500, 300, 0.95f, 0.027f, 0.0f);        //������ģʽ�µ�pidӦСһ��
   PID_struct_init(&pid_yaw_jy901_spd, POSITION_PID, 20000, 12000, 45.0f, 1.73f, 0.1f );	//pit���������ģʽ����һ��pid
	
	
	 //��̨����
	 PID_struct_init(&pid_pit_start, POSITION_PID, 400, 230, 0.75f, 0.015f, 0.0f);              //����pid,��ƫ���Ƚϴ󣬸���ֵ��С
   PID_struct_init(&pid_pit_start_spd, POSITION_PID, 18000, 10000, 41.0f, 0.75f, 0.1f ); 
	
	 PID_struct_init(&pid_yaw_start, POSITION_PID, 400, 230, 0.85f, 0.018f, 0.0f);              //����pid
   PID_struct_init(&pid_yaw_start_spd, POSITION_PID, 18000, 10000, 45.0f, 0.75f, 0.1f ); 
	
	 //�����ģʽ
   PID_struct_init(&pid_pit_dashen, POSITION_PID, 5000, 1000, 6.0f, 0.05f, 0.5f);  
   PID_struct_init(&pid_pit_dashen_spd, POSITION_PID, 5000, 1000, 2.0f, 0.0f, 0.0f);
	
   
	
}
/* �������岿�� -------------------------------------------------------------*/

/***************************************************************************************
**
	*	@brief	Gimbal_Contrl_Task(void const * argument)
	*	@param
	*	@supplement	��̨�������
	*	@retval	
****************************************************************************************/
void Gimbal_Task(void const * argument)
{
	static uint16_t gimbal_start_count = 0;
	static uint8_t gimbal_start_mode = 1;  //��ʼ�趨Ϊpit������ģʽ
	
	Pitch_Current_Value = 0;
	Yaw_Current_Value = 0;
	gimbal_pid_init();
	
	portTickType xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();
/*************************     ģʽ˵��     ********************************
		gimbal_status.minipc_mode              ��0  �ر��Ӿ�
	                                           1  ����               ��gimbal_status.gimbal_mode��Ϊ2
															               2  ��������           ��gimbal_status.gimbal_mode��Ϊ3
					
		gimbal_status.gimbal_mode	             ��0  ��̨ʧЧ
															               1  ������ģʽ(��ʱ����Ϊ����ģʽ)
															               2  ������ģʽ
															               3  С����ģʽ
					                                   4  ����ģʽ
																						 5
																						 6
																						 7
																						 8  pit������ģʽ�����ڸ��ϵ�����ʱ��ʹ�ô�ģʽ��
																						 9  yaw������ģʽ�����ڸ��ϵ�����ʱ��ʹ�ô�ģʽ��
																						 
		gimbal_start_mode                      ��0  ��̨�����׶ν���
															               1  pit������
															               2  pit�ᵽyaw�������Ĺ���
															               3  yaw������
	****************************************************************************/			
	for(;;)		
  {
		  IMU_Get_Data();
		  CAN_Send_Gimbal(&hcan2,&yaw_get,&gimbal_status);//���ط���
			
	    RefreshTaskOutLineTime(GimbalContrlTask_ON);
      
			if(gimbal_start_mode == 1 )        //pit������ 
			{
         gimbal_status.gimbal_mode = 8;     //pit������ģʽ
				
			}else if(gimbal_start_mode == 2 ) //pit��������yaw��������Ĺ���
			{
				 gimbal_status.gimbal_mode = 8;
				 gimbal_start_count++;
				 if(gimbal_start_count > 100 )   //����ʱ��ȴ�
				 {
						gimbal_start_mode = 3;  //yaw������
				 }
			}else if(gimbal_start_mode == 3 )    //yaw������
			{
         gimbal_status.gimbal_mode = 9;     //yaw������ģʽ    
			}                                              

			
		  if(gimbal_start_mode == 1)      //pit������ģʽ��
			{
		     if( ABS(pit_get.total_angle) <= 100)  //pit�ᵽ��Ŀ��λ�ø���
				 {
					 gimbal_start_mode = 2;   //���ɽ׶�
				 }
			}else if(gimbal_start_mode == 3)
			{
				if( ABS(yaw_get.total_angle) <= 100)  //yaw�ᵽ��Ŀ��λ�ø���
				 {
					 gimbal_start_mode = 0;   //�����׶ν���
					 gimbal_status.gimbal_mode = 2;
				 }
			}
			
			switch(gimbal_status.gimbal_mode)
			{	
				case 1: //������ģʽ
				{     
							pid_calc(&pid_yaw_jy901, ptr_jy901_t_yaw.total_angle, yaw_tly_set.expect);  
							pid_calc(&pid_yaw_jy901_spd, (imu_data.gx)/16.4, pid_yaw_jy901.pos_out);
					    Yaw_Current_Value = pid_yaw_jy901_spd.pos_out;
					
	            pid_calc(&pid_pit, pit_get.total_angle, pit_set.expect);
							pid_calc(&pid_pit_spd, (imu_data.gz)/16.4, pid_pit.pos_out);          //(imu_data.gz)/16.4  Ϊ��/s
              Pitch_Current_Value = pid_pit_spd.pos_out; 
					
				      yaw_set.expect = yaw_get.total_angle;               //��������ģʽ�±����������ʱ��ֵ
				}break;
				
        case 2: //������ģʽ 
			  { 
							pid_calc(&pid_yaw, yaw_get.total_angle, yaw_set.expect);
						  pid_calc(&pid_yaw_spd,(imu_data.gx)/16.4, pid_yaw.pos_out);
							Yaw_Current_Value = pid_yaw_spd.pos_out;
				
							pid_calc(&pid_pit, pit_get.total_angle, pit_set.expect);
						  pid_calc(&pid_pit_spd,(imu_data.gz)/16.4, pid_pit.pos_out);   
							Pitch_Current_Value = pid_pit_spd.pos_out; 
					
					    yaw_tly_set.expect = ptr_jy901_t_yaw.total_angle;   //�ڱ�����ģʽ�±��������Ǵ�ʱ��ֵ
			  }break;
				
			  case 3: //С����ģʽ����������ģʽ��ͬ�����ڵ��̿�����������  
			  {  
				      pid_calc(&pid_yaw, ptr_jy901_t_yaw.total_angle, yaw_tly_set.expect);  
							pid_calc(&pid_yaw_spd, (imu_data.gx)/16.4, pid_yaw.pos_out);
					    Yaw_Current_Value = pid_yaw_spd.pos_out;
					
	            pid_calc(&pid_pit, pit_get.total_angle, pit_set.expect);
							pid_calc(&pid_pit_spd, (imu_data.gz)/16.4, pid_pit.pos_out);          //(imu_data.gz)/16.4  Ϊ��/s
              Pitch_Current_Value = pid_pit_spd.pos_out; 
					
				      yaw_set.expect = yaw_get.total_angle;               //��������ģʽ�±����������ʱ��ֵ
			  }break; 

				case 4: //����ģʽ
			  {  
				      pid_calc(&pid_yaw, yaw_get.total_angle, yaw_set.expect_pc);  
							pid_calc(&pid_yaw_spd, (imu_data.gx)/16.4, pid_yaw.pos_out);
					    Yaw_Current_Value = pid_yaw_spd.pos_out;
					
	            pid_calc(&pid_pit, pit_get.total_angle, pit_set.expect_pc);
							pid_calc(&pid_pit_spd, (imu_data.gz)/16.4, pid_pit.pos_out);          //(imu_data.gz)/16.4  Ϊ��/s
              Pitch_Current_Value = pid_pit_spd.pos_out; 
					
				      yaw_set.expect = yaw_get.total_angle;               //������ģʽ�±����������ʱ��ֵ
					    yaw_tly_set.expect = ptr_jy901_t_yaw.total_angle;   //������ģʽ�±��������Ǵ�ʱ��ֵ
			  }break; 
				
				case 5:  //��������ģʽ
				{
				}break;
				
				
				case 8:  //pit������������������pit�ᣬ������yaw�ᣩ
				{
					    pid_calc(&pid_pit_start, pit_get.total_angle, 0);
						  pid_calc(&pid_pit_start_spd,(imu_data.gz)/16.4, pid_pit_start.pos_out);   
							Pitch_Current_Value = pid_pit_start_spd.pos_out;
				}break;
				
				case 9:  //yaw������
				{
					    pid_calc(&pid_pit_start, pit_get.total_angle, 0);
						  pid_calc(&pid_pit_start_spd,(imu_data.gz)/16.4, pid_pit_start.pos_out);   
							Pitch_Current_Value = pid_pit_start_spd.pos_out;
					
					    pid_calc(&pid_yaw_start, yaw_get.total_angle, 0);
						  pid_calc(&pid_yaw_start_spd,(imu_data.gx)/16.4, pid_yaw_start.pos_out);
							Yaw_Current_Value = pid_yaw_start_spd.pos_out;
				}break;
				
			  default: break;
				
			}   

		 if(gimbal_status.remote_mode == 1)
		 {
				Cloud_Platform_Motor(&hcan2, 0, 0);        
			  Cloud_Platform_Motor(&hcan1, 0, 0);      
		 }else
		 {
				Cloud_Platform_Motor(&hcan2, Yaw_Current_Value, 0);        //YAW��������
			  Cloud_Platform_Motor(&hcan1, 0, Pitch_Current_Value);      //PITCH��������					
		 }
		 
		 osDelayUntil(&xLastWakeTime, GIMBAL_PERIOD);
  }
 
}


void Gimbal_angle_Conversion(moto_measure_t *ptr)      //��̨�Ƕ�ת��������Ϊ���ԽǶ�(-4096��4096)
{
	  while(1)
		{
			 if(ptr->total_angle < -4096)
			 {
				  ptr->total_angle += 8192;
				  ptr->round_cnt++;
			 }else if(ptr->total_angle > 4096)
			 {
				  ptr->total_angle -= 8192;
				  ptr->round_cnt--;
			 }else
			 {
				  break;
			 }
		}
		
}
