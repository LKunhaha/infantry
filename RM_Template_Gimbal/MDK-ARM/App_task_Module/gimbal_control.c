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
Mode_Set Gimbal={0};
Mode_Set Minipc={0};
Mode_Set Shoot={0};

pid_t pid_yaw       = {0};  //yaw��λ�û�
pid_t pid_yaw_jy901 = {0};  //��������� /*Ŀǰֻ����λ�û�*/ 
pid_t pid_pit       = {0};	//pit��λ�û�
pid_t pid_pit_dashen = {0};       //���������
pid_t pid_pit_dashen_spd = {0};
pid_t pid_yaw_spd   = {0};	//yaw���ٶȻ�
pid_t pid_pit_spd   = {0};	//pit���ٶȻ�
pid_t pid_yaw_jy901_spd = {0};
pid_t pid_pit_jy901 = {0};
pid_t pid_pit_jy901_spd = {0};

pid_t pid_yaw_saber = {0};  //��������� /*Ŀǰֻ����λ�û�*/
pid_t pid_yaw_saber_spd = {0};
pid_t pid_pit_saber = {0};
pid_t pid_pit_saber_spd = {0};

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

void gimbal_pid_init(void)    //�´ε�һ��PID
{
	
   /* yaw axis motor pid parameter */
   //�����ģʽ
   PID_struct_init(&pid_pit_dashen, POSITION_PID, 5000, 1000, 6.0f, 0.05f, 0.5f);  
   PID_struct_init(&pid_pit_dashen_spd, POSITION_PID, 5000, 1000, 2.0f, 0.0f, 0.0f);
  
	 PID_struct_init(&pid_yaw, POSITION_PID, 5000, 500, 6.0f, 0.05f, 5.0f); 
	 PID_struct_init(&pid_yaw_spd, POSITION_PID, 5000, 1000, 2.0f, 0.0f, 0.0f);
		
   //����ģʽ
   PID_struct_init(&pid_pit_zimiao, POSITION_PID, 5000, 1000, 4.0f, 0.03f, 2.0f);  //4.0 0.03 2.0
   PID_struct_init(&pid_pit_zimiao_spd, POSITION_PID, 5000, 1000, 5.0f, 0.0f, 0.0f);  //1.5 0.0 0.0
  
	 PID_struct_init(&pid_yaw_zimiao, POSITION_PID, 5000, 300, 15.0f, 0.01f, 25.0f); // 6.0 0.05 5.0 //15.0 0.01 25.0
	 PID_struct_init(&pid_yaw_zimiao_spd, POSITION_PID, 5000, 1000, 1.5f, 0.0f, 0.0f);  
  
   //������ģʽ
   PID_struct_init(&pid_pit, POSITION_PID, 5000, 100, 4.5f, 0.01f, 10.0f); 
   PID_struct_init(&pid_pit_jy901_spd, POSITION_PID, 5000, 1000, 2.0f, 0.0f, 0.0f );
  
   PID_struct_init(&pid_yaw_jy901, POSITION_PID, 5000, 500, 2.5f, 0.01f, 0.0f); 
   PID_struct_init(&pid_yaw_jy901_spd, POSITION_PID, 5000, 500, 2.0f, 0.0f, 1.0f );	
	
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
	yaw_set.mode = 0;
	Gimbal.flag = 0;
	Gimbal.mode = 3;          //��ʼ�趨Ϊ������ģʽ
	Pitch_Current_Value = 0;
	Yaw_Current_Value = 0;
	gimbal_pid_init();
	
	osDelay(200);//��ʱ200ms
	portTickType xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();
/*************************     ģʽ˵��     ********************************
		minipc.mode              ��0  �ر��Ӿ�
	                             1  ����               ��gimbal.mode��Ϊ2
															 2  ��������           ��gimbal.mode��Ϊ3
					
		gimbal.mode	             ��0  ��̨ʧЧ
															 1  ������ģʽ(��Ӧ���̵ĸ���ģʽ,��Ť��ģʽ��)
															 2  ����
															 3  ������ģʽ(YAW�ᳬ��ʱ����������ģʽ Ҳ��Ϊ��ģʽ) 
					
		gimbal.flag	             ��0  yaw�᲻����
															 1  ��δʹ��
															 2  yaw���ޱ�־
															 3  ��δʹ��
	****************************************************************************/			
	for(;;)		
  {
		  IMU_Get_Data();
		  CAN_Send_Gimbal(&hcan2,yaw_get.angle,yaw_get.total_angle,Minipc.mode,Gimbal.mode,Gimbal.flag,Remote.Mode);//���ط���
			
	    RefreshTaskOutLineTime(GimbalContrlTask_ON);
      
			if(Minipc.mode == 2)   //��������
			{
            Gimbal.mode = 3;
            yaw_set.expect = yaw_set.expect_pc;
            pit_set.expect = pit_set.expect_pc;
        
            yaw_tly_set.expect_remote = ptr_jy901_t_yaw.final_angle;       //���浱ǰyaw�Ƕ�ֵ(�������ǵķ�ʽ,ʵʱ����)
            pit_set.expect_remote = pit_get.total_angle;                    //���浱ǰpitch��ԵĽǶ�ֵ(�Ե���Ƕȵķ�ʽ,ʵʱ����)
			}else if(Minipc.mode == 1 && Gimbal.flag == 0) //����&&������
			{
            Gimbal.mode = 2;
            yaw_set.expect = yaw_set.expect_pc;
            pit_set.expect = pit_set.expect_pc;
        
            yaw_tly_set.expect_remote = ptr_jy901_t_yaw.final_angle;     //���浱ǰyaw�Ƕ�ֵ(�������ǵķ�ʽ,ʵʱ����)����ֹȡ������󣬲����ϴ�ĽǶȲ���
            pit_set.expect_remote = pit_get.total_angle;                 //���浱ǰpitch��ԵĽǶ�ֵ(�Ե���Ƕȵķ�ʽ,ʵʱ����)��ͬ��
			}else                                                //ң��ģʽ
			{                                                    //���Ӿ�ģʽ  ��  ����&&����
            yaw_tly_set.expect = yaw_tly_set.expect_remote;            
            pit_set.expect = pit_set.expect_remote;
			}
			
			
			if((yaw_get.angle < 3900) )                 //��̨���޴��� 
			{
            yaw_set.expect = 3900 - yaw_get.offset_angle;
            Gimbal.mode = 3;                      //���޺��Ա�����ģʽ��������
            Gimbal.flag = 2;                      //���ޱ�־λ
			}else if((yaw_get.angle > 7500) )    
			{
            yaw_set.expect = 7500 - yaw_get.offset_angle;
            Gimbal.mode = 3;                      //���޺��Ա�����ģʽ��������
            Gimbal.flag = 2;                      //���ޱ�־λ       
			}
			else if (Gimbal.flag == 2)                  //Yaw�ᳬ�޺�ֻ�е�yaw�᲻���޲Ż������жϣ���������ޱ�־λ
			{   
//            Gimbal.mode = 1;                    //��Ϊ������ģʽ
				    Gimbal.flag = 0;                      //������ޱ�־
//            yaw_set.expect = 0;
            yaw_tly_set.expect = ptr_jy901_t_yaw.final_angle;  //yaw�����ֵΪ��ǰ��yaw��Ƕ�ֵ(�����Ƿ�ʽ)		                                                 
			}
     
	
			if ( pit_get.angle < 5700)
			{
				    pit_set.expect_remote = 5800 - pit_get.offset_angle;
				    pit_set.expect = 5800 - pit_get.offset_angle;
			}
			else if (pit_get.angle > 7500)
			{
				    pit_set.expect_remote = 7450 - pit_get.offset_angle;
				    pit_set.expect = 7450 - pit_get.offset_angle;
			}
	

			switch(Gimbal.mode)
			{	
				case 1: 
				{     //������ģʽ
							pid_calc(&pid_yaw_jy901,(ptr_jy901_t_yaw.final_angle), yaw_tly_set.expect);  //ptr_jy901_t_yaw.final_angleΪ����ڳ�ʼ�����ǵĽǶ�ֵ����������̨������ܽǶ��㷨
							pid_calc(&pid_yaw_jy901_spd,(imu_data.gx)/30, pid_yaw_jy901.pos_out);
					    Yaw_Current_Value = (-pid_yaw_jy901_spd.pos_out);
					
          		//pit��
	            pid_calc(&pid_pit, pit_get.total_angle, pit_set.expect);
							pid_calc(&pid_pit_jy901_spd,(imu_data.gz)/16.4, pid_pit.pos_out);
              Pitch_Current_Value = (-pid_pit_jy901_spd.pos_out); 
					
				      yaw_set.expect = yaw_get.total_angle;
				}break;
				
        case 2:
				{
					    //����ģʽ
							pid_calc(&pid_yaw_zimiao,yaw_get.total_angle, yaw_set.expect);
							pid_calc(&pid_yaw_zimiao_spd,(imu_data.gx)/30, pid_yaw_zimiao.pos_out);
					    Yaw_Current_Value = (-pid_yaw_zimiao_spd.pos_out);
          
          		//pit��
							pid_calc(&pid_pit_zimiao, pit_get.total_angle, pit_set.expect);
						  pid_calc(&pid_pit_zimiao_spd,(imu_data.gz)/16.4, pid_pit_zimiao.pos_out);
              Pitch_Current_Value = (-pid_pit_zimiao_spd.pos_out); 
				 }break;
				
				case 3: 
				{     //������ģʽ  
							pid_calc(&pid_yaw, yaw_get.total_angle, yaw_set.expect);
						  pid_calc(&pid_yaw_spd,(imu_data.gx)/30, pid_yaw.pos_out);
							Yaw_Current_Value = (-pid_yaw_spd.pos_out);
				
							pid_calc(&pid_pit_dashen, pit_get.total_angle, pit_set.expect);
						  pid_calc(&pid_pit_dashen_spd,(imu_data.gz)/16.4, pid_pit_dashen.pos_out);   
							Pitch_Current_Value = (-pid_pit_dashen_spd.pos_out); 
					
					    yaw_tly_set.expect = ptr_jy901_t_yaw.final_angle;
				 }break; 
//        case 4 ://����
//               {
//           
//               }break;
				 default: break;
				
			}                                                                             
//		 		
			Cloud_Platform_Motor(&hcan1,-Yaw_Current_Value,Pitch_Current_Value);
			pit_set.expect_remote_last = pit_set.expect_remote;
			
		  osDelayUntil(&xLastWakeTime, GIMBAL_PERIOD);
  }
 
}
