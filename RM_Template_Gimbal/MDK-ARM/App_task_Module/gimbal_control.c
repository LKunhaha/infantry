/* 包含头文件----------------------------------------------------------------*/
#include "gimbal_control.h"
#include "Power_restriction.h"
#include "SystemState.h"
#include "user_lib.h"

/* 内部变量------------------------------------------------------------------*/
static  int16_t Yaw_Current_Value = 0;
static  int16_t Pitch_Current_Value = 0;

Pos_Set  yaw_set={0};
Pos_Set  yaw_tly_set={0};
Pos_Set  pit_set={0};
Gimbal_Status_t gimbal_status;

pid_t pid_yaw       = {0};  //yaw轴位置环
pid_t pid_yaw_spd   = {0};	//yaw轴速度环

pid_t pid_pit       = {0};	//pit轴位置环
pid_t pid_pit_spd   = {0};	//pit轴速度环

pid_t pid_pit_start = {0};
pid_t pid_pit_start_spd = {0};
pid_t pid_yaw_start = {0};
pid_t pid_yaw_start_spd = {0};

pid_t pid_yaw_jy901 = {0};  //外接陀螺仪 /*目前只用于位置环*/ 
pid_t pid_yaw_jy901_spd = {0};

pid_t pid_pit_dashen = {0};       //大神符参数
pid_t pid_pit_dashen_spd = {0};

pid_t pid_pit_jy901 = {0};

//zimiao
pid_t pid_yaw_zimiao = {0};        //
pid_t pid_yaw_zimiao_spd = {0};
pid_t pid_pit_zimiao = {0};        //
pid_t pid_pit_zimiao_spd = {0};
/* 内部函数原型声明----------------------------------------------------------*/
/**                                                           //待续
	**************************************************************
	** Descriptions: 云台pid初始化
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
  
	 PID_struct_init(&pid_yaw_jy901, POSITION_PID, 500, 300, 0.95f, 0.027f, 0.0f);        //陀螺仪模式下的pid应小一点
   PID_struct_init(&pid_yaw_jy901_spd, POSITION_PID, 20000, 12000, 45.0f, 1.73f, 0.1f );	//pit轴与编码器模式共用一套pid
	
	
	 //云台启动
	 PID_struct_init(&pid_pit_start, POSITION_PID, 400, 230, 0.75f, 0.015f, 0.0f);              //启动pid,因偏差会比较大，给的值较小
   PID_struct_init(&pid_pit_start_spd, POSITION_PID, 18000, 10000, 41.0f, 0.75f, 0.1f ); 
	
	 PID_struct_init(&pid_yaw_start, POSITION_PID, 400, 230, 0.85f, 0.018f, 0.0f);              //启动pid
   PID_struct_init(&pid_yaw_start_spd, POSITION_PID, 18000, 10000, 45.0f, 0.75f, 0.1f ); 
	
	 //大神符模式
   PID_struct_init(&pid_pit_dashen, POSITION_PID, 5000, 1000, 6.0f, 0.05f, 0.5f);  
   PID_struct_init(&pid_pit_dashen_spd, POSITION_PID, 5000, 1000, 2.0f, 0.0f, 0.0f);
	
   
	
}
/* 任务主体部分 -------------------------------------------------------------*/

/***************************************************************************************
**
	*	@brief	Gimbal_Contrl_Task(void const * argument)
	*	@param
	*	@supplement	云台电机控制
	*	@retval	
****************************************************************************************/
void Gimbal_Task(void const * argument)
{
	static uint16_t gimbal_start_count = 0;
	static uint8_t gimbal_start_mode = 1;  //初始设定为pit轴启动模式
	
	Pitch_Current_Value = 0;
	Yaw_Current_Value = 0;
	gimbal_pid_init();
	
	portTickType xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();
/*************************     模式说明     ********************************
		gimbal_status.minipc_mode              ：0  关闭视觉
	                                           1  自瞄               将gimbal_status.gimbal_mode置为2
															               2  能量机关           将gimbal_status.gimbal_mode置为3
					
		gimbal_status.gimbal_mode	             ：0  云台失效
															               1  陀螺仪模式(此时底盘为跟随模式)
															               2  编码器模式
															               3  小陀螺模式
					                                   4  自瞄模式
																						 5
																						 6
																						 7
																						 8  pit轴启动模式（仅在刚上电启动时会使用此模式）
																						 9  yaw轴启动模式（仅在刚上电启动时会使用此模式）
																						 
		gimbal_start_mode                      ：0  云台启动阶段结束
															               1  pit轴启动
															               2  pit轴到yaw轴启动的过渡
															               3  yaw轴启动
	****************************************************************************/			
	for(;;)		
  {
		  IMU_Get_Data();
		  CAN_Send_Gimbal(&hcan2,&yaw_get,&gimbal_status);//主控发送
			
	    RefreshTaskOutLineTime(GimbalContrlTask_ON);
      
			if(gimbal_start_mode == 1 )        //pit轴启动 
			{
         gimbal_status.gimbal_mode = 8;     //pit轴启动模式
				
			}else if(gimbal_start_mode == 2 ) //pit轴启动到yaw轴启动间的过渡
			{
				 gimbal_status.gimbal_mode = 8;
				 gimbal_start_count++;
				 if(gimbal_start_count > 100 )   //过渡时间等待
				 {
						gimbal_start_mode = 3;  //yaw轴启动
				 }
			}else if(gimbal_start_mode == 3 )    //yaw轴启动
			{
         gimbal_status.gimbal_mode = 9;     //yaw轴启动模式    
			}                                              

			
		  if(gimbal_start_mode == 1)      //pit轴启动模式下
			{
		     if( ABS(pit_get.total_angle) <= 100)  //pit轴到达目标位置附近
				 {
					 gimbal_start_mode = 2;   //过渡阶段
				 }
			}else if(gimbal_start_mode == 3)
			{
				if( ABS(yaw_get.total_angle) <= 100)  //yaw轴到达目标位置附近
				 {
					 gimbal_start_mode = 0;   //启动阶段结束
					 gimbal_status.gimbal_mode = 2;
				 }
			}
			
			switch(gimbal_status.gimbal_mode)
			{	
				case 1: //陀螺仪模式
				{     
							pid_calc(&pid_yaw_jy901, ptr_jy901_t_yaw.total_angle, yaw_tly_set.expect);  
							pid_calc(&pid_yaw_jy901_spd, (imu_data.gx)/16.4, pid_yaw_jy901.pos_out);
					    Yaw_Current_Value = pid_yaw_jy901_spd.pos_out;
					
	            pid_calc(&pid_pit, pit_get.total_angle, pit_set.expect);
							pid_calc(&pid_pit_spd, (imu_data.gz)/16.4, pid_pit.pos_out);          //(imu_data.gz)/16.4  为°/s
              Pitch_Current_Value = pid_pit_spd.pos_out; 
					
				      yaw_set.expect = yaw_get.total_angle;               //在陀螺仪模式下保存编码器此时的值
				}break;
				
        case 2: //编码器模式 
			  { 
							pid_calc(&pid_yaw, yaw_get.total_angle, yaw_set.expect);
						  pid_calc(&pid_yaw_spd,(imu_data.gx)/16.4, pid_yaw.pos_out);
							Yaw_Current_Value = pid_yaw_spd.pos_out;
				
							pid_calc(&pid_pit, pit_get.total_angle, pit_set.expect);
						  pid_calc(&pid_pit_spd,(imu_data.gz)/16.4, pid_pit.pos_out);   
							Pitch_Current_Value = pid_pit_spd.pos_out; 
					
					    yaw_tly_set.expect = ptr_jy901_t_yaw.total_angle;   //在编码器模式下保存陀螺仪此时的值
			  }break;
				
			  case 3: //小陀螺模式（与陀螺仪模式相同，仅在底盘控制上有区别）  
			  {  
				      pid_calc(&pid_yaw, ptr_jy901_t_yaw.total_angle, yaw_tly_set.expect);  
							pid_calc(&pid_yaw_spd, (imu_data.gx)/16.4, pid_yaw.pos_out);
					    Yaw_Current_Value = pid_yaw_spd.pos_out;
					
	            pid_calc(&pid_pit, pit_get.total_angle, pit_set.expect);
							pid_calc(&pid_pit_spd, (imu_data.gz)/16.4, pid_pit.pos_out);          //(imu_data.gz)/16.4  为°/s
              Pitch_Current_Value = pid_pit_spd.pos_out; 
					
				      yaw_set.expect = yaw_get.total_angle;               //在陀螺仪模式下保存编码器此时的值
			  }break; 

				case 4: //自瞄模式
			  {  
				      pid_calc(&pid_yaw, yaw_get.total_angle, yaw_set.expect_pc);  
							pid_calc(&pid_yaw_spd, (imu_data.gx)/16.4, pid_yaw.pos_out);
					    Yaw_Current_Value = pid_yaw_spd.pos_out;
					
	            pid_calc(&pid_pit, pit_get.total_angle, pit_set.expect_pc);
							pid_calc(&pid_pit_spd, (imu_data.gz)/16.4, pid_pit.pos_out);          //(imu_data.gz)/16.4  为°/s
              Pitch_Current_Value = pid_pit_spd.pos_out; 
					
				      yaw_set.expect = yaw_get.total_angle;               //在自瞄模式下保存编码器此时的值
					    yaw_tly_set.expect = ptr_jy901_t_yaw.total_angle;   //在自瞄模式下保存陀螺仪此时的值
			  }break; 
				
				case 5:  //能量机关模式
				{
				}break;
				
				
				case 8:  //pit轴启动（必须先启动pit轴，在启动yaw轴）
				{
					    pid_calc(&pid_pit_start, pit_get.total_angle, 0);
						  pid_calc(&pid_pit_start_spd,(imu_data.gz)/16.4, pid_pit_start.pos_out);   
							Pitch_Current_Value = pid_pit_start_spd.pos_out;
				}break;
				
				case 9:  //yaw轴启动
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
				Cloud_Platform_Motor(&hcan2, Yaw_Current_Value, 0);        //YAW轴电机驱动
			  Cloud_Platform_Motor(&hcan1, 0, Pitch_Current_Value);      //PITCH轴电机驱动					
		 }
		 
		 osDelayUntil(&xLastWakeTime, GIMBAL_PERIOD);
  }
 
}


void Gimbal_angle_Conversion(moto_measure_t *ptr)      //云台角度转化，处理为绝对角度(-4096—4096)
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
