/* ����ͷ�ļ�----------------------------------------------------------------*/
#include "chassis_control.h"
#include "SystemState.h"
#include "arm_math.h"

/* �ڲ�����------------------------------------------------------------------*/
pid_t pid_3508_pos;     		 //���̵��λ�û�
pid_t pid_3508_spd[4];			 //���̵���ٶȻ�
pid_t pid_3508_current[4];	 //���̵����������	
pid_t pid_chassis_follow = {0};//���̸���λ�û�

moto3508_type  moto_3508_set = {.flag = 0}; 
Chassis_Status_t chassis_status;

static float Current_set[4] = {0};  //���ݸ��������ƵĻ���
extern float Total_power;
float total_set;
//���Ա���
int16_t angle[2];
uint32_t total_current;

/* �ڲ�����ԭ������----------------------------------------------------------*/
void Chassis_pid_init(void)
{
	
	 PID_struct_init(&pid_3508_pos, POSITION_PID, 10000, 1000, 1.5f,	0.01f,	2.0f);  // motos angular rate closeloop.pid:1.5,0.0,20.0
//	 pid_3508_pos.deadband = 150;
	
	 PID_struct_init(&pid_chassis_follow, POSITION_PID,10000,1000, 40.0f, 0.31f , 0.0f);
//	 pid_chassis_follow.deadband = 500;

	
	 for(int i=0; i<4; i++)
	 { 
		 PID_struct_init(&pid_3508_spd[i], POSITION_PID, 10000, 2000, 1.5f,	0.1f,	0.1f);  //4 motos angular rate closeloop.
   }
	
		PID_struct_init(&pid_3508_current[0], POSITION_PID, 6000, 500, 0.6f, 0.01f, 0.01f);  //4 motos angular rate closeloop.
		PID_struct_init(&pid_3508_current[1], POSITION_PID, 6000, 500, 0.6f, 0.01f,	0.01f);  //4 motos angular rate closeloop.
		PID_struct_init(&pid_3508_current[2], POSITION_PID, 6000, 500, 0.6f, 0.01f, 0.01f);  //4 motos angular rate closeloop.
		PID_struct_init(&pid_3508_current[3], POSITION_PID, 6000, 500, 0.6f, 0.01f,	0.01f);  //4 motos angular rate closeloop.
}
/* �������岿�� -------------------------------------------------------------*/

/***************************************************************************************
**
	*	@brief	Chassis_Contrl_Task(void const * argument)
	*	@param
	*	@supplement	���̿�������
	*	@retval	
****************************************************************************************/
void Chassis_Contrl_Task(void const * argument)
{
	static float  wheel[4] = {0,0,0,0};
	static int32_t Angle_err;
	static float absolute_angle_degree;
	static int16_t Speed_W;
  static int16_t Speed_X;
  static int16_t Speed_Y;
	float pSinVal;
	float pCosVal;

	osDelay(200);//��ʱ200ms
	portTickType xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();
	
	Chassis_pid_init();
	
	for(;;)
	{
		 chassis_status.chassis_mode = gimbal_status.gimbal_mode;
		
		 Angle_err = Middle_Angle - moto_gimbal_get.angle;
		 absolute_angle_degree = Absolute_angle_Conversion(Angle_err);   //ת���Ƕ�
		
		switch(chassis_status.chassis_mode)
		{
			case 1:		//���̸���
			{
			   if(ABS(absolute_angle_degree) < 3.3f)  //�ṩһ��5��ĽǶ�����
		     {
			      Speed_W = 0;
		     }else
		     {
			      Speed_W = pid_calc(&pid_chassis_follow, absolute_angle_degree, 0);
		     }
				 motor_move_setvmmps(wheel, moto_3508_set.dstVmmps_X, moto_3508_set.dstVmmps_Y, Speed_W);
	    }break;
			
			case 2:    //����
			{
				 Speed_W = 0;
				 motor_move_setvmmps(wheel, moto_3508_set.dstVmmps_X, moto_3508_set.dstVmmps_Y, Speed_W);
			}break;
			
			case 3:   //С����
			{
				 Speed_W = 3000;
				 arm_sin_cos_f32(absolute_angle_degree, &pSinVal, &pCosVal);
				 Speed_X = moto_3508_set.dstVmmps_X * pCosVal - moto_3508_set.dstVmmps_Y * pSinVal;
         Speed_Y = moto_3508_set.dstVmmps_X * pSinVal + moto_3508_set.dstVmmps_Y * pCosVal;
				
				 motor_move_setvmmps(wheel, Speed_X, Speed_Y, Speed_W);
			}break;
			
			case 4:    //����
			{
				 Speed_W = 0;
				 motor_move_setvmmps(wheel, moto_3508_set.dstVmmps_X, moto_3508_set.dstVmmps_Y, Speed_W);
			}break;
			
			default:break;
			
	  }
		
//		  motor_move_setvmmps(wheel, moto_3508_set.dstVmmps_X, moto_3508_set.dstVmmps_Y, moto_3508_set.dstVmmps_W);
			
	   for(int i = 0; i < 4; i++)
		 {		
			  pid_calc(&pid_3508_spd[i], moto_chassis_get[i].speed_rpm, wheel[i]);
		 }
		 
//		 total_current = ABS(moto_chassis_get[0].real_current) + ABS(moto_chassis_get[1].real_current) +
//		                   ABS(moto_chassis_get[2].real_current) + ABS(moto_chassis_get[3].real_current);
//		 
		 total_set = ABS(pid_3508_spd[0].pos_out) + ABS(pid_3508_spd[1].pos_out) + 
		 ABS(pid_3508_spd[2].pos_out) + ABS(pid_3508_spd[3].pos_out);
		 
//		 if(Total_power > 70)
//		 {
//			 pid_3508_spd[0].pos_out = (pid_3508_spd[0].pos_out / (total_set + 1.0f)) * 8000; 
//			 pid_3508_spd[1].pos_out = (pid_3508_spd[1].pos_out / (total_set + 1.0f)) * 8000; 
//			 pid_3508_spd[2].pos_out = (pid_3508_spd[2].pos_out / (total_set + 1.0f)) * 8000; 
//			 pid_3508_spd[3].pos_out = (pid_3508_spd[3].pos_out / (total_set + 1.0f)) * 8000; 
//		 }
		 
		 
		 if(gimbal_status.remote_mode == 1)
		 {
			 Chassis_Motor(&hcan1, 0, 0, 0, 0);
		 }else
		 {			 
			 Chassis_Motor(&hcan1,
													pid_3508_spd[0].pos_out,
													pid_3508_spd[1].pos_out, 
													pid_3508_spd[2].pos_out, 
													pid_3508_spd[3].pos_out);	
		 }			 
		
		 osDelayUntil(&xLastWakeTime, CHASSIS_PERIOD);
	}
}


float Absolute_angle_Conversion(int32_t angle)      //ת��Ϊ���ԽǶ�,��ת����λ������е�Ƕ�ת����-180�㡪180�㣩
{
	  float absolute_angle;
	  while(1)
		{
			 if(angle < -4096)
			 {
				  angle += 8192;
			 }else if(angle > 4096)
			 {
				  angle -= 8192;
			 }else
			 {
				  break;
			 }
		}
		absolute_angle = (float)angle * 360 / 8192;
		return absolute_angle;
}
