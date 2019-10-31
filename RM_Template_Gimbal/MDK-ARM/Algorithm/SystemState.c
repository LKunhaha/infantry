#include "SystemState.h"
#include "tim.h"
#include "gimbal_control.h"
/*   by  donglin   */


SystemStateDef SystemState = {0};
float g_TimePer[100] = {0};
float g_Time_DeviceOutLine[DeviceTotal_No] = {0};//�������һ��ͨ��ʱ������
float g_Time_TASKOutLine[TASKTotal_No] = {0};//�������һ��ͨ��ʱ������
static float Time;
static float Shangdan_time;

//���߼����
void OutLine_Check()
{
	short num = 0;//��ʱ�����ۼ���
	float time = GetSystemTimer();//��ǰϵͳʱ��

	for(num = 0; num < DeviceTotal_No; num++)
	{
		if(time - g_Time_DeviceOutLine[num] > OutLine_Time)
		{
			MyFlagSet(SystemState.OutLine_Flag,(num));//���ö��߱�־
		}
		else
		{
			MyFlagClear(SystemState.OutLine_Flag,(num));//������߱�־
		}
	}
}

//���߼����
void TASK_Check()
{
	short num = 0;//��ʱ�����ۼ���
	float time = GetSystemTimer();//��ǰϵͳʱ��

	for(num = 0; num < TASKTotal_No; num++)
	{
		if(time - g_Time_TASKOutLine[num] > OutLine_Time)
		{
			MyFlagSet(SystemState.task_OutLine_Flag,(num));//���ö��߱�־
		}
		else
		{
			MyFlagClear(SystemState.task_OutLine_Flag,(num));//������߱�־
		}
	}
}


//=====================================================
//							  �ڲ�����
//
//====================================================





int SystemState_Inite()
{
	int state;
	SystemState.Enable = 0;
	SystemState.State = 0;
	SystemState.Task = 0;
	SystemState.Time = 0;
	SystemState.htim = &htim6;//��ʱ�����趨 ÿ 10us ��һ����  ����ֵΪ 100-1 (1ms)  ���� Timer3 ��Ƶ168M Ԥ��Ƶ (840-1) ����ֵ (100-1)
//	state=HAL_TIM_Base_Start_IT(SystemState.htim);//����ʱ�������
  return state;
}

//�ж�ˢ���е��� ����ϵͳʱ�� ms
 void RefreshSysTime(void)
{
		SystemState.Time += 1;
}


//���ϵͳʱ��
inline float GetSystemTimer()
{
	return SystemState.htim->Instance->CNT / 100.0 + SystemState.Time;   //�����㣬�õ�������ʱ�䵥λΪms
}


//ˢ������ͨ��ʱ��ʱ������
void RefreshDeviceOutLineTime(DeviceX_NoDEF DevX_No)
{
	
	g_Time_DeviceOutLine[DevX_No] = GetSystemTimer();
	
}



//ˢ������ʱ������
void RefreshTaskOutLineTime(TASK_NoDEF Task_No)
{
	
	g_Time_TASKOutLine[Task_No] = GetSystemTimer();
	
}

void xianwei_check()     //��λ��⣨���ڵ��߼�������
{
		float time = GetSystemTimer();//��ǰϵͳʱ��
  
	  if(xianwei_flg)   //��λ��־
		{
        Shoot.shoot_flg = 1;                  // ѹ��λ����  Ϊ�ߵ�ƽ 
        Shangdan_time = GetSystemTimer();
		}else
		{
			  Shoot.shoot_flg = 0;                  //δѹ��λ����  ��  ����λ����  
		}
		
		if(Shoot.last_shoot_flg != Shoot.shoot_flg) 
		{
				Shoot.flag = 1;           //�����   δѹ��λ���� �� ѹ��λ����   ��   ѹ��λ���� �� ����λ����
				Shoot.cnt += 1;            //����һ�ŵ����2(���ε�ƽ�仯)
				
		}else  Shoot.flag = 0;         //����λ���ޱ仯
					        
    if(time - Shangdan_time > shangdan_time  )//�ϵ���⣨δѹ��λ���س���ʱ�����10��ͽ����ϵ�ģʽ��
		{
			  Shoot.flag = 2;
		}
     
    
    Shoot.last_shoot_flg = Shoot.shoot_flg;
}
