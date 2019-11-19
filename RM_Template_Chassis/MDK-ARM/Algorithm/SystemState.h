#ifndef __SysState_H__
#define __SysState_H__

#include "stm32f4xx_hal.h"
#include "FreeRTOS.h"
#include "cmsis_os.h"


#define OutLine_Time 100 //断线检测时间
#define Motor_Stall_Time 200
#define Motor_Stall_Spd 5
#define weidong_time 20  //子弹检测时间
#define shangdan_time  10000  //上弹时间


#define MyFlagSet(x,y) x=x|(0x00000001<<y) //设置标志位  y第几位
#define MyFlagClear(x,y) x=x&~(0x00000001<<y)
#define MyFlagGet(x,y) (x&(0x00000001<<y))

typedef struct{
	short Mode;//运行模式
	short Enable;//状态
	short State;//状态
	short Task;//任务
//	BEEPMode Beep;//蜂鸣器
	int Time;//System run time mm
	TIM_HandleTypeDef *htim;//时间计数器句柄
	uint16_t OutLine_Flag;//断线标志
	uint16_t task_OutLine_Flag;//断线标志	
//	RobotDistDef RobotDist;//机器人测量
}SystemStateDef;

typedef enum
{
		Motor1_NO,
	  Motor2_NO,
		Motor3_NO,
		Motor4_NO,
  
	
		DeviceTotal_No	
}DeviceX_NoDEF;

typedef enum
{
	testTask_ON,
	RemoteDataTask_ON,
	GimbalContrlTask_ON,
	GunTask_ON,
	LedTask_ON,
	vOutLineCheckTask_ON,
	
	TASKTotal_No	
}TASK_NoDEF;

extern SystemStateDef SystemState;
extern SystemStateDef Remote;
extern SystemStateDef Gimbal_Motor;
extern SystemStateDef JY61;
extern uint8_t xianwei_flg;

int SystemState_Inite(void);//SystemState初始化
void RefreshSysTime(void);//刷新系统时间（mm）
float GetSystemTimer(void);//获取系统当前准确时间


void OutLine_Check(void);//断线检测检测
void TASK_Check(void);//任务检测
void RefreshDeviceOutLineTime(DeviceX_NoDEF DevX_No);//刷新外设通信时间时间数组
void RefreshTaskOutLineTime(TASK_NoDEF Task_No);


void vOutLineCheck_Task(void const *argument);





#endif
