#ifndef __MOVE_H_
#define __MOVE_H_

#include "stdint.h"
#include "stm32f4xx_hal.h"
#include "leg.h"
#include "remote.h"
#include "WTGAHRS1.h"

typedef enum{
	Robot_Init_Mode,									//初始化模式
	Robot_Open_Motor_Mode,						//开启电机模式
	Robot_Start_Mode,									//启动模式
	Robot_Move_Mode,									//运动模式
	Robot_Shutdown_Mode								//关机模式
}Robot_Mode_t;

typedef enum{
	Robot_Stop_Move,									//停止行动
	Robot_Reset_Move,									//归位
	Robot_Step_Move,									//踏步
	Robot_Straignt_Move,							//直行
	Robot_Turn_Move,									//拐弯
	Robot_Circle_Move,								//原地自转
	Robot_Climb_Move,									//攀爬
	Robot_Jump_Move,									//跳跃
	Robot_Double_bridge_Move,					//双木桥行为(包含多个子行为)
	Robot_Seesaw_Move,								//跷跷板行为(包括多个子行为)
	Robot_Stair_Move									//阶梯行为(包扣多个子行为)
}Robot_Move_t;											//运动模式各个运动行为

typedef enum{
	Robot_Stop_State,									//静止状态
	Robot_Running_State								//正在运行中的不可打断
}Robot_Move_State_t;

typedef enum{
	Double_Bridge_None_Event,					//无事件产生
	Double_Bridge_Front_Rise_Event,		//前腿抬脚事件
	Double_Bridge_Back_Rise_Event,		//后腿抬脚事件
	Double_Bridge_Front_Down_Event,		//前腿落脚事件
	Double_Bridge_Back_Down_Event,		//后退落脚事件
	Double_Bridge_Straignt_Event,			//双木桥上行走事件
	Double_Bridge_Turn_Event,					//双目桥上拐弯事件
	Double_Bridge_Circle_Event				//双木桥上自转事件
}Double_Bridge_Event_t;

typedef enum{
	Double_Bridge_None_State,
	Double_Bridge_Front_Rise_State,
	Double_Bridge_Back_Rise_State,
	Double_Bridge_Front_Down_State,		
	Double_Bridge_Back_Down_State
}Double_Bridge_State_t;

typedef enum{
	Seesaw_None_Event,
	Seesaw_Sqat_Event,
	Seesaw_Stand_Event,
	Seesaw_Climb_Up_Event,
	Seesaw_Climb_Down_Event,
	Seesaw_Straight_Event,
	Seesaw_Turn_Event,
	Seesaw_Circle_Event
}Seesaw_Event_t;

typedef enum{
	Seesaw_Climb_None_State,
	Seesaw_Climb_Up_State,						//上坡状态
	Seesaw_Climb_Down_State						//下坡状态
}Seesaw_Climb_State_t;

typedef enum{
	Seesaw_Stand_State,
	Seesaw_Sqat_State
}Seesaw_Sqat_State_t;

typedef enum{
	Stair_None_Evnet
}Stair_Event_t;

typedef struct{
	uint8_t Open_Motor_State;					
	uint8_t Change_Mode;													//是否切换模式
	uint8_t can_list;															//can发送队列序号
	uint8_t Torque_Control;												//是否开启力矩增益控制(即关闭PID控制系统)
	Robot_Mode_t Mode;														//模式
	Robot_Move_t Move;														//运动行为
	Robot_Move_State_t State;											//状态
	Double_Bridge_Event_t double_bridge_event;		//双木桥事件
	Double_Bridge_State_t double_bridge_state;		//双木桥状态
	Seesaw_Event_t seesaw_event;									//跷跷板事件
	Seesaw_Climb_State_t seesaw_climb_state;			//跷跷板攀爬状态
	Seesaw_Sqat_State_t	seesaw_sqat_state;				//跷跷板蹲下起身状态
	Stair_Event_t stair_event;										//阶梯事件
}Robot_Handle;


/**
	* @brief 读取遥控器数值根据当前运行模式及其运行状态等确认是否需要切换状态
	*/
void Robot_Check(Robot_Handle *Robot,Remote_DataPack_Handle *Remote);

/**
	* @brief 运动行为归位后还会进行一次状态检测
	*/
void Robot_Move_State_Reset_Stop(Robot_Handle *Robot);

/**
	* @brief 运动管理
	*/
void Robot_Move_Master(Robot_Handle *Robot);

/**
	* @brief 直走
	*/
void Robot_Move_Straight(Robot_Handle *Robot,Leg_Handle *Leg,Ramp_Typedef *Ramp,Dev_Handle *IMU);

/**
	* @brief 踏步
	*/
void Robot_Move_Step(Robot_Handle *Robot,Leg_Handle *Leg,Ramp_Typedef *Ramp,Dev_Handle *IMU);
/**
	* @brief 拐弯
	*/
void Robot_Move_Turn(Robot_Handle *Robot,Leg_Handle *Leg,Ramp_Typedef *Ramp,Dev_Handle *IMU);

/**
	* @brief 原地自转
	*/
void Robot_Move_Circle(Robot_Handle *Robot,Leg_Handle *Leg,Ramp_Typedef *Ramp,Dev_Handle *IMU);

///**
//	* @brief 上下坡
//	*/
//void Robot_Move_Climb(Robot_Handle *Robot,Leg_Handle *Leg,Ramp_Typedef *Ramp,Dev_Handle *IMU);

/**
	* @brief 跳跃
	*/
void Robot_Move_Jump(Robot_Handle *Robot,Leg_Handle *Leg,Dev_Handle *IMU);

/**
	* @brief 双木桥
	*/
void Robot_Move_Double_bridge(Robot_Handle *Robot,Leg_Handle *Leg,Dev_Handle *IMU);

/**
	* @brief 跷跷板
	*/

void Robot_Move_Seesaw(Robot_Handle *Robot,Leg_Handle *Leg,Dev_Handle *IMU);

/**
	* @brief 阶梯
	*/
void Robot_Move_Stair(Robot_Handle *Robot,Leg_Handle *Leg,Dev_Handle *IMU);
#endif
