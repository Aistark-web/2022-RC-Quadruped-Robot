#ifndef __LEG_H_
#define __LEG_H_

#ifdef __cpluscplus
extern "C"{
#endif

#include "stdint.h"
#include "HT_03_motor.h"
#include "leg_types.h"
#include "ramp.h"

/**
 * @brief 初始化腿部信息
 * 
 * @param Leg 腿句柄
 * @param Motor1 腿所关联电机1
 * @param Motor2 腿所关联电机2
 * @param LEG_ID 腿ID
 * @param default_theta1 theta1初始位置
 * @param default_tehat2 theta2初始位置
 * @param L1    //大腿
 * @param L2    //小腿
 */
void Leg_binding(Leg_Handle *Leg,HT_03_MOTOR_Handle *Motor1,HT_03_MOTOR_Handle *Motor2,LEG_ID_ENUM Leg_ID);

/**
 * @brief 解析腿部信息
 * 
 * @param Leg 腿句柄
 * @param L1 腿长L1
 * @param L2 腿长L2
 */

void Leg_get(Leg_Handle *Leg);

/**
	* @brief 初始化腿部位置
	* 
	*/
uint8_t Leg_Move_Init(Leg_Handle *Leg,Ramp_Typedef *ramp);

/**
	* @brief 回归到刚启动的原点(内部已调用了Leg_Angle_Control)
	*/
uint8_t Leg_Reset_Ramp(Leg_Handle *Leg,Ramp_Typedef *Ramp);

/**
	* @brief 移动至关机缓冲点
	*/
uint8_t Leg_Shutdown_Ramp(Leg_Handle *Leg,Ramp_Typedef *Ramp);
/**
	* @brief 运动到固定点
	*/
void Leg_Point_RTO(float y,float z,Leg_Handle *Leg);
/**
	* @brief 运动到固定点（斜坡运动)
	*/
void Leg_Point_RTO_Ramp(float y,float z,Leg_Handle *Leg,Ramp_Typedef *Ramp);

/**
	* @brief 控制角度获取
	*/
void Leg_Angle_Control(Leg_Handle *Leg);

/**
	*	@brief 行走
	*/
void Leg_New_Walk(float s,float h,Leg_Handle *Leg,Ramp_Typedef *Ramp);
#ifdef __cpluscplus
}
#endif

#endif