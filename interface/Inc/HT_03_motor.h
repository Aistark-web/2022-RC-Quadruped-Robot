#ifndef __HT_03_MOTOR_H_
#define __HT_03_MOTOR_H_

#ifdef __cpluscplus
extern "C"{
#endif

#include "HT_03_motor_types.h"
#include "math_ops.h"

/**
 * @brief 解包电机数据
 * 
 * @param[in] data 电机反馈原始数据
 * @param[out] motor 电机句柄
 */
void HT_03_Motor_Data_Unpack(uint8_t *data,HT_03_MOTOR_Handle *motor);

/**
 * @brief 封包电机数据
 * 
 * @param[in] motor 电机句柄
 * @param[out] data 电机期望数据
 */
void HT_03_Motor_Data_packet(uint8_t *data,HT_03_MOTOR_Handle *motor);

/**
 * @brief 获取初始电机位置
 * 
 * @param[in] data 电机期望数据
 * @param[out] motor 电机句柄
 */
void HT_03_Motor_Get_init_positon(uint8_t *data,HT_03_MOTOR_Handle *motor);

/**
	* @brief 初始化腿部电机ID
	* @param[in] leg_motor_id 腿部电机内部ID
	* @param[int] can_slave_id CAN ID
	*/
void HT_03_Motor_Init_ID(HT_03_MOTOR_Handle *motor,Leg_Motor_enum leg_motor_id,uint8_t can_slave_id);
#ifdef __cpluscplus
}
#endif

#endif