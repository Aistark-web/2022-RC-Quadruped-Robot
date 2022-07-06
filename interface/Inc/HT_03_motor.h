#ifndef __HT_03_MOTOR_H_
#define __HT_03_MOTOR_H_

#ifdef __cpluscplus
extern "C"{
#endif

#include "HT_03_motor_types.h"
#include "math_ops.h"

/**
 * @brief ����������
 * 
 * @param[in] data �������ԭʼ����
 * @param[out] motor ������
 */
void HT_03_Motor_Data_Unpack(uint8_t *data,HT_03_MOTOR_Handle *motor);

/**
 * @brief ����������
 * 
 * @param[in] motor ������
 * @param[out] data �����������
 */
void HT_03_Motor_Data_packet(uint8_t *data,HT_03_MOTOR_Handle *motor);

/**
 * @brief ��ȡ��ʼ���λ��
 * 
 * @param[in] data �����������
 * @param[out] motor ������
 */
void HT_03_Motor_Get_init_positon(uint8_t *data,HT_03_MOTOR_Handle *motor);

/**
	* @brief ��ʼ���Ȳ����ID
	* @param[in] leg_motor_id �Ȳ�����ڲ�ID
	* @param[int] can_slave_id CAN ID
	*/
void HT_03_Motor_Init_ID(HT_03_MOTOR_Handle *motor,Leg_Motor_enum leg_motor_id,uint8_t can_slave_id);
#ifdef __cpluscplus
}
#endif

#endif