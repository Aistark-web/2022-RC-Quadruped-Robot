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
 * @brief ��ʼ���Ȳ���Ϣ
 * 
 * @param Leg �Ⱦ��
 * @param Motor1 �����������1
 * @param Motor2 �����������2
 * @param LEG_ID ��ID
 * @param default_theta1 theta1��ʼλ��
 * @param default_tehat2 theta2��ʼλ��
 * @param L1    //����
 * @param L2    //С��
 */
void Leg_binding(Leg_Handle *Leg,HT_03_MOTOR_Handle *Motor1,HT_03_MOTOR_Handle *Motor2,LEG_ID_ENUM Leg_ID);

/**
 * @brief �����Ȳ���Ϣ
 * 
 * @param Leg �Ⱦ��
 * @param L1 �ȳ�L1
 * @param L2 �ȳ�L2
 */

void Leg_get(Leg_Handle *Leg);

/**
	* @brief ��ʼ���Ȳ�λ��
	* 
	*/
uint8_t Leg_Move_Init(Leg_Handle *Leg,Ramp_Typedef *ramp);

/**
	* @brief �ع鵽��������ԭ��(�ڲ��ѵ�����Leg_Angle_Control)
	*/
uint8_t Leg_Reset_Ramp(Leg_Handle *Leg,Ramp_Typedef *Ramp);

/**
	* @brief �ƶ����ػ������
	*/
uint8_t Leg_Shutdown_Ramp(Leg_Handle *Leg,Ramp_Typedef *Ramp);
/**
	* @brief �˶����̶���
	*/
void Leg_Point_RTO(float y,float z,Leg_Handle *Leg);
/**
	* @brief �˶����̶��㣨б���˶�)
	*/
void Leg_Point_RTO_Ramp(float y,float z,Leg_Handle *Leg,Ramp_Typedef *Ramp);

/**
	* @brief ���ƽǶȻ�ȡ
	*/
void Leg_Angle_Control(Leg_Handle *Leg);

/**
	*	@brief ����
	*/
void Leg_New_Walk(float s,float h,Leg_Handle *Leg,Ramp_Typedef *Ramp);
#ifdef __cpluscplus
}
#endif

#endif