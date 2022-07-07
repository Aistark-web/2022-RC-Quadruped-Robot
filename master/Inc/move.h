#ifndef __MOVE_H_
#define __MOVE_H_

#include "stdint.h"
#include "stm32f4xx_hal.h"
#include "leg.h"
#include "remote.h"
#include "WTGAHRS1.h"

typedef enum{
	Robot_Init_Mode,									//��ʼ��ģʽ
	Robot_Open_Motor_Mode,						//�������ģʽ
	Robot_Start_Mode,									//����ģʽ
	Robot_Move_Mode,									//�˶�ģʽ
	Robot_Shutdown_Mode								//�ػ�ģʽ
}Robot_Mode_t;

typedef enum{
	Robot_Stop_Move,									//ֹͣ�ж�
	Robot_Step_Move,
	Robot_Straignt_Move,							//ֱ��
	Robot_Turn_Move,									//����
	Robot_Circle_Move,								//ԭ����ת
	Robot_Climb_Move,									//����
	Robot_Jump_Move,									//��Ծ
	Robot_Double_bridge_Move,					//˫ľ����Ϊ(�����������Ϊ)
	Robot_Seesaw_Move,								//���ΰ���Ϊ(�����������Ϊ)
	Robot_Stair_Move									//������Ϊ(���۶������Ϊ)
}Robot_Move_t;											//�˶�ģʽ�����˶���Ϊ

typedef enum{
	Robot_Stop_State,									//��ֹ״̬
	Robot_Running_State								//���������еĲ��ɴ��
}Robot_Move_State_t;

typedef struct{
	uint8_t Open_Motor_State;					
	uint8_t Change_Mode;							//�Ƿ��л�ģʽ
	uint8_t can_list;									//can���Ͷ������
	Robot_Mode_t Mode;								//ģʽ
	Robot_Move_t Move;								//�˶���Ϊ
	Robot_Move_State_t State;					//״̬
}Robot_Handle;

/**
	* @brief ��ȡң������ֵ���ݵ�ǰ����ģʽ��������״̬��ȷ���Ƿ���Ҫ�л�״̬
	*/
void Robot_Check(Robot_Handle *Robot,Remote_DataPack_Handle *Remote);

/**
	* @brief �˶���Ϊ��λ�󻹻����һ��״̬���
	*/
void Robot_Move_State_Reset_Stop(Robot_Handle *Robot);

/**
	* @brief �˶�����
	*/
void Robot_Move_Master(Robot_Handle *Robot);

/**
	* @brief ֱ��
	*/
void Robot_Move_Straight(Robot_Handle *Robot,Leg_Handle *Leg,Ramp_Typedef *Ramp,Dev_Handle *IMU);

/**
	* @brief ̤��
	*/
void Robot_Move_Step(Robot_Handle *Robot,Leg_Handle *Leg,Ramp_Typedef *Ramp,Dev_Handle *IMU);
/**
	* @brief ����
	*/
void Robot_Move_Turn(Robot_Handle *Robot,Leg_Handle *Leg,Ramp_Typedef *Ramp,Dev_Handle *IMU);

/**
	* @brief ԭ����ת
	*/
void Robot_Move_Circle(Robot_Handle *Robot,Leg_Handle *Leg,Ramp_Typedef *Ramp,Dev_Handle *IMU);

///**
//	* @brief ������
//	*/
//void Robot_Move_Climb(Robot_Handle *Robot,Leg_Handle *Leg,Ramp_Typedef *Ramp,Dev_Handle *IMU);

/**
	* @brief ��Ծ
	*/
void Robot_Move_Jump(Robot_Handle *Robot,Leg_Handle *Leg,Dev_Handle *IMU);

///**  
//	* @brief ����
//	*/
//void Robot_Move_Sqat(Robot_Handle *Robot,Leg_Handle *Leg,Ramp_Typedef *Ramp,Dev_Handle *IMU);

///**
//	* @brief ����
//	*/
//void Robot_Move_Rise(Robot_Handle *Robot,Leg_Handle *Leg,Ramp_Typedef *Ramp,Dev_Handle *IMU);
#endif
