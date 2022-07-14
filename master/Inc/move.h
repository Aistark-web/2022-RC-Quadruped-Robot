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
	Robot_Reset_Move,									//��λ
	Robot_Step_Move,									//̤��
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

typedef enum{
	Double_Bridge_None_Event,					//���¼�����
	Double_Bridge_Front_Rise_Event,		//ǰ��̧���¼�
	Double_Bridge_Back_Rise_Event,		//����̧���¼�
	Double_Bridge_Front_Down_Event,		//ǰ������¼�
	Double_Bridge_Back_Down_Event,		//��������¼�
	Double_Bridge_Straignt_Event,			//˫ľ���������¼�
	Double_Bridge_Turn_Event,					//˫Ŀ���Ϲ����¼�
	Double_Bridge_Circle_Event				//˫ľ������ת�¼�
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
	Seesaw_Climb_Up_State,						//����״̬
	Seesaw_Climb_Down_State						//����״̬
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
	uint8_t Change_Mode;													//�Ƿ��л�ģʽ
	uint8_t can_list;															//can���Ͷ������
	uint8_t Torque_Control;												//�Ƿ��������������(���ر�PID����ϵͳ)
	Robot_Mode_t Mode;														//ģʽ
	Robot_Move_t Move;														//�˶���Ϊ
	Robot_Move_State_t State;											//״̬
	Double_Bridge_Event_t double_bridge_event;		//˫ľ���¼�
	Double_Bridge_State_t double_bridge_state;		//˫ľ��״̬
	Seesaw_Event_t seesaw_event;									//���ΰ��¼�
	Seesaw_Climb_State_t seesaw_climb_state;			//���ΰ�����״̬
	Seesaw_Sqat_State_t	seesaw_sqat_state;				//���ΰ��������״̬
	Stair_Event_t stair_event;										//�����¼�
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

/**
	* @brief ˫ľ��
	*/
void Robot_Move_Double_bridge(Robot_Handle *Robot,Leg_Handle *Leg,Dev_Handle *IMU);

/**
	* @brief ���ΰ�
	*/

void Robot_Move_Seesaw(Robot_Handle *Robot,Leg_Handle *Leg,Dev_Handle *IMU);

/**
	* @brief ����
	*/
void Robot_Move_Stair(Robot_Handle *Robot,Leg_Handle *Leg,Dev_Handle *IMU);
#endif
