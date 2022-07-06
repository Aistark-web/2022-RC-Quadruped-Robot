#ifndef __HT_03_MOTOR_TYPES_H_
#define __HT_03_MOTOR_TYPES_H_

#ifdef __cpluscplus
extern "C"{
#endif

#include "stdint.h"


/**
 * @brief ��ǰ���״̬
 * 
 */
typedef struct
{
    float C_P;                  //���λ��      ��λ: rad
    float C_V;                  //����ٶ�      ��λ: rad/s
    float C_T;                  //���Ť��      ��λ: N��m
}HT_03_motor_Current_Handle;

/**
 * @brief ������Ϣ              
 * 
 */
typedef struct
{
    float E_P;                  //����λ��      ��λ: rad
    float E_V;                  //�����ٶ�      ��λ: rad/s
    float E_T;                  //����Ť��      ��λ: N��m
    float KP;                   //λ������
    float KD;                   //�ٶ�����
}HT_03_motor_Expect_Handle;

typedef enum{
	Leg_Motor_External_ID,
	Leg_Motor_Internal_ID
}Leg_Motor_enum;

typedef struct
{
		Leg_Motor_enum leg_motor_id;						//�����Ȳ�λ��ID
    uint8_t can_slave_id;                   //������� CAN_ID
		uint8_t first_state;
    HT_03_motor_Current_Handle Current;     //��ǰ
    HT_03_motor_Expect_Handle  Expect;			//����
		float Init_Position;										//��ʼλ��
		float Source_Positon;										//���ʵ��λ��
		float Source_EP;
}HT_03_MOTOR_Handle;


#ifdef __cpluscplus
}
#endif

#endif