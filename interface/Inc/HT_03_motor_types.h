#ifndef __HT_03_MOTOR_TYPES_H_
#define __HT_03_MOTOR_TYPES_H_

#ifdef __cpluscplus
extern "C"{
#endif

#include "stdint.h"


/**
 * @brief 当前电机状态
 * 
 */
typedef struct
{
    float C_P;                  //电机位置      单位: rad
    float C_V;                  //电机速度      单位: rad/s
    float C_T;                  //电机扭矩      单位: N·m
}HT_03_motor_Current_Handle;

/**
 * @brief 期望信息              
 * 
 */
typedef struct
{
    float E_P;                  //期望位置      单位: rad
    float E_V;                  //期望速度      单位: rad/s
    float E_T;                  //期望扭矩      单位: N·m
    float KP;                   //位置增益
    float KD;                   //速度增益
}HT_03_motor_Expect_Handle;

typedef enum{
	Leg_Motor_External_ID,
	Leg_Motor_Internal_ID
}Leg_Motor_enum;

typedef struct
{
		Leg_Motor_enum leg_motor_id;						//所属腿部位置ID
    uint8_t can_slave_id;                   //电机接收 CAN_ID
		uint8_t first_state;
    HT_03_motor_Current_Handle Current;     //当前
    HT_03_motor_Expect_Handle  Expect;			//期望
		float Init_Position;										//初始位置
		float Source_Positon;										//电机实际位置
		float Source_EP;
}HT_03_MOTOR_Handle;


#ifdef __cpluscplus
}
#endif

#endif