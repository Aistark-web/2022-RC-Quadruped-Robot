#ifndef __MOTOR_DRIVER_H_
#define __MOTOR_DRIVER_H_

#include "stm32f4xx_hal.h"
#include "string.h"
#include "HT_03_motor.h"
#define Motor_Driver_get_Len 	33				//发送给驱动数据长度
#define Motor_Driver_return_Len 25			//驱动返回数据长度
#if defined (__CC_ARM)								//ARM_Compiler v4/5
#pragma anon_unions
#endif

#define CAN_ID_Base_Mask			1
#define CAN_ID_1_MASK					(1<<0)
#define CAN_ID_2_MASK					(1<<1)
#define CAN_ID_3_MASK					(1<<2)
#define CAN_ID_4_MASK					(1<<3)
#define CAN_ID_5_MASK					(1<<4)
#define CAN_ID_6_MASK					(1<<5)
#define CAN_ID_7_MASK					(1<<6)
#define CAN_ID_8_MASK					(1<<7)

#define DEV1_PIN_MASK					(CAN_ID_1_MASK | CAN_ID_2_MASK | CAN_ID_3_MASK | CAN_ID_4_MASK)
#define DEV2_PIN_MASK					(CAN_ID_5_MASK | CAN_ID_6_MASK | CAN_ID_7_MASK | CAN_ID_8_MASK)

#define CAN_ID_MASK_OFFSET(OFFSET)  (CAN_ID_Base_Mask << (OFFSET))

#pragma pack(1)
/**
	* @breif 发送给下位机数据句柄
	*/
typedef struct Driver_get_handle{
	union {
		uint8_t data[33];
		struct{
			uint8_t Dev_PIN;							//驱动设备识别码
			uint8_t CAN_data[4][8];		
		}driver_data;
	};
}Driver_get_Handle;

/**
	* @brief 接收下位机返回数据句柄 
	*/
typedef struct Driver_return_handle{
	union{
		uint8_t data[25];
		struct{
			uint8_t Dev_PIN;							//驱动设备识别码
			uint8_t CAN_data[4][6];				
		}driver_data;
	};
}Driver_return_Handle;
#pragma pack()

/**
	* @brief 封包发送给下位机数据
	*/
void Driver_Data_Packet(Driver_get_Handle *driver,HT_03_MOTOR_Handle *motor_array);

/**
	* @brief 解包下位机返回数据
	*/
void Driver_Data_Unpack(Driver_return_Handle *driver,HT_03_MOTOR_Handle *motor_array);
#endif