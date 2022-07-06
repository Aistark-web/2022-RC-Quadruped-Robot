#ifndef __REMOTE_H_
#define __REMOTE_H_

#ifdef __cpluscplus
extern "C"{
#endif

#include "stdint.h"
#include "string.h"
#include "CRC.h"
/**
 * @brief 按键及拨杆数据句柄
 * 
 */
typedef struct
{
  uint16_t Left_Key_Up : 1;
  uint16_t Left_Key_Down : 1;
  uint16_t Left_Key_Left : 1;
  uint16_t Left_Key_Right : 1;
  uint16_t Left_Rocker : 1;
  uint16_t Left_Encoder : 1;
	uint16_t Left_Switch_Up: 1;
  uint16_t Left_Switch_Down : 1;
  uint16_t Right_Key_Up : 1;
  uint16_t Right_Key_Down : 1;
  uint16_t Right_Key_Left : 1;
  uint16_t Right_Key_Right : 1;
  uint16_t Right_Rocker : 1;
  uint16_t Right_Encoder : 1;
  uint16_t Right_Switch_Up : 1;
  uint16_t Right_Switch_Down : 1;
} hw_key_t;

#pragma pack(1)

/**
 * @brief 遥控器句柄
 * 
 */
typedef struct {
  uint8_t head;
  uint16_t L_X_rocker;
	uint16_t L_Y_rocker;
	uint16_t R_X_rocker;
	uint16_t R_Y_rocker;
  hw_key_t Key;
  int32_t Left_Encoder;
  int32_t Right_Encoder;
  uint16_t crc;
} Remote_DataPack_Handle;

#pragma pack()

/**
 * @brief 处理遥控器数据
 * 
 * @param[in] Remote_Receive_Data 遥控器原始数据
 * @param[out] DataPack 遥控器数据
 */
void Remote_Deal(uint8_t *Remote_Receive_Data,Remote_DataPack_Handle *DataPack);

uint8_t Remote_Vertify(uint8_t *Remote_Receive_Data,uint8_t check_len);
#ifdef __cpluscplus
}
#endif

#endif