#ifndef __MASTER_H_
#define __MASTER_H_

#include "stm32f4xx_hal.h"
#include "HT_03_motor.h"
#include "stdlib.h"
#include "leg.h"
#include "ramp.h"
#include "remote.h"
#include "WTGAHRS1.h"
#include "PID.h"
#include "can_filter.h"
#include "move.h"

#define Remote_Len 21						//遥控器数据长度
#define IMU_Len		 33						//IMU数据长度

void Robot_init(void);
void UART_IT(UART_HandleTypeDef *huart);

#endif
