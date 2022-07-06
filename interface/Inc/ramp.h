
#ifndef __RAMP_H_
#define __RAMP_H_

#include "stm32f4xx_hal.h"

/**
 * @brief 斜坡函数结构�?
 * @note  定义时应初始化斜坡时间RampTime为指定的时间
 */
typedef struct {
    uint8_t flag;       //!<@brief 启动标志
    uint32_t StartTick; //!<@brief 起始时间
    uint32_t RampTime;  //!<@brief 斜坡时间
} Ramp_Typedef;

/**
 * @brief 斜坡函数,弱函数可重写为自定义时钟�?,默认SysTick
 * @note 可重写该函数，更换为自定义时钟源
 * @return 时钟计数�?
 */
uint32_t Get_TimerTick(void);

/**
 * @brief 斜坡函数
 * @param[in] Ramp 斜坡函数结构体指�?
 * @return 0-1的随时间匀速上升的变量
 */
float Slope(Ramp_Typedef *Ramp);

/**
 * @brief 重置斜坡
 * @param[in] Ramp 斜坡函数结构体指�?
 */
inline void ResetSlope(Ramp_Typedef *Ramp) {
    Ramp->flag = 0;
}



#endif
