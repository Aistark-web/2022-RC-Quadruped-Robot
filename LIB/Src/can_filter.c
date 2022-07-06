#include "can_filter.h"

void CanFilter_Init(CAN_HandleTypeDef *hcan) {
    CAN_FilterTypeDef canfilter;

    canfilter.FilterMode = CAN_FILTERMODE_IDMASK;
    canfilter.FilterScale = CAN_FILTERSCALE_32BIT;

    canfilter.FilterIdHigh = 0x0000;
    canfilter.FilterIdLow = 0x0000;
    canfilter.FilterMaskIdHigh = 0x0000;
    canfilter.FilterMaskIdLow = 0x0000;

    /*! 从can的过滤器起始编号 只有当设置两个can时 该参数才有意义 */
    canfilter.SlaveStartFilterBank = 14;

    /*! can1和CAN2使用不同的滤波器*/
    if (hcan->Instance == CAN1) {
        /*! 主can的过滤器编号 */
        canfilter.FilterBank = 0;

        /*! CAN_FilterFIFO0 */
        canfilter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    }

#if defined(CAN2)
    if (hcan->Instance == CAN2) {
        /*! 从can的过滤器编号 */
        canfilter.FilterBank = 14;

        /*! CAN_FilterFIFO1 */
        canfilter.FilterFIFOAssignment = CAN_FILTER_FIFO1;
    }
#endif
    /*! 激活过滤器 */
    canfilter.FilterActivation = ENABLE;
    HAL_CAN_ConfigFilter(hcan, &canfilter);
}
