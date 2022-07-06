#include "can_filter.h"

void CanFilter_Init(CAN_HandleTypeDef *hcan) {
    CAN_FilterTypeDef canfilter;

    canfilter.FilterMode = CAN_FILTERMODE_IDMASK;
    canfilter.FilterScale = CAN_FILTERSCALE_32BIT;

    canfilter.FilterIdHigh = 0x0000;
    canfilter.FilterIdLow = 0x0000;
    canfilter.FilterMaskIdHigh = 0x0000;
    canfilter.FilterMaskIdLow = 0x0000;

    /*! ��can�Ĺ�������ʼ��� ֻ�е���������canʱ �ò����������� */
    canfilter.SlaveStartFilterBank = 14;

    /*! can1��CAN2ʹ�ò�ͬ���˲���*/
    if (hcan->Instance == CAN1) {
        /*! ��can�Ĺ�������� */
        canfilter.FilterBank = 0;

        /*! CAN_FilterFIFO0 */
        canfilter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    }

#if defined(CAN2)
    if (hcan->Instance == CAN2) {
        /*! ��can�Ĺ�������� */
        canfilter.FilterBank = 14;

        /*! CAN_FilterFIFO1 */
        canfilter.FilterFIFOAssignment = CAN_FILTER_FIFO1;
    }
#endif
    /*! ��������� */
    canfilter.FilterActivation = ENABLE;
    HAL_CAN_ConfigFilter(hcan, &canfilter);
}
