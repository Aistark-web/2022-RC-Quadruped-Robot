#ifndef __LEG_TYPES_H_
#define __LEG_TYPES_H_

#ifdef __cpluscplus
extern "C"{
#endif

#include "HT_03_motor_types.h"
/**
 * @brief ������ID
 * 
 */
typedef enum{
    FRONT_LEFT_LEG_ID = 1,             //��ǰ��
    FRONT_RIGHT_LEG_ID = 2,            //��ǰ��
    BACK_LEFT_LEG_ID = 3,              //�����
    BACK_RIGHT_LEG_ID = 4              //�Һ���
}LEG_ID_ENUM;

/**
 * @brief �Ȳ����
 * 
 */
typedef struct
{
    LEG_ID_ENUM Leg_ID;             //�Ȳ�ID
    HT_03_MOTOR_Handle *Motor_1;    //�����
    HT_03_MOTOR_Handle *Motor_2;    //�ڲ���
    float L1;                       //���ȳ�
    float L2;                       //С�ȳ�
    float theta1;                   //��1
    float theta2;                   //��2
    float Psi;                      //��
    float Phi;                      //��
    float L;                        //�ȳ�
    float y;                        //������Ȳ�����ϵ�µ�y����
    float z;                        //������Ȳ�����ϵ�µ�z����
    float Infer_theta1;             //������1
    float Infer_theta2;             //������2
    float Infer_y;                  //����y
    float Infer_z;                  //����z
    float Infer_L;                  //�����ȳ�
}Leg_Handle;


#ifdef __cpluscplus
}
#endif

#endif