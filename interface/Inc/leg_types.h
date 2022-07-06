#ifndef __LEG_TYPES_H_
#define __LEG_TYPES_H_

#ifdef __cpluscplus
extern "C"{
#endif

#include "HT_03_motor_types.h"
/**
 * @brief 腿所属ID
 * 
 */
typedef enum{
    FRONT_LEFT_LEG_ID = 1,             //左前腿
    FRONT_RIGHT_LEG_ID = 2,            //右前腿
    BACK_LEFT_LEG_ID = 3,              //左后腿
    BACK_RIGHT_LEG_ID = 4              //右后腿
}LEG_ID_ENUM;

/**
 * @brief 腿部句柄
 * 
 */
typedef struct
{
    LEG_ID_ENUM Leg_ID;             //腿部ID
    HT_03_MOTOR_Handle *Motor_1;    //外侧电机
    HT_03_MOTOR_Handle *Motor_2;    //内侧电机
    float L1;                       //大腿长
    float L2;                       //小腿长
    float theta1;                   //θ1
    float theta2;                   //θ2
    float Psi;                      //ψ
    float Phi;                      //φ
    float L;                        //腿长
    float y;                        //足尖在腿部坐标系下的y坐标
    float z;                        //足尖在腿部坐标系下的z坐标
    float Infer_theta1;             //期望θ1
    float Infer_theta2;             //期望θ2
    float Infer_y;                  //期望y
    float Infer_z;                  //期望z
    float Infer_L;                  //期望腿长
}Leg_Handle;


#ifdef __cpluscplus
}
#endif

#endif