#ifndef __CSYS_TYPES_H_
#define __CSYS_TYPES_H_

#ifdef __cpluscplus
extern "C"{
#endif

#include "leg_types.h"

/**
 * @brief �Ȳ�����ϵ
 * 
 */
typedef struct 
{
    LEG_ID_ENUM Leg_id;
    float x;
    float y;
    float z;
}CSYS_Leg_Handle;


/**
 * @brief ��е������ϵ
 * 
 */
typedef struct
{
    float Base_x;
    float Base_y;
    float Base_z;
    float Hand_x;
    float Hand_y;
    float Hand_z;
}CSYS_Mec_Arm_Handle;

/**
 * @brief �������������ϵ
 * 
 */
typedef struct 
{
    CSYS_Leg_Handle Leg[4];
    CSYS_Mec_Arm_Handle Mec_Arm;
}CSYS_Global_Handle;


/**
 * @brief ��������ϵ
 * 
 */
typedef struct
{
    //TODO
}CSYS_World_Handle;

#ifdef __cpluscplus
}
#endif

#endif