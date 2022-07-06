#ifndef MATH_OPS_H
#define MATH_OPS_H

#ifdef __cpluscplus
extern "C"{
#endif

#if defined(ARM_MATH_CM4)
#include "arm_math.h"
#else
#include "math.h"
#endif
/**
 * @brief 限幅
 * 
 */
#define LIMIT(IN,MAX,MIN)   \
        if(IN > MAX)        \
            IN = MAX;       \
        if(IN < MIN)        \
            IN = MIN        
/**
 * @brief float转int
 * 
 * @param x     当前参数
 * @param x_min 参数最小值
 * @param x_max 参数最大值
 * @param bits  转换数据所占位数
 * @return int  转换数据
 */
int float_to_uint(float x, float x_min, float x_max, int bits);

/**
 * @brief int转float
 * 
 * @param x_int 当前参数
 * @param x_min 参数最小值
 * @param x_max 参数最大值
 * @param bits  参数所占位数
 * @return float 转换数据
 */
float uint_to_float(int x_int, float x_min, float x_max, int bits);

/**
 * @brief 利用一元二次方程求解x
 * @solve Ax^2+Bx+C=0
 * @param A 系数A
 * @param B 系数B
 * @param C 系数C
 * @return float 未知数x (仅求解正向数值)
 */
float Solve_One_Quadratic_Equation(float A,float B,float C);


#ifdef __cpluscplus
}
#endif

#endif