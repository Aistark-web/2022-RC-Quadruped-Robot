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
 * @brief �޷�
 * 
 */
#define LIMIT(IN,MAX,MIN)   \
        if(IN > MAX)        \
            IN = MAX;       \
        if(IN < MIN)        \
            IN = MIN        
/**
 * @brief floatתint
 * 
 * @param x     ��ǰ����
 * @param x_min ������Сֵ
 * @param x_max �������ֵ
 * @param bits  ת��������ռλ��
 * @return int  ת������
 */
int float_to_uint(float x, float x_min, float x_max, int bits);

/**
 * @brief intתfloat
 * 
 * @param x_int ��ǰ����
 * @param x_min ������Сֵ
 * @param x_max �������ֵ
 * @param bits  ������ռλ��
 * @return float ת������
 */
float uint_to_float(int x_int, float x_min, float x_max, int bits);

/**
 * @brief ����һԪ���η������x
 * @solve Ax^2+Bx+C=0
 * @param A ϵ��A
 * @param B ϵ��B
 * @param C ϵ��C
 * @return float δ֪��x (�����������ֵ)
 */
float Solve_One_Quadratic_Equation(float A,float B,float C);


#ifdef __cpluscplus
}
#endif

#endif