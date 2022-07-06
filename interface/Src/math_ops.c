#include "math_ops.h"

int float_to_uint(float x, float x_min, float x_max, int bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    return (int) ((x-offset)*((float)((1<<bits)-1))/span);
}

float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int)*span/((float)((1<<bits)-1)) + offset;
}

float Solve_One_Quadratic_Equation(float A,float B,float C)
{
    #if defined(ARM_MATH_CM4)
    float mid;
		arm_status status = arm_sqrt_f32(B*B-4.0f*A*C,&mid);
    return (-B + mid)/(2.0f*A);
    #else
    return (-B + sqrtf(B*B-4.0f*A*C))/(2.0f*A);
    #endif
}

