#include "HT_03_motor.h"

/**
 * @brief 以下宏定义的限幅参数需与电机编码器相同
 *        限幅参数可在上位机中调节
 * 
 */
#define P_MIN -95.5f        //位置最小值 
#define P_MAX 95.5f         //位置最大值
#define V_MIN -45.0f        //速度最小值
#define V_MAX 45.0f         //速度最大值
#define KP_MIN 0.0f         //位置期望最小值
#define KP_MAX 500.0f       //位置期望最大值
#define KD_MIN 0.0f         //速度期望最小值
#define KD_MAX 5.0f         //速度期望最大值
#define T_MIN -18.0f        //扭矩期望最小值
#define T_MAX 18.0f         //扭矩期望最大值

/// CAN Reply Packet Structure ///
/// 16 bit position, between -4*pi and 4*pi
/// 12 bit velocity, between -30 and + 30 rad/s
/// 12 bit current, between -40 and 40;
/// CAN Packet is 5 8-bit words
/// Formatted as follows.  For each quantity, bit 0 is LSB
/// 0: [position[15-8]]
/// 1: [position[7-0]] 
/// 2: [velocity[11-4]]
/// 3: [velocity[3-0], current[11-8]]
/// 4: [current[7-0]]
void HT_03_Motor_Data_Unpack(uint8_t *data,HT_03_MOTOR_Handle *motor)
{
    motor->can_slave_id = data[0];
    uint16_t p_uint = (data[1] << 8) | data[2];
    uint16_t v_uint = (data[3] << 4) | data[4] >> 4;
    uint32_t t_uint  = ((data[4] & 0x0F) << 8) | data[5];
		motor->Source_Positon = uint_to_float(p_uint,P_MIN,P_MAX,16);
    motor->Current.C_P = motor->Source_Positon - motor->Init_Position;
    motor->Current.C_V = uint_to_float(v_uint,V_MIN,V_MAX,12);
    motor->Current.C_T = uint_to_float(t_uint,T_MIN,T_MAX,12);
}

/// CAN Command Packet Structure ///
/// 16 bit position command, between -4*pi and 4*pi
/// 12 bit velocity command, between -30 and + 30 rad/s
/// 12 bit kp, between 0 and 500 N-m/rad
/// 12 bit kd, between 0 and 100 N-m*s/rad
/// 12 bit feed forward torque, between -18 and 18 N-m
/// CAN Packet is 8 8-bit words
/// Formatted as follows.  For each quantity, bit 0 is LSB
/// 0: [position[15-8]]
/// 1: [position[7-0]] 
/// 2: [velocity[11-4]]
/// 3: [velocity[3-0], kp[11-8]]
/// 4: [kp[7-0]]
/// 5: [kd[11-4]]
/// 6: [kd[3-0], torque[11-8]]
/// 7: [torque[7-0]]
void HT_03_Motor_Data_packet(uint8_t *data,HT_03_MOTOR_Handle *motor)
{
	motor->Source_EP = motor->Expect.E_P + motor->Init_Position;
	LIMIT(motor->Source_EP,P_MAX,P_MIN);
	LIMIT(motor->Expect.E_V,V_MAX,V_MIN);
	LIMIT(motor->Expect.E_T,T_MAX,T_MIN);
	LIMIT(motor->Expect.KP,KP_MAX,KP_MIN);
	LIMIT(motor->Expect.KD,KD_MAX,KD_MIN);
	uint16_t p_uint 	= float_to_uint(motor->Source_EP,	P_MIN,	P_MAX,	16);
	uint16_t v_uint 	= float_to_uint(motor->Expect.E_V,	V_MIN,	V_MAX,	12);
	uint16_t kp_uint 	= float_to_uint(motor->Expect.KP,  KP_MIN,	KP_MAX,	12);
	uint16_t kd_uint 	= float_to_uint(motor->Expect.KD,   KD_MIN,	KD_MAX,	12);
	uint16_t t_uint 	= float_to_uint(motor->Expect.E_T,	T_MIN,	T_MAX,	12);
	data[0] = p_uint >> 8;
	data[1] = (p_uint & 0xFF);
	data[2] = v_uint >> 4;
	data[3] = ((v_uint & 0xF) << 4) | (kp_uint >> 8);
	data[4] = (kp_uint & 0xFF);
	data[5] = kd_uint >> 4;
	data[6] = ((kd_uint & 0xF) << 4) | (t_uint >> 8);
	data[7] = t_uint & 0xFF;
}

void HT_03_Motor_Get_init_positon(uint8_t *data,HT_03_MOTOR_Handle *motor)
{
	motor->can_slave_id = data[0];
	uint16_t p_uint = (data[1] << 8) | data[2];
	motor->Init_Position = uint_to_float(p_uint,P_MIN,P_MAX,16);
	motor->Source_Positon = motor->Init_Position;
	motor->first_state = 1;
}

void HT_03_Motor_Init_ID(HT_03_MOTOR_Handle *motor,Leg_Motor_enum leg_motor_id,uint8_t can_slave_id){
	motor->can_slave_id = can_slave_id;
	motor->leg_motor_id = leg_motor_id;
}