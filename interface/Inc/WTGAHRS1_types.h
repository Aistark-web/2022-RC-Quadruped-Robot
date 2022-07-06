#ifndef __WTGAHRS1_TYPES_H_
#define __WTGAHRS1_TYPES_H_

#ifdef __cpluscpuls
extern "C"{
#endif

#include "stdint.h"

/**
	* @brief ʱ����
	*/
typedef struct {
	uint32_t  year;						//��
	uint8_t month;						//��
	uint8_t day;							//��
	uint8_t h;								//ʱ
	uint8_t m;								//��
	uint8_t s;								//��
	uint16_t ms;							//����
}IMU_Time_Handle;

/**
	* @brief ���ٶȾ��
	*/
typedef struct {
	float ax;									//x����ٶ�		��λ: m/s^2
	float ay;									//y����ٶ�
	float az;									//z����ٶ�
}IMU_Acceleration_Handle;

/**
	* @brief ���ٶȾ��
	*/
typedef struct {
	float wx;									//x����ٶ�		��λ: ��/s
	float wy;									//y����ٶ�
	float wz;									//z����ٶ�		
}IMU_Argular_Velocity_handle;

/**
	* @brief ŷ���Ǿ��
	*/
typedef struct {
	float Roll;								//������			��λ: ��
	float Pitch;							//�����
	float Yaw;								//������		
}IMU_Argular_Handle;

/**
	* @brief ��Ԫ�����
	*/
typedef struct {
	float Q0;
	float Q1;
	float Q2;
	float Q3;
}IMU_Quaternions_Handle;

/**
	* @brief �ų����
	*/
typedef struct {
	float Hx;									//x��ų�			��λ: ut
	float Hy;									//y��ų�
	float Hz;									//z��ų�
}IMU_Magnetic_Field_Handle;

/**
	* @brief ��ѹ���
	*/
typedef struct {
	float P;									//��ѹ				��λ: Pa
	float H;									//��ѹ�߶�		��λ: m
}IMU_Atmospheric_Pressure_Handle;

/**
	* @brief GPS��λ��γ�Ⱦ��
	*/
typedef struct {
	uint8_t Lon_dd;						//���ȶ���		��λ: ��
	float Lon_mm;							//���ȷ���		��λ: ��
	uint8_t Lat_dd;						//γ�ȶ���		��λ: ��
	float Lat_mm;							//γ�ȷ���		��λ: ��
}GPS_Longitude_and_Latitude_Handle;

/**
	* @brief GPS����
	*/
typedef struct {
	float GPS_height;					//�߶�				��λ: m
	float GPS_yaw;						//����				��λ: ��
	float GPS_V;							//����				��λ: m/s
}GPS_Ground_Speed_Handle;

/**
	* @brief GPS��λ���Ⱦ��
	*/
typedef struct {
	uint16_t SN;							//���Ǹ���		
	float PDOP;								//λ�þ���			
	float HDOP;								//�߶Ⱦ���
	float VDOP;								//��ֱ����
}GPS_Positional_Accuracy_Handle;

/**
	* @brief �豸���
	*/
typedef struct {
	float T;																				//�¶�
	IMU_Time_Handle Time;														//ʱ��
	IMU_Acceleration_Handle A;											//���ٶ�
	IMU_Argular_Velocity_handle W;									//���ٶ�
	IMU_Argular_Handle Angle;												//�Ƕ�
	IMU_Quaternions_Handle	Q;											//��Ԫ��
	IMU_Magnetic_Field_Handle H;										//�ų�
	IMU_Atmospheric_Pressure_Handle P;							//��ѹ
	GPS_Longitude_and_Latitude_Handle GPS_Pos;			//GPS��λ
	GPS_Positional_Accuracy_Handle GPS_Acc;					//GPS��λ����
	GPS_Ground_Speed_Handle GPS_GS;									//GPS����
}Dev_Handle;

#ifdef __cpluscpuls
}
#endif


#endif