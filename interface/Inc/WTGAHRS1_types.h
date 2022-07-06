#ifndef __WTGAHRS1_TYPES_H_
#define __WTGAHRS1_TYPES_H_

#ifdef __cpluscpuls
extern "C"{
#endif

#include "stdint.h"

/**
	* @brief 时间句柄
	*/
typedef struct {
	uint32_t  year;						//年
	uint8_t month;						//月
	uint8_t day;							//日
	uint8_t h;								//时
	uint8_t m;								//分
	uint8_t s;								//秒
	uint16_t ms;							//毫秒
}IMU_Time_Handle;

/**
	* @brief 加速度句柄
	*/
typedef struct {
	float ax;									//x轴加速度		单位: m/s^2
	float ay;									//y轴加速度
	float az;									//z轴加速度
}IMU_Acceleration_Handle;

/**
	* @brief 角速度句柄
	*/
typedef struct {
	float wx;									//x轴角速度		单位: °/s
	float wy;									//y轴角速度
	float wz;									//z轴角速度		
}IMU_Argular_Velocity_handle;

/**
	* @brief 欧拉角句柄
	*/
typedef struct {
	float Roll;								//翻滚角			单位: °
	float Pitch;							//航向角
	float Yaw;								//俯仰角		
}IMU_Argular_Handle;

/**
	* @brief 四元数句柄
	*/
typedef struct {
	float Q0;
	float Q1;
	float Q2;
	float Q3;
}IMU_Quaternions_Handle;

/**
	* @brief 磁场句柄
	*/
typedef struct {
	float Hx;									//x轴磁场			单位: ut
	float Hy;									//y轴磁场
	float Hz;									//z轴磁场
}IMU_Magnetic_Field_Handle;

/**
	* @brief 气压句柄
	*/
typedef struct {
	float P;									//气压				单位: Pa
	float H;									//气压高度		单位: m
}IMU_Atmospheric_Pressure_Handle;

/**
	* @brief GPS定位经纬度句柄
	*/
typedef struct {
	uint8_t Lon_dd;						//经度度数		单位: °
	float Lon_mm;							//经度分数		单位: ′
	uint8_t Lat_dd;						//纬度度数		单位: °
	float Lat_mm;							//纬度分数		单位: ′
}GPS_Longitude_and_Latitude_Handle;

/**
	* @brief GPS地速
	*/
typedef struct {
	float GPS_height;					//高度				单位: m
	float GPS_yaw;						//航向				单位: °
	float GPS_V;							//地速				单位: m/s
}GPS_Ground_Speed_Handle;

/**
	* @brief GPS定位精度句柄
	*/
typedef struct {
	uint16_t SN;							//卫星个数		
	float PDOP;								//位置精度			
	float HDOP;								//高度精度
	float VDOP;								//垂直精度
}GPS_Positional_Accuracy_Handle;

/**
	* @brief 设备句柄
	*/
typedef struct {
	float T;																				//温度
	IMU_Time_Handle Time;														//时间
	IMU_Acceleration_Handle A;											//加速度
	IMU_Argular_Velocity_handle W;									//角速度
	IMU_Argular_Handle Angle;												//角度
	IMU_Quaternions_Handle	Q;											//四元数
	IMU_Magnetic_Field_Handle H;										//磁场
	IMU_Atmospheric_Pressure_Handle P;							//气压
	GPS_Longitude_and_Latitude_Handle GPS_Pos;			//GPS定位
	GPS_Positional_Accuracy_Handle GPS_Acc;					//GPS定位精度
	GPS_Ground_Speed_Handle GPS_GS;									//GPS地速
}Dev_Handle;

#ifdef __cpluscpuls
}
#endif


#endif