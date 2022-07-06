#ifndef __WTGAHRS1_H_
#define __WTGAHRS1_H_

#ifdef __cpluscpuls
extern "C"{
#endif

#include "WTGAHRS1_types.h"


#define g				9.8f				//重力加速度大小
#define A_range 16.0f				//加速度量程 默认16g 			 
#define W_range 2000.0f			//角速度量程 默认2000°/s
#define H_range 4900.0f			//磁场强度量程 4900ut
#define WTGAHRS1_DATA_OFFSET 11			//单个数据包偏移数(长度)


/**
	* @function 处理单个数据包
	* @param	head 单个数据包包头 
	*/
void WTGAHRS1_Data_deal(uint8_t *head,Dev_Handle *Dev);


/**
	* @function 校验单个数据包
	* @param	head 单个数据包包头
	* @return 1:检验正确
			  0:校验错误
	*/
uint8_t WTGAHRS1_Data_vertify(uint8_t *head);

#ifdef __cpluscpuls
}
#endif

#endif