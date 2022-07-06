#ifndef __WTGAHRS1_H_
#define __WTGAHRS1_H_

#ifdef __cpluscpuls
extern "C"{
#endif

#include "WTGAHRS1_types.h"


#define g				9.8f				//�������ٶȴ�С
#define A_range 16.0f				//���ٶ����� Ĭ��16g 			 
#define W_range 2000.0f			//���ٶ����� Ĭ��2000��/s
#define H_range 4900.0f			//�ų�ǿ������ 4900ut
#define WTGAHRS1_DATA_OFFSET 11			//�������ݰ�ƫ����(����)


/**
	* @function ���������ݰ�
	* @param	head �������ݰ���ͷ 
	*/
void WTGAHRS1_Data_deal(uint8_t *head,Dev_Handle *Dev);


/**
	* @function У�鵥�����ݰ�
	* @param	head �������ݰ���ͷ
	* @return 1:������ȷ
			  0:У�����
	*/
uint8_t WTGAHRS1_Data_vertify(uint8_t *head);

#ifdef __cpluscpuls
}
#endif

#endif