#include "remote.h"

void Remote_Deal(uint8_t *Remote_Receive_Data,Remote_DataPack_Handle *DataPack)
{
    memcpy(DataPack,Remote_Receive_Data,21);
}

uint8_t Remote_Vertify(uint8_t *Remote_Receive_Data,uint8_t check_len)
{
	uint16_t get_crc = Verify_CRC16_Check_Sum(Remote_Receive_Data,check_len);
	uint16_t *crc = (uint16_t *)&Remote_Receive_Data[19];
	if(*crc == get_crc)
		return 1;
	else
		return 0;
}