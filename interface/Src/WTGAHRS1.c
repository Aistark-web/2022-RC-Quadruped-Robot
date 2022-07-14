#include "WTGAHRS1.h"

void WTGAHRS1_Data_deal(uint8_t *head,Dev_Handle *Dev)
{
	switch(*(head+1))
	{
		/* 时间 */
		case 0x50:
		{
			Dev->Time.year 		= 2000 + *(head+2);
			Dev->Time.month 	= *(head+3);
			Dev->Time.day 		= *(head + 4);
			Dev->Time.h 			= *(head + 5);
			Dev->Time.m 			= *(head + 6);
			Dev->Time.s 			= *(head + 7);
			Dev->Time.ms 			= *(head + 8);
			break;
		}
		/* 加速度 */
		case 0x51:
		{
			int16_t mid_ax 	= *(head + 2) | (*(head + 3)<<8);
			int16_t mid_ay 	= *(head + 4) | (*(head + 5)<<8);
			int16_t mid_az 	= *(head + 6) | (*(head + 7)<<8);
			int16_t mid_T		=	(*(head+8) | (*(head+9)<<8));
			Dev->A.ax 			= mid_ax / 32768.0f * A_range * g;
			Dev->A.ay				=	mid_ay / 32768.0f * A_range * g;
			Dev->A.az				=	mid_az / 32768.0f * A_range * g;
			Dev->T					=	mid_T / 100.0f;
			break;
		}
		/* 角速度 */
		case 0x52:
		{
			int16_t mid_wx 		= *(head + 2) | (*(head + 3)<<8);
			int16_t mid_wy 		= *(head + 4) | (*(head + 5)<<8);
			int16_t mid_wz 		= *(head + 6) | (*(head + 7)<<8);
			int16_t mid_T			=	(*(head+8) | (*(head+9)<<8));
			Dev->W.wx					=	mid_wx / 32768.0f * W_range;
			Dev->W.wy					=	mid_wy / 32768.0f * W_range;
			Dev->W.wz					=	mid_wz / 32768.0f * W_range;			
			Dev->T						=	mid_T / 100.0f;
			break;
		}
		/* 欧拉角 */
		case 0x53:
		{
			int16_t mid_p			=	*(head + 2) | (*(head + 3)<<8);
			int16_t mid_r			= *(head + 4) | (*(head + 5)<<8);
			int16_t mid_y			=	*(head + 6) | (*(head + 7)<<8);
			int16_t mid_T			=	*(head + 8) | (*(head + 9)<<8);
			Dev->Angle.Pitch	=	mid_p / 32768.0f * 180.0f;		
			Dev->Angle.Roll		=	mid_r / 32768.0f * 180.0f;		
			Dev->Angle.Yaw		=	mid_y / 32768.0f * 180.0f;		
			Dev->T						=	mid_T / 100.0f;
			break;
		}
		/* 磁场 */
		case 0x54:
		{
			int16_t mid_hx	=	*(head + 2) | (*(head + 3)<<8);
			int16_t mid_hy	= *(head + 4) | (*(head + 5)<<8);
			int16_t mid_hz	=	*(head + 6) | (*(head + 7)<<8);
			int16_t mid_T		=	*(head + 8) | (*(head + 9)<<8);
			Dev->H.Hx				=	mid_hx / 32768.0f * H_range;
			Dev->H.Hy				=	mid_hy / 32768.0f * H_range;
			Dev->H.Hz				=	mid_hz / 32768.0f * H_range;
			Dev->T					=	mid_T	/ 100.0f;
			break;
		}
		/* 气压、高度 */
		case 0x56:
		{
			uint32_t mid_p 	= *(head + 2) | (*(head + 3) << 8) | (*(head + 4) << 16) | (*(head + 5) << 24);
			uint32_t mid_h 	= *(head + 6) | (*(head + 7) << 8) | (*(head + 8) << 16) | (*(head + 9) << 24);
			Dev->P.P				= *(float *)&mid_p;
			Dev->P.H				=	*(float *)&mid_h;
			break;
		}
		/* 经纬度 */
		case 0x57:
		{
			uint32_t mid_lon 		= *(head + 2) | (*(head + 3) << 8) | (*(head + 4) << 16) | (*(head + 5) << 24);
			uint32_t mid_lat		= *(head + 6) | (*(head + 7) << 8) | (*(head + 8) << 16) | (*(head + 9) << 24);
			Dev->GPS_Pos.Lon_dd = mid_lon/10000000;
			Dev->GPS_Pos.Lon_mm = (mid_lon % 10000000) / 10000000.0f;
			Dev->GPS_Pos.Lat_dd	= mid_lat/10000000;
			Dev->GPS_Pos.Lat_mm	=	(mid_lat % 10000000) / 10000000.0f;
			break;
		}
		/* 地速 */
		case 0x58:
		{
			uint32_t mid_v				=	*(head + 6) | (*(head + 7) << 8) | (*(head + 8) << 16) | (*(head + 9) << 24);
			Dev->GPS_GS.GPS_height = (*(head + 2) | (*(head + 3) << 8)) / 10.0f;
			Dev->GPS_GS.GPS_yaw 		= (*(head + 4) | (*(head + 5) << 8)) / 100.0f;
			Dev->GPS_GS.GPS_V			=	mid_v / 1000.0f;
			break;
		}
		/* 四元数 */
		case 0x59:
		{
			Dev->Q.Q0			=	(*(head + 2) | (*(head + 3) << 8)) / 32768.0f;
			Dev->Q.Q1			=	(*(head + 4) | (*(head + 5) << 8)) / 32768.0f;
			Dev->Q.Q2			=	(*(head + 6) | (*(head + 7) << 8)) / 32768.0f;
			Dev->Q.Q3			=	(*(head + 8) | (*(head + 9) << 8)) / 32768.0f;
			break;
		}
		/* 卫星定位精度 */
		case 0x5A:
		{
			Dev->GPS_Acc.SN			= *(head + 2) | (*(head + 3) << 8);
			Dev->GPS_Acc.PDOP		=	(*(head + 4) | (*(head + 5)) << 8) / 100.0f;
			Dev->GPS_Acc.HDOP		=	(*(head + 6) | (*(head + 7)) << 8) / 100.0f;
			Dev->GPS_Acc.VDOP		=	(*(head + 8) | (*(head + 9)) << 8) / 100.0f;
			break;
		}
		default:
			break;
	}
}

uint8_t WTGAHRS1_Data_vertify(uint8_t *head)
{
	uint8_t rec_sum = *(head+10);
	uint32_t mid_sum;
	uint8_t cul_sum;
	for(uint8_t i=0;i<10;i++)
	{
		mid_sum += *(head+i);
	}
	cul_sum = mid_sum & 0XFF;
	if(cul_sum == rec_sum)
		return 1;
	else
		return 0;
}