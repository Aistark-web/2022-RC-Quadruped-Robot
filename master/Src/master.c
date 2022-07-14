#include "master.h"

/* BEGIN定义 */
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

#define MOTOR_KP 15.0f											//KP
#define MOTOR_KD 5.0f												//KD

#define P_MIN -95.5f        //位置最小值 
#define P_MAX 95.5f         //位置最大值

HT_03_MOTOR_Handle HT_03_MOTOR[8];					//
HT_03_MOTOR_Handle HT_03_Zero_Motor;				//
Leg_Handle Leg[4];													//腿部
PID MOTOR_PID[8];														//PID数据
PID PID_P[8];																//组成此电机P部分
PID PID_ID[8];															//组成此电机I、D部分

Ramp_Typedef Init_Ramp;													//初始化Ramp
Ramp_Typedef Shutdown_Ramp;											//关机Ramp
Ramp_Typedef Step_Ramp[4];											//原地踏步Ramp
Ramp_Typedef Straight_Ramp[4];									//直走Ramp
Ramp_Typedef Turn_Ramp[4];											//拐弯Ramp
Ramp_Typedef Circle_Ramp[4];										//原地转Ramp
Ramp_Typedef Reset_Ramp;												//复位Ramp

Ramp_Typedef Jump_ready_Ramp[4];								//跳跃准备Ramp
Ramp_Typedef Jump_ready_buffer_Ramp;						//跳跃准备缓冲Ramp
Ramp_Typedef Jump_up_Ramp[4];										//跳跃起跳Ramp
Ramp_Typedef Jump_up_buffer_Ramp;								//跳跃起跳缓冲Ramp
Ramp_Typedef Jump_up_back_Ramp[4];							//跳跃返回Ramp
Ramp_Typedef Jump_up_back_buffer_Ramp;					//跳跃返回缓冲Ramp
Ramp_Typedef Jump_down_buffer_Ramp[4];					//跳跃缓冲Ramp
Ramp_Typedef Jump_back_buffer_Ramp[4];					//跳跃落下Ramp

Ramp_Typedef Double_Bridge_Up_Ramp[4];						//双木桥提脚Ramp
Ramp_Typedef Double_Bridge_Stand_Up_Down_Ramp[4];	//双木桥上下起身Ramp
Ramp_Typedef Double_Bridge_Stand_Up_Down_Buffer_Ramp;			//双木桥上下缓冲Ramp
Ramp_Typedef Double_Bridge_Front_Rise_Ramp[4];		//双木桥搭前脚Ramp
Ramp_Typedef Double_Bridge_Back_Rise_Ramp[4];			//双木桥后脚抬升Ram
Ramp_Typedef Double_Bridge_Rise_Buffer_Ramp;			//双木桥抬脚缓冲Ramp
Ramp_Typedef Double_Bridge_Reset_Ramp[4];					//双木桥完成一个动作后y轴归位 Ramp
Ramp_Typedef Double_Bridge_Front_Down_Ramp[4];		//双木桥落前脚Ramp
Ramp_Typedef Double_Bridge_Back_Down_Ramp[4];			//双木桥落后脚Ramp
Ramp_Typedef Double_Bridge_Down_Buffer_Ramp;			//双木桥落脚缓冲Ramp
Ramp_Typedef Double_Bridge_Down_Ramp[4];					//双木桥落脚Ramp
Ramp_Typedef Double_Bridge_Straight_Ramp[4];			//双木桥直行Ramp
Ramp_Typedef Double_Bridge_Turn_Ramp[4];					//双木桥拐弯Ramp
Ramp_Typedef Double_Bridge_Circle_Ramp[4];				//双木桥原地自旋Ramp

Ramp_Typedef Seesaw_Down_Sqat_Ramp[4];							//跷跷板蹲下Ramp
Ramp_Typedef Seesaw_Down_Sqat_Back_Ramp[4];					//跷跷板蹲下返回Ramp
Ramp_Typedef Seesaw_Climb_Up_Posture_Ramp[4];				//跷跷板上坡姿态Ramp
Ramp_Typedef Seesaw_Climb_Down_Posture_Ramp[4];			//跷跷板下坡姿态Ramp
Ramp_Typedef Seesaw_Straight_Ramp[4];								//跷跷板直行Ramp
Ramp_Typedef Seesaw_Turn_Ramp[4];										//跷跷板转弯Ramp
Ramp_Typedef Seesaw_Circle_Ramp[4];									//跷跷板原地自转Ramp


uint8_t Remote_Data[Remote_Len*2];					//遥控器数据双倍缓冲
uint8_t IMU_Data[IMU_Len*2];								//IMU数据长度
uint8_t CAN_TX_Data[8][8];									//CAN发送数据缓冲
uint16_t EP_uint16_t[8];										//
float EP_float[8];													//
CAN_TxHeaderTypeDef CAN_TX[8];							//CAN发送报文

Remote_DataPack_Handle Remote_DataPack;			//遥控器数据
Remote_DataPack_Handle Remote_Last_DataPack;//上一次遥控器数据
Dev_Handle IMU;															//IMU数据
Robot_Handle Robot;													//机器人状态


/* BEGIN 初始化 */

void Robot_init()
{
		/* PID 初始化 */
	#if 1
	for(uint8_t i=0;i<8;i++){
		MOTOR_PID[i].Kp = 3.0f;
		MOTOR_PID[i].Ki = 0.5f;
		MOTOR_PID[i].Kd = 0.8f;
		MOTOR_PID[i].limit = 12.0f;
	}
	#else
	for(uint8_t i = 0;i<8;i++){
		PID_P[i].Kp = 0;
		PID_P[i].Ki = 1.5f;						//组成P部分
		PID_P[i].Kd = 0;
		PID_P[i].limit = 5.0f;				//
		PID_ID[i].Kp = 0.0f;
		PID_ID[i].Ki = 0.1f;					
		PID_ID[i].limit = 10.0f;				//组成I部分
		PID_ID[i].Kd = 1.3f;					//组成D部分
	}
	#endif
	/* 电机初始化 */
	HT_03_Motor_Init_ID(&HT_03_MOTOR[0],Leg_Motor_External_ID,1);
	HT_03_Motor_Init_ID(&HT_03_MOTOR[1],Leg_Motor_Internal_ID,2);
	HT_03_Motor_Init_ID(&HT_03_MOTOR[2],Leg_Motor_External_ID,3);
	HT_03_Motor_Init_ID(&HT_03_MOTOR[3],Leg_Motor_Internal_ID,4);
	HT_03_Motor_Init_ID(&HT_03_MOTOR[4],Leg_Motor_External_ID,5);
	HT_03_Motor_Init_ID(&HT_03_MOTOR[5],Leg_Motor_Internal_ID,6);
	HT_03_Motor_Init_ID(&HT_03_MOTOR[6],Leg_Motor_External_ID,7);
	HT_03_Motor_Init_ID(&HT_03_MOTOR[7],Leg_Motor_Internal_ID,8);
	for(uint8_t i=0;i<8;i++){
		HT_03_MOTOR[i].Expect.KP = MOTOR_KP;
		HT_03_MOTOR[i].Expect.KD = MOTOR_KD;
	}
	HT_03_Zero_Motor.Expect.KP = 0;
	HT_03_Zero_Motor.Expect.KD = 0;
	HT_03_Zero_Motor.Expect.E_P = 0;
	HT_03_Zero_Motor.Expect.E_V = 0;
	HT_03_Zero_Motor.Expect.E_T = 0;
	
	/* 斜坡初始化 */
	Init_Ramp.RampTime = 10000;
	Shutdown_Ramp.RampTime = 2000;
	
	for(uint8_t i = 0;i<4;i++){
		Step_Ramp[i].RampTime = 300;
		Straight_Ramp[i].RampTime = 500;
		Turn_Ramp[i].RampTime = 500;
		Circle_Ramp[i].RampTime = 400;
		Reset_Ramp.RampTime = 200;
		
		Jump_ready_Ramp[i].RampTime = 125;
		Jump_ready_buffer_Ramp.RampTime = 250;
		Jump_up_Ramp[i].RampTime = 2000;
		Jump_up_buffer_Ramp.RampTime = 200;						//250
		Jump_up_back_Ramp[i].RampTime = 125;
		Jump_up_back_buffer_Ramp.RampTime = 200;			//250
		Jump_down_buffer_Ramp[i].RampTime = 1500;
		Jump_back_buffer_Ramp[i].RampTime = 2000;
		
		/* 双木桥 */
		Double_Bridge_Stand_Up_Down_Ramp[i].RampTime = 500;	//双木桥起身降下Ramp
		Double_Bridge_Up_Ramp[i].RampTime 				= 1000;		//双木桥提脚Ramp
		Double_Bridge_Front_Rise_Ramp[i].RampTime = 500;		//双木桥搭前脚Ramp
		Double_Bridge_Back_Rise_Ramp[i].RampTime 	= 500;		//双木桥后脚抬升Ramp
		Double_Bridge_Rise_Buffer_Ramp.RampTime 	= 1000;		//双木桥抬脚缓冲Ramp
		
		Double_Bridge_Down_Ramp[i].RampTime				= 1000;
		Double_Bridge_Stand_Up_Down_Buffer_Ramp.RampTime = 1000;
		Double_Bridge_Front_Down_Ramp[i].RampTime = 500;		//双木桥落前脚Ramp
		Double_Bridge_Back_Down_Ramp[i].RampTime	= 500;		//双木桥落后脚Ramp
		Double_Bridge_Down_Buffer_Ramp.RampTime		= 1000;		//双木桥落脚缓冲Ramp
		
		Double_Bridge_Reset_Ramp[i].RampTime 			= 2000;		//双木桥完成一个动作后y轴归位 Ramp
		Double_Bridge_Straight_Ramp[i].RampTime		=	500;		//双木桥直行Ramp
		Double_Bridge_Turn_Ramp[i].RampTime 			= 500;		//双木桥拐弯Ramp
		Double_Bridge_Circle_Ramp[i].RampTime			=	400;		//双木桥原地自旋Ramp
		
		/* 跷跷板 */
		Seesaw_Down_Sqat_Ramp[i].RampTime						=	300;	//跷跷板蹲下Ramp
		Seesaw_Down_Sqat_Back_Ramp[i].RampTime			=	300;	//跷跷板蹲下返回Ramp
		Seesaw_Climb_Up_Posture_Ramp[i].RampTime		=	200;	//跷跷板上坡姿态Ramp
		Seesaw_Climb_Down_Posture_Ramp[i].RampTime	=	200; 	//跷跷板下坡姿态Ramp
		Seesaw_Straight_Ramp[i].RampTime						=	500;	//跷跷板直行Ramp
		Seesaw_Turn_Ramp[i].RampTime								=	500;	//跷跷板拐弯Ramp
		Seesaw_Circle_Ramp[i].RampTime							=	400;	//跷跷板原地自转Ramp
	}
	
	/* 绑定腿部电机 */
	Leg_binding(&Leg[0],&HT_03_MOTOR[0],&HT_03_MOTOR[1],FRONT_LEFT_LEG_ID);
	Leg_binding(&Leg[1],&HT_03_MOTOR[2],&HT_03_MOTOR[3],FRONT_RIGHT_LEG_ID);
	Leg_binding(&Leg[2],&HT_03_MOTOR[4],&HT_03_MOTOR[5],BACK_LEFT_LEG_ID);
	Leg_binding(&Leg[3],&HT_03_MOTOR[6],&HT_03_MOTOR[7],BACK_RIGHT_LEG_ID);
	
	/* 初始化CAN报文 */
	for(uint8_t i = 1;i<=8;i++){
		CAN_TX[i-1].StdId = i;
		CAN_TX[i-1].IDE = CAN_ID_STD;
		CAN_TX[i-1].RTR = CAN_RTR_DATA;
		CAN_TX[i-1].DLC = 8;
		CAN_TX[i-1].TransmitGlobalTime = DISABLE;
	}
	/* 初始化外设 */
	HAL_UART_Receive_DMA(&huart2,Remote_Data,Remote_Len*2);
	__HAL_UART_ENABLE_IT(&huart2,UART_IT_IDLE);
	HAL_UART_Receive_DMA(&huart3,IMU_Data,IMU_Len*2);
	__HAL_UART_ENABLE_IT(&huart3,UART_IT_IDLE);
	HAL_CAN_Start(&hcan1);
	HAL_CAN_Start(&hcan2);
	CanFilter_Init(&hcan1);
	CanFilter_Init(&hcan2);
	HAL_CAN_ActivateNotification(&hcan1,CAN_IT_RX_FIFO0_MSG_PENDING);
	HAL_CAN_ActivateNotification(&hcan2,CAN_IT_RX_FIFO1_MSG_PENDING);
	HAL_TIM_Base_Start_IT(&htim2);
	HAL_TIM_Base_Start_IT(&htim3);
}

/* END 初始化 */

/* BEGIN 中断 */

/* 定时器中断 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if(htim->Instance == TIM2){
		Robot_Check(&Robot,&Remote_DataPack);
		Robot_Move_Master(&Robot);		
	}
	else if(htim->Instance == TIM3){
		
		static uint32_t mailbox;
		if(Robot.Mode > Robot_Open_Motor_Mode){
			PID_Control(HT_03_MOTOR[Robot.can_list*2].Current.C_P,HT_03_MOTOR[Robot.can_list*2].Expect.E_P,&MOTOR_PID[Robot.can_list*2]);
			PID_Control(HT_03_MOTOR[Robot.can_list*2+1].Current.C_P,HT_03_MOTOR[Robot.can_list*2+1].Expect.E_P,&MOTOR_PID[Robot.can_list*2+1]);
			PID_Control(HT_03_MOTOR[6-(Robot.can_list*2)].Current.C_P,HT_03_MOTOR[6-(Robot.can_list*2)].Expect.E_P,&MOTOR_PID[6-(Robot.can_list*2)]);
			PID_Control(HT_03_MOTOR[7-(Robot.can_list*2)].Current.C_P,HT_03_MOTOR[7-(Robot.can_list*2)].Expect.E_P,&MOTOR_PID[7-(Robot.can_list*2)]);
			if(!Robot.Torque_Control){							//不开启力控模式
				#if 1
	//			PID_Control(HT_03_MOTOR[Robot.can_list*2].Current.C_P,HT_03_MOTOR[Robot.can_list*2].Expect.E_P,&MOTOR_PID[Robot.can_list*2]);
	//			PID_Control(HT_03_MOTOR[Robot.can_list*2+1].Current.C_P,HT_03_MOTOR[Robot.can_list*2+1].Expect.E_P,&MOTOR_PID[Robot.can_list*2+1]);
	//			PID_Control(HT_03_MOTOR[Robot.can_list*2+4].Current.C_P,HT_03_MOTOR[Robot.can_list*2+4].Expect.E_P,&MOTOR_PID[Robot.can_list*2+4]);
	//			PID_Control(HT_03_MOTOR[Robot.can_list*2+5].Current.C_P,HT_03_MOTOR[Robot.can_list*2+5].Expect.E_P,&MOTOR_PID[Robot.can_list*2+5]);
	//			HT_03_MOTOR[Robot.can_list*2].Expect.E_T 		= MOTOR_PID[Robot.can_list*2].pid_out;
	//			HT_03_MOTOR[Robot.can_list*2+1].Expect.E_T	= MOTOR_PID[Robot.can_list*2+1].pid_out;
	//			HT_03_MOTOR[Robot.can_list*2+4].Expect.E_T	= MOTOR_PID[Robot.can_list*2+4].pid_out;
	//			HT_03_MOTOR[Robot.can_list*2+5].Expect.E_T	= MOTOR_PID[Robot.can_list*2+5].pid_out;
	//			
	//			/* 封包 */
	//			HT_03_Motor_Data_packet(CAN_TX_Data[Robot.can_list*2],&HT_03_MOTOR[Robot.can_list*2]);
	//			HT_03_Motor_Data_packet(CAN_TX_Data[Robot.can_list*2+1],&HT_03_MOTOR[Robot.can_list*2+1]);
	//			HT_03_Motor_Data_packet(CAN_TX_Data[Robot.can_list*2+4],&HT_03_MOTOR[Robot.can_list*2+4]);
	//			HT_03_Motor_Data_packet(CAN_TX_Data[Robot.can_list*2+5],&HT_03_MOTOR[Robot.can_list*2+5]);
	//			
	//			/* 发送数据 */
	//			HAL_CAN_AddTxMessage(&hcan1,&CAN_TX[Robot.can_list*2],CAN_TX_Data[Robot.can_list*2],&mailbox);
	//			HAL_CAN_AddTxMessage(&hcan2,&CAN_TX[Robot.can_list*2+4],CAN_TX_Data[Robot.can_list*2+4],&mailbox);
	//			HAL_CAN_AddTxMessage(&hcan1,&CAN_TX[Robot.can_list*2+1],CAN_TX_Data[Robot.can_list*2+1],&mailbox);
	//			HAL_CAN_AddTxMessage(&hcan2,&CAN_TX[Robot.can_list*2+5],CAN_TX_Data[Robot.can_list*2+5],&mailbox);
				
				HT_03_MOTOR[Robot.can_list*2].Expect.E_T	 		= MOTOR_PID[Robot.can_list*2].pid_out;
				HT_03_MOTOR[Robot.can_list*2+1].Expect.E_T 		= MOTOR_PID[Robot.can_list*2+1].pid_out;
				HT_03_MOTOR[6-(Robot.can_list*2)].Expect.E_T	= MOTOR_PID[6-(Robot.can_list*2)].pid_out;
				HT_03_MOTOR[7-(Robot.can_list*2)].Expect.E_T	= MOTOR_PID[7-(Robot.can_list*2)].pid_out;
				
				HT_03_Motor_Data_packet(CAN_TX_Data[Robot.can_list*2],&HT_03_MOTOR[Robot.can_list*2]);
				HT_03_Motor_Data_packet(CAN_TX_Data[Robot.can_list*2+1],&HT_03_MOTOR[Robot.can_list*2+1]);
				HT_03_Motor_Data_packet(CAN_TX_Data[6-(Robot.can_list*2)],&HT_03_MOTOR[6-(Robot.can_list*2)]);
				HT_03_Motor_Data_packet(CAN_TX_Data[7-(Robot.can_list*2)],&HT_03_MOTOR[7-(Robot.can_list*2)]);

				HAL_CAN_AddTxMessage(&hcan1,&CAN_TX[Robot.can_list*2],CAN_TX_Data[Robot.can_list*2],&mailbox);
				HAL_CAN_AddTxMessage(&hcan2,&CAN_TX[6-(Robot.can_list*2)],CAN_TX_Data[6-(Robot.can_list*2)],&mailbox);
				HAL_CAN_AddTxMessage(&hcan1,&CAN_TX[Robot.can_list*2+1],CAN_TX_Data[Robot.can_list*2+1],&mailbox);
				HAL_CAN_AddTxMessage(&hcan2,&CAN_TX[7-(Robot.can_list*2)],CAN_TX_Data[7-(Robot.can_list*2)],&mailbox);
				#endif
	
			}
			else{
				//电机数据包装
				HT_03_Motor_Data_packet(CAN_TX_Data[Robot.can_list*2],&HT_03_MOTOR[Robot.can_list*2]);
				HT_03_Motor_Data_packet(CAN_TX_Data[Robot.can_list*2+1],&HT_03_MOTOR[Robot.can_list*2+1]);
				HT_03_Motor_Data_packet(CAN_TX_Data[6-(Robot.can_list*2)],&HT_03_MOTOR[6-(Robot.can_list*2)]);
				HT_03_Motor_Data_packet(CAN_TX_Data[7-(Robot.can_list*2)],&HT_03_MOTOR[7-(Robot.can_list*2)]);

				HAL_CAN_AddTxMessage(&hcan1,&CAN_TX[Robot.can_list*2],CAN_TX_Data[Robot.can_list*2],&mailbox);
				HAL_CAN_AddTxMessage(&hcan2,&CAN_TX[6-(Robot.can_list*2)],CAN_TX_Data[6-(Robot.can_list*2)],&mailbox);
				HAL_CAN_AddTxMessage(&hcan1,&CAN_TX[Robot.can_list*2+1],CAN_TX_Data[Robot.can_list*2+1],&mailbox);
				HAL_CAN_AddTxMessage(&hcan2,&CAN_TX[7-(Robot.can_list*2)],CAN_TX_Data[7-(Robot.can_list*2)],&mailbox);
			}
			Robot.can_list++;										//更新发送列表
			if(Robot.can_list == 2){						//一次循环结束
				Robot.can_list = 0;
			}
		}
	}
}

/* UART IDLE 中断 */
void UART_IT(UART_HandleTypeDef *huart){
	if(__HAL_UART_GET_IT_SOURCE(huart,UART_IT_IDLE)&&__HAL_UART_GET_FLAG(huart,UART_FLAG_IDLE)){
		__HAL_UART_CLEAR_IDLEFLAG(huart);
		if(huart->Instance == USART2){
			HAL_UART_DMAStop(huart);
			for(uint8_t i=0;i<Remote_Len;i++){
				if(Remote_Data[i] == 0x5A){
					if(Remote_Vertify(Remote_Data,Remote_Len-2)){
						memcpy(&Remote_Last_DataPack,&Remote_DataPack,Remote_Len);					//更新遥控器数据	
						Remote_Deal(Remote_Data,&Remote_DataPack);
						break;
					}
					else
						continue;
				}
			}
			memset(Remote_Data,0,Remote_Len*2);
			HAL_UART_Receive_DMA(huart,Remote_Data,Remote_Len*2);
		}
		else if(huart->Instance == USART3){
			HAL_UART_DMAStop(huart);
			for(uint8_t i = 0;i<IMU_Len;i++){
				if(IMU_Data[i] == 0x55){
					if(IMU_Data[i+1] == 0x51){																						//判断是否成功	  获取加速度 角速度 欧拉角
						WTGAHRS1_Data_deal(&IMU_Data[i],&IMU);
						WTGAHRS1_Data_deal(&IMU_Data[i+WTGAHRS1_DATA_OFFSET],&IMU);
						WTGAHRS1_Data_deal(&IMU_Data[i+WTGAHRS1_DATA_OFFSET*2],&IMU);
						break;
					}
					else
						continue;
				}
			}
			memset(IMU_Data,0,IMU_Len*2);
			HAL_UART_Receive_DMA(huart,IMU_Data,IMU_Len*2);
		}
	}
}

/* DMA完全接收回调 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	if(huart->Instance == USART2){
		HAL_UART_Receive_DMA(huart,Remote_Data,Remote_Len*2);
	}
	else if(huart->Instance == USART3){
		HAL_UART_Receive_DMA(huart,IMU_Data,IMU_Len*2);
	}
}

/* CAN1 FIFO0 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan){
	if(hcan->Instance == CAN1){
		static CAN_RxHeaderTypeDef CAN_RX;
		static uint8_t data[8];
		HAL_CAN_GetRxMessage(hcan,CAN_RX_FIFO0,&CAN_RX,data);
		if(!HT_03_MOTOR[CAN_RX.StdId-1].first_state){
			HT_03_Motor_Get_init_positon(data,&HT_03_MOTOR[CAN_RX.StdId-1]);
		}
		else{
			HT_03_Motor_Data_Unpack(data,&HT_03_MOTOR[CAN_RX.StdId-1]);
			if(CAN_RX.StdId <= 2){
				Leg_get(&Leg[0]);
			}
			else if(CAN_RX.StdId <= 4){
				Leg_get(&Leg[1]);
			}
			else if(CAN_RX.StdId <= 6){
				Leg_get(&Leg[2]);
			}
			else if(CAN_RX.StdId <= 8){
				Leg_get(&Leg[3]);
			}
		}
	}
}

/* CAN2 FIFO1 */
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan){
	if(hcan->Instance == CAN2){
		static CAN_RxHeaderTypeDef CAN_RX;
		static uint8_t data[8];
		HAL_CAN_GetRxMessage(hcan,CAN_RX_FIFO1,&CAN_RX,data);
		if(!HT_03_MOTOR[CAN_RX.StdId-1].first_state){
			HT_03_Motor_Get_init_positon(data,&HT_03_MOTOR[CAN_RX.StdId-1]);
		}
		else{
			HT_03_Motor_Data_Unpack(data,&HT_03_MOTOR[CAN_RX.StdId-1]);
			if(CAN_RX.StdId <= 2){
				Leg_get(&Leg[0]);
			}
			else if(CAN_RX.StdId <= 4){
				Leg_get(&Leg[1]);
			}
			else if(CAN_RX.StdId <= 6){
				Leg_get(&Leg[2]);
			}
			else if(CAN_RX.StdId <= 8){
				Leg_get(&Leg[3]);
			}
		}
	}
}

/* END 中断 */
