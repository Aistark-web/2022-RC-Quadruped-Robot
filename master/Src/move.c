#include "move.h"

extern Ramp_Typedef Init_Ramp;										//初始化Ramp
extern Ramp_Typedef Shutdown_Ramp;								//关机Ramp
extern Ramp_Typedef Step_Ramp[4];										//原地踏步Ramp
extern Ramp_Typedef Straight_Ramp[4];							//直走Ramp
extern Ramp_Typedef Turn_Ramp[4];									//拐弯Ramp
extern Ramp_Typedef Circle_Ramp[4];								//原地转Ramp
extern Ramp_Typedef Jump_ready_Ramp;							//跳跃准备Ramp
extern Ramp_Typedef Jump_up_Ramp;									//跳跃起跳Ramp
extern Ramp_Typedef Jump_up_back_Ramp;						//跳跃返回Ramp
extern Ramp_Typedef Jump_down_buffer_Ramp[4];			//跳跃缓冲Ramp
extern Ramp_Typedef Jump_back_buffer_Ramp[4];			//跳跃后返回原点Ramp

extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
extern Dev_Handle IMU;
extern Remote_DataPack_Handle Remote_DataPack;			//遥控器数据
extern Remote_DataPack_Handle Remote_Last_DataPack;	//上一次遥控器数据
extern Leg_Handle Leg[4];
extern HT_03_MOTOR_Handle HT_03_Zero_Motor;				//零状态电机
extern uint8_t CAN_TX_Data[8][8];									//CAN发送数据缓冲
extern CAN_TxHeaderTypeDef CAN_TX[8];							//CAN发送报文
uint8_t Motor_Mode_data[8] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFC};						//电机控制模式

/* 模式状态调度 */
/********************
		Mode:    Robot_Init_Mode -> Robot_Open_Motor_Mode -> Robot_Start_Mode -> Robot_Move_Mode							
*/

void Robot_Check(Robot_Handle *Robot,Remote_DataPack_Handle *Remote)
{
	if(Robot->Change_Mode == 0){									//正在改变模式时置1，锁定此界面
		switch(Robot->Mode){
			case Robot_Init_Mode:{
				if(Remote->Key.Left_Key_Up){  
					Robot->Mode = Robot_Open_Motor_Mode;
					Robot->Change_Mode = 1;								
				}
				break;
			}
			case Robot_Open_Motor_Mode:{
				if(Remote->Key.Left_Key_Down){
					Robot->Mode = Robot_Start_Mode;
					Robot->Change_Mode = 1;
				}
				break;
			}
			case Robot_Start_Mode:{										//start模式执行完毕后自动切换到move模式
				break;
			}
			case Robot_Move_Mode:{
				//TODO	运动行为改变
				static uint32_t count;
				if(Robot->State == Robot_Stop_State){				//停止时调度动作
					if(Robot->Move == Robot_Stop_Move){
						if(Remote_DataPack.Key.Left_Rocker && Remote_DataPack.Key.Right_Rocker){
							if(++count >= 160){									//大约2s
								count = 0;
								Robot->Mode = Robot_Shutdown_Mode;		//进入关机模式
								Robot->Change_Mode = 1;
							}
							break;
						}
						/* 直行/拐弯 */
						else if(Remote_DataPack.L_Y_rocker < 500 || Remote_DataPack.L_Y_rocker > 3500){																							
							if(Remote_DataPack.R_X_rocker > 500 && Remote_DataPack.R_X_rocker < 3500){			//判断为直行动作
								Robot->Move = Robot_Straignt_Move;
								Robot->State = Robot_Running_State;
							}
							else if(Remote_DataPack.R_X_rocker < 500 || Remote_DataPack.R_X_rocker > 3500){	//判断为拐弯动作
								Robot->Move = Robot_Turn_Move;
								Robot->State = Robot_Running_State;																																												
							}
							break;
						}
						/* 自旋 */
						else if(Remote_DataPack.L_X_rocker < 500 || Remote_DataPack.L_X_rocker > 3500){
							Robot->Move = Robot_Circle_Move;
							Robot->State = Robot_Running_State;
							break;
						}
//						/* 高栏跳跃 */
//						else if(Remote_Last_DataPack.Key.Right_Key_Up && Remote_DataPack.Key.Right_Key_Up == 0){
//							Robot->Move = Robot_Jump_Move;
//							Robot->State = Robot_Running_State;
//						}
						else if(Remote_DataPack.Key.Left_Key_Right){
							Robot->Move = Robot_Step_Move;
							Robot->State  = Robot_Running_State;
						}
						else{
							count = 0;
						}
					}
				}
				break;
			}
			case Robot_Shutdown_Mode:{
				break;
			}
		}
	}
}

void Robot_Move_State_Reset_Stop(Robot_Handle *Robot){
	Robot->Move = Robot_Stop_Move;
	Robot->State = Robot_Stop_State;
}

void Robot_Move_Master(Robot_Handle *Robot)
{
	switch(Robot->Mode){
		case Robot_Init_Mode:{
			break;
		}
		case Robot_Open_Motor_Mode:{
			static uint32_t mailbox;
			if(!Robot->Open_Motor_State){				//开启电机
				HAL_CAN_AddTxMessage(&hcan1,&CAN_TX[Robot->can_list*2],Motor_Mode_data,&mailbox);
				HAL_CAN_AddTxMessage(&hcan2,&CAN_TX[Robot->can_list*2+4],Motor_Mode_data,&mailbox);
				HAL_CAN_AddTxMessage(&hcan1,&CAN_TX[Robot->can_list*2+1],Motor_Mode_data,&mailbox);
				HAL_CAN_AddTxMessage(&hcan2,&CAN_TX[Robot->can_list*2+5],Motor_Mode_data,&mailbox);
				Robot->can_list++;
				if(Robot->can_list == 2){
					Robot->can_list = 0;
					Robot->Open_Motor_State = 1;							//已开启电机
				}
			}
			else{																					//持续发零数据
				Robot->Change_Mode = 1;											//锁状态					
				HT_03_Motor_Data_packet(CAN_TX_Data[Robot->can_list*2],&HT_03_Zero_Motor);
				HT_03_Motor_Data_packet(CAN_TX_Data[Robot->can_list*2+4],&HT_03_Zero_Motor);
				HT_03_Motor_Data_packet(CAN_TX_Data[Robot->can_list*2+1],&HT_03_Zero_Motor);
				HT_03_Motor_Data_packet(CAN_TX_Data[Robot->can_list*2+5],&HT_03_Zero_Motor);
				HAL_CAN_AddTxMessage(&hcan1,&CAN_TX[Robot->can_list*2],Motor_Mode_data,&mailbox);
				HAL_CAN_AddTxMessage(&hcan2,&CAN_TX[Robot->can_list*2+4],Motor_Mode_data,&mailbox);
				HAL_CAN_AddTxMessage(&hcan1,&CAN_TX[Robot->can_list*2+1],Motor_Mode_data,&mailbox);
				HAL_CAN_AddTxMessage(&hcan2,&CAN_TX[Robot->can_list*2+5],Motor_Mode_data,&mailbox);
				Robot->can_list++;
				if(Robot->can_list == 2){
					Robot->can_list = 0;
					Robot->Change_Mode = 0;										//若周期发送完成释放锁状态
				}
			}
			break;
		}
		case Robot_Start_Mode:{
			if(Robot->Change_Mode){
				if(Leg_Move_Init(Leg,&Init_Ramp)){					//锁状态
					Robot->Change_Mode = 0;
					Robot->Mode = Robot_Move_Mode;
					Robot->Move = Robot_Stop_Move;
				}
			}
			break;
		}
		case Robot_Move_Mode:{
			switch(Robot->Move){
				case Robot_Stop_Move:{
					break;
				}
				case Robot_Step_Move:{
					Robot_Move_Step(Robot,Leg,Step_Ramp,&IMU);
					break;
				}
				case Robot_Straignt_Move:{
					Robot_Move_Straight(Robot,Leg,Straight_Ramp,&IMU);
					break;
				}
				case Robot_Turn_Move:{
					Robot_Move_Turn(Robot,Leg,Turn_Ramp,&IMU);
					break;
				}
				case Robot_Circle_Move:{
					Robot_Move_Circle(Robot,Leg,Circle_Ramp,&IMU);
					break;
				}
				case Robot_Jump_Move:{
					Robot_Move_Jump(Robot,Leg,&IMU);
					break;
				}
				case Robot_Climb_Move:{
					
					break;
				}
				case Robot_Double_bridge_Move:{
					break;
				}
				case Robot_Seesaw_Move:{
					break;
				}
				case Robot_Stair_Move:{
					break;
				}
			}
			break;
		}
		case Robot_Shutdown_Mode:{
			static uint8_t shutdown_flag;
			if(!shutdown_flag){
				if(Leg_Shutdown_Ramp(Leg,&Shutdown_Ramp)){
					shutdown_flag = 1;
				}
			}
			break;
		}
	}
}

/* 直走 */
void Robot_Move_Straight(Robot_Handle *Robot,Leg_Handle *Leg,Ramp_Typedef *Ramp,Dev_Handle *IMU){
	static uint8_t Part_State;																				//状态
	static uint16_t initial_L_Y;																			//数值
	static uint8_t start_flag;																				//开始标志
	static float source_y = 60.0f;																		//y方向距离
	static float source_walk_z = 50.0f;																//z方向距离								//80.0f						
	static float line_y;																							//抬脚/后移 y方向距离
	static float line_z;																							//后移		  y方向距离
	static float walk_z;																							//抬脚高度		z方向抬腿最高点		
	if(!start_flag){																									
		start_flag = 1;																									
		initial_L_Y = Remote_DataPack.L_Y_rocker;
		line_y = initial_L_Y < 500 ? source_y : -source_y;
		walk_z = source_walk_z;
	}
	if(Part_State == 0){
		Leg_New_Walk(line_y,walk_z,&Leg[0],&Ramp[0]);
		Leg_New_Walk(line_y,walk_z,&Leg[3],&Ramp[3]);
		Leg_Point_RTO_Ramp(-line_y,line_z,&Leg[1],&Ramp[1]);
		Leg_Point_RTO_Ramp(-line_y,line_z,&Leg[2],&Ramp[2]);
		for(uint8_t i = 0;i<4;i++){
			Leg_Angle_Control(&Leg[i]);
		}
		if(Slope(&Ramp[0]) == 1.0f && Slope(&Straight_Ramp[1]) == 1.0f &&
			 Slope(&Ramp[2]) == 1.0f && Slope(&Straight_Ramp[3]) == 1.0f){
			ResetSlope(&Ramp[0]);
			ResetSlope(&Ramp[1]);
			ResetSlope(&Ramp[2]);
			ResetSlope(&Ramp[3]);
			Part_State = 1;
		}
	}
	else if(Part_State == 1){
		Leg_New_Walk(line_y,walk_z,&Leg[1],&Ramp[1]);
		Leg_New_Walk(line_y,walk_z,&Leg[2],&Ramp[2]);
		Leg_Point_RTO_Ramp(-line_y,line_z,&Leg[0],&Ramp[0]);
		Leg_Point_RTO_Ramp(-line_y,line_z,&Leg[3],&Ramp[3]);
		for(uint8_t i=0;i<4;i++){
			Leg_Angle_Control(&Leg[i]);
		}
		if(Slope(&Ramp[0]) == 1.0f && Slope(&Ramp[1]) == 1.0f &&
			 Slope(&Ramp[2]) == 1.0f && Slope(&Ramp[3]) == 1.0f){
			ResetSlope(&Ramp[0]);
			ResetSlope(&Ramp[1]);
			ResetSlope(&Ramp[2]);
			ResetSlope(&Ramp[3]);
			Part_State = 0;
			start_flag = 0;								//置位
			if(Remote_DataPack.L_Y_rocker > 500  || Remote_DataPack.L_Y_rocker < 3500 || Remote_DataPack.R_X_rocker < 500 || Remote_DataPack.R_X_rocker > 3500){ //判断是否继续动作
				Robot_Move_State_Reset_Stop(Robot);											//状态归位0
			}
		}
	}
}


/* 拐弯 */
void Robot_Move_Turn(Robot_Handle *Robot,Leg_Handle *Leg,Ramp_Typedef *Ramp,Dev_Handle *IMU){
	static uint8_t start_flag;
	static uint8_t Part_State;
	static uint16_t initial_R_X;
	static float short_y = 30.0f;									//短步
	static float long_y  = 60.0f;									//长步
	static float walk_z  = 40.0f;									//抬步最高点
	static float line_z  = 0.0f;									//
	static float L_Y;
	static float R_Y;

	if(!start_flag){
		initial_R_X = Remote_DataPack.R_X_rocker;
		L_Y = initial_R_X < 500 	? short_y : long_y;
		R_Y = initial_R_X > 3500 	? short_y : long_y;
		start_flag = 1;
	}
	if(Part_State == 0){
		if(initial_R_X < 500){															//左拐 前左脚先迈
			Leg_New_Walk(L_Y,walk_z,&Leg[0],&Ramp[0]);
			Leg_New_Walk(R_Y,walk_z,&Leg[3],&Ramp[3]);
			Leg_Point_RTO_Ramp(-L_Y,line_z,&Leg[2],&Ramp[2]);
			Leg_Point_RTO_Ramp(-R_Y,line_z,&Leg[1],&Ramp[1]);
		}
		else{																								//右拐 前右脚先迈
			Leg_New_Walk(L_Y,walk_z,&Leg[2],&Ramp[2]);
			Leg_New_Walk(R_Y,walk_z,&Leg[1],&Ramp[1]);
			Leg_Point_RTO_Ramp(-L_Y,line_z,&Leg[0],&Ramp[0]);
			Leg_Point_RTO_Ramp(-R_Y,line_z,&Leg[3],&Ramp[3]);
		}
		for(uint8_t i = 0;i<4;i++){
			Leg_Angle_Control(&Leg[i]);
		}
		if(Slope(&Ramp[0]) == 1.0f && Slope(&Ramp[1]) == 1.0f &&
			 Slope(&Ramp[2]) == 1.0f && Slope(&Ramp[3]) == 1.0f){
			for(uint8_t i =0;i<4;i++){
				ResetSlope(&Ramp[i]);
			}
			Part_State = 1;
		}
	}
	else if(Part_State == 1){
		if(initial_R_X < 500){
			Leg_New_Walk(L_Y,walk_z,&Leg[2],&Ramp[2]);
			Leg_New_Walk(R_Y,walk_z,&Leg[1],&Ramp[1]);
			Leg_Point_RTO_Ramp(-L_Y,line_z,&Leg[0],&Ramp[0]);
			Leg_Point_RTO_Ramp(-R_Y,line_z,&Leg[3],&Ramp[3]);
		}
		else{
			Leg_New_Walk(L_Y,walk_z,&Leg[0],&Ramp[0]);
			Leg_New_Walk(R_Y,walk_z,&Leg[3],&Ramp[3]);
			Leg_Point_RTO_Ramp(-L_Y,line_z,&Leg[2],&Ramp[2]);
			Leg_Point_RTO_Ramp(-R_Y,line_z,&Leg[1],&Ramp[1]);
		}
		for(uint8_t i = 0;i<4;i++){
			Leg_Angle_Control(&Leg[i]);
		}
		if(Slope(&Ramp[0]) == 1.0f && Slope(&Ramp[1]) == 1.0f &&
			 Slope(&Ramp[2]) == 1.0f && Slope(&Ramp[3]) == 1.0f){
			for(uint8_t i = 0;i<4;i++){
				ResetSlope(&Ramp[i]);
			}
			Part_State = 0;
			start_flag = 0;																													//可能检测到换边拐弯信号
			initial_R_X = Remote_DataPack.R_X_rocker;
			if(initial_R_X > 500 || initial_R_X < 3500){														//检测到停止信号，状态归位
				Robot_Move_State_Reset_Stop(Robot);
			}
		}
	}
}

/* 踏步 */
void Robot_Move_Step(Robot_Handle *Robot,Leg_Handle *Leg,Ramp_Typedef *Ramp,Dev_Handle *IMU)
{
	static uint8_t Part_State;
	static float step_z = 20.0f;
	if(Part_State == 0){
		Leg_Point_RTO_Ramp(0,step_z,&Leg[0],&Ramp[0]);
		Leg_Point_RTO_Ramp(0,step_z,&Leg[3],&Ramp[3]);
		if(Slope(&Ramp[0]) ==1.0f && Slope(&Ramp[3]) == 1.0f){
			ResetSlope(&Ramp[0]);
			ResetSlope(&Ramp[3]);
			Part_State = 1;
		}
	}
	else if(Part_State == 1){
		Leg_Point_RTO_Ramp(0,-step_z,&Leg[0],&Ramp[0]);
		Leg_Point_RTO_Ramp(0,-step_z,&Leg[3],&Ramp[3]);
		if(Slope(&Ramp[0]) == 1.0f && Slope(&Ramp[3]) == 1.0f){
			ResetSlope(&Ramp[0]);
			ResetSlope(&Ramp[3]);
			Part_State = 2;
		}
	}
	else if(Part_State == 2){
		Leg_Point_RTO_Ramp(0,step_z,&Leg[1],&Ramp[1]);
		Leg_Point_RTO_Ramp(0,step_z,&Leg[2],&Ramp[2]);
		if(Slope(&Ramp[1]) == 1.0f && Slope(&Ramp[2]) == 1.0f){
			ResetSlope(&Ramp[1]);
			ResetSlope(&Ramp[2]);
			Part_State = 3;
		}
	}
	else if(Part_State == 3){
		Leg_Point_RTO_Ramp(0,-step_z,&Leg[1],&Ramp[1]);
		Leg_Point_RTO_Ramp(0,-step_z,&Leg[2],&Ramp[2]);
		if(Slope(&Ramp[1]) == 1.0f && Slope(&Ramp[2]) == 1.0f){
			ResetSlope(&Ramp[1]);
			ResetSlope(&Ramp[2]);
			Part_State = 0;
			Robot_Move_State_Reset_Stop(Robot);											//状态置位
		}
	}
	
}
/* 原地自转 */
void Robot_Move_Circle(Robot_Handle *Robot,Leg_Handle *Leg,Ramp_Typedef *Ramp,Dev_Handle *IMU){
	static uint8_t start_flag;
	static uint16_t initial_L_X;
	static uint8_t Part_State;
	static float line_y = 40.0f;
	static float walk_z = 40.0f;
	static float line_z = 0;
	static float L_Y;
	static float R_Y;
	if(!start_flag){
		initial_L_X = Remote_DataPack.L_X_rocker;
		L_Y = initial_L_X < 500 	? -line_y : line_y;
		R_Y = initial_L_X > 3500  ? -line_y : line_y;
		start_flag = 1;
	}
	if(Part_State == 0){
		Leg_New_Walk(L_Y,walk_z,&Leg[0],&Ramp[0]);
		Leg_New_Walk(R_Y,walk_z,&Leg[3],&Ramp[3]);
		Leg_Point_RTO_Ramp(-L_Y,line_z,&Leg[2],&Ramp[2]);
		Leg_Point_RTO_Ramp(-R_Y,line_z,&Leg[1],&Ramp[1]);
		for(uint8_t i = 0;i<4;i++){
			Leg_Angle_Control(&Leg[i]);
		}
		if(Slope(&Ramp[0]) == 1.0f && Slope(&Ramp[1]) == 1.0f && 
			 Slope(&Ramp[2]) == 1.0f && Slope(&Ramp[3]) == 1.0f){
			ResetSlope(&Ramp[0]);
			ResetSlope(&Ramp[1]);
			ResetSlope(&Ramp[2]);
			ResetSlope(&Ramp[3]);
			Part_State = 1;
		}
	}
	else if(Part_State == 1){
		Leg_New_Walk(L_Y,walk_z,&Leg[2],&Ramp[2]);
		Leg_New_Walk(R_Y,walk_z,&Leg[1],&Ramp[1]);
		Leg_Point_RTO_Ramp(-L_Y,line_z,&Leg[0],&Ramp[0]);
		Leg_Point_RTO_Ramp(-R_Y,line_z,&Leg[3],&Ramp[3]);
		for(uint8_t i = 0;i<4;i++){
			Leg_Angle_Control(&Leg[i]);
		}
		if(Slope(&Ramp[0]) == 1.0f && Slope(&Ramp[1]) == 1.0f &&
			 Slope(&Ramp[2]) == 1.0f && Slope(&Ramp[3]) == 1.0f){
			ResetSlope(&Ramp[0]);
			ResetSlope(&Ramp[1]);
			ResetSlope(&Ramp[2]);
			ResetSlope(&Ramp[3]);
			Part_State = 0;
			start_flag = 0;
			Robot_Move_State_Reset_Stop(Robot);
		}
	}
}

/* 跳跃 */
void Robot_Move_Jump(Robot_Handle *Robot,Leg_Handle *Leg,Dev_Handle *IMU){
	static float ready_y = -60.0f;								//准备起跳位置y		-60.0f				//以下几个系数均有关系
	static float ready_z = 90.0f;									//准备起跳位置z		150.0f
	static float jump_up_y	= -60.0f;							//起跳位置y				-60.0f
	static float jump_up_z  = -220.0f;						//起跳位置z				-150.0f
	static float jump_up_back_y = 120.0f;					//起跳后腿收回			120.0f
	static float jump_up_back_z = 220.0f;					//跳跃后腿收回			150.0f
	static float jump_down_buffer_y = 120.0f;			//跳跃缓冲y				120.0f
	static float jump_down_buffer_z = -180.0f;		//跳跃缓冲z				-150.0f
	static float jump_back_y = -120.0f;						//返回原位					-120.0f
	static float jump_back_z = 30.0f;							//返回原位					0.0f
	static float Part_State;											//状态
	/* 准备起跳 */
	if(Part_State == 0){													
		if(!Jump_ready_Ramp.flag){
			for(uint8_t i = 0;i<4;i++){
				Leg_Point_RTO(ready_y,ready_z,&Leg[i]);
				Leg_Angle_Control(&Leg[i]);
			}
			Slope(&Jump_ready_Ramp);										// 开启斜坡后flag置1
		}
		else{
			if(Slope(&Jump_ready_Ramp) == 1.0f){				// 达到缓冲时间
				ResetSlope(&Jump_ready_Ramp);							// 复位
				Part_State = 1;
			}
		}
	}
	
	/* 起跳 */
	else if(Part_State == 1){							
		if(!Jump_up_Ramp.flag){
			for(uint8_t i = 0;i<4;i++){
				Leg_Point_RTO(jump_up_y,jump_up_z,&Leg[i]);
				Leg_Angle_Control(&Leg[i]);
			}
			Slope(&Jump_up_Ramp);
		}
		else{
			if(Slope(&Jump_up_Ramp) == 1.0f){
				ResetSlope(&Jump_up_Ramp);
				Part_State = 2;
			}
		}
	}
	/* 起跳后收腿 */
	else if(Part_State == 2){
		if(!Jump_up_back_Ramp.flag){
			for(uint8_t i = 0;i<4;i++){
				Leg_Point_RTO(jump_up_back_y,jump_up_back_z,&Leg[i]);
				Leg_Angle_Control(&Leg[i]);
			}
			Slope(&Jump_up_back_Ramp);
		}
		else{
			if(Slope(&Jump_up_back_Ramp) == 1.0f){
				ResetSlope(&Jump_up_back_Ramp);
				Part_State = 3;
			}
		}
	}
	/* 落地缓冲 */
	else if(Part_State == 3){
		for(uint8_t i = 0;i<4;i++){
			Leg_Point_RTO_Ramp(jump_down_buffer_y,jump_down_buffer_z,&Leg[i],&Jump_down_buffer_Ramp[i]);
			Leg_Angle_Control(&Leg[i]);
		}
		if(Slope(&Jump_down_buffer_Ramp[0]) == 1.0f && Slope(&Jump_down_buffer_Ramp[1]) == 1.0f && 
			 Slope(&Jump_down_buffer_Ramp[2]) == 1.0f && Slope(&Jump_down_buffer_Ramp[3]) == 1.0f){
			ResetSlope(&Jump_down_buffer_Ramp[0]);
			ResetSlope(&Jump_down_buffer_Ramp[1]);
			ResetSlope(&Jump_down_buffer_Ramp[2]);
			ResetSlope(&Jump_down_buffer_Ramp[3]);
			Part_State = 4;
		}
	}
	/* 归位 */										
	else if(Part_State == 4){
		Leg_Point_RTO_Ramp(jump_back_y,jump_back_z,&Leg[0],&Jump_back_buffer_Ramp[0]);
		Leg_Point_RTO_Ramp(jump_back_y,jump_back_z,&Leg[1],&Jump_back_buffer_Ramp[1]);
		Leg_Point_RTO_Ramp(jump_back_y,jump_back_z,&Leg[2],&Jump_back_buffer_Ramp[2]);
		Leg_Point_RTO_Ramp(jump_back_y,jump_back_z,&Leg[3],&Jump_back_buffer_Ramp[3]);
		for(uint8_t i = 0;i<4;i++){
			Leg_Angle_Control(&Leg[i]);
		}
		if(Slope(&Jump_back_buffer_Ramp[0]) == 1.0f && Slope(&Jump_back_buffer_Ramp[1]) == 1.0f && 
			 Slope(&Jump_back_buffer_Ramp[2]) == 1.0f && Slope(&Jump_back_buffer_Ramp[3]) == 1.0f){
			ResetSlope(&Jump_back_buffer_Ramp[0]);
			ResetSlope(&Jump_back_buffer_Ramp[1]);
			ResetSlope(&Jump_back_buffer_Ramp[2]);
			ResetSlope(&Jump_back_buffer_Ramp[3]);
			Robot_Move_State_Reset_Stop(Robot);
			Part_State = 0;
		}
	}
}

