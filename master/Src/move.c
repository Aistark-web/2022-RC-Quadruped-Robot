#include "move.h"

extern Ramp_Typedef Init_Ramp;										//��ʼ��Ramp
extern Ramp_Typedef Shutdown_Ramp;								//�ػ�Ramp
extern Ramp_Typedef Step_Ramp[4];									//ԭ��̤��Ramp
extern Ramp_Typedef Straight_Ramp[4];							//ֱ��Ramp
extern Ramp_Typedef Turn_Ramp[4];									//����Ramp
extern Ramp_Typedef Circle_Ramp[4];								//ԭ��תRamp
extern Ramp_Typedef Reset_Ramp;										//��λRamp

extern Ramp_Typedef Jump_ready_Ramp[4];						//��Ծ׼��Ramp
extern Ramp_Typedef Jump_ready_buffer_Ramp;				//��Ծ׼������Ramp
extern Ramp_Typedef Jump_up_Ramp[4];							//��Ծ����Ramp
extern Ramp_Typedef Jump_up_buffer_Ramp;					//��Ծ��������Ramp
extern Ramp_Typedef Jump_up_back_Ramp[4];					//��Ծ����Ramp
extern Ramp_Typedef Jump_up_back_buffer_Ramp;			//��Ծ���ػ���Ramp
extern Ramp_Typedef Jump_down_buffer_Ramp[4];			//��Ծ����Ramp
extern Ramp_Typedef Jump_back_buffer_Ramp[4];			//��Ծ����Ramp

extern Ramp_Typedef Double_Bridge_Up_Ramp[4];											//˫ľ�����Ramp
extern Ramp_Typedef Double_Bridge_Stand_Up_Down_Ramp[4];					//˫ľ����������Ramp
extern Ramp_Typedef Double_Bridge_Stand_Up_Down_Buffer_Ramp;			//˫ľ�����»���Ramp
extern Ramp_Typedef Double_Bridge_Front_Rise_Ramp[4];							//˫ľ�Ŵ�ǰ��Ramp
extern Ramp_Typedef Double_Bridge_Back_Rise_Ramp[4];							//˫ľ�ź��̧��Ramp
extern Ramp_Typedef Double_Bridge_Rise_Buffer_Ramp;								//˫ľ��̧�Ż���Ramp
extern Ramp_Typedef Double_Bridge_Reset_Ramp[4];									//˫ľ�����һ��������y���λ Ramp
extern Ramp_Typedef Double_Bridge_Front_Down_Ramp[4];							//˫ľ����ǰ��Ramp
extern Ramp_Typedef Double_Bridge_Back_Down_Ramp[4];							//˫ľ������Ramp
extern Ramp_Typedef Double_Bridge_Down_Buffer_Ramp;								//˫ľ����Ż���Ramp
extern Ramp_Typedef Double_Bridge_Down_Ramp[4];										//˫ľ�����Ramp
extern Ramp_Typedef Double_Bridge_Straight_Ramp[4];								//˫ľ��ֱ��Ramp
extern Ramp_Typedef Double_Bridge_Turn_Ramp[4];										//˫ľ�Ź���Ramp
extern Ramp_Typedef Double_Bridge_Circle_Ramp[4];									//˫ľ��ԭ������Ramp

extern Ramp_Typedef Seesaw_Sqat_Ramp[4];									//���ΰ����Ramp
extern Ramp_Typedef Seesaw_Sqat_Back_Ramp[4];							//���ΰ���·���Ramp
extern Ramp_Typedef Seesaw_Climb_Up_Posture_Ramp[4];			//���ΰ�������̬Ramp
extern Ramp_Typedef Seesaw_Climb_Down_Posture_Ramp[4];		//���ΰ�������̬Ramp
extern Ramp_Typedef Seesaw_Climb_Buffer_Ramp;							//��������Ramp
extern Ramp_Typedef Seesaw_Sqat_Buffer_Ramp;							//���»���Ramp
extern Ramp_Typedef Seesaw_Straight_Ramp[4];							//���ΰ�ֱ��Ramp
extern Ramp_Typedef Seesaw_Turn_Ramp[4];									//���ΰ�ת��Ramp
extern Ramp_Typedef Seesaw_Circle_Ramp[4];								//���ΰ�ԭ����תRamp

extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
extern Dev_Handle IMU;
extern Remote_DataPack_Handle Remote_DataPack;			//ң��������
extern Remote_DataPack_Handle Remote_Last_DataPack;	//��һ��ң��������
extern Leg_Handle Leg[4];
extern HT_03_MOTOR_Handle HT_03_Zero_Motor;				//��״̬���
extern uint8_t CAN_TX_Data[8][8];									//CAN�������ݻ���
extern CAN_TxHeaderTypeDef CAN_TX[8];							//CAN���ͱ���
uint8_t Motor_Mode_data[8] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFC};						//�������ģʽ

/* ģʽ״̬���� */
/********************
		Mode:    Robot_Init_Mode -> Robot_Open_Motor_Mode -> Robot_Start_Mode -> Robot_Move_Mode							
*/

void Robot_Check(Robot_Handle *Robot,Remote_DataPack_Handle *Remote)
{
	if(Robot->Change_Mode == 0){									//���ڸı�ģʽʱ��1�������˽���
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
			case Robot_Start_Mode:{										//startģʽִ����Ϻ��Զ��л���moveģʽ
				break;
			}
			case Robot_Move_Mode:{
				//TODO	�˶���Ϊ�ı�
				static uint32_t count;
				if(Robot->State == Robot_Stop_State){				//ֹͣʱ���ȶ���
					if(Robot->Move == Robot_Stop_Move){
						if(Remote_DataPack.Key.Left_Rocker && Remote_DataPack.Key.Right_Rocker){
							if(++count >= 160){									//��Լ2s
								count = 0;
								Robot->Mode = Robot_Shutdown_Mode;		//����ػ�ģʽ
								Robot->Change_Mode = 1;
							}
							break;
						}
						/* ˫ľ�� */
						else if(Remote_DataPack.Key.Left_Switch_Up){							
							Robot->Move 	= Robot_Double_bridge_Move;
							Robot->State	=	Robot_Running_State;
						}
						/* ���ΰ� */
						else if(Remote_DataPack.Key.Right_Switch_Up){
							Robot->Move 	= Robot_Seesaw_Move;
							Robot->State	=	Robot_Running_State;
						}
						/* ֱ��/���� */
						else if(Remote_DataPack.L_Y_rocker < 500 || Remote_DataPack.L_Y_rocker > 3500){																							
							if(Remote_DataPack.R_X_rocker > 500 && Remote_DataPack.R_X_rocker < 3500){			//�ж�Ϊֱ�ж���
								Robot->Move = Robot_Straignt_Move;
								Robot->State = Robot_Running_State;
							}
							else if(Remote_DataPack.R_X_rocker < 500 || Remote_DataPack.R_X_rocker > 3500){	//�ж�Ϊ���䶯��
								Robot->Move = Robot_Turn_Move;
								Robot->State = Robot_Running_State;																																												
							}
							break;
						}
						/* ���� */
						else if(Remote_DataPack.L_X_rocker < 500 || Remote_DataPack.L_X_rocker > 3500){
							Robot->Move = Robot_Circle_Move;
							Robot->State = Robot_Running_State;
							break;
						}
//						/* ������Ծ */
//						else if(Remote_Last_DataPack.Key.Right_Key_Up && Remote_DataPack.Key.Right_Key_Up == 0){
//							Robot->Move = Robot_Jump_Move;
//							Robot->State = Robot_Running_State;
//						}
						else if(Remote_DataPack.Key.Left_Key_Right){
							Robot->Move		= Robot_Step_Move;
							Robot->State	= Robot_Running_State;
						}
						/* ��λ */
						else if(Remote_DataPack.Key.Left_Key_Left == 0 && Remote_Last_DataPack.Key.Left_Key_Left){
							Robot->Move		= Robot_Reset_Move;
							Robot->State	= Robot_Running_State;
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
			if(!Robot->Open_Motor_State){									//�������
				HAL_CAN_AddTxMessage(&hcan1,&CAN_TX[Robot->can_list*2],Motor_Mode_data,&mailbox);
				HAL_CAN_AddTxMessage(&hcan2,&CAN_TX[Robot->can_list*2+4],Motor_Mode_data,&mailbox);
				HAL_CAN_AddTxMessage(&hcan1,&CAN_TX[Robot->can_list*2+1],Motor_Mode_data,&mailbox);
				HAL_CAN_AddTxMessage(&hcan2,&CAN_TX[Robot->can_list*2+5],Motor_Mode_data,&mailbox);
				Robot->can_list++;
				if(Robot->can_list == 2){
					Robot->can_list = 0;
					Robot->Open_Motor_State = 1;							//�ѿ������
				}
			}
			else{																					//������������
				Robot->Change_Mode = 1;											//��״̬					
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
					Robot->Change_Mode = 0;										//�����ڷ�������ͷ���״̬
				}
			}
			break;
		}
		case Robot_Start_Mode:{
			if(Robot->Change_Mode){
				if(Leg_Move_Init(Leg,&Init_Ramp)){					//��״̬
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
				case Robot_Reset_Move:{
					if(Leg_Reset_Ramp(Leg,&Reset_Ramp)){											//��λ
						Robot_Move_State_Reset_Stop(Robot);											//��λ���
						Robot->double_bridge_state = Double_Bridge_None_State;	//˫ľ��״̬��λ
						Robot->seesaw_sqat_state = Seesaw_Stand_State;					//���ΰ����״̬��λ
						Robot->seesaw_climb_state = Seesaw_Climb_None_State;		//���ΰ�����״̬��λ
					}
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
//					Robot_Move_Jump(Robot,Leg,&IMU);
					break;
				}
				case Robot_Climb_Move:{
					
					break;
				}
				case Robot_Double_bridge_Move:{
					Robot_Move_Double_bridge(Robot,Leg,&IMU);
					break;
				}
				case Robot_Seesaw_Move:{
					Robot_Move_Seesaw(Robot,Leg,&IMU);
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

/* ֱ�� */
void Robot_Move_Straight(Robot_Handle *Robot,Leg_Handle *Leg,Ramp_Typedef *Ramp,Dev_Handle *IMU){
	static uint8_t Part_State;																				//״̬
	static uint16_t initial_L_Y;																			//��ֵ
	static uint8_t start_flag;																				//��ʼ��־
	static float source_y = 40.0f;																		//y�������								//60.0f
	static float source_walk_z = 30.0f;																//z�������								//50.0f 80.0f						
	static float line_y;																							//̧��/���� y�������
	static float line_z;																							//����		  y�������
	static float walk_z;																							//̧�Ÿ߶�		z����̧����ߵ�		
	if(!start_flag){																									
		start_flag = 1;																									
		initial_L_Y = Remote_DataPack.L_Y_rocker;
		line_y = initial_L_Y < 500 ? source_y : -source_y;

		walk_z = source_walk_z;
	}
	#if 0
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
			start_flag = 0;								//��λ
			if(Remote_DataPack.L_Y_rocker > 500  || Remote_DataPack.L_Y_rocker < 3500 || Remote_DataPack.R_X_rocker < 500 || Remote_DataPack.R_X_rocker > 3500){ //�ж��Ƿ��������
				Robot_Move_State_Reset_Stop(Robot);											//״̬��λ0
			}
		}
	}
	#else
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
	if(Part_State == 1){
		Leg_New_Walk(2.0f*line_y,walk_z,&Leg[1],&Ramp[1]);
		Leg_New_Walk(2.0f*line_y,walk_z,&Leg[2],&Ramp[2]);
		Leg_Point_RTO_Ramp(-2.0f*line_y,line_z,&Leg[0],&Ramp[0]);
		Leg_Point_RTO_Ramp(-2.0f*line_y,line_z,&Leg[3],&Ramp[3]);
		for(uint8_t i = 0;i<4;i++){
			Leg_Angle_Control(&Leg[i]);
		}
		if(Slope(&Ramp[0]) == 1.0f && Slope(&Straight_Ramp[1]) == 1.0f &&
			 Slope(&Ramp[2]) == 1.0f && Slope(&Straight_Ramp[3]) == 1.0f){
			ResetSlope(&Ramp[0]);
			ResetSlope(&Ramp[1]);
			ResetSlope(&Ramp[2]);
			ResetSlope(&Ramp[3]);
			Part_State = 2;
		}
	}
	else if(Part_State == 2){
		Leg_New_Walk(2.0f*line_y,walk_z,&Leg[0],&Ramp[0]);
		Leg_New_Walk(2.0f*line_y,walk_z,&Leg[3],&Ramp[3]);
		Leg_Point_RTO_Ramp(-2.0f*line_y,line_z,&Leg[1],&Ramp[1]);
		Leg_Point_RTO_Ramp(-2.0f*line_y,line_z,&Leg[2],&Ramp[2]);
		for(uint8_t i=0;i<4;i++){
			Leg_Angle_Control(&Leg[i]);
		}
		if(Slope(&Ramp[0]) == 1.0f && Slope(&Ramp[1]) == 1.0f &&
			 Slope(&Ramp[2]) == 1.0f && Slope(&Ramp[3]) == 1.0f){
			ResetSlope(&Ramp[0]);
			ResetSlope(&Ramp[1]);
			ResetSlope(&Ramp[2]);
			ResetSlope(&Ramp[3]);
			if((Remote_DataPack.R_X_rocker > 500 && Remote_DataPack.R_X_rocker < 3500) &&
				 ((Remote_DataPack.L_Y_rocker < 500 && initial_L_Y < 500) || (Remote_DataPack.L_Y_rocker > 3500 && initial_L_Y > 3500)) ){ //�ж��Ƿ��������
				Part_State = 3;
			}
			else{
				Part_State = 1;
			}
		}
	}
	else if(Part_State == 3){
		Leg_New_Walk(line_y,walk_z,&Leg[1],&Ramp[1]);
		Leg_New_Walk(line_y,walk_z,&Leg[2],&Ramp[2]);
		Leg_Point_RTO_Ramp(-line_y,line_z,&Leg[0],&Ramp[0]);
		Leg_Point_RTO_Ramp(-line_y,line_z,&Leg[3],&Ramp[3]);
		for(uint8_t i=0;i<4;i++){
			Leg_Angle_Control(&Leg[i]);
		}
		if(Slope(&Ramp[0]) == 1.0f && Slope(&Ramp[1]) == 1.0f && Slope(&Ramp[2]) == 1.0f && Slope(&Ramp[3]) == 1.0f){
			ResetSlope(&Ramp[0]);
			ResetSlope(&Ramp[1]);
			ResetSlope(&Ramp[2]);
			ResetSlope(&Ramp[3]);
			Part_State = 0;
			Robot_Move_State_Reset_Stop(Robot);											//״̬��λ0
			start_flag = 0;																					//��λ
		}
	}
	#endif
}


/* ���� */
void Robot_Move_Turn(Robot_Handle *Robot,Leg_Handle *Leg,Ramp_Typedef *Ramp,Dev_Handle *IMU){
	static uint8_t start_flag;
	static uint8_t Part_State;
	static uint16_t initial_R_X;
	static uint16_t initial_L_Y;
	static float source_short_y = 15.0f;					//�̲�					//30.0f
	static float source_long_y  = 30.0f;					//����					//60.0f
	static float short_y;
	static float long_y;
	static float walk_z  = 30.0f;									//̧����ߵ�			//40.0f
	static float line_z  = 0.0f;									//
	static float L_Y;
	static float R_Y;
	if(!start_flag){
		initial_R_X = Remote_DataPack.R_X_rocker;
		initial_L_Y = Remote_DataPack.L_Y_rocker;
		short_y = initial_L_Y < 500 ? source_short_y 	: -source_short_y;
		long_y 	= initial_L_Y < 500 ? source_long_y		: -source_long_y;
		L_Y = initial_R_X < 500 	? short_y : long_y;
		R_Y = initial_R_X > 3500 	? short_y : long_y;
		start_flag = 1;
	}
	#if 0
	if(Part_State == 0){
		if(initial_R_X < 500){															//��� ǰ�������
			Leg_New_Walk(L_Y,walk_z,&Leg[0],&Ramp[0]);
			Leg_New_Walk(R_Y,walk_z,&Leg[3],&Ramp[3]);
			Leg_Point_RTO_Ramp(-L_Y,line_z,&Leg[2],&Ramp[2]);
			Leg_Point_RTO_Ramp(-R_Y,line_z,&Leg[1],&Ramp[1]);
		}
		else{																								//�ҹ� ǰ�ҽ�����
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
			start_flag = 0;																													//���ܼ�⵽���߹����ź�
			initial_R_X = Remote_DataPack.R_X_rocker;
			if(initial_R_X > 500 || initial_R_X < 3500){														//��⵽ֹͣ�źţ�״̬��λ
				Robot_Move_State_Reset_Stop(Robot);
			}
		}
	}
	#else
	if(Part_State == 0){
		if(initial_R_X < 500){															//��� ǰ�������
			Leg_New_Walk(L_Y,walk_z,&Leg[0],&Ramp[0]);
			Leg_New_Walk(R_Y,walk_z,&Leg[3],&Ramp[3]);
			Leg_Point_RTO_Ramp(-L_Y,line_z,&Leg[2],&Ramp[2]);
			Leg_Point_RTO_Ramp(-R_Y,line_z,&Leg[1],&Ramp[1]);
		}
		else{																								//�ҹ� ǰ�ҽ�����
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
			Leg_New_Walk(2.0f*L_Y,walk_z,&Leg[2],&Ramp[2]);
			Leg_New_Walk(2.0f*R_Y,walk_z,&Leg[1],&Ramp[1]);
			Leg_Point_RTO_Ramp(-2.0f*L_Y,line_z,&Leg[0],&Ramp[0]);
			Leg_Point_RTO_Ramp(-2.0f*R_Y,line_z,&Leg[3],&Ramp[3]);
		}
		else{
			Leg_New_Walk(2.0f*L_Y,walk_z,&Leg[0],&Ramp[0]);
			Leg_New_Walk(2.0f*R_Y,walk_z,&Leg[3],&Ramp[3]);
			Leg_Point_RTO_Ramp(-2.0f*L_Y,line_z,&Leg[2],&Ramp[2]);
			Leg_Point_RTO_Ramp(-2.0f*R_Y,line_z,&Leg[1],&Ramp[1]);
		}
		for(uint8_t i = 0;i<4;i++){
			Leg_Angle_Control(&Leg[i]);
		}
		if(Slope(&Ramp[0]) == 1.0f && Slope(&Ramp[1]) == 1.0f &&
			 Slope(&Ramp[2]) == 1.0f && Slope(&Ramp[3]) == 1.0f){
			for(uint8_t i = 0;i<4;i++){
				ResetSlope(&Ramp[i]);
			}
			Part_State = 2;
		}
	}
	else if(Part_State == 2){
		if(initial_R_X < 500){
			Leg_New_Walk(2.0f*L_Y,walk_z,&Leg[0],&Ramp[0]);
			Leg_New_Walk(2.0f*R_Y,walk_z,&Leg[3],&Ramp[3]);
			Leg_Point_RTO_Ramp(-2.0f*L_Y,line_z,&Leg[2],&Ramp[2]);
			Leg_Point_RTO_Ramp(-2.0f*R_Y,line_z,&Leg[1],&Ramp[1]);
		}
		else{
			Leg_New_Walk(2.0f*L_Y,walk_z,&Leg[2],&Ramp[2]);
			Leg_New_Walk(2.0f*R_Y,walk_z,&Leg[1],&Ramp[1]);
			Leg_Point_RTO_Ramp(-2.0f*L_Y,line_z,&Leg[0],&Ramp[0]);
			Leg_Point_RTO_Ramp(-2.0f*R_Y,line_z,&Leg[3],&Ramp[3]);
		}
		for(uint8_t i = 0;i<4;i++){
			Leg_Angle_Control(&Leg[i]);
		}
		if(Slope(&Ramp[0]) == 1.0f && Slope(&Ramp[1]) == 1.0f &&
			 Slope(&Ramp[2]) == 1.0f && Slope(&Ramp[3]) == 1.0f){
			for(uint8_t i = 0;i<4;i++){
				ResetSlope(&Ramp[i]);
			}
			if(((Remote_DataPack.L_Y_rocker < 500 && initial_L_Y < 500) || (Remote_DataPack.L_Y_rocker > 3500 && initial_L_Y > 3500)) &&
				 ((Remote_DataPack.R_X_rocker < 500 && initial_R_X < 500) || (Remote_DataPack.R_X_rocker > 3500 && initial_R_X > 3500))){
				Part_State = 1;
			}
			else
				Part_State = 3;
		}
	}
	else if(Part_State == 3){
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
			start_flag = 0;																													
			Robot_Move_State_Reset_Stop(Robot);
		}
	}
	#endif
}

/* ̤�� */
void Robot_Move_Step(Robot_Handle *Robot,Leg_Handle *Leg,Ramp_Typedef *Ramp,Dev_Handle *IMU)
{
	static uint8_t Part_State;
	static float step_z = 20.0f;
	if(Part_State == 0){
		Leg_Point_RTO_Ramp(0,step_z,&Leg[0],&Ramp[0]);
		Leg_Point_RTO_Ramp(0,step_z,&Leg[3],&Ramp[3]);
		Leg_Angle_Control(&Leg[0]);
		Leg_Angle_Control(&Leg[3]);
		if(Slope(&Ramp[0]) ==1.0f && Slope(&Ramp[3]) == 1.0f){
			ResetSlope(&Ramp[0]);
			ResetSlope(&Ramp[3]);
			Part_State = 1;
		}
	}
	else if(Part_State == 1){
		Leg_Point_RTO_Ramp(0,-step_z,&Leg[0],&Ramp[0]);
		Leg_Point_RTO_Ramp(0,-step_z,&Leg[3],&Ramp[3]);
		Leg_Angle_Control(&Leg[0]);
		Leg_Angle_Control(&Leg[3]);
		if(Slope(&Ramp[0]) == 1.0f && Slope(&Ramp[3]) == 1.0f){
			ResetSlope(&Ramp[0]);
			ResetSlope(&Ramp[3]);
			Part_State = 2;
		}
	}
	else if(Part_State == 2){
		if(Slope(&Ramp[0]) == 1.0f){
			ResetSlope(&Ramp[0]);
			Part_State = 3;
		}
	}
	else if(Part_State == 3){
		Leg_Point_RTO_Ramp(0,step_z,&Leg[1],&Ramp[1]);
		Leg_Point_RTO_Ramp(0,step_z,&Leg[2],&Ramp[2]);
		Leg_Angle_Control(&Leg[1]);
		Leg_Angle_Control(&Leg[2]);
		if(Slope(&Ramp[1]) == 1.0f && Slope(&Ramp[2]) == 1.0f){
			ResetSlope(&Ramp[1]);
			ResetSlope(&Ramp[2]);
			Part_State = 4;
		}
	}
	else if(Part_State == 4){
		Leg_Point_RTO_Ramp(0,-step_z,&Leg[1],&Ramp[1]);
		Leg_Point_RTO_Ramp(0,-step_z,&Leg[2],&Ramp[2]);
		Leg_Angle_Control(&Leg[1]);
		Leg_Angle_Control(&Leg[2]);
		if(Slope(&Ramp[1]) == 1.0f && Slope(&Ramp[2]) == 1.0f){
			ResetSlope(&Ramp[1]);
			ResetSlope(&Ramp[2]);
			Part_State = 0;
			Robot_Move_State_Reset_Stop(Robot);											//״̬��λ
		}
	}
	
}
/* ԭ����ת */
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

/* ��Ծ */
void Robot_Move_Jump(Robot_Handle *Robot,Leg_Handle *Leg,Dev_Handle *IMU){
	#if 1
	static float ready_y = 0;
	static float ready_z = 50.0f;
	static float jump_up_y = 0;
	static float jump_up_z = -180.0f;
	static float jump_up_back_y = 0;
	static float jump_up_back_z = 180.0f;
	static float jump_down_buffer_y = 0;
	static float jump_down_buffer_z = -100.0f;
	static float jump_back_y = 0;
	static float jump_back_z = 50.0f;
	static float jump_up_torque_add = 15.0f;				//������������
	static float jump_up_back_torque_add = 15.0f;		//������������
	#else
	
	static float ready_y = -40.0f;								//׼������λ��y		-40.0f				//���¼���ϵ�����й�ϵ
	static float ready_z = 50.0f;									//׼������λ��z		50.0f
	static float jump_up_y	= -70.0f;							//����λ��y				-40.0f
	static float jump_up_z  = -180.0f;						//����λ��z				-120.0f
	static float jump_up_back_y = 110.0f;					//���������ջ�			80.0f
	static float jump_up_back_z = 180.0f;					//��Ծ�����ջ�			170.0f
	static float jump_down_buffer_y = 70.0f;			//��Ծ����y				120.0f
	static float jump_down_buffer_z = -100.0f;		//��Ծ����z				-150.0f
	static float jump_back_y = -70.0f;						//����ԭλ					-120.0f
	static float jump_back_z = 50.0f;							//����ԭλ					0.0f
	#endif
	static float Part_State;											//״̬
	static uint8_t start_flag;										//��ʼ��־λ
	/* ׼������ */
	if(Part_State == 0){
		if(!start_flag){
			for(uint8_t i = 0;i<4;i++){
				Leg_Point_RTO(ready_y,ready_z,&Leg[i]);
				Leg_Angle_Control(&Leg[i]);
			}
			start_flag = 1;
		}
		else{
			if(Slope(&Jump_ready_buffer_Ramp) == 1.0f){
				ResetSlope(&Jump_ready_buffer_Ramp);
				start_flag = 0;
				Part_State = 1;
			}
		}
	}

	/* ���� */
	else if(Part_State == 1){
		if(!start_flag){
			for(uint8_t i = 0;i<4;i++){
				Leg_Point_RTO(jump_up_y,jump_up_z,&Leg[i]);
				Leg_Angle_Control(&Leg[i]);
			}
			Leg[0].Motor_1->Expect.E_T = -jump_up_torque_add;
			Leg[0].Motor_2->Expect.E_T = -jump_up_torque_add;
			Leg[1].Motor_1->Expect.E_T = jump_up_torque_add;
			Leg[1].Motor_2->Expect.E_T = jump_up_torque_add;
			Leg[2].Motor_1->Expect.E_T = -jump_up_torque_add;
			Leg[2].Motor_2->Expect.E_T = -jump_up_torque_add;
			Leg[3].Motor_1->Expect.E_T = jump_up_torque_add;
			Leg[3].Motor_2->Expect.E_T = jump_up_torque_add;
			start_flag = 1;
			//��������
			Robot->Torque_Control = 1;
		}
		else{
			if(Slope(&Jump_up_buffer_Ramp) == 1.0f){
				ResetSlope(&Jump_up_buffer_Ramp);
				start_flag = 0;
				Part_State = 2;
			}
		}
	}

	/* ���������� */
	else if(Part_State == 2){
		if(!start_flag){
			for(uint8_t i = 0;i<4;i++){
				Leg_Point_RTO(jump_up_back_y,jump_up_back_z,&Leg[i]);
				Leg_Angle_Control(&Leg[i]);
			}
			Leg[0].Motor_1->Expect.E_T = jump_up_back_torque_add;
			Leg[0].Motor_2->Expect.E_T = jump_up_back_torque_add;
			Leg[1].Motor_1->Expect.E_T = -jump_up_back_torque_add;
			Leg[1].Motor_2->Expect.E_T = -jump_up_back_torque_add;
			Leg[2].Motor_1->Expect.E_T = jump_up_back_torque_add;
			Leg[2].Motor_2->Expect.E_T = jump_up_back_torque_add;
			Leg[3].Motor_1->Expect.E_T = -jump_up_back_torque_add;
			Leg[3].Motor_2->Expect.E_T = -jump_up_back_torque_add;
			
			start_flag = 1;
		}
		else{
			if(Slope(&Jump_up_back_buffer_Ramp) == 1.0f){
				ResetSlope(&Jump_up_back_buffer_Ramp);
				start_flag = 0;
				Part_State = 3;												
				Robot->Torque_Control = 0;						//�ر����ؿ���
			}
		}
	}
	/* ��ػ��� */
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
	/* ��λ */										
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

/* ˫ľ�� */
void Robot_Move_Double_bridge(Robot_Handle *Robot,Leg_Handle *Leg,Dev_Handle *IMU){
	static Remote_DataPack_Handle initial_Remote;
	static uint8_t start_flag;					
	static uint8_t Part_State;						//״̬
	static uint8_t Div_State;							//״̬��̬
	static float line_z = 100.0f;					//��ֱ�����/���z
	static float stand_up_down_z = 80.0f; //̧���߶�
	static float rise_y = 100.0f;					//����̧��y
	static float rise_z = 50.0f;					//����̧��z
	static float down_y = 100.0f;					//�������y
	static float down_z = 50.0f;					//�������z
	static float rise_up_z = 30.0f;				//����̧��z
	
	
	static float straight_source_walk_y = 40.0f;//ԭʼֱ��y
	static float straight_walk_y;								//ֱ��y
	static float straight_walk_z = 30.0f;				//ֱ��z
	
	static float turn_short_y = 30.0f;					//����̾�y
	static float turn_long_y = 50.0f;						//���䳤��y
	static float turn_walk_z = 50.0f;						//����z
	static float turn_walk_l_y;									//�������y
	static float turn_walk_r_y;									//�����ҽ�y
	
	static float circle_walk_y = 30.0f;					//ԭ����תy
	static float circle_walk_z = 30.0f;					//ԭ����תz
	static float circle_walk_l_y;								//ԭ����ת���y
	static float circle_walk_r_y;								//ԭ����ת�ҽ�y
	if(!start_flag){
		/* ��ߵ�����Ϊ */
		memcpy(&initial_Remote,&Remote_DataPack,21);
		/* ��˫ľ��̧ǰ�� */
		if(initial_Remote.Key.Right_Key_Up && Remote_Last_DataPack.Key.Right_Key_Up == 0 && 
			 Robot->double_bridge_state == Double_Bridge_None_State){							
			start_flag = 1;
			Robot->double_bridge_event = Double_Bridge_Front_Rise_Event;
			Robot->double_bridge_state = Double_Bridge_Front_Rise_State;
		}
		
		/* ��˫ľ��̧��� */
		else if(initial_Remote.Key.Right_Key_Down && Remote_Last_DataPack.Key.Right_Key_Down == 0 &&
						Robot->double_bridge_state == Double_Bridge_Front_Rise_State){				
			start_flag = 1;
			Robot->double_bridge_event = Double_Bridge_Back_Rise_Event;
			Robot->double_bridge_state = Double_Bridge_Back_Rise_State;
		}
		
		/* ��˫ľ��̧ǰ�� */
		else if(initial_Remote.Key.Right_Key_Left && Remote_Last_DataPack.Key.Right_Key_Left == 0 &&
						Robot->double_bridge_state == Double_Bridge_Back_Rise_State){				
			start_flag = 1;
			Robot->double_bridge_event = Double_Bridge_Front_Down_Event;
			Robot->double_bridge_state = Double_Bridge_Front_Down_State;
		}
		
		/* ��˫ľ��̧��� */
		else if(initial_Remote.Key.Right_Key_Right && Remote_Last_DataPack.Key.Right_Key_Right == 0 &&
						Robot->double_bridge_state == Double_Bridge_Front_Down_State){
			start_flag = 1;
			Robot->double_bridge_event = Double_Bridge_Back_Down_Event;
			Robot->double_bridge_state = Double_Bridge_Back_Down_State;
		}
		
		/* ֱ��/ת�� */
		else if(initial_Remote.L_Y_rocker < 500 || initial_Remote.L_Y_rocker > 3500){
			start_flag = 1;
			if(initial_Remote.R_X_rocker > 500 && initial_Remote.R_X_rocker < 3500){
				straight_walk_y = initial_Remote.L_Y_rocker < 500 ? straight_source_walk_y : - straight_source_walk_y;			//ȷ��ǰ�л����
				Robot->double_bridge_event = Double_Bridge_Straignt_Event;					//ֱ��
			}
			else{
				turn_walk_l_y = initial_Remote.R_X_rocker < 500 ? turn_short_y : turn_long_y;
				turn_walk_r_y = initial_Remote.R_X_rocker > 3500 ? turn_short_y : turn_long_y;
				Robot->double_bridge_event = Double_Bridge_Turn_Event;							//ת��
			}
		}
		/* ԭ�ع��� */
		else if(initial_Remote.L_X_rocker < 500 || initial_Remote.L_X_rocker > 3500){
			start_flag = 1;
			circle_walk_l_y = initial_Remote.L_X_rocker < 500 ? -circle_walk_y : circle_walk_y;
			circle_walk_r_y = initial_Remote.L_X_rocker > 3500 ? -circle_walk_y : circle_walk_y;
			Robot->double_bridge_event = Double_Bridge_Circle_Event;							//ԭ��ת��
		}
		else{
			Robot_Move_State_Reset_Stop(Robot);
		}
	}
	else{
		switch(Robot->double_bridge_event){
			case Double_Bridge_None_Event:{
//				Robot_Move_State_Reset_Stop(Robot);					//���޶���ָʾ��״̬��λ
				break;												
			}
			case Double_Bridge_Front_Rise_Event:{
				if(Part_State == 0){
					if(Div_State == 0){
						for(uint8_t i = 0;i<4;i++){
							Leg_Point_RTO_Ramp(0,-stand_up_down_z,&Leg[i],&Double_Bridge_Stand_Up_Down_Ramp[i]);
							Leg_Angle_Control(&Leg[i]);
						}
						if(Slope(&Double_Bridge_Stand_Up_Down_Ramp[0]) == 1.0f && Slope(&Double_Bridge_Stand_Up_Down_Ramp[1]) == 1.0f &&
							 Slope(&Double_Bridge_Stand_Up_Down_Ramp[2]) == 1.0f && Slope(&Double_Bridge_Stand_Up_Down_Ramp[3]) == 1.0f){
							for(uint8_t i = 0;i<4;i++){
								ResetSlope(&Double_Bridge_Stand_Up_Down_Ramp[i]);
							}
							Div_State = 1;
						}
					}
					else if(Div_State == 1){
						if(Slope(&Double_Bridge_Stand_Up_Down_Buffer_Ramp) == 1.0f){
							ResetSlope(&Double_Bridge_Stand_Up_Down_Buffer_Ramp);
							Div_State = 0;
							Part_State = 1;
						}
					}
				}
				
				/* ̧��ǰ��� */
				else if(Part_State ==1){
					if(Div_State == 0){
						Leg_Point_RTO_Ramp(0,line_z,&Leg[0],&Double_Bridge_Up_Ramp[0]);
						Leg_Angle_Control(&Leg[0]);
						if(Slope(&Double_Bridge_Up_Ramp[0]) == 1.0f){
							ResetSlope(&Double_Bridge_Up_Ramp[0]);
							Div_State = 1;
						}
					}
					else if(Div_State == 1){
						Leg_New_Walk(rise_y,rise_z,&Leg[0],&Double_Bridge_Front_Rise_Ramp[0]);
						Leg_Angle_Control(&Leg[0]);
						if(Slope(&Double_Bridge_Front_Rise_Ramp[0]) == 1.0f){
							ResetSlope(&Double_Bridge_Front_Rise_Ramp[0]);
							Div_State = 2;
						}
					}
					else if(Div_State == 2){
						if(Slope(&Double_Bridge_Rise_Buffer_Ramp) == 1.0f){
							ResetSlope(&Double_Bridge_Rise_Buffer_Ramp);
							Div_State = 0;
							Part_State = 2;
						}
					}
				}
				
				/* ̧��ǰ�ҽ� */
				else if(Part_State == 2){
					if(Div_State == 0){
						Leg_Point_RTO_Ramp(0,line_z,&Leg[1],&Double_Bridge_Up_Ramp[1]);
						Leg_Angle_Control(&Leg[1]);
						if(Slope(&Double_Bridge_Up_Ramp[1]) == 1.0f){
							ResetSlope(&Double_Bridge_Up_Ramp[1]);
							Div_State = 1;
						}
					}
					
					else if(Div_State == 1){
						Leg_New_Walk(rise_y,rise_z,&Leg[1],&Double_Bridge_Front_Rise_Ramp[1]);
						Leg_Angle_Control(&Leg[1]);
						if(Slope(&Double_Bridge_Front_Rise_Ramp[1]) == 1.0f){
							ResetSlope(&Double_Bridge_Front_Rise_Ramp[1]);
							Div_State = 2;
						}
					}
					
					else if(Div_State == 2){
						if(Slope(&Double_Bridge_Rise_Buffer_Ramp) == 1.0f){
							ResetSlope(&Double_Bridge_Rise_Buffer_Ramp);
							Div_State = 0;
							Part_State = 3;
						}
					}
				}
				
				/* վֱ */
				else if(Part_State == 3){
					if(Div_State == 0){
						if(Slope(&Double_Bridge_Stand_Up_Down_Buffer_Ramp) == 1.0f){
							ResetSlope(&Double_Bridge_Stand_Up_Down_Buffer_Ramp);
							Div_State = 1;
						}
					}
					else if(Div_State == 1){
						for(uint8_t i = 0;i<4;i++){
							Leg_Point_RTO_Ramp(-rise_y,0,&Leg[i],&Double_Bridge_Reset_Ramp[i]);
							Leg_Angle_Control(&Leg[i]);
						}
						if(Slope(&Double_Bridge_Reset_Ramp[0]) == 1.0f && Slope(&Double_Bridge_Reset_Ramp[1]) == 1.0f &&
							 Slope(&Double_Bridge_Reset_Ramp[2]) == 1.0f && Slope(&Double_Bridge_Reset_Ramp[3]) == 1.0f){
							for(uint8_t i = 0;i<4;i++){
								ResetSlope(&Double_Bridge_Reset_Ramp[i]);
							}
							Div_State = 2;
						}
					}
					else if(Div_State == 2){
						if(Slope(&Double_Bridge_Rise_Buffer_Ramp) == 1.0f){
							ResetSlope(&Double_Bridge_Rise_Buffer_Ramp);
							Part_State = 4;
							Div_State = 0;
						}
					}
				}
				
				/* ����Ż��� */
				if(Part_State == 4){
					if(Div_State == 0){
						Leg_New_Walk(rise_y,rise_up_z,&Leg[2],&Double_Bridge_Front_Rise_Ramp[2]);
						Leg_Angle_Control(&Leg[2]);
						if(Slope(&Double_Bridge_Front_Rise_Ramp[2]) == 1.0f){
							ResetSlope(&Double_Bridge_Front_Rise_Ramp[2]);
							Div_State = 1;
						}
					}
					else if(Div_State == 1){
						if(Slope(&Double_Bridge_Rise_Buffer_Ramp) == 1.0f){
							ResetSlope(&Double_Bridge_Rise_Buffer_Ramp);
							Div_State = 0;
							Part_State = 5;
						}
					}
			 	}
				
				/* ���ҽŻ��� */
				else if(Part_State == 5){
					if(Div_State == 0){
						Leg_New_Walk(rise_y,rise_up_z,&Leg[3],&Double_Bridge_Front_Rise_Ramp[3]);
						Leg_Angle_Control(&Leg[3]);
						if(Slope(&Double_Bridge_Front_Rise_Ramp[3]) == 1.0f){
							ResetSlope(&Double_Bridge_Front_Rise_Ramp[3]);
							Div_State = 1;
						}
					}
					else if(Div_State == 1){
						if(Slope(&Double_Bridge_Rise_Buffer_Ramp) == 1.0f){
							ResetSlope(&Double_Bridge_Rise_Buffer_Ramp);
							Div_State = 0;
							Part_State = 0;
							Robot->double_bridge_event = Double_Bridge_None_Event;
							Robot_Move_State_Reset_Stop(Robot);
							start_flag = 0;
						}
					}
				}
				
				break;
			}
			case Double_Bridge_Back_Rise_Event:{
				
				/* ֧��ǰ��� */
				if(Part_State == 0){
					if(Div_State == 0){
						Leg_New_Walk(rise_y,rise_up_z,&Leg[0],&Double_Bridge_Front_Rise_Ramp[0]);
						Leg_Angle_Control(&Leg[0]);
						if(Slope(&Double_Bridge_Front_Rise_Ramp[0]) == 1.0f){
							ResetSlope(&Double_Bridge_Front_Rise_Ramp[0]);
							Div_State = 1;
						}
					}
					else if(Div_State == 1){
						if(Slope(&Double_Bridge_Rise_Buffer_Ramp) == 1.0f){
							ResetSlope(&Double_Bridge_Rise_Buffer_Ramp);
							Div_State = 0;
							Part_State = 1;
						}
					}
				}
				
				/* ֧��ǰ�ҽ� */
				else if(Part_State == 1){
					if(Div_State == 0){
						Leg_New_Walk(rise_y,rise_up_z,&Leg[1],&Double_Bridge_Front_Rise_Ramp[1]);
						Leg_Angle_Control(&Leg[1]);
						if(Slope(&Double_Bridge_Front_Rise_Ramp[1]) == 1.0f){
							ResetSlope(&Double_Bridge_Front_Rise_Ramp[1]);
							Div_State = 1;
						}
					}
					else if(Div_State == 1){
						if(Slope(&Double_Bridge_Rise_Buffer_Ramp) == 1.0f){
							ResetSlope(&Double_Bridge_Rise_Buffer_Ramp);
							Div_State = 0;
							Part_State = 2;
						}
					}
				}
				/* վֱ */
				else if(Part_State == 2){
					if(Div_State == 0){
						if(Slope(&Double_Bridge_Stand_Up_Down_Buffer_Ramp) == 1.0f){
							ResetSlope(&Double_Bridge_Stand_Up_Down_Buffer_Ramp);
							Div_State = 1;
						}
					}
					else if(Div_State == 1){
						for(uint8_t i = 0;i<4;i++){
							Leg_Point_RTO_Ramp(-rise_y,0,&Leg[i],&Double_Bridge_Reset_Ramp[i]);
							Leg_Angle_Control(&Leg[i]);
						}
						if(Slope(&Double_Bridge_Reset_Ramp[0]) == 1.0f && Slope(&Double_Bridge_Reset_Ramp[1]) == 1.0f &&
							 Slope(&Double_Bridge_Reset_Ramp[2]) == 1.0f && Slope(&Double_Bridge_Reset_Ramp[3]) == 1.0f){
							for(uint8_t i = 0;i<4;i++){
								ResetSlope(&Double_Bridge_Reset_Ramp[i]);
							}
							Div_State = 2;
						}
					}
					else if(Div_State == 2){
						if(Slope(&Double_Bridge_Rise_Buffer_Ramp) == 1.0f){
							ResetSlope(&Double_Bridge_Rise_Buffer_Ramp);
							Div_State = 0;
							Part_State = 3;
						}
					}
				}
				/* �����̧�� */
				else if(Part_State == 3){
					if(Div_State == 0){
						Leg_Point_RTO_Ramp(0,line_z,&Leg[2],&Double_Bridge_Up_Ramp[2]);
						Leg_Angle_Control(&Leg[2]);
						if(Slope(&Double_Bridge_Up_Ramp[2]) == 1.0f){
							ResetSlope(&Double_Bridge_Up_Ramp[2]);
							Div_State = 1;
						}
					}
					else if(Div_State == 1){
						Leg_New_Walk(rise_y,rise_z,&Leg[2],&Double_Bridge_Front_Rise_Ramp[2]);
						Leg_Angle_Control(&Leg[2]);
						if(Slope(&Double_Bridge_Front_Rise_Ramp[2]) == 1.0f){
							ResetSlope(&Double_Bridge_Front_Rise_Ramp[2]);
							Div_State = 2;
						}
					}
					else if(Div_State == 2){
						if(Slope(&Double_Bridge_Rise_Buffer_Ramp) == 1.0f){
							ResetSlope(&Double_Bridge_Rise_Buffer_Ramp);
							Div_State = 0;
							Part_State = 4;
						}
					}
				}
				
				/* ���ҽ�̧�� */
				else if(Part_State == 4){
					if(Div_State == 0){
						Leg_Point_RTO_Ramp(0,line_z,&Leg[3],&Double_Bridge_Up_Ramp[3]);
						Leg_Angle_Control(&Leg[3]);
						if(Slope(&Double_Bridge_Up_Ramp[3]) == 1.0f){
							ResetSlope(&Double_Bridge_Up_Ramp[3]);
							Div_State = 1;
						}
					}
					
					else if(Div_State == 1){
						Leg_New_Walk(rise_y,rise_z,&Leg[3],&Double_Bridge_Front_Rise_Ramp[3]);
						Leg_Angle_Control(&Leg[3]);
						if(Slope(&Double_Bridge_Front_Rise_Ramp[3]) == 1.0f){
							ResetSlope(&Double_Bridge_Front_Rise_Ramp[3]);
							Div_State = 2;
						}
					}
					
					else if(Div_State == 2){
						if(Slope(&Double_Bridge_Rise_Buffer_Ramp) == 1.0f){
							ResetSlope(&Double_Bridge_Rise_Buffer_Ramp);
							Div_State = 0;
							start_flag = 0;
							Part_State = 0;
							Robot->double_bridge_event = Double_Bridge_None_Event;
							Robot_Move_State_Reset_Stop(Robot);
						}
					}
				}
				
				break;
			}
			
			case Double_Bridge_Front_Down_Event:{
				/* ǰ����µ� */
				if(Part_State == 0){
					if(Div_State == 0){
						Leg_New_Walk(down_y,down_z,&Leg[0],&Double_Bridge_Front_Down_Ramp[0]);
						Leg_Angle_Control(&Leg[0]);
						if(Slope(&Double_Bridge_Front_Down_Ramp[0]) == 1.0f){
							ResetSlope(&Double_Bridge_Front_Down_Ramp[0]);
							Div_State = 1;
						}
					}
					
					else if(Div_State == 1){
						Leg_Point_RTO_Ramp(0,-line_z,&Leg[0],&Double_Bridge_Down_Ramp[0]);
						Leg_Angle_Control(&Leg[0]);
						if(Slope(&Double_Bridge_Down_Ramp[0]) == 1.0f){
							ResetSlope(&Double_Bridge_Down_Ramp[0]);
							Div_State = 2;
						}  
					}

					else if(Div_State == 2){
						if(Slope(&Double_Bridge_Down_Buffer_Ramp) == 1.0f){
							ResetSlope(&Double_Bridge_Down_Buffer_Ramp);
							Div_State = 0;
							Part_State = 1;
						}
					}
					
				}
				/* ǰ�ҽ��µ� */
				else if(Part_State == 1){
					if(Div_State == 0){
						Leg_New_Walk(down_y,down_z,&Leg[1],&Double_Bridge_Back_Down_Ramp[1]);
						Leg_Angle_Control(&Leg[1]);
						if(Slope(&Double_Bridge_Back_Down_Ramp[1]) == 1.0f){
							ResetSlope(&Double_Bridge_Back_Down_Ramp[1]);
							Div_State = 1;
						}
					}
					else if(Div_State == 1){
						Leg_Point_RTO_Ramp(0,-line_z,&Leg[1],&Double_Bridge_Down_Ramp[1]);
						Leg_Angle_Control(&Leg[1]);
						if(Slope(&Double_Bridge_Down_Ramp[1]) == 1.0f){
							ResetSlope(&Double_Bridge_Down_Ramp[1]);
							Div_State = 2;
						}
					}
					else if(Div_State == 2){
						if(Slope(&Double_Bridge_Down_Buffer_Ramp)){
							ResetSlope(&Double_Bridge_Down_Buffer_Ramp);
							Div_State = 0;
							Part_State = 2;
						}
					}
				}
				/* վֱ */
				else if(Part_State == 2){
					if(Div_State == 0){
						if(Slope(&Double_Bridge_Stand_Up_Down_Buffer_Ramp) == 1.0f){
							ResetSlope(&Double_Bridge_Stand_Up_Down_Buffer_Ramp);
							Div_State = 1;
						}
					}
					else if(Div_State == 1){
						for(uint8_t i = 0;i<4;i++){
							Leg_Point_RTO_Ramp(-down_y,0,&Leg[i],&Double_Bridge_Reset_Ramp[i]);
							Leg_Angle_Control(&Leg[i]);
						}
						if(Slope(&Double_Bridge_Reset_Ramp[0]) == 1.0f && Slope(&Double_Bridge_Reset_Ramp[1]) == 1.0f &&
							 Slope(&Double_Bridge_Reset_Ramp[2]) == 1.0f && Slope(&Double_Bridge_Reset_Ramp[3]) == 1.0f){
							for(uint8_t i = 0;i<4;i++){
								ResetSlope(&Double_Bridge_Reset_Ramp[i]);
							}
							Div_State = 2;
						}
					}
					else if(Div_State == 2){
						if(Slope(&Double_Bridge_Down_Buffer_Ramp) == 1.0f){
							ResetSlope(&Double_Bridge_Down_Buffer_Ramp);
							Div_State = 0;
							Part_State = 3;
						}
					}
				}
				/* ����Żع� */
				else if(Part_State == 3){
					if(Div_State == 0){
						Leg_New_Walk(down_y,rise_up_z,&Leg[2],&Double_Bridge_Front_Down_Ramp[2]);
						Leg_Angle_Control(&Leg[2]);
						if(Slope(&Double_Bridge_Front_Down_Ramp[2]) == 1.0f){
							ResetSlope(&Double_Bridge_Front_Down_Ramp[2]);
							Div_State = 1;
						}
					}
					else if(Div_State == 1){
						if(Slope(&Double_Bridge_Down_Buffer_Ramp) == 1.0f){
							ResetSlope(&Double_Bridge_Down_Buffer_Ramp);
							Div_State = 0;
							Part_State = 4;
						}
					}
				}
				
				/* ���ҽŻع� */
				else if(Part_State == 4){
					if(Div_State == 0){
						Leg_New_Walk(down_y,rise_up_z,&Leg[3],&Double_Bridge_Front_Down_Ramp[3]);
						Leg_Angle_Control(&Leg[3]);
						if(Slope(&Double_Bridge_Front_Down_Ramp[3]) == 1.0f){
							ResetSlope(&Double_Bridge_Front_Down_Ramp[3]);
							Div_State = 1;
						}
					}
					else if(Div_State == 1){
						if(Slope(&Double_Bridge_Down_Buffer_Ramp) == 1.0f){
							ResetSlope(&Double_Bridge_Down_Buffer_Ramp);
							Div_State = 0;
							Part_State = 0;
							Robot->double_bridge_event = Double_Bridge_None_Event;
							Robot_Move_State_Reset_Stop(Robot);
							start_flag = 0;
						}
					}
				}
				
				break;
			}
			
			case Double_Bridge_Back_Down_Event:{
				/* ǰ���֧�� */
				if(Part_State == 0){
					if(Div_State == 0){
						Leg_New_Walk(down_y,rise_up_z,&Leg[0],&Double_Bridge_Front_Down_Ramp[0]);
						Leg_Angle_Control(&Leg[0]);
						if(Slope(&Double_Bridge_Front_Down_Ramp[0]) == 1.0f){
							ResetSlope(&Double_Bridge_Front_Down_Ramp[0]);
							Div_State = 1;
						}
					}
					else if(Div_State == 1){
						if(Slope(&Double_Bridge_Down_Buffer_Ramp) == 1.0f){
							ResetSlope(&Double_Bridge_Down_Buffer_Ramp);
							Div_State = 0;
							Part_State = 1;
						}
					}
				}
				
				/* ǰ�ҽ�֧�� */
				else if(Part_State == 1){
					if(Div_State == 0){
						Leg_New_Walk(down_y,rise_up_z,&Leg[1],&Double_Bridge_Front_Down_Ramp[1]);
						Leg_Angle_Control(&Leg[1]);
						if(Slope(&Double_Bridge_Front_Down_Ramp[1]) == 1.0f){
							ResetSlope(&Double_Bridge_Front_Down_Ramp[1]);
							Div_State = 1;
						}
					}
					else if(Div_State == 1){
						if(Slope(&Double_Bridge_Down_Buffer_Ramp) == 1.0f){
							ResetSlope(&Double_Bridge_Down_Buffer_Ramp);
							Div_State = 0;
							Part_State = 2;
						}
					}
				}
				/* վֱ */
				else if(Part_State == 2){
					if(Div_State == 0){
						if(Slope(&Double_Bridge_Stand_Up_Down_Buffer_Ramp) == 1.0f){
							ResetSlope(&Double_Bridge_Stand_Up_Down_Buffer_Ramp);
							Div_State = 1;
						}
					}
					else if(Div_State == 1){
						for(uint8_t i = 0;i<4;i++){
							Leg_Point_RTO_Ramp(-down_y,0,&Leg[i],&Double_Bridge_Reset_Ramp[i]);
							Leg_Angle_Control(&Leg[i]);
						}
						if(Slope(&Double_Bridge_Reset_Ramp[0]) == 1.0f && Slope(&Double_Bridge_Reset_Ramp[1]) == 1.0f &&
							 Slope(&Double_Bridge_Reset_Ramp[2]) == 1.0f && Slope(&Double_Bridge_Reset_Ramp[3]) == 1.0f){
							for(uint8_t i = 0;i<4;i++){
								ResetSlope(&Double_Bridge_Reset_Ramp[i]);
							}
							Div_State = 2;
						}
					}
					else if(Div_State == 2){
						if(Slope(&Double_Bridge_Stand_Up_Down_Buffer_Ramp) == 1.0f){
							ResetSlope(&Double_Bridge_Stand_Up_Down_Buffer_Ramp);
							Div_State = 0;
							Part_State = 3;
						}
					}
				}
				/* ������µ� */
				else if(Part_State == 3){
					if(Div_State == 0){
						Leg_New_Walk(down_y,down_z,&Leg[2],&Double_Bridge_Front_Down_Ramp[2]);
						Leg_Angle_Control(&Leg[2]);
						if(Slope(&Double_Bridge_Front_Down_Ramp[2]) == 1.0f){
							ResetSlope(&Double_Bridge_Front_Down_Ramp[0]);
							Div_State = 1;
						}
					}
					
					else if(Div_State == 1){
						Leg_Point_RTO_Ramp(0,-line_z,&Leg[2],&Double_Bridge_Down_Ramp[2]);
						Leg_Angle_Control(&Leg[2]);
						if(Slope(&Double_Bridge_Down_Ramp[2]) == 1.0f){
							ResetSlope(&Double_Bridge_Down_Ramp[2]);
							Div_State = 2;
						}  
					}

					else if(Div_State == 2){
						if(Slope(&Double_Bridge_Down_Buffer_Ramp) == 1.0f){
							ResetSlope(&Double_Bridge_Down_Buffer_Ramp);
							Div_State = 0;
							Part_State = 4;
						}
					}
				}
				
				/* ���ҽ��µ� */
				else if(Part_State == 4){
					if(Div_State == 0){
						Leg_New_Walk(down_y,down_z,&Leg[3],&Double_Bridge_Back_Down_Ramp[3]);
						Leg_Angle_Control(&Leg[3]);
						if(Slope(&Double_Bridge_Back_Down_Ramp[3]) == 1.0f){
							ResetSlope(&Double_Bridge_Back_Down_Ramp[3]);
							Div_State = 1;
						}
					}
					else if(Div_State == 1){
						Leg_Point_RTO_Ramp(0,-line_z,&Leg[3],&Double_Bridge_Down_Ramp[3]);
						Leg_Angle_Control(&Leg[3]);
						if(Slope(&Double_Bridge_Down_Ramp[3]) == 1.0f){
							ResetSlope(&Double_Bridge_Down_Ramp[3]);
							Div_State = 2;
						}
					}
					else if(Div_State == 2){
						if(Slope(&Double_Bridge_Stand_Up_Down_Buffer_Ramp)){
							ResetSlope(&Double_Bridge_Stand_Up_Down_Buffer_Ramp);
							Div_State = 0;
							Part_State = 5;
						}
					}
				}
				
				/* վֱ */
				else if(Part_State == 5){
					if(Div_State == 0){
						if(Slope(&Double_Bridge_Stand_Up_Down_Buffer_Ramp) == 1.0f){
							ResetSlope(&Double_Bridge_Stand_Up_Down_Buffer_Ramp);
							Div_State = 1;
						}
					}
					else if(Div_State == 1){
						
						for(uint8_t i = 0;i<4;i++){
							Leg_Point_RTO_Ramp(0,stand_up_down_z,&Leg[i],&Double_Bridge_Reset_Ramp[i]);
							Leg_Angle_Control(&Leg[i]);
						}
						if(Slope(&Double_Bridge_Reset_Ramp[0]) == 1.0f && Slope(&Double_Bridge_Reset_Ramp[1]) == 1.0f &&
							 Slope(&Double_Bridge_Reset_Ramp[2]) == 1.0f && Slope(&Double_Bridge_Reset_Ramp[3]) == 1.0f){
							for(uint8_t i = 0;i<4;i++){
								ResetSlope(&Double_Bridge_Reset_Ramp[i]);
							}
							Robot->double_bridge_event = Double_Bridge_None_Event;
							Robot->double_bridge_state = Double_Bridge_None_State;
							Robot_Move_State_Reset_Stop(Robot);
							start_flag = 0;
							Part_State = 0;
							Div_State = 0;
						}
					}
				}
				
				break;
			}
			case Double_Bridge_Straignt_Event:{
				if(Part_State == 0){
					Leg_New_Walk(straight_walk_y,straight_walk_z,&Leg[0],&Double_Bridge_Straight_Ramp[0]);
					Leg_New_Walk(straight_walk_y,straight_walk_z,&Leg[3],&Double_Bridge_Straight_Ramp[3]);
					Leg_Point_RTO_Ramp(-straight_walk_y,0,&Leg[1],&Double_Bridge_Straight_Ramp[1]);
					Leg_Point_RTO_Ramp(-straight_walk_y,0,&Leg[2],&Double_Bridge_Straight_Ramp[2]);
					for(uint8_t i = 0;i<4;i++){
						Leg_Angle_Control(&Leg[i]);
					}
					if(Slope(&Double_Bridge_Straight_Ramp[0]) == 1.0f && Slope(&Double_Bridge_Straight_Ramp[1]) == 1.0f &&
						 Slope(&Double_Bridge_Straight_Ramp[2]) == 1.0f && Slope(&Double_Bridge_Straight_Ramp[3]) == 1.0f){
						for(uint8_t i  = 0;i<4;i++){
							ResetSlope(&Double_Bridge_Straight_Ramp[i]);
						}
						Part_State = 1;
					}
				}
				else if(Part_State == 1){
					Leg_New_Walk(straight_walk_y,straight_walk_z,&Leg[1],&Double_Bridge_Straight_Ramp[1]);
					Leg_New_Walk(straight_walk_y,straight_walk_z,&Leg[2],&Double_Bridge_Straight_Ramp[2]);
					Leg_Point_RTO_Ramp(-straight_walk_y,0,&Leg[0],&Double_Bridge_Straight_Ramp[0]);
					Leg_Point_RTO_Ramp(-straight_walk_y,0,&Leg[3],&Double_Bridge_Straight_Ramp[3]);
					for(uint8_t i = 0;i<4;i++){
						Leg_Angle_Control(&Leg[i]);
					}
					if(Slope(&Double_Bridge_Straight_Ramp[0]) == 1.0f && Slope(&Double_Bridge_Straight_Ramp[1]) == 1.0f &&
						 Slope(&Double_Bridge_Straight_Ramp[2]) == 1.0f && Slope(&Double_Bridge_Straight_Ramp[3]) == 1.0f){
						for(uint8_t i = 0;i<4;i++){
							ResetSlope(&Double_Bridge_Straight_Ramp[i]);
						}
						start_flag = 0;
						Part_State = 0;
						Robot->double_bridge_event = Double_Bridge_None_Event;
						Robot_Move_State_Reset_Stop(Robot);
					}
				}
				break;
			}
			case Double_Bridge_Turn_Event:{
				if(Part_State == 0){
					if(initial_Remote.R_X_rocker < 500){
						Leg_New_Walk(turn_walk_l_y,turn_walk_z,&Leg[0],&Double_Bridge_Turn_Ramp[0]);
						Leg_New_Walk(turn_walk_r_y,turn_walk_z,&Leg[3],&Double_Bridge_Turn_Ramp[3]);
						Leg_Point_RTO_Ramp(-turn_walk_l_y,0,&Leg[2],&Double_Bridge_Turn_Ramp[2]);
						Leg_Point_RTO_Ramp(-turn_walk_r_y,0,&Leg[1],&Double_Bridge_Turn_Ramp[1]);
					}
					else{
						Leg_New_Walk(turn_walk_l_y,turn_walk_z,&Leg[2],&Double_Bridge_Turn_Ramp[2]);
						Leg_New_Walk(turn_walk_r_y,turn_walk_z,&Leg[1],&Double_Bridge_Turn_Ramp[1]);
						Leg_Point_RTO_Ramp(-turn_walk_l_y,0,&Leg[0],&Double_Bridge_Turn_Ramp[0]);
						Leg_Point_RTO_Ramp(-turn_walk_r_y,0,&Leg[3],&Double_Bridge_Turn_Ramp[3]);
					}
					for(uint8_t i = 0;i<4;i++){
						Leg_Angle_Control(&Leg[i]);
					}
					if(Slope(&Double_Bridge_Turn_Ramp[0]) == 1.0f && Slope(&Double_Bridge_Turn_Ramp[1]) == 1.0f &&
						 Slope(&Double_Bridge_Turn_Ramp[2]) == 1.0f && Slope(&Double_Bridge_Turn_Ramp[3]) == 1.0f){
						for(uint8_t i = 0;i<4;i++){
							ResetSlope(&Double_Bridge_Turn_Ramp[i]);
						}
						Part_State = 1;
					}
				}
				else if(Part_State == 1){
					if(initial_Remote.R_X_rocker < 500){
						Leg_New_Walk(turn_walk_l_y,turn_walk_z,&Leg[2],&Double_Bridge_Turn_Ramp[2]);
						Leg_New_Walk(turn_walk_r_y,turn_walk_z,&Leg[1],&Double_Bridge_Turn_Ramp[1]);
						Leg_Point_RTO_Ramp(-turn_walk_l_y,0,&Leg[0],&Double_Bridge_Turn_Ramp[0]);
						Leg_Point_RTO_Ramp(-turn_walk_r_y,0,&Leg[3],&Double_Bridge_Turn_Ramp[3]);
					}
					else{
						Leg_New_Walk(turn_walk_l_y,turn_walk_z,&Leg[0],&Double_Bridge_Turn_Ramp[0]);
						Leg_New_Walk(turn_walk_r_y,turn_walk_z,&Leg[3],&Double_Bridge_Turn_Ramp[3]);
						Leg_Point_RTO_Ramp(-turn_walk_l_y,0,&Leg[2],&Double_Bridge_Turn_Ramp[2]);
						Leg_Point_RTO_Ramp(-turn_walk_r_y,0,&Leg[1],&Double_Bridge_Turn_Ramp[1]);
					}
					for(uint8_t i = 0;i<4;i++){
						Leg_Angle_Control(&Leg[i]);
					}
					if(Slope(&Double_Bridge_Turn_Ramp[0]) == 1.0f && Slope(&Double_Bridge_Turn_Ramp[1]) == 1.0f &&
						 Slope(&Double_Bridge_Turn_Ramp[2]) == 1.0f && Slope(&Double_Bridge_Turn_Ramp[3]) == 1.0f){
						for(uint8_t i = 0;i<4;i++){
							ResetSlope(&Double_Bridge_Turn_Ramp[i]);
						}
						start_flag = 0;
						Part_State = 0;
						Robot->double_bridge_event = Double_Bridge_None_Event;
						Robot_Move_State_Reset_Stop(Robot);
					}
				}
				break;
			}
			case Double_Bridge_Circle_Event:{
				if(Part_State == 0){
					Leg_New_Walk(circle_walk_l_y,circle_walk_z,&Leg[0],&Double_Bridge_Circle_Ramp[0]);
					Leg_New_Walk(circle_walk_r_y,circle_walk_z,&Leg[3],&Double_Bridge_Circle_Ramp[3]);
					Leg_Point_RTO_Ramp(-circle_walk_l_y,0,&Leg[2],&Double_Bridge_Circle_Ramp[2]);
					Leg_Point_RTO_Ramp(-circle_walk_r_y,0,&Leg[1],&Double_Bridge_Circle_Ramp[1]);
					for(uint8_t i = 0;i<4;i++){
						Leg_Angle_Control(&Leg[i]);
					}
					if(Slope(&Double_Bridge_Circle_Ramp[0]) == 1.0f && Slope(&Double_Bridge_Circle_Ramp[1]) == 1.0f && 
						 Slope(&Double_Bridge_Circle_Ramp[2]) == 1.0f && Slope(&Double_Bridge_Circle_Ramp[3]) == 1.0f){
						for(uint8_t i = 0;i<4;i++){
							ResetSlope(&Double_Bridge_Circle_Ramp[i]);
						}
						Part_State = 1;
					}
				}
				else if(Part_State == 1){
					Leg_New_Walk(circle_walk_l_y,circle_walk_z,&Leg[2],&Double_Bridge_Circle_Ramp[2]);
					Leg_New_Walk(circle_walk_r_y,circle_walk_z,&Leg[1],&Double_Bridge_Circle_Ramp[1]);
					Leg_Point_RTO_Ramp(-circle_walk_l_y,0,&Leg[0],&Double_Bridge_Circle_Ramp[0]);
					Leg_Point_RTO_Ramp(-circle_walk_r_y,0,&Leg[3],&Double_Bridge_Circle_Ramp[3]);
					for(uint8_t i = 0;i<4;i++){
						Leg_Angle_Control(&Leg[i]);
					}
					if(Slope(&Double_Bridge_Circle_Ramp[0]) == 1.0f && Slope(&Double_Bridge_Circle_Ramp[1]) == 1.0f && 
						 Slope(&Double_Bridge_Circle_Ramp[2]) == 1.0f && Slope(&Double_Bridge_Circle_Ramp[3]) == 1.0f){
						for(uint8_t i = 0;i<4;i++){
							ResetSlope(&Double_Bridge_Circle_Ramp[i]);
						}
						Part_State = 0;
						start_flag = 0;
						Robot->double_bridge_event = Double_Bridge_None_Event;
						Robot_Move_State_Reset_Stop(Robot);
					}
				}
				break;
			}
			
		}
	}
}

uint32_t climb_up_count;
uint32_t sqat_count;
/* ���ΰ� */
void Robot_Move_Seesaw(Robot_Handle *Robot,Leg_Handle *Leg,Dev_Handle *IMU){
	static uint8_t Part_State;											//״̬
	static uint8_t Div_State;												//״̬��̬
	static uint8_t start_flag;											//��ʼ��־
	static Remote_DataPack_Handle initial_Remote;		//��ʼ״̬
	static float sqat_z = 20.0f;										//����z
	static float stand_y;
	static float source_climb_up_posture_y = 60.0f;					
	static float source_climb_down_posture_y = 60.0f;
	static float climb_up_posture_y;								
	static float climb_down_posture_y;
	
	static float straight_source_walk_y = 30.0f;		//ԭʼֱ��y
	static float straight_walk_y;										//ֱ��y
	static float straight_walk_z = 25.0f;						//ֱ��z
	
	static float turn_short_y = 20.0f;							//����̾�y
	static float turn_long_y = 40.0f;								//���䳤��y
	static float turn_walk_z = 25.0f;								//����z
	static float turn_walk_l_y;											//�������y
	static float turn_walk_r_y;											//�����ҽ�y
	
	static float circle_walk_y = 20.0f;							//ԭ����תy
	static float circle_walk_z = 20.0f;							//ԭ����תz
	static float circle_walk_l_y;										//ԭ����ת���y
	static float circle_walk_r_y;										//ԭ����ת�ҽ�y
	/* ��� */
	if(!start_flag){
		memcpy(&initial_Remote,&Remote_DataPack,21);
		if(initial_Remote.Key.Right_Key_Down == 0 && Remote_Last_DataPack.Key.Right_Key_Down){
			start_flag = 1;
			if(Robot->seesaw_sqat_state == Seesaw_Stand_State){
				Robot->seesaw_event = Seesaw_Sqat_Event;											//�����¼�
				Robot->seesaw_sqat_state = Seesaw_Sqat_State;									//׼������
			}
			else if(Robot->seesaw_sqat_state == Seesaw_Sqat_State){

				if(Robot->seesaw_climb_state == Seesaw_Climb_None_State){
					stand_y = 0;
				}
				else if(Robot->seesaw_climb_state == Seesaw_Climb_Up_State){
					stand_y = source_climb_up_posture_y;
				}
				else if(Robot->seesaw_climb_state == Seesaw_Climb_Down_State){
					stand_y = -source_climb_down_posture_y;
				}
				Robot->seesaw_event = Seesaw_Stand_Event;											//�����¼�
				Robot->seesaw_sqat_state = Seesaw_Stand_State;								//վ��״̬
				Robot->seesaw_climb_state = Seesaw_Climb_None_State;					//����״̬��λ
			}
			sqat_count++;
		}
		
		/* ������̬ */
		else if((Robot->seesaw_climb_state == Seesaw_Climb_None_State || Robot->seesaw_climb_state == Seesaw_Climb_Down_State) && 
						IMU->Angle.Pitch > 4.0f && Robot->seesaw_sqat_state == Seesaw_Sqat_State){
//		else if((Robot->seesaw_climb_state == Seesaw_Climb_None_State || Robot->seesaw_climb_state == Seesaw_Climb_Down_State) &&
//						 initial_Remote.Key.Right_Key_Left  == 0 && Remote_Last_DataPack.Key.Right_Key_Left && 
//						 Robot->seesaw_sqat_state == Seesaw_Sqat_State){  
			start_flag = 1;
			
			if(Robot->seesaw_climb_state == Seesaw_Climb_None_State){
				climb_up_posture_y = -source_climb_up_posture_y;
			}
			else if(Robot->seesaw_climb_state == Seesaw_Climb_Down_State){
				climb_up_posture_y = -(source_climb_down_posture_y + source_climb_up_posture_y);
			}
			Robot->seesaw_climb_state = Seesaw_Climb_Up_State;
			Robot->seesaw_event = Seesaw_Climb_Up_Event;
			
			climb_up_count++;
		}
		
		else if(Robot->seesaw_climb_state == Seesaw_Climb_Up_State && IMU->Angle.Pitch < -4.0f && Robot->seesaw_sqat_state == Seesaw_Sqat_State){
			start_flag = 1;
			climb_down_posture_y = source_climb_down_posture_y + source_climb_up_posture_y;
			Robot->seesaw_climb_state = Seesaw_Climb_Down_State;
			Robot->seesaw_event = Seesaw_Climb_Down_Event;
		}
		/* ֱ��/ת�� */
		else if(initial_Remote.L_Y_rocker < 500 || initial_Remote.L_Y_rocker > 3500){
			start_flag = 1;
			if(initial_Remote.R_X_rocker > 500 && initial_Remote.R_X_rocker < 3500){
				straight_walk_y = initial_Remote.L_Y_rocker < 500 ? straight_source_walk_y : - straight_source_walk_y;			//ȷ��ǰ�л����
				Robot->seesaw_event = Seesaw_Straight_Event;					//ֱ��
			}
			else{
				turn_walk_l_y = initial_Remote.R_X_rocker < 500 ? turn_short_y : turn_long_y;
				turn_walk_r_y = initial_Remote.R_X_rocker > 3500 ? turn_short_y : turn_long_y;
				Robot->seesaw_event = Seesaw_Turn_Event;							//ת��
			}
		}
		/* ԭ�ع��� */
		else if(initial_Remote.L_X_rocker < 500 || initial_Remote.L_X_rocker > 3500){
			start_flag = 1;
			circle_walk_l_y = initial_Remote.L_X_rocker < 500 ? -circle_walk_y : circle_walk_y;
			circle_walk_r_y = initial_Remote.L_X_rocker > 3500 ? -circle_walk_y : circle_walk_y;
			Robot->seesaw_event = Seesaw_Circle_Event;							//ԭ��ת��
		}
		else{
			Robot_Move_State_Reset_Stop(Robot);
		}
	}
	else{
		switch(Robot->seesaw_event){
			case Seesaw_None_Event:{
				break;
			}
			case Seesaw_Sqat_Event:{
				if(Part_State == 0){
					for(uint8_t i = 0;i<4;i++){
						Leg_Point_RTO_Ramp(0,sqat_z,&Leg[i],&Seesaw_Sqat_Ramp[i]);
						Leg_Angle_Control(&Leg[i]);
					}
					if(Slope(&Seesaw_Sqat_Ramp[0]) == 1.0f && Slope(&Seesaw_Sqat_Ramp[1]) == 1.0f && 
						 Slope(&Seesaw_Sqat_Ramp[2]) == 1.0f && Slope(&Seesaw_Sqat_Ramp[3]) == 1.0f){
						for(uint8_t i = 0;i<4;i++){
							ResetSlope(&Seesaw_Sqat_Ramp[i]);
						}
						Part_State = 1;

					}
				}
				else if(Part_State == 1){
					if(Slope(&Seesaw_Sqat_Buffer_Ramp) == 1.0f){
						ResetSlope(&Seesaw_Sqat_Buffer_Ramp);
						Robot->seesaw_event = Seesaw_None_Event;
						Part_State = 0;
						start_flag = 0;
						Robot_Move_State_Reset_Stop(Robot);
					}
				}
				break;
			}
			case Seesaw_Stand_Event:{
				for(uint8_t i = 0;i<4;i++){
					Leg_Point_RTO_Ramp(stand_y,-sqat_z,&Leg[i],&Seesaw_Sqat_Back_Ramp[i]);
					Leg_Angle_Control(&Leg[i]);
				}
				if(Slope(&Seesaw_Sqat_Back_Ramp[0]) == 1.0f && Slope(&Seesaw_Sqat_Back_Ramp[1]) == 1.0f &&
					 Slope(&Seesaw_Sqat_Back_Ramp[2]) == 1.0f && Slope(&Seesaw_Sqat_Back_Ramp[3]) == 1.0f){
					for(uint8_t i = 0;i<4;i++){
						ResetSlope(&Seesaw_Sqat_Back_Ramp[i]);
					}
					start_flag = 0;
					Robot->seesaw_event = Seesaw_None_Event;
					Robot_Move_State_Reset_Stop(Robot);
				}
				break;
			}
			case Seesaw_Climb_Up_Event:{
				if(Part_State == 0){
					for(uint8_t i = 0;i<4;i++){
						Leg_Point_RTO_Ramp(climb_up_posture_y,0,&Leg[i],&Seesaw_Climb_Up_Posture_Ramp[i]);
						Leg_Angle_Control(&Leg[i]);
					}
					if(Slope(&Seesaw_Climb_Up_Posture_Ramp[0]) == 1.0f && Slope(&Seesaw_Climb_Up_Posture_Ramp[1]) == 1.0f &&
						 Slope(&Seesaw_Climb_Up_Posture_Ramp[2]) == 1.0f && Slope(&Seesaw_Climb_Up_Posture_Ramp[3]) == 1.0f){
						for(uint8_t i = 0;i<4;i++){
							ResetSlope(&Seesaw_Climb_Up_Posture_Ramp[i]);
						}
						Part_State = 1;
					}
				}
				else if(Part_State == 1){
					if(Slope(&Seesaw_Climb_Buffer_Ramp) == 1.0f){
						ResetSlope(&Seesaw_Climb_Buffer_Ramp);
						Robot->seesaw_event = Seesaw_None_Event;
						Part_State = 0;
						start_flag = 0;
						Robot_Move_State_Reset_Stop(Robot);
					}
				}
				break;
			}
			case Seesaw_Climb_Down_Event:{
				if(Part_State == 0){		
					for(uint8_t i = 0;i<4;i++){
						Leg_Point_RTO_Ramp(climb_down_posture_y,0,&Leg[i],&Seesaw_Climb_Down_Posture_Ramp[i]);
						Leg_Angle_Control(&Leg[i]);
					}
					if(Slope(&Seesaw_Climb_Down_Posture_Ramp[0]) == 1.0f && Slope(&Seesaw_Climb_Down_Posture_Ramp[1]) == 1.0f &&
						 Slope(&Seesaw_Climb_Down_Posture_Ramp[2]) == 1.0f && Slope(&Seesaw_Climb_Down_Posture_Ramp[3]) == 1.0f){
						for(uint8_t i = 0;i<4;i++){
							ResetSlope(&Seesaw_Climb_Down_Posture_Ramp[i]);
						}
						Part_State = 1;
					}
				}
				else if(Part_State == 1){
					if(Slope(&Seesaw_Climb_Buffer_Ramp) == 1.0f){
						ResetSlope(&Seesaw_Climb_Buffer_Ramp);
						Robot->seesaw_event = Seesaw_None_Event;
						start_flag = 0;
						Part_State = 0;
						Robot_Move_State_Reset_Stop(Robot);
					}
				}
				break;
			}
			case Seesaw_Straight_Event:{
				if(Part_State == 0){
					Leg_New_Walk(straight_walk_y,straight_walk_z,&Leg[0],&Seesaw_Straight_Ramp[0]);
					Leg_New_Walk(straight_walk_y,straight_walk_z,&Leg[3],&Seesaw_Straight_Ramp[3]);
					Leg_Point_RTO_Ramp(-straight_walk_y,0,&Leg[1],&Seesaw_Straight_Ramp[1]);
					Leg_Point_RTO_Ramp(-straight_walk_y,0,&Leg[2],&Seesaw_Straight_Ramp[2]);
					for(uint8_t i = 0;i<4;i++){
						Leg_Angle_Control(&Leg[i]);
					}
					if(Slope(&Seesaw_Straight_Ramp[0]) == 1.0f && Slope(&Seesaw_Straight_Ramp[1]) == 1.0f &&
						 Slope(&Seesaw_Straight_Ramp[2]) == 1.0f && Slope(&Seesaw_Straight_Ramp[3]) == 1.0f){
						for(uint8_t i  = 0;i<4;i++){
							ResetSlope(&Seesaw_Straight_Ramp[i]);
						}
						Part_State = 1;
					}
				}
				else if(Part_State == 1){
					Leg_New_Walk(straight_walk_y,straight_walk_z,&Leg[1],&Seesaw_Straight_Ramp[1]);
					Leg_New_Walk(straight_walk_y,straight_walk_z,&Leg[2],&Seesaw_Straight_Ramp[2]);
					Leg_Point_RTO_Ramp(-straight_walk_y,0,&Leg[0],&Seesaw_Straight_Ramp[0]);
					Leg_Point_RTO_Ramp(-straight_walk_y,0,&Leg[3],&Seesaw_Straight_Ramp[3]);
					for(uint8_t i = 0;i<4;i++){
						Leg_Angle_Control(&Leg[i]);
					}
					if(Slope(&Seesaw_Straight_Ramp[0]) == 1.0f && Slope(&Seesaw_Straight_Ramp[1]) == 1.0f &&
						 Slope(&Seesaw_Straight_Ramp[2]) == 1.0f && Slope(&Seesaw_Straight_Ramp[3]) == 1.0f){
						for(uint8_t i = 0;i<4;i++){
							ResetSlope(&Seesaw_Straight_Ramp[i]);
						}
						start_flag = 0;
						Part_State = 0;
						Robot->seesaw_event = Seesaw_None_Event;
						Robot_Move_State_Reset_Stop(Robot);
					}
				}
				break;
			}
			case Seesaw_Turn_Event:{
				if(Part_State == 0){
					if(initial_Remote.R_X_rocker < 500){
						Leg_New_Walk(turn_walk_l_y,turn_walk_z,&Leg[0],&Seesaw_Turn_Ramp[0]);
						Leg_New_Walk(turn_walk_r_y,turn_walk_z,&Leg[3],&Seesaw_Turn_Ramp[3]);
						Leg_Point_RTO_Ramp(-turn_walk_l_y,0,&Leg[2],&Seesaw_Turn_Ramp[2]);
						Leg_Point_RTO_Ramp(-turn_walk_r_y,0,&Leg[1],&Seesaw_Turn_Ramp[1]);
					}
					else{
						Leg_New_Walk(turn_walk_l_y,turn_walk_z,&Leg[2],&Seesaw_Turn_Ramp[2]);
						Leg_New_Walk(turn_walk_r_y,turn_walk_z,&Leg[1],&Seesaw_Turn_Ramp[1]);
						Leg_Point_RTO_Ramp(-turn_walk_l_y,0,&Leg[0],&Seesaw_Turn_Ramp[0]);
						Leg_Point_RTO_Ramp(-turn_walk_r_y,0,&Leg[3],&Seesaw_Turn_Ramp[3]);
					}
					for(uint8_t i = 0;i<4;i++){
						Leg_Angle_Control(&Leg[i]);
					}
					if(Slope(&Seesaw_Turn_Ramp[0]) == 1.0f && Slope(&Seesaw_Turn_Ramp[1]) == 1.0f &&
						 Slope(&Seesaw_Turn_Ramp[2]) == 1.0f && Slope(&Seesaw_Turn_Ramp[3]) == 1.0f){
						for(uint8_t i = 0;i<4;i++){
							ResetSlope(&Seesaw_Turn_Ramp[i]);
						}
						Part_State = 1;
					}
				}
				else if(Part_State == 1){
					if(initial_Remote.R_X_rocker < 500){
						Leg_New_Walk(turn_walk_l_y,turn_walk_z,&Leg[2],&Seesaw_Turn_Ramp[2]);
						Leg_New_Walk(turn_walk_r_y,turn_walk_z,&Leg[1],&Seesaw_Turn_Ramp[1]);
						Leg_Point_RTO_Ramp(-turn_walk_l_y,0,&Leg[0],&Seesaw_Turn_Ramp[0]);
						Leg_Point_RTO_Ramp(-turn_walk_r_y,0,&Leg[3],&Seesaw_Turn_Ramp[3]);
					}
					else{
						Leg_New_Walk(turn_walk_l_y,turn_walk_z,&Leg[0],&Seesaw_Turn_Ramp[0]);
						Leg_New_Walk(turn_walk_r_y,turn_walk_z,&Leg[3],&Seesaw_Turn_Ramp[3]);
						Leg_Point_RTO_Ramp(-turn_walk_l_y,0,&Leg[2],&Seesaw_Turn_Ramp[2]);
						Leg_Point_RTO_Ramp(-turn_walk_r_y,0,&Leg[1],&Seesaw_Turn_Ramp[1]);
					}
					for(uint8_t i = 0;i<4;i++){
						Leg_Angle_Control(&Leg[i]);
					}
					if(Slope(&Seesaw_Turn_Ramp[0]) == 1.0f && Slope(&Seesaw_Turn_Ramp[1]) == 1.0f &&
						 Slope(&Seesaw_Turn_Ramp[2]) == 1.0f && Slope(&Seesaw_Turn_Ramp[3]) == 1.0f){
						for(uint8_t i = 0;i<4;i++){
							ResetSlope(&Seesaw_Turn_Ramp[i]);
						}
						start_flag = 0;
						Part_State = 0;
						Robot->seesaw_event = Seesaw_None_Event;
						Robot_Move_State_Reset_Stop(Robot);
					}
				}
				break;
			}
			case Seesaw_Circle_Event:{
				if(Part_State == 0){
					Leg_New_Walk(circle_walk_l_y,circle_walk_z,&Leg[0],&Seesaw_Circle_Ramp[0]);
					Leg_New_Walk(circle_walk_r_y,circle_walk_z,&Leg[3],&Seesaw_Circle_Ramp[3]);
					Leg_Point_RTO_Ramp(-circle_walk_l_y,0,&Leg[2],&Seesaw_Circle_Ramp[2]);
					Leg_Point_RTO_Ramp(-circle_walk_r_y,0,&Leg[1],&Seesaw_Circle_Ramp[1]);
					for(uint8_t i = 0;i<4;i++){
						Leg_Angle_Control(&Leg[i]);
					}
					if(Slope(&Seesaw_Circle_Ramp[0]) == 1.0f && Slope(&Seesaw_Circle_Ramp[1]) == 1.0f && 
						 Slope(&Seesaw_Circle_Ramp[2]) == 1.0f && Slope(&Seesaw_Circle_Ramp[3]) == 1.0f){
						for(uint8_t i = 0;i<4;i++){
							ResetSlope(&Seesaw_Circle_Ramp[i]);
						}
						Part_State = 1;
					}
				}
				else if(Part_State == 1){
					Leg_New_Walk(circle_walk_l_y,circle_walk_z,&Leg[2],&Seesaw_Circle_Ramp[2]);
					Leg_New_Walk(circle_walk_r_y,circle_walk_z,&Leg[1],&Seesaw_Circle_Ramp[1]);
					Leg_Point_RTO_Ramp(-circle_walk_l_y,0,&Leg[0],&Seesaw_Circle_Ramp[0]);
					Leg_Point_RTO_Ramp(-circle_walk_r_y,0,&Leg[3],&Seesaw_Circle_Ramp[3]);
					for(uint8_t i = 0;i<4;i++){
						Leg_Angle_Control(&Leg[i]);
					}
					if(Slope(&Seesaw_Circle_Ramp[0]) == 1.0f && Slope(&Seesaw_Circle_Ramp[1]) == 1.0f && 
						 Slope(&Seesaw_Circle_Ramp[2]) == 1.0f && Slope(&Seesaw_Circle_Ramp[3]) == 1.0f){
						for(uint8_t i = 0;i<4;i++){
							ResetSlope(&Seesaw_Circle_Ramp[i]);
						}
						Part_State = 0;
						start_flag = 0;
						Robot->seesaw_event = Seesaw_None_Event;
						Robot_Move_State_Reset_Stop(Robot);
					}
				}
				break;
			}
		}
	}
}

void Robot_Move_Stair(Robot_Handle *Robot,Leg_Handle *Leg,Dev_Handle *IMU){
	
}
