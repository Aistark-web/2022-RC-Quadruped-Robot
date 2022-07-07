#include "move.h"

extern Ramp_Typedef Init_Ramp;										//��ʼ��Ramp
extern Ramp_Typedef Shutdown_Ramp;								//�ػ�Ramp
extern Ramp_Typedef Step_Ramp[4];										//ԭ��̤��Ramp
extern Ramp_Typedef Straight_Ramp[4];							//ֱ��Ramp
extern Ramp_Typedef Turn_Ramp[4];									//����Ramp
extern Ramp_Typedef Circle_Ramp[4];								//ԭ��תRamp
extern Ramp_Typedef Jump_ready_Ramp;							//��Ծ׼��Ramp
extern Ramp_Typedef Jump_up_Ramp;									//��Ծ����Ramp
extern Ramp_Typedef Jump_up_back_Ramp;						//��Ծ����Ramp
extern Ramp_Typedef Jump_down_buffer_Ramp[4];			//��Ծ����Ramp
extern Ramp_Typedef Jump_back_buffer_Ramp[4];			//��Ծ�󷵻�ԭ��Ramp

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
			if(!Robot->Open_Motor_State){				//�������
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

/* ֱ�� */
void Robot_Move_Straight(Robot_Handle *Robot,Leg_Handle *Leg,Ramp_Typedef *Ramp,Dev_Handle *IMU){
	static uint8_t Part_State;																				//״̬
	static uint16_t initial_L_Y;																			//��ֵ
	static uint8_t start_flag;																				//��ʼ��־
	static float source_y = 60.0f;																		//y�������
	static float source_walk_z = 50.0f;																//z�������								//80.0f						
	static float line_y;																							//̧��/���� y�������
	static float line_z;																							//����		  y�������
	static float walk_z;																							//̧�Ÿ߶�		z����̧����ߵ�		
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
			start_flag = 0;								//��λ
			if(Remote_DataPack.L_Y_rocker > 500  || Remote_DataPack.L_Y_rocker < 3500 || Remote_DataPack.R_X_rocker < 500 || Remote_DataPack.R_X_rocker > 3500){ //�ж��Ƿ��������
				Robot_Move_State_Reset_Stop(Robot);											//״̬��λ0
			}
		}
	}
}


/* ���� */
void Robot_Move_Turn(Robot_Handle *Robot,Leg_Handle *Leg,Ramp_Typedef *Ramp,Dev_Handle *IMU){
	static uint8_t start_flag;
	static uint8_t Part_State;
	static uint16_t initial_R_X;
	static float short_y = 30.0f;									//�̲�
	static float long_y  = 60.0f;									//����
	static float walk_z  = 40.0f;									//̧����ߵ�
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
}

/* ̤�� */
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
	static float ready_y = -60.0f;								//׼������λ��y		-60.0f				//���¼���ϵ�����й�ϵ
	static float ready_z = 90.0f;									//׼������λ��z		150.0f
	static float jump_up_y	= -60.0f;							//����λ��y				-60.0f
	static float jump_up_z  = -220.0f;						//����λ��z				-150.0f
	static float jump_up_back_y = 120.0f;					//���������ջ�			120.0f
	static float jump_up_back_z = 220.0f;					//��Ծ�����ջ�			150.0f
	static float jump_down_buffer_y = 120.0f;			//��Ծ����y				120.0f
	static float jump_down_buffer_z = -180.0f;		//��Ծ����z				-150.0f
	static float jump_back_y = -120.0f;						//����ԭλ					-120.0f
	static float jump_back_z = 30.0f;							//����ԭλ					0.0f
	static float Part_State;											//״̬
	/* ׼������ */
	if(Part_State == 0){													
		if(!Jump_ready_Ramp.flag){
			for(uint8_t i = 0;i<4;i++){
				Leg_Point_RTO(ready_y,ready_z,&Leg[i]);
				Leg_Angle_Control(&Leg[i]);
			}
			Slope(&Jump_ready_Ramp);										// ����б�º�flag��1
		}
		else{
			if(Slope(&Jump_ready_Ramp) == 1.0f){				// �ﵽ����ʱ��
				ResetSlope(&Jump_ready_Ramp);							// ��λ
				Part_State = 1;
			}
		}
	}
	
	/* ���� */
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
	/* ���������� */
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

