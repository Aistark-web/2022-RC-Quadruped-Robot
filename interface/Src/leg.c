#include "leg.h"


#define LEG_Base_y                          0.0f//初始化y坐标
#define LEG_Base_z                          0.0f//初始化z坐标
#define LEG_DEFAULT_theta1                  0.0f//自启动后的初始化θ1
#define LEG_DEFAULT_theta2                  0.0f//自启动后的初始化θ2
#define LEG_L1                              130.0f//初始化L1
#define LEG_L2                              260.0f//初始化L2

   

#define DEFAULT_theta1															60.0f/180.0f*PI//自启动后的theta1   						//单位：度(°)
#define DEFAULT_theta2															60.0f/180.0f*PI//自启动后的theta2								//以下均为度数
#define FRONT_LEFT_LEG_initial_theta1               85.59f/180.0f*PI//前左腿未自启动前初始化theta1  85.59
#define FRONT_LEFT_LEG_initial_theta2               201.06f/180.0f*PI//前左腿未自启动前初始化theta2 201.06
#define FRONT_RIGHT_LEG_initial_theta1              85.59f/180.0f*PI//前右腿未自启动前初始化theta1  85.59
#define FRONT_RIGHT_LEG_initial_theta2              201.06f/180.0f*PI//前右腿未自启动前初始化theta2 201.06
#define BACK_LEFT_LEG_initial_theta1                68.12f/180.0f*PI//后左腿未自启动前初始化theta1  68.12
#define BACK_LEFT_LEG_initial_theta2                218.87f/180.0f*PI//后左腿未自启动前初始化theta2 218.87
#define BACK_RIGHT_LEG_initial_theta1               68.12f/180.0f*PI//后右腿未自启动前初始化theta1  68.12
#define BACK_RIGHT_LEG_initial_theta2               218.87f/180.0f*PI//后右腿未自启动前初始化theta2 218.87

void Leg_binding(Leg_Handle *Leg,HT_03_MOTOR_Handle *Motor1,HT_03_MOTOR_Handle *Motor2,LEG_ID_ENUM Leg_ID)
{
    Leg->Motor_1 = Motor1;
    Leg->Motor_2 = Motor2;
    Leg->Leg_ID = Leg_ID;
}

void Leg_get(Leg_Handle *Leg)
{
	#if ONE_LEG
	Leg->theta1 = Leg->MOTOR_2->Current.C_P + 45.0f/180.0f*PI;
	Leg->theta2 = Leg->Motor_1->Current.C_P + 45.0f/180.0f*PI;
	#else

    switch(Leg->Leg_ID){
        case FRONT_LEFT_LEG_ID:{
            Leg->theta1 = Leg->Motor_2->Current.C_P + FRONT_LEFT_LEG_initial_theta1;
            Leg->theta2 = Leg->Motor_1->Current.C_P + FRONT_LEFT_LEG_initial_theta2;
            break;
        }
        case FRONT_RIGHT_LEG_ID:{
            Leg->theta1 = -Leg->Motor_2->Current.C_P + FRONT_RIGHT_LEG_initial_theta1;
            Leg->theta2 = -Leg->Motor_1->Current.C_P + FRONT_RIGHT_LEG_initial_theta2;
            break; 
        }
        case BACK_LEFT_LEG_ID:{
            Leg->theta1 = Leg->Motor_2->Current.C_P + BACK_LEFT_LEG_initial_theta1;
            Leg->theta2 = Leg->Motor_1->Current.C_P + BACK_LEFT_LEG_initial_theta2;
            break;
        }
        case BACK_RIGHT_LEG_ID:{
            Leg->theta1 = -Leg->Motor_2->Current.C_P + BACK_RIGHT_LEG_initial_theta1;
            Leg->theta2 = -Leg->Motor_1->Current.C_P + BACK_RIGHT_LEG_initial_theta2;
            break;
        }
    }
    #endif
    Leg->Phi = (Leg->theta1 + Leg->theta2) / 2.0f;
    Leg->Psi = (Leg->theta1 - Leg->theta2) / 2.0f;
    #if defined(ARM_MATH_CM4)
    Leg->L = Solve_One_Quadratic_Equation(1.0f,-2.0f*LEG_L1*arm_cos_f32(Leg->Phi),LEG_L1*LEG_L1-LEG_L2*LEG_L2);
    Leg->y = LEG_Base_y + Leg->L * arm_sin_f32(Leg->Psi);
    Leg->z = LEG_Base_z - Leg->L * arm_cos_f32(Leg->Psi);
    #else
    Leg->L = Solve_One_Quadratic_Equation(1.0f,-2.0f*LEG_L1*cosf(Leg->Phi),LEG_L1*LEG_L1-LEG_L2*LEG_L2);
    Leg->y = LEG_Base_y + Leg->L * sinf(Leg->Psi);
    Leg->z = LEG_Base_z - Leg->L * cosf(Leg->Psi);
    #endif
		
}

void Leg_Angle_Control(Leg_Handle *Leg)
{
	switch(Leg->Leg_ID){
		case FRONT_LEFT_LEG_ID:
		{
			Leg->Motor_2->Expect.E_P = Leg->Infer_theta1 - FRONT_LEFT_LEG_initial_theta1;
			Leg->Motor_1->Expect.E_P = Leg->Infer_theta2 - FRONT_LEFT_LEG_initial_theta2;
			break;
		}
		case FRONT_RIGHT_LEG_ID:
		{
			Leg->Motor_2->Expect.E_P = -Leg->Infer_theta1 + FRONT_RIGHT_LEG_initial_theta1;
			Leg->Motor_1->Expect.E_P = -Leg->Infer_theta2 + FRONT_RIGHT_LEG_initial_theta2;
			break;
		}
		case BACK_LEFT_LEG_ID:
		{
			Leg->Motor_2->Expect.E_P = Leg->Infer_theta1 - BACK_LEFT_LEG_initial_theta1;
			Leg->Motor_1->Expect.E_P = Leg->Infer_theta2 - BACK_LEFT_LEG_initial_theta2;
			break;
		}
		case BACK_RIGHT_LEG_ID:
		{
			Leg->Motor_2->Expect.E_P = -Leg->Infer_theta1 + BACK_RIGHT_LEG_initial_theta1;
			Leg->Motor_1->Expect.E_P = -Leg->Infer_theta2 + BACK_RIGHT_LEG_initial_theta2;
			break;
		}
	}
}

uint8_t Leg_Move_Init(Leg_Handle *Leg,Ramp_Typedef *ramp){
	
	float ramp_t = Slope(ramp);
	Leg[0].Infer_theta1 = FRONT_LEFT_LEG_initial_theta1 + (DEFAULT_theta1 - FRONT_LEFT_LEG_initial_theta1) * ramp_t;
	Leg[0].Infer_theta2 = FRONT_LEFT_LEG_initial_theta2 + (DEFAULT_theta2 - FRONT_LEFT_LEG_initial_theta2) * ramp_t;
	Leg[1].Infer_theta1 = FRONT_RIGHT_LEG_initial_theta1 + (DEFAULT_theta1 - FRONT_RIGHT_LEG_initial_theta1) * ramp_t;
	Leg[1].Infer_theta2 = FRONT_RIGHT_LEG_initial_theta2 + (DEFAULT_theta2 - FRONT_RIGHT_LEG_initial_theta2) * ramp_t;
	Leg[2].Infer_theta1 = BACK_LEFT_LEG_initial_theta1 + (DEFAULT_theta1 - BACK_LEFT_LEG_initial_theta1) * ramp_t;
	Leg[2].Infer_theta2 = BACK_LEFT_LEG_initial_theta2 + (DEFAULT_theta2 - BACK_LEFT_LEG_initial_theta2) * ramp_t;
	Leg[3].Infer_theta1 = BACK_RIGHT_LEG_initial_theta1 + (DEFAULT_theta1 - BACK_RIGHT_LEG_initial_theta1) * ramp_t;
	Leg[3].Infer_theta2 = BACK_RIGHT_LEG_initial_theta2 + (DEFAULT_theta2 - BACK_RIGHT_LEG_initial_theta2) * ramp_t;

	for(uint8_t i=0;i<4;i++){
		float Psi;
		float Phi;
    Phi = (Leg[i].Infer_theta1 + Leg[i].Infer_theta2) / 2.0f;
    Psi = (Leg[i].Infer_theta1 - Leg[i].Infer_theta2) / 2.0f;
		Leg[i].Infer_L = Solve_One_Quadratic_Equation(1.0f,-2.0f*LEG_L1*arm_cos_f32(Phi),LEG_L1*LEG_L1-LEG_L2*LEG_L2);
    Leg[i].Infer_y = LEG_Base_y + Leg[i].Infer_L * arm_sin_f32(Psi);
    Leg[i].Infer_z = LEG_Base_z - Leg[i].Infer_L * arm_cos_f32(Psi);
		Leg_Angle_Control(&Leg[i]);
	}
	if(ramp_t == 1.0f)
		return 1;
	else 
		return 0;
}

uint8_t Leg_Reset_Ramp(Leg_Handle *Leg,Ramp_Typedef *Ramp)
{
	float ramp_t;
	static float initial_theta1[4];
	static float initial_theta2[4];
	static float Psi;
	static float Phi;
	if(!Ramp->flag){
		for(uint8_t i = 0;i<4;i++){
			initial_theta1[i] = Leg[i].Infer_theta1;
			initial_theta2[i] = Leg[i].Infer_theta2;
		}
	}
	ramp_t = Slope(Ramp);
	for(uint8_t i =0;i<4;i++){
		Leg[i].Infer_theta1 = initial_theta1[i] + (DEFAULT_theta1 - initial_theta1[i]) * ramp_t;
		Leg[i].Infer_theta2 = initial_theta2[i] + (DEFAULT_theta2 - initial_theta2[i]) * ramp_t;
	}
	for(uint8_t i=0;i<4;i++){
    Phi = (Leg[i].Infer_theta1 + Leg[i].Infer_theta2) / 2.0f;
    Psi = (Leg[i].Infer_theta1 - Leg[i].Infer_theta2) / 2.0f;
		Leg[i].Infer_L = Solve_One_Quadratic_Equation(1.0f,-2.0f*LEG_L1*arm_cos_f32(Phi),LEG_L1*LEG_L1-LEG_L2*LEG_L2);
    Leg[i].Infer_y = LEG_Base_y + Leg[i].Infer_L * arm_sin_f32(Psi);
    Leg[i].Infer_z = LEG_Base_z - Leg[i].Infer_L * arm_cos_f32(Psi);
		Leg_Angle_Control(&Leg[i]);
	}
	if(ramp_t == 1.0f)
		return 1;
	else
		return 0;
}

void Leg_Point_RTO(float y,float z,Leg_Handle *Leg)
{
	float initial_y;
	float initial_z;
	float Psi,Phi;
	initial_y = Leg->Infer_y;
	initial_z = Leg->Infer_z;
	Leg->Infer_y = initial_y + y;
	Leg->Infer_z = initial_z + z;
	Leg->Infer_L = sqrtf(Leg->Infer_y*Leg->Infer_y+Leg->Infer_z*Leg->Infer_z);
	Psi = asinf(Leg->Infer_y/Leg->Infer_L);
	Phi = acosf((LEG_L1*LEG_L1 + Leg->Infer_L*Leg->Infer_L - LEG_L2 * LEG_L2)/(2.0f*LEG_L1*Leg->Infer_L));
	Leg->Infer_theta1 = (Phi + Psi);
	Leg->Infer_theta2 = (Phi - Psi);
}

void Leg_Point_RTO_Ramp(float y,float z,Leg_Handle *Leg,Ramp_Typedef *Ramp)
{
	static float initial_z[4];
	static float initial_y[4];
	static float Psi;
	static float Phi;
	static float ramp_t;													
	if(!Ramp->flag){
		initial_z[Leg->Leg_ID-1] = Leg->Infer_z;
		initial_y[Leg->Leg_ID-1] = Leg->Infer_y;
	}
	ramp_t = Slope(Ramp);
	Leg->Infer_y = initial_y[Leg->Leg_ID-1] + y*ramp_t;
	Leg->Infer_z = initial_z[Leg->Leg_ID-1] + z*ramp_t;
	Leg->Infer_L = sqrtf(Leg->Infer_y*Leg->Infer_y+Leg->Infer_z*Leg->Infer_z);
	Psi = asinf(Leg->Infer_y/Leg->Infer_L);
	Phi = acosf((LEG_L1*LEG_L1 + Leg->Infer_L*Leg->Infer_L - LEG_L2 * LEG_L2)/(2.0f*LEG_L1*Leg->Infer_L));
	Leg->Infer_theta1 = (Phi + Psi);
	Leg->Infer_theta2 = (Phi - Psi);
}


void Leg_New_Walk(float s,float h,Leg_Handle *Leg,Ramp_Typedef *Ramp)
{
	static float initial_y[4];
	static float initial_z[4];
	static float Psi,Phi;
	static float ramp_t;
	if(!Ramp->flag){
		initial_y[Leg->Leg_ID-1] = Leg->Infer_y;
		initial_z[Leg->Leg_ID-1] = Leg->Infer_z;
	}
	ramp_t = Slope(Ramp);
	if(ramp_t < 0.5f)
	{
		#if defined(ARM_MATH_CM4)
			Leg->Infer_z = initial_z[Leg->Leg_ID-1] + ((2.0f*h*(ramp_t - arm_sin_f32(4.0f*PI*ramp_t)/(4.0f*PI))));
		#else
			Leg->Infer_z = initial_z[Leg->Leg_ID-1] + ((2.0f*h*(ramp_t - sinf(4.0f*PI*ramp_t)/(4.0f*PI))));
		#endif
	}
	else
	{
		#if defined(ARM_MATH_CM4)
			Leg->Infer_z = initial_z[Leg->Leg_ID-1] + ((2.0f*h*(1.0f-ramp_t + (arm_sin_f32(4.0f*PI*ramp_t)/(4.0f*PI)))));
		#else
			Leg->Infer_z = initial_y[Leg->Leg_ID-1] + ((2.0f*h*(1.0f-ramp_t + (sinf(4.0f*PI*ramp_t)/(4.0f*PI)))));
		#endif
	}
	Leg->Infer_y  = initial_y[Leg->Leg_ID-1] + (s*(ramp_t - arm_sin_f32(2.0f*PI*ramp_t)/(2.0f*PI)));
	Leg->Infer_L = sqrtf(Leg->Infer_y*Leg->Infer_y+Leg->Infer_z*Leg->Infer_z);
	Psi = asinf(Leg->Infer_y/Leg->Infer_L);
	Phi = acosf((LEG_L1*LEG_L1 + Leg->Infer_L*Leg->Infer_L - LEG_L2 * LEG_L2)/(2.0f*LEG_L1*Leg->Infer_L));
	Leg->Infer_theta1 = (Phi + Psi);
	Leg->Infer_theta2 = (Phi - Psi);
	
}

