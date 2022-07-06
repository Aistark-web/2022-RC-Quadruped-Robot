#include "motor_driver.h"

void Driver_Data_Packet(Driver_get_Handle *driver,HT_03_MOTOR_Handle *motor_array)
{
	switch(driver->driver_data.Dev_PIN){
		case DEV1_PIN_MASK:{
			HT_03_Motor_Data_packet(driver->driver_data.CAN_data[0],&motor_array[0]);
			HT_03_Motor_Data_packet(driver->driver_data.CAN_data[1],&motor_array[1]);
			HT_03_Motor_Data_packet(driver->driver_data.CAN_data[2],&motor_array[2]);
			HT_03_Motor_Data_packet(driver->driver_data.CAN_data[3],&motor_array[3]);
			break;
		}
		case DEV2_PIN_MASK:{
			HT_03_Motor_Data_packet(driver->driver_data.CAN_data[0],&motor_array[4]);
			HT_03_Motor_Data_packet(driver->driver_data.CAN_data[1],&motor_array[5]);
			HT_03_Motor_Data_packet(driver->driver_data.CAN_data[2],&motor_array[6]);
			HT_03_Motor_Data_packet(driver->driver_data.CAN_data[3],&motor_array[7]);
			break;
		}
	}
}

void Driver_Data_Unpack(Driver_return_Handle *driver,HT_03_MOTOR_Handle *motor_array)
{
	switch(driver->driver_data.Dev_PIN){
		case DEV1_PIN_MASK:{
			HT_03_Motor_Data_Unpack(driver->driver_data.CAN_data[0],&motor_array[0]);
			HT_03_Motor_Data_Unpack(driver->driver_data.CAN_data[1],&motor_array[1]);
			HT_03_Motor_Data_Unpack(driver->driver_data.CAN_data[2],&motor_array[2]);
			HT_03_Motor_Data_Unpack(driver->driver_data.CAN_data[3],&motor_array[3]);
			break;
		}
		case DEV2_PIN_MASK:{
			HT_03_Motor_Data_Unpack(driver->driver_data.CAN_data[0],&motor_array[4]);
			HT_03_Motor_Data_Unpack(driver->driver_data.CAN_data[1],&motor_array[5]);
			HT_03_Motor_Data_Unpack(driver->driver_data.CAN_data[2],&motor_array[6]);
			HT_03_Motor_Data_Unpack(driver->driver_data.CAN_data[3],&motor_array[7]);
			break;
		}
		default:
			break;
	}
}