#include "user_tasks.h"

uint8_t pData[3];
QueueHandle_t xQueue_rec;
mpu6050_value mpu_value;

void sensor_task(void const * argument)
{	
    while (1)
    {
        //HAL_GPIO_TogglePin(LED_GPIO_Port,LED_Pin);
        while(mpu_dmp_get_data(&mpu_value.pitch,&mpu_value.roll,&mpu_value.yaw)!=0){};
		MPU_Get_Gyroscope(&mpu_value.gyro_x, &mpu_value.gyro_y, &mpu_value.gyro_z);

        vTaskDelay(5);   
    }
    
}
void motor_task(void const * argument)
{
	PID_Init(&pid_vertical, 42.0f, 0.0f, -0.28f);         //kp正值 140   kd:负值  -0.7   
	PID_Init(&pid_velocity, -0.98f, -0.02f, 0.0f);		  //kp负值  ki负值 
	PID_Init(&pid_turn, 20.0f, 0.0f, 0.1f);
    while (1)
    {
		if( mpu_value.roll<35 && mpu_value.roll>-35)
			motor_control();
		else
		{
			motor1_control(0);
			motor2_control(0);
		}
        vTaskDelay(10);
    }
    
}
void Bluetooth_task(void const * argument)
{
	uint8_t buff[20];
	xQueue_rec = xQueueCreate( 5 ,sizeof(pData));                        
	HAL_UART_Receive_DMA(&huart3, pData, 3);
    while (1)
    {		
		Data_rec();	 
		
		sprintf(buff,"gyro_x:%hd  ",mpu_value.gyro_x);
		OLED_ShowString(0,0,buff,16);
		sprintf(buff,"gyro_z:%hd  ",mpu_value.gyro_z);
		OLED_ShowString(0,2,buff,16);

		sprintf(buff,"roll:%f   ",mpu_value.roll);
		OLED_ShowString(0,4,buff,16);
//		sprintf(buff,"right_v:%d   ",right_v);
//		OLED_ShowString(0,6,buff,16);
        vTaskDelay(30);
    }
    
}

