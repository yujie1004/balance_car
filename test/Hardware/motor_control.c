#include "motor_control.h"

#define PWM_MAX 700
#define PWM_MIN -700

int16_t velocity=0,angle_turn=0;
float mechanical_angle=-0.3f;  //陀螺仪初始角度误差

PID pid_vertical = { 0 };
PID pid_velocity = { 0 };
PID pid_turn = { 0 };

int16_t left_v;
int16_t right_v;

extern mpu6050_value mpu_value;

int16_t pwm_limit(int16_t pwm_value)  //为保护驱动芯片10V，设置pwm_value范围-700~700
{
	if(pwm_value > PWM_MAX)
		pwm_value = PWM_MAX;
	else if(pwm_value < PWM_MIN)
		pwm_value = PWM_MIN;
	else 
		pwm_value = pwm_value;
	return pwm_value;	
}

void motor1_control(int16_t pwm_value) 
{
    if(pwm_value > 0)   //正转  
    {
        HAL_GPIO_WritePin(AIN1_GPIO_Port,AIN1_Pin,GPIO_PIN_SET);
        HAL_GPIO_WritePin(AIN2_GPIO_Port,AIN2_Pin,GPIO_PIN_RESET);
        __HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_4,pwm_limit(pwm_value));
    }
    else
    {
        HAL_GPIO_WritePin(AIN1_GPIO_Port,AIN1_Pin,GPIO_PIN_RESET);
        HAL_GPIO_WritePin(AIN2_GPIO_Port,AIN2_Pin,GPIO_PIN_SET);
        __HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_4,-pwm_limit(pwm_value));
    }
}

void motor2_control(int16_t pwm_value)
{
    if(pwm_value > 0)   //正转
    {
        HAL_GPIO_WritePin(BIN1_GPIO_Port,BIN1_Pin,GPIO_PIN_SET);
        HAL_GPIO_WritePin(BIN2_GPIO_Port,BIN2_Pin,GPIO_PIN_RESET);
        __HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_1,pwm_limit(pwm_value));   	//PWM:0~900
    }
    else
    {
        HAL_GPIO_WritePin(BIN1_GPIO_Port,BIN1_Pin,GPIO_PIN_RESET);
        HAL_GPIO_WritePin(BIN2_GPIO_Port,BIN2_Pin,GPIO_PIN_SET);
        __HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_1,-pwm_limit(pwm_value));		//PWM:0~900
    }
}
//左轮速度
int16_t encoder_left_velocity(void)
{
	int16_t velocity;
	velocity = (short)__HAL_TIM_GetCounter(&htim2); 
	__HAL_TIM_SetCounter(&htim2 , 0); 
	return velocity;
}
//右轮速度
int16_t encoder_right_velocity(void)
{
	int16_t velocity;
	velocity = -(short)__HAL_TIM_GetCounter(&htim4); 
	__HAL_TIM_SetCounter(&htim4 , 0);
	return velocity;
}
//PID初始化
void PID_Init(PID *pid, float kp, float ki, float kd) 
{
    pid->kp = kp;             // 设置比例增益
    pid->ki = ki;             // 设置积分增益
    pid->kd = kd;             // 设置微分增益
}
//直立环--串级pid内环
float vertical_pid(float angle_real,float gyro_x, float angle)
{
	pid_vertical.value_expect = angle;
	pid_vertical.err = pid_vertical.value_expect-angle_real;
	pid_vertical.kp_out = pid_vertical.kp*pid_vertical.err;
	pid_vertical.kd_out = pid_vertical.kd*gyro_x;
	pid_vertical.pid_out = pid_vertical.kp_out + pid_vertical.kd_out;
	return pid_vertical.pid_out;
}

//lowpass--低通滤波
float lowpass(float input,float *previous_output,float alpha)
{
	float output = alpha * input + (1 - alpha) * (*previous_output);
	*previous_output = output;
	return output;
}


//速度环--串级pid外环
float velocity_pid(int16_t v_left,int16_t v_right, int16_t velocity)	//(v_left+v_right)/2最大约为56
{
	static float previous_output = 0.0f;
	
	pid_velocity.value_expect = velocity;
	pid_velocity.err = pid_velocity.value_expect-(v_left+v_right)/2;
	pid_velocity.err = lowpass(pid_velocity.err, &previous_output, 0.3);
	pid_velocity.kp_out = pid_velocity.kp*pid_velocity.err;
	pid_velocity.ki_out += pid_velocity.err;
	pid_velocity.ki_out = pid_velocity.ki * pid_velocity.ki_out;
	pid_velocity.ki_out = pid_velocity.ki_out>60?60:(pid_velocity.ki_out<-60?-60:pid_velocity.ki_out);//积分限幅
	pid_velocity.pid_out = pid_velocity.kp_out + pid_velocity.ki_out;
	return pid_velocity.pid_out;
}
//转向环
float turn_pid(float gyro_z, int8_t angle_turn)
{
	pid_turn.value_expect = angle_turn;
	pid_turn.err = pid_turn.value_expect;
	pid_turn.kp_out = pid_turn.kp * pid_turn.err;
	pid_turn.kd_out = pid_turn.kd * gyro_z;
	pid_turn.pid_out = pid_turn.kp_out + pid_turn.kd_out;
	return pid_turn.pid_out;
}
float velocity_pid_out;
float vertical_pid_out;
float turn_pid_out;

void motor_control(void)
{	
//	float velocity_pid_out;
//	float vertical_pid_out;
//	float turn_pid_out;
	int16_t pid_out1;
	int16_t pid_out2;
	left_v = encoder_left_velocity();
	right_v = encoder_right_velocity();
	velocity_pid_out = velocity_pid(left_v, right_v, velocity);
	vertical_pid_out = vertical_pid(mpu_value.roll,mpu_value.gyro_x,velocity_pid_out + mechanical_angle);
	turn_pid_out = turn_pid(mpu_value.gyro_z,angle_turn);
	motor1_control(vertical_pid_out + turn_pid_out);
	motor2_control(vertical_pid_out - turn_pid_out);
}