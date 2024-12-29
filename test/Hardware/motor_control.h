#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include "main.h"
#include "tim.h"

typedef struct 
{
    float pitch;
    float roll;
    float yaw;
	
	short gyro_x;
	short gyro_y;
	short gyro_z;
}mpu6050_value;

typedef struct 
{
    float kp;
    float ki;
    float kd;
	
	float err;
	float err_last;   
	float value_expect;
	//float out_max;   //输出限幅
	//float iout_max;  //积分限幅
	
	float kp_out;
	float ki_out;
	float kd_out;
	float pid_out;
}PID;


int16_t pwm_limit(int16_t pwm_value);
void motor1_control(int16_t pwm_value);
void motor2_control(int16_t pwm_value);
int16_t encoder_left_velocity(void);
int16_t encoder_right_velocity(void);
void PID_Init(PID *pid, float kp, float ki, float kd);
float vertical_pid(float angle_real,float gyro_x, float angle);
float velocity_pid(int16_t v_left,int16_t v_right, int16_t velocity);
float lowpass(float input,float *previous_output,float alpha);
float turn_pid(float gyro_z, int8_t angle_turn);
void motor_control(void);

extern int16_t velocity,angle_turn;
extern int16_t left_v;
extern int16_t right_v;

extern PID pid_vertical;
extern PID pid_velocity;
extern PID pid_turn;

#endif

