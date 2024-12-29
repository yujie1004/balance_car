#ifndef USER_TASKS_H
#define USER_TASKS_H

#include "main.h"
#include "usart.h"
#include "motor_control.h"
#include "oled.h"
#include "cmsis_os.h"
#include "Data_Analysis.h"

extern uint8_t pData[3];
extern QueueHandle_t xQueue_rec;

void sensor_task(void const * argument);
void motor_task(void const * argument);
void Bluetooth_task(void const * argument);

#endif
