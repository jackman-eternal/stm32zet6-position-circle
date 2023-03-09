#ifndef __PID_H
#define __PID_H

#include "stm32f10x.h"
#include "pwm.h"
#include "PID.h"
#include "Encoder.h"
#include "Serial.h"

float PID_realize(float actual_val,float tar1);
void autocallback(void);
float pid1(int16_t speed1,float tar1);
float pid_speed(int16_t speed1,float tar1);
void pwm_control(int pwm);
int16_t myabs(int a);


#endif


