#include "PID.h"                  // Device header
//p=3.5,i=0.4   //���õ� p i d 2.9 0.2 
float Err=0,last_err=0,next_err=0,pwm=0,add=0,p=3.1,i=0.002,d=0.2,ind=0;
float actual_speed = 0;  /*ʵ�ʲ���ٶ�*/
int actual_speed_int = 0;

int16_t myabs(int a)
{ 		   
	  int temp;
		if(a<0)  temp=-a;  
	  else temp=a;
	  return temp;
}

void pwm_control(int pwm)
{
      if(pwm>99)
         pwm=99;
      if(pwm<0)
         pwm=0;
}

float pid1(int16_t speed1,float tar1)
{
    speed1=myabs(speed1);
    Err=tar1-speed1;
    add=p*(Err-last_err)+i*(Err)+d*(Err+next_err-2*last_err);
    pwm+=add;
    pwm_control(pwm);
    next_err=last_err;
    last_err=Err;
    return pwm;
}

float pid_speed(int16_t speed1,float tar1)
{
	  speed1=myabs(speed1);
    Err=tar1-speed1;
	  Err+=Err;
	  add=p*Err+i*Err+d*(Err-last_err);
	  last_err = Err;
	  
    pwm = add;
	  pwm_control(pwm);
	  
	 return pwm;
}

void autocallback(void)
{
	static int encodernow;
	static int encoderlast;
	int res_pwm;/*PID����õ���PWMֵ*/
	int encodedelta; //encoder change
	encodernow = read_encoder()+overtime*65536;
	encodedelta = encodernow - encoderlast;
	encoderlast = encodernow;
	
	res_pwm =(int)PID_realize(encodernow,30*4*11);
	//res_pwm = pwm_val_protect((int)PID_realize(encodernow,30*4*11));
	/*�����������[�ܼ���ֵ]��ʵ�ֵ����λ�á�����*/
	printf("angle:%d,%d \r\n",encodernow,30*4*11);
//	printf("angle=%d,code=%d,over=%d,res=%d\r\n",encodernow,read_encoder(),overtime,res_pwm);
	set_motor_rotate(res_pwm/9);
}

float PID_realize(float actual_val,float tar1)
{
	/*����Ŀ��ֵ��ʵ��ֵ�����*/
	Err = tar1 - actual_val;
	/*������*/
  ind+= Err;
	/*PID�㷨ʵ��*/
	add = p*Err+i*ind+d*(Err-last_err);
	/*����*/
	last_err=Err;

	/*���ص�ǰʵ��ֵ*/
	return add;
}
