//=====================================================================================================
// main.c
//=====================================================================================================
//
// Design and implementation of auto-guided vehicle
//
// Details: 	http://www.mi-itec.com/auto-guided-vehicle
// Code:			http://github.com/MI-ITEC/auto-guided-vehicle
//				
// Date					Author       Notes
// 14/04/2018		Qian Zhao    Initial release
//
//=====================================================================================================


//---------------------------------------------------------------------------------------------------
// Header files

#include<intrins.h>
#include<reg52.h>

//---------------------------------------------------------------------------------------------------
// Definitions

//#define 	LINE	1		//��������ɫ��0 ��ɫ 1 ��ɫ��
//#define 	BKG		0		//�����ɫ

#define		PWM_PERIOD	90		// һ�����ڵ�PWM������������(ԭʼֵ20)

//---------------------------------------------------------------------------------------------------
// Port definitions

// Motors Enable
sbit P20 = P2 ^ 0;
sbit P21 = P2 ^ 1;
sbit P22 = P2 ^ 2;
sbit P23 = P2 ^ 3;
sbit P24 = P2 ^ 4;
sbit P25 = P2 ^ 5;
sbit P26 = P2 ^ 6;
sbit P27 = P2 ^ 7;

// PWM Ports
sbit left_pwm_port0 = P0 ^ 0;
sbit left_pwm_port1 = P0 ^ 1;
sbit right_pwm_port0 = P0 ^ 2;
sbit right_pwm_port1 = P0 ^ 3;

// Sensors
sbit c1 = P1 ^ 0;
sbit c2 = P1 ^ 1;
sbit c3 = P1 ^ 2;
sbit c4 = P1 ^ 3;
sbit c5 = P1 ^ 4;
sbit c6 = P1 ^ 5;
sbit c7 = P1 ^ 6;

// Jumper ��������
sbit line_config = P3^7;		// ���������ã��͵�ƽ��ɫ���ߵ�ƽ��ɫ��
sbit moter_state = P0^7 ;		// �ϵ繤�����ã��͵�ƽ���У��ߵ�ƽͣ����


//---------------------------------------------------------------------------------------------------
// Variable definitions

unsigned char left_pwm_cnt = 0;
unsigned char right_pwm_cnt = 15;
unsigned char left_pwm_ratio = 0;
unsigned char right_pwm_ratio = 0;

char motor_state = 'M';

unsigned char LINE = 1;
unsigned char BKG  = 0;


//---------------------------------------------------------------------------------------------------
// Function declarations

void initial(void);
	
void left_forward(unsigned char speed);
void left_backward(unsigned char speed);
//void left_brake(void);
void right_forward(unsigned char speed);
void right_backward(unsigned char speed);
//void right_brake(void);

void sensor_state(void);
void drive_motor(void);


//====================================================================================================
// Functions

//---------------------------------------------------------------------------------------------------
// ��ں���
void main()
{	
	initial();

	while (1) {
		sensor_state();
		drive_motor();
	}
}

//---------------------------------------------------------------------------------------------------
// ��ʼ�����ڡ���ʱ�������������Ϣ
void initial(void)
{
	// �������ã�����ͣ��
	if (moter_state) {
		while(1) {}
	}
		
	// �������ã�Ѳ������
	if (line_config) {
		// ����
		LINE = 0;
		BKG  = 1;
	}
	else {
		// ����
		LINE = 1;
		BKG  = 0;
	}
	
	EA = 1;       //Open master interrupt switch
	TMOD |= 0X01;
	TL0 = 0xA4;		//���ö�ʱ��ֵ
	TH0 = 0xFF;		//���ö�ʱ��ֵֵ
	ET0 = 1;
	TR0 = 1;
}

//---------------------------------------------------------------------------------------------------
// ����ռ�ձ�����PWM��ƽ
void pwm_switch(void)
{
	if (left_pwm_cnt <= left_pwm_ratio) {
		left_pwm_port0 = 1;
		left_pwm_port1 = 1;
	}
	else {
		left_pwm_port0 = 0;
		left_pwm_port1 = 0;
	}

	if (left_pwm_cnt >= PWM_PERIOD) {
		left_pwm_cnt = 0;
	}

	if (right_pwm_cnt <= right_pwm_ratio) {
		right_pwm_port0 = 1;
		right_pwm_port1 = 1;
	}
	else {
		right_pwm_port0 = 0;
		right_pwm_port1 = 0;
	}

	if (right_pwm_cnt >= PWM_PERIOD) {
		right_pwm_cnt = 0;
	}
}

//---------------------------------------------------------------------------------------------------
// ��ʱ��0 �� ����PWM���� 100us (��� 0.17 %)
void timer0() interrupt 1
{
	TL0 = 0xA4;		//���ö�ʱ��ֵ
	TH0 = 0xFF;		//���ö�ʱ��ֵ
	
	left_pwm_cnt++;
	right_pwm_cnt++;
	pwm_switch();
}

//---------------------------------------------------------------------------------------------------
// ���ֿ��ƺ���
void right_backward(unsigned char speed)
{
	right_pwm_ratio = speed;
	P24 = 1;
	P25 = 0;
	P26 = 1;
	P27 = 0;
}
void right_forward(unsigned char speed)
{
	right_pwm_ratio = speed;
	P24 = 0;
	P25 = 1;
	P26 = 0;
	P27 = 1;
}
//void right_brake(void)
//{
//	right_pwm_ratio = 0;
//	P24 = 0;
//	P25 = 0;
//	P26 = 0;
//	P27 = 0;
//}
void left_backward(unsigned char speed)
{
	left_pwm_ratio = speed;
	P20 = 1;
	P21 = 0;
	P22 = 1;
	P23 = 0;
}
void left_forward(unsigned char speed)
{
	left_pwm_ratio = speed;
	P20 = 0;
	P21 = 1;
	P22 = 0;
	P23 = 1;
}
//void left_brake(void)
//{
//	left_pwm_ratio = 0;
//	P20 = 0;
//	P21 = 0;
//	P22 = 0;
//	P23 = 0;
//}

//---------------------------------------------------------------------------------------------------
// ������״̬����
void sensor_state(void)
{
	if (c4 == LINE) {
		motor_state = 'M';
	}
	else if (c5 == LINE) {
		motor_state = 'l';
	}
	else if (c3 == LINE) {
		motor_state = 'r';
	}
	else if (c6 == LINE) {
		motor_state = 'L';
	}
	else if (c2 == LINE) {
		motor_state = 'R';
	}
	else if (c7 == LINE) {
		motor_state = '<';
	}
	else if (c1 == LINE) {
		motor_state = '>';
	}
	// else 
	// no changes;
	// �޴�����������״̬���䣻
}

//---------------------------------------------------------------------------------------------------
// �����������
void drive_motor(void)
{
	if (motor_state == 'M') {
		left_forward(7);
		right_forward(7);
	}
	else if (motor_state == 'l') {
		left_backward(3);
		right_forward(7);
	}
	else if (motor_state == 'r') {
		left_forward(7);
		right_backward(3);
	}
	else if (motor_state == 'L') {
		left_backward(7);
		right_forward(7);
	}
	else if (motor_state == 'R') {
		left_forward(7);
		right_backward(7);
	}
	// ����״̬����
	else if (motor_state == '<') {
		left_backward(7);
		right_forward(4);
	}
	else if (motor_state == '>') {
		left_forward(4);
		right_backward(7);
	}
}

//====================================================================================================
// END OF CODE
//====================================================================================================