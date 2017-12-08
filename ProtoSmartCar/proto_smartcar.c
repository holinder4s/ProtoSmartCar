/*
 * proto_smartcar.c
 *
 *  Created on: 2017. 11. 24.
 *      Author: holinder4s
 */

#include <misc.h>
#include <stm32f10x.h>
#include <stm32f10x_gpio.h>
#include <stm32f10x_rcc.h>
#include <stm32f10x_usart.h>
#include <stm32f10x_adc.h>
#include <stm32f10x_tim.h>
#include <stm32f10x_dma.h>
#include <string.h>
#include <lcd.h>
#include <Touch.h>
#include <MPU6050.h>
#include <stdbool.h>
#include <math.h>
#include <fingerprint.h>

#define GPIOD_IDR (*(volatile unsigned *)0x40011408)

/* Globl Variables
 *    1) ADC_result_value_arr[5] : [0](조도센서), [1](빗물감지센서), [2](인체감지센서), [3](온습도센서-온도), [4](온습도센서-습도)
 *    2) command[100] : 차 움직임 명령어 버퍼
 *    3) command_pos : 명령어 위치 초기화 용도
 *    4) voiceBuffer : 보이스 명령어 문자열 셋
 *    5) rain_power_flag : 비의 세기
 *    6) servo_direction : 와이퍼를 주기적으로 왕복시키기 위한 방향 플래그
 *    7) timer_count : 비의 세기에 따라 와이퍼의 속도를 조절하기 위한 timer count
 *    8) voice_command_enable : 음성 명령 모드 활성화 플래그
 *    9) AccelGyro[6] : 가속도 센서 Raw 데이터
 *    10) AccelGyroAverage_flag : true이면 average계산 모드(초기화), false이면 정상 동작
 *    11) base_???_? : 가속도 센서 Raw 데이터 평균 값
 *    12) last_???_??? : 최신 가속도 센서 가공 값
 *    13) g_currentTick : TickCount값 => 시간 계산을 위해 이용(1msec)
 *    14) speed_updown_timer_count : 속도 변조를 1초마다 하기위해 세팅
 *    15) catcher_status : 트랩상태(0-상승엣지,1-하강엣지)
 *    16) duration : 마지막으로 캡처한 펄스의 지속 시간
 *    17) temperature : 온도 값
 *    18) humidity : 습도 값
 *    19) fingerprint_mode : 지문 센서 동작 모드(0:등록, 1:검증)
 *    20) lock_flag : 문열림잠금상태 */
__IO uint32_t ADC_result_value_arr[5];
char command[100];
int command_pos=0;

u8 *voiceBuffer[] = {"Turn on the light", "Turn off the light", "Play music", "Pause", "Next", "Previous",
		"Up", "Down", "Turn on the TV", "Turn off the TV", "Increase temperature", "Decrease temperature",
		"What's the time", "Open the door", "Close the door", "Left", "Right", "Stop", "Start", "Mode 1", "Mode 2", "Go",};

int rain_power_flag = 0;
int servo_direction = 0;
int timer_count = 0;
int voice_command_enable = 0;

/* 가속도 센서 관련 */
int16_t  AccelGyro[6]={0};
bool AccelGyroAverage_flag = true;
uint16_t base_accel_x=0, base_accel_y=0, base_accel_z=0;
uint16_t base_gyro_x=0, base_gyro_y=0, base_gyro_z=0;
unsigned long last_read_time;
float last_angle_x, last_angle_y, last_angle_z;
float last_gyro_angle_x, last_gyro_angle_y, last_gyro_angle_z;

/* 현재 Tick Count값을 저장하는 전역변수 */
uint32_t g_currentTick = 0;

/* 속도 변조를 위한 타이머 카운트 전역변수 */
int speed_updown_timer_count = 0;

/* 초음파 거리센서(HC-SR04모듈)
 *    1) catcher_status : 트랩상태(0-상승엣지,1-하강엣지)
 *    2) duration : 마지막으로 캡처한 펄스의 지속 시간 */
uint8_t catcher_status = 0;
uint16_t duration = 0;

/* ETH-01DV 온습도 센서 */
u8 temperature;
u8 humidity;

/* Fingerprint Mode Flag
 * -1 : sleep 상태
 * 0 : 모든 사용자 초기화
 * 1 : 지문대조모드 */
int fingerprint_mode = -1;
bool lock_flag = false;

bool stop_flag = true;

void RCC_Configure(void);
void USART_Configure(void);
void _GPIO_LEDInit(void);
void _GPIO_MOTOR1Init(void);
void _GPIO_MOTOR2Init(void);
void _GPIO_MOTORInit(void);
void _GPIO_ADCInit(void);
void _GPIO_TIM3Ch3Ch2Ch1Init(void);
void _GPIO_UltraDistance(void);
void _GPIO_Button(void);
void GPIO_Configure(void);
void ADC_Configure(void);
void _GPIO_LedControl(void);
void _GPIO_TempControl(void);
void ADC_Initialize(void);
void _TIM3Ch3Ch2Ch1_PWM_Configure(void);
void _TIM2_Configure(void);
void _TIM1_Configure(void);
void _TIM6_Configure(void);
void _TIM7_Configure(void);
void TIM_PWM_Configure(void);
void DMA_Configure(void);
void _USART_ITR_Configure(void);
void _TIM2_ITR_Configure(void);
void _TIM6_ITR_Configure(void);
void _TIM7_ITR_Configure(void);
void _EXTI_Ultra_ITR_Configure(void);
void Interrupt_Configure(void);
void SendString(USART_TypeDef* USARTx, char* string);
void _command_forward(void);
void _command_backward(void);
void _command_left(void);
void _command_right(void);
void _command_stop(void);
void command_move(int select);
void change_pwm_servo_duty_cycle(int percentx10);
void door_control(bool flag);
void speed_up_down(int percentx10);
void AccelGyroInitialize(void);
void calculate_accelgyro_average(int total_readnum);
void set_last_read_angle_data(unsigned long time, float x, float y, float z, float gyro_x, float gyro_y, float gyro_z);
void PrintAccelGryroRaw2Angle(int16_t accelgyro[6]);
uint32_t GetTickCount(void);
void UIOutline_Init(void);
void Delay_ms(uint32_t ms);
void UltraDistance_Init(void);
uint16_t getDistance(void);
//void USART1_IRQHandler(void);
void USART2_IRQHandler(void);
void USART3_IRQHandler(void);
void TIM6_IRQHandler(void);
void TIM2_IRQHandler(void);
void TIM1_IRQHandler(void);
void TIM6_IRQHandler(void);
void TIM7_IRQHandler(void);
void EXTI0_IRQHandler(void);

int main(void) {
	/* 온습도센서 Variables */
	float humi_val_real = 0.0;
	float temp_val_real = 0.0;

	/* 초음파 거리센서 거리 값 */
	int distance;

	/* 지문인식센서 등록 사람 수 */
	int i=1;

	/* 초기화 및 Configuration 작업 */
	RCC_Configure();
	LCD_Init();
	USART_Configure();
	GPIO_Configure();
	ADC_Configure();
	TIM_PWM_Configure();
	DMA_Configure();
	MPU6050_I2C_Init();			// MPU6050을 사용하기 위한 I2C 설정
	Interrupt_Configure();

	ADC_Initialize();

	/* I2C를 이용한 MPU6050 초기화 */
	MPU6050_Initialize();

	/* HC-SR04 초음파 거리센서 초기화 */
	UltraDistance_Init();

	/* 메인 UI틀 초기화 */
	LCD_Clear(WHITE);
	UIOutline_Init();
	//LCD_ShowString(50, 130, "Accident Occur!!!", RED, WHITE);

	/* 지문인식센서 초기화 : 실제로는 삭제해야할 부분 */
	ClearAllUser();
	if(GetUserCount() != 1) {
		AddUser(1);
		delay();
	}

	/* 속도 초기화 */
	LCD_ShowString(180,100, "0 m/s", BLACK, WHITE);

	/* Debug용 : 나중에 삭제할 것! */
	//GPIOResetBits(GPIOD, GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_7);
	while (1) {
		/* 조도센서 값 LCD에 출력
		 *    1) x > 3000 : 밤(어두움)
		 *    2) else : 낮(밝음) */
		LCD_ShowNum(190,225,ADC_result_value_arr[0],5,BLACK,WHITE);
		if(ADC_result_value_arr[0] > 3000) {
			GPIO_SetBits(GPIOD,GPIO_Pin_4);
			GPIO_SetBits(GPIOD,GPIO_Pin_7);
			GPIO_SetBits(GPIOC,GPIO_Pin_10);
			GPIO_SetBits(GPIOC,GPIO_Pin_11);
		}
		else {
			GPIO_ResetBits(GPIOD,GPIO_Pin_4);
			GPIO_ResetBits(GPIOD,GPIO_Pin_7);
			GPIO_ResetBits(GPIOC,GPIO_Pin_10);
			GPIO_ResetBits(GPIOC,GPIO_Pin_11);
		}

		/* 빗물 감지 센서 값 LCD에 출력
		 *    1) x < 3000 : 강한 빗물
		 *    2) x < 3500 : 약한 빗물
		 *    3) else : 빗물 x */
		LCD_ShowNum(190,245,ADC_result_value_arr[1],5,BLACK,WHITE);
		if(ADC_result_value_arr[1]<3000) {
			rain_power_flag = 2;
			//GPIO_ResetBits(GPIOD,GPIO_Pin_3);
		}
		else if(ADC_result_value_arr[1]<3500) {
			rain_power_flag = 1;
			//GPIO_ResetBits(GPIOD,GPIO_Pin_4);
		}
		else {
			rain_power_flag = 0;
			//GPIO_ResetBits(GPIOD,GPIO_Pin_7);
		}

		/* 인체 감지 센서 값 LCD에 출력
		 *    1) x < 2000 : 사람 없음
		 *    2) x > 2000 : 사람 감지 */
		LCD_ShowNum(190,265,ADC_result_value_arr[2],5,BLACK,WHITE);
		if(ADC_result_value_arr[2] > 2000) {
			voice_command_enable = 1;
		}else {
			voice_command_enable = 0;
		}

		/* MPU6050 6축 자이로 가속도 센서 사용
		 *    1) 경사 측정 : raw sensor데이터를 받아 각도 계산
		 *    2) 20도 이상 : DC 모터 출력 60%(속도 향상)
		 *    3) 40도 이상 : DC 모터 출력 70%(속도 향상)
		 *    4) -20도 ~ 20도 : DC 모터 출력 50%(평균 속도)
		 *    5) -20도 이하 : DC 모터 출력 40%(속도 저하)
		 *    6) -40도 이하 : DC 모터 출력 30%(속도 저하) */
		if (MPU6050_TestConnection() != 0) {
			if(AccelGyroAverage_flag == true) {
				AccelGyroInitialize();
				calculate_accelgyro_average(10);
				AccelGyroAverage_flag = false;
			}else {

				/* Sensor로부터 데이터 받아오기 */
				MPU6050_GetRawAccelGyro(AccelGyro);
				/* 센서 raw데이터를 각도로 가공 및 출력 */
				PrintAccelGryroRaw2Angle(AccelGyro);

				/* 경사각측정과 DC모터 속도 제어를 통해 차량 속도를 유지 */
				/* 1초마다 TIM2 Interrupt Handler에서 속도 제어 */

			}
		}

		/* HC-SR04 초음파 거리센서 : 앞에 장애물이 있을 경우 멈춤. */
		distance = getDistance();
		LCD_ShowNum(190,285, distance, 5, BLACK, WHITE);
		if(distance <= 2500)
			command_move(0);

		/* 온습도센서
		 *   1) 온도 측정 : float형의 temp_val_real에 저장
		 *   2) 습도 측정 : float형의 humi_val_real에 저장 */
		//temp_val_real = ((ADC_result_value_arr[3] / 6000) / 5.0) * 217.75 - 66.875;
		//humi_val_real = ((ADC_result_value_arr[4] / 6000) / 5.0) * 125 -12.5;
		temp_val_real = ADC_result_value_arr[3] / 100;
		humi_val_real = ADC_result_value_arr[4] / 100;
		LCD_ShowNum(180,40, (int)temp_val_real, 4, BLACK, WHITE);
		LCD_ShowNum(180,70, (int)humi_val_real, 4, BLACK, WHITE);

		if(temp_val_real > 20) {
			GPIO_SetBits(GPIOC, GPIO_Pin_14);
		}else if(temp_val_real < 23) {
			GPIO_SetBits(GPIOC, GPIO_Pin_15);
		}else {
			GPIO_ResetBits(GPIOC, GPIO_Pin_14);
			GPIO_ResetBits(GPIOC, GPIO_Pin_15);
		}

		/* 지문인식센서 */
		if(!(GPIOD_IDR & 0x800))
			fingerprint_mode = 0;
		else if(!(GPIOD_IDR & 0x1000))
			fingerprint_mode = 1;

		switch(fingerprint_mode) {
		case 0:
			if(VerifyUser(1) == (u8)1) {
				ClearAllUser();
				LCD_ShowString(50, 130, "                           ", BLACK, WHITE);
				LCD_ShowString(50, 130, "All users cleared!", BLACK, WHITE);
				i = 1;
				delay();
				LCD_ShowString(50, 130, "                           ", BLACK, WHITE);
				LCD_ShowString(50, 130, "Register Admin!", BLACK, WHITE);
				AddUser(i);
				i++;
				delay();
				LCD_ShowString(50, 130, "                           ", BLACK, WHITE);
				LCD_ShowString(50, 130, "Register User1!", BLACK, WHITE);
				AddUser(i);
				i++;
				LCD_ShowString(50, 130, "                           ", BLACK, WHITE);
				LCD_ShowString(50, 130, "Register Success!", BLACK, WHITE);
			}
			fingerprint_mode = -1;
			break;
		case 1:
			if(VerifyUser(1)==(u8)1) {
				LCD_ShowString(50, 130, "                           ", BLACK, WHITE);
				LCD_ShowString(50, 130, "Admin Match success!", BLACK, WHITE);
				lock_flag = true;
			}else if(VerifyUser(2)==(u8)2) {
				LCD_ShowString(50, 130, "                           ", BLACK, WHITE);
				LCD_ShowString(50, 130, "User1 Match success!", BLACK, WHITE);
				lock_flag = true;
			}else {
				LCD_ShowString(50, 130, "                           ", BLACK, WHITE);
				LCD_ShowString(50, 130, "Match Failed..:(", BLACK, WHITE);
				lock_flag = false;
			}
			door_control(lock_flag);
			lock_flag = false;
			delay();
			delay();
			door_control(lock_flag);
			fingerprint_mode = -1;
			break;
		}
	}
}

void RCC_Configure(void) {
	/* Enable Clock for Interrupt EXTI(외부 인터럽트) */
	//RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO);

	/* Enable Clock for Bluetooth(USART2)(안드로이드->보드) and USART1(지문인식센서) and USART3(PortB)(음성인식센서) */
	// GPIOA포트는 USART에 clock때문에 사용해야하는지 사용 안해도 되는지 잘 모르겠음
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2ENR_USART1EN, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1ENR_USART2EN | RCC_APB1ENR_USART3EN, ENABLE);

	/* 모터드라이버
	 *    1) 모터드라이버1(in1:PA4,in2:PA5,in3:PE2,in4:PE3)
	 *    2) 모터드라이버2(in1:PE7,in2:PE8,in3:PE9,in4:PE10)  */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE);

	/* LED & Button */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);

	/* 조도센서와 빗물감지센서를 사용하기 위한 ADC & DMA 클럭 인가 */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1,ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1,ENABLE);

	/* Servo Motor를 제어하기 위한 TIM3_Channel3(PB0) 사용 */
	/* 바퀴 속도 제어(PWM)를 위한 TIM3_Channel1(PA6) 사용 */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

	/* 와이퍼 작동시 주기 설정을 위한 TIM2 사용 */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

	/* 가속도 GetTickCount()역할을 하는 함수를 위한 TIM1(1msec주기) 클럭 인가 */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);

	/* 초음파 거리센서(Trig:PE6, Echo:PA0)
	 *    1) 에코펄스 지속시간 계산 + 에코 라인의 잔류 진동 감쇄에 필요 : TIM6 사용
	 *    2) 타이머의 타이밍을 켬 : TIM7
	 *    3) 외부인터럽트는 기본적으로 전체 포트 A에 대해 활성화됨(여기서는 PA0사용) */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2ENR_AFIOEN , ENABLE);
}

void USART_Configure(void) {
	/* ## UART 통신 & BT 세팅 or 지문인식센서 : USART1
	 *    1) 지문인식센서일 경우 baudrate는 19200
	 *    2) 나머지 경우는 9600 */
	/* Bluetooth module : USART2 */
	USART_InitTypeDef USART_init;

	/* USART 기본 설정 값 세팅 */
	//USART_init.USART_BaudRate = 9600;			// Debug 용도로 사용할 때
	USART_init.USART_BaudRate = 19200;			// 지문인식센서로 사용할 때
	USART_init.USART_WordLength = USART_WordLength_8b;
	USART_init.USART_StopBits = USART_StopBits_1;
	USART_init.USART_Parity = USART_Parity_No;
	USART_init.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_init.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

	/* USART1 Init(블루투스모듈 설정 또는 지문인식센서모듈을 위해 사용) */
	/* USART2 Init(블루투스 수신을 위해 사용, 안드로이드->보드)
	 * USART3 Init(음성인식센서 사용) */
	USART_Init(USART1, &USART_init);

	USART_init.USART_BaudRate = 9600;
	USART_Init(USART2, &USART_init);
	USART_Init(USART3, &USART_init);

	/* Enable USART1, USART2, USART3 Receive/Transmit interrupts */
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);

	/* Enable USART1, USART2, USART3 */
	USART_Cmd(USART1, ENABLE);
	USART_Cmd(USART2, ENABLE);
	USART_Cmd(USART3, ENABLE);
}

void _GPIO_USARTInit(void) {
	GPIO_InitTypeDef GPIO_USART_init;

	/* USART의 input clock을 위한 GPIOA의 기본 값 세팅 */
	GPIO_USART_init.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_USART_init.GPIO_Speed = GPIO_Speed_50MHz;

	/* USART1 GPIO 세팅 */
	/* GPIOA의 9번 10번 핀을 Alternative Function으로 세팅 */
	GPIO_USART_init.GPIO_Pin = GPIO_Pin_9;
	GPIO_Init(GPIOA, &GPIO_USART_init);

	GPIO_USART_init.GPIO_Pin = GPIO_Pin_10;
	GPIO_USART_init.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA, &GPIO_USART_init);

	/* USART2 GPIO 세팅 */
	/* GPIOA의 2번 3번 핀을 Alternative Function으로 세팅 */
	GPIO_USART_init.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_USART_init.GPIO_Speed = GPIO_Speed_50MHz;

	GPIO_USART_init.GPIO_Pin = GPIO_Pin_2;
	GPIO_Init(GPIOA, &GPIO_USART_init);

	GPIO_USART_init.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_USART_init.GPIO_Pin = GPIO_Pin_3;
	GPIO_Init(GPIOA, &GPIO_USART_init);

	/* USART3 GPIO 세팅 */
	/* GPIOB의 10번 11번 핀을 Alternative Function으로 세팅 */
	GPIO_USART_init.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_USART_init.GPIO_Speed = GPIO_Speed_50MHz;

	GPIO_USART_init.GPIO_Pin = GPIO_Pin_10;
	GPIO_Init(GPIOB, &GPIO_USART_init);

	GPIO_USART_init.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_USART_init.GPIO_Pin = GPIO_Pin_11;
	GPIO_Init(GPIOB, &GPIO_USART_init);
}

void _GPIO_LEDInit(void) {
	GPIO_InitTypeDef GPIOD_LED_init;

	GPIOD_LED_init.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIOD_LED_init.GPIO_Speed = GPIO_Speed_50MHz;

	GPIOD_LED_init.GPIO_Pin = GPIO_Pin_2;
	GPIO_Init(GPIOD, &GPIOD_LED_init);

	GPIOD_LED_init.GPIO_Pin = GPIO_Pin_3;
	GPIO_Init(GPIOD, &GPIOD_LED_init);

	GPIOD_LED_init.GPIO_Pin = GPIO_Pin_4;
	GPIO_Init(GPIOD, &GPIOD_LED_init);

	GPIOD_LED_init.GPIO_Pin = GPIO_Pin_7;
	GPIO_Init(GPIOD, &GPIOD_LED_init);

	/* LED Init State : LED1, LED2, LED3, LED4 on*/
	/* Todo : GPIO_SetBits로 바꿔주기(지문인식센서때문에 임시로 Reset) */
	GPIO_ResetBits(GPIOD, GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_7);
	GPIO_SetBits(GPIOD, GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_7);
}

void _GPIO_MOTOR1Init(void) {
	GPIO_InitTypeDef GPIOB_MOTOR1_init;

	GPIOB_MOTOR1_init.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIOB_MOTOR1_init.GPIO_Speed = GPIO_Speed_50MHz;

	GPIOB_MOTOR1_init.GPIO_Pin = GPIO_Pin_5;
	GPIO_Init(GPIOB, &GPIOB_MOTOR1_init);

	GPIOB_MOTOR1_init.GPIO_Pin = GPIO_Pin_8;
	GPIO_Init(GPIOB, &GPIOB_MOTOR1_init);

	GPIOB_MOTOR1_init.GPIO_Pin = GPIO_Pin_12;
	GPIO_Init(GPIOC, &GPIOB_MOTOR1_init);

	GPIOB_MOTOR1_init.GPIO_Pin = GPIO_Pin_13;
	GPIO_Init(GPIOC, &GPIOB_MOTOR1_init);

	/* 모터 정지 모드로 초기화 */
	GPIO_ResetBits(GPIOB, GPIO_Pin_5 | GPIO_Pin_8);
	GPIO_ResetBits(GPIOC, GPIO_Pin_12 | GPIO_Pin_13);
}

void _GPIO_MOTOR2Init(void) {
	GPIO_InitTypeDef GPIOE_MOTOR2_init;

	GPIOE_MOTOR2_init.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIOE_MOTOR2_init.GPIO_Speed = GPIO_Speed_50MHz;

	GPIOE_MOTOR2_init.GPIO_Pin = GPIO_Pin_1;
	GPIO_Init(GPIOB, &GPIOE_MOTOR2_init);

	GPIOE_MOTOR2_init.GPIO_Pin = GPIO_Pin_2;
	GPIO_Init(GPIOB, &GPIOE_MOTOR2_init);

	GPIOE_MOTOR2_init.GPIO_Pin = GPIO_Pin_7;
	GPIO_Init(GPIOC, &GPIOE_MOTOR2_init);

	GPIOE_MOTOR2_init.GPIO_Pin = GPIO_Pin_8;
	GPIO_Init(GPIOC, &GPIOE_MOTOR2_init);

	/* 모터 정지 모드로 초기화 */
	GPIO_ResetBits(GPIOB, GPIO_Pin_1 | GPIO_Pin_2);
	GPIO_ResetBits(GPIOC, GPIO_Pin_7 | GPIO_Pin_8);
}

void _GPIO_MOTORInit(void) {
	_GPIO_MOTOR1Init();
	_GPIO_MOTOR2Init();
}

void _GPIO_ADCInit(void){
	/* ADC Channel 11 : GPIOC Pin 1(조도센서)
	 * ADC Channel 12 : GPIOC Pin 2(빗물감지센서)
	 * ADC Channel 13 : GPIOC Pin 3(인체감지센서)
	 * ADC Channel 14 : GPIOC Pin 4(온습도 - 온도)
	 * ADC Channel 15 : GPIOC Pin 5(온습도 - 습도)
	 * GPIOC mode : Analog Input Mode */
	GPIO_InitTypeDef GPIOC_ADC_init;
	GPIOC_ADC_init.GPIO_Mode = GPIO_Mode_AIN;
	GPIOC_ADC_init.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5;
	GPIO_Init(GPIOC, &GPIOC_ADC_init);
}

void _GPIO_TIM3Ch3Ch2Ch1Init(void) {
	/* TIM3 Channel 3 : GPIOB Pin 0(Alternative Function)
	 * TIM3 Channel 2 : GPIOA Pin 7(Alternative Function)
	 * TIM3 Channel 1 : GPIOA Pin 6(Alternative Function) */
	GPIO_InitTypeDef GPIOAB_TIM3Ch3Ch2Ch1_init;

	GPIOAB_TIM3Ch3Ch2Ch1_init.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIOAB_TIM3Ch3Ch2Ch1_init.GPIO_Speed = GPIO_Speed_50MHz;

	/* 와이퍼 각도 제어 : TIM3_Channel3(PB0) GPIO 초기화 */
	GPIOAB_TIM3Ch3Ch2Ch1_init.GPIO_Pin = GPIO_Pin_0;
	GPIO_Init(GPIOB, &GPIOAB_TIM3Ch3Ch2Ch1_init);

	/* 와이퍼 각도 제어 : TIM3_Channel3(PB0) GPIO 초기화 */
	GPIOAB_TIM3Ch3Ch2Ch1_init.GPIO_Pin = GPIO_Pin_7;
	GPIO_Init(GPIOA, &GPIOAB_TIM3Ch3Ch2Ch1_init);

	/* DC모터 속도 제어 : TIM3_Channel1(PA6) GPIO 초기화 */
	GPIOAB_TIM3Ch3Ch2Ch1_init.GPIO_Pin = GPIO_Pin_6;
	GPIO_Init(GPIOA, &GPIOAB_TIM3Ch3Ch2Ch1_init);
}

void _GPIO_UltraDistance(void) {
	GPIO_InitTypeDef GPIOE_UltraDistance_init;

	GPIOE_UltraDistance_init.GPIO_Pin = GPIO_Pin_6;
	GPIOE_UltraDistance_init.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIOE_UltraDistance_init.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOE, &GPIOE_UltraDistance_init);
}

void _GPIO_Button(void) {
	/* 지문인식 센서의 동작 기능을 위한 Button1(PD11), Button2(PD12) 인풋 모드 설정 */
	GPIO_InitTypeDef GPIOD_Button_init;

	GPIOD_Button_init.GPIO_Pin = GPIO_Pin_11 | GPIO_Pin_12;
	GPIOD_Button_init.GPIO_Mode = GPIO_Mode_IPU;
	GPIOD_Button_init.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOD, &GPIOD_Button_init);
}

void _GPIO_LedControl(void) {
	/* 주위 밝기(밤, 낮)에 따른 라이트 제어를 위한 LED(PC10, PC11) 아웃풋 모드 설정 */
	GPIO_InitTypeDef GPIOE_TempContol_init;

	GPIOE_TempContol_init.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
	GPIOE_TempContol_init.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIOE_TempContol_init.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIOE_TempContol_init);

	GPIO_ResetBits(GPIOC, GPIO_Pin_10);
	GPIO_ResetBits(GPIOC, GPIO_Pin_11);
}

void _GPIO_TempControl(void) {
	/* 실내온도 조절을 위한 에어컨/히터(PC14, PC15) 아웃풋 모드 설정 */
	GPIO_InitTypeDef GPIOE_TempContol_init;

	GPIOE_TempContol_init.GPIO_Pin = GPIO_Pin_14 | GPIO_Pin_15;
	GPIOE_TempContol_init.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIOE_TempContol_init.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIOE_TempContol_init);

	GPIO_ResetBits(GPIOC, GPIO_Pin_14);
	GPIO_ResetBits(GPIOC, GPIO_Pin_15);
}

void GPIO_Configure(void) {
	_GPIO_USARTInit();
	_GPIO_LEDInit();
	_GPIO_MOTORInit();
	_GPIO_ADCInit();
	_GPIO_TIM3Ch3Ch2Ch1Init();
	_GPIO_UltraDistance();
	_GPIO_Button();
	_GPIO_LedControl();
	_GPIO_TempControl();
}

void ADC_Configure(void){
	/* ADC Channel 11(PC1), 12(PC2), 13(PC3) 사용 */
	ADC_InitTypeDef ADC_InitStructure;
	ADC_DeInit(ADC1);
	ADC_InitStructure.ADC_Mode=ADC_Mode_Independent;
	ADC_InitStructure.ADC_ScanConvMode=ENABLE;
	ADC_InitStructure.ADC_ContinuousConvMode=ENABLE;
	ADC_InitStructure.ADC_ExternalTrigConv=ADC_ExternalTrigConv_None;
	ADC_InitStructure.ADC_DataAlign=ADC_DataAlign_Right;
	ADC_InitStructure.ADC_NbrOfChannel=5;
	ADC_RegularChannelConfig(ADC1,ADC_Channel_11,1,ADC_SampleTime_239Cycles5);
	ADC_RegularChannelConfig(ADC1,ADC_Channel_12,2,ADC_SampleTime_239Cycles5);
	ADC_RegularChannelConfig(ADC1,ADC_Channel_13,3,ADC_SampleTime_239Cycles5);
	ADC_RegularChannelConfig(ADC1,ADC_Channel_14,4,ADC_SampleTime_239Cycles5);
	ADC_RegularChannelConfig(ADC1,ADC_Channel_15,5,ADC_SampleTime_239Cycles5);
	ADC_Init(ADC1,&ADC_InitStructure);

	ADC_DMACmd(ADC1,ENABLE);
	ADC_Cmd(ADC1,ENABLE);
}

void ADC_Initialize(void){
	/* ADC Calibration 초기화 */
	ADC_ResetCalibration(ADC1);
	while(ADC_GetResetCalibrationStatus(ADC1));
	ADC_StartCalibration(ADC1);
	while(ADC_GetCalibrationStatus(ADC1));

	/* ADC Convert 시작 */
	ADC_SoftwareStartConvCmd(ADC1,ENABLE);
}

void _TIM3Ch3Ch2Ch1_PWM_Configure(void) {
	/* 기본 System Clock : 72MHz
	 * Prescale = 72MHz / 1MHz - 1 = 71 => 72MHz를 1MHz Timer clock으로 세팅
	 * TIM_Period = 1MHz / 50Hz = 20,000 => 1MHz Timer Clock을 20,000주기를 통해 50Hz 주기로 세팅 */
	TIM_TimeBaseInitTypeDef TIM3Ch3Ch2Ch1_Init;

	TIM_OCInitTypeDef PWM_TIM3Ch3Ch2Ch1_Init;

	TIM3Ch3Ch2Ch1_Init.TIM_Prescaler = (uint16_t)(SystemCoreClock / 1000000) - 1;
	TIM3Ch3Ch2Ch1_Init.TIM_Period = 20000 - 1;
	TIM3Ch3Ch2Ch1_Init.TIM_ClockDivision = 0;
	TIM3Ch3Ch2Ch1_Init.TIM_CounterMode = TIM_CounterMode_Down;
	TIM_TimeBaseInit(TIM3, &TIM3Ch3Ch2Ch1_Init);

	PWM_TIM3Ch3Ch2Ch1_Init.TIM_OCMode = TIM_OCMode_PWM1;
	PWM_TIM3Ch3Ch2Ch1_Init.TIM_OCPolarity = TIM_OCPolarity_High;
	PWM_TIM3Ch3Ch2Ch1_Init.TIM_OutputState = TIM_OutputState_Enable;

	/* 자동차 와이퍼 */
	/* Servo Moter 0도 : 7.5% Duty Cycle을 초기 값으로 PWM mode 설정 */
	/* 서보 모터 각도 제어 : 초기 duty cycle을 7.5%로 초기화 */
	PWM_TIM3Ch3Ch2Ch1_Init.TIM_Pulse = 1500;
	TIM_OC3Init(TIM3, &PWM_TIM3Ch3Ch2Ch1_Init);
	TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Disable);

	/* Door Control : Open / Close */
	/* Servo Moter 0도 : 7.5% Duty Cycle을 초기 값으로 PWM mode 설정 */
	/* 서보 모터 각도 제어 : 초기 duty cycle을 7.5%로 초기화 */
	PWM_TIM3Ch3Ch2Ch1_Init.TIM_Pulse = 1500;
	TIM_OC2Init(TIM3, &PWM_TIM3Ch3Ch2Ch1_Init);
	TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Disable);

	/* 속도 */
	/* DC 모터 속도 제어 : 초기 duty cycle을 50%로 초기화 */
	PWM_TIM3Ch3Ch2Ch1_Init.TIM_Pulse = 10000;
	TIM_OC1Init(TIM3, &PWM_TIM3Ch3Ch2Ch1_Init);
	TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Disable);

	TIM_ARRPreloadConfig(TIM3, ENABLE);
	TIM_Cmd(TIM3, ENABLE);
}

void _TIM2_Configure(void) {
	/* 기본 System Clock : 72MHz
	 * Prescale = 72MHz / 100KHz - 1 = 719 => 72MHz를 100KHz Timer clock으로 세팅
	 * TIM_Period = 100KHz / 2Hz = 50,000 => 100KHz Timer clock을 50,000주기를 통해 2Hz(0.5sec) 주기로 세팅 */
	TIM_TimeBaseInitTypeDef TIM2_Init;

	TIM2_Init.TIM_Prescaler = (uint16_t)(SystemCoreClock / 100000) - 1;
	TIM2_Init.TIM_Period = 50000 - 1;
	TIM2_Init.TIM_ClockDivision = 0;
	TIM2_Init.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM2, &TIM2_Init);

	/* TIM2 Interrupt Enable */
	TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
	TIM_Cmd(TIM2, ENABLE);
}

void _TIM1_Configure(void) {
	/* 기본 System Clock : 72MHz
	 * Prescale = 72MHz / 100KHz - 1 = 719 => 72MHz를 100KHz Timer clock으로 세팅
	 * TIM_Period = 100KHz / 1KHz - 1 = 99 => 100KHz Timer clock을 100주기를 통해 1KHz(1msec) 주기로 세팅 */
	TIM_TimeBaseInitTypeDef TIM1_Init;

	TIM1_Init.TIM_Prescaler = (uint16_t)(SystemCoreClock / 100000) - 1;
	TIM1_Init.TIM_Period = 100 - 1;
	TIM1_Init.TIM_ClockDivision = 0;
	TIM1_Init.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM1, &TIM1_Init);

	/* TIM2 Interrupt Enable */
	TIM_ITConfig(TIM1, TIM_IT_Update, ENABLE);
	TIM_Cmd(TIM1, ENABLE);
}

void _TIM6_Configure(void) {
	/* 프리스케일러가 마이크로 초에 한번 트리거하도록 설정 */
	TIM6->PSC = 24 - 1;
	/* 응답 제한은 50msec = 50,000마이크로 초 */
	TIM6->ARR = 50000;
}

void _TIM7_Configure(void) {
	/* 프리스케일러가 마이크로 초에 한번 트리거하도록 설정 */
	TIM7->PSC = 24 - 1;
	/* 동작한계는 10마이크로 초 */
	TIM7->ARR = 10;
}

void TIM_PWM_Configure(void){
	_TIM3Ch3Ch2Ch1_PWM_Configure();
	_TIM2_Configure();
	_TIM1_Configure();
	_TIM6_Configure();
	_TIM7_Configure();
}

void DMA_Configure(void){
	/* DMA1 Channel 11번 Configure*/
	DMA_InitTypeDef DMA_init;

	DMA_DeInit(DMA1_Channel1);
	DMA_init.DMA_PeripheralBaseAddr=(uint32_t)&ADC1->DR;
	DMA_init.DMA_MemoryBaseAddr=(uint32_t)ADC_result_value_arr;
	DMA_init.DMA_DIR=DMA_DIR_PeripheralSRC;
	DMA_init.DMA_BufferSize=5;
	DMA_init.DMA_PeripheralInc=DMA_PeripheralInc_Disable;
	DMA_init.DMA_MemoryInc=DMA_MemoryInc_Enable;
	DMA_init.DMA_PeripheralDataSize=DMA_PeripheralDataSize_Word;
	DMA_init.DMA_MemoryDataSize=DMA_MemoryDataSize_Word;
	DMA_init.DMA_Mode=DMA_Mode_Circular;
	DMA_init.DMA_Priority=DMA_Priority_High;
	DMA_init.DMA_M2M=DMA_M2M_Disable;

	DMA_Init(DMA1_Channel1,&DMA_init);

	DMA_Cmd(DMA1_Channel1,ENABLE);
}

void _USART_ITR_Configure(void) {
	NVIC_InitTypeDef NVIC_init;

	/* USART를 위한 NVIC 기본 값 세팅(우선순위 0) */
	NVIC_init.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_init.NVIC_IRQChannelSubPriority = 0;
	NVIC_init.NVIC_IRQChannelCmd = ENABLE;

	/* Enable the USART1 NVIC_init */
	NVIC_init.NVIC_IRQChannel = USART1_IRQn;
	NVIC_Init(&NVIC_init);

	/* Enable the USART2 NVIC_init */
	NVIC_init.NVIC_IRQChannel = USART2_IRQn;
	NVIC_Init(&NVIC_init);

	/* Enable the USART3 NVIC_init */
	NVIC_init.NVIC_IRQChannel = USART3_IRQn;
	NVIC_Init(&NVIC_init);
}

void _TIM2_ITR_Configure(void) {
	NVIC_InitTypeDef NVIC_init;
	NVIC_init.NVIC_IRQChannel = TIM2_IRQn;
	NVIC_init.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_init.NVIC_IRQChannelSubPriority = 0;
	NVIC_init.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_init);
}

void _TIM6_ITR_Configure(void) {
	/* 해결방법 TIM6_IRQn 인터럽트 주기를 계산하는 데 필요. */
	NVIC_SetPriority(TIM6_IRQn, 3);
	NVIC_EnableIRQ(TIM6_IRQn);
}

void _TIM7_ITR_Configure(void) {
	/* 해상도 TIM7_IRQn 인터럽트 신호 펄스를 계산하는 데 필요함. */
	NVIC_SetPriority(TIM7_IRQn, 2);
	NVIC_EnableIRQ(TIM7_IRQn);
}

void _EXTI_Ultra_ITR_Configure(void) {
	/* 제로 레그 중단이 허용됨. */
	EXTI->IMR |= EXTI_IMR_MR0;
	/* 상승 앞면에 따른 중단 */
	EXTI->RTSR |= EXTI_RTSR_TR0;

	NVIC_SetPriority(EXTI0_IRQn, 1);
	NVIC_EnableIRQ(EXTI0_IRQn);
}

void Interrupt_Configure(void) {
	_USART_ITR_Configure();
	_TIM2_ITR_Configure();
	_TIM6_ITR_Configure();
	_TIM7_ITR_Configure();
	_EXTI_Ultra_ITR_Configure();
}

/* Todo : 이게 필요할지 잘 모르겠다. 나중에 필요없으면 삭제할 것 */
void SendString(USART_TypeDef* USARTx, char* string) {
	int i=0;
	for(i=0; i<strlen(string); i++) {
		USART_SendData(USARTx, string[i]);
		while (USART_GetFlagStatus(USARTx, USART_FLAG_TXE) == RESET);
	}
}

void _command_forward(void) {
	/* 모터드라이버1 제어 */
	/* 왼쪽 바퀴 : in1(High)/in2(Low) */
	GPIO_SetBits(GPIOB, GPIO_Pin_5);
	GPIO_ResetBits(GPIOB, GPIO_Pin_8);
	/* 오른쪽 바퀴 : in3(High)/in4(Low) */
	GPIO_SetBits(GPIOC, GPIO_Pin_12);
	GPIO_ResetBits(GPIOC, GPIO_Pin_13);

	/* 모터드라이버2 제어 */
	/* 왼쪽 바퀴 : in1(High)/in2(Low) */
	GPIO_ResetBits(GPIOB, GPIO_Pin_1);
	GPIO_SetBits(GPIOB, GPIO_Pin_2);
	/* 오른쪽 바퀴 : in3(High)/in4(Low) */
	GPIO_ResetBits(GPIOC, GPIO_Pin_7);
	GPIO_SetBits(GPIOC, GPIO_Pin_8);
}

void _command_backward(void) {
	/* 모터드라이버1 제어 */
	/* 왼쪽 바퀴 : in1(Low)/in2(High) */
	GPIO_ResetBits(GPIOB, GPIO_Pin_5);
	GPIO_SetBits(GPIOB, GPIO_Pin_8);
	/* 오른쪽 바퀴 : in3(Low)/in4(High) */
	GPIO_ResetBits(GPIOC, GPIO_Pin_12);
	GPIO_SetBits(GPIOC, GPIO_Pin_13);

	/* 모터드라이버2 제어 */
	/* 왼쪽 바퀴 : in1(Low)/in2(High) */
	GPIO_SetBits(GPIOB, GPIO_Pin_1);
	GPIO_ResetBits(GPIOB, GPIO_Pin_2);
	/* 오른쪽 바퀴 : in3(Low)/in4(High) */
	GPIO_SetBits(GPIOC, GPIO_Pin_7);
	GPIO_ResetBits(GPIOC, GPIO_Pin_8);
}

void _command_left(void) {
	/* 모터드라이버1이 앞쪽바퀴 */

	/* 모터드라이버1 제어 */
	/* 왼쪽 바퀴 : in1(Low)/in2(High) => 역방향 */
	GPIO_ResetBits(GPIOB, GPIO_Pin_5);
	GPIO_SetBits(GPIOB, GPIO_Pin_8);
	/* 오른쪽 바퀴 : in3(High)/in4(Low) => 정방향 */
	GPIO_SetBits(GPIOC, GPIO_Pin_12);
	GPIO_ResetBits(GPIOC, GPIO_Pin_13);

	/* 모터드라이버2 제어 */
	/* 왼쪽 바퀴 : in1(Low)/in2(High) => 역방향 */
	GPIO_ResetBits(GPIOB, GPIO_Pin_1);
	GPIO_SetBits(GPIOB, GPIO_Pin_2);
	/* 오른쪽 바퀴 : in3(High)/in4(Low) => 정방향 */
	GPIO_SetBits(GPIOC, GPIO_Pin_7);
	GPIO_ResetBits(GPIOC, GPIO_Pin_8);
}

void _command_right(void) {
	/* 모터드라이버1이 앞쪽바퀴 */

	/* 모터드라이버1 제어 */
	/* 왼쪽 바퀴 : in1(High)/in2(Low) => 정방향 */
	GPIO_SetBits(GPIOB, GPIO_Pin_5);
	GPIO_ResetBits(GPIOB, GPIO_Pin_8);
	/* 오른쪽 바퀴 : in3(Low)/in4(High) => 역방향 */
	GPIO_ResetBits(GPIOC, GPIO_Pin_12);
	GPIO_SetBits(GPIOC, GPIO_Pin_13);

	/* 모터드라이버2 제어 */
	/* 왼쪽 바퀴 : in1(High)/in2(Low) => 정방향 */
	GPIO_SetBits(GPIOB, GPIO_Pin_1);
	GPIO_ResetBits(GPIOB, GPIO_Pin_2);
	/* 오른쪽 바퀴 : in3(Low)/in4(High) => 역방향 */
	GPIO_ResetBits(GPIOC, GPIO_Pin_7);
	GPIO_SetBits(GPIOC, GPIO_Pin_8);
}

void _command_stop(void) {
	/* 모터드라이버1 제어 */
	/* 왼쪽 바퀴 : in1(Low)/in2(Low) => 정지
	 * 오른쪽 바퀴 : in3(Low)/in4(Low) => 정지 */
	GPIO_ResetBits(GPIOB, GPIO_Pin_5 | GPIO_Pin_8);
	GPIO_ResetBits(GPIOC, GPIO_Pin_12 | GPIO_Pin_13);

	/* 모터드라이버2 제어 */
	/* 왼쪽 바퀴 : in1(Low)/in2(Low) => 정지
	 * 오른쪽 바퀴 : in3(Low)/in4(Low) => 정지 */
	GPIO_ResetBits(GPIOB, GPIO_Pin_1 | GPIO_Pin_2);
	GPIO_ResetBits(GPIOC, GPIO_Pin_7 | GPIO_Pin_8);
}

void command_move(int select) {
	/* Motor Driver Function
	 *    1) 정방향 : in1(in3) - High / in2(in4) - Low
	 *    2) 역방향 : in1(in3) - Low / in2(in4) - High
	 *    3) 정지 : in1(in3) - High / in2(in4) - High
	 *    4) 정지 : in1(in3) - Low / in2(in4) - Low */
	switch(select) {
	case 0:
		_command_stop();
		break;
	case 1:
		_command_forward();
		break;
	case 2:
		_command_backward();
		break;
	case 3:
		_command_left();
		break;
	case 4:
		_command_right();
		break;
	default:
		_command_stop();
	}
}

void change_pwm_servo_duty_cycle(int percentx10) {
	/* ## PWM 파형 주기 : 50Hz
	 * ## Duty Cycle
	 *    1) 3.5% : -90도
	 *    2) 7.5% : 0도
	 *    3) 11.5% : 90도
	 * ## PWM Duty Cycle 계산 => TIM_Pulse = percent * 20,000 / 100 */
	TIM_OCInitTypeDef PWM_TIM3Ch3_Init;
	int pwm_pulse;
	pwm_pulse = percentx10 * 20000 / 100 / 10;
	PWM_TIM3Ch3_Init.TIM_OCMode = TIM_OCMode_PWM1;
	PWM_TIM3Ch3_Init.TIM_OCPolarity = TIM_OCPolarity_High;
	PWM_TIM3Ch3_Init.TIM_OutputState = TIM_OutputState_Enable;
	PWM_TIM3Ch3_Init.TIM_Pulse = pwm_pulse;
	TIM_OC3Init(TIM3, &PWM_TIM3Ch3_Init);
}

void door_control(bool flag) {
	/* ## PWM 파형 주기 : 50Hz
	 * ## Duty Cycle
	 *    1) 3.5% : -90도
	 *    2) 7.5% : 0도
	 *    3) 11.5% : 90도
	 * ## PWM Duty Cycle 계산 => TIM_Pulse = percent * 20,000 / 100
	 *
	 * ## flag : open(true), close(false) */
	TIM_OCInitTypeDef PWM_TIM3Ch3_Init;
	int pwm_pulse;

	PWM_TIM3Ch3_Init.TIM_OCMode = TIM_OCMode_PWM1;
	PWM_TIM3Ch3_Init.TIM_OCPolarity = TIM_OCPolarity_High;
	PWM_TIM3Ch3_Init.TIM_OutputState = TIM_OutputState_Enable;
	if(flag) {
		pwm_pulse = 115 * 20000 / 100 / 10;
	}else {
		pwm_pulse = 75 * 20000 / 100 / 10;
	}
	PWM_TIM3Ch3_Init.TIM_Pulse = pwm_pulse;
	TIM_OC2Init(TIM3, &PWM_TIM3Ch3_Init);
}

void speed_up_down(int percentx10) {
	/* ## 속도 PWM 파형 주기 : 15KHz
	 * PWM Duty Cycle 계산 => TIM_Pulse = percent * 200 / 100 */
	TIM_OCInitTypeDef PWM_TIM3Ch1_Init;
	int pwm_pulse;
	pwm_pulse = percentx10 * 20000 / 100 / 10;
	PWM_TIM3Ch1_Init.TIM_OCMode = TIM_OCMode_PWM1;
	PWM_TIM3Ch1_Init.TIM_OCPolarity = TIM_OCPolarity_High;
	PWM_TIM3Ch1_Init.TIM_OutputState = TIM_OutputState_Enable;
	PWM_TIM3Ch1_Init.TIM_Pulse = pwm_pulse;
	TIM_OC1Init(TIM3, &PWM_TIM3Ch1_Init);
}

void AccelGyroInitialize(void) {
	set_last_read_angle_data(GetTickCount(), 0, 0, 0, 0, 0, 0);
}

void calculate_accelgyro_average(int total_readnum) {
	int i=0;
	for(i=0; i<total_readnum; i++) {
		MPU6050_GetRawAccelGyro(AccelGyro);
		base_accel_x += AccelGyro[0];
		base_accel_y += AccelGyro[1];
		base_accel_z += AccelGyro[2];
		base_gyro_x += AccelGyro[3];
		base_gyro_y += AccelGyro[4];
		base_gyro_z += AccelGyro[5];
	}
	base_accel_x /= total_readnum;
	base_accel_y /= total_readnum;
	base_accel_z /= total_readnum;
	base_gyro_x /= total_readnum;
	base_gyro_y /= total_readnum;
	base_gyro_z /= total_readnum;
}

void set_last_read_angle_data(unsigned long time, float x, float y, float z, float gyro_x, float gyro_y, float gyro_z) {
	/* 가속도 센서 값을 활용해 처리한 최종 각도 값을 전역 변수에 저장 */
	last_read_time = time;
	last_angle_x = x;
	last_angle_y = y;
	last_angle_z = z;
	last_gyro_angle_x = gyro_x;
	last_gyro_angle_y = gyro_y;
	last_gyro_angle_z = gyro_z;
}

void PrintAccelGryroRaw2Angle(int16_t accelgyro[6]) {
	/* 250 degree/s 범위를 가지고, 이 값을 16bit 분해능으로 표현하고 있으므로,
	 * mpu-6050에서 보내는 값은 -32766 ~ +32766 값이다.
	 * 그러므로 이 값을 실제 250 degree/s로 변환하려면 131로 나눠줘야 한다.
	 * 범위가 다르면 이 값도 바뀌어야 한다. */

	float FS_SEL = 131;

	/* 회전을 했을 떄 시간 알기 */
	unsigned long t_now = GetTickCount();

	float gyro_x = (accelgyro[3] - base_gyro_x)/FS_SEL;
	float gyro_y = (accelgyro[4] - base_gyro_y)/FS_SEL;
	float gyro_z = (accelgyro[5] - base_gyro_z)/FS_SEL;


	/* 가속도 값 범위는 16bit이기 때문에 -32766 ~ + 32766 범위이고,
	 * +-2g 범위이면 mpu-6050으로부터 넘어온 값을 실제 g단위로 환산하려면
	 * scale factor(16384)로 나눠야 한다. +-2g 범위는 +-32766값이다. 즉 32766갑이면 2가 된다. */

	//acceleration 원시 데이터 저장
	float accel_x = accelgyro[0];
	float accel_y = accelgyro[1];
	float accel_z = accelgyro[2];


	//accelerometer로 부터 각도 얻기
	float RADIANS_TO_DEGREES = 180/3.14159;

	// float accel_vector_length = sqrt(pow(accel_x,2) + pow(accel_y,2) + pow(accel_z,2));
	float accel_angle_y = atan(-1*accel_x/sqrt(pow(accel_y,2) + pow(accel_z,2)))*RADIANS_TO_DEGREES;
	float accel_angle_x = atan(accel_y/sqrt(pow(accel_x,2) + pow(accel_z,2)))*RADIANS_TO_DEGREES;
	float accel_angle_z = 0;


	//gyro angles 계산1
	float dt =(t_now - last_read_time)/1000.0;
	float gyro_angle_x = gyro_x*dt + last_angle_x;
	float gyro_angle_y = gyro_y*dt + last_angle_y;
	float gyro_angle_z = gyro_z*dt + last_angle_z;

	//gyro angles 계산2
	float unfiltered_gyro_angle_x = gyro_x*dt + last_gyro_angle_x;
	float unfiltered_gyro_angle_y = gyro_y*dt + last_gyro_angle_y;
	float unfiltered_gyro_angle_z = gyro_z*dt + last_gyro_angle_z;

	//알파를 이용해서 최종 각도 계산3
	float alpha = 0.96;
	float angle_x = alpha*gyro_angle_x + (1.0 - alpha)*accel_angle_x;
	float angle_y = alpha*gyro_angle_y + (1.0 - alpha)*accel_angle_y;
	float angle_z = gyro_angle_z;  //Accelerometer는 z-angle 없음

	//최종 각도 저장
	set_last_read_angle_data(t_now, angle_x, angle_y, angle_z, unfiltered_gyro_angle_x, unfiltered_gyro_angle_y, unfiltered_gyro_angle_z);

	// 2 바이트 정수로 보내기 위해 100을 곱하고, 받을 때, 100을 나눠준다.
	//LCD_ShowNum(20,20, (int)accel_angle_x*100,10,BLACK,WHITE);
	//LCD_ShowNum(20,40, (int)accel_angle_y*100,10,BLACK,WHITE);
	//LCD_ShowNum(20,60,(int)accel_angle_z*100,10,BLACK,WHITE);
	//LCD_ShowNum(20,80,(int)unfiltered_gyro_angle_x*100,10,BLACK,WHITE);
	//LCD_ShowNum(20,100,(int)unfiltered_gyro_angle_y*100,10,BLACK,WHITE);
	//LCD_ShowNum(20,120,(int)unfiltered_gyro_angle_z*100,10,BLACK,WHITE);

	if(angle_x > 0) {
		LCD_ShowString(200, 205, "+", BLACK, WHITE);
		LCD_ShowNum(210,205,(int)angle_x,2,BLACK,WHITE);		// 실질적으로 필요한 경사 각도
	}else {
		LCD_ShowString(200, 205, "-", BLACK, WHITE);
		LCD_ShowNum(210,205,(int)(-1)*angle_x,2,BLACK,WHITE);		// 실질적으로 필요한 경사 각도
	}

	//LCD_ShowNum(20,160,(int)angle_y,10,BLACK,WHITE);
	//LCD_ShowNum(20,180,(int)angle_z,10,BLACK,WHITE);
}

uint32_t GetTickCount(void) {
	return g_currentTick;
}

void UIOutline_Init(void) {
	LCD_Clear(WHITE);
	LCD_ShowString(40, 10, "## ProtoSmartCar ##", BLACK, WHITE);
	LCD_DrawRectangle(5, 30, 235, 170);
	LCD_ShowString(20, 40, "[+] Temperature : ", BLACK, WHITE);
	LCD_ShowString(20, 70, "[+] Humidity    : ", BLACK, WHITE);
	LCD_ShowString(20, 100, "[+] Velocity    : ", BLACK, WHITE);

	LCD_ShowString(25, 180, "- Accident Log -", BLACK, WHITE);
	LCD_DrawRectangle(5, 200, 175, 315);
	LCD_ShowString(185, 180, "Sensor", BLACK, WHITE);
	LCD_DrawRectangle(180, 200, 235, 315);
}

void Delay_ms(uint32_t ms) {
	volatile uint32_t nCount;
	RCC_ClocksTypeDef RCC_Clocks;
	RCC_GetClocksFreq (&RCC_Clocks);

	nCount = (RCC_Clocks.HCLK_Frequency/10000)*ms;
	for(; nCount!=0; nCount--);
}

void UltraDistance_Init(void) {
	/* 처음으로 타이머7을 시작하여 10ms를 카운트함. */
	TIM7->DIER |= TIM_DIER_UIE;		// 타이머 7에서 인터럽트 사용
	GPIOE->ODR |= GPIO_Pin_6;		// 신호 충동 켜기
	TIM7->CR1 |= TIM_CR1_CEN;		// 타이머 시작
}

uint16_t getDistance(void) {
	Delay_ms(100);
	return duration;
}

/* Todo : 개발자 디버그용이므로 나중에 지울 것 */
//void USART1_IRQHandler(void) {
//	char recv_data;
//
//	/* USART1에서 정보를 받아 출력한다.(블루투스 세팅 용도 & 안드로이드 어플로 송신) */
//	if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET) {
//		recv_data = USART_ReceiveData(USART1);
//		gRsBuf[gRsLength++] = recv_data;
//		LCD_ShowNum(50, 130, gRsLength, 10, BLACK, WHITE);
//		if(recv_data) {
//			//GPIO_SetBits(GPIOD, GPIO_Pin_2);
//			//USART_SendData(USART1, recv_data+0x30);									// wjdebug
//			//while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);				// wjdebug
//			LCD_ShowNum(50, 150, gRsLength, 10, BLACK, WHITE);
//			USART_SendData(USART2, recv_data);
//			while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);
//			USART_ClearITPendingBit(USART1, USART_IT_RXNE);
//		}
//	}
//}

//void USART1_IRQHandler(void) {
//	char recv_data;
//
//	/* 안드로이드 어플리케이션에서 보내는 문자열 명령을 받아서 명령어별로 처리하는 인터럽트 */
//	/* Command End : "\n"
//	   Command List : "FORWARD", "BACKWARD", "LEFT", "RIGHT", "STOP", "ON", "OFF"
//	   Optional Command List : "LIGHT_ON", "LIGHT_OFF", "TEMPER_UP", "TEMPER_DOWN", "WIPER_ON", "WIPER_OFF"
//	*/
//	if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET) {
//		recv_data = USART_ReceiveData(USART1);
//
//		if(recv_data == '!') {
//			if(strstr(command, "CLEAR") != NULL) {
//				GPIO_ResetBits(GPIOD, GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_7);
//			}else if(strstr(command, "POWERON") != NULL) {
//				//GPIO_SetBits(GPIOD, GPIO_Pin_2);
//				//speed_up_down(600);
//				//UIOutline_Init();
//			}else if(strstr(command, "POWEROFF") != NULL) {
//				//GPIO_ResetBits(GPIOD, GPIO_Pin_2);
//				//speed_up_down(400);
//				//LCD_Clear(BLACK);
//			}else if(strstr(command, "FORWARD") != NULL) {
//				GPIO_SetBits(GPIOD, GPIO_Pin_3);
//				//speed_up_down(500);
//				command_move(1);
//			}else if(strstr(command, "BACKWARD") != NULL) {
//				GPIO_SetBits(GPIOD, GPIO_Pin_2);
//				//GPIO_SetBits(GPIOD, GPIO_Pin_4);
//				command_move(2);
//			}else if(strstr(command, "LEFT") != NULL) {
//				//GPIO_SetBits(GPIOD, GPIO_Pin_4);
//				command_move(3);
//			}else if(strstr(command, "RIGHT") != NULL) {
//				//GPIO_SetBits(GPIOD, GPIO_Pin_7);
//				command_move(4);
//			}else if(strstr(command, "STOP") != NULL) {
//				GPIO_ResetBits(GPIOD, GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_7);
//				command_move(0);
//			}
//			command_pos = 0;
//			memset(command, 0x00, 100);
//		}else {
//			command[command_pos++] = recv_data;
//			USART_SendData(USART1, recv_data);								// wjdebug
//			while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);		// wjdebug
//		}
//		USART_ClearITPendingBit(USART1, USART_IT_RXNE);
//	}
//}

void USART2_IRQHandler(void) {
	char recv_data;

	/* 안드로이드 어플리케이션에서 보내는 문자열 명령을 받아서 명령어별로 처리하는 인터럽트 */
	/* Command End : "\n"
	   Command List : "FORWARD", "BACKWARD", "LEFT", "RIGHT", "STOP", "ON", "OFF"
	   Optional Command List : "LIGHT_ON", "LIGHT_OFF", "TEMPER_UP", "TEMPER_DOWN", "WIPER_ON", "WIPER_OFF"
	*/
	if (USART_GetITStatus(USART2, USART_IT_RXNE) != RESET) {
		recv_data = USART_ReceiveData(USART2);

		GPIO_ResetBits(GPIOD, GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_7);

		if(recv_data == '!') {
			if(strstr(command, "CLEAR") != NULL) {
				GPIO_ResetBits(GPIOD, GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_7);
			}else if(strstr(command, "POWERON") != NULL) {
				UIOutline_Init();
			}else if(strstr(command, "POWEROFF") != NULL) {
				LCD_Clear(BLACK);
			}else if(strstr(command, "FORWARD") != NULL) {
				stop_flag = false;
				LCD_ShowString(180,100, "40 m/s", BLACK, WHITE);
				command_move(1);
			}else if(strstr(command, "BACKWARD") != NULL) {
				LCD_ShowString(180,100, "40 m/s", BLACK, WHITE);
				command_move(2);
			}else if(strstr(command, "LEFT") != NULL) {
				command_move(3);
			}else if(strstr(command, "RIGHT") != NULL) {
				command_move(4);
			}else if(strstr(command, "STOP") != NULL) {
				stop_flag = true;
				LCD_ShowString(180,100, "0 m/s", BLACK, WHITE);
				GPIO_ResetBits(GPIOD, GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_7);
				command_move(0);
			}
			command_pos = 0;
			memset(command, 0x00, 100);
		}else {
			command[command_pos++] = recv_data;
			//USART_SendData(USART1, recv_data);								// wjdebug
			//while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);		// wjdebug
		}
		USART_ClearITPendingBit(USART2, USART_IT_RXNE);
	}
}

void USART3_IRQHandler(void) {
	char voice_command;

	/* USART3에서 정보를 받아 출력한다.(음성인식센서에서 값을 받아 PC로 전송)
	 *    1) voice_command : 음성인식센서가 명령어를 인식하면 리턴 값을 반환
	 *    2) 리턴 값에 따라 voiceBuffer에 저장된 명령어를 인식 */
	if (USART_GetITStatus(USART3, USART_IT_RXNE) != RESET) {
		voice_command = USART_ReceiveData(USART3);

		if(voice_command) {
			if(voice_command_enable) {
				UIOutline_Init();
				LCD_ShowString(50, 130, voiceBuffer[voice_command-1], BLACK, WHITE);
			}else {
				UIOutline_Init();
				LCD_ShowString(50, 130, "Voice Command Disabled!", BLACK, WHITE);
			}
			USART_ClearITPendingBit(USART3, USART_IT_RXNE);
		}
	}
}

void TIM2_IRQHandler(void) {
	if(TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET) {
		/* ## rain_power_flag : 빗물의 세기
		 *    1) 0 : 비 안옴
		 *    2) 1 : 비 약함
		 *    3) 2 : 비 강함
		 * ## servo_direction : 서보모터 방향 조절 플래그 */

		/* 비의 세기가 약할 때 : 1초 간격으로 동작 */
		if(rain_power_flag == 1) {
			if(timer_count % 2 == 0) {
				if(servo_direction == 1) {
					change_pwm_servo_duty_cycle(35);
					servo_direction = 0;
				}
				else {
					change_pwm_servo_duty_cycle(115);
					servo_direction = 1;
				}
				timer_count = timer_count % 50;
			}
			timer_count++;

		/* 비의 세기가 강할 때 : 0.5초 간격으로 동작 */
		}else if(rain_power_flag == 2) {
			if(servo_direction == 1) {
				change_pwm_servo_duty_cycle(35);
				servo_direction = 0;
			}
			else {
				change_pwm_servo_duty_cycle(115);
				servo_direction = 1;
			}
		}

		if(speed_updown_timer_count % 2 == 0) {
			if((int)last_angle_x >= 40) {
				LCD_ShowString(180,100, "56 m/s", BLACK, WHITE);
				speed_up_down(700);
			}else if((int)last_angle_x >= 20) {
				LCD_ShowString(180,100, "48 m/s", BLACK, WHITE);
				speed_up_down(600);
			}else if((int)last_angle_x <= (-1)*40) {
				LCD_ShowString(180,100, "24 m/s", BLACK, WHITE);
				speed_up_down(300);
			}else if((int)last_angle_x <= (-1)*20) {
				LCD_ShowString(180,100, "32 m/s", BLACK, WHITE);
				speed_up_down(400);
			}else {
				if(stop_flag)
					LCD_ShowString(180,100, "0 m/s", BLACK, WHITE);
				else
					LCD_ShowString(180,100, "40 m/s", BLACK, WHITE);
				speed_up_down(500);
			}
			speed_updown_timer_count = speed_updown_timer_count % 2;
		}
		speed_updown_timer_count++;

		TIM_ClearITPendingBit(TIM2, TIM_IT_Update);		// clear interrupt flag
	}
}

void TIM1_IRQHandler(void) {
	if(TIM_GetITStatus(TIM1, TIM_IT_Update) != RESET) {
		/* TickCount 값을 1씩 증가시키면서 g_currentTick에 저장 */
		g_currentTick++;
		TIM_ClearITPendingBit(TIM1, TIM_IT_Update);		// clear interrupt flag
	}
}

// 인터럽트 처리기 TIM6_DAC
// 타이머 6이 사이클주기 동안 50 μs를 계산 한 후에 호출됨
void TIM6_IRQHandler(void)
{
	TIM6->SR &= ~TIM_SR_UIF;             // UIF 플래그 재설정
	GPIOE->ODR |= GPIO_Pin_6;            // 신호 충동 켜기
	// 10ms 카운트 다운시 타이머 7이 시작됨
	TIM7->DIER |= TIM_DIER_UIE;          // 타이머 7에서 인터럽트 사용
	TIM7->CR1 |= TIM_CR1_CEN;            // 타이머 시작
}

// 인터럽트 처리기 TIM7_DAC
// 타이머 7이 신호 펄스에 대해 10 μs를 계산 한 후 호출됨
void TIM7_IRQHandler(void)
{
	TIM7->SR &= ~TIM_SR_UIF;             // UIF 플래그 재설정
	GPIOE->ODR &= ~GPIO_Pin_6;           // 신호 충동을 멈춤
	TIM7->DIER &= ~TIM_DIER_UIE;         // 타이머 7의 인터럽트 금지
}

// 인터럽트 핸들러 EXTI0 : 신호 레벨 변경
void EXTI0_IRQHandler(void)
{
	// 상승하는 정면을 따라 잡으면
	if (!catcher_status)
	{
		// 카운트 펄스 지속 시간을 시작
		TIM6->CR1 |= TIM_CR1_CEN;
		// 하강 에지 잡기로 전환
		catcher_status = 1;
		EXTI->RTSR &= ~EXTI_RTSR_TR0;
		EXTI->FTSR |= EXTI_FTSR_TR0;
	}
	// 포착 된 경우
	else
	{
		TIM6->CR1 &= ~TIM_CR1_CEN;         // 타이머 정지
		duration = TIM6->CNT;              // 기간을 μs 단위로 읽음
		TIM6->CNT = 0;                     // 카운터 레지스터 재설정
		// 증가하는 전선 잡기로 전환
		catcher_status = 0;
		EXTI->FTSR &= ~EXTI_FTSR_TR0;
		EXTI->RTSR |= EXTI_RTSR_TR0;
		// 50ms의 카운트 다운 시에 타이머 6을 시작
		TIM6->DIER |= TIM_DIER_UIE;        // 타이머에서 인터럽트 허용
		TIM6->CR1 |= TIM_CR1_CEN;          // 타이머 시작
	}
	EXTI->PR |= 0x01;                    // flag 지우기
}
