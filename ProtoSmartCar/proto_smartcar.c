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
#include <SHT15.h>

/* Globl Variables
 *    1) ADC_result_value_arr[2] : [0](조도센서), [1](빗물감지센서), [2](인체감지센서)
 *    2) command[100] : 명령어 버퍼
 *    3) command_pos : 명령어 위치 초기화 용도
 *    4) voiceBuffer : 보이스 명령어 문자열 셋
 *    5) rain_power_flag : 비의 세기
 *    6) servo_direction : 와이퍼를 주기적으로 왕복시키기 위한 방향 플래그
 *    7) timer_count : 비의 세기에 따라 와이퍼의 속도를 조절하기 위한 timer count
 *    8) voice_command_enable : 음성 명령 모드 활성화 플래그 */
__IO uint32_t ADC_result_value_arr[3];
char command[100];
int command_pos=0;

u8 *voiceBuffer[] =
{
    "Turn on the light",
    "Turn off the light",
    "Play music",
    "Pause",
    "Next",
    "Previous",
    "Up",
    "Down",
    "Turn on the TV",
    "Turn off the TV",
    "Increase temperature",
    "Decrease temperature",
    "What's the time",
    "Open the door",
    "Close the door",
    "Left",
    "Right",
    "Stop",
    "Start",
    "Mode 1",
    "Mode 2",
    "Go",
};

int rain_power_flag = 0;
int servo_direction = 0;
int timer_count = 0;
int voice_command_enable = 0;


void RCC_Configure(void);
void USART_Configure(void);
void _GPIO_LEDInit(void);
void _GPIO_MOTOR1Init(void);
void _GPIO_MOTOR2Init(void);
void _GPIO_MOTORInit(void);
void _GPIO_ADCInit(void);
void _GPIO_TIM3Ch3Init(void);
void GPIO_Configure(void);
void ADC_Configure(void);
void ADC_Initialize(void);
void _TIM3Ch3_PWM_Configure(void);
void _TIM2_Configure(void);
void TIM_PWM_Configure(void);
void DMA_Configure(void);
void _USART_ITR_Configure(void);
void _TIM2_ITR_Configure(void);
void Interrupt_Configure(void);
void SendString(USART_TypeDef* USARTx, char* string);
void _command_forward(void);
void _command_backward(void);
void _command_left(void);
void _command_right(void);
void _command_stop(void);
void command_move(int select);
void USART1_IRQHandler(void);
void USART2_IRQHandler(void);


int main(void) {
	/* SHT15 온습도센서 Variables */
	uint16_t humi_val, temp_val;
	uint8_t err = 0, checksum = 0;
	float humi_val_real = 0.0;
	float temp_val_real = 0.0;
	float dew_point = 0.0;

	/* 초기화 및 Configuration 작업 */
	RCC_Configure();
	LCD_Init();
	USART_Configure();
	GPIO_Configure();
	ADC_Configure();
	TIM_PWM_Configure();
	DMA_Configure();
	Interrupt_Configure();

	ADC_Initialize();
	LCD_Clear(WHITE);

	/* SHT15 초기화 */
	SHT15_Init();

	//LCD_ShowString(40, 10, "## ProtoSmartCar ##", BLACK, WHITE);
	//LCD_DrawRectangle(5, 25, 280, 140);
	while (1) {
		/* 조도센서 값 LCD에 출력
		 *    1) x < 3000 : 밤(어두움)
		 *    2) else : 낮(밝음) */
		LCD_ShowNum(20,40,ADC_result_value_arr[0],10,BLACK,WHITE);
		if(ADC_result_value_arr[0]<3000)
			GPIO_SetBits(GPIOD,GPIO_Pin_2);
		else
			GPIO_ResetBits(GPIOD,GPIO_Pin_2);

		/* 빗물 감지 센서 값 LCD에 출력
		 *    1) x < 3000 : 강한 빗물
		 *    2) x < 3500 : 약한 빗물
		 *    3) else : 빗물 x */
		LCD_ShowNum(20,60,ADC_result_value_arr[1],10,BLACK,WHITE);
		if(ADC_result_value_arr[1]<3000) {
			rain_power_flag = 2;
			GPIO_ResetBits(GPIOD,GPIO_Pin_3);
		}
		else if(ADC_result_value_arr[1]<3500) {
			rain_power_flag = 1;
			GPIO_ResetBits(GPIOD,GPIO_Pin_4);
		}
		else {
			rain_power_flag = 0;
			GPIO_ResetBits(GPIOD,GPIO_Pin_7);
		}

		/* 인체 감지 센서 값 LCD에 출력
		 *    1) x < 2000 : 사람 없음
		 *    2) x > 2000 : 사람 감지 */
		LCD_ShowNum(20, 80, ADC_result_value_arr[2], 10, BLACK, WHITE);
		if(ADC_result_value_arr[2] > 2000) {
			voice_command_enable = 1;
		}else {
			voice_command_enable = 0;
		}

		/* SHT15 온습도 센서 사용
		 *    1) 온도 측정 : float형의 temp_val_real에 저장
		 *    2) 습도 측정 : float형의 humi_val_real에 저장
		 *    3) 이슬점 게산 : float형의 dew_point에 저장 */
		err += SHT15_Measure(&temp_val, &checksum, TEMP);                  // 온도 측정
		err += SHT15_Measure(&humi_val, &checksum, HUMI);                  // 습도 측정
		if(err != 0)
			SHT15_ConReset();
		else {
			SHT15_Calculate(temp_val, humi_val, &temp_val_real, &humi_val_real);	// 실제 온도 및 습도 값 계산
			dew_point = SHT15_CalcuDewPoint(temp_val_real, humi_val_real);			// 이슬점 온도 계산
		}
		LCD_ShowNum(20,100,(int)temp_val_real,10,BLACK,WHITE);
		LCD_ShowNum(20,120,(int)humi_val_real,10,BLACK,WHITE);
		LCD_ShowNum(20,140,(int)dew_point,10,BLACK,WHITE);
	}
}


void RCC_Configure(void) {
	/* Enable Clock for Interrupt EXTI(외부 인터럽트) */
	//RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO);

	/* Enable Clock for Bluetooth(USART2)(안드로이드->보드) and USART1 and USART3(PortB)(음성인식센서) */
	// GPIOA포트는 USART에 clock때문에 사용해야하는지 사용 안해도 되는지 잘 모르겠음
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2ENR_USART1EN, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1ENR_USART2EN | RCC_APB1ENR_USART3EN, ENABLE);

	/* 모터드라이버
	 *    1) 모터드라이버1(in1:PB8,in2:PB9,in3:PB14,in4:PB15)
	 *    2) 모터드라이버2(in1:PE4,in2:PE5,in3:PB10,in4:PE15)  */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE);

	/* LED */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);

	/* 조도센서와 빗물감지센서를 사용하기 위한 ADC & DMA 클럭 인가 */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1,ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1,ENABLE);

	/* Servo Motor를 제어하기 위한 TIM3_Channel3 사용 */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

	/* 와이퍼 작동시 주기 설정을 위한 TIM2 사용 */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
}

void USART_Configure(void) {
	/* UART 통신 & BT 세팅 : USART1 */
	/* Bluetooth module : USART2 */
	USART_InitTypeDef USART_init;

	/* USART 기본 설정 값 세팅 */
	USART_init.USART_BaudRate = 9600;
	USART_init.USART_WordLength = USART_WordLength_8b;
	USART_init.USART_StopBits = USART_StopBits_1;
	USART_init.USART_Parity = USART_Parity_No;
	USART_init.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_init.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

	/* USART1 Init(블루투스모듈 설정을 위해 사용) */
	/* USART2 Init(블루투스 수신을 위해 사용, 안드로이드->보드)
	 * USART3 Init(음성인식센서 사용) */
	USART_Init(USART1, &USART_init);
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

	/* GPIOA의 9번 10번 핀을 Alternative Function으로 세팅 */
	GPIO_USART_init.GPIO_Pin = GPIO_Pin_9;
	GPIO_Init(GPIOA, &GPIO_USART_init);

	GPIO_USART_init.GPIO_Pin = GPIO_Pin_10;
	GPIO_USART_init.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA, &GPIO_USART_init);

	/* GPIOA의 2번 3번 핀을 Alternative Function으로 세팅 */
	GPIO_USART_init.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_USART_init.GPIO_Speed = GPIO_Speed_50MHz;

	GPIO_USART_init.GPIO_Pin = GPIO_Pin_2;
	GPIO_Init(GPIOA, &GPIO_USART_init);

	GPIO_USART_init.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_USART_init.GPIO_Pin = GPIO_Pin_3;
	GPIO_Init(GPIOA, &GPIO_USART_init);

	/* GPIOA의 2번 3번 핀을 Alternative Function으로 세팅 */
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
	GPIO_SetBits(GPIOD, GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_7);
}

void _GPIO_MOTOR1Init(void) {
	GPIO_InitTypeDef GPIOB_MOTOR1_init;

	GPIOB_MOTOR1_init.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIOB_MOTOR1_init.GPIO_Speed = GPIO_Speed_50MHz;

	GPIOB_MOTOR1_init.GPIO_Pin = GPIO_Pin_8;
	GPIO_Init(GPIOB, &GPIOB_MOTOR1_init);

	GPIOB_MOTOR1_init.GPIO_Pin = GPIO_Pin_9;
	GPIO_Init(GPIOB, &GPIOB_MOTOR1_init);

	GPIOB_MOTOR1_init.GPIO_Pin = GPIO_Pin_14;
	GPIO_Init(GPIOB, &GPIOB_MOTOR1_init);

	GPIOB_MOTOR1_init.GPIO_Pin = GPIO_Pin_15;
	GPIO_Init(GPIOB, &GPIOB_MOTOR1_init);

	/* 모터 정지 모드로 초기화 */
	GPIO_ResetBits(GPIOB, GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_14 | GPIO_Pin_15);
}

void _GPIO_MOTOR2Init(void) {
	GPIO_InitTypeDef GPIOBE_MOTOR2_init;

	GPIOBE_MOTOR2_init.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIOBE_MOTOR2_init.GPIO_Speed = GPIO_Speed_50MHz;

	GPIOBE_MOTOR2_init.GPIO_Pin = GPIO_Pin_4;
	GPIO_Init(GPIOE, &GPIOBE_MOTOR2_init);

	GPIOBE_MOTOR2_init.GPIO_Pin = GPIO_Pin_5;
	GPIO_Init(GPIOE, &GPIOBE_MOTOR2_init);

	GPIOBE_MOTOR2_init.GPIO_Pin = GPIO_Pin_10;
	GPIO_Init(GPIOB, &GPIOBE_MOTOR2_init);

	GPIOBE_MOTOR2_init.GPIO_Pin = GPIO_Pin_15;
	GPIO_Init(GPIOE, &GPIOBE_MOTOR2_init);

	/* 모터 정지 모드로 초기화 */
	GPIO_ResetBits(GPIOB, GPIO_Pin_10);
	GPIO_ResetBits(GPIOE, GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_15);
}

void _GPIO_MOTORInit(void) {
	_GPIO_MOTOR1Init();
	_GPIO_MOTOR2Init();
}

void _GPIO_ADCInit(void){
	/* ADC Channel 11 : GPIOC Pin 1
	 * ADC Channel 12 : GPIOC Pin 2
	 * ADC Channel 13 : GPIOC Pin 3
	 * GPIOC mode : Analog Input Mode */
	GPIO_InitTypeDef GPIOC_ADC_init;
	GPIOC_ADC_init.GPIO_Mode = GPIO_Mode_AIN;
	GPIOC_ADC_init.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3;
	GPIO_Init(GPIOC, &GPIOC_ADC_init);
}

void _GPIO_TIM3Ch3Init(void) {
	/* TIM3 Channel 3 : GPIOB Pin 0(Alternative Function) */
	GPIO_InitTypeDef GPIOB_TIM3Ch3_init;

	GPIOB_TIM3Ch3_init.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIOB_TIM3Ch3_init.GPIO_Speed = GPIO_Speed_50MHz;
	GPIOB_TIM3Ch3_init.GPIO_Pin = GPIO_Pin_0;
	GPIO_Init(GPIOB, &GPIOB_TIM3Ch3_init);
}

void GPIO_Configure(void) {
	_GPIO_USARTInit();
	_GPIO_LEDInit();
	_GPIO_MOTORInit();
	_GPIO_ADCInit();
	_GPIO_TIM3Ch3Init();
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
	ADC_InitStructure.ADC_NbrOfChannel=3;
	ADC_RegularChannelConfig(ADC1,ADC_Channel_11,1,ADC_SampleTime_239Cycles5);
	ADC_RegularChannelConfig(ADC1,ADC_Channel_12,2,ADC_SampleTime_239Cycles5);
	ADC_RegularChannelConfig(ADC1,ADC_Channel_13,3,ADC_SampleTime_239Cycles5);
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

void _TIM3Ch3_PWM_Configure(void) {
	/* 기본 System Clock : 72MHz
	 * Prescale = 72MHz / 1MHz - 1 = 71 => 72MHz를 1MHz Timer clock으로 세팅
	 * TIM_Period = 1MHz / 50Hz = 20,000 => 1MHz Timer Clock을 20,000주기를 통해 50Hz 주기로 세팅 */
	TIM_TimeBaseInitTypeDef TIM3Ch3_Init;

	/* Servo Moter 0도 : 7.5% Duty Cycle을 초기 값으로 PWM mode 설정 */
	TIM_OCInitTypeDef PWM_TIM3Ch3_Init;

	TIM3Ch3_Init.TIM_Prescaler = (uint16_t)(SystemCoreClock / 1000000) - 1;
	TIM3Ch3_Init.TIM_Period = 20000 - 1;
	TIM3Ch3_Init.TIM_ClockDivision = 0;
	TIM3Ch3_Init.TIM_CounterMode = TIM_CounterMode_Down;
	TIM_TimeBaseInit(TIM3, &TIM3Ch3_Init);

	PWM_TIM3Ch3_Init.TIM_OCMode = TIM_OCMode_PWM1;
	PWM_TIM3Ch3_Init.TIM_OCPolarity = TIM_OCPolarity_High;
	PWM_TIM3Ch3_Init.TIM_OutputState = TIM_OutputState_Enable;
	PWM_TIM3Ch3_Init.TIM_Pulse = 1500;		// 50% duty cycle value
	TIM_OC3Init(TIM3, &PWM_TIM3Ch3_Init);
	TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Disable);

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

void TIM_PWM_Configure(void){
	_TIM3Ch3_PWM_Configure();
	_TIM2_Configure();
}

void DMA_Configure(void){
	/* DMA1 Channel 11번 Configure*/
	DMA_InitTypeDef DMA_init;

	DMA_DeInit(DMA1_Channel1);
	DMA_init.DMA_PeripheralBaseAddr=(uint32_t)&ADC1->DR;
	DMA_init.DMA_MemoryBaseAddr=(uint32_t)ADC_result_value_arr;
	DMA_init.DMA_DIR=DMA_DIR_PeripheralSRC;
	DMA_init.DMA_BufferSize=3;
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

void Interrupt_Configure(void) {
	_USART_ITR_Configure();
	_TIM2_ITR_Configure();
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
	GPIO_SetBits(GPIOB, GPIO_Pin_8);
	GPIO_ResetBits(GPIOB, GPIO_Pin_9);
	/* 오른쪽 바퀴 : in3(High)/in4(Low) */
	GPIO_SetBits(GPIOB, GPIO_Pin_14);
	GPIO_ResetBits(GPIOB, GPIO_Pin_15);

	/* 모터드라이버2 제어 */
	/* 왼쪽 바퀴 : in1(High)/in2(Low) */
	GPIO_SetBits(GPIOE, GPIO_Pin_4);
	GPIO_ResetBits(GPIOE, GPIO_Pin_5);
	/* 오른쪽 바퀴 : in3(High)/in4(Low) */
	GPIO_SetBits(GPIOB, GPIO_Pin_10);
	GPIO_ResetBits(GPIOE, GPIO_Pin_15);
}

void _command_backward(void) {
	/* 모터드라이버1 제어 */
	/* 왼쪽 바퀴 : in1(Low)/in2(High) */
	GPIO_ResetBits(GPIOB, GPIO_Pin_8);
	GPIO_SetBits(GPIOB, GPIO_Pin_9);
	/* 오른쪽 바퀴 : in3(Low)/in4(High) */
	GPIO_ResetBits(GPIOB, GPIO_Pin_14);
	GPIO_SetBits(GPIOB, GPIO_Pin_15);

	/* 모터드라이버2 제어 */
	/* 왼쪽 바퀴 : in1(Low)/in2(High) */
	GPIO_ResetBits(GPIOE, GPIO_Pin_4);
	GPIO_SetBits(GPIOE, GPIO_Pin_5);
	/* 오른쪽 바퀴 : in3(Low)/in4(High) */
	GPIO_ResetBits(GPIOB, GPIO_Pin_10);
	GPIO_SetBits(GPIOE, GPIO_Pin_15);
}

void _command_left(void) {
	/* 모터드라이버1이 앞쪽바퀴 */

	/* 모터드라이버1 제어 */
	/* 왼쪽 바퀴 : in1(Low)/in2(High) => 역방향 */
	GPIO_ResetBits(GPIOB, GPIO_Pin_8);
	GPIO_SetBits(GPIOB, GPIO_Pin_9);
	/* 오른쪽 바퀴 : in3(High)/in4(Low) => 정방향 */
	GPIO_SetBits(GPIOB, GPIO_Pin_14);
	GPIO_ResetBits(GPIOB, GPIO_Pin_15);

	/* 모터드라이버2 제어 */
	/* 왼쪽 바퀴 : in1(Low)/in2(High) => 역방향 */
	GPIO_ResetBits(GPIOE, GPIO_Pin_4);
	GPIO_SetBits(GPIOE, GPIO_Pin_5);
	/* 오른쪽 바퀴 : in3(High)/in4(Low) => 정방향 */
	GPIO_SetBits(GPIOB, GPIO_Pin_10);
	GPIO_ResetBits(GPIOE, GPIO_Pin_15);
}

void _command_right(void) {
	/* 모터드라이버1이 앞쪽바퀴 */

	/* 모터드라이버1 제어 */
	/* 왼쪽 바퀴 : in1(High)/in2(Low) => 정방향 */
	GPIO_SetBits(GPIOB, GPIO_Pin_8);
	GPIO_ResetBits(GPIOB, GPIO_Pin_9);
	/* 오른쪽 바퀴 : in3(Low)/in4(High) => 역방향 */
	GPIO_ResetBits(GPIOB, GPIO_Pin_14);
	GPIO_SetBits(GPIOB, GPIO_Pin_15);

	/* 모터드라이버2 제어 */
	/* 왼쪽 바퀴 : in1(High)/in2(Low) => 정방향 */
	GPIO_SetBits(GPIOE, GPIO_Pin_4);
	GPIO_ResetBits(GPIOE, GPIO_Pin_5);
	/* 오른쪽 바퀴 : in3(Low)/in4(High) => 역방향 */
	GPIO_ResetBits(GPIOB, GPIO_Pin_10);
	GPIO_SetBits(GPIOE, GPIO_Pin_15);
}

void _command_stop(void) {
	/* 모터드라이버1 제어 */
	/* 왼쪽 바퀴 : in1(Low)/in2(Low) => 정지
	 * 오른쪽 바퀴 : in3(Low)/in4(Low) => 정지 */
	GPIO_ResetBits(GPIOB, GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_14 | GPIO_Pin_15);

	/* 모터드라이버2 제어 */
	/* 왼쪽 바퀴 : in1(Low)/in2(Low) => 정지
	 * 오른쪽 바퀴 : in3(Low)/in4(Low) => 정지 */
	GPIO_ResetBits(GPIOE, GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_15);
	GPIO_ResetBits(GPIOB, GPIO_Pin_10);
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

/* Todo : 개발자 디버그용이므로 나중에 지울 것 */
void USART1_IRQHandler(void) {
	char recv_data;

	/* USART1에서 정보를 받아 출력한다.(블루투스 세팅 용도 & 안드로이드 어플로 송신) */
	if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET) {
		recv_data = USART_ReceiveData(USART1);

		if(recv_data) {
			//USART_SendData(USART1, recv_data);								// wjdebug
			//while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);		// wjdebug
			USART_SendData(USART2, recv_data);
			while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);
			USART_ClearITPendingBit(USART1, USART_IT_RXNE);
		}
	}
}

void USART2_IRQHandler(void) {
	char recv_data;

	/* 안드로이드 어플리케이션에서 보내는 문자열 명령을 받아서 명령어별로 처리하는 인터럽트 */
	/* Command End : "\n"
	   Command List : "FORWARD", "BACKWARD", "LEFT", "RIGHT", "STOP", "ON", "OFF"
	   Optional Command List : "LIGHT_ON", "LIGHT_OFF", "TEMPER_UP", "TEMPER_DOWN", "WIPER_ON", "WIPER_OFF"
	*/
	if (USART_GetITStatus(USART2, USART_IT_RXNE) != RESET) {
		recv_data = USART_ReceiveData(USART2);

		if(recv_data == '!') {
			if(strstr(command, "CLEAR") != NULL) {
				GPIO_ResetBits(GPIOD, GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_7);
			}else if(strstr(command, "POWERON") != NULL) {
				GPIO_SetBits(GPIOD, GPIO_Pin_2);
			}else if(strstr(command, "POWEROFF") != NULL) {
				GPIO_ResetBits(GPIOD, GPIO_Pin_2);
			}else if(strstr(command, "FORWARD") != NULL) {
				GPIO_SetBits(GPIOD, GPIO_Pin_3);
				command_move(1);
			}else if(strstr(command, "BACKWARD") != NULL) {
				GPIO_ResetBits(GPIOD, GPIO_Pin_3);
				command_move(2);
			}else if(strstr(command, "LEFT") != NULL) {
				GPIO_SetBits(GPIOD, GPIO_Pin_4);
				command_move(3);
			}else if(strstr(command, "RIGHT") != NULL) {
				GPIO_SetBits(GPIOD, GPIO_Pin_7);
				command_move(4);
			}else if(strstr(command, "STOP") != NULL) {
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
			LCD_ShowString(20, 140, voiceBuffer[voice_command-1], BLACK, WHITE);
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
		TIM_ClearITPendingBit(TIM2, TIM_IT_Update);		// clear interrupt flag
	}
}
