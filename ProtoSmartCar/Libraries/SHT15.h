/*************************************************************
                             \(^o^)/
  Copyright (C), 2013-2020, ZheJiang University of Technology
  File name  : SHT15.h 
  Author     : ziye334    
  Version    : V1.0 
  Data       : 2014/3/10      
  Description: Digital temperature and humidity sensor driver code
  
*************************************************************/
#ifndef __SHT15_H__
#define __SHT15_H__
#include "stm32f10x.h"

enum {TEMP, HUMI};

/* GPIO 매크로 정의 */
#define SHT15_RCC_Periph      RCC_APB2Periph_GPIOA
#define SHT15_DATA_PIN        GPIO_Pin_12
#define SHT15_SCK_PIN         GPIO_Pin_11
#define SHT15_DATA_PORT       GPIOA
#define SHT15_SCK_PORT        GPIOA

#define SHT15_DATA_H()        GPIO_SetBits(SHT15_DATA_PORT, SHT15_DATA_PIN)				// Data 라인 Pull
#define SHT15_DATA_L()        GPIO_ResetBits(SHT15_DATA_PORT, SHT15_DATA_PIN)			// Data 라인 Pull-down
#define SHT15_DATA_R()        GPIO_ReadInputDataBit(SHT15_DATA_PORT, SHT15_DATA_PIN)	// Data 라인 읽기

#define SHT15_SCK_H()        GPIO_SetBits(SHT15_SCK_PORT, SHT15_SCK_PIN)				// SCK 클럭 라인 High
#define SHT15_SCK_L()        GPIO_ResetBits(SHT15_SCK_PORT, SHT15_SCK_PIN)				// SCK 클럭 라인 Pull-down

/* 센서 관련 매크로 정의 */
#define noACK			0
#define ACK				1
									//addr  	  command       r/w
#define STATUS_REG_W	0x06        //000         0011          0          상태 레지스터 쓰기
#define STATUS_REG_R	0x07		//000         0011          1          상태 레지스터 읽기
#define MEASURE_TEMP	0x03		//000         0001          1          온도 측정
#define MEASURE_HUMI	0x05		//000         0010          1          습도 측정
#define SOFTRESET		0x1E		//000         1111          0          재설정


void SHT15_Init(void);
void SHT15_ConReset(void);
u8 SHT15_SoftReset(void);
u8 SHT15_Measure(u16 *p_value, u8 *p_checksum, u8 mode);
void SHT15_Calculate(u16 t, u16 rh,float *p_temperature, float *p_humidity);
float SHT15_CalcuDewPoint(float t, float h);


#endif



