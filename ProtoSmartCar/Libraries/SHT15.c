#include "stm32f10x.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_gpio.h"
#include "SHT15.h"
#include <math.h>

/*************************************************************
  Function   : SHT15_Dly
  Description: SHT15 Delay
  Input      : none        
  return     : none    
*************************************************************/
void SHT15_Dly(void)
{
    uint16_t i;
    for(i = 500; i > 0; i--);
}


/*************************************************************
  Function   : SHT15_Config
  Description: SHT15 핀 초기화
  Input      : none        
  return     : none    
*************************************************************/
void SHT15_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;        
    /* SHT15 핀 클럭 초기화 */
    RCC_APB2PeriphClockCmd(SHT15_RCC_Periph ,ENABLE);
            
    /* DATA push-pull Output */
    GPIO_InitStructure.GPIO_Pin = SHT15_DATA_PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(SHT15_DATA_PORT, &GPIO_InitStructure);
    /* SCK push-pull Output */
    GPIO_InitStructure.GPIO_Pin = SHT15_SCK_PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;    
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(SHT15_SCK_PORT, &GPIO_InitStructure);

    /* 통신 재설정 */
    SHT15_ConReset();
}


/*************************************************************
  Function   : SHT15_DATAOut
  Description: Data Pin을 출력모드로 지정
  Input      : none        
  return     : none    
*************************************************************/
void SHT15_DATAOut(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    /* Data Pin push-pull Output모드로 지정 */
    GPIO_InitStructure.GPIO_Pin = SHT15_DATA_PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;         
    GPIO_Init(SHT15_DATA_PORT, &GPIO_InitStructure);
}


/*************************************************************
  Function   : SHT15_DATAIn
  Description: Data Pin을 입력모드로 지정
  Input      : none        
  return     : none    
*************************************************************/
void SHT15_DATAIn(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    /* Data 플로팅 Input모드로 지정 */
    GPIO_InitStructure.GPIO_Pin = SHT15_DATA_PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(SHT15_DATA_PORT, &GPIO_InitStructure);
}


/*************************************************************
  Function   : SHT15_WriteByte
  Description: 1byte 쓰기
  Input      : value:쓸 바이트
  return     : err: 0-수정  1-오류
*************************************************************/
uint8_t SHT15_WriteByte(uint8_t value)
{
    uint8_t i, err = 0;
    
    SHT15_DATAOut();                // Data라인을 출력으로 설정

    for(i = 0x80; i > 0; i /= 2)    // 1byte 쓰기
    {
        if(i & value)
                SHT15_DATA_H();
        else
                SHT15_DATA_L();
        SHT15_Dly();
        SHT15_SCK_H();
        SHT15_Dly();
        SHT15_SCK_L();
        SHT15_Dly();
    }
    SHT15_DATAIn();                 // Data라인을 입력으로 설정하고 Data라인을 해제
    SHT15_SCK_H();
    err = SHT15_DATA_R();           // SHT15 확인 비트 읽기
    SHT15_SCK_L();

    return err;
}

/*************************************************************
  Function   : SHT15_ReadByte
  Description: 1byte 데이터 읽기
  Input      : Ack: 0-no_answer  1-answer
  return     : err: 0-수정 1-오류
*************************************************************/
uint8_t SHT15_ReadByte(uint8_t Ack)
{
    uint8_t i, val = 0;

    SHT15_DATAIn();                // Data 라인을 입력으로 설정
    for(i = 0x80; i > 0; i /= 2)   // 1byte 데이터 읽기
    {
        SHT15_Dly();
        SHT15_SCK_H();
        SHT15_Dly();
        if(SHT15_DATA_R())
        {
           val = (val | i);
        }
        SHT15_SCK_L();
    }
    SHT15_DATAOut();               // Data 라인을 출력으로 지정
    if(Ack)
            SHT15_DATA_L();        // ACK를 받으면, 그 다음 데이터를 읽기
    else
            SHT15_DATA_H();        // ACK를 받지 못하면, 데이터 끝
    SHT15_Dly();
    SHT15_SCK_H();
    SHT15_Dly();
    SHT15_SCK_L();
    SHT15_Dly();

    return val;                    // Read한 값 반환
}


/*************************************************************
  Function   : SHT15_TransStart
  Description: Transmit을 시작하면 다음과 같은 타이밍을 가짐
                     _____         ________
               DATA:      |_______|
                         ___     ___
               SCK : ___|   |___|   |______        
  Input      : none        
  return     : none    
*************************************************************/
void SHT15_TransStart(void)
{
    SHT15_DATAOut();               // Data 라인을 출력으로 지정

    SHT15_DATA_H();
    SHT15_SCK_L();
    SHT15_Dly();
    SHT15_SCK_H();
    SHT15_Dly();
    SHT15_DATA_L();
    SHT15_Dly();
    SHT15_SCK_L();
    SHT15_Dly();
    SHT15_SCK_H();
    SHT15_Dly();
    SHT15_DATA_H();
    SHT15_Dly();
    SHT15_SCK_L();
}


/*************************************************************
  Function   : SHT15_ConReset
  Description: 통신 재설정하는 함수, 아래와 같은 타이밍 클럭을 가짐
                     _____________________________________________________         ________
               DATA:                                                      |_______|
                        _    _    _    _    _    _    _    _    _        ___     ___
               SCK : __| |__| |__| |__| |__| |__| |__| |__| |__| |______|   |___|   |______
  Input      : none        
  return     : none    
*************************************************************/
void SHT15_ConReset(void)
{
    uint8_t i;

    SHT15_DATAOut();

    SHT15_DATA_H();
    SHT15_SCK_L();

    for(i = 0; i < 9; i++)              // SCK 클럭을 9번 Trigger
    {
        SHT15_SCK_H();
        SHT15_Dly();
        SHT15_SCK_L();
        SHT15_Dly();
    }
    SHT15_TransStart();                 // 전송 시작
}



/*************************************************************
  Function   : SHT15_SoftReset
  Description: 소프트 리셋
  Input      : none        
  return     : err: 0-수정  1-오류
*************************************************************/
uint8_t SHT15_SoftReset(void)
{
    uint8_t err = 0;

    SHT15_ConReset();                   // 통신 재설정
    err += SHT15_WriteByte(SOFTRESET);  // RESET 명령 쓰기

    return err;
}


/*************************************************************
  Function   : SHT15_ReadStatusReg
  Description: Status 레지스터 읽기
  Input      : p_value-데이터 읽기, p_checksun-체크섬 테이터 읽기
  return     : err: 0-수정  1-오류
*************************************************************/
uint8_t SHT15_ReadStatusReg(uint8_t *p_value, uint8_t *p_checksum)
{
    uint8_t err = 0;

    SHT15_TransStart();						// 전송 시작
    err = SHT15_WriteByte(STATUS_REG_R);	// Status 레지스터 명령을 읽기위해 STATUS_REG_R을 씀.
    *p_value = SHT15_ReadByte(ACK);         // Status 데이터 읽기
    *p_checksum = SHT15_ReadByte(noACK);	// checksum 데이터 읽기
    
    return err;
}



/*************************************************************
  Function   : SHT15_WriteStatusReg
  Description: Status 레지스터 쓰기
  Input      : p_value-기록할 데이터 값
  return     : err: 0-수정  1-오류
*************************************************************/
uint8_t SHT15_WriteStatusReg(uint8_t *p_value)
{
        uint8_t err = 0;

        SHT15_TransStart();                     // 전송 시작
        err += SHT15_WriteByte(STATUS_REG_W);   // 상태 레지스터에 쓰기 위해 STATUS_REG_W를 씀
        err += SHT15_WriteByte(*p_value);       // 구성 값 작성

        return err;
}



/*************************************************************
  Function   : SHT15_Measure
  Description: 온도 및 습도 센서 읽기
  Input      : p_value-읽을 값, p_checksum-체크섬 값
  return     : err: 0-수정 1―오류
*************************************************************/
uint8_t SHT15_Measure(uint16_t *p_value, uint8_t *p_checksum, uint8_t mode)
{
    uint8_t err = 0;
    uint32_t i;
    uint8_t value_H = 0;
    uint8_t value_L = 0;

    SHT15_TransStart();								// 전송 시작
    switch(mode)                                                         
    {
    case TEMP:										// 온도 측정
        err += SHT15_WriteByte(MEASURE_TEMP);		// 온도를 측정하기 위해 MEASURE_TEMP를 씀
        break;
    case HUMI:
        err += SHT15_WriteByte(MEASURE_HUMI);		// 습도를 측정하기 위해 MEASURE_HUMI를 씀
        break;
    default:
        break;
    }
    if(err != 0)
    {
        return err;
    }
    SHT15_DATAIn();
    for(i = 0; i < 1200000; i++)					// Data 신호가 Low가 될때까지 기다림
    {
        if(SHT15_DATA_R() == 0) break;				// DATA가 루프 밖으로 빠져 나가는 것 감지
    }
    if(SHT15_DATA_R() == 1)							// 대기 시간이 초과 될 경우
    {
        err += 1;
        return err;
    }
    value_H = SHT15_ReadByte(ACK);
    value_L = SHT15_ReadByte(ACK);
    *p_checksum = SHT15_ReadByte(noACK);			// Calibration 데이터 읽기
    *p_value = (value_H << 8) | value_L;
    return err;
}


/*************************************************************
  Function   : SHT15_Calculate
  Description: 온도 및 습도 값 계산
  Input      : Temp-센서에서 읽은 온도 값, Humi-센서에서 읽은 습도 값
               p_humidity-계산된 실제 습도 값, p_temperature-계산된 실제 온도 값
  return     : none    
*************************************************************/
void SHT15_Calculate(uint16_t t, uint16_t rh, float *p_temperature, float *p_humidity)
{
    // Datasheet의 매개 변수 데이터
    const float d1 = -39.55;
    const float d2 = +0.01;
    const float C1 = -4;
    const float C2 = +0.0405;
    const float C3 = -0.0000028;        
    const float T1 = +0.01;
    const float T2 = +0.00008;

    float RH_Lin;                                                     // RH 선형 값
    float RH_Ture;                                                    // RH 실제 값
    float temp_C;

    temp_C = d1 + d2 * t;                                            // 온도 값 계산
    RH_Lin = C1 + C2 * rh + C3 * rh * rh;                            // 습도 값 계산
    RH_Ture = (temp_C -25) * (T1 + T2 * rh) + RH_Lin;                // 습도 온도 보상, 실제 습도 값 계산
    RH_Ture = (RH_Ture > 100) ? 100 : RH_Ture;
    RH_Ture = (RH_Ture < 0.1) ? 0.1 : RH_Ture;                       // 습도 하한선 설정

    *p_humidity = RH_Ture;
    *p_temperature = temp_C;

}


/*************************************************************
  Function   : SHT15_CalcuDewPoint
  Description: 이슬점 계산
  Input      : h-실제 습도, t-실제 온도
  return     : dew_point-이슬점
*************************************************************/
float SHT15_CalcuDewPoint(float t, float h)
{
    float logEx, dew_point;

    logEx = 0.66077 + 7.5 * t / (237.3 + t) + (log10(h) - 2);
    dew_point = ((0.66077 - logEx) * 237.3) / (logEx - 8.16077);

    return dew_point; 
}


//int main(void)
//{
//        uint16_t humi_val, temp_val;
//        uint8_t err = 0, checksum = 0;
//        float humi_val_real = 0.0;
//        float temp_val_real = 0.0;
//        float dew_point = 0.0;
//
//        BSP_Init();
//        printf("\nSHT15 Sensor Test!!!\n");
//        SHT15_Config();
//        while(1)
//        {
//                err += SHT15_Measure(&temp_val, &checksum, TEMP);                  // 온도 측정
//                err += SHT15_Measure(&humi_val, &checksum, HUMI);                  // 습도 측정
//                if(err != 0)
//                        SHT15_ConReset();
//                else
//                {
//                        SHT15_Calculate(temp_val, humi_val, &temp_val_real, &humi_val_real);	// 실제 온도 및 습도 값 계산
//                        dew_point = SHT15_CalcuDewPoint(temp_val_real, humi_val_real);		// 이슬점 온도 계산
//                }
//                printf("Ambient temperature : %2.1f→，Humidity : %2.1f%%，dew_point_temperature : %2.1f→\r\n", temp_val_real, humi_val_real, dew_point);
//                LED1_Toggle();
//                Delay_ms(1000);
//        }
//}





