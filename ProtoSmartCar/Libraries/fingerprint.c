#include "fingerprint.h"
#include <stm32f10x_usart.h>
#include <stm32f10x_gpio.h>
#include <lcd.h>


u8 gRsBuf[9];			
u8 gTxBuf[9];				 
u8 gRsLength = 0;

void delay(void)
{
	u16 i, j;
	for (i = 0; i < 1000; i++)
		for(j = 0; j < 10000; j++);
}


void TxByte(u8 temp)
{
	USART_SendData(FG_UART,temp);
	while (USART_GetFlagStatus(FG_UART, USART_FLAG_TXE) == RESET);
}

u8 TxAndRsCmd(u8 Scnt, u8 Rcnt, u8 Delay)
{
	u8  i, j, CheckSum;
	u32 RsTimeCnt;
	TxByte(CMD_HEAD);
	CheckSum = 0;
	for (i = 0; i < Scnt; i++)
	{
		TxByte(gTxBuf[i]);
		CheckSum ^= gTxBuf[i];
	}
	TxByte(CheckSum);
	TxByte(CMD_TAIL);
	gRsLength = 0;
	RsTimeCnt = Delay * 120000;
	while (gRsLength < Rcnt && RsTimeCnt > 0)
		RsTimeCnt--;
	GPIO_SetBits(GPIOD, GPIO_Pin_2);

	if (gRsLength != Rcnt)return ACK_TIMEOUT;
	if (gRsBuf[0] != CMD_HEAD) return ACK_FAIL;
	if (gRsBuf[Rcnt - 1] != CMD_TAIL) return ACK_FAIL;
	if (gRsBuf[1] != (gTxBuf[0])) return ACK_FAIL;

	CheckSum = 0;
	for (j = 1; j < gRsLength - 1; j++) CheckSum ^= gRsBuf[j];
	if (CheckSum != 0) return ACK_FAIL;

	return ACK_SUCCESS;
}	 


u8 GetUserCount(void)
{
	u8 m;

	gTxBuf[0] = CMD_USER_CNT;
	gTxBuf[1] = 0;
	gTxBuf[2] = 0;
	gTxBuf[3] = 0;
	gTxBuf[4] = 0;	

	m = TxAndRsCmd(5, 8, 10);

	if (m == ACK_SUCCESS && gRsBuf[4] == ACK_SUCCESS)
	{
		return gRsBuf[3];
	}
	else
	{
		return 0xFF;
	}
}

u8 GetcompareLevel(void)
{
	u8 m;

	gTxBuf[0] = CMD_COM_LEV;
	gTxBuf[1] = 0;
	gTxBuf[2] = 0;
	gTxBuf[3] = 1;
	gTxBuf[4] = 0;	

	m = TxAndRsCmd(5, 8, 10);

	if (m == ACK_SUCCESS && gRsBuf[4] == ACK_SUCCESS)
	{
		return gRsBuf[3];
	}
	else
	{
		return 0xFF;
	}
}

u8 SetcompareLevel(u8 temp)
{
	u8 m;

	gTxBuf[0] = CMD_COM_LEV;
	gTxBuf[1] = 0;
	gTxBuf[2] = temp;
	gTxBuf[3] = 0;
	gTxBuf[4] = 0;	

	m = TxAndRsCmd(5, 8, 100);

	if (m == ACK_SUCCESS && gRsBuf[4] == ACK_SUCCESS)
	{
		return gRsBuf[3];
	}
	else
	{
		return 0xFF;
	}
}

u8 GetTimeOut(void)
{
	u8 m;

	gTxBuf[0] = CMD_TIMEOUT;
	gTxBuf[1] = 0;
	gTxBuf[2] = 0;
	gTxBuf[3] = 1;
	gTxBuf[4] = 0;	

	m = TxAndRsCmd(5, 8, 10);

	if (m == ACK_SUCCESS && gRsBuf[4] == ACK_SUCCESS)
	{
		return gRsBuf[3];
	}
	else
	{
		return 0xFF;
	}
}

u8 AddUser(u8 k)
{
	u8 m;

	m = GetUserCount();
	if (m >= USER_MAX_CNT)
		return ACK_FULL;


	gTxBuf[0] = CMD_ADD_1;
	gTxBuf[1] = 0;
	gTxBuf[2] = k;
	gTxBuf[3] = 3;
	gTxBuf[4] = 0;	

	GPIO_SetBits(GPIOD,GPIO_Pin_2);			// LED1 ON
	m = TxAndRsCmd(5, 8, 200);
	GPIO_ResetBits(GPIOD,GPIO_Pin_2);		// LED1 OFF

	if (m == ACK_SUCCESS && gRsBuf[4] == ACK_SUCCESS)
	{
		gTxBuf[0] = CMD_ADD_2;
		GPIO_SetBits(GPIOD,GPIO_Pin_2);		// LED1 ON
		m = TxAndRsCmd(5, 8, 200);
		GPIO_ResetBits(GPIOD,GPIO_Pin_2);	// LED1 OFF
		if (m == ACK_SUCCESS && gRsBuf[4] == ACK_SUCCESS)
		{
			gTxBuf[0] = CMD_ADD_3;
			GPIO_SetBits(GPIOD,GPIO_Pin_2);
			m = TxAndRsCmd(5, 8, 200);
			GPIO_ResetBits(GPIOD,GPIO_Pin_2);
			if (m == ACK_SUCCESS && gRsBuf[4] == ACK_SUCCESS)
			{
				return ACK_SUCCESS;
			}
			else
				return ACK_FAIL;
		}
		else
			return ACK_FAIL;
	}
	else
		return ACK_FAIL;
}

void ClearAllUser(void)
{
	u8 m;

	gTxBuf[0] = CMD_DEL_ALL;
	gTxBuf[1] = 0;
	gTxBuf[2] = 0;
	gTxBuf[3] = 0;
	gTxBuf[4] = 0;

	m = TxAndRsCmd(5, 8, 50);

	if (m == ACK_SUCCESS && gRsBuf[4] == ACK_SUCCESS)
	{	    
		GPIO_SetBits(GPIOD,GPIO_Pin_3);			// LED2_ON;
		delay();
		GPIO_ResetBits(GPIOD,GPIO_Pin_3);		// LED2_OFF;
	}
	else
	{
		GPIO_SetBits(GPIOD,GPIO_Pin_4);			// LED3_ON;
		delay();
		GPIO_ResetBits(GPIOD,GPIO_Pin_4);		// LED3_OFF;
	}
}


u8 IsMasterUser(u8 UserID)
{
	if ((UserID == 1) || (UserID == 2) || (UserID == 3)) return TRUE;
	else  return FALSE;
}	 


u8 VerifyUser(u8 userid)
{
	u8 m;

	gTxBuf[0] = CMD_MATCH;
	gTxBuf[1] = userid;
	gTxBuf[2] = 0;
	gTxBuf[3] = 0;
	gTxBuf[4] = 0;

	m = TxAndRsCmd(5, 8, 150);

	if ((m == ACK_SUCCESS) && (IsMasterUser(gRsBuf[4]) == TRUE))
	{	
		return ACK_SUCCESS;
	}
	else if(gRsBuf[4] == ACK_NO_USER)
	{
		return ACK_NO_USER;
	}
	else if(gRsBuf[4] == ACK_TIMEOUT)
	{
		return ACK_TIMEOUT;
	}
	else
	{
		return ACK_GO_OUT;
	}
}

void USART1_IRQHandler(void) {
	char recv_data;

	/* Fingerprint Reader의 response를 받아서 전역변수인 gRsBuf에 저장한다. */
	if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET) {
		recv_data = USART_ReceiveData(USART1);
		gRsBuf[gRsLength++] = recv_data;

		USART_ClearITPendingBit(USART1, USART_IT_RXNE);
	}
}

void PrintResponse(void) {
	LCD_ShowNum(50, 130, gRsBuf[0], 10, BLACK, WHITE);
	LCD_ShowNum(50, 150, gRsBuf[1], 10, BLACK, WHITE);
	LCD_ShowNum(50, 170, gRsBuf[2], 10, BLACK, WHITE);
	LCD_ShowNum(50, 190, gRsBuf[3], 10, BLACK, WHITE);
	LCD_ShowNum(50, 210, gRsBuf[4], 10, BLACK, WHITE);
}
