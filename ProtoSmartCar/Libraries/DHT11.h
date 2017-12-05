#ifndef __DHT11_H
#define __DHT11_H 
#include "sys.h"   
											   
#define	DHT11_DQ_OUT PEout(0)
#define	DHT11_DQ_IN  PEin(0)


u8 DHT11_Init(void); //Init DHT11
u8 DHT11_Read_Data(u8 *temperature,u8 *humidity); //Read DHT11 Value
u8 DHT11_Read_Byte(void);//Read One Byte
u8 DHT11_Read_Bit(void);//Read One Bit
u8 DHT11_Check(void);//Chack DHT11
void DHT11_IO_IN(void);//Set GPIO Direction
void DHT11_IO_OUT(void);//Set GPIO Direction
void DHT11_Rst(void);//Reset DHT11    
#endif















