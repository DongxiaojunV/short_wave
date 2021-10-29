#include "NTP.h"
#include "W5500.h"

uint8_t NTP_Data[48]={0xa3,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}; //48字节的报文
//00100011, 0xa3,100 版本4
//00011011, 0x1b,011 版本3
//00010011, 0x13,010 版本2
//00001011, 0x0b,001 版本1

void NTP_Pack_Send(void)
{
		Write_SOCK_Data_Buffer(3, NTP_Data, 48);	
}


