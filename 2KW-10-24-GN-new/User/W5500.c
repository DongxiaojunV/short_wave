#include "stm32f10x.h"
#include "stm32f10x_spi.h"				
#include "W5500.h"	
/***************----- 网络参数变量定义 -----***************/
unsigned char Gateway_IP[4];//网关IP地址 
unsigned char Sub_Mask[4];	//子网掩码 
unsigned char Phy_Addr[6];	//物理地址(MAC) 
unsigned char IP_Addr[4];	//本机IP地址 

unsigned char S0_Port[2];	//端口0的端口号(5000) 
unsigned char S0_DIP[4];	//端口0目的IP地址 
unsigned char S0_DPort[2];	//端口0目的端口号(6000) 
unsigned char S1_Port[2];	//端口1的端口号
unsigned char S1_DIP[4];		//端口1目的IP地址 
unsigned char S1_DPort[2];	//端口1目的端口号
unsigned char S2_Port[2];	//端口2的端口号
unsigned char S2_DIP[4];		//端口2目的IP地址 
unsigned char S2_DPort[2];	//端口2目的端口号
unsigned char S3_Port[2];	//端口3的端口号
unsigned char S3_DIP[4];		//端口3目的IP地址 
unsigned char S3_DPort[2];	//端口3目的端口号

unsigned char UDP_DIPR[4];	//UDP(广播)模式,目的主机IP地址
unsigned char UDP_DPORT[2];	//UDP(广播)模式,目的主机端口号

unsigned char NTP_DIPR[4];	//UDP(广播)模式,NTP服务器主机IP地址
unsigned char NTP_DPORT[2];	//UDP(广播)模式,NTP服务器主机端口号

/***************----- 端口的运行模式 -----***************/
unsigned char S0_Mode =3;	//端口0的运行模式,0:TCP服务器模式,1:TCP客户端模式,2:UDP(广播)模式
unsigned char S1_Mode =3;	//端口1的运行模式,0:TCP服务器模式,1:TCP客户端模式,2:UDP(广播)模式
unsigned char S2_Mode =3;	//端口2的运行模式,0:TCP服务器模式,1:TCP客户端模式,2:UDP(广播)模式
unsigned char S3_Mode =3;	//端口2的运行模式,0:TCP服务器模式,1:TCP客户端模式,2:UDP(广播)模式
#define TCP_SERVER	0x00	//TCP服务器模式
#define TCP_CLIENT	0x01	//TCP客户端模式 
#define UDP_MODE	0x02	//UDP(广播)模式 

/***************----- 端口的运行状态 -----***************/
unsigned char S0_State =0;	//端口0状态记录,1:端口完成初始化,2端口完成连接(可以正常传输数据) 
unsigned char S1_State =0;	//端口1状态记录,1:端口完成初始化,2端口完成连接(可以正常传输数据) 
unsigned char S2_State =0;	//端口2状态记录,1:端口完成初始化,2端口完成连接(可以正常传输数据) 
unsigned char S3_State =0;	//端口2状态记录,1:端口完成初始化,2端口完成连接(可以正常传输数据) 
#define S_INIT		0x01	//端口完成初始化 
#define S_CONN		0x02	//端口完成连接,可以正常传输数据 

/***************----- 端口收发数据的状态 -----***************/
unsigned char S0_Data;		//端口0接收和发送数据的状态,1:端口接收到数据,2:端口发送数据完成
unsigned char S1_Data;		//端口1接收和发送数据的状态,1:端口接收到数据,2:端口发送数据完成 
unsigned char S2_Data;		//端口2接收和发送数据的状态,1:端口接收到数据,2:端口发送数据完成 
unsigned char S3_Data;		//端口2接收和发送数据的状态,1:端口接收到数据,2:端口发送数据完成 
#define S_RECEIVE	 0x01	//端口接收到一个数据包 
#define S_TRANSMITOK 0x02	//端口发送一个数据包完成 

/***************----- 端口数据缓冲区 -----***************/
unsigned char Socket0_Rx_Buffer[512];	//端口0接收数据缓冲区 
unsigned char Socket0_Tx_Buffer[256];	//端口0发送数据缓冲区

unsigned char Socket1_Rx_Buffer[2048];	//端口接收数据缓冲区 
unsigned char Socket1_Tx_Buffer[128];	//端口发送数据缓冲区 

unsigned char Socket2_Rx_Buffer[128];	//端口接收数据缓冲区 

unsigned char Socket3_Rx_Buffer[128];	//端口接收数据缓冲区 

unsigned char W5500_Interrupt=0;	//W5500中断标志(0:无中断,1:有中断)

/*******************************************************************************
* 函数名  : W5500_GPIO_Configuration
* 描述    : W5500 GPIO初始化配置
* 输入    : 无
* 输出    : 无
* 返回值  : 无
* 说明    : 无
*******************************************************************************/
void W5500_GPIO_Configuration(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	EXTI_InitTypeDef  EXTI_InitStructure;	

	/* W5500_RST引脚初始化配置(PB10) */
	GPIO_InitStructure.GPIO_Pin  = W5500_RST;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_10MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(W5500_RST_PORT, &GPIO_InitStructure);
	GPIO_ResetBits(W5500_RST_PORT, W5500_RST);
	
	/* W5500_INT引脚初始化配置(PB11) */	
	GPIO_InitStructure.GPIO_Pin = W5500_INT;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(W5500_INT_PORT, &GPIO_InitStructure);
		
	/* Connect EXTI Line11 to PB11 */
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource11);

	/* PC4 as W5500 interrupt input */
	EXTI_InitStructure.EXTI_Line = EXTI_Line11;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);
}

/*******************************************************************************
* 函数名  : W5500_NVIC_Configuration
* 描述    : W5500 接收引脚中断优先级设置
* 输入    : 无
* 输出    : 无
* 返回值  : 无
* 说明    : 无
*******************************************************************************/
void W5500_NVIC_Configuration(void)
{
  	NVIC_InitTypeDef NVIC_InitStructure;

	/* Enable the EXTI11 Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

/*******************************************************************************
* 函数名  : EXTI11_IRQHandler
* 描述    : 中断线11中断服务函数(W5500 INT引脚中断服务函数)
* 输入    : 无
* 输出    : 无
* 返回值  : 无
* 说明    : 无
*******************************************************************************/
void EXTI15_10_IRQHandler(void)
{
	if(EXTI_GetITStatus(EXTI_Line11) != RESET)
	{
		EXTI_ClearITPendingBit(EXTI_Line11);
		W5500_Interrupt=1;	
	}
}

/*******************************************************************************
* 函数名  : SPI_Configuration
* 描述    : W5500 SPI初始化配置(STM32 SPI2)
* 输入    : 无
* 输出    : 无
* 返回值  : 无
* 说明    : 无
*******************************************************************************/
void SPI_Configuration(void)
{
	GPIO_InitTypeDef 	GPIO_InitStructure;
	SPI_InitTypeDef   	SPI_InitStructure;	   

	/* 初始化SCK、MISO、MOSI引脚 */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	GPIO_SetBits(GPIOB,GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15);

	/* 初始化CS引脚 */
	GPIO_InitStructure.GPIO_Pin = W5500_SCS;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_PP;
	GPIO_Init(W5500_SCS_PORT, &GPIO_InitStructure);
	GPIO_SetBits(W5500_SCS_PORT, W5500_SCS);

	/* 初始化配置STM32 SPI2 */
	SPI_InitStructure.SPI_Direction=SPI_Direction_2Lines_FullDuplex;	//SPI设置为双线双向全双工
	SPI_InitStructure.SPI_Mode=SPI_Mode_Master;							//设置为主SPI
	SPI_InitStructure.SPI_DataSize=SPI_DataSize_8b;						//SPI发送接收8位帧结构
	SPI_InitStructure.SPI_CPOL=SPI_CPOL_Low;							//时钟悬空低
	SPI_InitStructure.SPI_CPHA=SPI_CPHA_1Edge;							//数据捕获于第1个时钟沿
	SPI_InitStructure.SPI_NSS=SPI_NSS_Soft;								//NSS由外部管脚管理
	SPI_InitStructure.SPI_BaudRatePrescaler=SPI_BaudRatePrescaler_2;	//波特率预分频值为2
	SPI_InitStructure.SPI_FirstBit=SPI_FirstBit_MSB;					//数据传输从MSB位开始
	SPI_InitStructure.SPI_CRCPolynomial=7;								//CRC多项式为7
	SPI_Init(SPI2,&SPI_InitStructure);									//根据SPI_InitStruct中指定的参数初始化外设SPI2寄存器

	SPI_Cmd(SPI2,ENABLE);	//STM32使能SPI2
}

/*******************************************************************************
* 函数名  : SPI2_Send_Byte
* 描述    : SPI2发送1个字节数据
* 输入    : dat:待发送的数据
* 输出    : 无
* 返回值  : 无
* 说明    : 无
*******************************************************************************/
void SPI2_Send_Byte(unsigned char dat)
{
	SPI_I2S_SendData(SPI2,dat);//写1个字节数据
	while(SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET);//等待数据寄存器空
}

/*******************************************************************************
* 函数名  : SPI2_Send_Short
* 描述    : SPI2发送2个字节数据(16位)
* 输入    : dat:待发送的16位数据
* 输出    : 无
* 返回值  : 无
* 说明    : 无
*******************************************************************************/
void SPI2_Send_Short(unsigned short dat)
{
	SPI2_Send_Byte(dat/256);//写数据高位
	SPI2_Send_Byte(dat);	//写数据低位
}

/*******************************************************************************
* 函数名  : Write_W5500_1Byte
* 描述    : 通过SPI2向指定地址寄存器写1个字节数据
* 输入    : reg:16位寄存器地址,dat:待写入的数据
* 输出    : 无
* 返回值  : 无
* 说明    : 无
*******************************************************************************/
void Write_W5500_1Byte(unsigned short reg, unsigned char dat)
{
	GPIO_ResetBits(W5500_SCS_PORT, W5500_SCS);//置W5500的SCS为低电平

	SPI2_Send_Short(reg);//通过SPI2写16位寄存器地址
	SPI2_Send_Byte(FDM1|RWB_WRITE|COMMON_R);//通过SPI2写控制字节,1个字节数据长度,写数据,选择通用寄存器
	SPI2_Send_Byte(dat);//写1个字节数据

	GPIO_SetBits(W5500_SCS_PORT, W5500_SCS); //置W5500的SCS为高电平
}

/*******************************************************************************
* 函数名  : Write_W5500_2Byte
* 描述    : 通过SPI2向指定地址寄存器写2个字节数据
* 输入    : reg:16位寄存器地址,dat:16位待写入的数据(2个字节)
* 输出    : 无
* 返回值  : 无
* 说明    : 无
*******************************************************************************/
void Write_W5500_2Byte(unsigned short reg, unsigned short dat)
{
	GPIO_ResetBits(W5500_SCS_PORT, W5500_SCS);//置W5500的SCS为低电平
		
	SPI2_Send_Short(reg);//通过SPI2写16位寄存器地址
	SPI2_Send_Byte(FDM2|RWB_WRITE|COMMON_R);//通过SPI2写控制字节,2个字节数据长度,写数据,选择通用寄存器
	SPI2_Send_Short(dat);//写16位数据

	GPIO_SetBits(W5500_SCS_PORT, W5500_SCS); //置W5500的SCS为高电平
}

/*******************************************************************************
* 函数名  : Write_W5500_nByte
* 描述    : 通过SPI2向指定地址寄存器写n个字节数据
* 输入    : reg:16位寄存器地址,*dat_ptr:待写入数据缓冲区指针,size:待写入的数据长度
* 输出    : 无
* 返回值  : 无
* 说明    : 无
*******************************************************************************/
void Write_W5500_nByte(unsigned short reg, unsigned char *dat_ptr, unsigned short size)
{
	unsigned short i;

	GPIO_ResetBits(W5500_SCS_PORT, W5500_SCS);//置W5500的SCS为低电平	
		
	SPI2_Send_Short(reg);//通过SPI2写16位寄存器地址
	SPI2_Send_Byte(VDM|RWB_WRITE|COMMON_R);//通过SPI2写控制字节,N个字节数据长度,写数据,选择通用寄存器

	for(i=0;i<size;i++)//循环将缓冲区的size个字节数据写入W5500
	{
		SPI2_Send_Byte(*dat_ptr++);//写一个字节数据
	}

	GPIO_SetBits(W5500_SCS_PORT, W5500_SCS); //置W5500的SCS为高电平
}

/*******************************************************************************
* 函数名  : Write_W5500_SOCK_1Byte
* 描述    : 通过SPI2向指定端口寄存器写1个字节数据
* 输入    : s:端口号,reg:16位寄存器地址,dat:待写入的数据
* 输出    : 无
* 返回值  : 无
* 说明    : 无
*******************************************************************************/
void Write_W5500_SOCK_1Byte(SOCKET s, unsigned short reg, unsigned char dat)
{
	GPIO_ResetBits(W5500_SCS_PORT, W5500_SCS);//置W5500的SCS为低电平	
		
	SPI2_Send_Short(reg);//通过SPI2写16位寄存器地址
	SPI2_Send_Byte(FDM1|RWB_WRITE|(s*0x20+0x08));//通过SPI2写控制字节,1个字节数据长度,写数据,选择端口s的寄存器
	SPI2_Send_Byte(dat);//写1个字节数据

	GPIO_SetBits(W5500_SCS_PORT, W5500_SCS); //置W5500的SCS为高电平
}

/*******************************************************************************
* 函数名  : Write_W5500_SOCK_2Byte
* 描述    : 通过SPI2向指定端口寄存器写2个字节数据
* 输入    : s:端口号,reg:16位寄存器地址,dat:16位待写入的数据(2个字节)
* 输出    : 无
* 返回值  : 无
* 说明    : 无
*******************************************************************************/
void Write_W5500_SOCK_2Byte(SOCKET s, unsigned short reg, unsigned short dat)
{
	GPIO_ResetBits(W5500_SCS_PORT, W5500_SCS);//置W5500的SCS为低电平
			
	SPI2_Send_Short(reg);//通过SPI2写16位寄存器地址
	SPI2_Send_Byte(FDM2|RWB_WRITE|(s*0x20+0x08));//通过SPI2写控制字节,2个字节数据长度,写数据,选择端口s的寄存器
	SPI2_Send_Short(dat);//写16位数据

	GPIO_SetBits(W5500_SCS_PORT, W5500_SCS); //置W5500的SCS为高电平
}

/*******************************************************************************
* 函数名  : Write_W5500_SOCK_4Byte
* 描述    : 通过SPI2向指定端口寄存器写4个字节数据
* 输入    : s:端口号,reg:16位寄存器地址,*dat_ptr:待写入的4个字节缓冲区指针
* 输出    : 无
* 返回值  : 无
* 说明    : 无
*******************************************************************************/
void Write_W5500_SOCK_4Byte(SOCKET s, unsigned short reg, unsigned char *dat_ptr)
{
	GPIO_ResetBits(W5500_SCS_PORT, W5500_SCS);//置W5500的SCS为低电平
			
	SPI2_Send_Short(reg);//通过SPI2写16位寄存器地址
	SPI2_Send_Byte(FDM4|RWB_WRITE|(s*0x20+0x08));//通过SPI2写控制字节,4个字节数据长度,写数据,选择端口s的寄存器

	SPI2_Send_Byte(*dat_ptr++);//写第1个字节数据
	SPI2_Send_Byte(*dat_ptr++);//写第2个字节数据
	SPI2_Send_Byte(*dat_ptr++);//写第3个字节数据
	SPI2_Send_Byte(*dat_ptr++);//写第4个字节数据

	GPIO_SetBits(W5500_SCS_PORT, W5500_SCS); //置W5500的SCS为高电平
}

/*******************************************************************************
* 函数名  : Read_W5500_1Byte
* 描述    : 读W5500指定地址寄存器的1个字节数据
* 输入    : reg:16位寄存器地址
* 输出    : 无
* 返回值  : 读取到寄存器的1个字节数据
* 说明    : 无
*******************************************************************************/
unsigned char Read_W5500_1Byte(unsigned short reg)
{
	unsigned char i;

	GPIO_ResetBits(W5500_SCS_PORT, W5500_SCS);//置W5500的SCS为低电平
			
	SPI2_Send_Short(reg);//通过SPI2写16位寄存器地址
	SPI2_Send_Byte(FDM1|RWB_READ|COMMON_R);//通过SPI2写控制字节,1个字节数据长度,读数据,选择通用寄存器

	i=SPI_I2S_ReceiveData(SPI2);
	SPI2_Send_Byte(0x00);//发送一个哑数据
	i=SPI_I2S_ReceiveData(SPI2);//读取1个字节数据

	GPIO_SetBits(W5500_SCS_PORT, W5500_SCS);//置W5500的SCS为高电平
	return i;//返回读取到的寄存器数据
}

/*******************************************************************************
* 函数名  : Read_W5500_SOCK_1Byte
* 描述    : 读W5500指定端口寄存器的1个字节数据
* 输入    : s:端口号,reg:16位寄存器地址
* 输出    : 无
* 返回值  : 读取到寄存器的1个字节数据
* 说明    : 无
*******************************************************************************/
unsigned char Read_W5500_SOCK_1Byte(SOCKET s, unsigned short reg)
{
	unsigned char i;

	GPIO_ResetBits(W5500_SCS_PORT, W5500_SCS);//置W5500的SCS为低电平
			
	SPI2_Send_Short(reg);//通过SPI2写16位寄存器地址
	SPI2_Send_Byte(FDM1|RWB_READ|(s*0x20+0x08));//通过SPI2写控制字节,1个字节数据长度,读数据,选择端口s的寄存器

	i=SPI_I2S_ReceiveData(SPI2);
	SPI2_Send_Byte(0x00);//发送一个哑数据
	i=SPI_I2S_ReceiveData(SPI2);//读取1个字节数据

	GPIO_SetBits(W5500_SCS_PORT, W5500_SCS);//置W5500的SCS为高电平
	return i;//返回读取到的寄存器数据
}

/*******************************************************************************
* 函数名  : Read_W5500_SOCK_2Byte
* 描述    : 读W5500指定端口寄存器的2个字节数据
* 输入    : s:端口号,reg:16位寄存器地址
* 输出    : 无
* 返回值  : 读取到寄存器的2个字节数据(16位)
* 说明    : 无
*******************************************************************************/
unsigned short Read_W5500_SOCK_2Byte(SOCKET s, unsigned short reg)
{
	unsigned short i;

	GPIO_ResetBits(W5500_SCS_PORT, W5500_SCS);//置W5500的SCS为低电平
			
	SPI2_Send_Short(reg);//通过SPI2写16位寄存器地址
	SPI2_Send_Byte(FDM2|RWB_READ|(s*0x20+0x08));//通过SPI2写控制字节,2个字节数据长度,读数据,选择端口s的寄存器

	i=SPI_I2S_ReceiveData(SPI2);
	SPI2_Send_Byte(0x00);//发送一个哑数据
	i=SPI_I2S_ReceiveData(SPI2);//读取高位数据
	SPI2_Send_Byte(0x00);//发送一个哑数据
	i*=256;
	i+=SPI_I2S_ReceiveData(SPI2);//读取低位数据

	GPIO_SetBits(W5500_SCS_PORT, W5500_SCS);//置W5500的SCS为高电平
	return i;//返回读取到的寄存器数据
}

/*******************************************************************************
* 函数名  : Read_SOCK_Data_Buffer
* 描述    : 从W5500接收数据缓冲区中读取数据
* 输入    : s:端口号,*dat_ptr:数据保存缓冲区指针
* 输出    : 无
* 返回值  : 读取到的数据长度,rx_size个字节
* 说明    : 无
*******************************************************************************/
unsigned short Read_SOCK_Data_Buffer(SOCKET s, unsigned char *dat_ptr)
{
	unsigned short rx_size;
	unsigned short offset, offset1;
	unsigned short i;
	unsigned char j;

	rx_size=Read_W5500_SOCK_2Byte(s,Sn_RX_RSR);
	if(rx_size==0) return 0;//没接收到数据则返回
	if(rx_size>1460) rx_size=1460;

	offset=Read_W5500_SOCK_2Byte(s,Sn_RX_RD);
	offset1=offset;
	offset&=(S_RX_SIZE-1);//计算实际的物理地址

	GPIO_ResetBits(W5500_SCS_PORT, W5500_SCS);//置W5500的SCS为低电平

	SPI2_Send_Short(offset);//写16位地址
	SPI2_Send_Byte(VDM|RWB_READ|(s*0x20+0x18));//写控制字节,N个字节数据长度,读数据,选择端口s的寄存器
	j=SPI_I2S_ReceiveData(SPI2);
	
	if((offset+rx_size)<S_RX_SIZE)//如果最大地址未超过W5500接收缓冲区寄存器的最大地址
	{
		for(i=0;i<rx_size;i++)//循环读取rx_size个字节数据
		{
			SPI2_Send_Byte(0x00);//发送一个哑数据
			j=SPI_I2S_ReceiveData(SPI2);//读取1个字节数据
			*dat_ptr=j;//将读取到的数据保存到数据保存缓冲区
			dat_ptr++;//数据保存缓冲区指针地址自增1
		}
	}
	else//如果最大地址超过W5500接收缓冲区寄存器的最大地址
	{
		offset=S_RX_SIZE-offset;
		for(i=0;i<offset;i++)//循环读取出前offset个字节数据
		{
			SPI2_Send_Byte(0x00);//发送一个哑数据
			j=SPI_I2S_ReceiveData(SPI2);//读取1个字节数据
			*dat_ptr=j;//将读取到的数据保存到数据保存缓冲区
			dat_ptr++;//数据保存缓冲区指针地址自增1
		}
		GPIO_SetBits(W5500_SCS_PORT, W5500_SCS); //置W5500的SCS为高电平

		GPIO_ResetBits(W5500_SCS_PORT, W5500_SCS);//置W5500的SCS为低电平

		SPI2_Send_Short(0x00);//写16位地址
		SPI2_Send_Byte(VDM|RWB_READ|(s*0x20+0x18));//写控制字节,N个字节数据长度,读数据,选择端口s的寄存器
		j=SPI_I2S_ReceiveData(SPI2);

		for(;i<rx_size;i++)//循环读取后rx_size-offset个字节数据
		{
			SPI2_Send_Byte(0x00);//发送一个哑数据
			j=SPI_I2S_ReceiveData(SPI2);//读取1个字节数据
			*dat_ptr=j;//将读取到的数据保存到数据保存缓冲区
			dat_ptr++;//数据保存缓冲区指针地址自增1
		}
	}
	GPIO_SetBits(W5500_SCS_PORT, W5500_SCS); //置W5500的SCS为高电平

	offset1+=rx_size;//更新实际物理地址,即下次读取接收到的数据的起始地址
	Write_W5500_SOCK_2Byte(s, Sn_RX_RD, offset1);
	Write_W5500_SOCK_1Byte(s, Sn_CR, RECV);//发送启动接收命令
	return rx_size;//返回接收到数据的长度
}

/*******************************************************************************
* 函数名  : Write_SOCK_Data_Buffer
* 描述    : 将数据写入W5500的数据发送缓冲区
* 输入    : s:端口号,*dat_ptr:数据保存缓冲区指针,size:待写入数据的长度
* 输出    : 无
* 返回值  : 无
* 说明    : 无
*******************************************************************************/
void Write_SOCK_Data_Buffer(SOCKET s, unsigned char *dat_ptr, uint8_t size)
{
	unsigned short offset,offset1;
	unsigned short i;

	//如果是UDP模式,可以在此设置目的主机的IP和端口号
	if((Read_W5500_SOCK_1Byte(s,Sn_MR)&0x0f) != SOCK_UDP)//如果Socket打开失败
	{	
			if(s==3)
			{
				Write_W5500_SOCK_4Byte(s, Sn_DIPR, NTP_DIPR);//设置目的主机IP  		
				Write_W5500_SOCK_2Byte(s, Sn_DPORTR, NTP_DPORT[0]*256+NTP_DPORT[1]);//设置目的主机端口号	
			}
			else
			{
				Write_W5500_SOCK_4Byte(s, Sn_DIPR, UDP_DIPR);//设置目的主机IP  		
				Write_W5500_SOCK_2Byte(s, Sn_DPORTR, UDP_DPORT[0]*256+UDP_DPORT[1]);//设置目的主机端口号	
			}				
	}

	offset=Read_W5500_SOCK_2Byte(s,Sn_TX_WR);
	offset1=offset;
	offset&=(S_TX_SIZE-1);//计算实际的物理地址

	GPIO_ResetBits(W5500_SCS_PORT, W5500_SCS);//置W5500的SCS为低电平

	SPI2_Send_Short(offset);//写16位地址
	SPI2_Send_Byte(VDM|RWB_WRITE|(s*0x20+0x10));//写控制字节,N个字节数据长度,写数据,选择端口s的寄存器

	if((offset+size)<S_TX_SIZE)//如果最大地址未超过W5500发送缓冲区寄存器的最大地址
	{
		for(i=0;i<size;i++)//循环写入size个字节数据
		{
			SPI2_Send_Byte(*dat_ptr++);//写入一个字节的数据		
		}
	}
	else//如果最大地址超过W5500发送缓冲区寄存器的最大地址
	{
		offset=S_TX_SIZE-offset;
		for(i=0;i<offset;i++)//循环写入前offset个字节数据
		{
			SPI2_Send_Byte(*dat_ptr++);//写入一个字节的数据
		}
		GPIO_SetBits(W5500_SCS_PORT, W5500_SCS); //置W5500的SCS为高电平

		GPIO_ResetBits(W5500_SCS_PORT, W5500_SCS);//置W5500的SCS为低电平

		SPI2_Send_Short(0x00);//写16位地址
		SPI2_Send_Byte(VDM|RWB_WRITE|(s*0x20+0x10));//写控制字节,N个字节数据长度,写数据,选择端口s的寄存器

		for(;i<size;i++)//循环写入size-offset个字节数据
		{
			SPI2_Send_Byte(*dat_ptr++);//写入一个字节的数据
		}
	}
	GPIO_SetBits(W5500_SCS_PORT, W5500_SCS); //置W5500的SCS为高电平

	offset1+=size;//更新实际物理地址,即下次写待发送数据到发送数据缓冲区的起始地址
	Write_W5500_SOCK_2Byte(s, Sn_TX_WR, offset1);
	Write_W5500_SOCK_1Byte(s, Sn_CR, SEND);//发送启动发送命令				
}

/*******************************************************************************
* 函数名  : SOCKN_Data_Send
* 描述    : 将数据写入W5500的数据发送缓冲区,发送内容为char类型
* 输入    : s:端口号,*dat_ptr:数据保存缓冲区指针,size:待写入数据的长度
* 输出    : 无
* 返回值  : 无
* 说明    : 无
*******************************************************************************/
void Socketn_Char_Send(SOCKET s,char *dat_ptr, uint8_t size)
{
	unsigned short offset,offset1;
	unsigned short i;

	//如果是UDP模式,可以在此设置目的主机的IP和端口号
	if((Read_W5500_SOCK_1Byte(s,Sn_MR)&0x0f) != SOCK_UDP)//如果Socket打开失败
	{		
			if(s==3)
			{
				Write_W5500_SOCK_4Byte(s, Sn_DIPR, NTP_DIPR);//设置目的主机IP  		
				Write_W5500_SOCK_2Byte(s, Sn_DPORTR, NTP_DPORT[0]*256+NTP_DPORT[1]);//设置目的主机端口号	
			}
			else
			{
				Write_W5500_SOCK_4Byte(s, Sn_DIPR, UDP_DIPR);//设置目的主机IP  		
				Write_W5500_SOCK_2Byte(s, Sn_DPORTR, UDP_DPORT[0]*256+UDP_DPORT[1]);//设置目的主机端口号	
			}				
	}

	offset=Read_W5500_SOCK_2Byte(s,Sn_TX_WR);
	offset1=offset;
	offset&=(S_TX_SIZE-1);//计算实际的物理地址

	GPIO_ResetBits(W5500_SCS_PORT, W5500_SCS);//置W5500的SCS为低电平

	SPI2_Send_Short(offset);//写16位地址
	SPI2_Send_Byte(VDM|RWB_WRITE|(s*0x20+0x10));//写控制字节,N个字节数据长度,写数据,选择端口s的寄存器

	if((offset+size)<S_TX_SIZE)//如果最大地址未超过W5500发送缓冲区寄存器的最大地址
	{
		for(i=0;i<size;i++)//循环写入size个字节数据
		{
			SPI2_Send_Byte(*dat_ptr++);//写入一个字节的数据		
		}
	}
	else//如果最大地址超过W5500发送缓冲区寄存器的最大地址
	{
		offset=S_TX_SIZE-offset;
		for(i=0;i<offset;i++)//循环写入前offset个字节数据
		{
			SPI2_Send_Byte(*dat_ptr++);//写入一个字节的数据
		}
		GPIO_SetBits(W5500_SCS_PORT, W5500_SCS); //置W5500的SCS为高电平

		GPIO_ResetBits(W5500_SCS_PORT, W5500_SCS);//置W5500的SCS为低电平

		SPI2_Send_Short(0x00);//写16位地址
		SPI2_Send_Byte(VDM|RWB_WRITE|(s*0x20+0x10));//写控制字节,N个字节数据长度,写数据,选择端口s的寄存器

		for(;i<size;i++)//循环写入size-offset个字节数据
		{
			SPI2_Send_Byte(*dat_ptr++);//写入一个字节的数据
		}
	}
	GPIO_SetBits(W5500_SCS_PORT, W5500_SCS); //置W5500的SCS为高电平

	offset1+=size;//更新实际物理地址,即下次写待发送数据到发送数据缓冲区的起始地址
	Write_W5500_SOCK_2Byte(s, Sn_TX_WR, offset1);
	Write_W5500_SOCK_1Byte(s, Sn_CR, SEND);//发送启动发送命令				
}

/*******************************************************************************
* 函数名  : W5500_Hardware_Reset
* 描述    : 硬件复位W5500
* 输入    : 无
* 输出    : 无
* 返回值  : 无
* 说明    : W5500的复位引脚保持低电平至少500us以上,才能重围W5500
*******************************************************************************/
void W5500_Hardware_Reset(void)
{
	GPIO_ResetBits(W5500_RST_PORT, W5500_RST);//复位引脚拉低
	My_Delay_xms(50);
	GPIO_SetBits(W5500_RST_PORT, W5500_RST);//复位引脚拉高
	My_Delay_xms(200);
//	while((Read_W5500_1Byte(PHYCFGR)&LINK)==0);//等待以太网连接完成
}

/*******************************************************************************
* 函数名  : W5500_Init
* 描述    : 初始化W5500寄存器函数
* 输入    : 无
* 输出    : 无
* 返回值  : 无
* 说明    : 在使用W5500之前，先对W5500初始化
*******************************************************************************/
void W5500_Init(void)
{
	u8 i=0;

	Write_W5500_1Byte(MR, RST);//软件复位W5500,置1有效,复位后自动清0
	My_Delay_xms(10);//延时10ms,自己定义该函数

	//设置网关(Gateway)的IP地址,Gateway_IP为4字节unsigned char数组,自己定义 
	//使用网关可以使通信突破子网的局限，通过网关可以访问到其它子网或进入Internet
	Write_W5500_nByte(GAR, Gateway_IP, 4);
			
	//设置子网掩码(MASK)值,SUB_MASK为4字节unsigned char数组,自己定义
	//子网掩码用于子网运算
	Write_W5500_nByte(SUBR,Sub_Mask,4);		
	
	//设置物理地址,PHY_ADDR为6字节unsigned char数组,自己定义,用于唯一标识网络设备的物理地址值
	//该地址值需要到IEEE申请，按照OUI的规定，前3个字节为厂商代码，后三个字节为产品序号
	//如果自己定义物理地址，注意第一个字节必须为偶数
	Write_W5500_nByte(SHAR,Phy_Addr,6);		

	//设置本机的IP地址,IP_ADDR为4字节unsigned char数组,自己定义
	//注意，网关IP必须与本机IP属于同一个子网，否则本机将无法找到网关
	Write_W5500_nByte(SIPR,IP_Addr,4);		
	
	//设置发送缓冲区和接收缓冲区的大小，参考W5500数据手册
	for(i=0;i<8;i++)
	{
		Write_W5500_SOCK_1Byte(i,Sn_RXBUF_SIZE, 0x02);//Socket Rx memory size=2k
		Write_W5500_SOCK_1Byte(i,Sn_TXBUF_SIZE, 0x02);//Socket Tx mempry size=2k
	}

	//设置重试时间，默认为2000(200ms) 
	//每一单位数值为100微秒,初始化时值设为2000(0x07D0),等于200毫秒
	Write_W5500_2Byte(RTR, 0x07d0);

	//设置重试次数，默认为8次 
	//如果重发的次数超过设定值,则产生超时中断(相关的端口中断寄存器中的Sn_IR 超时位(TIMEOUT)置“1”)
	Write_W5500_1Byte(RCR,8);

	//启动中断，参考W5500数据手册确定自己需要的中断类型
	//IMR_CONFLICT是IP地址冲突异常中断,IMR_UNREACH是UDP通信时，地址无法到达的异常中断
	//其它是Socket事件中断，根据需要添加
  Write_W5500_1Byte(IMR,IM_IR7 | IM_IR6);	//打开IP冲突中断和目的地址不能到达中断
	Write_W5500_1Byte(SIMR,S0_IMR|S1_IMR|S2_IMR|S3_IMR);
	Write_W5500_SOCK_1Byte(0, Sn_IMR, IMR_SENDOK | IMR_TIMEOUT | IMR_RECV | IMR_DISCON | IMR_CON);
	Write_W5500_SOCK_1Byte(1, Sn_IMR, IMR_SENDOK | IMR_TIMEOUT | IMR_RECV | IMR_DISCON | IMR_CON);
	Write_W5500_SOCK_1Byte(2, Sn_IMR, IMR_SENDOK | IMR_TIMEOUT | IMR_RECV | IMR_DISCON | IMR_CON);
	Write_W5500_SOCK_1Byte(3, Sn_IMR, IMR_SENDOK | IMR_TIMEOUT | IMR_RECV | IMR_DISCON | IMR_CON);
	Write_W5500_SOCK_1Byte(0,Sn_KPALVTR,0x01);
}

/*******************************************************************************
* 函数名  : Detect_Gateway
* 描述    : 检查网关服务器
* 输入    : 无
* 输出    : 无
* 返回值  : 成功返回TRUE(0xFF),失败返回FALSE(0x00)
* 说明    : 无
*******************************************************************************/
unsigned char Detect_Gateway(SOCKET s)
{
	unsigned char ip_adde[4];
	ip_adde[0]=IP_Addr[0]+1;
	ip_adde[1]=IP_Addr[1]+1;
	ip_adde[2]=IP_Addr[2]+1;
	ip_adde[3]=IP_Addr[3]+1;

	//检查网关及获取网关的物理地址
	Write_W5500_SOCK_4Byte(s,Sn_DIPR,ip_adde);//向目的地址寄存器写入与本机IP不同的IP值
	Write_W5500_SOCK_1Byte(s,Sn_MR,MR_TCP);//设置socket为TCP协议模式
	Write_W5500_SOCK_1Byte(s,Sn_CR,OPEN);//打开Socket	
	My_Delay_xms(5);//延时5ms 	
	
	if(Read_W5500_SOCK_1Byte(s,Sn_SR) != SOCK_INIT)//如果socket打开失败
	{
		Write_W5500_SOCK_1Byte(s,Sn_CR,CLOSE);//打开不成功,关闭Socket
		return FALSE_W5500;//返回FALSE(0x00)
	}

	Write_W5500_SOCK_1Byte(s,Sn_CR,CONNECT);//设置Socket为Connect模式						

	do
	{
		u8 j=0;
		j=Read_W5500_SOCK_1Byte(s,Sn_IR);//读取Socket0中断标志寄存器
		if(j!=0)
		Write_W5500_SOCK_1Byte(s,Sn_IR,j);
		My_Delay_xms(5);//延时5ms 
		if((j&IR_TIMEOUT) == IR_TIMEOUT)
		{
			return FALSE_W5500;	
		}
		else if(Read_W5500_SOCK_1Byte(s,Sn_DHAR) != 0xff)
		{
			Write_W5500_SOCK_1Byte(s,Sn_CR,CLOSE);//关闭Socket
			return TRUE_W5500;							
		}
	}while(1);
}

/*******************************************************************************
* 函数名  : Socket_Init
* 描述    : 指定Socket(0~7)初始化
* 输入    : s:待初始化的端口
* 输出    : 无
* 返回值  : 无
* 说明    : 无
*******************************************************************************/
void Socket_Init(SOCKET s)
{
	//设置分片长度，参考W5500数据手册，该值可以不修改	
	Write_W5500_SOCK_2Byte(s, Sn_MSSR, 1460);//最大分片字节数=1460(0x5b4)
	//设置指定端口
	switch(s)
	{
		case 0:
			//设置端口0的端口号
			Write_W5500_SOCK_2Byte(s, Sn_PORT, S0_Port[0]*256+S0_Port[1]);				
			break;

		case 1:
			Write_W5500_SOCK_2Byte(s, Sn_PORT, S1_Port[0]*256+S1_Port[1]);
			break;

		case 2:
			Write_W5500_SOCK_2Byte(s, Sn_PORT, S2_Port[0]*256+S2_Port[1]);
			break;

		case 3:
			Write_W5500_SOCK_2Byte(s, Sn_PORT, S3_Port[0]*256+S3_Port[1]);
			break;

		case 4:
			break;

		case 5:
			break;

		case 6:
			break;

		case 7:
			break;

		default:
			break;
	}
}

/*******************************************************************************
* 函数名  : Socket_Connect
* 描述    : 设置指定Socket(0~7)为客户端与远程服务器连接
* 输入    : s:待设定的端口
* 输出    : 无
* 返回值  : 成功返回TRUE(0xFF),失败返回FALSE(0x00)
* 说明    : 当本机Socket工作在客户端模式时,引用该程序,与远程服务器建立连接
*			如果启动连接后出现超时中断，则与服务器连接失败,需要重新调用该程序连接
*			该程序每调用一次,就与服务器产生一次连接
*******************************************************************************/
unsigned char Socket_Connect(SOCKET s)
{
	Write_W5500_SOCK_1Byte(s,Sn_MR,MR_TCP);//设置socket为TCP模式
	Write_W5500_SOCK_1Byte(s,Sn_CR,OPEN);//打开Socket
	My_Delay_xms(5);//延时5ms
	if(Read_W5500_SOCK_1Byte(s,Sn_SR)!=SOCK_INIT)//如果socket打开失败
	{
		Write_W5500_SOCK_1Byte(s,Sn_CR,CLOSE);//打开不成功,关闭Socket
		return FALSE_W5500;//返回FALSE(0x00)
	}
	Write_W5500_SOCK_1Byte(s,Sn_CR,CONNECT);//设置Socket为Connect模式
	return TRUE_W5500;//返回TRUE,设置成功
}

/*******************************************************************************
* 函数名  : Socket_Listen
* 描述    : 设置指定Socket(0~7)作为服务器等待远程主机的连接
* 输入    : s:待设定的端口
* 输出    : 无
* 返回值  : 成功返回TRUE(0xFF),失败返回FALSE(0x00)
* 说明    : 当本机Socket工作在服务器模式时,引用该程序,等等远程主机的连接
*			该程序只调用一次,就使W5500设置为服务器模式
*******************************************************************************/
unsigned char Socket_Listen(SOCKET s)
{
	Write_W5500_SOCK_1Byte(s,Sn_MR,MR_TCP);//设置socket为TCP模式 
	Write_W5500_SOCK_1Byte(s,Sn_CR,OPEN);//打开Socket	
	My_Delay_xms(5);//延时5ms
	if(Read_W5500_SOCK_1Byte(s,Sn_SR)!=SOCK_INIT)//如果socket打开失败
	{
		Write_W5500_SOCK_1Byte(s,Sn_CR,CLOSE);//打开不成功,关闭Socket
		return FALSE_W5500;//返回FALSE(0x00)
	}	
	Write_W5500_SOCK_1Byte(s,Sn_CR,LISTEN);//设置Socket为侦听模式	
	My_Delay_xms(5);//延时5ms
	if(Read_W5500_SOCK_1Byte(s,Sn_SR)!=SOCK_LISTEN)//如果socket设置失败
	{
		Write_W5500_SOCK_1Byte(s,Sn_CR,CLOSE);//设置不成功,关闭Socket
		return FALSE_W5500;//返回FALSE(0x00)
	}

	return TRUE_W5500;

	//至此完成了Socket的打开和设置侦听工作,至于远程客户端是否与它建立连接,则需要等待Socket中断，
	//以判断Socket的连接是否成功。参考W5500数据手册的Socket中断状态
	//在服务器侦听模式不需要设置目的IP和目的端口号
}

/*******************************************************************************
* 函数名  : Socket_UDP
* 描述    : 设置指定Socket(0~7)为UDP模式
* 输入    : s:待设定的端口
* 输出    : 无
* 返回值  : 成功返回TRUE(0xFF),失败返回FALSE(0x00)
* 说明    : 如果Socket工作在UDP模式,引用该程序,在UDP模式下,Socket通信不需要建立连接
*			该程序只调用一次，就使W5500设置为UDP模式
*******************************************************************************/
unsigned char Socket_UDP(SOCKET s)
{
	Write_W5500_SOCK_1Byte(s,Sn_MR,MR_UDP);//设置Socket为UDP模式*/
	Write_W5500_SOCK_1Byte(s,Sn_CR,OPEN);//打开Socket*/
	My_Delay_xms(5);//延时5ms
	if(Read_W5500_SOCK_1Byte(s,Sn_SR)!=SOCK_UDP)//如果Socket打开失败
	{
		Write_W5500_SOCK_1Byte(s,Sn_CR,CLOSE);//打开不成功,关闭Socket
		return FALSE_W5500;//返回FALSE(0x00)
	}
	else
		return TRUE_W5500;

	//至此完成了Socket的打开和UDP模式设置,在这种模式下它不需要与远程主机建立连接
	//因为Socket不需要建立连接,所以在发送数据前都可以设置目的主机IP和目的Socket的端口号
	//如果目的主机IP和目的Socket的端口号是固定的,在运行过程中没有改变,那么也可以在这里设置
}

/*******************************************************************************
* 函数名  : W5500_Interrupt_Process
* 描述    : W5500中断处理程序框架
* 输入    : 无
* 输出    : 无
* 返回值  : 无
* 说明    : 无
*******************************************************************************/
void W5500_Process(void)
{
	unsigned char i,j;
IntDispose:
	W5500_Interrupt=0;//清零中断标志
	i = Read_W5500_1Byte(IR);//读取中断标志寄存器
	Write_W5500_1Byte(IR, (i&0xf0));//回写清除中断标志

	if((i & CONFLICT) == CONFLICT)//IP地址冲突异常处理
	{
		 //自己添加代码
	}

	if((i & UNREACH) == UNREACH)//UDP模式下地址无法到达异常处理
	{
		//自己添加代码
	}
	
	if((i& IMR_TIMEOUT)==IMR_TIMEOUT)
	{
		//保留
	}
	
	i=Read_W5500_1Byte(SIR);//读取端口中断标志寄存器	
	if((i & S0_INT) == S0_INT)//Socket0事件处理 
	{
		j=Read_W5500_SOCK_1Byte(0,Sn_IR);//读取Socket0中断标志寄存器
		Write_W5500_SOCK_1Byte(0,Sn_IR,j);
		if(j&IR_CON)//在TCP模式下,Socket0成功连接 
		{
			S0_State|=S_CONN;//网络连接状态0x02,端口完成连接，可以正常传输数据
		}
		if(j&IR_DISCON)//在TCP模式下Socket断开连接处理
		{
			Write_W5500_SOCK_1Byte(0,Sn_CR,CLOSE);//关闭端口,等待重新打开连接 
			Socket_Init(0);		//指定Socket(0~7)初始化,初始化端口0
			S0_State=0;//网络连接状态0x00,端口连接失败
		}
		if(j&IR_SEND_OK)//Socket0数据发送完成,可以再次启动S_tx_process()函数发送数据 
		{
			S0_Data|=S_TRANSMITOK;//端口发送一个数据包完成 
		}
		if(j&IR_RECV)//Socket接收到数据,可以启动S_rx_process()函数 
		{
			S0_Data|=S_RECEIVE;//端口接收到一个数据包
		}
		if(j&IR_TIMEOUT)//Socket连接或数据传输超时处理 
		{
			Write_W5500_SOCK_1Byte(0,Sn_CR,CLOSE);// 关闭端口,等待重新打开连接
			S0_State=0;//网络连接状态0x00,端口连接失败
		}
	}

	if((i & S1_INT) == S1_INT)//Socket1事件处理 
	{
		j=Read_W5500_SOCK_1Byte(1,Sn_IR);//读取Socket1中断标志寄存器
		Write_W5500_SOCK_1Byte(1,Sn_IR,j);
		if(j&IR_CON)//在TCP模式下,Socket0成功连接 
		{
			S1_State|=S_CONN;//网络连接状态0x02,端口完成连接，可以正常传输数据
		}
		if(j&IR_DISCON)//在TCP模式下Socket断开连接处理
		{
			Write_W5500_SOCK_1Byte(1,Sn_CR,CLOSE);//关闭端口,等待重新打开连接 
			Socket_Init(1);		//指定Socket(0~7)初始化,初始化端口0
			S1_State=0;//网络连接状态0x00,端口连接失败
		}
		if(j&IR_SEND_OK)//Socket1数据发送完成,可以再次启动S_tx_process()函数发送数据 
		{
			S1_Data|=S_TRANSMITOK;//端口发送一个数据包完成 
		}
		if(j&IR_RECV)//Socket接收到数据,可以启动S_rx_process()函数 
		{
			S1_Data|=S_RECEIVE;//端口接收到一个数据包
		}
		if(j&IR_TIMEOUT)//Socket连接或数据传输超时处理 
		{
			Write_W5500_SOCK_1Byte(1,Sn_CR,CLOSE);// 关闭端口,等待重新打开连接
			S1_State=0;//网络连接状态0x00,端口连接失败
		}
	}
	
	if((i & S2_INT) == S2_INT)//Socket2事件处理 
	{
		j=Read_W5500_SOCK_1Byte(2,Sn_IR);//读取Socket2中断标志寄存器
		Write_W5500_SOCK_1Byte(2,Sn_IR,j);
		if(j&IR_CON)//在TCP模式下,Socket2成功连接 
		{
			S2_State|=S_CONN;//网络连接状态0x02,端口完成连接，可以正常传输数据
		}
		if(j&IR_DISCON)//在TCP模式下Socket断开连接处理
		{
			Write_W5500_SOCK_1Byte(2,Sn_CR,CLOSE);//关闭端口,等待重新打开连接 
			Socket_Init(2);		//指定Socket(0~7)初始化,初始化端口0
			S2_State=0;//网络连接状态0x00,端口连接失败
		}
		if(j&IR_SEND_OK)//Socket0数据发送完成,可以再次启动S_tx_process()函数发送数据 
		{
			S2_Data|=S_TRANSMITOK;//端口发送一个数据包完成 
		}
		if(j&IR_RECV)//Socket接收到数据,可以启动S_rx_process()函数 
		{
			S2_Data|=S_RECEIVE;//端口接收到一个数据包
		}
		if(j&IR_TIMEOUT)//Socket连接或数据传输超时处理 
		{
			Write_W5500_SOCK_1Byte(2,Sn_CR,CLOSE);// 关闭端口,等待重新打开连接
			S2_State=0;//网络连接状态0x00,端口连接失败
		}
	}
	
	if((i & S3_INT) == S3_INT)//Socket3事件处理 
	{
		j=Read_W5500_SOCK_1Byte(3,Sn_IR);//读取Socket3中断标志寄存器
		Write_W5500_SOCK_1Byte(3,Sn_IR,j);
		if(j&IR_CON)//在TCP模式下,Socket2成功连接 
		{
			S3_State|=S_CONN;//网络连接状态0x02,端口完成连接，可以正常传输数据
		}
		if(j&IR_DISCON)//在TCP模式下Socket断开连接处理
		{
			Write_W5500_SOCK_1Byte(3,Sn_CR,CLOSE);//关闭端口,等待重新打开连接 
			Socket_Init(3);		//指定Socket(0~7)初始化,初始化端口0
			S3_State=0;//网络连接状态0x00,端口连接失败
		}
		if(j&IR_SEND_OK)//Socket0数据发送完成,可以再次启动S_tx_process()函数发送数据 
		{
			S3_Data|=S_TRANSMITOK;//端口发送一个数据包完成 
		}
		if(j&IR_RECV)//Socket接收到数据,可以启动S_rx_process()函数 
		{
			S3_Data|=S_RECEIVE;//端口接收到一个数据包
		}
		if(j&IR_TIMEOUT)//Socket连接或数据传输超时处理 
		{
			Write_W5500_SOCK_1Byte(3,Sn_CR,CLOSE);// 关闭端口,等待重新打开连接
			S3_State=0;//网络连接状态0x00,端口连接失败
		}
	}
	
		if(Read_W5500_1Byte(SIR) != 0) 
		goto IntDispose;
}

