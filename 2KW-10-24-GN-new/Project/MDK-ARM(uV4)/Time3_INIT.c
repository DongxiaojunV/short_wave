#include "W5500.h"			
#include "main.h"
#include "my_protocol.h"
#include "bsp_uart_fifo.h"
#include "firmware_upgrade.h"
volatile uint32_t Timer4_Counter=0,Timer4_Counter_2=0; //Timer2定时器计数变量(ms)
uint8_t STM32_ID[12];								//储存STM32的id号
/*******************************************************************************
* 函数名  : W5500_Initialization
* 描述    : W5500初始货配置
* 输入    : 无
* 输出    : 无
* 返回值  : 无
* 说明    : 无
*******************************************************************************/
void W5500_Initialization(void)
{
	W5500_Init();		//初始化W5500寄存器函数
	Detect_Gateway(0);	//检查网关服务器 
	Detect_Gateway(1);	//检查网关服务器
	Detect_Gateway(2);	//检查网关服务器
	Detect_Gateway(3);	//检查网关服务器
	Socket_Init(0);		//指定Socket(0~7)初始化,初始化端口0
	Socket_Init(1);		//指定Socket(0~7)初始化,初始化端口1
	Socket_Init(2);		//指定Socket(0~7)初始化,初始化端口2
	Socket_Init(3);		//指定Socket(0~7)初始化,初始化端口3
}

/*******************************************************************************
* 函数名  : Load_Net_Parameters
* 描述    : 装载网络参数
* 输入    : 无
* 输出    : 无
* 返回值  : 无
* 说明    : 网关、掩码、物理地址、本机IP地址、端口号、目的IP地址、目的端口号、端口工作模式
*******************************************************************************/
void Load_Net_Parameters(void)
{
	uint32_t Address=0;
	uint8_t tx_ip[8];
	uint8_t tx_port[4];
	Address=Ip_Port_Addr;
	FLASH_Read(Address,tx_ip,8);//读出IP地址16位
	FLASH_Read(Address+16,tx_port,4);//读出端口号16位

	Sub_Mask[0]=255;//加载子网掩码
	Sub_Mask[1]=255;
	Sub_Mask[2]=255;
	Sub_Mask[3]=0;

	Get_STM32_ID();						//获取stm32ID号
	if(STM32_ID[6]%2!=0)
	{
			STM32_ID[6]+=0x01;
	}
	for(uint8_t i=0;i<6;i++)
	{
		Phy_Addr[i]=STM32_ID[i+6];
	}
	
	if(tx_ip[0]==0xFF||tx_ip[2]==0xFF||tx_ip[4]==0xFF||tx_ip[6]==0xFF||tx_port[0]==0xFF||tx_port[2]==0xFF \
		 || tx_ip[0]==0 || tx_ip[2]==0 || tx_ip[4]==0 || tx_ip[6]==0 || tx_port[0]==0 || tx_port[2]==0)
	{
		IP_Addr[0]=192;//加载本机IP地址
		IP_Addr[1]=168;
		IP_Addr[2]=8;
		IP_Addr[3]=120;

		Gateway_IP[0] = 192;//加载网关参数
		Gateway_IP[1] = 168;
		Gateway_IP[2] = 8;
		Gateway_IP[3] = 1;
		
		S0_Port[0] = 0x04;//加载端口0的端口号1225 
		S0_Port[1] = 0xC9;
	}

	else
	{
		IP_Addr[0]=tx_ip[0];
		IP_Addr[1]=tx_ip[2];
		IP_Addr[2]=tx_ip[4];
		IP_Addr[3]=tx_ip[6];
		
		Gateway_IP[0]=tx_ip[0];
		Gateway_IP[1]=tx_ip[2];
		Gateway_IP[2]=tx_ip[4];
		Gateway_IP[3]=1;
		
		S0_Port[0]=tx_port[0];
		S0_Port[1]=tx_port[2];
	}
	
	S1_Port[0] = 0x04;//加载端口1的端口号1228
	S1_Port[1] = 0xCC;
	
	S2_Port[0] = 0x04;//加载端口2的端口号1229
	S2_Port[1] = 0xCD;
	
	S3_Port[0] = 0x04;//加载端口3的端口号1230
	S3_Port[1] = 0xCE;
	
	NTP_DIPR[0]=192;
	NTP_DIPR[1]=168;
	NTP_DIPR[2]=8;
	NTP_DIPR[3]=5;

	NTP_DPORT[0]=0;
	NTP_DPORT[1]=0x7B;	//端口号123
	
	S0_Mode=TCP_SERVER;	//加载端口0的工作模式,TCP服务器模式
	S1_Mode=TCP_SERVER;	//加载端口1的工作模式,TCP服务器模式
	S2_Mode=UDP_MODE;		//加载端口2的工作模式,UDP服务器模式
	S3_Mode=UDP_MODE;		//加载端口3的工作模式,UDP服务器模式
}

/*******************************************************************************
* 函数名  : W5500_Socket_Set
* 描述    : W5500端口初始化配置
* 输入    : 无
* 输出    : 无
* 返回值  : 无
* 说明    : 分别设置4个端口,根据端口工作模式,将端口置于TCP服务器、TCP客户端或UDP模式.
*			从端口状态字节Socket_State可以判断端口的工作情况
*******************************************************************************/
void W5500_Socket_Set(void)
{
	if(S0_State==0)//端口0初始化配置
	{
		if(S0_Mode==TCP_SERVER)//TCP服务器模式 
		{
			if(Socket_Listen(0)==TRUE_W5500)
				S0_State=S_INIT;
			else
				S0_State=0;
		}
		else if(S0_Mode==TCP_CLIENT)//TCP客户端模式 
		{
			if(Socket_Connect(0)==TRUE_W5500)
				S0_State=S_INIT;
			else
				S0_State=0;
		}
		else//UDP模式 
		{
			if(Socket_UDP(0)==TRUE_W5500)
				S0_State=S_INIT|S_CONN;
			else
				S0_State=0;
		}
	}
	
	if(S1_State==0)//端口1初始化配置
	{
		if(S1_Mode==TCP_SERVER)//TCP服务器模式 
		{
			if(Socket_Listen(1)==TRUE_W5500)
				S1_State=S_INIT;
			else
				S1_State=0;
		}
		else if(S1_Mode==TCP_CLIENT)//TCP客户端模式 
		{
			if(Socket_Connect(1)==TRUE_W5500)
				S1_State=S_INIT;
			else
				S1_State=0;
		}
		else//UDP模式 
		{
			if(Socket_UDP(1)==TRUE_W5500)
				S1_State=S_INIT|S_CONN;
			else
				S1_State=0;
		}
	}
	
	if(S2_State==0)//端口2初始化配置
	{
		if(S2_Mode==TCP_SERVER)//TCP服务器模式 
		{
			if(Socket_Listen(2)==TRUE_W5500)
				S2_State=S_INIT;
			else
				S2_State=0;
		}
		else if(S2_Mode==TCP_CLIENT)//TCP客户端模式 
		{
			if(Socket_Connect(2)==TRUE_W5500)
				S2_State=S_INIT;
			else
				S2_State=0;
		}
		else//UDP模式 
		{
			if(Socket_UDP(2)==TRUE_W5500)
				S2_State=S_INIT|S_CONN;
			else
				S2_State=0;
		}
	}
	
	if(S3_State==0)//端口2初始化配置
	{
		if(S3_Mode==TCP_SERVER)//TCP服务器模式 
		{
			if(Socket_Listen(3)==TRUE_W5500)
				S3_State=S_INIT;
			else
				S3_State=0;
		}
		else if(S3_Mode==TCP_CLIENT)//TCP客户端模式 
		{
			if(Socket_Connect(3)==TRUE_W5500)
				S3_State=S_INIT;
			else
				S3_State=0;
		}
		else//UDP模式 
		{
			if(Socket_UDP(3)==TRUE_W5500)
				S3_State=S_INIT|S_CONN;
			else
				S3_State=0;
		}
	}
	
}

/*读取STM32ID号*/
void Get_STM32_ID(void)
{
    u32 temp0,temp1,temp2;
temp0 = *(__IO u32*)(0x1FFFF7E8);    //产品唯一身份标识寄存器（96位）
    temp1 = *(__IO u32*)(0x1FFFF7EC);
temp2 = *(__IO u32*)(0x1FFFF7F0);
                                  
//ID码地址： 0x1FFFF7E8   0x1FFFF7EC  0x1FFFF7F0 ，只需要读取这个地址中的数据就可以了。
 
    STM32_ID[0] = (u8)(temp0 & 0x000000FF);
    STM32_ID[1] = (u8)((temp0 & 0x0000FF00)>>8);
    STM32_ID[2] = (u8)((temp0 & 0x00FF0000)>>16);
    STM32_ID[3] = (u8)((temp0 & 0xFF000000)>>24);
    STM32_ID[4] = (u8)(temp1 & 0x000000FF);
    STM32_ID[5] = (u8)((temp1 & 0x0000FF00)>>8);
    STM32_ID[6] = (u8)((temp1 & 0x00FF0000)>>16);
    STM32_ID[7] = (u8)((temp1 & 0xFF000000)>>24);
    STM32_ID[8] = (u8)(temp2 & 0x000000FF);
    STM32_ID[9] = (u8)((temp2 & 0x0000FF00)>>8);
    STM32_ID[10] = (u8)((temp2 & 0x00FF0000)>>16);
    STM32_ID[11] = (u8)((temp2 & 0xFF000000)>>24);         
}

/*******************************************************************************
* 函数名  : Process_Socket_Data
* 描述    : W5500接收并发送接收到的数据
* 输入    : s:端口号 msg:数据数组  len:数据长度
* 输出    : 无
* 返回值  : 无
* 说明    : 无
*******************************************************************************/
void Process_Socket_Data(SOCKET s,uint8_t * msg, uint8_t len)
{
    uint16_t i=0;

    for(i=0; i<len; i++)
	{
      Socket0_Tx_Buffer[i] = msg[i];
	}
	
	Write_SOCK_Data_Buffer(s, Socket0_Tx_Buffer, len);
}

/*******************************************************************************
* 函数名  : RCC_Configuration
* 描述    : 时钟使能
* 输入    : 无
* 输出    : 无
* 返回值  : 无
* 说明    : 无
*******************************************************************************/
void RCC_Configuration(void)
{
  	/* Enable I2C1 and I2C1 clock */
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2 | RCC_APB1Periph_TIM4, ENABLE);

  	/* Enable GPIOA GPIOB SPI2 and USART1 clocks */
  	RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE); 
					
}

/*******************************************************************************
* 函数名  : NVIC_Configuration
* 描述    : STM32中断向量表配配置
* 输入    : 无
* 输出    : 无
* 返回值  : 无
* 说明    : 设置KEY1(PC11)的中断优先组
*******************************************************************************/

void NVIC_Configuration(void)
{
	/*定时器中断配置*/
	NVIC_InitTypeDef NVIC_InitStructure;						//定义NVIC初始化结构体

  	/* Set the Vector Table base location at 0x08000000 */
//  	NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x0);
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);				//设置中断优先级组为1，优先组(可设0～4位)
	
	NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;				//设置中断向量号
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;	//设置抢先优先级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;			//设置响应优先级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;				//使能NVIC
	NVIC_Init(&NVIC_InitStructure);
	
	W5500_NVIC_Configuration(); //W5500中断配置
}

/*******************************************************************************
* 函数名  : Timer3_Init_Config
* 描述    : Timer2初始化配置
* 输入    : 无
* 输出    : 无
* 返回    : 无 
* 说明    : 无
*******************************************************************************/

void Timer4_Init_Config(void)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	
	TIM_TimeBaseStructure.TIM_Period = 9;						//设置在下一个更新事件装入活动的自动重装载寄存器周期的值(计数到10为1ms)
	TIM_TimeBaseStructure.TIM_Prescaler = 7199;					//设置用来作为TIMx时钟频率除数的预分频值(10KHz的计数频率)
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;		//设置时钟分割:TDTS = TIM_CKD_DIV1
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;	//TIM向上计数模式
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);				//根据TIM_TimeBaseInitStruct中指定的参数初始化TIMx的时间基数单位
	 
	TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE ); 				//使能TIM4指定的中断
	
	TIM_Cmd(TIM4, ENABLE);  									//使能TIMx外设
}

/*******************************************************************************
* 函数名  : TIM4_IRQHandler
* 描述    : 定时器4中断断服务函数
* 输入    : 无
* 输出    : 无
* 返回    : 无 
* 说明    : 无
*******************************************************************************/
void TIM4_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM4, TIM_IT_Update) == SET)
	{
		TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
		Timer4_Counter++;
		Timer4_Counter_2++;
		if(Timer4_Counter>=0xEFFFFFFF)
		{
			Timer4_Counter=0;
		}
		if(Timer4_Counter_2>=0xEFFFFFFF)
		{
			Timer4_Counter_2=0;
		}
	}
}

/*******************************************************************************
* 函数名  : System_Initialization
* 描述    : STM32系统初始化函数(初始化STM32时钟及外设)
* 输入    : 无
* 输出    : 无
* 返回    : 无 
* 说明    : 无
*******************************************************************************/
void System_Initialization(void)
{
	RCC_Configuration();
	Timer4_Init_Config();		//定时器初始化
	NVIC_Configuration();		//中断配置
	SPI_Configuration();		//W5500 SPI初始化配置(STM32 SPI2)
	W5500_GPIO_Configuration();	//W5500 GPIO初始化配置	
	
}

/*延时函数*/
void My_Delay_xms(uint32_t ms)
{
    uint32_t temp=Timer4_Counter;
	while( Timer4_Counter <= temp  + ms );
	Timer4_Counter=0;
}

void My_Delay_ms(uint32_t ms)
{
    uint32_t temp=Timer4_Counter_2;
	while( Timer4_Counter_2 <= temp  + ms );
	Timer4_Counter_2=0;
}

