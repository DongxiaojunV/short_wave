#include "W5500.h"			
#include "main.h"
#include "my_protocol.h"
#include "bsp_uart_fifo.h"
#include "firmware_upgrade.h"
volatile uint32_t Timer4_Counter=0,Timer4_Counter_2=0; //Timer2��ʱ����������(ms)
uint8_t STM32_ID[12];								//����STM32��id��
/*******************************************************************************
* ������  : W5500_Initialization
* ����    : W5500��ʼ������
* ����    : ��
* ���    : ��
* ����ֵ  : ��
* ˵��    : ��
*******************************************************************************/
void W5500_Initialization(void)
{
	W5500_Init();		//��ʼ��W5500�Ĵ�������
	Detect_Gateway(0);	//������ط����� 
	Detect_Gateway(1);	//������ط�����
	Detect_Gateway(2);	//������ط�����
	Detect_Gateway(3);	//������ط�����
	Socket_Init(0);		//ָ��Socket(0~7)��ʼ��,��ʼ���˿�0
	Socket_Init(1);		//ָ��Socket(0~7)��ʼ��,��ʼ���˿�1
	Socket_Init(2);		//ָ��Socket(0~7)��ʼ��,��ʼ���˿�2
	Socket_Init(3);		//ָ��Socket(0~7)��ʼ��,��ʼ���˿�3
}

/*******************************************************************************
* ������  : Load_Net_Parameters
* ����    : װ���������
* ����    : ��
* ���    : ��
* ����ֵ  : ��
* ˵��    : ���ء����롢�����ַ������IP��ַ���˿ںš�Ŀ��IP��ַ��Ŀ�Ķ˿ںš��˿ڹ���ģʽ
*******************************************************************************/
void Load_Net_Parameters(void)
{
	uint32_t Address=0;
	uint8_t tx_ip[8];
	uint8_t tx_port[4];
	Address=Ip_Port_Addr;
	FLASH_Read(Address,tx_ip,8);//����IP��ַ16λ
	FLASH_Read(Address+16,tx_port,4);//�����˿ں�16λ

	Sub_Mask[0]=255;//������������
	Sub_Mask[1]=255;
	Sub_Mask[2]=255;
	Sub_Mask[3]=0;

	Get_STM32_ID();						//��ȡstm32ID��
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
		IP_Addr[0]=192;//���ر���IP��ַ
		IP_Addr[1]=168;
		IP_Addr[2]=8;
		IP_Addr[3]=120;

		Gateway_IP[0] = 192;//�������ز���
		Gateway_IP[1] = 168;
		Gateway_IP[2] = 8;
		Gateway_IP[3] = 1;
		
		S0_Port[0] = 0x04;//���ض˿�0�Ķ˿ں�1225 
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
	
	S1_Port[0] = 0x04;//���ض˿�1�Ķ˿ں�1228
	S1_Port[1] = 0xCC;
	
	S2_Port[0] = 0x04;//���ض˿�2�Ķ˿ں�1229
	S2_Port[1] = 0xCD;
	
	S3_Port[0] = 0x04;//���ض˿�3�Ķ˿ں�1230
	S3_Port[1] = 0xCE;
	
	NTP_DIPR[0]=192;
	NTP_DIPR[1]=168;
	NTP_DIPR[2]=8;
	NTP_DIPR[3]=5;

	NTP_DPORT[0]=0;
	NTP_DPORT[1]=0x7B;	//�˿ں�123
	
	S0_Mode=TCP_SERVER;	//���ض˿�0�Ĺ���ģʽ,TCP������ģʽ
	S1_Mode=TCP_SERVER;	//���ض˿�1�Ĺ���ģʽ,TCP������ģʽ
	S2_Mode=UDP_MODE;		//���ض˿�2�Ĺ���ģʽ,UDP������ģʽ
	S3_Mode=UDP_MODE;		//���ض˿�3�Ĺ���ģʽ,UDP������ģʽ
}

/*******************************************************************************
* ������  : W5500_Socket_Set
* ����    : W5500�˿ڳ�ʼ������
* ����    : ��
* ���    : ��
* ����ֵ  : ��
* ˵��    : �ֱ�����4���˿�,���ݶ˿ڹ���ģʽ,���˿�����TCP��������TCP�ͻ��˻�UDPģʽ.
*			�Ӷ˿�״̬�ֽ�Socket_State�����ж϶˿ڵĹ������
*******************************************************************************/
void W5500_Socket_Set(void)
{
	if(S0_State==0)//�˿�0��ʼ������
	{
		if(S0_Mode==TCP_SERVER)//TCP������ģʽ 
		{
			if(Socket_Listen(0)==TRUE_W5500)
				S0_State=S_INIT;
			else
				S0_State=0;
		}
		else if(S0_Mode==TCP_CLIENT)//TCP�ͻ���ģʽ 
		{
			if(Socket_Connect(0)==TRUE_W5500)
				S0_State=S_INIT;
			else
				S0_State=0;
		}
		else//UDPģʽ 
		{
			if(Socket_UDP(0)==TRUE_W5500)
				S0_State=S_INIT|S_CONN;
			else
				S0_State=0;
		}
	}
	
	if(S1_State==0)//�˿�1��ʼ������
	{
		if(S1_Mode==TCP_SERVER)//TCP������ģʽ 
		{
			if(Socket_Listen(1)==TRUE_W5500)
				S1_State=S_INIT;
			else
				S1_State=0;
		}
		else if(S1_Mode==TCP_CLIENT)//TCP�ͻ���ģʽ 
		{
			if(Socket_Connect(1)==TRUE_W5500)
				S1_State=S_INIT;
			else
				S1_State=0;
		}
		else//UDPģʽ 
		{
			if(Socket_UDP(1)==TRUE_W5500)
				S1_State=S_INIT|S_CONN;
			else
				S1_State=0;
		}
	}
	
	if(S2_State==0)//�˿�2��ʼ������
	{
		if(S2_Mode==TCP_SERVER)//TCP������ģʽ 
		{
			if(Socket_Listen(2)==TRUE_W5500)
				S2_State=S_INIT;
			else
				S2_State=0;
		}
		else if(S2_Mode==TCP_CLIENT)//TCP�ͻ���ģʽ 
		{
			if(Socket_Connect(2)==TRUE_W5500)
				S2_State=S_INIT;
			else
				S2_State=0;
		}
		else//UDPģʽ 
		{
			if(Socket_UDP(2)==TRUE_W5500)
				S2_State=S_INIT|S_CONN;
			else
				S2_State=0;
		}
	}
	
	if(S3_State==0)//�˿�2��ʼ������
	{
		if(S3_Mode==TCP_SERVER)//TCP������ģʽ 
		{
			if(Socket_Listen(3)==TRUE_W5500)
				S3_State=S_INIT;
			else
				S3_State=0;
		}
		else if(S3_Mode==TCP_CLIENT)//TCP�ͻ���ģʽ 
		{
			if(Socket_Connect(3)==TRUE_W5500)
				S3_State=S_INIT;
			else
				S3_State=0;
		}
		else//UDPģʽ 
		{
			if(Socket_UDP(3)==TRUE_W5500)
				S3_State=S_INIT|S_CONN;
			else
				S3_State=0;
		}
	}
	
}

/*��ȡSTM32ID��*/
void Get_STM32_ID(void)
{
    u32 temp0,temp1,temp2;
temp0 = *(__IO u32*)(0x1FFFF7E8);    //��ƷΨһ��ݱ�ʶ�Ĵ�����96λ��
    temp1 = *(__IO u32*)(0x1FFFF7EC);
temp2 = *(__IO u32*)(0x1FFFF7F0);
                                  
//ID���ַ�� 0x1FFFF7E8   0x1FFFF7EC  0x1FFFF7F0 ��ֻ��Ҫ��ȡ�����ַ�е����ݾͿ����ˡ�
 
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
* ������  : Process_Socket_Data
* ����    : W5500���ղ����ͽ��յ�������
* ����    : s:�˿ں� msg:��������  len:���ݳ���
* ���    : ��
* ����ֵ  : ��
* ˵��    : ��
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
* ������  : RCC_Configuration
* ����    : ʱ��ʹ��
* ����    : ��
* ���    : ��
* ����ֵ  : ��
* ˵��    : ��
*******************************************************************************/
void RCC_Configuration(void)
{
  	/* Enable I2C1 and I2C1 clock */
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2 | RCC_APB1Periph_TIM4, ENABLE);

  	/* Enable GPIOA GPIOB SPI2 and USART1 clocks */
  	RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE); 
					
}

/*******************************************************************************
* ������  : NVIC_Configuration
* ����    : STM32�ж�������������
* ����    : ��
* ���    : ��
* ����ֵ  : ��
* ˵��    : ����KEY1(PC11)���ж�������
*******************************************************************************/

void NVIC_Configuration(void)
{
	/*��ʱ���ж�����*/
	NVIC_InitTypeDef NVIC_InitStructure;						//����NVIC��ʼ���ṹ��

  	/* Set the Vector Table base location at 0x08000000 */
//  	NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x0);
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);				//�����ж����ȼ���Ϊ1��������(����0��4λ)
	
	NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;				//�����ж�������
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;	//�����������ȼ�
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;			//������Ӧ���ȼ�
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;				//ʹ��NVIC
	NVIC_Init(&NVIC_InitStructure);
	
	W5500_NVIC_Configuration(); //W5500�ж�����
}

/*******************************************************************************
* ������  : Timer3_Init_Config
* ����    : Timer2��ʼ������
* ����    : ��
* ���    : ��
* ����    : �� 
* ˵��    : ��
*******************************************************************************/

void Timer4_Init_Config(void)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	
	TIM_TimeBaseStructure.TIM_Period = 9;						//��������һ�������¼�װ�����Զ���װ�ؼĴ������ڵ�ֵ(������10Ϊ1ms)
	TIM_TimeBaseStructure.TIM_Prescaler = 7199;					//����������ΪTIMxʱ��Ƶ�ʳ�����Ԥ��Ƶֵ(10KHz�ļ���Ƶ��)
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;		//����ʱ�ӷָ�:TDTS = TIM_CKD_DIV1
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;	//TIM���ϼ���ģʽ
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);				//����TIM_TimeBaseInitStruct��ָ���Ĳ�����ʼ��TIMx��ʱ�������λ
	 
	TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE ); 				//ʹ��TIM4ָ�����ж�
	
	TIM_Cmd(TIM4, ENABLE);  									//ʹ��TIMx����
}

/*******************************************************************************
* ������  : TIM4_IRQHandler
* ����    : ��ʱ��4�ж϶Ϸ�����
* ����    : ��
* ���    : ��
* ����    : �� 
* ˵��    : ��
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
* ������  : System_Initialization
* ����    : STM32ϵͳ��ʼ������(��ʼ��STM32ʱ�Ӽ�����)
* ����    : ��
* ���    : ��
* ����    : �� 
* ˵��    : ��
*******************************************************************************/
void System_Initialization(void)
{
	RCC_Configuration();
	Timer4_Init_Config();		//��ʱ����ʼ��
	NVIC_Configuration();		//�ж�����
	SPI_Configuration();		//W5500 SPI��ʼ������(STM32 SPI2)
	W5500_GPIO_Configuration();	//W5500 GPIO��ʼ������	
	
}

/*��ʱ����*/
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

