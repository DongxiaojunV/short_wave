#include "Task_Config.h"
#include "W5500.h"
#include "my_protocol.h"
#include "int_char_change.h"
#include "NTP.h"
#include "time.h"
TaskHandle_t xW5500Task_APP;
Socket2 socket2;

extern Buffer_t RxBuf_FromPC;
extern uint8_t RxBuf[2048];
extern uint8_t STM32_ID[12];
extern uint8_t Updata_time_flag;//�ϵ�ͬ��һ��ʱ���־
uint16_t W5500_rx_buf_len = 0;
uint16_t RxBuf_len = 0;

void W5500_APP(void * pvParameters)
{	
	uint16_t i=0;
	uint16_t s0_len=0,s1_len=0;
	uint16_t S0_cut_count=0;
	socket2.Pack_Creat_Mark=0;		//��������־λ���ϵ�һ�ι���һ�ξ�����
	
    while(1)
	{
		
	  W5500_Socket_Set();	//W5500�˿ڳ�ʼ������
		
		if(W5500_Interrupt)	//����жϱ�־Ϊ:1
	  {
	   	W5500_Process();	//W5500���������
			if(Updata_time_flag==0)
			{
					Updata_time_flag++;
			}
	  }
		if((Read_W5500_1Byte(PHYCFGR)&LINK)==0)
		{
			S0_State=0;
			S1_State=0;
			S2_State=0;
			S3_State=0;
		}
		else
		{
			//����
		}
		
		if(Read_W5500_SOCK_1Byte(0,Sn_SR)==SOCK_ESTABLISHED)
		{
			S0_cut_count++;
			if(S0_cut_count>=100)
			{
				S0_cut_count=0;
				S0_State=0;
			}
		}
		if((S0_Data & S_RECEIVE) == S_RECEIVE)		//���Socket0���յ�����
		{
			s0_len=0;
			S0_Data&=~S_RECEIVE;					//�����־λ
			s0_len=Read_SOCK_Data_Buffer(0,Socket0_Rx_Buffer);	//�������ݣ������������ݰ�����
			
			for(i=0; i<s0_len; i++)
			{
				RxBuf_FromPC.data[i] = Socket0_Rx_Buffer[i];
			}
			W5500_rx_buf_len=s0_len;
			S0_cut_count=0;
		}
		
		if((S1_Data & S_RECEIVE) == S_RECEIVE)		//���Socket1���յ�����
		{
			s1_len=0;
			S1_Data&=~S_RECEIVE;					//�����־λ
			s1_len=Read_SOCK_Data_Buffer(1,Socket1_Rx_Buffer);	//�������ݣ������������ݰ�����
			
			for(i=0;i<s1_len;i++)
			{
				RxBuf[i]=Socket1_Rx_Buffer[i];
			}
			RxBuf_len=s1_len;
		}
		
		if((S2_Data & S_RECEIVE) == S_RECEIVE)		//���Socket2���յ�����
		{
			S2_Data&=~S_RECEIVE;					//�����־λ
			Read_SOCK_Data_Buffer(2,Socket2_Rx_Buffer);	//�������ݣ������������ݰ�����	
			
			UDP_DIPR[0] = Socket2_Rx_Buffer[0];//�յ����Ƿ��ͷ���IP�Ͷ˿�
			UDP_DIPR[1] = Socket2_Rx_Buffer[1];
			UDP_DIPR[2] = Socket2_Rx_Buffer[2];
			UDP_DIPR[3] = Socket2_Rx_Buffer[3];
		
			UDP_DPORT[0] = Socket2_Rx_Buffer[4];
			UDP_DPORT[1] = Socket2_Rx_Buffer[5];
			
			if(socket2.Pack_Creat_Mark==0)
			{
				Socket2_Pack_Creat();
				socket2.Pack_Creat_Mark=1;
			}
			
			Socketn_Char_Send(2,socket2.Buff_Pack,strlen(socket2.Buff_Pack));
		}
		
		if((S3_Data & S_RECEIVE) == S_RECEIVE)		//���Socket3���յ�����
		{
			S3_Data&=~S_RECEIVE;					//�����־λ
			uint32_t local_timestamp; 
			ntp_packet packet ;
			struct tm * Net_time; 
			
			Read_SOCK_Data_Buffer(3,Socket3_Rx_Buffer);	//�������ݣ������������ݰ�����
			packet.txTm_s = Socket3_Rx_Buffer[40]<<24 | Socket3_Rx_Buffer[41]<<16|Socket3_Rx_Buffer[42]<<8 |Socket3_Rx_Buffer[43]; 
			
			local_timestamp = packet.txTm_s - NTP_TIMESTAMP_DELTA;//��ȥ1970��1900�Ĳ�ֵ
			local_timestamp +=SEC_TIME_ZONE; //���ϱ�����ʱ��GMT+8
			Net_time = localtime(&local_timestamp); //����ת��λ��׼ʱ�䣬��time.h��
			if(Net_time->tm_year+1900==2036)		//���˵���һ��ѯ��ʱ�����ص�2036����
			{
					NTP_Pack_Send();
			}
			else
			{
				g_inquire_stamp=local_timestamp;			//Ϊ�˺�����Э��ͬ������1970�꿪ʼ����
				g_stamp_distance = g_inquire_stamp - RTC_GetCounter();	//��ȡ����ʱ���������ʱ��Ĳ�ֵ
				System.time_update_flag = 0x01;//������ʱ���־λ
			}
		}
		vTaskDelay(100);
	}
    		
}

/*SOCKET2�Ĵ𸴰�����*/
void Socket2_Pack_Creat(void)
{
	socket2.C_IP=AP_ICC(IP_Addr);
	socket2.Socket0_Port=P_ICC(S0_Port);
	socket2.Socket1_Port=P_ICC(S1_Port);
	socket2.Socket2_Port=P_ICC(S2_Port);
	socket2.Stm32_ID=Array_ch(STM32_ID,12);
	sprintf(socket2.Buff_Pack,"IP:%s;Port0:%s;Port1:%s;Port2:%s;Stm32_ID:%s;",socket2.C_IP,socket2.Socket0_Port,socket2.Socket1_Port,socket2.Socket2_Port,socket2.Stm32_ID);
}

