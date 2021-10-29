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
extern uint8_t Updata_time_flag;//上电同步一次时间标志
uint16_t W5500_rx_buf_len = 0;
uint16_t RxBuf_len = 0;

void W5500_APP(void * pvParameters)
{	
	uint16_t i=0;
	uint16_t s0_len=0,s1_len=0;
	uint16_t S0_cut_count=0;
	socket2.Pack_Creat_Mark=0;		//构建包标志位，上电一次构建一次就行了
	
    while(1)
	{
		
	  W5500_Socket_Set();	//W5500端口初始化配置
		
		if(W5500_Interrupt)	//如果中断标志为:1
	  {
	   	W5500_Process();	//W5500处理程序框架
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
			//保留
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
		if((S0_Data & S_RECEIVE) == S_RECEIVE)		//如果Socket0接收到数据
		{
			s0_len=0;
			S0_Data&=~S_RECEIVE;					//处理标志位
			s0_len=Read_SOCK_Data_Buffer(0,Socket0_Rx_Buffer);	//接收数据，返回整个数据包长度
			
			for(i=0; i<s0_len; i++)
			{
				RxBuf_FromPC.data[i] = Socket0_Rx_Buffer[i];
			}
			W5500_rx_buf_len=s0_len;
			S0_cut_count=0;
		}
		
		if((S1_Data & S_RECEIVE) == S_RECEIVE)		//如果Socket1接收到数据
		{
			s1_len=0;
			S1_Data&=~S_RECEIVE;					//处理标志位
			s1_len=Read_SOCK_Data_Buffer(1,Socket1_Rx_Buffer);	//接收数据，返回整个数据包长度
			
			for(i=0;i<s1_len;i++)
			{
				RxBuf[i]=Socket1_Rx_Buffer[i];
			}
			RxBuf_len=s1_len;
		}
		
		if((S2_Data & S_RECEIVE) == S_RECEIVE)		//如果Socket2接收到数据
		{
			S2_Data&=~S_RECEIVE;					//处理标志位
			Read_SOCK_Data_Buffer(2,Socket2_Rx_Buffer);	//接收数据，返回整个数据包长度	
			
			UDP_DIPR[0] = Socket2_Rx_Buffer[0];//收到的是发送方的IP和端口
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
		
		if((S3_Data & S_RECEIVE) == S_RECEIVE)		//如果Socket3接收到数据
		{
			S3_Data&=~S_RECEIVE;					//处理标志位
			uint32_t local_timestamp; 
			ntp_packet packet ;
			struct tm * Net_time; 
			
			Read_SOCK_Data_Buffer(3,Socket3_Rx_Buffer);	//接收数据，返回整个数据包长度
			packet.txTm_s = Socket3_Rx_Buffer[40]<<24 | Socket3_Rx_Buffer[41]<<16|Socket3_Rx_Buffer[42]<<8 |Socket3_Rx_Buffer[43]; 
			
			local_timestamp = packet.txTm_s - NTP_TIMESTAMP_DELTA;//减去1970和1900的差值
			local_timestamp +=SEC_TIME_ZONE; //加上北京的时间差，GMT+8
			Net_time = localtime(&local_timestamp); //秒数转换位标准时间，在time.h中
			if(Net_time->tm_year+1900==2036)		//过滤掉第一次询问时，返回的2036数据
			{
					NTP_Pack_Send();
			}
			else
			{
				g_inquire_stamp=local_timestamp;			//为了和以往协议同步，从1970年开始计算
				g_stamp_distance = g_inquire_stamp - RTC_GetCounter();	//获取本地时间与服务器时间的差值
				System.time_update_flag = 0x01;//更新了时间标志位
			}
		}
		vTaskDelay(100);
	}
    		
}

/*SOCKET2的答复包创建*/
void Socket2_Pack_Creat(void)
{
	socket2.C_IP=AP_ICC(IP_Addr);
	socket2.Socket0_Port=P_ICC(S0_Port);
	socket2.Socket1_Port=P_ICC(S1_Port);
	socket2.Socket2_Port=P_ICC(S2_Port);
	socket2.Stm32_ID=Array_ch(STM32_ID,12);
	sprintf(socket2.Buff_Pack,"IP:%s;Port0:%s;Port1:%s;Port2:%s;Stm32_ID:%s;",socket2.C_IP,socket2.Socket0_Port,socket2.Socket1_Port,socket2.Socket2_Port,socket2.Stm32_ID);
}

