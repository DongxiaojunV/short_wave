#include "Task_Config.h"
#include "bsp_uart_fifo.h"
#include "my_protocol.h"
#include "main.h"

#include "iap.h"
#include "stmflash.h"
#include "firmware_upgrade.h"

#include "fifo.h"
#include "bsp_uart.h"
#include "w5500.h"
#pragma		pack(1)

typedef struct
{
	uint8_t	upgrade_package_total;
	uint8_t	upgrade_package_num;
	uint8_t upgrade_version[4];
	uint8_t	upgrade_package[2048];
}App_Upgrade_t;

typedef struct
{
	uint8_t	results;
}App_Upgrade_Back_t;

typedef struct
{
	uint8_t	CAN_ID[1];
	uint8_t IP[4];		//设置网口IP
	uint8_t Socket0_Port[2];	//设置端口号
}App_SetID_t;

typedef struct
{
	uint8_t	results;
}App_SetID_Back_t;

typedef struct
{
	uint8_t	CAN_ID[1];
	uint8_t	emit_type[1];
	uint8_t	version[4];
	uint8_t App_Address;
	uint8_t IP[4];
	uint8_t Socket0_Port[2];	
}App_Version_Back_t;

App_Upgrade_t		App_Upgrade;
App_Upgrade_Back_t	App_Upgrade_Back;
App_SetID_t			App_SetID;
App_SetID_Back_t	App_SetID_Back;
App_Version_Back_t	App_Version_Back;

TaskHandle_t xHandleTask_App;

/*需要使用固件升级时取消注释*/
static uint8_t	Check_Data_Valid(uint8_t *Func_code, uint8_t *p_data, uint16_t data_len);
static uint8_t Send_App(uint8_t *Func_code, uint8_t *p_data, uint8_t data_len);
uint8_t	RxBuf_FromUpgrade[2100];
uint8_t RxBuf[2048];
uint16_t RxBuf_FromUpgrade_len=0;
void Task_App(void * pvParameters)
{
#if	FIRMWARE_UPDATE_EN
	uint8_t			sys_init = 0x00;
	uint8_t			function_code[2];
	uint32_t		address = 0;
	uint32_t		upgrade_version;

	uint32_t		count =0;
  uint32_t		ret;

#endif
	
	
	while(1)
	{
#if	FIRMWARE_UPDATE_EN

		count=RxBuf_len;
		
		if(count>2)
		{
			
			if(RxBuf[count-1]!=0x04)
			{
				for(uint16_t i=0;i<count;i++)
				{
					RxBuf_FromUpgrade[RxBuf_FromUpgrade_len]=RxBuf[i];
					RxBuf_FromUpgrade_len++;
				}
					RxBuf_len=0;
				
			}
			else
			{
					for(uint16_t i=0;i<count;i++)
					{
						RxBuf_FromUpgrade[RxBuf_FromUpgrade_len]=RxBuf[i];
						RxBuf_FromUpgrade_len++;
					}
					RxBuf_len=0;
					count=RxBuf_FromUpgrade_len;
					RxBuf_FromUpgrade_len=0;
			}
				
			ret = Check_Data_Valid(function_code, RxBuf_FromUpgrade, count);
			
			count = 0;
			if( ret == 0x01)
			{
				if( (function_code[0]==0x0E) && (function_code[1]==0x01) )				//远程升级，更新固件
				{

					upgrade_version = (App_Upgrade.upgrade_version[0]<<24) + (App_Upgrade.upgrade_version[1]<<16) + (App_Upgrade.upgrade_version[2]<<8) + (App_Upgrade.upgrade_version[3]<<0);
					if( upgrade_version < APP_VERSION )
					{
						ret = 0x00;		//清零
					}
					else	if( System.emission == 0x01 )
					{
						ret = 0x00;		//清零
					}
					
					if( ret == 0x00 )
					{
						App_Upgrade_Back.results = 0xFD;
					}
					else	if( (System.status!=SYSTEM_OPENING) && (System.emission!=0x01) )	//正在开机、正在调谐、发射状态不能升级
					{
						address = Get_Old_Application_Address()+(App_Upgrade.upgrade_package_num-1)*APPLICATION_PACK_SIZE;
						iap_write_appbin(address, App_Upgrade.upgrade_package, APPLICATION_PACK_SIZE);
						
						App_Upgrade_Back.results = 0xFE;
						
					}
					else
					{
						App_Upgrade_Back.results = 0xFD;
					}
					
					Send_App(function_code, (uint8_t *)&App_Upgrade_Back, sizeof(App_Upgrade_Back.results));
					
					if( (App_Upgrade_Back.results == 0xFE) && (App_Upgrade.upgrade_package_total == App_Upgrade.upgrade_package_num) && (App_Upgrade.upgrade_package_total>0) )
					{
						if( Get_Old_Application_Address() == APPLICATION_ADDRESS_BASE+APPLICATION_ADDRESS_OFFSET_1 )
						{
							APP_Version_1[0] = App_Upgrade.upgrade_version[0];
							APP_Version_1[1] = App_Upgrade.upgrade_version[1];
							APP_Version_1[2] = App_Upgrade.upgrade_version[2];
							APP_Version_1[3] = App_Upgrade.upgrade_version[3];
						}
						else
						{
							APP_Version_2[0] = App_Upgrade.upgrade_version[0];
							APP_Version_2[1] = App_Upgrade.upgrade_version[1];
							APP_Version_2[2] = App_Upgrade.upgrade_version[2];
							APP_Version_2[3] = App_Upgrade.upgrade_version[3];
						}
						
						APP_Version_Write(APPLICATION_VERSION_ADDRESS, APP_Version_1, APP_Version_2);
						APP_Version_Read(APPLICATION_VERSION_ADDRESS, APP_Version_1, APP_Version_2);
						
						//App_printf("App Upgrade...\r\n");
						vTaskDelay(1000);		//等待发送完成
						
						BootLoad_Jump();
					}
				}
				
				else	if( (function_code[0]==0x0F) && (function_code[1]==0x01) )		//修改System.CAN_ID,ip,port
				{
					System.Init_Mark=0;				//不是初始化指令，为0表示要写IP和Port进Flash
					System.CAN_ID[0] = App_SetID.CAN_ID[0];
					System.Ip[0]=App_SetID.IP[0];
					System.Ip[1]=App_SetID.IP[1];
					System.Ip[2]=App_SetID.IP[2];
					System.Ip[3]=App_SetID.IP[3];
					System.Port[0]=App_SetID.Socket0_Port[0];
					System.Port[1]=App_SetID.Socket0_Port[1];
					
					Alarm_threshold.Transmitte_id[0] = System.CAN_ID[0];	//存在flash

					if( System.already_init == 0x01 )
						sys_init = 0x01;
					else
						sys_init = 0x00;
					
					if( InternalFlash_SaveData_1(sys_init) == 0x00 )
					{
						App_SetID_Back.results = 0xFD;
						//App_printf("Set System ID fail\r\n");
					}
					else
					{
						Flash_to_AcceptAPP();
						
						App_SetID_Back.results = 0xFE;
						//App_printf("Set System ID sucessfully\r\n");
					}
					
					Send_App(function_code, (uint8_t *)&App_SetID_Back, sizeof(App_SetID_Back));
					vTaskDelay(50);
					__set_FAULTMASK(1);//关闭中断
					NVIC_SystemReset();//软件复位,保证网口重新初始化
				}
				else	if( (function_code[0]==0x10) && (function_code[1]==0x01) )		//查询APP版本号
				{
					App_Version_Back.CAN_ID[0] = System.CAN_ID[0];
					App_Version_Back.emit_type[0] = NEW_PROCOTOL;			//NEW_PROCOTOL	locate in main.h
					App_Version_Back.version[0] = (APP_VERSION>>24)&0xFF;
					App_Version_Back.version[1] = (APP_VERSION>>16)&0xFF;
					App_Version_Back.version[2] = (APP_VERSION>>8)&0xFF;
					App_Version_Back.version[3] = (APP_VERSION>>0)&0xFF;
					App_Version_Back.App_Address = Get_APP_Version()+1;
					App_Version_Back.IP[0]=IP_Addr[0];
					App_Version_Back.IP[1]=IP_Addr[1];
					App_Version_Back.IP[2]=IP_Addr[2];
					App_Version_Back.IP[3]=IP_Addr[3];
					App_Version_Back.Socket0_Port[0]=S0_Port[0];
					App_Version_Back.Socket0_Port[1]=S0_Port[1];
					
					Send_App(function_code, (uint8_t *)&App_Version_Back, sizeof(App_Version_Back));//暂时没改(升级网口)
				}
				else																	//保留
				{

				}
			}
			else
			{
				
			}
		}
		else
		{

		}
		
#endif
		vTaskDelay(100);
	}
}

#if FIRMWARE_UPDATE_EN
static uint8_t Check_Data_Valid(uint8_t *Func_code, uint8_t *p_data, uint16_t data_len)
{
	uint16_t	i = 0;
	uint16_t	len = 0;
	uint8_t		ret = 0;
	
	uint8_t		buf_CRC[2];
	uint16_t	CRC_Back;
	uint16_t	ReceiveData_CRC;
    
	for(i=0; data_len>8; i++, data_len--)
	{
		if( (p_data[i] == 0x05) && (p_data[i+1] == 0x02) )
		{
			len = (p_data[i+3]<<8) + p_data[i+4];
			if( data_len < len )
			{
				ret = 0x02;				
				return ret;
			}
			else	if( (p_data[5+len+2] == 0x03) && (p_data[5+len+2+1] == 0x04) )
			{
				Func_code[0] = p_data[5];
				Func_code[1] = p_data[6];
				
				buf_CRC[0] = *(p_data+len+5);
				buf_CRC[1] = *(p_data+len+6);
				ReceiveData_CRC = ((buf_CRC[0]<<8) +(buf_CRC[1]));
				
				CRC_Back = CRC16_XMODEM(p_data+5, len);
				if( CRC_Back == ReceiveData_CRC )
				{
					ret = 0x01;
					
					if( (Func_code[0]==0x0E) && (Func_code[1]==0x01) )				//远程升级，更新固件
					{
						App_Upgrade.upgrade_package_total = p_data[5+3];
						App_Upgrade.upgrade_package_num = p_data[5+4];
						memcpy(App_Upgrade.upgrade_version, (p_data+5+5), sizeof(App_Upgrade.upgrade_version));
						memcpy(App_Upgrade.upgrade_package, (p_data+5+9), APPLICATION_PACK_SIZE);
					}
					else	if( (Func_code[0]==0x0F) && (Func_code[1]==0x01) )		//修改System.CAN_ID
					{
						App_SetID.CAN_ID[0] = p_data[8];
						App_SetID.IP[0]=p_data[9];
						App_SetID.IP[1]=p_data[10];
						App_SetID.IP[2]=p_data[11];
						App_SetID.IP[3]=p_data[12];
						App_SetID.Socket0_Port[0]=p_data[13];
						App_SetID.Socket0_Port[1]=p_data[14];
					}
					else	if( (Func_code[0]==0x10) && (Func_code[1]==0x01) )		//查询APP版本号
					{
						
					}
					else
					{
						ret = 0x02;
					}
					
					return ret;
				}
				else
				{
					ret = 0x02;			
					return ret;
				}
			}
			else
			{
				ret = 0x02;				
				return ret;
			}
		}
	}

	return 0x02;
}

static uint8_t Send_App(uint8_t *Func_code, uint8_t *p_data, uint8_t data_len)
{
  Buffer_t COM_buffer;

	uint16_t CRC_Back;
	uint8_t Start_Data[8] = {0x05, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
	uint8_t End_Data[4] = {0x00, 0x00, 0x03, 0x04};

	//补零
    uint8_t temp = 0x00;
    uint8_t temp_len = 0x00;
	
	if( (Func_code[0] == 0x0E) && (Func_code[1] == 0x01) )				//固件升级
	{
		
	}
	else	if( (Func_code[0] == 0x0F) && (Func_code[1] == 0x01) )		//修改System ID
    {
		
	}
	else	if( (Func_code[0] == 0x10) && (Func_code[1] == 0x01) )		//查询APP版本号
    {
		
	}
	else
	{
		return 0;
	}

	data_len += 3;
	Start_Data[2] = System.CAN_ID[0];
	Start_Data[3] = (data_len>>8)&0xFF;		//数据长度
	Start_Data[4] = (data_len>>0)&0xFF;		//数据长度
	Start_Data[5] = Func_code[0];		//功能码
	Start_Data[6] = 0x02;				//功能码

	#if	ADD_CAN_LEN
	Start_Data[7] = 5 + data_len + 4;
	#endif

	memcpy(COM_buffer.data, Start_Data, 8);
	memcpy(COM_buffer.data+8, p_data, data_len-3);
	CRC_Back = CRC16_XMODEM(COM_buffer.data+5, data_len);
	End_Data[0] = (uint8_t)((CRC_Back >> 8) & 0xff);		//存高八位
	End_Data[1] = (uint8_t)((CRC_Back >> 0) & 0xff);		//低八位
	memcpy(COM_buffer.data+data_len+5, End_Data, 4);
	COM_buffer.len = 5+data_len+4;

	#if	1	//不够8位，补零
	temp = COM_buffer.len % 8;

	if( temp == 0x00 )
	{

	}
	else
	{
		temp_len = 8-temp;
		memset(COM_buffer.data+COM_buffer.len, 0, temp_len);
		COM_buffer.len += temp_len;
	}
	#endif
	
	Write_SOCK_Data_Buffer(1,COM_buffer.data,COM_buffer.len);
		
	return 1;
}

#endif
