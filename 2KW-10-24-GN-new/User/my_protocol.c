#include "my_protocol.h"
#include "MT2000_protocol.h"
#include "stand_wave_rate.h"
#include "bsp_uart_fifo.h"

#include "main.h"
#include <string.h>
#include <math.h>

#include "alarm.h"
#include "power.h"

#include "bsp_internal_flash.h"

//#include "can.h"
#include "W5500.h"
#include "firmware_upgrade.h"

extern uint8_t re_scan_flag;

extern F_I_t  flash2_arry[480];		//扫频数据

extern TaskHandle_t xHandleTask_MT2000;
extern TaskHandle_t xHandleTask_Alarm;
/*---------------------------------更新状态-----------------------------------*/
void update_status_without_inquire(void)
{
    static uint8_t n = 0;

    n++;

    if(	System.status == SYSTEM_UNINITIALIZE )	//未初始化状态
    {
        if( n >= 10 )	//1000ms更新一次
        {
            n = 0;
            App_printf("system have no init\r\n");
        }
    }
    else	if( (Alarm.emission==0x01) || (Alarm.no_respond==0x01) || (Alarm.no_power==0x01) )	//激励器自身报警，无应答报警、无功率报警
    {
        //激励器自身报警，无应答报警，无功率报警，驻波比报警属于大报警，查询时马上返回，其他报警则先返回操作失败，回到待机/发射/报警状态，等待查询时再报警

        System.status = SYSTEM_ALARM;
        System.close = 0;
        System.open = 0;

        System.sweep = 0;
//		System.cancel_sweeping = 0;
        System.achieve_add_sub = 0;
        System.cancel_add_sub = 0;
        System.modify_power = 0;

        if( n >= 10 )	//1000ms更新一次
        {
            n = 0;
            App_printf("history 1 alarm!\r\n");
        }
    }
    else	if( System.status == SYSTEM_OPENING )		//正在开机
    {
        if( Power_onBack.results[0] == 0xFD )			//开机失败
        {
            App_printf("system open fail\r\n");

            System.status = SYSTEM_ALARM;				//开机失败，直接报警
            Power_onBack.results[0] = 0x00;
        }
        else	if( Power_onBack.results[0]==0xFC )		//开机成功
        {
            System.status = SYSTEM_STANDBY;
            App_printf("system open successfully!\r\n");

            Power_onBack.results[0] = 0x00;
        }
        else	if( Power_onBack.results[0]==0xFE )		//正在开机
        {
            System.status = SYSTEM_OPENING;
            if( n >= 10 )	//1000ms更新一次
            {
                n = 0;
                App_printf("system Opening ...\r\n");
            }
        }
    }
    else	if( System.status == SYSTEM_SCAN )			//扫频
    {
        if(System.sweep == 0x02)		//扫频完成
        {
            System.sweep = 0;
            System.already_swept = 0x01;
            System.status = SYSTEM_STANDBY;	//待机
            System.emission = 0x00;		//发射停止标志位
        }
        else	if(System.sweep == 0x00)
        {
            System.status = SYSTEM_STANDBY;
            System.emission = 0x00;
        }
        else	if(System.sweep == 0x01)
        {
//			System.emission = 0x01;
            System.status = SYSTEM_SCAN;
            if( n >= 10 )	//1000ms更新一次
            {
                n = 0;
                App_printf("system Sweeping ...\r\n");
            }
        }
    }
    else	if( System.status == SYSTEM_SHUTDOWN )		//关机
    {
		System.Voltage = 0.0;
		System.Electricity = 0.0;
		
        Power_onBack.results[0] = 0x00;

        if( n >= 10 )	//1000ms更新一次
        {
            n = 0;
            App_printf("2system close !\r\n");
        }
    }

    else	if( System.status == SYSTEM_STANDBY )		//待机状态
    {
        if(get_history_alarm()==1)						//历史报警
        {
            System.sweep = 0;
            System.achieve_add_sub = 0;
//			System_Status_Clean();		//System.emission清零
            System.status = SYSTEM_ALARM;

            if( n >= 10 )	//1000ms更新一次
            {
                n = 0;
                Trans_printf("history 3 alarm!\r\n");
            }

            if( System.mode == 0 )					//清零
            {
                memset(System.freq1, 0, 8);
                memset(System.freq2, 0, 8);
                memset(System.freq3, 0, 8);
            }
            else	if( System.mode == 1 )
            {
                memset(System.freq2, 0, 8);
                memset(System.freq3, 0, 8);
            }
            else	if( System.mode == 3 )
            {
                memset(System.freq3, 0, 8);
            }
        }
        else	if( System.Voltage <= 10.0 && GT2000_Rx.sys_open==0)			//关机状态
        {
            System.open = 0x00;
            System.status = SYSTEM_SHUTDOWN;
            Trans_printf("1system close !\r\n");
					  App_printf("1system close !\r\n");
        }
    }
    else	if( System.status == SYSTEM_EMISSING )		//发射状态
    {
        if(get_history_alarm()==1)						//历史报警
        {
            System.sweep = 0;
            System.achieve_add_sub = 0;
//			System_Status_Clean();		//System.emission清零
            System.status = SYSTEM_ALARM;

            if( n >= 10 )	//1000ms更新一次
            {
                n = 0;
                Trans_printf("history 4 alarm!\r\n");
            }

            if( System.mode == 0 )					//清零
            {
                memset(System.freq1, 0, 8);
                memset(System.freq2, 0, 8);
                memset(System.freq3, 0, 8);
            }
            else	if( System.mode == 1 )
            {
                memset(System.freq2, 0, 8);
                memset(System.freq3, 0, 8);
            }
            else	if( System.mode == 2 )
            {
                memset(System.freq3, 0, 8);
            }
        }
        else	if( System.emission == 0x00 && System.Power_Adjustment!=1)			//待机状态
        {
            System.status = SYSTEM_STANDBY;
            App_printf("waiting state...\r\n");
        }
        else	if( System.emission == 0x01 )			//发射状态
        {
            if( System.modify_power == 0x01 )  //1:正在增减功率
            {
                if( n >= 10 )	//1000ms更新一次
                {
                    n = 0;
                    App_printf("modify power...\r\n");
                }
            }
            else	if(System.achieve_add_sub == 1)		//增加功率完成
            {
                System.achieve_add_sub = 0;
                System.modify_power = 0;   //1:正在增减功率
                System.cancel_add_sub = 0;
                System.modify_power = 0;
                System.status = SYSTEM_EMISSING;
                App_printf("power adjust successfully\r\n");
            }
            else	if( Add_PowerBack.results[0] == 0xFC )	//增加功率失败
            {
                Add_PowerBack.results[0] = 0x00;
                System.achieve_add_sub = 0;
                System.modify_power = 0;
                System.cancel_add_sub = 0;
                System.modify_power = 0;

                if( get_history_alarm() == 0x01 )
                {
                    System.status = SYSTEM_ALARM;
                }
                else
                {
                    System.status = SYSTEM_EMISSING;
                }
                App_printf("power add fail\r\n");
            }
            else	if( Sub_PowerBack.results[0] == 0xFC )	//减小功率失败
            {
                Sub_PowerBack.results[0] = 0x00;
                System.achieve_add_sub = 0;
                System.modify_power = 0;
                System.cancel_add_sub = 0;
                System.modify_power = 0;

                if( get_history_alarm() == 0x01 )
                {
                    System.status = SYSTEM_ALARM;
                }
                else
                {
                    System.status = SYSTEM_EMISSING;
                }
                App_printf("power sub fail\r\n");
            }
            else
            {
                if( n >= 10 )	//1000ms更新一次
                {
                    n = 0;
                    App_printf("working state\r\n");
                }

                if( System.mode == 1 )					//清零
                {
                    memset(System.freq2, 0, 8);
                    memset(System.freq3, 0, 8);
                }
                else	if( System.mode == 2 )
                {
                    memset(System.freq3, 0, 8);
                }
            }
        }
    }
    else	if( System.status == SYSTEM_ALARM )			//报警状态
    {
        if(get_history_alarm()==1)						//历史报警
        {
            System.sweep = 0;
            System.achieve_add_sub = 0;
            System.status = SYSTEM_ALARM;

            if( n >= 10 )	//1000ms更新一次
            {
                n = 0;
                App_printf("history 2 alarm!\r\n");
            }

            if( System.mode == 0 )					//清零
            {
                memset(System.freq1, 0, 8);
                System.freq1[8] = 0;
                memset(System.freq2, 0, 8);
                System.freq2[8] = 0;
                memset(System.freq3, 0, 8);
                System.freq3[8] = 0;
            }
            else	if( System.mode == 1 )
            {
                memset(System.freq2, 0, 8);
                System.freq2[8] = 0;
                memset(System.freq3, 0, 8);
                System.freq3[8] = 0;
            }
            else	if( System.mode == 2 )
            {
                memset(System.freq3, 0, 8);
                System.freq3[8] = 0;
            }
        }
        else	if( System.Voltage >= 40.0 )			//待机状态
        {
            System.status = SYSTEM_STANDBY;
            App_printf("waiting state...\r\n");
        }
        else
        {
            System.open = 0x00;
            System.status = SYSTEM_SHUTDOWN;
            App_printf("3system close !\r\n");
        }
    }
    else												//保留
    {
        App_printf("PC inquire when System.status = ???\r\n");
    }
}

/**************************************************
*p_func_code 是返回指向功能码的指针
 p_PC_data是传入的串口的数据
*返回1，数据解析成功，并且参数正确
*返回2，数据不完整，不是05 02开头，或不是03 04结尾
*返回3，数据解析成功，但是发射机ID不匹配
*返回4，没有这个功能码
*返回5，CRC解析错误
***************************************************/
uint8_t CAN_data_analyze(uint8_t *p_func_code, Buffer_t *p_PC_data)
{
    uint8_t i = 0;
    uint8_t err_code;
    App_printf("\r\n=====================================\r\n");		//进入解析
    W5500_data_t W5500_data_buf;

    err_code = judge_is_valid_can_data(p_PC_data);//判断数据正确性
    if( err_code == 0x00 )
        return 0x02;

#if	STM32_W5500_EN
    W5500_data_buf  = Get_CAN_Data(p_PC_data);
#else

#endif

    if(W5500_data_buf.eff_sign == 0)
        return 5;
		
    App_printf("CAN_data_buf.Func_code	= %02X%02X\r\n", W5500_data_buf.Func_code[0],W5500_data_buf.Func_code[1]);
	/*提取功能码两位*/
    p_func_code[0] = W5500_data_buf.Func_code[0];
    p_func_code[1] = W5500_data_buf.Func_code[1];

    App_printf(	"CAN_Specific Data	= ");
    for(i = 0; i<(W5500_data_buf.len-3); i++)
    {
        App_printf("%02X ",W5500_data_buf.data_buf[i]);
    }
    App_printf("\r\n=====================================\r\n");


    err_code = Data_Storage(&W5500_data_buf);//传入数据为：长度(有效数据长度，数组第四位)及有效数据,CANid,有效数据标志位
    return err_code;
}


/* 判断是否为有效CAN数据 */
uint8_t judge_is_valid_can_data(Buffer_t *buffer)
{
#if	!STM32_W5500_EN
    if( buffer->data[0] != 0x05 )
        return 0;

    if( buffer->data[1] != 0x02 )
        return 0;
#endif

    //05 02 03 05 08 01 00 03 00 21 ca 03 05 02 03 05 04
    //03 04结尾，或者03 xx xx xx xx 04结尾(中间插了0502xxxx帧头)
    if( (buffer->data[buffer->len-2] != 0x03) && (buffer->data[buffer->len-6] != 0x03) )
        return 0;

    if( buffer->data[buffer->len-1] != 0x04 )
        return 0;

    return 1;
}

/* 去帧尾，校验并且返回数据 */
W5500_data_t Get_CAN_Data(Buffer_t *p_PC_data)
{
	uint8_t i = 0;
    int ReceiveData_CRC;
    unsigned char buf_CRC[2];
    unsigned int CRC_Back;
    unsigned char Vaild[128];
    W5500_data_t CAN_data;

    CAN_data.CAN_ID = p_PC_data->data[2];//发送数据包中的ID号

    CAN_data.Func_code[0] = p_PC_data->data[4];//功能码所在区域
    CAN_data.Func_code[1] = p_PC_data->data[5];

#if	1
    CAN_data.len = p_PC_data->data[3];//发送数据包中数据部分的长度(功能码开始到CRC检验位前结束)
    for(i=0;i< CAN_data.len-3;i++)
	{
		CAN_data.data_buf[i]=p_PC_data->data[7+i];//保存帧内数据部分
	}	
    buf_CRC[0] = p_PC_data->data[p_PC_data->len-4];
    buf_CRC[1] = p_PC_data->data[p_PC_data->len-3];
#else
    while( 1 )
    {
        //02 01 00 05 5b e5 85 95 03 04 03 04 校验和等于0304
        if( (p_PC_data->data[i+3]==0x03) && (p_PC_data->data[i+4]==0x04) )
        {
            break;
        }
        else
        {
            CAN_data.data_buf[i] = p_PC_data->data[i+3];
            i++;
        }
    }

    CAN_data.len = i+3-2;		//校验和不算数据，所以减2
    buf_CRC[0] = p_PC_data->data[i+1];
    buf_CRC[1] = p_PC_data->data[i+2];
#endif

    ReceiveData_CRC = ((buf_CRC[0]<<8) +(buf_CRC[1]));
    for(i=0;i<CAN_data.len;i++)
	{
		Vaild[i]=p_PC_data->data[4+i];//第五位开始是数据部分(功能码开始)
	}
    CRC_Back = CRC16_XMODEM(Vaild, CAN_data.len);

    if(CRC_Back == ReceiveData_CRC)
    {
        CAN_data.eff_sign = 1;
    }
    else
    {
        CAN_data.eff_sign = 0;//标志结构体里面是数据无效
        App_printf("CRC错误\r\n");
    }

    return CAN_data;
}

/* 数据赋值更新 */
uint8_t Data_Storage(W5500_data_t *CAN_data_cmp)
{
    uint8_t temp = 0;

    if(CAN_data_cmp->Func_code[0] == 0x01 && CAN_data_cmp->Func_code[1] == 0x01)		//PC下发报警参数阈值
    {
				memcpy(System.CAN_ID, 						CAN_data_cmp->data_buf, 	1);
        memcpy(Alarm_threshold.Transmitte_id, 		CAN_data_cmp->data_buf,     1);
        memcpy(Alarm_threshold.Low_temp_limit,		CAN_data_cmp->data_buf+1,	4);
        memcpy(Alarm_threshold.Upp_temp_limit,		CAN_data_cmp->data_buf+5,	4);
        memcpy(Alarm_threshold.Low_humidity_limit,	CAN_data_cmp->data_buf+9,	4);
        memcpy(Alarm_threshold.Upp_humidity_limit,	CAN_data_cmp->data_buf+13,	4);
        memcpy(Alarm_threshold.Low_45I_limit,		CAN_data_cmp->data_buf+17,	4);
        memcpy(Alarm_threshold.Upp_45I_limit,		CAN_data_cmp->data_buf+21,	4);
        memcpy(Alarm_threshold.Low_45V_limit,		CAN_data_cmp->data_buf+25,	4);
        memcpy(Alarm_threshold.Upp_45V_limit,		CAN_data_cmp->data_buf+29,	4);
        			  
        return 1;
    }
    else if(CAN_data_cmp->Func_code[0] == 0x02 && CAN_data_cmp->Func_code[1] == 0x01)	//PC下发状态查询
    {
        if( System.CAN_ID[0] == 0xFF )				//Flash读取CAN_ID错误(修改了flash位置，会一直显示CAN_ID匹配失败，自适应CAN_ID)
        {
            System.CAN_ID[0] = CAN_data_cmp->data_buf[0];
            Work_paraBack.Transmitte_id[0] = CAN_data_cmp->data_buf[0];	//为了不让PC显示掉线，现在的状态该是啥就啥，但是返回的是未初始化
            return 3;
        }
        else	if( System.CAN_ID[0] == 0x00 )		//初始化CAN_ID失败(会一直显示CAN_ID匹配失败，自适应CAN_ID)
        {
            System.CAN_ID[0] = CAN_data_cmp->data_buf[0];
            Work_paraBack.Transmitte_id[0] = CAN_data_cmp->data_buf[0];	//为了不让PC显示掉线，现在的状态该是啥就啥，但是返回的是未初始化
            return 3;
        }
        else
        {
            if(CAN_data_cmp->data_buf[0] == System.CAN_ID[0])   //第一个字节表示发射机ID，必须判断与自身是否相同
            {
                //时间戳	大小端转换
                memcpy(&temp, CAN_data_cmp->data_buf+1, 1);
                g_inquire_stamp = temp<<24;
                memcpy(&temp, CAN_data_cmp->data_buf+2, 1);
                g_inquire_stamp += temp<<16;
                memcpy(&temp, CAN_data_cmp->data_buf+3, 1);
                g_inquire_stamp += temp<<8;
                memcpy(&temp, CAN_data_cmp->data_buf+4, 1);
                g_inquire_stamp += temp;
                return 1;
            }
            else
            {
                Work_paraBack.Transmitte_id[0] = CAN_data_cmp->data_buf[0];	//为了不让PC显示掉线，现在的状态该是啥就啥，但是返回的是未初始化
                return 3;
            }
        }
    }
    else if(CAN_data_cmp->Func_code[0] == 0x03 && CAN_data_cmp->Func_code[1] == 0x01)	//开启发射机
    {
        if(CAN_data_cmp->data_buf[0] == System.CAN_ID[0])   //第一个字节表示发射机ID，必须判断与自身是否相同
        {
            PC_Cmd.mode=CAN_data_cmp->data_buf[1];		//工作模式
            memcpy(PC_Cmd.channel,		CAN_data_cmp->data_buf+2,	3);		//信道	新协议只有一个信道，即存三个，用一个(0x01)
            memcpy(PC_Cmd.freq,			CAN_data_cmp->data_buf+5,	12);	//频率
            memcpy(PC_Cmd.power,		CAN_data_cmp->data_buf+17,	3);		//功率等级
            memcpy(System.time,			CAN_data_cmp->data_buf+20,	6);		//北斗时间
            return 1;
        }
        else
        {
            return 3;
        }
    }
    else if(CAN_data_cmp->Func_code[0] == 0x04 && CAN_data_cmp->Func_code[1] == 0x01)	//关闭发射机
    {
        if(CAN_data_cmp->data_buf[0] == System.CAN_ID[0])   //第一个字节表示发射机ID，必须判断与自身是否相同
        {
            memcpy(System.time,	CAN_data_cmp->data_buf+1,	6);	//北斗时间
            return 1;
        }
        else
        {
            return 3;
        }
    }
    else if(CAN_data_cmp->Func_code[0] == 0x05 && CAN_data_cmp->Func_code[1] == 0x01)	//取消报警
    {
        if(CAN_data_cmp->data_buf[0] == System.CAN_ID[0])   //第一个字节表示发射机ID，必须判断与自身是否相同
        {
            memcpy(Disalarm.Disalarm_type, CAN_data_cmp->data_buf+1,2);//解除报警类型
            return 1;
        }
        else
        {
            return 3;
        }
    }
    else if(CAN_data_cmp->Func_code[0] == 0x06 && CAN_data_cmp->Func_code[1] == 0x01)	//查询历史报警状态
    {
        if(CAN_data_cmp->data_buf[0] == System.CAN_ID[0])   //第一个字节表示发射机ID，必须判断与自身是否相同
        {
            memcpy(System.time,	CAN_data_cmp->data_buf+1,	6);//北斗时间
            return 1;
        }
        else
        {
            return 3;
        }
    }
    else if(CAN_data_cmp->Func_code[0] == 0x07 && CAN_data_cmp->Func_code[1] == 0x01)	//发射机上电
    {
        if(CAN_data_cmp->data_buf[0] == System.CAN_ID[0])   //第一个字节表示发射机ID，必须判断与自身是否相同
        {
//            memcpy(Power_on.Retain,CAN_data_cmp->data_buf+1,	1);			//保留
            memcpy(System.time,	CAN_data_cmp->data_buf+2,	6);		//北斗时间
            return 1;
        }
        else
        {
            return 3;
        }
    }
    else if(CAN_data_cmp->Func_code[0] == 0x08 && CAN_data_cmp->Func_code[1] == 0x01)	//发射机断电
    {
        if(CAN_data_cmp->data_buf[0] == System.CAN_ID[0])   //第一个字节表示发射机ID，必须判断与自身是否相同
        {
//            memcpy(Power_off.Retain,CAN_data_cmp->data_buf+1,1);	//保留
            return 1;
        }
        else
        {
            return 3;
        }
    }
    else if(CAN_data_cmp->Func_code[0] == 0x09 && CAN_data_cmp->Func_code[1] == 0x01)	//扫描最佳工作频段
    {
        if(CAN_data_cmp->data_buf[0] == System.CAN_ID[0])   //第一个字节表示发射机ID，必须判断与自身是否相同
        {
            memcpy(Scan_Frq.state,CAN_data_cmp->data_buf+1,1);		//扫频/查频
            memcpy(Scan_Frq.Fre_Band,CAN_data_cmp->data_buf+2,8);	//需要查询的频段
            return 1;
        }
        else
        {
            return 3;
        }
    }
    else if(CAN_data_cmp->Func_code[0] == 0x0A && CAN_data_cmp->Func_code[1] == 0x01)	//停止扫频
    {
        if(CAN_data_cmp->data_buf[0] == System.CAN_ID[0])   //第一个字节表示发射机ID，必须判断与自身是否相同
        {
            memcpy(Sacn_stop.state,CAN_data_cmp->data_buf+1,1);
            return 1;
        }
        else
        {
            return 3;
        }
    }
    else if(CAN_data_cmp->Func_code[0] == 0x0B && CAN_data_cmp->Func_code[1] == 0x01)	//增加功率
    {
        if(CAN_data_cmp->data_buf[0] == System.CAN_ID[0])   //第一个字节表示发射机ID，必须判断与自身是否相同
        {
            memcpy(Add_Power.Power_UP,CAN_data_cmp->data_buf+1,2);
            return 1;
        }
        else
        {
            return 3;
        }
    }
    else if(CAN_data_cmp->Func_code[0] == 0x0C && CAN_data_cmp->Func_code[1] == 0x01)	//减小功率
    {
        if(CAN_data_cmp->data_buf[0] == System.CAN_ID[0])   //第一个字节表示发射机ID，必须判断与自身是否相同
        {
            memcpy(Sub_Power.Power_DOWN,CAN_data_cmp->data_buf+1,2);
            return 1;
        }
        else
        {
            return 3;
        }
    }
    else if(CAN_data_cmp->Func_code[0] == 0x0D && CAN_data_cmp->Func_code[1] == 0x01)	//运行图
    {
        if(CAN_data_cmp->data_buf[0] == System.CAN_ID[0])   //第一个字节表示发射机ID，必须判断与自身是否相同
        {
//            memcpy(Run_Diagram.count,	CAN_data_cmp->data_buf+1,1);
					  Run_Diagram.count[0]=CAN_data_cmp->data_buf[1];
//            memcpy(Run_Diagram.Continue,CAN_data_cmp->data_buf+2,2);
						Run_Diagram.Continue[0]=CAN_data_cmp->data_buf[2];
						Run_Diagram.Continue[1]=CAN_data_cmp->data_buf[3];
            memcpy(Run_Diagram.Start_Time1,	CAN_data_cmp->data_buf+4,2);
            memcpy(Run_Diagram.End_Time1,	CAN_data_cmp->data_buf+6,2);
            memcpy(Run_Diagram.Power1,		CAN_data_cmp->data_buf+8,1);
            memcpy(Run_Diagram.Frq1,		CAN_data_cmp->data_buf+9,4);
//            memcpy(Run_Diagram.channel1,	CAN_data_cmp->data_buf+13,1);

            memcpy(Run_Diagram.Start_Time2,	CAN_data_cmp->data_buf+14,2);
            memcpy(Run_Diagram.End_Time2,	CAN_data_cmp->data_buf+16,2);
            memcpy(Run_Diagram.Power2,		CAN_data_cmp->data_buf+18,1);
            memcpy(Run_Diagram.Frq2,		CAN_data_cmp->data_buf+19,4);
//            memcpy(Run_Diagram.channel2,	CAN_data_cmp->data_buf+23,1);

            memcpy(Run_Diagram.Start_Time3,	CAN_data_cmp->data_buf+24,2);
            memcpy(Run_Diagram.End_Time3,	CAN_data_cmp->data_buf+26,2);
            memcpy(Run_Diagram.Power3,		CAN_data_cmp->data_buf+28,1);
            memcpy(Run_Diagram.Frq3,		CAN_data_cmp->data_buf+29,4);
//            memcpy(Run_Diagram.channel3,	CAN_data_cmp->data_buf+33,1);

            memcpy(Run_Diagram.Start_Time4,	CAN_data_cmp->data_buf+34,2);
            memcpy(Run_Diagram.End_Time4,	CAN_data_cmp->data_buf+36,2);
            memcpy(Run_Diagram.Power4,		CAN_data_cmp->data_buf+38,1);
            memcpy(Run_Diagram.Frq4,		CAN_data_cmp->data_buf+39,4);
//            memcpy(Run_Diagram.channel4,	CAN_data_cmp->data_buf+43,1);

            memcpy(Run_Diagram.Start_Time5,	CAN_data_cmp->data_buf+44,2);
            memcpy(Run_Diagram.End_Time5,	CAN_data_cmp->data_buf+46,2);
            memcpy(Run_Diagram.Power5,		CAN_data_cmp->data_buf+48,1);
            memcpy(Run_Diagram.Frq5,		CAN_data_cmp->data_buf+49,4);
//            memcpy(Run_Diagram.channel5,	CAN_data_cmp->data_buf+53,1);

            memcpy(Run_Diagram.Start_Time6,	CAN_data_cmp->data_buf+54,2);
            memcpy(Run_Diagram.End_Time6,	CAN_data_cmp->data_buf+56,2);
            memcpy(Run_Diagram.Power6,		CAN_data_cmp->data_buf+58,1);
            memcpy(Run_Diagram.Frq6,		CAN_data_cmp->data_buf+59,4);
//            memcpy(Run_Diagram.channel6,	CAN_data_cmp->data_buf+63,1);

            memcpy(Run_Diagram.Start_Time7,	CAN_data_cmp->data_buf+64,2);
            memcpy(Run_Diagram.End_Time7,	CAN_data_cmp->data_buf+66,2);
            memcpy(Run_Diagram.Power7,		CAN_data_cmp->data_buf+68,1);
            memcpy(Run_Diagram.Frq7,		CAN_data_cmp->data_buf+69,4);
//            memcpy(Run_Diagram.channel7,	CAN_data_cmp->data_buf+73,1);

            memcpy(Run_Diagram.Start_Time8,	CAN_data_cmp->data_buf+74,2);
            memcpy(Run_Diagram.End_Time8,	CAN_data_cmp->data_buf+76,2);
            memcpy(Run_Diagram.Power8,		CAN_data_cmp->data_buf+78,1);
            memcpy(Run_Diagram.Frq8,		CAN_data_cmp->data_buf+79,4);
//            memcpy(Run_Diagram.channel8,	CAN_data_cmp->data_buf+83,1);

            memcpy(Run_Diagram.Start_Time9,	CAN_data_cmp->data_buf+84,2);
            memcpy(Run_Diagram.End_Time9,	CAN_data_cmp->data_buf+86,2);
            memcpy(Run_Diagram.Power9,		CAN_data_cmp->data_buf+88,1);
            memcpy(Run_Diagram.Frq9,		CAN_data_cmp->data_buf+89,4);
//            memcpy(Run_Diagram.channel9,	CAN_data_cmp->data_buf+93,1);

            memcpy(Run_Diagram.Start_Time10,	CAN_data_cmp->data_buf+94,2);
            memcpy(Run_Diagram.End_Time10,		CAN_data_cmp->data_buf+96,2);
            memcpy(Run_Diagram.Power10,			CAN_data_cmp->data_buf+98,1);
            memcpy(Run_Diagram.Frq10,			CAN_data_cmp->data_buf+99,4);
//            memcpy(Run_Diagram.channel10,		CAN_data_cmp->data_buf+103,1);

            return 1;
        }
        else
        {
            return 3;
        }
    }

    return 4;//没有功能码
}

/********************************************************
		协议包组合
	*Func_code功能码
	*p_data 具体数据，也就是结构体里面的数据(数据包)
	Can_ID是标志发射机编号
	len是p_data的数据长度
	Buffer_t *Buffer 返回的结构体指针
	返回有效数据的长度
********************************************************/
uint8_t Data_Assemble(uint8_t *Func_code, uint8_t *p_data, uint8_t Can_ID, uint8_t len, Buffer_t *Buffer)
{
    uint16_t CRC_Back;
    uint8_t Start_Data[7] = {0x05, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00};
    uint8_t End_Data[4] = {0x00, 0x00, 0x03, 0x04};

    //补零
    uint8_t temp = 0x00;
    uint8_t temp_len = 0x00;

    if( Work_paraBack.Transmitte_id[0] != 0x00 )			//已经初始化，但是MCU和CAN没有接匹配的，错乱了，就按收到的CAN_ID来返回，防止掉线
    {
        Start_Data[2] = Work_paraBack.Transmitte_id[0];		//写入发射机ID		--20181031 Luonus
    }

    if( Start_Data[2] == 0x00 )	
    {
        Start_Data[2] = System.CAN_ID[0];				//写入发射机ID		--20181102 Luonus
    }

    Start_Data[3] = len + 3;			//数据长度
    Start_Data[4] = Func_code[0];		//功能码
    Start_Data[5] = Func_code[1];		//功能码

#if	ADD_CAN_LEN
    Start_Data[6] = (CEIL_DIV(Start_Data[3]+8-12,8))*4 + Start_Data[3] + 8;
#endif

    memcpy(Buffer->data, Start_Data,7);
    memcpy(Buffer->data+7, p_data, len);
    CRC_Back = CRC16_XMODEM(Buffer->data+4, len+3);
    End_Data[0] = (uint8_t)((CRC_Back >> 8) & 0xff);		//存高八位
    End_Data[1] = (uint8_t)((CRC_Back >> 0) & 0xff) ;		//低八位
    memcpy(Buffer->data+len+7, End_Data,4);
    Buffer->len = len+11;

#if	1	//不够8位，补零
    temp = Buffer->len % 8;

    if( temp == 0x00 )
    {

    }
    else
    {
        temp_len = 8-temp;
        memset(Buffer->data+Buffer->len, 0, temp_len);
        Buffer->len += temp_len;
    }
#endif

    return Start_Data[3];
}


/***************************************************
	只需要传递功能码，自动发送到串口5
	若是返回1，说明执行成功
	返回2，说明传递的功能码不存在
	*Func_code_buf指向功能码的指针
****************************************************/
uint8_t Send_PC(uint8_t *Func_code)
{
    uint8_t i = 0;
    uint8_t len = 0;
    uint8_t valid_len = 0;
    Buffer_t COM_buffer;


    if(Func_code[0] ==0x01 && Func_code[1] == 0x02)			//发射机报警参数初始化请求应答
    {
        valid_len = Data_Assemble(Func_code,(uint8_t *)&Alarm_backPC, System.CAN_ID[0], sizeof(Alarm_backPC), &COM_buffer);
        if( valid_len != 0 )
        {
#if	STM32_W5500_EN
            len = COM_buffer.len;
            for(i=0; ; i++)
            {
                if( len >= 0x08 )
                {	
										if(S0_State == 0x03)		//03表示可以正常传输
										{
											S0_Data&=~S_TRANSMITOK;	//处理标志位
											Process_Socket_Data(0,COM_buffer.data+8*i, 8);
										}
                    len = len - 8;
                    if( len == 0x00 )
										{
                       break;
										}
                }
                else
                {
										if(S0_State == 0x03)		//03表示可以正常传输
										{
											S0_Data&=~S_TRANSMITOK;	//处理标志位
											Process_Socket_Data(0,COM_buffer.data+8*i, len);
										}
                    break;
                }
            }
#else
            comSendBuf(COM5,COM_buffer.data, COM_buffer.len);
#endif

            App_printf("COM_buffer: ");
            for(i=0; i<COM_buffer.len; i++)
            {
                App_printf("%02x ",COM_buffer.data[i]);
            }
            App_printf("\r\n");
            return 1;
        }
    }
    else	if(Func_code[0] ==0x02 && Func_code[1] == 0x02)	//查询
    {
        valid_len = Data_Assemble(Func_code, (uint8_t *)&Work_paraBack, System.CAN_ID[0], sizeof(Work_paraBack), &COM_buffer);
        if( valid_len != 0 )
        {
#if	STM32_W5500_EN
            len = COM_buffer.len;
            for(i=0; ; i++)
            {
                if( len >= 0x08 )
                {
 					if(S0_State == 0x03)		//03表示可以正常传输
					{
						S0_Data&=~S_TRANSMITOK;	//处理标志位
						Process_Socket_Data(0,COM_buffer.data+8*i, 8);
					}
                    len = len - 8;
                    if( len == 0x00 )
                        break;
                }
                else
                {
					if(S0_State == 0x03)		//03表示可以正常传输
					{
						S0_Data&=~S_TRANSMITOK;	//处理标志位
						Process_Socket_Data(0,COM_buffer.data+8*i, len);
					}
                    break;
                }
            }
#else
            comSendBuf(COM5,COM_buffer.data, COM_buffer.len);
#endif

            App_printf("COM_buffer: ");
            for(i=0; i<COM_buffer.len; i++)
            {
                App_printf("%02x ",COM_buffer.data[i]);
            }
            App_printf("\r\n");
            return 1;
        }
    }
    else	if(Func_code[0] ==0x03 && Func_code[1] == 0x02)	//发射
    {
        valid_len = Data_Assemble(Func_code, (uint8_t *)&Trans_openBack, System.CAN_ID[0], sizeof(Trans_openBack), &COM_buffer);
        if( valid_len != 0)
        {
#if	STM32_W5500_EN
            len = COM_buffer.len;
            for(i=0; ; i++)
            {
                if( len >= 0x08 )
                {
					if(S0_State == 0x03)		//03表示可以正常传输
					{
						S0_Data&=~S_TRANSMITOK;	//处理标志位
						Process_Socket_Data(0,COM_buffer.data+8*i, 8);
					}
                    len = len - 8;
                    if( len == 0x00 )
                        break;
                }
                else
                {
					if(S0_State == 0x03)		//03表示可以正常传输
					{
						S0_Data&=~S_TRANSMITOK;	//处理标志位
						Process_Socket_Data(0,COM_buffer.data+8*i, len);
					}
                    break;
                }
            }
#else
            comSendBuf(COM5,COM_buffer.data, COM_buffer.len);
#endif

            App_printf("COM_buffer: ");
            for(i=0; i<COM_buffer.len; i++)
            {
                App_printf("%02x ",COM_buffer.data[i]);
            }
            App_printf("\r\n");
            return 1;
        }
    }
    else	if(Func_code[0] ==0x04 && Func_code[1] == 0x02)	//停止
    {
        valid_len = Data_Assemble(Func_code, (uint8_t *)&Trans_stopBack, System.CAN_ID[0], sizeof(Trans_stopBack), &COM_buffer);
        if( valid_len != 0 )
        {
#if	STM32_W5500_EN
            len = COM_buffer.len;
            for(i=0; ; i++)
            {
                if( len >= 0x08 )
                {
					if(S0_State == 0x03)		//03表示可以正常传输
					{
						S0_Data&=~S_TRANSMITOK;	//处理标志位
						Process_Socket_Data(0,COM_buffer.data+8*i, 8);
					}
                    len = len - 8;
                    if( len == 0x00 )
                        break;
                }
                else
                {
					if(S0_State == 0x03)		//03表示可以正常传输
					{
						S0_Data&=~S_TRANSMITOK;	//处理标志位
						Process_Socket_Data(0,COM_buffer.data+8*i, len);
					}
                    break;
                }
            }
#else
            comSendBuf(COM5,COM_buffer.data, COM_buffer.len);
#endif

            App_printf("COM_buffer: ");
            for(i=0; i<COM_buffer.len; i++)
            {
                App_printf("%02x ",COM_buffer.data[i]);
            }
            App_printf("\r\n");
            return 1;
        }
    }
    else	if(Func_code[0] ==0x05 && Func_code[1] == 0x02)	//解除报警
    {
        valid_len = Data_Assemble(Func_code, (uint8_t *)&DisalarmBack, System.CAN_ID[0], sizeof(DisalarmBack), &COM_buffer);
        if( valid_len != 0 )
        {
#if	STM32_W5500_EN
            len = COM_buffer.len;
            for(i=0; ; i++)
            {
                if( len >= 0x08 )
                {
					if(S0_State == 0x03)		//03表示可以正常传输
					{
						S0_Data&=~S_TRANSMITOK;	//处理标志位
						Process_Socket_Data(0,COM_buffer.data+8*i, 8);
					}
                    len = len - 8;
                    if( len == 0x00 )
                        break;
                }
                else
                {
					if(S0_State == 0x03)		//03表示可以正常传输
					{
						S0_Data&=~S_TRANSMITOK;	//处理标志位
						Process_Socket_Data(0,COM_buffer.data+8*i, len);
					}
                    break;
                }
            }
#else
            comSendBuf(COM5,COM_buffer.data, COM_buffer.len);
#endif

            App_printf("COM_buffer: ");
            for(i=0; i<COM_buffer.len; i++)
            {
                App_printf("%02x ",COM_buffer.data[i]);
            }
            App_printf("\r\n");
            return 1;
        }
    }
    else	if(Func_code[0] ==0x06 && Func_code[1] == 0x02)	//报警信息查询
    {
        valid_len = Data_Assemble(Func_code, (uint8_t *)&Alarm_historyBack, System.CAN_ID[0], sizeof(Alarm_historyBack), &COM_buffer);
        if( valid_len != 0 )
        {
#if	STM32_W5500_EN
            len = COM_buffer.len;
            for(i=0; ; i++)
            {
                if( len >= 0x08 )
                {
 					if(S0_State == 0x03)		//03表示可以正常传输
					{
						S0_Data&=~S_TRANSMITOK;	//处理标志位
						Process_Socket_Data(0,COM_buffer.data+8*i, 8);
					}
                    len = len - 8;
                    if( len == 0x00 )
                        break;
                }
                else
                {
					if(S0_State == 0x03)		//03表示可以正常传输
					{
						S0_Data&=~S_TRANSMITOK;	//处理标志位
						Process_Socket_Data(0,COM_buffer.data+8*i, len);
					}
                    break;
                }
            }
#else
            comSendBuf(COM5,COM_buffer.data, COM_buffer.len);
#endif

            App_printf("COM_buffer: ");
            for(i=0; i<COM_buffer.len; i++)
            {
                App_printf("%02x ",COM_buffer.data[i]);
            }
            App_printf("\r\n");
            return 1;
        }
    }
    else	if(Func_code[0] ==0x07 && Func_code[1] == 0x02)	//开机
    {
        valid_len = Data_Assemble(Func_code, (uint8_t *)&Power_onBack, System.CAN_ID[0], sizeof(Power_onBack), &COM_buffer);
        if( valid_len != 0 )
        {
#if	STM32_W5500_EN
            len = COM_buffer.len;
            for(i=0; ; i++)
            {
                if( len >= 0x08 )
                {
					if(S0_State == 0x03)		//03表示可以正常传输
					{
						S0_Data&=~S_TRANSMITOK;	//处理标志位
						Process_Socket_Data(0,COM_buffer.data+8*i, 8);
					}
                    len = len - 8;
                    if( len == 0x00 )
                        break;
                }
                else
                {
					if(S0_State == 0x03)		//03表示可以正常传输
					{
						S0_Data&=~S_TRANSMITOK;	//处理标志位
						Process_Socket_Data(0,COM_buffer.data+8*i, len);
					}
                    break;
                }
            }
#else
            comSendBuf(COM5,COM_buffer.data, COM_buffer.len);
#endif

            App_printf("COM_buffer: ");
            for(i=0; i<COM_buffer.len; i++)
            {
                App_printf("%02x ",COM_buffer.data[i]);
            }
            App_printf("\r\n");
            return 1;
        }
    }
    else	if(Func_code[0] ==0x08 && Func_code[1] == 0x02)	//关机
    {
        valid_len = Data_Assemble(Func_code, (uint8_t *)&Power_offBack, System.CAN_ID[0], sizeof(Power_offBack), &COM_buffer);
        if( valid_len != 0 )
        {
#if	STM32_W5500_EN
            len = COM_buffer.len;
            for(i=0; ; i++)
            {
                if( len >= 0x08 )
                {
					if(S0_State == 0x03)		//03表示可以正常传输
					{
						S0_Data&=~S_TRANSMITOK;	//处理标志位
						Process_Socket_Data(0,COM_buffer.data+8*i, 8);
					}
                    len = len - 8;
                    if( len == 0x00 )
                        break;
                }
                else
                {
					if(S0_State == 0x03)		//03表示可以正常传输
					{
						S0_Data&=~S_TRANSMITOK;	//处理标志位
						Process_Socket_Data(0,COM_buffer.data+8*i, len);
					}
                    break;
                }
            }
#else
            comSendBuf(COM5,COM_buffer.data, COM_buffer.len);
#endif

            App_printf("COM_buffer: ");
            for(i=0; i<COM_buffer.len; i++)
            {
                App_printf("%02x ",COM_buffer.data[i]);
            }
            App_printf("\r\n");
            return 1;
        }
    }
    else	if(Func_code[0] ==0x09 && Func_code[1] == 0x02)	//扫频
    {
        App_printf("COM_buffer sweeping: \r\n");
        switch(System.Scan_Freq_flag)
		{
			case 0:
			{
				valid_len = Data_Assemble(Func_code, (uint8_t *)&Scan_FrqBack, System.CAN_ID[0], sizeof(Scan_FrqBack), &COM_buffer);
				if( valid_len == 0 )
				{
					return 2;
				}

				break;
			}

			case 1:
			{
				valid_len = Data_Assemble(Func_code, (uint8_t *)&Scan_FrqBack1, System.CAN_ID[0], sizeof(Scan_FrqBack1), &COM_buffer);
				if( valid_len == 0 )
				{
					return 2;
				}

				break;
			}

			case 2:
			{
				valid_len = Data_Assemble(Func_code, (uint8_t *)&Scan_FrqBack2, System.CAN_ID[0], sizeof(Scan_FrqBack2), &COM_buffer);
				if( valid_len == 0 )
				{
					return 2;
				}

				break;
			}

			case 3:
			{
				valid_len = Data_Assemble(Func_code, (uint8_t *)&Scan_FrqBack3, System.CAN_ID[0], sizeof(Scan_FrqBack3), &COM_buffer);
				if( valid_len == 0 )
				{
					return 2;
				}

				break;
			}

			case 4:
			{
				valid_len = Data_Assemble(Func_code, (uint8_t *)&Scan_FrqBack4, System.CAN_ID[0], sizeof(Scan_FrqBack4), &COM_buffer);
				if( valid_len == 0 )
				{
					return 2;
				}

				break;
			}

			case 5:
			{
				valid_len = Data_Assemble(Func_code, (uint8_t *)&Scan_FrqBack5, System.CAN_ID[0], sizeof(Scan_FrqBack5), &COM_buffer);
				if( valid_len == 0 )
				{
					return 2;
				}

				break;
			}

			case 6:
			{
				valid_len = Data_Assemble(Func_code, (uint8_t *)&Scan_FrqBack6, System.CAN_ID[0], sizeof(Scan_FrqBack6), &COM_buffer);
				if( valid_len == 0 )
				{
					return 2;
				}

				break;
			}

			case 7:
			{
				valid_len = Data_Assemble(Func_code, (uint8_t *)&Scan_FrqBack7, System.CAN_ID[0],sizeof(Scan_FrqBack7), &COM_buffer);
				if( valid_len == 0 )
				{
					return 2;
				}

				break;
			}

			case 8:
			{
				valid_len = Data_Assemble(Func_code, (uint8_t *)&Scan_FrqBack8, System.CAN_ID[0], sizeof(Scan_FrqBack8), &COM_buffer);
				if( valid_len == 0 )
				{
					return 2;
				}

				break;
			}

			case 9:
			{
				valid_len = Data_Assemble(Func_code, (uint8_t *)&Scan_FrqBack9, System.CAN_ID[0], sizeof(Scan_FrqBack9), &COM_buffer);
				if( valid_len == 0 )
				{
					return 2;
				}

				break;
			}

			case 10:
			{
				valid_len = Data_Assemble(Func_code, (uint8_t *)&Scan_FrqBack10, System.CAN_ID[0], sizeof(Scan_FrqBack10), &COM_buffer);
				if( valid_len == 0 )
				{
					return 2;
				}

				break;
			}

			case 11:
			{
				valid_len = Data_Assemble(Func_code, (uint8_t *)&Scan_FrqBack11, System.CAN_ID[0], sizeof(Scan_FrqBack11), &COM_buffer);
				if( valid_len == 0 )
				{
					return 2;
				}

				break;
			}

			case 12:
			{
				valid_len = Data_Assemble(Func_code, (uint8_t *)&Scan_FrqBack12, System.CAN_ID[0], sizeof(Scan_FrqBack12), &COM_buffer);
				if( valid_len == 0 )
				{
					return 2;
				}

				break;
			}
        }

#if	STM32_W5500_EN
        len = COM_buffer.len;
        for(i=0; ; i++)
        {
            if( len >= 0x08 )
            {
                	if(S0_State == 0x03)		//03表示可以正常传输
					{
						S0_Data&=~S_TRANSMITOK;	//处理标志位
						Process_Socket_Data(0,COM_buffer.data+8*i, 8);
					}
                len = len - 8;
                if( len == 0x00 )
                    break;
            }
            else
            {
                	if(S0_State == 0x03)		//03表示可以正常传输
					{
						S0_Data&=~S_TRANSMITOK;	//处理标志位
						Process_Socket_Data(0,COM_buffer.data+8*i, len);
					}
                break;
            }
        }
#else
        comSendBuf(COM5,COM_buffer.data, COM_buffer.len);
#endif

        App_printf("Scan_COM_buffer: ");
        for(i=0; i<COM_buffer.len; i++)
        {
            App_printf("%02x ",COM_buffer.data[i]);
        }
        App_printf("\r\n");
        return 1;
    }
    else	if(Func_code[0] ==0x0A && Func_code[1] == 0x02)	//停止扫频
    {
        valid_len = Data_Assemble(Func_code, (uint8_t *)&Sacn_stopBack, System.CAN_ID[0], sizeof(Sacn_stopBack), &COM_buffer);
        if( valid_len != 0 )
        {
#if	STM32_W5500_EN
            len = COM_buffer.len;
            for(i=0; ; i++)
            {
                if( len >= 0x08 )
                {
					if(S0_State == 0x03)		//03表示可以正常传输
					{
						S0_Data&=~S_TRANSMITOK;	//处理标志位
						Process_Socket_Data(0,COM_buffer.data+8*i, 8);
					}
                    len = len - 8;
                    if( len == 0x00 )
                        break;
                }
                else
                {
					if(S0_State == 0x03)		//03表示可以正常传输
					{
						S0_Data&=~S_TRANSMITOK;	//处理标志位
						Process_Socket_Data(0,COM_buffer.data+8*i, len);
					}
                    break;
                }
            }
#else
            comSendBuf(COM5,COM_buffer.data, COM_buffer.len);
#endif

            App_printf("COM_buffer: ");
            for(i=0; i<COM_buffer.len; i++)
            {
                App_printf("%02x ",COM_buffer.data[i]);
            }
            App_printf("\r\n");
            return 1;
        }
    }
    else	if(Func_code[0] ==0x0B && Func_code[1] == 0x02)	//增加功率
    {
        valid_len = Data_Assemble(Func_code, (uint8_t *)&Add_PowerBack, System.CAN_ID[0], sizeof(Add_PowerBack), &COM_buffer);
        if( valid_len != 0 )
        {
#if	STM32_W5500_EN
            len = COM_buffer.len;
            for(i=0; ; i++)
            {
                if( len >= 0x08 )
                {
 					if(S0_State == 0x03)		//03表示可以正常传输
					{
						S0_Data&=~S_TRANSMITOK;	//处理标志位
						Process_Socket_Data(0,COM_buffer.data+8*i, 8);
					}
                    len = len - 8;
                    if( len == 0x00 )
                        break;
                }
                else
                {
					if(S0_State == 0x03)		//03表示可以正常传输
					{
						S0_Data&=~S_TRANSMITOK;	//处理标志位
						Process_Socket_Data(0,COM_buffer.data+8*i, len);
					}
                    break;
                }
            }
#else
            comSendBuf(COM5,COM_buffer.data, COM_buffer.len);
#endif

            App_printf("COM_buffer: ");
            for(i=0; i<COM_buffer.len; i++)
            {
                App_printf("%02x ",COM_buffer.data[i]);
            }
            App_printf("\r\n");
            return 1;
        }
    }
    else	if(Func_code[0] ==0x0C && Func_code[1] == 0x02)	//减小功率
    {
        valid_len = Data_Assemble(Func_code, (uint8_t *)&Sub_PowerBack, System.CAN_ID[0], sizeof(Sub_PowerBack), &COM_buffer);
        if( valid_len != 0 )
        {
#if	STM32_W5500_EN
            len = COM_buffer.len;
            for(i=0; ; i++)
            {
                if( len >= 0x08 )
                {
					if(S0_State == 0x03)		//03表示可以正常传输
					{
						S0_Data&=~S_TRANSMITOK;	//处理标志位
						Process_Socket_Data(0,COM_buffer.data+8*i, 8);
					}
                    len = len - 8;
                    if( len == 0x00 )
                        break;
                }
                else
                {
					if(S0_State == 0x03)		//03表示可以正常传输
					{
						S0_Data&=~S_TRANSMITOK;	//处理标志位
						Process_Socket_Data(0,COM_buffer.data+8*i, len);
					}
                    break;
                }
            }
#else
            comSendBuf(COM5,COM_buffer.data, COM_buffer.len);
#endif

            App_printf("COM_buffer: ");
            for(i=0; i<COM_buffer.len; i++)
            {
                App_printf("%02x ",COM_buffer.data[i]);
            }
            App_printf("\r\n");
            return 1;
        }
    }
    else	if(Func_code[0] ==0x0D && Func_code[1] == 0x02)	//运行图
    {
        valid_len = Data_Assemble(Func_code, (uint8_t *)&Run_DiagramBack, System.CAN_ID[0], sizeof(Run_DiagramBack), &COM_buffer);
        if( valid_len != 0 )
        {
#if	STM32_W5500_EN
            len = COM_buffer.len;
            for(i=0; ; i++)
            {
                if( len >= 0x08 )
                {
					if(S0_State == 0x03)		//03表示可以正常传输
					{
						S0_Data&=~S_TRANSMITOK;	//处理标志位
						Process_Socket_Data(0,COM_buffer.data+8*i, 8);
					}
                    len = len - 8;
                    if( len == 0x00 )
                        break;
                }
                else
                {
					if(S0_State == 0x03)		//03表示可以正常传输
					{
						S0_Data&=~S_TRANSMITOK;	//处理标志位
						Process_Socket_Data(0,COM_buffer.data+8*i, len);
					}
                    break;
                }
            }
#else
            comSendBuf(COM5,COM_buffer.data, COM_buffer.len);
#endif

            App_printf("COM_buffer: ");
            for(i=0; i<COM_buffer.len; i++)
            {
                App_printf("%02x ",COM_buffer.data[i]);
            }
            App_printf("\r\n");
            return 1;
        }
    }
    return 2;
}

/*---------------------------------扫频断点保存与读取-------------------------------------*/
//读取扫频断点，并返回下一个频点
uint16_t Scan_Breakpoint_Read(void)
{
	uint8_t		buf[2];
	uint16_t	next_point = 0;
	
	FLASH_Read(SCAN_BREAKPOINT_ADDRESS,		buf,	SCAN_BREAKPOINT_LEN);
	
	next_point = 10*buf[0] + buf[1];
	return next_point;
}

//获取扫频断点
void Scan_Breakpoint_Save(uint8_t current_point_int, uint8_t current_point_dec)
{
	uint8_t p_data[SCAN_BREAKPOINT_LEN];
	uint8_t next_point_int = 0;
	uint8_t next_point_dec = 0;

	if( (current_point_int > 26) && (current_point_dec == 0) )
	{
		next_point_int = 5;
		next_point_dec = 1;
	}
	else	if( (current_point_int > 26) && (current_point_dec == 1) )
	{
		next_point_int = 5;
		next_point_dec = 2;
	}
	else	if( (current_point_int > 25) && (current_point_dec == 2) )
	{
		next_point_int = 5;
		next_point_dec = 3;
	}
	else	if( (current_point_int > 25) && (current_point_dec == 3) )
	{
		next_point_int = 5;
		next_point_dec = 4;
	}
	else	if( (current_point_int > 25) && (current_point_dec == 4) )
	{
		next_point_int = 5;
		next_point_dec = 5;
	}
	else	if( (current_point_int > 25) && (current_point_dec == 5) )
	{
		next_point_int = 5;
		next_point_dec = 6;
	}
	else	if( (current_point_int > 25) && (current_point_dec == 6) )
	{
		next_point_int = 5;
		next_point_dec = 7;
	}
	else	if( (current_point_int > 25) && (current_point_dec == 7) )
	{
		next_point_int = 5;
		next_point_dec = 8;
	}
	else	if( (current_point_int > 25) && (current_point_dec == 8) )
	{
		next_point_int = 5;
		next_point_dec = 9;
	}
	else	if( (current_point_int > 25) && (current_point_dec == 9) )
	{
		next_point_int = 5;
		next_point_dec = 0;
	}
	else
	{
		next_point_int = current_point_int;
		next_point_dec = current_point_dec;
	}
	
	p_data[0] = next_point_int;
	p_data[1] = next_point_dec;
	
	Flash_Write(SCAN_BREAKPOINT_ADDRESS, p_data, SCAN_BREAKPOINT_LEN);
}


/*------------------------------------扫频函数1----------------------------------------*/
void Band_scan(uint8_t freq_begin, uint8_t freq_end, uint8_t index_dec)
{
    int ret = 0,Flag=0,n;
	uint8_t protect_count = 0;		//保护机器
	char freq_scan_to_Tx[4];    //保存十六进制扫频下发的频率数据
	unsigned char test_to_Tx[2] = {0};
	
	uint8_t		temp_begin = freq_begin;
	uint8_t		temp_point = freq_begin;
	uint8_t		temp_end = freq_end;
	
	uint8_t		next_point_int = 0;
	uint8_t		next_point_dec = 0;
	uint16_t	next_point = 0;
	
    float		temp_Forward_power = 0.0;
    float		temp_Reverse_power = 0.0;
    float		temp_Standing_wave_ratio = 0.0;
		float		temp_swr = 0.0;
	
    for(temp_point=temp_begin; temp_point<=temp_end; temp_point++)   //从freq_begin开扫 freq_end结束
    {
		if( re_scan_flag == 0x01 )		//存在有效的扫描数据，扫描按钮已被激活
		{
			temp_swr = SWR_array[(temp_point-3)*10+index_dec];
			if( (temp_swr < 1.0) || (temp_swr >= 2.0) )	//unvalid data  无效数据
			{
				//scan
			}
			else	if( temp_point == temp_end )		//end  如果扫频起点和终点一样，退出
			{
				return;
			}
			else	//the data is well  数据很好，跳过这一个频点
			{
				continue;
			}
		}
		
        if( (System.cancel_sweeping == 0x01) || (System.stop == 0x01) )		//取消扫频
        {
            GT2000_Stop();  //停止发射
	
            System.sweep = 0;
            System.stop = 0;
            System.achieve_add_sub = 0;
            System.modify_power = 0;
            System.cancel_add_sub = 0;
            System_Status_Clean();
            System.status = SYSTEM_STANDBY;
				
						Scan_Breakpoint_Save(temp_point, index_dec);  //保存扫频断点，退出扫频
            return;
        }
        else	if( System.Voltage < 40.0 )  //如果是关机状态，退出扫频
        {
            System.open = 0x00;
            return;
        }
		
		System.status = SYSTEM_SCAN;  //扫频状态
    System.sweep = 0x01;  //扫频标志位	1:正在扫频，2:扫频完成，更新status标志位，并将sweep清零		最后，保存在flash
		
		//channel method都在MT2000_Cmd_Channel()固定写死了的，不需要再设置
		GT2000_Tx.Gt2000_mode = 1;		//固频
		GT2000_Tx.Frequency_power[1] = 0x04 ;		//北京机器扫频功率值1040W
    GT2000_Tx.Frequency_power[0] = 0x10 ;		
//		MT2000_Tx.method = 'F';		//FM
		
		n=(temp_point*1000)+(index_dec*100);  //扫频频率扩大1000倍
		sprintf(freq_scan_to_Tx, "%X", n); //把十进制数字转为16进制大写字符串
	  StrToHex(test_to_Tx,(uint8_t*)freq_scan_to_Tx, 2); //remarks : 将字符串转化为16进制数	保存在test_to_Tx				
		GT2000_Tx.Gt2000_freq1[0] = test_to_Tx[1];
		GT2000_Tx.Gt2000_freq1[1] = test_to_Tx[0];
		GT2000_Tx.Gt2000_freq1[2] = 0x00;					//小数点保持不变
		GT2000_Tx.Gt2000_freq1[3] = 0x00;
				Trans_printf(" scan1现在的扫频频点: temp_point= %d index_dec= %d \n",temp_point,index_dec);
				Trans_printf(" 扫频下发的scan1_freq_Tx: %02X %02X ",GT2000_Tx.Gt2000_freq1[0],GT2000_Tx.Gt2000_freq1[1]);

		temp_Forward_power = 0.0;
		temp_Reverse_power = 0.0;
		temp_Standing_wave_ratio = 0.0;

		next_point = Scan_Breakpoint_Read(); //在FLASH中读取扫频断点
		next_point_int = next_point/10;
		next_point_dec = next_point%10;
		
		//有效数据，下一个频点的整数部分大于当前频点，小数部分相同，则扫下一个频点
		if( (next_point!=0x0000) && (next_point!=0xFFFF) && (next_point_int>temp_point) && (next_point_dec==index_dec) )
		{
			continue;
		}

        Flag=GT2000_Emit();
		    Trans_printf(" 1_T0_GT2000_Emit() Flag= %d \n",Flag);
		    vTaskDelay(6000);
				Gt_Inquire_All();		//查询一次
				Flag=GT2000_Wait_Ack(MT2000_CMD_CHANNEL);
		
	    	if(Flag==MT2000_ACK_ALARM)
				{		
					temp_Forward_power = 0.0;
					temp_Reverse_power = 0.0;
					temp_Standing_wave_ratio = SCAN_ALARM_SWR_THRESHOLD;
					Scan_Breakpoint_Save(temp_point+1, index_dec);	
					continue;
				}
        if( System.emission == 0x01 )	//发射成功
        {
            App_printf("Sweep//发射成功\r\n");

            temp_Forward_power = System.Forward_Power;
            temp_Reverse_power = System.Reverse_Power;
            temp_Standing_wave_ratio = System.Standing_wave_ratio;

            //无应答报警、无功率报警
            if( (Alarm.no_respond==0x01) || (Alarm.no_power==0x01) )					//退出扫频
            {
                System.cancel_sweeping = 0x01;		//退出扫频
                GT2000_Stop();

								System.status = SYSTEM_ALARM;
                System.sweep = 0;
                System.stop = 0;
				
								Scan_Breakpoint_Save(temp_point, index_dec);
                return;
            }
							if( Alarm.emission == 0x01 )										//退出扫频，并在下次扫频时，退出当前频率
						{
							System.cancel_sweeping = 0x01;		//退出扫频
							GT2000_Stop();
							System.status = SYSTEM_ALARM;
							System.sweep = 0;
							System.stop = 0;
							
							temp_Forward_power = 0.0;
							temp_Reverse_power = 0.0;
							temp_Standing_wave_ratio = SCAN_ALARM_SWR_THRESHOLD;
							
							Scan_Breakpoint_Save(temp_point+1, index_dec);
							return;
						}
							 else	if( (Alarm.swr_alarm == 0x01) || (Alarm.over_Electric==0x01) )		//跳过当前频点
						{
							//先记录下当前电压电流驻波比，后停止
			//				MT2000_Cmd_Inquire();			//查询
			//				ret = MT2000_Wait_Ack(MT2000_CMD_INQUIRE);

							Alarm.swr_alarm = 0x00;
							Alarm.over_Electric = 0x00;
							
							temp_Forward_power = System.Forward_Power;
							temp_Reverse_power = System.Reverse_Power;
							temp_Standing_wave_ratio = System.Standing_wave_ratio;
							
							GT2000_Stop();

							//改回扫频状态，继续扫下一个频率
							System.status = SYSTEM_SCAN;
							System.sweep = 0x01;
							System.stop = 0;
							
							Scan_Breakpoint_Save(temp_point+1, index_dec);
						}
					}
				
        else							//发射失败
        {
            App_printf("Sweep//发射失败\r\n");

						System.status = SYSTEM_ALARM;
						System.sweep = 0;
						System.stop = 0;
			
            temp_Forward_power = 0.0;
            temp_Reverse_power = 0.0;
            temp_Standing_wave_ratio = SCAN_ALARM_SWR_THRESHOLD;

						if( Alarm.emission == 0x01 )									//退出扫频，并在下次扫频时，退出当前频率
						{
							System.cancel_sweeping = 0x01;		//退出扫频
							GT2000_Stop();
							System.status = SYSTEM_ALARM;
							System.sweep = 0;
							System.stop = 0;
							
							Scan_Breakpoint_Save(temp_point+1, index_dec);							//调谐时报警，先停止，跳过当前频点
						}				
						else
						{
							Scan_Breakpoint_Save(temp_point+1, index_dec);
						}
        }

				vTaskDelay(4000);
				Gt_Inquire_All();		//查询一次
				ret=GT2000_Wait_Ack(MT2000_CMD_CHANNEL);
				if(ret==MT2000_ACK_OK)
				{
					temp_Forward_power = System.Forward_Power;
					temp_Reverse_power = System.Reverse_Power;
					temp_Standing_wave_ratio = System.Standing_wave_ratio;
				}
				vTaskDelay(2000);
        //数组下标为	( 频率(MHz)-5 )*10
        //				5.0MHz => Forward_Power_array[0]	5.1MHz => Forward_Power_array[1]
        //				6.0MHz => Forward_Power_array[10]	6.1MHz => Forward_Power_array[11]
        //	index_dec:0~9	i:freq_end-freq_begin = 26.0-5.0 = 21
        Forward_Power_array[(temp_point-3)*10+index_dec] = temp_Forward_power/10.0;			//232，需要存进flash，返回扫频结果
        Reverse_Power_array[(temp_point-3)*10+index_dec] = temp_Reverse_power;					//不需要存，也不需要返回，所以不除
        SWR_array[(temp_point-3)*10+index_dec] = temp_Standing_wave_ratio;	//232，需要存进flash，返回扫频结果
				
				Gt_RF_Close();		
				vTaskDelay(500);   //新发射机响应较慢
				ret=MT2000_ACK_OK;	//默认成功，待多次测试

        if( ret == MT2000_ACK_OK )							//停止成功
        {
            System.stop = 0x00;
            App_printf("Sweep//发射停止\r\n");

            if( (System.cancel_sweeping == 0x01) || (System.stop == 0x01)  )
            {
                System.sweep = 0;
                System.stop = 0;
                System.achieve_add_sub = 0;
                System.modify_power = 0;
                System.cancel_add_sub = 0;
                System_Status_Clean();
                System.status = SYSTEM_STANDBY;

								Scan_Breakpoint_Save(temp_point, index_dec);
                return;
            }
            else
            {
                System_Status_Clean();
            }

            System.status = SYSTEM_SCAN;	//正在扫频
            System.sweep = 0x01;

            if(temp_point<temp_end)
            {
                for(protect_count=0; protect_count<50; protect_count++)
                {
                    if( (System.cancel_sweeping == 0x01) || (System.stop == 0x01)  )		//取消扫频
                    {
												Scan_Breakpoint_Save(temp_point+1, index_dec);
                        return;
                    }
                    else
                    {
                        vTaskDelay(100);						//保护机器
                    }
                }
            }
						else
						{
							Scan_Breakpoint_Save(temp_point+1, index_dec);
							return;
						}
        }
        else     //不成功，返回停止失败
        {
            Alarm.no_respond_locate = 5;
            App_printf("Sweep stop 5 no respond.\r\n");
            return;
        }
    }
    return;
}

/*------------------------------------扫频函数2----------------------------------------*/
void Optional_Band_scan(uint8_t Now_Frq,uint8_t Scan_end)
{
	int ret = 0,Flag=0,n;
	uint8_t protect_count = 0;		//保护机器
	char freq_scan_to_Tx[4]={0x00};    //保存十六进制扫频下发的频率数据
	unsigned char test_to_Tx[2] = {0};
	
	uint8_t		temp_point ;
	uint8_t		temp_end ;
	uint8_t   index_dec;
	
	float		temp_Forward_power = 0.0;
	float		temp_Reverse_power = 0.0;
	float		temp_Standing_wave_ratio = 0.0;
	float		temp_swr = 0.0;
	
	
		temp_point=Now_Frq/10;
		temp_end=Scan_end/10;
		index_dec=Now_Frq%10;
		
		if( re_scan_flag == 0x01 )		//there are valid data of scan, and the button of scan have been click again.
		{
			temp_swr = SWR_array[(temp_point-3)*10+index_dec];
			if( (temp_swr < 1.0) || (temp_swr >= 2.0) )	//unvalid data
			{
				//scan
			}
			
			else	//the data is well
			{
				return;
			}
		}
		
        if( (System.cancel_sweeping == 0x01) || (System.stop == 0x01) )		//取消扫频
        {
            GT2000_Stop();

            System.sweep = 0;
            System.stop = 0;
            System.achieve_add_sub = 0;
            System.modify_power = 0;
            System.cancel_add_sub = 0;
            System_Status_Clean();
            System.status = SYSTEM_STANDBY;
			
						Scan_Breakpoint_Save(temp_point, index_dec);
            return;
        }
        else	if( System.Voltage < 40.0 )
        {
            System.open = 0x00;
					  Trans_printf(" System.Voltage < 40.0 退出扫频！\n");
            return;
        }

				System.status = SYSTEM_SCAN;
				System.sweep = 0x01;//扫频标志位	1:正在扫频，2:扫频完成，更新status标志位，并将sweep清零		最后，保存在flash
				
				//channel method都在MT2000_Cmd_Channel()固定写死了的，不需要再设置
				GT2000_Tx.Gt2000_mode = 1;		//固频
				GT2000_Tx.Frequency_power[1] = 0x04 ;		//北京广通机器扫频功率值1001
				GT2000_Tx.Frequency_power[0] = 0x10 ;		
		//		MT2000_Tx.method = 'F';		//FM
				
				n=(temp_point*1000)+(index_dec*100);  //扫频频率扩大1000倍
				Trans_printf(" scan2频点n=  %d \n",n);
				sprintf(freq_scan_to_Tx, "%04X", n); //把十进制数字转为16进制大写字符串
				StrToHex(test_to_Tx,(uint8_t*)freq_scan_to_Tx, 2); //remarks : 将字符串转化为16进制数	保存在test_to_Tx				
				GT2000_Tx.Gt2000_freq1[0] = test_to_Tx[1];
				GT2000_Tx.Gt2000_freq1[1] = test_to_Tx[0];
				GT2000_Tx.Gt2000_freq1[2] = 0x00;					//小数点保持不变
				GT2000_Tx.Gt2000_freq1[3] = 0x00;
						Trans_printf(" scan2现在的扫频频点: temp_point= %d index_dec= %d freq_scan_to_Tx=%02x %02x \n",temp_point,index_dec,freq_scan_to_Tx[0],freq_scan_to_Tx[1]);
						Trans_printf(" 扫频下发的scan2_freq_Tx: %02X %02X ",GT2000_Tx.Gt2000_freq1[0],GT2000_Tx.Gt2000_freq1[1]);

				temp_Forward_power = 0.0;
				temp_Reverse_power = 0.0;
				temp_Standing_wave_ratio = 0.0;


        Flag=GT2000_Emit();   //触发发射，在上面赋值发射参数
				Trans_printf(" 2_T0_GT2000_Emit() Flag= %d \n",Flag);
				vTaskDelay(6000);
				Gt_Inquire_All();		//查询一次
				Flag=GT2000_Wait_Ack(MT2000_CMD_CHANNEL);
				
				if(Flag==MT2000_ACK_ALARM)
				{		
					temp_Forward_power = 0.0;
					temp_Reverse_power = 0.0;
					temp_Standing_wave_ratio = SCAN_ALARM_SWR_THRESHOLD;
					Scan_Breakpoint_Save(temp_point+1, index_dec);	
					return;
				}

        if( System.emission == 0x01 )	//发射成功
        {
					temp_Forward_power = System.Forward_Power;
					temp_Reverse_power = System.Reverse_Power;
					temp_Standing_wave_ratio = System.Standing_wave_ratio;

            //无应答报警、无功率报警
            if( (Alarm.no_respond==0x01) || (Alarm.no_power==0x01) )					//退出扫频
            {
                System.cancel_sweeping = 0x01;		//退出扫频
                GT2000_Stop();

								System.status = SYSTEM_ALARM;
                System.sweep = 0;
                System.stop = 0;
				         
							  Trans_printf(" 无应答报警、无功率报警、退出扫频 \n");
								Scan_Breakpoint_Save(temp_point, index_dec);
                return;
            }
				if( Alarm.emission == 0x01 )										//退出扫频，并在下次扫频时，退出当前频率
			{
				System.cancel_sweeping = 0x01;		//退出扫频
				GT2000_Stop();
				System.status = SYSTEM_ALARM;
				System.sweep = 0;
				System.stop = 0;
				
				temp_Forward_power = 0.0;
				temp_Reverse_power = 0.0;
				temp_Standing_wave_ratio = SCAN_ALARM_SWR_THRESHOLD;
				
				Trans_printf(" 激励器自身报警、退出扫频 \n");
				Scan_Breakpoint_Save(temp_point+1, index_dec);
				return;
			}
         else	if( (Alarm.swr_alarm == 0x01) || (Alarm.over_Electric==0x01) )		//跳过当前频点
			{
				Alarm.swr_alarm = 0x00;
				Alarm.over_Electric = 0x00;
				
				temp_Forward_power = System.Forward_Power;
				temp_Reverse_power = System.Reverse_Power;
				temp_Standing_wave_ratio = System.Standing_wave_ratio;
				
				GT2000_Stop();

				//改回扫频状态，继续扫下一个频率
				System.status = SYSTEM_SCAN;
				System.sweep = 0x01;
				System.stop = 0;
				
				Trans_printf(" 驻波比报警、过流报警、退出扫频 \n");
				Scan_Breakpoint_Save(temp_point+1, index_dec);
			}
           
        }
        else							//发射失败
        {
					Trans_printf(" 扫频发射失败、退出扫频 \n");
						System.status = SYSTEM_ALARM;
						System.sweep = 0;
						System.stop = 0;
			
            temp_Forward_power = 0.0;
            temp_Reverse_power = 0.0;
            temp_Standing_wave_ratio = SCAN_ALARM_SWR_THRESHOLD;


					if( Alarm.emission == 0x01 )									//退出扫频，并在下次扫频时，退出当前频率
					{
						System.cancel_sweeping = 0x01;		//退出扫频
						GT2000_Stop();
						System.status = SYSTEM_ALARM;
						System.sweep = 0;
						System.stop = 0;
						
						Scan_Breakpoint_Save(temp_point+1, index_dec);							//调谐时报警，先停止，跳过当前频点
					}				
					else
					{
						Scan_Breakpoint_Save(temp_point+1, index_dec);
					}
        }
				
				vTaskDelay(4000);
				Gt_Inquire_All();		//查询一次
				ret=GT2000_Wait_Ack(MT2000_CMD_CHANNEL);
				if(ret==MT2000_ACK_OK)
				{
					temp_Forward_power = System.Forward_Power;
					temp_Reverse_power = System.Reverse_Power;
					temp_Standing_wave_ratio = System.Standing_wave_ratio;
				}
				vTaskDelay(2000);
        //数组下标为	( 频率(MHz)-5 )*10
        //				5.0MHz => Forward_Power_array[0]	5.1MHz => Forward_Power_array[1]
        //				6.0MHz => Forward_Power_array[10]	6.1MHz => Forward_Power_array[11]
        //	index_dec:0~9	i:freq_end-freq_begin = 26.0-5.0 = 21
        Forward_Power_array[(temp_point-3)*10+index_dec] = temp_Forward_power/10.0;			//233，需要存进flash，返回扫频结果
        Reverse_Power_array[(temp_point-3)*10+index_dec] = temp_Reverse_power;					//不需要存，也不需要返回，所以不除
        SWR_array[(temp_point-3)*10+index_dec] = temp_Standing_wave_ratio;	//233，需要存进flash，返回扫频结果
				
				Gt_RF_Close();		//关发射
				vTaskDelay(50);
				ret=MT2000_ACK_OK;			//默认成功

        if( ret == MT2000_ACK_OK )							//停止成功
        {
            System.stop = 0x00;
            App_printf("Sweep//发射停止\r\n");

            if( (System.cancel_sweeping == 0x01) || (System.stop == 0x01)  )
            {
                System.sweep = 0;
                System.stop = 0;
                System.achieve_add_sub = 0;
                System.modify_power = 0;
                System.cancel_add_sub = 0;
                System_Status_Clean();
                System.status = SYSTEM_STANDBY;

								Scan_Breakpoint_Save(temp_point, index_dec);
                return;
            }
            else
            {
                System_Status_Clean();
            }

            System.status = SYSTEM_SCAN;	//正在扫频
            System.sweep = 0x01;

            if(temp_point<temp_end)
            {
                for(protect_count=0; protect_count<50; protect_count++)
                {
                    if( (System.cancel_sweeping == 0x01) || (System.stop == 0x01)  )		//取消扫频
                    {
												Scan_Breakpoint_Save(temp_point+1, index_dec);
                        return;
                    }
                    else
                    {
                        vTaskDelay(100);						//保护机器
                    }
                }
            }
						else
						{
							Scan_Breakpoint_Save(temp_point+1, index_dec);
							return;
						}
        }
        else     //不成功，返回停止失败
        {
            Alarm.no_respond_locate = 5;
            App_printf("Sweep stop 5 no respond.\r\n");
				  	Trans_printf("Sweep stop 5 no respond.\r\n");
            return;
        }
								
    return;
}

/*-------------------------------------------------------------------------------------------------------------*/
void func_code_printf(void)
{
	Trans_printf(" 打印功能码g_fuc_cod : ");
    if(g_fuc_cod[0]==0x01&&g_fuc_cod[1]==0x01)				//初始化
        Trans_printf("初始化\r\n");
    else	if(g_fuc_cod[0]==0x02&&g_fuc_cod[1]==0x01)		//查询
        Trans_printf("查询\r\n");
    else	if(g_fuc_cod[0]==0x03&&g_fuc_cod[1]==0x01)		//发射
        Trans_printf("发射\r\n");
    else	if(g_fuc_cod[0]==0x04&&g_fuc_cod[1]==0x01)		//停止
        Trans_printf("停止\r\n");
    else	if(g_fuc_cod[0]==0x05&&g_fuc_cod[1]==0x01)		//解除报警
        Trans_printf("解除报警\r\n");
    else	if(g_fuc_cod[0]==0x06&&g_fuc_cod[1]==0x01)		//报警查询
        Trans_printf("报警查询\r\n");
    else	if(g_fuc_cod[0]==0x07&&g_fuc_cod[1]==0x01)		//开机
        Trans_printf("开机\r\n");
    else	if(g_fuc_cod[0]==0x08&&g_fuc_cod[1]==0x01)		//关机
        Trans_printf("关机\r\n");
    else	if(g_fuc_cod[0]==0x09&&g_fuc_cod[1]==0x01)		//扫频
        Trans_printf("扫频\r\n");
    else	if(g_fuc_cod[0]==0x0A&&g_fuc_cod[1]==0x01)		//停止扫频
        Trans_printf("停止扫频\r\n");
    else	if(g_fuc_cod[0]==0x0B&&g_fuc_cod[1]==0x01)		//增加功率
        Trans_printf("增加功率\r\n");
    else	if(g_fuc_cod[0]==0x0C&&g_fuc_cod[1]==0x01)		//减小功率
        Trans_printf("减小功率\r\n");
    else	if(g_fuc_cod[0]==0x0D&&g_fuc_cod[1]==0x01)		//运行图
        Trans_printf("运行图\r\n");
}

void freq_hex_to_str(uint8_t *freq_hex, uint8_t *freq_str)		//hex的频率转换为str频率
{
    *(freq_str + 0) = (*(freq_hex + 0) / 16) + '0';
    *(freq_str + 1) = (*(freq_hex + 0) % 16) + '0';
    *(freq_str + 2) = (*(freq_hex + 1) / 16) + '0';
    *(freq_str + 3) = (*(freq_hex + 1) % 16) + '0';
    *(freq_str + 4) = (*(freq_hex + 2) / 16) + '0';
    *(freq_str + 5) = (*(freq_hex + 2) % 16) + '0';
    *(freq_str + 6) = (*(freq_hex + 3) / 16) + '0';
    *(freq_str + 7) = (*(freq_hex + 3) % 16) + '0';
    *(freq_str + 8) = '\0';

//	*(freq_str+0) = (*(freq_hex+0)>>4)&0x0F + '0';
//	*(freq_str+1) = (*(freq_hex+0)>>0)&0x0F + '0';
//	*(freq_str+2) = (*(freq_hex+1)>>4)&0x0F + '0';
//	*(freq_str+3) = (*(freq_hex+1)>>0)&0x0F + '0';
//	*(freq_str+4) = (*(freq_hex+2)>>4)&0x0F + '0';
//	*(freq_str+5) = (*(freq_hex+2)>>0)&0x0F + '0';
//	*(freq_str+6) = (*(freq_hex+3)>>4)&0x0F + '0';
//	*(freq_str+7) = (*(freq_hex+3)>>0)&0x0F + '0';
//	*(freq_str+8) = '\0';
}
/*
*   freq_str_to_hex此函数为旧协议使用
*/
void freq_str_to_hex(uint8_t *freq_str, uint8_t *freq_hex)		//str的频率转换为hex频率
{
    *(freq_hex+0) = ( ( *(freq_str+0)-'0') << 4 ) + ( *(freq_str+1)-'0');
    *(freq_hex+1) = ( ( *(freq_str+2)-'0') << 4 ) + ( *(freq_str+3)-'0');
    *(freq_hex+2) = ( ( *(freq_str+4)-'0') << 4 ) + ( *(freq_str+5)-'0');
    *(freq_hex+3) = ( ( *(freq_str+6)-'0') << 4 ) + ( *(freq_str+7)-'0');
}  


/*将字符串s转换成相应的整数*/
int str_to_10D(uint8_t s[])
 {
        int i;
	    int n = 0;
	    for (i = 0; s[i] >= '0' && s[i] <= 'F'; ++i)
		   {
		         n = 10 * n + (s[i] - '0');
		     }
	     return n;
 }
 
 

void freq_PChex_to_GT2000( uint8_t *freq_PC_str,uint8_t *freq_Trans_hex)  //PC下发的频率赋值给GT2000，低位在前
{
  int n;
	char str[4];
	unsigned char test[2] = {0};
	uint8_t arry[2];   //保存前两位地址中的数据
	arry[0] = *(freq_PC_str + 0);
	arry[1] = *(freq_PC_str + 1);
	if((arry[0]!=0x00)||(arry[1]!=0x00))  //PC下发的前两位不为0才进入判断
	{
	freq_Trans_hex = mystrncpy(freq_Trans_hex, 5);//截取前5个字符串
	n = str_to_10D(freq_Trans_hex);   //将字符串freq_Trans_hex转换成相应的整数
	sprintf(str, "%X", n); //把十进制数字转为16进制大写字符串
	StrToHex(test,(uint8_t*)str, 2); //remarks : 将字符串转化为16进制数
//	printf(" 转化后频率test== %02X  %02X",test[0],test[1]);
	*(freq_Trans_hex + 1) = test[0];
	*(freq_Trans_hex + 0) = test[1];
	*(freq_Trans_hex + 2) = 0x00;
	*(freq_Trans_hex + 3) = 0x00;
	}
	else
	{
		memset((freq_Trans_hex + 0),0x00,4);  //屏蔽PC单频时赋值其他两个频率。
//		Trans_printf(" 无效频率 ");
	}
}

void freq_GT2000Hex_to_PChex(unsigned char *freq_Trans_hex, unsigned char *freq_PC_Hex)  //GT2000的频率赋值给GT2000，低位在前
{
	int n = 0;
  unsigned char freq[6];
	freq[5] = '\0';
	n = (*(freq_Trans_hex + 0) << 8) | (*(freq_Trans_hex + 1));
	printf(" 获取到的十六进制频率为= %X \n", n);
	sprintf((char *)freq, "%05d", n); //把十六进制数字转为十进制字符串
	
		freq_str_to_hex(freq, freq_PC_Hex);
		Trans_printf(" 获取到的十进制频率为freq= %s \n", freq);
	
	*(freq_PC_Hex + 2) = 0x00;
	*(freq_PC_Hex + 3) = 0x00;

	Trans_printf(" 转化后的频率为= %02X %02X %02X %02X \n", freq_PC_Hex[0], freq_PC_Hex[1], freq_PC_Hex[2], freq_PC_Hex[3]);

}

void GT2000_Tx_freq_Hex_to_PChex(unsigned char *freq_Trans_hex, unsigned char *freq_PC_Hex)  //供查询扫频函数使用
{
	int n = 0;
  unsigned char freq[6];
	freq[5] = '\0';
	n = (*(freq_Trans_hex + 1) << 8) | (*(freq_Trans_hex + 0));
	printf(" 2获取到的十六进制频率为= %X \n", n);
	sprintf((char *)freq, "%05d", n); //把十六进制数字转为十进制字符串
	
		freq_str_to_hex(freq, freq_PC_Hex);
		Trans_printf(" 2获取到的十进制频率为freq= %s \n", freq);
	
	*(freq_PC_Hex + 2) = 0x00;
	*(freq_PC_Hex + 3) = 0x00;

	Trans_printf(" 2转化后的频率为= %02X %02X %02X %02X \n", freq_PC_Hex[0], freq_PC_Hex[1], freq_PC_Hex[2], freq_PC_Hex[3]);

}

uint8_t *mystrncpy( uint8_t *string, int n)   //要求截取的字符串不可以改变，但指向字符串的指针可以改变
{
	uint8_t *p = string;
	if (p == NULL)
	{//如果截取的字符串是空的直接返回
		return NULL;
	}
	else
	{
		int i = 0;
		while (*p != '\0')
		{//循环直到达n个字符串终止
			if (i == n)
			{
				break;
			}
			i++;
			p++;
		}
		*(p++) = '\0';//赋值结束字符串
		return string;
	}
}

/*
// C prototype : void StrToHex(BYTE *pbDest, BYTE *pbSrc, int nLen)
// parameter(s): [OUT] pbDest - 输出缓冲区
// [IN] pbSrc - 字符串
// [IN] nLen - 16进制数的字节数(字符串的长度/2)
// return value:
// remarks : 将字符串转化为16进制数
*/
void StrToHex(unsigned char *pbDest, unsigned char *pbSrc, int nLen)
{
	char h1, h2;
	unsigned char s1, s2;
	int i;

	for (i = 0; i<nLen; i++)
	{
		h1 = pbSrc[2 * i];
		h2 = pbSrc[2 * i + 1];

		s1 = h1 - 0x30;
		if (s1 > 9)
			s1 -= 7;

		s2 = h2 - 0x30;
		if (s2 > 9)
			s2 -= 7;

		pbDest[i] = s1 * 16 + s2;
	}
}


void set_run_diagram_new_power_level(uint8_t channel, uint8_t power_level)	//设置运行图的功率等级				Run_Diagram_data
{
   Run_Diagram_data.power[channel-'1'] = power_level;
}

void freq_range_judge(uint8_t *freq)  //频段值核对变量
{
	int freq_float=0;
//	freq_float=((freq[0]-'0')*10000+(freq[1]-'0')*1000+(freq[2]-'0')*100+(freq[3]-'0')*10+(freq[4]-'0'))/1000.0;
	freq_float=(freq[1]<<8|freq[0]);
	
	Trans_printf(" freq_float= %04x ",freq_float);
	if(0x0140<=freq_float&&freq_float<=0x14B4)
	{
		System.fbv_c=1;		//一频段
	}
	
	else if(0x14B4<=freq_float&&freq_float<=0x1B58)
	{
		System.fbv_c=2;		//二频段
	}
	
	else if(0x1B58<freq_float&&freq_float<=0x251C)
	{
		System.fbv_c=3;		//三频段
	}
	
	else if(0x251C<freq_float&&freq_float<=0x3A98)
	{
		System.fbv_c=4;		//四频段
	}
	
	else if(0x3A98<freq_float&&freq_float<=0x555A)
	{
		System.fbv_c=5;		//五频段
	}
	else if(0x555A<freq_float&&freq_float<=0x65F4)
	{
		System.fbv_c=6;		//六频段
	}
	Trans_printf(" 频段值核对变量freq_float== %04x System.fbv_c= %x   \n",freq_float,System.fbv_c);
}

