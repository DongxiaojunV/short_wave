#include "Data_pars.h"
#include "crc16.h"

#include "new_protocol.h"

volatile uint8_t Scan_Freq_flag;
Assemble_buffer_t Assemble_buffer;//拼包结构体定义

volatile uint8_t CAN_ID = 0x00;//这个值表示本身存在的发射机ID，通过拨码开关来确定，应该是一个全局变量，目前打算固化



/********************************************************************************
p_PC_data 串口收到的数据，p_PC_data->uart_pc是具体收到的数据内容
CAN_data 是解析出来的数据 CAN_data->CAN_ID_buf CAN编号 	CAN_data->功能码 CAN_data->p_Spe_data->Spe_data具体是数据
***************************************************************************************************************/
/* 去帧头，校验并且返回数据 */
static CAN_data_t Accpet_CAN_data(pc_data_t *PC_data)//这个是用在发射机那个主控的解析函数
{
    CAN_data_t CAN_data;
    int a = 0;
    int i;
    unsigned char buff_len[1];
    unsigned char buff_CRC[2];
    unsigned int CRC_Back;
    int AcceptData_Len,AcceptData_CRC;
    int n = 0;
    unsigned char Test_Data[150];
    volatile unsigned int j = 0;

    while((PC_data->uart_pc[j++] != 0x05) && (PC_data->uart_pc[j] != 0x02))
        ;
    if(j>250)
    {
        j = 0;
    }
    App_printf("j = %d\r\n",j);
    buff_len[0] = PC_data->uart_pc[j+2];
    App_printf("buff_len[0] = %x\r\n",buff_len[0]);
    AcceptData_Len = (buff_len[0]);
    buff_CRC[0] = PC_data->uart_pc[AcceptData_Len+3+j];
    buff_CRC[1] = PC_data->uart_pc[AcceptData_Len+4+j];
    App_printf("buff_CRC[0]= %x\r\n",buff_CRC[0]);
    App_printf("buff_CRC[1]= %x\r\n",buff_CRC[1]);
    AcceptData_CRC = ((buff_CRC[0]<<8) +(buff_CRC[1]));
    App_printf("AcceptData_CRC = %x\r\n",AcceptData_CRC);
    for(n = j+3; n < AcceptData_Len+3+j; n++)
    {
        Test_Data[a] = PC_data->uart_pc[n];
        a++;
    }
    a = 0;
    App_printf("Test_Data = ");
    for(i = 0; i<AcceptData_Len ; i++ )
    {
        App_printf("%02x ",Test_Data[i]);
    }
    App_printf("\r\n");
    CRC_Back = CRC16_XMODEM(Test_Data, AcceptData_Len);
    App_printf("CRC_Back = %x\r\n",CRC_Back);

    if(CRC_Back == AcceptData_CRC)
    {
        CAN_data.eff_sign = 1;
        CAN_data.CAN_ID_buf = PC_data->uart_pc[j+1];
        CAN_data.Func_code_buf[0] = PC_data->uart_pc[j+3];
        CAN_data.Func_code_buf[1] = PC_data->uart_pc[j+4];

        memcpy(CAN_data.p_Spe_data.Spe_data,PC_data->uart_pc+j+6,AcceptData_Len-3);

        CAN_data.p_Spe_data.Spe_len = AcceptData_Len-3;

        App_printf("CAN_ID = %X\r\n",CAN_data.CAN_ID_buf);
        for(int i = 0; i<2; i++)
        {
            App_printf("Func_code_buf = %02X\r\n",CAN_data.Func_code_buf[i]);
        }
        for(int i = 0; i<AcceptData_Len-3; i++)
        {
            App_printf("%02X ",CAN_data.p_Spe_data.Spe_data[i]);
        }
        App_printf("\r\n");
        App_printf("CAN_data->p_Spe_data->Spe_len = %02X",CAN_data.p_Spe_data.Spe_len);
    }
    else
    {
        CAN_data.eff_sign = 0;//标志结构体里面是数据无效
        App_printf("CRC错误\r\n");
    }
    return CAN_data;
}
/************************************************************
*返回1，数据解析成功，并且参数正确
*返回3，数据解析成功，但是发射机ID不匹配
*返回4，没有这个功能码
************************************************************/
static uint8_t Data_Storage(CAN_data_t *CAN_data_cmp)
{
    if(CAN_data_cmp->Func_code_buf[0] == 0x01 && CAN_data_cmp->Func_code_buf[1] == 0x01)		//Pc下发报警参数阈值
    {
        memcpy(Alarm_threshold.Transmitte_id,CAN_data_cmp->p_Spe_data.Spe_data,1);
        memcpy(Alarm_threshold.Low_temp_limit,CAN_data_cmp->p_Spe_data.Spe_data+1,4);
        memcpy(Alarm_threshold.Upp_temp_limit,CAN_data_cmp->p_Spe_data.Spe_data+5,4);
        memcpy(Alarm_threshold.Low_humidity_limit,CAN_data_cmp->p_Spe_data.Spe_data+9,4);
        memcpy(Alarm_threshold.Upp_humidity_limit,CAN_data_cmp->p_Spe_data.Spe_data+13,4);
        memcpy(Alarm_threshold.Low_45I_limit,CAN_data_cmp->p_Spe_data.Spe_data+17,4);
        memcpy(Alarm_threshold.Upp_45I_limit,CAN_data_cmp->p_Spe_data.Spe_data+21,4);
        memcpy(Alarm_threshold.Low_45V_limit,CAN_data_cmp->p_Spe_data.Spe_data+25,4);
        memcpy(Alarm_threshold.Upp_45V_limit,CAN_data_cmp->p_Spe_data.Spe_data+29,4);
        memcpy(Alarm_threshold.timer,CAN_data_cmp->p_Spe_data.Spe_data+33,6);
        App_printf("\r\n");
        App_printf("%f  %f",Alarm_threshold.Low_temp_limit[0],Alarm_threshold.Upp_temp_limit[0]);
        App_printf("%f  %f",Alarm_threshold.Low_humidity_limit[0],Alarm_threshold.Upp_humidity_limit[0]);
        App_printf("%f  %f",Alarm_threshold.Low_45I_limit[0],Alarm_threshold.Upp_45I_limit[0]);
        App_printf("%f  %f",Alarm_threshold.Low_45V_limit[0],Alarm_threshold.Upp_45V_limit[0]);
        App_printf("\r\n");

//			memcpy(Alarm_threshold.Low_temp_limit,CAN_data_cmp->p_Spe_data.Spe_data,4);
//			memcpy(Alarm_threshold.Upp_temp_limit,CAN_data_cmp->p_Spe_data.Spe_data+4,4);
//			memcpy(Alarm_threshold.Low_humidity_limit,CAN_data_cmp->p_Spe_data.Spe_data+8,4);
//			memcpy(Alarm_threshold.Upp_humidity_limit,CAN_data_cmp->p_Spe_data.Spe_data+12,4);
//			memcpy(Alarm_threshold.Low_45I_limit,CAN_data_cmp->p_Spe_data.Spe_data+16,4);
//			memcpy(Alarm_threshold.Upp_45I_limit,CAN_data_cmp->p_Spe_data.Spe_data+20,4);
//			memcpy(Alarm_threshold.Low_45V_limit,CAN_data_cmp->p_Spe_data.Spe_data+24,4);
//			memcpy(Alarm_threshold.Upp_45V_limit,CAN_data_cmp->p_Spe_data.Spe_data+28,4);
//			memcpy(Alarm_threshold.timer,CAN_data_cmp->p_Spe_data.Spe_data+32,6);
//			memcpy(Alarm_threshold.Transmitte_id,CAN_data_cmp->p_Spe_data.Spe_data+38,1);
        return 1;
    }
    else if(CAN_data_cmp->Func_code_buf[0] == 0x02 && CAN_data_cmp->Func_code_buf[1] == 0x01)	//下发状态查询
    {
        if(CAN_data_cmp->p_Spe_data.Spe_data[0] == CAN_ID)   //第一个字节表示发射机ID，必须判断与自身是否相同
        {
            memcpy(Working_paramet.Retain,CAN_data_cmp->p_Spe_data.Spe_data+1,2);
            return 1;
        }
        else
        {
            return 3;
        }
    }
    else if(CAN_data_cmp->Func_code_buf[0] == 0x03 && CAN_data_cmp->Func_code_buf[1] == 0x01)	//开启发射机
    {
        if(CAN_data_cmp->p_Spe_data.Spe_data[0] == CAN_ID)   //第一个字节表示发射机ID，必须判断与自身是否相同
        {
            memcpy(Trans_open.Type, CAN_data_cmp->p_Spe_data.Spe_data+1, 1);		//工作种类
            memcpy(Trans_open.Channel, CAN_data_cmp->p_Spe_data.Spe_data+2, 3);		//信道
            memcpy(Trans_open.Freq, CAN_data_cmp->p_Spe_data.Spe_data+5, 12);		//频率
            memcpy(Trans_open.Power_grade, CAN_data_cmp->p_Spe_data.Spe_data+17, 3);//功率等级
            memcpy(Trans_open.Berdou_time, CAN_data_cmp->p_Spe_data.Spe_data+20, 6);//北斗时间

            MT2000_Tx.method = 'F';		//FM

            switch( Trans_open.Type[0] )	//工作种类，固频/双频/三频
            {
            case 0x01:
                MT2000_Tx.mode = '1';
                break;
            case 0x02:
                MT2000_Tx.mode = '2';
                break;
            case 0x03:
                MT2000_Tx.mode = '3';
                break;
            default:
                break;
            }

            MT2000_Tx.channel[0] = '0';	//信道01
            MT2000_Tx.channel[1] = '1';
            MT2000_Tx.channel[2] = '\0';

            MT2000_Tx.fre1[0] = ( ( (Trans_open.Freq[0]) >> 4 ) & 0x0F) + '0';
            MT2000_Tx.fre1[1] = ( ( (Trans_open.Freq[0]) >> 0 ) & 0x0F) + '0';
            MT2000_Tx.fre1[2] = ( ( (Trans_open.Freq[1]) >> 4 ) & 0x0F) + '0';
            MT2000_Tx.fre1[3] = ( ( (Trans_open.Freq[1]) >> 0 ) & 0x0F) + '0';
            MT2000_Tx.fre1[4] = ( ( (Trans_open.Freq[2]) >> 4 ) & 0x0F) + '0';
            MT2000_Tx.fre1[5] = ( ( (Trans_open.Freq[2]) >> 0 ) & 0x0F) + '0';
            MT2000_Tx.fre1[6] = ( ( (Trans_open.Freq[3]) >> 4 ) & 0x0F) + '0';
            MT2000_Tx.fre1[7] = ( ( (Trans_open.Freq[3]) >> 0 ) & 0x0F) + '0';
            MT2000_Tx.fre1[8] = '\0';

            MT2000_Tx.fre2[0] = ( ( (Trans_open.Freq[4]) >> 4 ) & 0x0F) + '0';
            MT2000_Tx.fre2[1] = ( ( (Trans_open.Freq[4]) >> 0 ) & 0x0F) + '0';
            MT2000_Tx.fre2[2] = ( ( (Trans_open.Freq[5]) >> 4 ) & 0x0F) + '0';
            MT2000_Tx.fre2[3] = ( ( (Trans_open.Freq[5]) >> 0 ) & 0x0F) + '0';
            MT2000_Tx.fre2[4] = ( ( (Trans_open.Freq[6]) >> 4 ) & 0x0F) + '0';
            MT2000_Tx.fre2[5] = ( ( (Trans_open.Freq[6]) >> 0 ) & 0x0F) + '0';
            MT2000_Tx.fre2[6] = ( ( (Trans_open.Freq[7]) >> 4 ) & 0x0F) + '0';
            MT2000_Tx.fre2[7] = ( ( (Trans_open.Freq[7]) >> 0 ) & 0x0F) + '0';
            MT2000_Tx.fre2[8] = '\0';

            MT2000_Tx.fre3[0] = ( ( (Trans_open.Freq[8]) >> 4 ) & 0x0F) + '0';
            MT2000_Tx.fre3[1] = ( ( (Trans_open.Freq[8]) >> 0 ) & 0x0F) + '0';
            MT2000_Tx.fre3[2] = ( ( (Trans_open.Freq[9]) >> 4 ) & 0x0F) + '0';
            MT2000_Tx.fre3[3] = ( ( (Trans_open.Freq[9]) >> 0 ) & 0x0F) + '0';
            MT2000_Tx.fre3[4] = ( ( (Trans_open.Freq[10]) >> 4 ) & 0x0F) + '0';
            MT2000_Tx.fre3[5] = ( ( (Trans_open.Freq[10]) >> 0 ) & 0x0F) + '0';
            MT2000_Tx.fre3[6] = ( ( (Trans_open.Freq[11]) >> 4 ) & 0x0F) + '0';
            MT2000_Tx.fre3[7] = ( ( (Trans_open.Freq[11]) >> 0 ) & 0x0F) + '0';
            MT2000_Tx.fre3[8] = '\0';

            switch( Trans_open.Power_grade[0] )	//功率等级
            {
            case 0x00:
                MT2000_Tx.power = '3';
                break;	//全功率
            case 0x01:
                MT2000_Tx.power = '2';
                break;	//1/2功率
            case 0x02:
                MT2000_Tx.power = '1';
                break;	//1/4功率
            default:
                break;
            }
            return 1;
        }
        else
        {
            return 3;
        }
    }
    else if(CAN_data_cmp->Func_code_buf[0] == 0x04 && CAN_data_cmp->Func_code_buf[1] == 0x01)	//关闭发射机
    {
        if(CAN_data_cmp->p_Spe_data.Spe_data[0] == CAN_ID)   //第一个字节表示发射机ID，必须判断与自身是否相同
        {
            memcpy(Trans_stop.Berdou_time,CAN_data_cmp->p_Spe_data.Spe_data+1,6);//北斗时间
            return 1;
        }
        else
        {
            return 3;
        }
    }
    else if(CAN_data_cmp->Func_code_buf[0] == 0x05 && CAN_data_cmp->Func_code_buf[1] == 0x01)	//取消报警
    {
        if(CAN_data_cmp->p_Spe_data.Spe_data[0] == CAN_ID)   //第一个字节表示发射机ID，必须判断与自身是否相同
        {
            memcpy(Disalarm.Disalarm_type,CAN_data_cmp->p_Spe_data.Spe_data+1,2);//解除报警类型
            memcpy(Disalarm.Berdou_time,CAN_data_cmp->p_Spe_data.Spe_data+3,6);//北斗时间
            return 1;
        }
        else
        {
            return 3;
        }
    }
    else if(CAN_data_cmp->Func_code_buf[0] == 0x06 && CAN_data_cmp->Func_code_buf[1] == 0x01)	//查询历史报警状态
    {
        if(CAN_data_cmp->p_Spe_data.Spe_data[0] == CAN_ID)   //第一个字节表示发射机ID，必须判断与自身是否相同
        {
            memcpy(Alarm_history.Berdou_time,CAN_data_cmp->p_Spe_data.Spe_data+1,6);//北斗时间
            return 1;
        }
        else
        {
            return 3;
        }
    }
    else if(CAN_data_cmp->Func_code_buf[0] == 0x07 && CAN_data_cmp->Func_code_buf[1] == 0x01)	//发射机上电
    {
        if(CAN_data_cmp->p_Spe_data.Spe_data[0] == CAN_ID)   //第一个字节表示发射机ID，必须判断与自身是否相同
        {
            memcpy(Power_on.Retain,CAN_data_cmp->p_Spe_data.Spe_data+1,1);//北斗时间
            memcpy(Power_on.Berdou_time,CAN_data_cmp->p_Spe_data.Spe_data+2,6);//北斗时间
            return 1;
        }
        else
        {
            return 3;
        }
    }
    else if(CAN_data_cmp->Func_code_buf[0] == 0x08 && CAN_data_cmp->Func_code_buf[1] == 0x01)	//发射机断电
    {
        if(CAN_data_cmp->p_Spe_data.Spe_data[0] == CAN_ID)   //第一个字节表示发射机ID，必须判断与自身是否相同
        {
            memcpy(Power_off.Retain,CAN_data_cmp->p_Spe_data.Spe_data+1,1);//
            return 1;
        }
        else
        {
            return 3;
        }
    }
    else if(CAN_data_cmp->Func_code_buf[0] == 0x09 && CAN_data_cmp->Func_code_buf[1] == 0x01)	//扫描最佳工作频段
    {
        if(CAN_data_cmp->p_Spe_data.Spe_data[0] == CAN_ID)   //第一个字节表示发射机ID，必须判断与自身是否相同
        {
            memcpy(Scan_Frq.state,CAN_data_cmp->p_Spe_data.Spe_data+1,1);//状态
            memcpy(Scan_Frq.Fre_Band,CAN_data_cmp->p_Spe_data.Spe_data+2,8);
            return 1;
        }
        else
        {
            return 3;
        }
    }
    else if(CAN_data_cmp->Func_code_buf[0] == 0x0A && CAN_data_cmp->Func_code_buf[1] == 0x01)	//停止扫频
    {
        if(CAN_data_cmp->p_Spe_data.Spe_data[0] == CAN_ID)   //第一个字节表示发射机ID，必须判断与自身是否相同
        {
            memcpy(Sacn_stop.state,CAN_data_cmp->p_Spe_data.Spe_data+1,1);//
            return 1;
        }
        else
        {
            return 3;
        }
    }
    else if(CAN_data_cmp->Func_code_buf[0] == 0x0B && CAN_data_cmp->Func_code_buf[1] == 0x01)	//增加功率
    {
        if(CAN_data_cmp->p_Spe_data.Spe_data[0] == CAN_ID)   //第一个字节表示发射机ID，必须判断与自身是否相同
        {
            memcpy(Add_Power.Power_UP,CAN_data_cmp->p_Spe_data.Spe_data+1,2);
            return 1;
        }
        else
        {
            return 3;
        }
    }
    else if(CAN_data_cmp->Func_code_buf[0] == 0x0C && CAN_data_cmp->Func_code_buf[1] == 0x01)	//减小功率
    {
        if(CAN_data_cmp->p_Spe_data.Spe_data[0] == CAN_ID)   //第一个字节表示发射机ID，必须判断与自身是否相同
        {
            memcpy(Sub_Power.Power_DOWN,CAN_data_cmp->p_Spe_data.Spe_data+1,2);
            return 1;
        }
        else
        {
            return 3;
        }
    }
    else if(CAN_data_cmp->Func_code_buf[0] == 0x0D && CAN_data_cmp->Func_code_buf[1] == 0x01)	//运行图
    {
        if(CAN_data_cmp->p_Spe_data.Spe_data[0] == CAN_ID)   //第一个字节表示发射机ID，必须判断与自身是否相同
        {
            memcpy(Run_Diagram.count,CAN_data_cmp->p_Spe_data.Spe_data+1,1);
            memcpy(Run_Diagram.Continue,CAN_data_cmp->p_Spe_data.Spe_data+2,2);

            memcpy(Run_Diagram.Start_Time1,CAN_data_cmp->p_Spe_data.Spe_data+4,2);
            memcpy(Run_Diagram.End_Timep1,CAN_data_cmp->p_Spe_data.Spe_data+6,2);
            memcpy(Run_Diagram.Power1,CAN_data_cmp->p_Spe_data.Spe_data+8,1);
            memcpy(Run_Diagram.Frq1,CAN_data_cmp->p_Spe_data.Spe_data+9,4);
            memcpy(Run_Diagram.channel1,CAN_data_cmp->p_Spe_data.Spe_data+13,1);

            memcpy(Run_Diagram.Start_Time2,CAN_data_cmp->p_Spe_data.Spe_data+14,2);
            memcpy(Run_Diagram.End_Timep2,CAN_data_cmp->p_Spe_data.Spe_data+16,2);
            memcpy(Run_Diagram.Power2,CAN_data_cmp->p_Spe_data.Spe_data+18,1);
            memcpy(Run_Diagram.Frq2,CAN_data_cmp->p_Spe_data.Spe_data+19,4);
            memcpy(Run_Diagram.channel2,CAN_data_cmp->p_Spe_data.Spe_data+23,1);

            memcpy(Run_Diagram.Start_Time3,CAN_data_cmp->p_Spe_data.Spe_data+24,2);
            memcpy(Run_Diagram.End_Timep3,CAN_data_cmp->p_Spe_data.Spe_data+26,2);
            memcpy(Run_Diagram.Power3,CAN_data_cmp->p_Spe_data.Spe_data+28,1);
            memcpy(Run_Diagram.Frq3,CAN_data_cmp->p_Spe_data.Spe_data+29,4);
            memcpy(Run_Diagram.channel3,CAN_data_cmp->p_Spe_data.Spe_data+33,1);

            memcpy(Run_Diagram.Start_Time4,CAN_data_cmp->p_Spe_data.Spe_data+34,2);
            memcpy(Run_Diagram.End_Timep4,CAN_data_cmp->p_Spe_data.Spe_data+36,2);
            memcpy(Run_Diagram.Power4,CAN_data_cmp->p_Spe_data.Spe_data+38,1);
            memcpy(Run_Diagram.Frq4,CAN_data_cmp->p_Spe_data.Spe_data+39,4);
            memcpy(Run_Diagram.channel4,CAN_data_cmp->p_Spe_data.Spe_data+43,1);

            memcpy(Run_Diagram.Start_Time5,CAN_data_cmp->p_Spe_data.Spe_data+44,2);
            memcpy(Run_Diagram.End_Timep5,CAN_data_cmp->p_Spe_data.Spe_data+46,2);
            memcpy(Run_Diagram.Power5,CAN_data_cmp->p_Spe_data.Spe_data+48,1);
            memcpy(Run_Diagram.Frq5,CAN_data_cmp->p_Spe_data.Spe_data+49,4);
            memcpy(Run_Diagram.channel5,CAN_data_cmp->p_Spe_data.Spe_data+53,1);

            memcpy(Run_Diagram.Start_Time6,CAN_data_cmp->p_Spe_data.Spe_data+54,2);
            memcpy(Run_Diagram.End_Timep6,CAN_data_cmp->p_Spe_data.Spe_data+56,2);
            memcpy(Run_Diagram.Power6,CAN_data_cmp->p_Spe_data.Spe_data+58,1);
            memcpy(Run_Diagram.Frq6,CAN_data_cmp->p_Spe_data.Spe_data+59,4);
            memcpy(Run_Diagram.channel6,CAN_data_cmp->p_Spe_data.Spe_data+63,1);

            memcpy(Run_Diagram.Start_Time7,CAN_data_cmp->p_Spe_data.Spe_data+64,2);
            memcpy(Run_Diagram.End_Timep7,CAN_data_cmp->p_Spe_data.Spe_data+66,2);
            memcpy(Run_Diagram.Power7,CAN_data_cmp->p_Spe_data.Spe_data+68,1);
            memcpy(Run_Diagram.Frq7,CAN_data_cmp->p_Spe_data.Spe_data+69,4);
            memcpy(Run_Diagram.channel7,CAN_data_cmp->p_Spe_data.Spe_data+73,1);

            memcpy(Run_Diagram.Start_Time8,CAN_data_cmp->p_Spe_data.Spe_data+74,2);
            memcpy(Run_Diagram.End_Timep8,CAN_data_cmp->p_Spe_data.Spe_data+76,2);
            memcpy(Run_Diagram.Power8,CAN_data_cmp->p_Spe_data.Spe_data+78,1);
            memcpy(Run_Diagram.Frq8,CAN_data_cmp->p_Spe_data.Spe_data+79,4);
            memcpy(Run_Diagram.channel8,CAN_data_cmp->p_Spe_data.Spe_data+83,1);

            memcpy(Run_Diagram.Start_Time9,CAN_data_cmp->p_Spe_data.Spe_data+84,2);
            memcpy(Run_Diagram.End_Timep9,CAN_data_cmp->p_Spe_data.Spe_data+86,2);
            memcpy(Run_Diagram.Power9,CAN_data_cmp->p_Spe_data.Spe_data+88,1);
            memcpy(Run_Diagram.Frq9,CAN_data_cmp->p_Spe_data.Spe_data+89,4);
            memcpy(Run_Diagram.channel9,CAN_data_cmp->p_Spe_data.Spe_data+93,1);

            memcpy(Run_Diagram.Start_Time10,CAN_data_cmp->p_Spe_data.Spe_data+94,2);
            memcpy(Run_Diagram.End_Timep10,CAN_data_cmp->p_Spe_data.Spe_data+96,2);
            memcpy(Run_Diagram.Power10,CAN_data_cmp->p_Spe_data.Spe_data+98,1);
            memcpy(Run_Diagram.Frq10,CAN_data_cmp->p_Spe_data.Spe_data+99,4);
            memcpy(Run_Diagram.channel10,CAN_data_cmp->p_Spe_data.Spe_data+103,1);

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
		*Specific_data 具体数据，也就是结构体里面的数据(数据包)
		Can_ID是标志发射机编号
		Len是Specific_data的数据长度，可用sizeof来求整个结构体
		Assemble_buffer_t *Buffer 返回的结构体指针
********************************************************/
static uint8_t Data_Assemble(uint8_t *Func_code,uint8_t *Specific_data,uint8_t Can_ID_buf,uint8_t Len,Assemble_buffer_t *Buffer)
{
    uint16_t CRC_Back;
    uint8_t Start_Data[7] = {0x05,0x02,0x00,0x00,0x00,0x00,0x00};
    uint8_t End_Data[4] = {0,0,3,4};
    Start_Data[2] = Can_ID_buf;//写入发射机ID
    Start_Data[3] = (((Len + 3) >> 0) & 0xff);//数据长度
    Start_Data[4] = Func_code[0];//功能码
    Start_Data[5] = Func_code[1];//功能码
#ifdef ADD_CAN_len
    Start_Data[6] = (CEIL_DIV(Start_Data[3]+8-12,8))*4+Start_Data[3]+8;
#endif
    memcpy(Buffer->Data_buffer, Start_Data,7);
    memcpy(Buffer->Data_buffer+7, Specific_data,Len);
    CRC_Back = CRC16_XMODEM(Buffer->Data_buffer+4, Len+3);
    End_Data[0] = (uint8_t)((CRC_Back >> 8) & 0xff);		//存高八位
    End_Data[1] = (uint8_t)((CRC_Back >> 0) & 0xff) ;		//低八位
    memcpy(Buffer->Data_buffer+Len+7, End_Data,4);
    Buffer->Len_buffer = Len+11;
    return 1;
}


/* 如果是从CAN接收到的数据，去除数据中间的帧头(4个字节) */
static uint8_t Data_extract(pc_data_t *p_PC_datextract,pc_data_t *pp_PC_datextract)
{
    uint8_t j = 0;
    uint8_t _12number  = 0;
    uint8_t remainder  = 0;
    uint8_t len_buf  = 0;
    while((p_PC_datextract->uart_pc[j++] != 0x05) && (p_PC_datextract->uart_pc[j] != 0x02))
        ;
    if(j>250)
    {
        j = 0;
        return 0;
    }
    len_buf = p_PC_datextract->uart_pc[j+2];
    App_printf("len_buf = %d\r\n",len_buf);
    App_printf("CEIL_DIV(len_buf+8-12,8)) = %d\r\n",CEIL_DIV(len_buf+8-12,8));
    p_PC_datextract->len = (CEIL_DIV(len_buf+8-12,8))*4+len_buf+8;
    App_printf("p_PC_datextract->len = %d\r\n",p_PC_datextract->len);
    _12number = (p_PC_datextract->len)/12;
    remainder = (p_PC_datextract->len)%12;//这个数肯定大于等于5,或则或则是0
    if(_12number == 0)   //接到的数小于12个
    {
        memcpy(pp_PC_datextract->uart_pc,p_PC_datextract->uart_pc+j-1,remainder);
        pp_PC_datextract->len = remainder;
    }
    else if(_12number == 1 && remainder == 0)     //接到的数等于12个
    {
        memcpy(pp_PC_datextract->uart_pc,p_PC_datextract->uart_pc+j-1,12);
        pp_PC_datextract->len = 12;
    }
    else     //串口接到的数大于12个
    {
        int i = 1;
        memcpy(pp_PC_datextract->uart_pc,p_PC_datextract->uart_pc+j-1,12);
        for(i = 1; i<_12number; i++)
        {
            memcpy(pp_PC_datextract->uart_pc+12+8*(i-1),p_PC_datextract->uart_pc+j-1+12*(i-0)+4,8);
        }
        if(remainder != 0)
        {
            memcpy(pp_PC_datextract->uart_pc+12+8*(i-1),p_PC_datextract->uart_pc+j-1+12*i+4,remainder-4);
            pp_PC_datextract->len = 12+8*(i-1)+remainder-4;
        }
        else
        {
            pp_PC_datextract->len = 12+8*(i-1);
        }
    }
    return 1;
}
/**************************************************
只需要调用，取串口收到的值来解析，其中串口是放在一个结构体里面
此结构体包含数据，和接受的数据长度，其中数据长度在解析的时候没有用到
*p_func_code_buf 是返回指向功能码的指针
 p_PC_dat是传递的串口的数据
*返回1，数据解析成功，并且参数正确
*返回3，数据解析成功，但是发射机ID不匹配
*返回4，没有这个功能码
*返回5，CRC解析错误
***************************************************/
uint8_t CAN_data_func(uint8_t *p_func_code_buf,pc_data_t *p_PC_dat)
{
    uint8_t err_code;
    App_printf("\r\n进入解析\r\n");
    CAN_data_t CAN_data_buf;
#ifdef CAN_ID_Remove
    __align(8) static pc_data_t pp_PC_dat;
    Data_extract(p_PC_dat,&pp_PC_dat);
    App_printf("数据长度 %02x \r\n",pp_PC_dat.len);
//	for(int i = 0;i<pp_PC_dat.len;i++)
//	{
//		App_printf("%02X ",pp_PC_dat.uart_pc[i]);
//	}
    CAN_data_buf  = Accpet_CAN_data(&pp_PC_dat);
#else
    CAN_data_buf  = Accpet_CAN_data(p_PC_dat);
#endif
    if(CAN_data_buf.eff_sign == 0)
        return 5;
    App_printf("解析出来的数据：\r\n");
    App_printf("AN_data_buf.CAN_ID_buf = %02X\r\n",CAN_data_buf.CAN_ID_buf);
    App_printf("CAN_data_buf.Func_code_buf[0] = %02X\r\n",CAN_data_buf.Func_code_buf[0]);
    App_printf("CAN_data_buf.Func_code_buf[1] = %02X\r\n",CAN_data_buf.Func_code_buf[1]);
    p_func_code_buf[0] = CAN_data_buf.Func_code_buf[0];
    p_func_code_buf[1] = CAN_data_buf.Func_code_buf[1];
    for(int i = 0; i<CAN_data_buf.p_Spe_data.Spe_len; i++)
    {
        App_printf("%02X ",CAN_data_buf.p_Spe_data.Spe_data[i]);
    }
    App_printf("\r\nCAN_data_buf.p_Spe_data->Spe_da = %d\r\n",CAN_data_buf.p_Spe_data.Spe_len);
    err_code = Data_Storage(&CAN_data_buf);
    return err_code;
}

/***************************************************
	只需要传递功能码，自动发送到串口二
	若是返回1，说明执行成功
	返回2，说明传递的功能码不存在、
	*Func_code_buf指向功能码的指针
****************************************************/
uint8_t Send_PC(uint8_t *Func_code_buf)
{
    Assemble_buffer_t COM_buffer;

    if(Func_code_buf[0] ==0x01 && Func_code_buf[1] == 0x02)			//发射机报警参数初始化请求应答
    {
        if(Data_Assemble(Func_code_buf,(uint8_t *)&Alarm_backPC,CAN_ID,sizeof(Alarm_backPC),&COM_buffer) == 1)
        {
            comSendBuf(COM5,COM_buffer.Data_buffer,COM_buffer.Len_buffer);
            App_printf("COM_buffer: ");
            for(int i=0; i<COM_buffer.Len_buffer; i++)
            {
                App_printf("%02x ",COM_buffer.Data_buffer[i]);
            }
            App_printf("\r\n");
            return 1;
        }
    }
    else if(Func_code_buf[0] ==0x02 && Func_code_buf[1] == 0x02)	//查询
    {
#if	NEW_PROTOCOL_EN
        /*----------------兼容旧协议，给旧协议的变量赋值-----------------*/
        Work_paraBack.Channel[0] = 0x01;		//0x01通道
        Work_paraBack.Channel[1] = 0x01;		//0x01通道
        Work_paraBack.Channel[2] = 0x01;		//0x01通道

        switch( MT2000_Rx.mode )
        {
        case '1':
            Work_paraBack.Type[0] = 0x01;
            break;	//固频
        case '2':
            Work_paraBack.Type[0] = 0x02;
            break;	//双频
        case '3':
            Work_paraBack.Type[0] = 0x03;
            break;	//三频
        default:
            break;
        }

        switch( MT2000_Rx.power )
        {
        case '1':
            Work_paraBack.Power_grade[0] = 0x02;
            break;	//1/4功率
        case '2':
            Work_paraBack.Power_grade[0] = 0x01;
            break;	//1/2功率
        case '3':
            Work_paraBack.Power_grade[0] = 0x00;
            break;	//全功率
        default:
            break;
        }

        Work_paraBack.Freq[0] = (MT2000_Rx.fre1[0]-'0')<<4 + (MT2000_Rx.fre1[1]-'0');
        Work_paraBack.Freq[0] = (MT2000_Rx.fre1[0]-'0')<<4 + (MT2000_Rx.fre1[1]-'0');
        Work_paraBack.Freq[0] = (MT2000_Rx.fre1[0]-'0')<<4 + (MT2000_Rx.fre1[1]-'0');
        Work_paraBack.Freq[0] = (MT2000_Rx.fre1[0]-'0')<<4 + (MT2000_Rx.fre1[1]-'0');

        /* 固频时无效 */
        Work_paraBack.Freq[0] = (MT2000_Rx.fre2[0]-'0')<<4 + (MT2000_Rx.fre2[1]-'0');
        Work_paraBack.Freq[1] = (MT2000_Rx.fre2[2]-'0')<<4 + (MT2000_Rx.fre2[3]-'0');
        Work_paraBack.Freq[2] = (MT2000_Rx.fre2[4]-'0')<<4 + (MT2000_Rx.fre2[5]-'0');
        Work_paraBack.Freq[3] = (MT2000_Rx.fre2[6]-'0')<<4 + (MT2000_Rx.fre2[7]-'0');

        /* 非三频无效 */
        Work_paraBack.Freq[0] = (MT2000_Rx.fre3[0]-'0')<<4 + (MT2000_Rx.fre3[1]-'0');
        Work_paraBack.Freq[1] = (MT2000_Rx.fre3[2]-'0')<<4 + (MT2000_Rx.fre3[3]-'0');
        Work_paraBack.Freq[2] = (MT2000_Rx.fre3[4]-'0')<<4 + (MT2000_Rx.fre3[5]-'0');
        Work_paraBack.Freq[3] = (MT2000_Rx.fre3[6]-'0')<<4 + (MT2000_Rx.fre3[7]-'0');
#endif
        if(Data_Assemble(Func_code_buf,(uint8_t *)&Work_paraBack,CAN_ID,sizeof(Work_paraBack),&COM_buffer) ==1)
        {
            comSendBuf(COM5,COM_buffer.Data_buffer,COM_buffer.Len_buffer);
            App_printf("COM_buffer: ");
            for(int i=0; i<COM_buffer.Len_buffer; i++)
            {
                App_printf("%02x ",COM_buffer.Data_buffer[i]);
            }
            App_printf("\r\n");
            return 1;
        }
    }
    else if(Func_code_buf[0] ==0x03 && Func_code_buf[1] == 0x02)	//发射
    {
        if(Data_Assemble(Func_code_buf,(uint8_t *)&Trans_openBack,CAN_ID,sizeof(Trans_openBack),&COM_buffer)  ==1)
        {
            comSendBuf(COM5,COM_buffer.Data_buffer,COM_buffer.Len_buffer);
            App_printf("COM_buffer: ");
            for(int i=0; i<COM_buffer.Len_buffer; i++)
            {
                App_printf("%02x ",COM_buffer.Data_buffer[i]);
            }
            App_printf("\r\n");
            return 1;
        }
    }
    else if(Func_code_buf[0] ==0x04 && Func_code_buf[1] == 0x02)	//停止
    {
        if(Data_Assemble(Func_code_buf,(uint8_t *)&Trans_stopBack,CAN_ID,sizeof(Trans_stopBack),&COM_buffer) ==1)
        {
            comSendBuf(COM5,COM_buffer.Data_buffer,COM_buffer.Len_buffer);
            App_printf("COM_buffer: ");
            for(int i=0; i<COM_buffer.Len_buffer; i++)
            {
                App_printf("%02x ",COM_buffer.Data_buffer[i]);
            }
            App_printf("\r\n");
            return 1;
        }
    }
    else if(Func_code_buf[0] ==0x05 && Func_code_buf[1] == 0x02)	//解除报警
    {
        if(Data_Assemble(Func_code_buf,(uint8_t *)&DisalarmBack,CAN_ID,sizeof(DisalarmBack),&COM_buffer) ==1)
        {
            comSendBuf(COM5,COM_buffer.Data_buffer,COM_buffer.Len_buffer);
            App_printf("COM_buffer: ");
            for(int i=0; i<COM_buffer.Len_buffer; i++)
            {
                App_printf("%02x ",COM_buffer.Data_buffer[i]);
            }
            App_printf("\r\n");
            return 1;
        }
    }
    else if(Func_code_buf[0] ==0x06 && Func_code_buf[1] == 0x02)	//报警信息查询
    {
        if(Data_Assemble(Func_code_buf,(uint8_t *)&Alarm_historyBack,CAN_ID,sizeof(Alarm_historyBack),&COM_buffer) ==1)
        {
            comSendBuf(COM5,COM_buffer.Data_buffer,COM_buffer.Len_buffer);
            App_printf("COM_buffer: ");
            for(int i=0; i<COM_buffer.Len_buffer; i++)
            {
                App_printf("%02x ",COM_buffer.Data_buffer[i]);
            }
            App_printf("\r\n");
            return 1;
        }
    }
    else if(Func_code_buf[0] ==0x07 && Func_code_buf[1] == 0x02)	//开机
    {
        if(Data_Assemble(Func_code_buf,(uint8_t *)&Power_onBack,CAN_ID,sizeof(Power_onBack),&COM_buffer) ==1)
        {
            comSendBuf(COM5,COM_buffer.Data_buffer,COM_buffer.Len_buffer);
            App_printf("COM_buffer: ");
            for(int i=0; i<COM_buffer.Len_buffer; i++)
            {
                App_printf("%02x ",COM_buffer.Data_buffer[i]);
            }
            App_printf("\r\n");
            return 1;
        }
    }
    else if(Func_code_buf[0] ==0x08 && Func_code_buf[1] == 0x02)	//关机
    {
        if(Data_Assemble(Func_code_buf,(uint8_t *)&Power_offBack,CAN_ID,sizeof(Power_offBack),&COM_buffer) ==1)
        {
            comSendBuf(COM5,COM_buffer.Data_buffer,COM_buffer.Len_buffer);
            App_printf("COM_buffer: ");
            for(int i=0; i<COM_buffer.Len_buffer; i++)
            {
                App_printf("%02x ",COM_buffer.Data_buffer[i]);
            }
            App_printf("\r\n");
            return 1;
        }
    }
    else if(Func_code_buf[0] ==0x09 && Func_code_buf[1] == 0x02)	//扫频
    {
        App_printf("COM_buffer sweepning: \r\n");
        switch(Scan_Freq_flag)
        {
        case 0:
        {
            if(Data_Assemble(Func_code_buf,(uint8_t *)&Scan_FrqBack,CAN_ID,sizeof(Scan_FrqBack),&COM_buffer) ==1)
            {
                comSendBuf(COM5,COM_buffer.Data_buffer,COM_buffer.Len_buffer);
                return 1;
            }
        }
        case 1:
        {
            if(Data_Assemble(Func_code_buf,(uint8_t *)&Scan_FrqBack1,CAN_ID,sizeof(Scan_FrqBack1),&COM_buffer) ==1)
            {
                comSendBuf(COM5,COM_buffer.Data_buffer,COM_buffer.Len_buffer);
                return 1;
            }
        }
        case 2:
        {
            if(Data_Assemble(Func_code_buf,(uint8_t *)&Scan_FrqBack2,CAN_ID,sizeof(Scan_FrqBack2),&COM_buffer) ==1)
            {
                comSendBuf(COM5,COM_buffer.Data_buffer,COM_buffer.Len_buffer);
                return 1;
            }
        }
        case 3:
        {
            if(Data_Assemble(Func_code_buf,(uint8_t *)&Scan_FrqBack3,CAN_ID,sizeof(Scan_FrqBack3),&COM_buffer) ==1)
            {
                comSendBuf(COM5,COM_buffer.Data_buffer,COM_buffer.Len_buffer);
                return 1;
            }
        }
        case 4:
        {
            if(Data_Assemble(Func_code_buf,(uint8_t *)&Scan_FrqBack4,CAN_ID,sizeof(Scan_FrqBack4),&COM_buffer) ==1)
            {
                comSendBuf(COM5,COM_buffer.Data_buffer,COM_buffer.Len_buffer);
                return 1;
            }
        }
        case 5:
        {
            if(Data_Assemble(Func_code_buf,(uint8_t *)&Scan_FrqBack5,CAN_ID,sizeof(Scan_FrqBack5),&COM_buffer) ==1)
            {
                comSendBuf(COM5,COM_buffer.Data_buffer,COM_buffer.Len_buffer);
                return 1;
            }
        }
        case 6:
        {
            if(Data_Assemble(Func_code_buf,(uint8_t *)&Scan_FrqBack6,CAN_ID,sizeof(Scan_FrqBack6),&COM_buffer) ==1)
            {
                comSendBuf(COM5,COM_buffer.Data_buffer,COM_buffer.Len_buffer);
                return 1;
            }
        }
        case 7:
        {
            if(Data_Assemble(Func_code_buf,(uint8_t *)&Scan_FrqBack7,CAN_ID,sizeof(Scan_FrqBack7),&COM_buffer) ==1)
            {
                comSendBuf(COM5,COM_buffer.Data_buffer,COM_buffer.Len_buffer);
                return 1;
            }
        }
        case 8:
        {
            if(Data_Assemble(Func_code_buf,(uint8_t *)&Scan_FrqBack8,CAN_ID,sizeof(Scan_FrqBack8),&COM_buffer) ==1)
            {
                comSendBuf(COM5,COM_buffer.Data_buffer,COM_buffer.Len_buffer);
                return 1;
            }
        }
        case 9:
        {
            if(Data_Assemble(Func_code_buf,(uint8_t *)&Scan_FrqBack9,CAN_ID,sizeof(Scan_FrqBack9),&COM_buffer) ==1)
            {
                comSendBuf(COM5,COM_buffer.Data_buffer,COM_buffer.Len_buffer);
                return 1;
            }
        }
        case 10:
        {
            if(Data_Assemble(Func_code_buf,(uint8_t *)&Scan_FrqBack10,CAN_ID,sizeof(Scan_FrqBack10),&COM_buffer) ==1)
            {
                comSendBuf(COM5,COM_buffer.Data_buffer,COM_buffer.Len_buffer);
                return 1;
            }
        }
        case 11:
        {
            if(Data_Assemble(Func_code_buf,(uint8_t *)&Scan_FrqBack11,CAN_ID,sizeof(Scan_FrqBack11),&COM_buffer) ==1)
            {
                comSendBuf(COM5,COM_buffer.Data_buffer,COM_buffer.Len_buffer);
                return 1;
            }
        }
        case 12:
        {
            if(Data_Assemble(Func_code_buf,(uint8_t *)&Scan_FrqBack12,CAN_ID,sizeof(Scan_FrqBack12),&COM_buffer) ==1)
            {
                comSendBuf(COM5,COM_buffer.Data_buffer,COM_buffer.Len_buffer);
                return 1;
            }
        }
        case 13:
        {
            if(Data_Assemble(Func_code_buf,(uint8_t *)&Scan_FrqBack13,CAN_ID,sizeof(Scan_FrqBack13),&COM_buffer) ==1)
            {
                comSendBuf(COM5,COM_buffer.Data_buffer,COM_buffer.Len_buffer);
                return 1;
            }
        }
        }
    }
    else if(Func_code_buf[0] ==0x0A && Func_code_buf[1] == 0x02)	//停止扫频
    {
        if(Data_Assemble(Func_code_buf,(uint8_t *)&Sacn_stopBack,CAN_ID,sizeof(Sacn_stopBack),&COM_buffer) ==1)
        {
            comSendBuf(COM5,COM_buffer.Data_buffer,COM_buffer.Len_buffer);
            App_printf("COM_buffer: ");
            for(int i=0; i<COM_buffer.Len_buffer; i++)
            {
                App_printf("%02x ",COM_buffer.Data_buffer[i]);
            }
            App_printf("\r\n");
            return 1;
        }
    }
    else if(Func_code_buf[0] ==0x0B && Func_code_buf[1] == 0x02)	//增加功率
    {
        if(Data_Assemble(Func_code_buf,(uint8_t *)&Add_PowerBack,CAN_ID,sizeof(Add_PowerBack),&COM_buffer) ==1)
        {
            comSendBuf(COM5,COM_buffer.Data_buffer,COM_buffer.Len_buffer);
            App_printf("COM_buffer: ");
            for(int i=0; i<COM_buffer.Len_buffer; i++)
            {
                App_printf("%02x ",COM_buffer.Data_buffer[i]);
            }
            App_printf("\r\n");
            return 1;
        }
    }
    else if(Func_code_buf[0] ==0x0C && Func_code_buf[1] == 0x02)	//减小功率
    {
        if(Data_Assemble(Func_code_buf,(uint8_t *)&Sub_PowerBack,CAN_ID,sizeof(Sub_PowerBack),&COM_buffer) ==1)
        {
            comSendBuf(COM5,COM_buffer.Data_buffer,COM_buffer.Len_buffer);
            App_printf("COM_buffer: ");
            for(int i=0; i<COM_buffer.Len_buffer; i++)
            {
                App_printf("%02x ",COM_buffer.Data_buffer[i]);
            }
            App_printf("\r\n");
            return 1;
        }
    }
    else if(Func_code_buf[0] ==0x0D && Func_code_buf[1] == 0x02)	//运行图
    {
        if(Data_Assemble(Func_code_buf,(uint8_t *)&Run_DiagramBack,CAN_ID,sizeof(Run_DiagramBack),&COM_buffer) ==1)
        {
            comSendBuf(COM5,COM_buffer.Data_buffer,COM_buffer.Len_buffer);
            App_printf("COM_buffer: ");
            for(int i=0; i<COM_buffer.Len_buffer; i++)
            {
                App_printf("%02x ",COM_buffer.Data_buffer[i]);
            }
            App_printf("\r\n");
            return 1;
        }
    }
    return 2;
}

/*********************************************
这个函数是模拟PC下发的数据
然后算数据长度和CRC值的，用于调试时候
一旦调试完成这个函数就没有实际的作用
***************************************/
void Ata_(pc_data_t *pc_data)
{
    Assemble_buffer_t COM_buffer;
    if(pc_data->uart_pc[0] == 0x02 && pc_data->uart_pc[1] == 0x01)   //表示发查询包
    {
        uint8_t func_code[2] = {0x02,0x01};
        Working_paramet.Transmitte_id[0]= pc_data->uart_pc[2];
        CAN_ID = pc_data->uart_pc[2];
        if(Data_Assemble(func_code,(uint8_t *)&Working_paramet,CAN_ID,sizeof(Working_paramet),&COM_buffer) ==1)
        {
            comSendBuf(COM1,COM_buffer.Data_buffer,COM_buffer.Len_buffer);
        }
    }
    if(pc_data->uart_pc[0] == 0x03 && pc_data->uart_pc[1] == 0x01)   //表示发发射包
    {
        uint8_t func_code[2] = {0x03,0x01};
        Trans_open.Transmitte_id[0]= pc_data->uart_pc[2];
        CAN_ID = pc_data->uart_pc[2];
        Trans_open.Type[0] = pc_data->uart_pc[3];
        Trans_open.Channel[0] = pc_data->uart_pc[4];
        Trans_open.Channel[1] = pc_data->uart_pc[5];
        Trans_open.Channel[2] = pc_data->uart_pc[6];
        for(int  i = 0; i < 12; i++)
        {
            Trans_open.Freq[i] = pc_data->uart_pc[7+i];
        }
        Trans_open.Power_grade[0] = pc_data->uart_pc[20];
        Trans_open.Power_grade[1] = pc_data->uart_pc[21];
        Trans_open.Power_grade[2] = pc_data->uart_pc[22];
        if(Data_Assemble(func_code,(uint8_t *)&Trans_open,CAN_ID,sizeof(Trans_open),&COM_buffer) ==1)
        {
            comSendBuf(COM1,COM_buffer.Data_buffer,COM_buffer.Len_buffer);
        }
    }
    if(pc_data->uart_pc[0] == 0x04 && pc_data->uart_pc[1] == 0x01)   //表示发停止包
    {
        uint8_t func_code[2] = {0x04,0x01};
        Trans_stop.Transmitte_id[0]= pc_data->uart_pc[2];
        CAN_ID = pc_data->uart_pc[2];
        if(Data_Assemble(func_code,(uint8_t *)&Trans_stop,CAN_ID,sizeof(Trans_stop),&COM_buffer) ==1)
        {
            comSendBuf(COM1,COM_buffer.Data_buffer,COM_buffer.Len_buffer);
        }
    }
    if(pc_data->uart_pc[0] == 0x05 && pc_data->uart_pc[1] == 0x01)   //解除报警
    {
        uint8_t func_code[2] = {0x05,0x01};
        Disalarm.Transmitte_id[0]= pc_data->uart_pc[2];
        CAN_ID = pc_data->uart_pc[2];
        Disalarm.Disalarm_type[0] = pc_data->uart_pc[3];
        Disalarm.Disalarm_type[1] = pc_data->uart_pc[4];
        if(Data_Assemble(func_code,(uint8_t *)&Disalarm,CAN_ID,sizeof(Disalarm),&COM_buffer) ==1)
        {
            comSendBuf(COM1,COM_buffer.Data_buffer,COM_buffer.Len_buffer);
        }
    }
    {

    }
}


