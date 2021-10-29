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

extern F_I_t  flash2_arry[480];		//ɨƵ����

extern TaskHandle_t xHandleTask_MT2000;
extern TaskHandle_t xHandleTask_Alarm;
/*---------------------------------����״̬-----------------------------------*/
void update_status_without_inquire(void)
{
    static uint8_t n = 0;

    n++;

    if(	System.status == SYSTEM_UNINITIALIZE )	//δ��ʼ��״̬
    {
        if( n >= 10 )	//1000ms����һ��
        {
            n = 0;
            App_printf("system have no init\r\n");
        }
    }
    else	if( (Alarm.emission==0x01) || (Alarm.no_respond==0x01) || (Alarm.no_power==0x01) )	//����������������Ӧ�𱨾����޹��ʱ���
    {
        //����������������Ӧ�𱨾����޹��ʱ�����פ���ȱ������ڴ󱨾�����ѯʱ���Ϸ��أ������������ȷ��ز���ʧ�ܣ��ص�����/����/����״̬���ȴ���ѯʱ�ٱ���

        System.status = SYSTEM_ALARM;
        System.close = 0;
        System.open = 0;

        System.sweep = 0;
//		System.cancel_sweeping = 0;
        System.achieve_add_sub = 0;
        System.cancel_add_sub = 0;
        System.modify_power = 0;

        if( n >= 10 )	//1000ms����һ��
        {
            n = 0;
            App_printf("history 1 alarm!\r\n");
        }
    }
    else	if( System.status == SYSTEM_OPENING )		//���ڿ���
    {
        if( Power_onBack.results[0] == 0xFD )			//����ʧ��
        {
            App_printf("system open fail\r\n");

            System.status = SYSTEM_ALARM;				//����ʧ�ܣ�ֱ�ӱ���
            Power_onBack.results[0] = 0x00;
        }
        else	if( Power_onBack.results[0]==0xFC )		//�����ɹ�
        {
            System.status = SYSTEM_STANDBY;
            App_printf("system open successfully!\r\n");

            Power_onBack.results[0] = 0x00;
        }
        else	if( Power_onBack.results[0]==0xFE )		//���ڿ���
        {
            System.status = SYSTEM_OPENING;
            if( n >= 10 )	//1000ms����һ��
            {
                n = 0;
                App_printf("system Opening ...\r\n");
            }
        }
    }
    else	if( System.status == SYSTEM_SCAN )			//ɨƵ
    {
        if(System.sweep == 0x02)		//ɨƵ���
        {
            System.sweep = 0;
            System.already_swept = 0x01;
            System.status = SYSTEM_STANDBY;	//����
            System.emission = 0x00;		//����ֹͣ��־λ
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
            if( n >= 10 )	//1000ms����һ��
            {
                n = 0;
                App_printf("system Sweeping ...\r\n");
            }
        }
    }
    else	if( System.status == SYSTEM_SHUTDOWN )		//�ػ�
    {
		System.Voltage = 0.0;
		System.Electricity = 0.0;
		
        Power_onBack.results[0] = 0x00;

        if( n >= 10 )	//1000ms����һ��
        {
            n = 0;
            App_printf("2system close !\r\n");
        }
    }

    else	if( System.status == SYSTEM_STANDBY )		//����״̬
    {
        if(get_history_alarm()==1)						//��ʷ����
        {
            System.sweep = 0;
            System.achieve_add_sub = 0;
//			System_Status_Clean();		//System.emission����
            System.status = SYSTEM_ALARM;

            if( n >= 10 )	//1000ms����һ��
            {
                n = 0;
                Trans_printf("history 3 alarm!\r\n");
            }

            if( System.mode == 0 )					//����
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
        else	if( System.Voltage <= 10.0 && GT2000_Rx.sys_open==0)			//�ػ�״̬
        {
            System.open = 0x00;
            System.status = SYSTEM_SHUTDOWN;
            Trans_printf("1system close !\r\n");
					  App_printf("1system close !\r\n");
        }
    }
    else	if( System.status == SYSTEM_EMISSING )		//����״̬
    {
        if(get_history_alarm()==1)						//��ʷ����
        {
            System.sweep = 0;
            System.achieve_add_sub = 0;
//			System_Status_Clean();		//System.emission����
            System.status = SYSTEM_ALARM;

            if( n >= 10 )	//1000ms����һ��
            {
                n = 0;
                Trans_printf("history 4 alarm!\r\n");
            }

            if( System.mode == 0 )					//����
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
        else	if( System.emission == 0x00 && System.Power_Adjustment!=1)			//����״̬
        {
            System.status = SYSTEM_STANDBY;
            App_printf("waiting state...\r\n");
        }
        else	if( System.emission == 0x01 )			//����״̬
        {
            if( System.modify_power == 0x01 )  //1:������������
            {
                if( n >= 10 )	//1000ms����һ��
                {
                    n = 0;
                    App_printf("modify power...\r\n");
                }
            }
            else	if(System.achieve_add_sub == 1)		//���ӹ������
            {
                System.achieve_add_sub = 0;
                System.modify_power = 0;   //1:������������
                System.cancel_add_sub = 0;
                System.modify_power = 0;
                System.status = SYSTEM_EMISSING;
                App_printf("power adjust successfully\r\n");
            }
            else	if( Add_PowerBack.results[0] == 0xFC )	//���ӹ���ʧ��
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
            else	if( Sub_PowerBack.results[0] == 0xFC )	//��С����ʧ��
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
                if( n >= 10 )	//1000ms����һ��
                {
                    n = 0;
                    App_printf("working state\r\n");
                }

                if( System.mode == 1 )					//����
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
    else	if( System.status == SYSTEM_ALARM )			//����״̬
    {
        if(get_history_alarm()==1)						//��ʷ����
        {
            System.sweep = 0;
            System.achieve_add_sub = 0;
            System.status = SYSTEM_ALARM;

            if( n >= 10 )	//1000ms����һ��
            {
                n = 0;
                App_printf("history 2 alarm!\r\n");
            }

            if( System.mode == 0 )					//����
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
        else	if( System.Voltage >= 40.0 )			//����״̬
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
    else												//����
    {
        App_printf("PC inquire when System.status = ???\r\n");
    }
}

/**************************************************
*p_func_code �Ƿ���ָ�������ָ��
 p_PC_data�Ǵ���Ĵ��ڵ�����
*����1�����ݽ����ɹ������Ҳ�����ȷ
*����2�����ݲ�����������05 02��ͷ������03 04��β
*����3�����ݽ����ɹ������Ƿ����ID��ƥ��
*����4��û�����������
*����5��CRC��������
***************************************************/
uint8_t CAN_data_analyze(uint8_t *p_func_code, Buffer_t *p_PC_data)
{
    uint8_t i = 0;
    uint8_t err_code;
    App_printf("\r\n=====================================\r\n");		//�������
    W5500_data_t W5500_data_buf;

    err_code = judge_is_valid_can_data(p_PC_data);//�ж�������ȷ��
    if( err_code == 0x00 )
        return 0x02;

#if	STM32_W5500_EN
    W5500_data_buf  = Get_CAN_Data(p_PC_data);
#else

#endif

    if(W5500_data_buf.eff_sign == 0)
        return 5;
		
    App_printf("CAN_data_buf.Func_code	= %02X%02X\r\n", W5500_data_buf.Func_code[0],W5500_data_buf.Func_code[1]);
	/*��ȡ��������λ*/
    p_func_code[0] = W5500_data_buf.Func_code[0];
    p_func_code[1] = W5500_data_buf.Func_code[1];

    App_printf(	"CAN_Specific Data	= ");
    for(i = 0; i<(W5500_data_buf.len-3); i++)
    {
        App_printf("%02X ",W5500_data_buf.data_buf[i]);
    }
    App_printf("\r\n=====================================\r\n");


    err_code = Data_Storage(&W5500_data_buf);//��������Ϊ������(��Ч���ݳ��ȣ��������λ)����Ч����,CANid,��Ч���ݱ�־λ
    return err_code;
}


/* �ж��Ƿ�Ϊ��ЧCAN���� */
uint8_t judge_is_valid_can_data(Buffer_t *buffer)
{
#if	!STM32_W5500_EN
    if( buffer->data[0] != 0x05 )
        return 0;

    if( buffer->data[1] != 0x02 )
        return 0;
#endif

    //05 02 03 05 08 01 00 03 00 21 ca 03 05 02 03 05 04
    //03 04��β������03 xx xx xx xx 04��β(�м����0502xxxx֡ͷ)
    if( (buffer->data[buffer->len-2] != 0x03) && (buffer->data[buffer->len-6] != 0x03) )
        return 0;

    if( buffer->data[buffer->len-1] != 0x04 )
        return 0;

    return 1;
}

/* ȥ֡β��У�鲢�ҷ������� */
W5500_data_t Get_CAN_Data(Buffer_t *p_PC_data)
{
	uint8_t i = 0;
    int ReceiveData_CRC;
    unsigned char buf_CRC[2];
    unsigned int CRC_Back;
    unsigned char Vaild[128];
    W5500_data_t CAN_data;

    CAN_data.CAN_ID = p_PC_data->data[2];//�������ݰ��е�ID��

    CAN_data.Func_code[0] = p_PC_data->data[4];//��������������
    CAN_data.Func_code[1] = p_PC_data->data[5];

#if	1
    CAN_data.len = p_PC_data->data[3];//�������ݰ������ݲ��ֵĳ���(�����뿪ʼ��CRC����λǰ����)
    for(i=0;i< CAN_data.len-3;i++)
	{
		CAN_data.data_buf[i]=p_PC_data->data[7+i];//����֡�����ݲ���
	}	
    buf_CRC[0] = p_PC_data->data[p_PC_data->len-4];
    buf_CRC[1] = p_PC_data->data[p_PC_data->len-3];
#else
    while( 1 )
    {
        //02 01 00 05 5b e5 85 95 03 04 03 04 У��͵���0304
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

    CAN_data.len = i+3-2;		//У��Ͳ������ݣ����Լ�2
    buf_CRC[0] = p_PC_data->data[i+1];
    buf_CRC[1] = p_PC_data->data[i+2];
#endif

    ReceiveData_CRC = ((buf_CRC[0]<<8) +(buf_CRC[1]));
    for(i=0;i<CAN_data.len;i++)
	{
		Vaild[i]=p_PC_data->data[4+i];//����λ��ʼ�����ݲ���(�����뿪ʼ)
	}
    CRC_Back = CRC16_XMODEM(Vaild, CAN_data.len);

    if(CRC_Back == ReceiveData_CRC)
    {
        CAN_data.eff_sign = 1;
    }
    else
    {
        CAN_data.eff_sign = 0;//��־�ṹ��������������Ч
        App_printf("CRC����\r\n");
    }

    return CAN_data;
}

/* ���ݸ�ֵ���� */
uint8_t Data_Storage(W5500_data_t *CAN_data_cmp)
{
    uint8_t temp = 0;

    if(CAN_data_cmp->Func_code[0] == 0x01 && CAN_data_cmp->Func_code[1] == 0x01)		//PC�·�����������ֵ
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
    else if(CAN_data_cmp->Func_code[0] == 0x02 && CAN_data_cmp->Func_code[1] == 0x01)	//PC�·�״̬��ѯ
    {
        if( System.CAN_ID[0] == 0xFF )				//Flash��ȡCAN_ID����(�޸���flashλ�ã���һֱ��ʾCAN_IDƥ��ʧ�ܣ�����ӦCAN_ID)
        {
            System.CAN_ID[0] = CAN_data_cmp->data_buf[0];
            Work_paraBack.Transmitte_id[0] = CAN_data_cmp->data_buf[0];	//Ϊ�˲���PC��ʾ���ߣ����ڵ�״̬����ɶ��ɶ�����Ƿ��ص���δ��ʼ��
            return 3;
        }
        else	if( System.CAN_ID[0] == 0x00 )		//��ʼ��CAN_IDʧ��(��һֱ��ʾCAN_IDƥ��ʧ�ܣ�����ӦCAN_ID)
        {
            System.CAN_ID[0] = CAN_data_cmp->data_buf[0];
            Work_paraBack.Transmitte_id[0] = CAN_data_cmp->data_buf[0];	//Ϊ�˲���PC��ʾ���ߣ����ڵ�״̬����ɶ��ɶ�����Ƿ��ص���δ��ʼ��
            return 3;
        }
        else
        {
            if(CAN_data_cmp->data_buf[0] == System.CAN_ID[0])   //��һ���ֽڱ�ʾ�����ID�������ж��������Ƿ���ͬ
            {
                //ʱ���	��С��ת��
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
                Work_paraBack.Transmitte_id[0] = CAN_data_cmp->data_buf[0];	//Ϊ�˲���PC��ʾ���ߣ����ڵ�״̬����ɶ��ɶ�����Ƿ��ص���δ��ʼ��
                return 3;
            }
        }
    }
    else if(CAN_data_cmp->Func_code[0] == 0x03 && CAN_data_cmp->Func_code[1] == 0x01)	//���������
    {
        if(CAN_data_cmp->data_buf[0] == System.CAN_ID[0])   //��һ���ֽڱ�ʾ�����ID�������ж��������Ƿ���ͬ
        {
            PC_Cmd.mode=CAN_data_cmp->data_buf[1];		//����ģʽ
            memcpy(PC_Cmd.channel,		CAN_data_cmp->data_buf+2,	3);		//�ŵ�	��Э��ֻ��һ���ŵ���������������һ��(0x01)
            memcpy(PC_Cmd.freq,			CAN_data_cmp->data_buf+5,	12);	//Ƶ��
            memcpy(PC_Cmd.power,		CAN_data_cmp->data_buf+17,	3);		//���ʵȼ�
            memcpy(System.time,			CAN_data_cmp->data_buf+20,	6);		//����ʱ��
            return 1;
        }
        else
        {
            return 3;
        }
    }
    else if(CAN_data_cmp->Func_code[0] == 0x04 && CAN_data_cmp->Func_code[1] == 0x01)	//�رշ����
    {
        if(CAN_data_cmp->data_buf[0] == System.CAN_ID[0])   //��һ���ֽڱ�ʾ�����ID�������ж��������Ƿ���ͬ
        {
            memcpy(System.time,	CAN_data_cmp->data_buf+1,	6);	//����ʱ��
            return 1;
        }
        else
        {
            return 3;
        }
    }
    else if(CAN_data_cmp->Func_code[0] == 0x05 && CAN_data_cmp->Func_code[1] == 0x01)	//ȡ������
    {
        if(CAN_data_cmp->data_buf[0] == System.CAN_ID[0])   //��һ���ֽڱ�ʾ�����ID�������ж��������Ƿ���ͬ
        {
            memcpy(Disalarm.Disalarm_type, CAN_data_cmp->data_buf+1,2);//�����������
            return 1;
        }
        else
        {
            return 3;
        }
    }
    else if(CAN_data_cmp->Func_code[0] == 0x06 && CAN_data_cmp->Func_code[1] == 0x01)	//��ѯ��ʷ����״̬
    {
        if(CAN_data_cmp->data_buf[0] == System.CAN_ID[0])   //��һ���ֽڱ�ʾ�����ID�������ж��������Ƿ���ͬ
        {
            memcpy(System.time,	CAN_data_cmp->data_buf+1,	6);//����ʱ��
            return 1;
        }
        else
        {
            return 3;
        }
    }
    else if(CAN_data_cmp->Func_code[0] == 0x07 && CAN_data_cmp->Func_code[1] == 0x01)	//������ϵ�
    {
        if(CAN_data_cmp->data_buf[0] == System.CAN_ID[0])   //��һ���ֽڱ�ʾ�����ID�������ж��������Ƿ���ͬ
        {
//            memcpy(Power_on.Retain,CAN_data_cmp->data_buf+1,	1);			//����
            memcpy(System.time,	CAN_data_cmp->data_buf+2,	6);		//����ʱ��
            return 1;
        }
        else
        {
            return 3;
        }
    }
    else if(CAN_data_cmp->Func_code[0] == 0x08 && CAN_data_cmp->Func_code[1] == 0x01)	//������ϵ�
    {
        if(CAN_data_cmp->data_buf[0] == System.CAN_ID[0])   //��һ���ֽڱ�ʾ�����ID�������ж��������Ƿ���ͬ
        {
//            memcpy(Power_off.Retain,CAN_data_cmp->data_buf+1,1);	//����
            return 1;
        }
        else
        {
            return 3;
        }
    }
    else if(CAN_data_cmp->Func_code[0] == 0x09 && CAN_data_cmp->Func_code[1] == 0x01)	//ɨ����ѹ���Ƶ��
    {
        if(CAN_data_cmp->data_buf[0] == System.CAN_ID[0])   //��һ���ֽڱ�ʾ�����ID�������ж��������Ƿ���ͬ
        {
            memcpy(Scan_Frq.state,CAN_data_cmp->data_buf+1,1);		//ɨƵ/��Ƶ
            memcpy(Scan_Frq.Fre_Band,CAN_data_cmp->data_buf+2,8);	//��Ҫ��ѯ��Ƶ��
            return 1;
        }
        else
        {
            return 3;
        }
    }
    else if(CAN_data_cmp->Func_code[0] == 0x0A && CAN_data_cmp->Func_code[1] == 0x01)	//ֹͣɨƵ
    {
        if(CAN_data_cmp->data_buf[0] == System.CAN_ID[0])   //��һ���ֽڱ�ʾ�����ID�������ж��������Ƿ���ͬ
        {
            memcpy(Sacn_stop.state,CAN_data_cmp->data_buf+1,1);
            return 1;
        }
        else
        {
            return 3;
        }
    }
    else if(CAN_data_cmp->Func_code[0] == 0x0B && CAN_data_cmp->Func_code[1] == 0x01)	//���ӹ���
    {
        if(CAN_data_cmp->data_buf[0] == System.CAN_ID[0])   //��һ���ֽڱ�ʾ�����ID�������ж��������Ƿ���ͬ
        {
            memcpy(Add_Power.Power_UP,CAN_data_cmp->data_buf+1,2);
            return 1;
        }
        else
        {
            return 3;
        }
    }
    else if(CAN_data_cmp->Func_code[0] == 0x0C && CAN_data_cmp->Func_code[1] == 0x01)	//��С����
    {
        if(CAN_data_cmp->data_buf[0] == System.CAN_ID[0])   //��һ���ֽڱ�ʾ�����ID�������ж��������Ƿ���ͬ
        {
            memcpy(Sub_Power.Power_DOWN,CAN_data_cmp->data_buf+1,2);
            return 1;
        }
        else
        {
            return 3;
        }
    }
    else if(CAN_data_cmp->Func_code[0] == 0x0D && CAN_data_cmp->Func_code[1] == 0x01)	//����ͼ
    {
        if(CAN_data_cmp->data_buf[0] == System.CAN_ID[0])   //��һ���ֽڱ�ʾ�����ID�������ж��������Ƿ���ͬ
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

    return 4;//û�й�����
}

/********************************************************
		Э������
	*Func_code������
	*p_data �������ݣ�Ҳ���ǽṹ�����������(���ݰ�)
	Can_ID�Ǳ�־��������
	len��p_data�����ݳ���
	Buffer_t *Buffer ���صĽṹ��ָ��
	������Ч���ݵĳ���
********************************************************/
uint8_t Data_Assemble(uint8_t *Func_code, uint8_t *p_data, uint8_t Can_ID, uint8_t len, Buffer_t *Buffer)
{
    uint16_t CRC_Back;
    uint8_t Start_Data[7] = {0x05, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00};
    uint8_t End_Data[4] = {0x00, 0x00, 0x03, 0x04};

    //����
    uint8_t temp = 0x00;
    uint8_t temp_len = 0x00;

    if( Work_paraBack.Transmitte_id[0] != 0x00 )			//�Ѿ���ʼ��������MCU��CANû�н�ƥ��ģ������ˣ��Ͱ��յ���CAN_ID�����أ���ֹ����
    {
        Start_Data[2] = Work_paraBack.Transmitte_id[0];		//д�뷢���ID		--20181031 Luonus
    }

    if( Start_Data[2] == 0x00 )	
    {
        Start_Data[2] = System.CAN_ID[0];				//д�뷢���ID		--20181102 Luonus
    }

    Start_Data[3] = len + 3;			//���ݳ���
    Start_Data[4] = Func_code[0];		//������
    Start_Data[5] = Func_code[1];		//������

#if	ADD_CAN_LEN
    Start_Data[6] = (CEIL_DIV(Start_Data[3]+8-12,8))*4 + Start_Data[3] + 8;
#endif

    memcpy(Buffer->data, Start_Data,7);
    memcpy(Buffer->data+7, p_data, len);
    CRC_Back = CRC16_XMODEM(Buffer->data+4, len+3);
    End_Data[0] = (uint8_t)((CRC_Back >> 8) & 0xff);		//��߰�λ
    End_Data[1] = (uint8_t)((CRC_Back >> 0) & 0xff) ;		//�Ͱ�λ
    memcpy(Buffer->data+len+7, End_Data,4);
    Buffer->len = len+11;

#if	1	//����8λ������
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
	ֻ��Ҫ���ݹ����룬�Զ����͵�����5
	���Ƿ���1��˵��ִ�гɹ�
	����2��˵�����ݵĹ����벻����
	*Func_code_bufָ�������ָ��
****************************************************/
uint8_t Send_PC(uint8_t *Func_code)
{
    uint8_t i = 0;
    uint8_t len = 0;
    uint8_t valid_len = 0;
    Buffer_t COM_buffer;


    if(Func_code[0] ==0x01 && Func_code[1] == 0x02)			//���������������ʼ������Ӧ��
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
										if(S0_State == 0x03)		//03��ʾ������������
										{
											S0_Data&=~S_TRANSMITOK;	//�����־λ
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
										if(S0_State == 0x03)		//03��ʾ������������
										{
											S0_Data&=~S_TRANSMITOK;	//�����־λ
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
    else	if(Func_code[0] ==0x02 && Func_code[1] == 0x02)	//��ѯ
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
 					if(S0_State == 0x03)		//03��ʾ������������
					{
						S0_Data&=~S_TRANSMITOK;	//�����־λ
						Process_Socket_Data(0,COM_buffer.data+8*i, 8);
					}
                    len = len - 8;
                    if( len == 0x00 )
                        break;
                }
                else
                {
					if(S0_State == 0x03)		//03��ʾ������������
					{
						S0_Data&=~S_TRANSMITOK;	//�����־λ
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
    else	if(Func_code[0] ==0x03 && Func_code[1] == 0x02)	//����
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
					if(S0_State == 0x03)		//03��ʾ������������
					{
						S0_Data&=~S_TRANSMITOK;	//�����־λ
						Process_Socket_Data(0,COM_buffer.data+8*i, 8);
					}
                    len = len - 8;
                    if( len == 0x00 )
                        break;
                }
                else
                {
					if(S0_State == 0x03)		//03��ʾ������������
					{
						S0_Data&=~S_TRANSMITOK;	//�����־λ
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
    else	if(Func_code[0] ==0x04 && Func_code[1] == 0x02)	//ֹͣ
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
					if(S0_State == 0x03)		//03��ʾ������������
					{
						S0_Data&=~S_TRANSMITOK;	//�����־λ
						Process_Socket_Data(0,COM_buffer.data+8*i, 8);
					}
                    len = len - 8;
                    if( len == 0x00 )
                        break;
                }
                else
                {
					if(S0_State == 0x03)		//03��ʾ������������
					{
						S0_Data&=~S_TRANSMITOK;	//�����־λ
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
    else	if(Func_code[0] ==0x05 && Func_code[1] == 0x02)	//�������
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
					if(S0_State == 0x03)		//03��ʾ������������
					{
						S0_Data&=~S_TRANSMITOK;	//�����־λ
						Process_Socket_Data(0,COM_buffer.data+8*i, 8);
					}
                    len = len - 8;
                    if( len == 0x00 )
                        break;
                }
                else
                {
					if(S0_State == 0x03)		//03��ʾ������������
					{
						S0_Data&=~S_TRANSMITOK;	//�����־λ
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
    else	if(Func_code[0] ==0x06 && Func_code[1] == 0x02)	//������Ϣ��ѯ
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
 					if(S0_State == 0x03)		//03��ʾ������������
					{
						S0_Data&=~S_TRANSMITOK;	//�����־λ
						Process_Socket_Data(0,COM_buffer.data+8*i, 8);
					}
                    len = len - 8;
                    if( len == 0x00 )
                        break;
                }
                else
                {
					if(S0_State == 0x03)		//03��ʾ������������
					{
						S0_Data&=~S_TRANSMITOK;	//�����־λ
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
    else	if(Func_code[0] ==0x07 && Func_code[1] == 0x02)	//����
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
					if(S0_State == 0x03)		//03��ʾ������������
					{
						S0_Data&=~S_TRANSMITOK;	//�����־λ
						Process_Socket_Data(0,COM_buffer.data+8*i, 8);
					}
                    len = len - 8;
                    if( len == 0x00 )
                        break;
                }
                else
                {
					if(S0_State == 0x03)		//03��ʾ������������
					{
						S0_Data&=~S_TRANSMITOK;	//�����־λ
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
    else	if(Func_code[0] ==0x08 && Func_code[1] == 0x02)	//�ػ�
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
					if(S0_State == 0x03)		//03��ʾ������������
					{
						S0_Data&=~S_TRANSMITOK;	//�����־λ
						Process_Socket_Data(0,COM_buffer.data+8*i, 8);
					}
                    len = len - 8;
                    if( len == 0x00 )
                        break;
                }
                else
                {
					if(S0_State == 0x03)		//03��ʾ������������
					{
						S0_Data&=~S_TRANSMITOK;	//�����־λ
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
    else	if(Func_code[0] ==0x09 && Func_code[1] == 0x02)	//ɨƵ
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
                	if(S0_State == 0x03)		//03��ʾ������������
					{
						S0_Data&=~S_TRANSMITOK;	//�����־λ
						Process_Socket_Data(0,COM_buffer.data+8*i, 8);
					}
                len = len - 8;
                if( len == 0x00 )
                    break;
            }
            else
            {
                	if(S0_State == 0x03)		//03��ʾ������������
					{
						S0_Data&=~S_TRANSMITOK;	//�����־λ
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
    else	if(Func_code[0] ==0x0A && Func_code[1] == 0x02)	//ֹͣɨƵ
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
					if(S0_State == 0x03)		//03��ʾ������������
					{
						S0_Data&=~S_TRANSMITOK;	//�����־λ
						Process_Socket_Data(0,COM_buffer.data+8*i, 8);
					}
                    len = len - 8;
                    if( len == 0x00 )
                        break;
                }
                else
                {
					if(S0_State == 0x03)		//03��ʾ������������
					{
						S0_Data&=~S_TRANSMITOK;	//�����־λ
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
    else	if(Func_code[0] ==0x0B && Func_code[1] == 0x02)	//���ӹ���
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
 					if(S0_State == 0x03)		//03��ʾ������������
					{
						S0_Data&=~S_TRANSMITOK;	//�����־λ
						Process_Socket_Data(0,COM_buffer.data+8*i, 8);
					}
                    len = len - 8;
                    if( len == 0x00 )
                        break;
                }
                else
                {
					if(S0_State == 0x03)		//03��ʾ������������
					{
						S0_Data&=~S_TRANSMITOK;	//�����־λ
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
    else	if(Func_code[0] ==0x0C && Func_code[1] == 0x02)	//��С����
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
					if(S0_State == 0x03)		//03��ʾ������������
					{
						S0_Data&=~S_TRANSMITOK;	//�����־λ
						Process_Socket_Data(0,COM_buffer.data+8*i, 8);
					}
                    len = len - 8;
                    if( len == 0x00 )
                        break;
                }
                else
                {
					if(S0_State == 0x03)		//03��ʾ������������
					{
						S0_Data&=~S_TRANSMITOK;	//�����־λ
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
    else	if(Func_code[0] ==0x0D && Func_code[1] == 0x02)	//����ͼ
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
					if(S0_State == 0x03)		//03��ʾ������������
					{
						S0_Data&=~S_TRANSMITOK;	//�����־λ
						Process_Socket_Data(0,COM_buffer.data+8*i, 8);
					}
                    len = len - 8;
                    if( len == 0x00 )
                        break;
                }
                else
                {
					if(S0_State == 0x03)		//03��ʾ������������
					{
						S0_Data&=~S_TRANSMITOK;	//�����־λ
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

/*---------------------------------ɨƵ�ϵ㱣�����ȡ-------------------------------------*/
//��ȡɨƵ�ϵ㣬��������һ��Ƶ��
uint16_t Scan_Breakpoint_Read(void)
{
	uint8_t		buf[2];
	uint16_t	next_point = 0;
	
	FLASH_Read(SCAN_BREAKPOINT_ADDRESS,		buf,	SCAN_BREAKPOINT_LEN);
	
	next_point = 10*buf[0] + buf[1];
	return next_point;
}

//��ȡɨƵ�ϵ�
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


/*------------------------------------ɨƵ����1----------------------------------------*/
void Band_scan(uint8_t freq_begin, uint8_t freq_end, uint8_t index_dec)
{
    int ret = 0,Flag=0,n;
	uint8_t protect_count = 0;		//��������
	char freq_scan_to_Tx[4];    //����ʮ������ɨƵ�·���Ƶ������
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
	
    for(temp_point=temp_begin; temp_point<=temp_end; temp_point++)   //��freq_begin��ɨ freq_end����
    {
		if( re_scan_flag == 0x01 )		//������Ч��ɨ�����ݣ�ɨ�谴ť�ѱ�����
		{
			temp_swr = SWR_array[(temp_point-3)*10+index_dec];
			if( (temp_swr < 1.0) || (temp_swr >= 2.0) )	//unvalid data  ��Ч����
			{
				//scan
			}
			else	if( temp_point == temp_end )		//end  ���ɨƵ�����յ�һ�����˳�
			{
				return;
			}
			else	//the data is well  ���ݺܺã�������һ��Ƶ��
			{
				continue;
			}
		}
		
        if( (System.cancel_sweeping == 0x01) || (System.stop == 0x01) )		//ȡ��ɨƵ
        {
            GT2000_Stop();  //ֹͣ����
	
            System.sweep = 0;
            System.stop = 0;
            System.achieve_add_sub = 0;
            System.modify_power = 0;
            System.cancel_add_sub = 0;
            System_Status_Clean();
            System.status = SYSTEM_STANDBY;
				
						Scan_Breakpoint_Save(temp_point, index_dec);  //����ɨƵ�ϵ㣬�˳�ɨƵ
            return;
        }
        else	if( System.Voltage < 40.0 )  //����ǹػ�״̬���˳�ɨƵ
        {
            System.open = 0x00;
            return;
        }
		
		System.status = SYSTEM_SCAN;  //ɨƵ״̬
    System.sweep = 0x01;  //ɨƵ��־λ	1:����ɨƵ��2:ɨƵ��ɣ�����status��־λ������sweep����		��󣬱�����flash
		
		//channel method����MT2000_Cmd_Channel()�̶�д���˵ģ�����Ҫ������
		GT2000_Tx.Gt2000_mode = 1;		//��Ƶ
		GT2000_Tx.Frequency_power[1] = 0x04 ;		//��������ɨƵ����ֵ1040W
    GT2000_Tx.Frequency_power[0] = 0x10 ;		
//		MT2000_Tx.method = 'F';		//FM
		
		n=(temp_point*1000)+(index_dec*100);  //ɨƵƵ������1000��
		sprintf(freq_scan_to_Tx, "%X", n); //��ʮ��������תΪ16���ƴ�д�ַ���
	  StrToHex(test_to_Tx,(uint8_t*)freq_scan_to_Tx, 2); //remarks : ���ַ���ת��Ϊ16������	������test_to_Tx				
		GT2000_Tx.Gt2000_freq1[0] = test_to_Tx[1];
		GT2000_Tx.Gt2000_freq1[1] = test_to_Tx[0];
		GT2000_Tx.Gt2000_freq1[2] = 0x00;					//С���㱣�ֲ���
		GT2000_Tx.Gt2000_freq1[3] = 0x00;
				Trans_printf(" scan1���ڵ�ɨƵƵ��: temp_point= %d index_dec= %d \n",temp_point,index_dec);
				Trans_printf(" ɨƵ�·���scan1_freq_Tx: %02X %02X ",GT2000_Tx.Gt2000_freq1[0],GT2000_Tx.Gt2000_freq1[1]);

		temp_Forward_power = 0.0;
		temp_Reverse_power = 0.0;
		temp_Standing_wave_ratio = 0.0;

		next_point = Scan_Breakpoint_Read(); //��FLASH�ж�ȡɨƵ�ϵ�
		next_point_int = next_point/10;
		next_point_dec = next_point%10;
		
		//��Ч���ݣ���һ��Ƶ����������ִ��ڵ�ǰƵ�㣬С��������ͬ����ɨ��һ��Ƶ��
		if( (next_point!=0x0000) && (next_point!=0xFFFF) && (next_point_int>temp_point) && (next_point_dec==index_dec) )
		{
			continue;
		}

        Flag=GT2000_Emit();
		    Trans_printf(" 1_T0_GT2000_Emit() Flag= %d \n",Flag);
		    vTaskDelay(6000);
				Gt_Inquire_All();		//��ѯһ��
				Flag=GT2000_Wait_Ack(MT2000_CMD_CHANNEL);
		
	    	if(Flag==MT2000_ACK_ALARM)
				{		
					temp_Forward_power = 0.0;
					temp_Reverse_power = 0.0;
					temp_Standing_wave_ratio = SCAN_ALARM_SWR_THRESHOLD;
					Scan_Breakpoint_Save(temp_point+1, index_dec);	
					continue;
				}
        if( System.emission == 0x01 )	//����ɹ�
        {
            App_printf("Sweep//����ɹ�\r\n");

            temp_Forward_power = System.Forward_Power;
            temp_Reverse_power = System.Reverse_Power;
            temp_Standing_wave_ratio = System.Standing_wave_ratio;

            //��Ӧ�𱨾����޹��ʱ���
            if( (Alarm.no_respond==0x01) || (Alarm.no_power==0x01) )					//�˳�ɨƵ
            {
                System.cancel_sweeping = 0x01;		//�˳�ɨƵ
                GT2000_Stop();

								System.status = SYSTEM_ALARM;
                System.sweep = 0;
                System.stop = 0;
				
								Scan_Breakpoint_Save(temp_point, index_dec);
                return;
            }
							if( Alarm.emission == 0x01 )										//�˳�ɨƵ�������´�ɨƵʱ���˳���ǰƵ��
						{
							System.cancel_sweeping = 0x01;		//�˳�ɨƵ
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
							 else	if( (Alarm.swr_alarm == 0x01) || (Alarm.over_Electric==0x01) )		//������ǰƵ��
						{
							//�ȼ�¼�µ�ǰ��ѹ����פ���ȣ���ֹͣ
			//				MT2000_Cmd_Inquire();			//��ѯ
			//				ret = MT2000_Wait_Ack(MT2000_CMD_INQUIRE);

							Alarm.swr_alarm = 0x00;
							Alarm.over_Electric = 0x00;
							
							temp_Forward_power = System.Forward_Power;
							temp_Reverse_power = System.Reverse_Power;
							temp_Standing_wave_ratio = System.Standing_wave_ratio;
							
							GT2000_Stop();

							//�Ļ�ɨƵ״̬������ɨ��һ��Ƶ��
							System.status = SYSTEM_SCAN;
							System.sweep = 0x01;
							System.stop = 0;
							
							Scan_Breakpoint_Save(temp_point+1, index_dec);
						}
					}
				
        else							//����ʧ��
        {
            App_printf("Sweep//����ʧ��\r\n");

						System.status = SYSTEM_ALARM;
						System.sweep = 0;
						System.stop = 0;
			
            temp_Forward_power = 0.0;
            temp_Reverse_power = 0.0;
            temp_Standing_wave_ratio = SCAN_ALARM_SWR_THRESHOLD;

						if( Alarm.emission == 0x01 )									//�˳�ɨƵ�������´�ɨƵʱ���˳���ǰƵ��
						{
							System.cancel_sweeping = 0x01;		//�˳�ɨƵ
							GT2000_Stop();
							System.status = SYSTEM_ALARM;
							System.sweep = 0;
							System.stop = 0;
							
							Scan_Breakpoint_Save(temp_point+1, index_dec);							//��гʱ��������ֹͣ��������ǰƵ��
						}				
						else
						{
							Scan_Breakpoint_Save(temp_point+1, index_dec);
						}
        }

				vTaskDelay(4000);
				Gt_Inquire_All();		//��ѯһ��
				ret=GT2000_Wait_Ack(MT2000_CMD_CHANNEL);
				if(ret==MT2000_ACK_OK)
				{
					temp_Forward_power = System.Forward_Power;
					temp_Reverse_power = System.Reverse_Power;
					temp_Standing_wave_ratio = System.Standing_wave_ratio;
				}
				vTaskDelay(2000);
        //�����±�Ϊ	( Ƶ��(MHz)-5 )*10
        //				5.0MHz => Forward_Power_array[0]	5.1MHz => Forward_Power_array[1]
        //				6.0MHz => Forward_Power_array[10]	6.1MHz => Forward_Power_array[11]
        //	index_dec:0~9	i:freq_end-freq_begin = 26.0-5.0 = 21
        Forward_Power_array[(temp_point-3)*10+index_dec] = temp_Forward_power/10.0;			//232����Ҫ���flash������ɨƵ���
        Reverse_Power_array[(temp_point-3)*10+index_dec] = temp_Reverse_power;					//����Ҫ�棬Ҳ����Ҫ���أ����Բ���
        SWR_array[(temp_point-3)*10+index_dec] = temp_Standing_wave_ratio;	//232����Ҫ���flash������ɨƵ���
				
				Gt_RF_Close();		
				vTaskDelay(500);   //�·������Ӧ����
				ret=MT2000_ACK_OK;	//Ĭ�ϳɹ�������β���

        if( ret == MT2000_ACK_OK )							//ֹͣ�ɹ�
        {
            System.stop = 0x00;
            App_printf("Sweep//����ֹͣ\r\n");

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

            System.status = SYSTEM_SCAN;	//����ɨƵ
            System.sweep = 0x01;

            if(temp_point<temp_end)
            {
                for(protect_count=0; protect_count<50; protect_count++)
                {
                    if( (System.cancel_sweeping == 0x01) || (System.stop == 0x01)  )		//ȡ��ɨƵ
                    {
												Scan_Breakpoint_Save(temp_point+1, index_dec);
                        return;
                    }
                    else
                    {
                        vTaskDelay(100);						//��������
                    }
                }
            }
						else
						{
							Scan_Breakpoint_Save(temp_point+1, index_dec);
							return;
						}
        }
        else     //���ɹ�������ֹͣʧ��
        {
            Alarm.no_respond_locate = 5;
            App_printf("Sweep stop 5 no respond.\r\n");
            return;
        }
    }
    return;
}

/*------------------------------------ɨƵ����2----------------------------------------*/
void Optional_Band_scan(uint8_t Now_Frq,uint8_t Scan_end)
{
	int ret = 0,Flag=0,n;
	uint8_t protect_count = 0;		//��������
	char freq_scan_to_Tx[4]={0x00};    //����ʮ������ɨƵ�·���Ƶ������
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
		
        if( (System.cancel_sweeping == 0x01) || (System.stop == 0x01) )		//ȡ��ɨƵ
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
					  Trans_printf(" System.Voltage < 40.0 �˳�ɨƵ��\n");
            return;
        }

				System.status = SYSTEM_SCAN;
				System.sweep = 0x01;//ɨƵ��־λ	1:����ɨƵ��2:ɨƵ��ɣ�����status��־λ������sweep����		��󣬱�����flash
				
				//channel method����MT2000_Cmd_Channel()�̶�д���˵ģ�����Ҫ������
				GT2000_Tx.Gt2000_mode = 1;		//��Ƶ
				GT2000_Tx.Frequency_power[1] = 0x04 ;		//������ͨ����ɨƵ����ֵ1001
				GT2000_Tx.Frequency_power[0] = 0x10 ;		
		//		MT2000_Tx.method = 'F';		//FM
				
				n=(temp_point*1000)+(index_dec*100);  //ɨƵƵ������1000��
				Trans_printf(" scan2Ƶ��n=  %d \n",n);
				sprintf(freq_scan_to_Tx, "%04X", n); //��ʮ��������תΪ16���ƴ�д�ַ���
				StrToHex(test_to_Tx,(uint8_t*)freq_scan_to_Tx, 2); //remarks : ���ַ���ת��Ϊ16������	������test_to_Tx				
				GT2000_Tx.Gt2000_freq1[0] = test_to_Tx[1];
				GT2000_Tx.Gt2000_freq1[1] = test_to_Tx[0];
				GT2000_Tx.Gt2000_freq1[2] = 0x00;					//С���㱣�ֲ���
				GT2000_Tx.Gt2000_freq1[3] = 0x00;
						Trans_printf(" scan2���ڵ�ɨƵƵ��: temp_point= %d index_dec= %d freq_scan_to_Tx=%02x %02x \n",temp_point,index_dec,freq_scan_to_Tx[0],freq_scan_to_Tx[1]);
						Trans_printf(" ɨƵ�·���scan2_freq_Tx: %02X %02X ",GT2000_Tx.Gt2000_freq1[0],GT2000_Tx.Gt2000_freq1[1]);

				temp_Forward_power = 0.0;
				temp_Reverse_power = 0.0;
				temp_Standing_wave_ratio = 0.0;


        Flag=GT2000_Emit();   //�������䣬�����渳ֵ�������
				Trans_printf(" 2_T0_GT2000_Emit() Flag= %d \n",Flag);
				vTaskDelay(6000);
				Gt_Inquire_All();		//��ѯһ��
				Flag=GT2000_Wait_Ack(MT2000_CMD_CHANNEL);
				
				if(Flag==MT2000_ACK_ALARM)
				{		
					temp_Forward_power = 0.0;
					temp_Reverse_power = 0.0;
					temp_Standing_wave_ratio = SCAN_ALARM_SWR_THRESHOLD;
					Scan_Breakpoint_Save(temp_point+1, index_dec);	
					return;
				}

        if( System.emission == 0x01 )	//����ɹ�
        {
					temp_Forward_power = System.Forward_Power;
					temp_Reverse_power = System.Reverse_Power;
					temp_Standing_wave_ratio = System.Standing_wave_ratio;

            //��Ӧ�𱨾����޹��ʱ���
            if( (Alarm.no_respond==0x01) || (Alarm.no_power==0x01) )					//�˳�ɨƵ
            {
                System.cancel_sweeping = 0x01;		//�˳�ɨƵ
                GT2000_Stop();

								System.status = SYSTEM_ALARM;
                System.sweep = 0;
                System.stop = 0;
				         
							  Trans_printf(" ��Ӧ�𱨾����޹��ʱ������˳�ɨƵ \n");
								Scan_Breakpoint_Save(temp_point, index_dec);
                return;
            }
				if( Alarm.emission == 0x01 )										//�˳�ɨƵ�������´�ɨƵʱ���˳���ǰƵ��
			{
				System.cancel_sweeping = 0x01;		//�˳�ɨƵ
				GT2000_Stop();
				System.status = SYSTEM_ALARM;
				System.sweep = 0;
				System.stop = 0;
				
				temp_Forward_power = 0.0;
				temp_Reverse_power = 0.0;
				temp_Standing_wave_ratio = SCAN_ALARM_SWR_THRESHOLD;
				
				Trans_printf(" ���������������˳�ɨƵ \n");
				Scan_Breakpoint_Save(temp_point+1, index_dec);
				return;
			}
         else	if( (Alarm.swr_alarm == 0x01) || (Alarm.over_Electric==0x01) )		//������ǰƵ��
			{
				Alarm.swr_alarm = 0x00;
				Alarm.over_Electric = 0x00;
				
				temp_Forward_power = System.Forward_Power;
				temp_Reverse_power = System.Reverse_Power;
				temp_Standing_wave_ratio = System.Standing_wave_ratio;
				
				GT2000_Stop();

				//�Ļ�ɨƵ״̬������ɨ��һ��Ƶ��
				System.status = SYSTEM_SCAN;
				System.sweep = 0x01;
				System.stop = 0;
				
				Trans_printf(" פ���ȱ����������������˳�ɨƵ \n");
				Scan_Breakpoint_Save(temp_point+1, index_dec);
			}
           
        }
        else							//����ʧ��
        {
					Trans_printf(" ɨƵ����ʧ�ܡ��˳�ɨƵ \n");
						System.status = SYSTEM_ALARM;
						System.sweep = 0;
						System.stop = 0;
			
            temp_Forward_power = 0.0;
            temp_Reverse_power = 0.0;
            temp_Standing_wave_ratio = SCAN_ALARM_SWR_THRESHOLD;


					if( Alarm.emission == 0x01 )									//�˳�ɨƵ�������´�ɨƵʱ���˳���ǰƵ��
					{
						System.cancel_sweeping = 0x01;		//�˳�ɨƵ
						GT2000_Stop();
						System.status = SYSTEM_ALARM;
						System.sweep = 0;
						System.stop = 0;
						
						Scan_Breakpoint_Save(temp_point+1, index_dec);							//��гʱ��������ֹͣ��������ǰƵ��
					}				
					else
					{
						Scan_Breakpoint_Save(temp_point+1, index_dec);
					}
        }
				
				vTaskDelay(4000);
				Gt_Inquire_All();		//��ѯһ��
				ret=GT2000_Wait_Ack(MT2000_CMD_CHANNEL);
				if(ret==MT2000_ACK_OK)
				{
					temp_Forward_power = System.Forward_Power;
					temp_Reverse_power = System.Reverse_Power;
					temp_Standing_wave_ratio = System.Standing_wave_ratio;
				}
				vTaskDelay(2000);
        //�����±�Ϊ	( Ƶ��(MHz)-5 )*10
        //				5.0MHz => Forward_Power_array[0]	5.1MHz => Forward_Power_array[1]
        //				6.0MHz => Forward_Power_array[10]	6.1MHz => Forward_Power_array[11]
        //	index_dec:0~9	i:freq_end-freq_begin = 26.0-5.0 = 21
        Forward_Power_array[(temp_point-3)*10+index_dec] = temp_Forward_power/10.0;			//233����Ҫ���flash������ɨƵ���
        Reverse_Power_array[(temp_point-3)*10+index_dec] = temp_Reverse_power;					//����Ҫ�棬Ҳ����Ҫ���أ����Բ���
        SWR_array[(temp_point-3)*10+index_dec] = temp_Standing_wave_ratio;	//233����Ҫ���flash������ɨƵ���
				
				Gt_RF_Close();		//�ط���
				vTaskDelay(50);
				ret=MT2000_ACK_OK;			//Ĭ�ϳɹ�

        if( ret == MT2000_ACK_OK )							//ֹͣ�ɹ�
        {
            System.stop = 0x00;
            App_printf("Sweep//����ֹͣ\r\n");

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

            System.status = SYSTEM_SCAN;	//����ɨƵ
            System.sweep = 0x01;

            if(temp_point<temp_end)
            {
                for(protect_count=0; protect_count<50; protect_count++)
                {
                    if( (System.cancel_sweeping == 0x01) || (System.stop == 0x01)  )		//ȡ��ɨƵ
                    {
												Scan_Breakpoint_Save(temp_point+1, index_dec);
                        return;
                    }
                    else
                    {
                        vTaskDelay(100);						//��������
                    }
                }
            }
						else
						{
							Scan_Breakpoint_Save(temp_point+1, index_dec);
							return;
						}
        }
        else     //���ɹ�������ֹͣʧ��
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
	Trans_printf(" ��ӡ������g_fuc_cod : ");
    if(g_fuc_cod[0]==0x01&&g_fuc_cod[1]==0x01)				//��ʼ��
        Trans_printf("��ʼ��\r\n");
    else	if(g_fuc_cod[0]==0x02&&g_fuc_cod[1]==0x01)		//��ѯ
        Trans_printf("��ѯ\r\n");
    else	if(g_fuc_cod[0]==0x03&&g_fuc_cod[1]==0x01)		//����
        Trans_printf("����\r\n");
    else	if(g_fuc_cod[0]==0x04&&g_fuc_cod[1]==0x01)		//ֹͣ
        Trans_printf("ֹͣ\r\n");
    else	if(g_fuc_cod[0]==0x05&&g_fuc_cod[1]==0x01)		//�������
        Trans_printf("�������\r\n");
    else	if(g_fuc_cod[0]==0x06&&g_fuc_cod[1]==0x01)		//������ѯ
        Trans_printf("������ѯ\r\n");
    else	if(g_fuc_cod[0]==0x07&&g_fuc_cod[1]==0x01)		//����
        Trans_printf("����\r\n");
    else	if(g_fuc_cod[0]==0x08&&g_fuc_cod[1]==0x01)		//�ػ�
        Trans_printf("�ػ�\r\n");
    else	if(g_fuc_cod[0]==0x09&&g_fuc_cod[1]==0x01)		//ɨƵ
        Trans_printf("ɨƵ\r\n");
    else	if(g_fuc_cod[0]==0x0A&&g_fuc_cod[1]==0x01)		//ֹͣɨƵ
        Trans_printf("ֹͣɨƵ\r\n");
    else	if(g_fuc_cod[0]==0x0B&&g_fuc_cod[1]==0x01)		//���ӹ���
        Trans_printf("���ӹ���\r\n");
    else	if(g_fuc_cod[0]==0x0C&&g_fuc_cod[1]==0x01)		//��С����
        Trans_printf("��С����\r\n");
    else	if(g_fuc_cod[0]==0x0D&&g_fuc_cod[1]==0x01)		//����ͼ
        Trans_printf("����ͼ\r\n");
}

void freq_hex_to_str(uint8_t *freq_hex, uint8_t *freq_str)		//hex��Ƶ��ת��ΪstrƵ��
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
*   freq_str_to_hex�˺���Ϊ��Э��ʹ��
*/
void freq_str_to_hex(uint8_t *freq_str, uint8_t *freq_hex)		//str��Ƶ��ת��ΪhexƵ��
{
    *(freq_hex+0) = ( ( *(freq_str+0)-'0') << 4 ) + ( *(freq_str+1)-'0');
    *(freq_hex+1) = ( ( *(freq_str+2)-'0') << 4 ) + ( *(freq_str+3)-'0');
    *(freq_hex+2) = ( ( *(freq_str+4)-'0') << 4 ) + ( *(freq_str+5)-'0');
    *(freq_hex+3) = ( ( *(freq_str+6)-'0') << 4 ) + ( *(freq_str+7)-'0');
}  


/*���ַ���sת������Ӧ������*/
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
 
 

void freq_PChex_to_GT2000( uint8_t *freq_PC_str,uint8_t *freq_Trans_hex)  //PC�·���Ƶ�ʸ�ֵ��GT2000����λ��ǰ
{
  int n;
	char str[4];
	unsigned char test[2] = {0};
	uint8_t arry[2];   //����ǰ��λ��ַ�е�����
	arry[0] = *(freq_PC_str + 0);
	arry[1] = *(freq_PC_str + 1);
	if((arry[0]!=0x00)||(arry[1]!=0x00))  //PC�·���ǰ��λ��Ϊ0�Ž����ж�
	{
	freq_Trans_hex = mystrncpy(freq_Trans_hex, 5);//��ȡǰ5���ַ���
	n = str_to_10D(freq_Trans_hex);   //���ַ���freq_Trans_hexת������Ӧ������
	sprintf(str, "%X", n); //��ʮ��������תΪ16���ƴ�д�ַ���
	StrToHex(test,(uint8_t*)str, 2); //remarks : ���ַ���ת��Ϊ16������
//	printf(" ת����Ƶ��test== %02X  %02X",test[0],test[1]);
	*(freq_Trans_hex + 1) = test[0];
	*(freq_Trans_hex + 0) = test[1];
	*(freq_Trans_hex + 2) = 0x00;
	*(freq_Trans_hex + 3) = 0x00;
	}
	else
	{
		memset((freq_Trans_hex + 0),0x00,4);  //����PC��Ƶʱ��ֵ��������Ƶ�ʡ�
//		Trans_printf(" ��ЧƵ�� ");
	}
}

void freq_GT2000Hex_to_PChex(unsigned char *freq_Trans_hex, unsigned char *freq_PC_Hex)  //GT2000��Ƶ�ʸ�ֵ��GT2000����λ��ǰ
{
	int n = 0;
  unsigned char freq[6];
	freq[5] = '\0';
	n = (*(freq_Trans_hex + 0) << 8) | (*(freq_Trans_hex + 1));
	printf(" ��ȡ����ʮ������Ƶ��Ϊ= %X \n", n);
	sprintf((char *)freq, "%05d", n); //��ʮ����������תΪʮ�����ַ���
	
		freq_str_to_hex(freq, freq_PC_Hex);
		Trans_printf(" ��ȡ����ʮ����Ƶ��Ϊfreq= %s \n", freq);
	
	*(freq_PC_Hex + 2) = 0x00;
	*(freq_PC_Hex + 3) = 0x00;

	Trans_printf(" ת�����Ƶ��Ϊ= %02X %02X %02X %02X \n", freq_PC_Hex[0], freq_PC_Hex[1], freq_PC_Hex[2], freq_PC_Hex[3]);

}

void GT2000_Tx_freq_Hex_to_PChex(unsigned char *freq_Trans_hex, unsigned char *freq_PC_Hex)  //����ѯɨƵ����ʹ��
{
	int n = 0;
  unsigned char freq[6];
	freq[5] = '\0';
	n = (*(freq_Trans_hex + 1) << 8) | (*(freq_Trans_hex + 0));
	printf(" 2��ȡ����ʮ������Ƶ��Ϊ= %X \n", n);
	sprintf((char *)freq, "%05d", n); //��ʮ����������תΪʮ�����ַ���
	
		freq_str_to_hex(freq, freq_PC_Hex);
		Trans_printf(" 2��ȡ����ʮ����Ƶ��Ϊfreq= %s \n", freq);
	
	*(freq_PC_Hex + 2) = 0x00;
	*(freq_PC_Hex + 3) = 0x00;

	Trans_printf(" 2ת�����Ƶ��Ϊ= %02X %02X %02X %02X \n", freq_PC_Hex[0], freq_PC_Hex[1], freq_PC_Hex[2], freq_PC_Hex[3]);

}

uint8_t *mystrncpy( uint8_t *string, int n)   //Ҫ���ȡ���ַ��������Ըı䣬��ָ���ַ�����ָ����Ըı�
{
	uint8_t *p = string;
	if (p == NULL)
	{//�����ȡ���ַ����ǿյ�ֱ�ӷ���
		return NULL;
	}
	else
	{
		int i = 0;
		while (*p != '\0')
		{//ѭ��ֱ����n���ַ�����ֹ
			if (i == n)
			{
				break;
			}
			i++;
			p++;
		}
		*(p++) = '\0';//��ֵ�����ַ���
		return string;
	}
}

/*
// C prototype : void StrToHex(BYTE *pbDest, BYTE *pbSrc, int nLen)
// parameter(s): [OUT] pbDest - ���������
// [IN] pbSrc - �ַ���
// [IN] nLen - 16���������ֽ���(�ַ����ĳ���/2)
// return value:
// remarks : ���ַ���ת��Ϊ16������
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


void set_run_diagram_new_power_level(uint8_t channel, uint8_t power_level)	//��������ͼ�Ĺ��ʵȼ�				Run_Diagram_data
{
   Run_Diagram_data.power[channel-'1'] = power_level;
}

void freq_range_judge(uint8_t *freq)  //Ƶ��ֵ�˶Ա���
{
	int freq_float=0;
//	freq_float=((freq[0]-'0')*10000+(freq[1]-'0')*1000+(freq[2]-'0')*100+(freq[3]-'0')*10+(freq[4]-'0'))/1000.0;
	freq_float=(freq[1]<<8|freq[0]);
	
	Trans_printf(" freq_float= %04x ",freq_float);
	if(0x0140<=freq_float&&freq_float<=0x14B4)
	{
		System.fbv_c=1;		//һƵ��
	}
	
	else if(0x14B4<=freq_float&&freq_float<=0x1B58)
	{
		System.fbv_c=2;		//��Ƶ��
	}
	
	else if(0x1B58<freq_float&&freq_float<=0x251C)
	{
		System.fbv_c=3;		//��Ƶ��
	}
	
	else if(0x251C<freq_float&&freq_float<=0x3A98)
	{
		System.fbv_c=4;		//��Ƶ��
	}
	
	else if(0x3A98<freq_float&&freq_float<=0x555A)
	{
		System.fbv_c=5;		//��Ƶ��
	}
	else if(0x555A<freq_float&&freq_float<=0x65F4)
	{
		System.fbv_c=6;		//��Ƶ��
	}
	Trans_printf(" Ƶ��ֵ�˶Ա���freq_float== %04x System.fbv_c= %x   \n",freq_float,System.fbv_c);
}

