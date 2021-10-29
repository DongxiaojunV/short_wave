#include "MT2000_protocol.h"
#include "my_protocol.h"
#include "bsp.h"
#include "main.h"
#include "bsp_uart_fifo.h"
#include "alarm.h"

#include <string.h>
#include <stdint.h>

extern TaskHandle_t  xHandleTask_Alarm;

GT2000_t  GT2000_Rx;
GT2000_t  GT2000_Tx;

uint8_t Emission_Tx[Emis_Size];		//���ͻ�����
uint8_t Emission_Rx[Emis_Size];		//���ջ�����


void GT2000_Init(void)
{
    uint8_t i=0;

    /*--------------------------GT2000_Tx------------------------------*/
    GT2000_Tx.Gt2000_mode = 0;		//��Ƶ
	  GT2000_Tx.method = 0;		//FM
    Power_Range(&GT2000_Tx,(uint8_t*)(0xAF));  //����ֵת��Ϊ����975W����ֵ��Frequency_power
	
	  GT2000_Tx.channel[0] = 0;	//�ŵ�01
    GT2000_Tx.channel[1] = 1;
    GT2000_Tx.channel[2] = 0;
    
    for(i=0; i<8; i++)
        GT2000_Tx.Gt2000_freq1[i] = 0;

    for(i=0; i<8; i++)
        GT2000_Tx.Gt2000_freq2[i] = 0;

    for(i=0; i<8; i++)
        GT2000_Tx.Gt2000_freq3[i] = 0;

    for(i=0; i<4; i++)
        GT2000_Tx.forward_power[i] = 0;

    for(i=0; i<4; i++)
        GT2000_Tx.reverse_power[i] = 0;

    /*--------------------------GT2000_Rx------------------------------*/
    GT2000_Rx.Gt2000_mode = 1;		//��Ƶ
		GT2000_Rx.method= 0;		//FM
		GT2000_Rx.Frequency_power[1] = 0x03;	
    GT2000_Rx.Frequency_power[0] = 0xCF;		//����ֵ7.5
    GT2000_Rx.sys_open = 0;		//ֹͣģʽ
		
		GT2000_Rx.channel[0] = 0;	//�ŵ�01
    GT2000_Rx.channel[1] = 1;
    GT2000_Rx.channel[2] = 0;

     for(i=0; i<10; i++)
        GT2000_Rx.Gt2000_freq1[i] = 0;

    for(i=0; i<10; i++)
        GT2000_Rx.Gt2000_freq2[i] = 0;

    for(i=0; i<10; i++)
        GT2000_Rx.Gt2000_freq3[i] = 0;

    for(i=0; i<5; i++)
        GT2000_Rx.forward_power[i] = 0;


    for(i=0; i<5; i++)
        GT2000_Rx.reverse_power[i] = 0;
    GT2000_Rx.reverse_power[3] = 0;
}

void GT2000_Open(void)
{
    int ret;
				Gt_Inquire_All();
				ret=GT2000_Wait_Ack(MT2000_CMD_INQUIRE);
    if( ret == MT2000_ACK_ALARM )	//����������
    {
        GT2000_Alarm_Stop();
        return;
    }
    else	if( ret == MT2000_ACK_OK )
    {
        GT2000_Buffer_Clean();
			  vTaskDelay(30);
				Gt2000_Open();   //����
			  vTaskDelay(1000);
				System.status=SYSTEM_OPENING;
    }
    else						
    {
        Alarm.no_respond = 0x01;
		  	Trans_printf(" Alarm.no_respond 3 ! ");
        Alarm.no_respond_count = 0;		//����ֻ��һ��

        System.open = 0x00;
        Power_onBack.results[0]=0xFD;
        Trans_printf("system open fail!\n");

        if( Monitor.hard_control == 1 )
        {
            if( get_history_alarm() == 0x01 )
            {
                System.status = SYSTEM_ALARM;
            }
        }
    }

}

void GT2000_Close(void)
{
    int ret;

		Gt_RF_Close();		//�ȹط���
		vTaskDelay(50);
		Gt2000_Close();   //�ػ�
		vTaskDelay(50);
		ret=MT2000_ACK_OK;		//Ĭ�ϳɹ�

    if( ret == MT2000_ACK_OK )
    {
        Power_offBack.results[0]= 0xFE;	//�ػ��ɹ�
        Power_onBack.results[0]	= 0xFD;	//����ʧ��#endif

        if( get_history_alarm() == 0x01 )
        {
            System.status = SYSTEM_ALARM;
        }
        else
        {
            System.status = SYSTEM_SHUTDOWN;
        }
    }
    else
    {
        Power_offBack.results[0]=0xFD;	//�ػ�ʧ��
//        Trans_printf("N02:system close fail!\n");

        Alarm.no_respond_locate = 7;
//        Trans_printf("GT2000_Close 7 no respond.\r\n");
    }
}

int GT2000_Emit(void)  //������
{
    uint8_t i = 0;
    uint8_t repeat_send_counter = 0;  //�ظ����ͼ�����
    int ret=0;

	  System.protect_adjust = 0x00;//1:���ڽ��б�������		0:����Ҫ���ڻ���ڽ���(��ɻ�ʧ��)		(����/פ���� > 80%������ֵ)

    if( System.sweep == 0x01 )  //����ɨƵ
		{
      System.status = SYSTEM_SCAN;   //ɨƵ״̬
		}

    GT2000_Buffer_Clean();  //������ڻ���
		
	  Trans_printf(" 1�ɴ˽������ù���ģʽ��Ƶ��GT2000_Tx.Gt2000_mode== %d \n" ,GT2000_Tx.Gt2000_mode);	
        for(i=0;i<3;i++)	
       {		
		    GT_Set_Value(GT2000_Tx);	 /* ���ù���ģʽ��Ƶ��,GT2000_Tx��Task_Upper_Computer.c�и�ֵ */
			  vTaskDelay(50);   //�ȴ�
				Gt_Inquire_All();		//��ѯһ��
				 vTaskDelay(50);   //�ȴ�
			  ret=GT2000_Wait_Ack(MT2000_CMD_CHANNEL);  
			  GT2000_Buffer_Clean();
					if( ret == MT2000_ACK_OK )
					{
						if((GT2000_Tx.Gt2000_freq1[0]==GT2000_Rx.Gt2000_freq1[1])||(GT2000_Tx.Gt2000_freq1[1]==GT2000_Rx.Gt2000_freq1[0]))
						{
							GT2000_Buffer_Clean();
							break;
						}
						else
						{
							Trans_printf(" �� %d �η����������ʧ�� \n",i);
						}
					}				 
       }
			

		Trans_printf("  �����깤��������ȴ�10s! \n");
		vTaskDelay(10000);   //�ȴ�10s

    for(repeat_send_counter=0; repeat_send_counter<3; repeat_send_counter++)
    {		
			Gt_Inquire_All();		//��ѯһ��
			ret=GT2000_Wait_Ack(MT2000_CMD_CHANNEL);  

			if( ret == MT2000_ACK_ALARM )
			{
 					Trans_printf("  ����GT2000_Alarm_Stop! \n");
				  GT2000_Alarm_Stop();				//returnΪֱ���˳����������
					return ret;
			}
			else	if( ret == MT2000_ACK_OK )
			{
					GT2000_Buffer_Clean();
					break;
			}
			vTaskDelay(50);
    }

    if( ret == MT2000_ACK_OK )
    {      
        freq_range_judge(GT2000_Tx.Gt2000_freq1);  //�˶�Ƶ��ֵ			
				if( System.stop == 0x01 )
				{
						System.stop = 0x00;
						System.cancel_sweeping = 0x00;

						System.open = 0x02;
						System.achieve_add_sub = 0;
						System.modify_power = 0;
						System.cancel_add_sub = 0;
						System_Status_Clean();

					  Trans_printf(" �������ж��Ƿ�����ʷ���� �� \n");
						if( get_history_alarm() == 0x01 )
						{
								System.status = SYSTEM_ALARM;
						}
						else
						{
								System.status = SYSTEM_STANDBY;
						}
						return 0;
				}
				else	if( (Alarm.emission==0x01) || (Alarm.no_respond==0x01) )
				{
									System.status = SYSTEM_ALARM;
									System.stop = 0x00;
									System.cancel_sweeping = 0x00;

									System.open = 0x02;
									System.achieve_add_sub = 0;
									System.modify_power = 0;
									System.cancel_add_sub = 0;
									System_Status_Clean();
					return 0;
				}
				
				if(System.fbv_c==System.Frequency_band_value)  //���鷢���Ƶ���Ƿ���Ƶ����
				{
					Gt_RF_Open();   //������
					vTaskDelay(100);
					Trans_printf(" Ƶ��һ�£������� \n ");
					if(System.status!=SYSTEM_SCAN)		//�������ɨƵ��ֱ�Ӹ���״̬
					{
						System.status=SYSTEM_EMISSING;
					}
				}
				else
				{
					Gt2000_Close(); //�ػ�
					Trans_printf(" Ƶ�β�һ�£��ػ�System.fbv_c== %d  System.Frequency_band_value== %d \n ",System.fbv_c,System.Frequency_band_value);
					return 0;
				}

					if( ret != MT2000_ACK_OK)	
					{
							System.stop = 0x00;

							System.open = 0x02;
							System.achieve_add_sub = 0;
							System.modify_power = 0;
							System.cancel_add_sub = 0;
							System_Status_Clean();

							Alarm.no_respond_locate = 10;
						Trans_printf(" Alarm.no_respond_locate = 10  \n");
							return 0;
					}

					//����ɹ�֮��System.emission=0x01; ��Щ��־λδ��λ������Task_Hardware_Monitor()���ٴ��ж�Ϊ��Ҫ���䣬���־���

					/* ����ɹ�֮�󣬲������ϸ������ݣ�������ȴ����ݸ���(���ʻ���µıȽ���) */
					for(i=0; i<50; i++)
					{
						Gt_Inquire_All();
						vTaskDelay(50);
						ret=GT2000_Wait_Ack(MT2000_CMD_INQUIRE);
						if( ret == MT2000_ACK_ALARM )
						{
								GT2000_Alarm_Stop();
								return ret;
						}
						if( System.Forward_Power >= 150.0 )
						{
								System.emission = 0x01;
								break;
						}
						else
						{
								System.emission = 0x00;		//�ȴ����صĹ��ʲ�Ϊ�㣬��������
						}

							vTaskDelay(200);
					}

					if( ret != MT2000_ACK_OK )
					{
							Alarm.no_respond_locate = 12;
						Trans_printf(" Alarm.no_respond_locate = 12  \n");
							return 0;
					}
					
					if( (System.Forward_Power <= 10.0) && (System.Electricity > 10.0) )	//�޹��ʱ���
					{
					
							if( Alarm.no_power == 0 )
							{
								if( System.time_update_flag == 0x01 )
								{
									Alarm.no_power_time_flag = 0x01;
									Time_Printf( (RTC_GetCounter()+g_stamp_distance), &set_time );
									Alarm_historyBack.no_power_time[0]=set_time.tm_year-2000;
									Alarm_historyBack.no_power_time[1]=set_time.tm_mon;
									Alarm_historyBack.no_power_time[2]=set_time.tm_mday;
									Alarm_historyBack.no_power_time[3]=set_time.tm_hour;
									Alarm_historyBack.no_power_time[4]=set_time.tm_min;
									Alarm_historyBack.no_power_time[5]=set_time.tm_sec;
								}

								Alarm.alarm_history = 1;
								Alarm.no_power = 1;
								Trans_printf(" ��������⵽�޹��ʱ���1�� \n");
							}
					}
		      //����ɹ�����鱨��
					swr_power_alarm_check();	//פ���ȼ��
					Trans_printf(" ���е������ʾ����ɹ�������פ���ȼ��  \n");

					if( Alarm.emission == 0x01 )	//������������������ֹͣ
					{
							GT2000_Alarm_Stop();
							return 0;
					}
              
    }
    else
    {
        Trans_printf("Emis//Channel fail\r\n");
        System.stop = 0x00;

        System.open = 0x02;
        System.achieve_add_sub = 0;
        System.modify_power = 0;
        System.cancel_add_sub = 0;
        System_Status_Clean();

        Trans_printf("����ʧ��\r\n");

        Alarm.no_respond_locate = 8;
        Trans_printf("BJ2000_Emit Channel 8 no respond.\r\n");
        return 0;
    }

return 0;
}

void GT2000_Stop(void)
{
    uint8_t repeat_send_counter = 0;
    int ret;
	  uint8_t i;  //����forѭ��

    System.stop = 0x01;
		System.protect_adjust = 0x00;

    //ֹͣ����,��ֹͣ��ʱ���ٷ���ָֹͣ�Ҳ������Ӧ
    //��ѯ
    for(repeat_send_counter=0; repeat_send_counter<NO_RESPOND_MAX; repeat_send_counter++)
    {
				Gt_Inquire_All();		//�����״̬��ѯ			
				ret=GT2000_Wait_Ack(MT2000_CMD_INQUIRE);
			  if( (ret == MT2000_ACK_OK) || (ret == MT2000_ACK_ALARM) )
            break;
    }
    if( ret == MT2000_ACK_OK )			//��ѯ�ɹ��ˣ��ż�������ģ�������Ǳ���״̬
    {
        if( System.emission != 0x01 )	//������Ƿ���״̬������ֹͣ
        {
            System.sweep = 0;
            System.stop = 0;
            System.achieve_add_sub = 0;
            System.modify_power = 0;
            System.cancel_add_sub = 0;
            System_Status_Clean();
            Trans_stopBack.Trans_state[0]=0xFC;	//�Ѿ���ֹͣ״̬
        }
        else
        {
				 for(i=0;i<10;i++)
					{
					Gt_RF_Close();	 //�ط���	
				  vTaskDelay(1500);  //����ָ����Ҫ���1.5s
					Gt_Inquire_All();		//�����״̬��ѯ
				  ret=GT2000_Wait_Ack(MT2000_CMD_INQUIRE);
						if(ret == MT2000_ACK_OK && GT2000_Rx.sys_emit_open==0) //��ѯ�Ƿ�ط���ɹ�
						{
							Trans_printf(" Gt_RF_Close OK�� ");
							break;  //������ǰforѭ��
						}
					}

					System.sweep = 0;
					System.stop = 0;
					System.achieve_add_sub = 0;
					System.modify_power = 0;
					System.cancel_add_sub = 0;
					System_Status_Clean();

					if( get_history_alarm() == 0x01 )
					{
							System.status = SYSTEM_ALARM;
				  Trans_printf(" GT2000_Stop_have_history_alarm �� \n");
					}
					else
					{
							System.status = SYSTEM_STANDBY;
					Trans_printf(" GT2000_Stop_SYSTEM_STANDBY �� \n");
					}
					
					System.emission=0;
					Trans_printf(" GT2000_Stop__System.emission=0 Stop_ok�� \n");
					Trans_stopBack.Trans_state[0]=0xFE;		//ֹͣ�ɹ�
        }
    }
    else     //����ѯ�����ɹ�������ֹͣʧ�ܣ����ڱ���״̬(���ﲻ�޸�״̬)
    {
        Trans_stopBack.Trans_state[0]=0x02;		//ֹͣʧ��
        Alarm.no_respond_locate = 13;
			Trans_printf(" GT2000_Stop_Stop_fail �� \n");
    }

    if( System.emission == 0x00 ) //��鸳ֵ���,�����汻��ֵ0
    {
		
		if( System.sweep == 0x01 )
		{
			if( (Alarm.emission==0x01) || (Alarm.no_respond==0x01) || (Alarm.no_power==0x01) )
			{
				System.status = SYSTEM_ALARM;
				
				System.sweep = 0;
        System.stop = 0;
			}
			else
			{
				System.status = SYSTEM_SCAN;		//ɨ��һ��Ƶ��
			}
		}
		else
		{
			System.status = SYSTEM_STANDBY;
		}
		
        System_Status_Clean();
        App_printf("stop successfully!\r\n");
		    Trans_printf(" GT2000_Stopֹͣ�ɹ��� \n");
    }
    else	//����ֹͣʧ�ܣ��������λ
    {

    }
}

void GT2000_Alarm_Stop(void)
{
	Trans_printf(" ����GT2000_Alarm_Stop,�ȹط��䣬5S���ٹػ�����λAlarm.emission=1 ");
				Gt_RF_Close();		//�ȹط���
				vTaskDelay(5000); //������
				Gt2000_Close();   //�ٹػ�,�޷�������
			vTaskDelay(50);

			System.status = SYSTEM_ALARM;
	
			Alarm.emission=1;		//����������	
			System.sweep = 0;
			System.stop = 0;
			System.achieve_add_sub = 0;
			System.modify_power = 0;
			System.cancel_add_sub = 0;
			System_Status_Clean();
			Trans_stopBack.Trans_state[0]=0xFE;		//ֹͣ�ɹ�
	
}

void GT2000_Buffer_Clean(void)  //���㴮��fifo����
{
    comClearRxFifo(COM4);
	  comClearRxFifo(COM1);
}

/*---------------------------------------------------����------------------------------------------------*/

/* ���� */
uint8_t BCC_Check_Sum(uint8_t *pdata, uint8_t len)
{
    uint8_t i = 0;
    uint8_t bcc_result = 0;

    bcc_result = *pdata;
    for(i=1; i<len; i++)
        bcc_result ^= *(pdata+i);

    return bcc_result;
}

/* ���� */
void Gt2000_Open(void)
{
    uint8_t buf[22] ={0x42,0x0A,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,\
		                  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x41,0x41};
    comSendBuf(COM4, buf, 22);								
}

/* �ػ� */
void Gt2000_Close(void)
{
    uint8_t buf[22] ={0x42,0x0A,0x02,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,\
		                  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x41,0x41};
    comSendBuf(COM4, buf, 22);
}

/*������*/
void Gt_RF_Open(void)
{
    uint8_t buf[22] ={0x42,0x0A,0x03,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,\
		                  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x41,0x41};
    comSendBuf(COM4, buf, 22);
}

/*�ط���*/
void Gt_RF_Close(void)
{
    uint8_t buf[22] ={0x42,0x0A,0x04,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,\
		                  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x41,0x41};
    comSendBuf(COM4, buf, 22);
}

/*���Ÿ�λ*/
void Gt2000_Reset(void)
{
    uint8_t buf[22] ={0x42,0x0A,0x05,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,\
		                  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x41,0x41};
    comSendBuf(COM4, buf, 22);
}

/*��ѯ����״̬*/
void Gt_Inquire_All(void)
{
    uint8_t buf[22] ={0x42,0x0A,0x14,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,\
		                  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x41,0x41};
    comSendBuf(COM4, buf, 22);
}

/*����Ƶ��1��2��3*/
void Gt_Set_Freq(GT2000_t Set_CMD)
{
	uint8_t i;
		if(Set_CMD.Gt2000_mode==1)		//����Ƶ��1
		{
			uint8_t buf[22];
			buf[0]=0x42;
			buf[1]=0x0A;
			buf[2]=0x06;
			buf[3]=0x00;
			buf[4]=Set_CMD.Gt2000_freq1[0];
			buf[5]=Set_CMD.Gt2000_freq1[1];
			buf[6]=0x00;
			buf[7]=0x00;
			memset(buf+8,0x00,12);
      buf[20]=0x41;buf[21]=0x41;
      comSendBuf(COM4, buf, 22);
			vTaskDelay(1500);
			Gt_Set_Power(Set_CMD);  //���ù���
      vTaskDelay(1000);			
			Trans_printf(" ģʽ1 buf[0-21]= ");
			for(i=0;i<22;i++)
		{
      Trans_printf(" %02X ",buf[i]);	
		}
      Trans_printf(" \n ");		
		}
		else if(Set_CMD.Gt2000_mode==2)		//������Ƶ��1��������Ƶ��2
		{
			uint8_t buf[22];
			buf[0]=0x42;
			buf[1]=0x0A;
			buf[2]=0x06;
			buf[3]=0x00;
			memcpy(buf+4,Set_CMD.Gt2000_freq1,4);
			memset(buf+8,0x00,12);
      buf[20]=0x41;buf[21]=0x41;
      comSendBuf(COM4, buf, 22);
			vTaskDelay(1500);  //�ȴ��������			
			GT2000_Buffer_Clean(); //������ڷ��Ͷ���
			memset(buf,0x00,22);  //����buf
			
			buf[0]=0x42;
			buf[1]=0x0A;
			buf[2]=0x07;
			buf[3]=0x00;
			memset(buf+4,0x00,4);
			memcpy(buf+8,Set_CMD.Gt2000_freq2,2);
			memset(buf+10,0x00,8);
      buf[20]=0x41;buf[21]=0x41;
      comSendBuf(COM4, buf, 22);
			vTaskDelay(1500);
			
      Trans_printf(" ģʽ2 buf[0-21]= ");
			for(i=0;i<22;i++)
		{
      Trans_printf(" %02X ",buf[i]);	
		}
      Trans_printf(" \n ");		
		}
		else if(Set_CMD.Gt2000_mode==3)		//������Ƶ��1��������Ƶ��2������Ƶ��3
		{
			uint8_t buf[22];
			buf[0]=0x42;
			buf[1]=0x0A;
			buf[2]=0x06;
			buf[3]=0x00;
			memcpy(buf+4,Set_CMD.Gt2000_freq1,4); 
			memset(buf+8,0x00,4);
			memset(buf+12,0x00,4);
			memset(buf+16,0x00,4);
      buf[20]=0x41;buf[21]=0x41;
      comSendBuf(COM4, buf, 22);
			vTaskDelay(1500);  //�ȴ��������	
			GT2000_Buffer_Clean();
			memset(buf,0x00,22);  //����buf
			
			buf[0]=0x42;          //����Ƶ��2
			buf[1]=0x0A;
			buf[2]=0x07;
			buf[3]=0x00;
			memset(buf+4,0x00,4);
			memcpy(buf+8,Set_CMD.Gt2000_freq2,2);
			memset(buf+10,0x00,8);
      buf[20]=0x41;buf[21]=0x41;
      comSendBuf(COM4, buf, 22);
			vTaskDelay(1500);  //�ȴ��������
			GT2000_Buffer_Clean();
			memset(buf,0x00,22);  //����buf
			
			buf[0]=0x42;          //����Ƶ��3
			buf[1]=0x0A;
			buf[2]=0x08;
			buf[3]=0x00;
			memset(buf+4,0x00,8);
			memcpy(buf+12,Set_CMD.Gt2000_freq3,2);
			memset(buf+14,0x00,4);
      buf[20]=0x41;buf[21]=0x41;
      comSendBuf(COM4, buf, 22);
			vTaskDelay(1500);	
			Trans_printf(" ģʽ3 buf[0-21]= ");
     for(i=0;i<22;i++)
		{
      Trans_printf(" %02X ",buf[i]);	
		}
      Trans_printf(" \n ");				
		
		}		
}

/* ���ù��� */
void Gt_Set_Power(GT2000_t Set_Power)
{	
			uint8_t buf1[22];
			buf1[0]=0x42;
			buf1[1]=0x0A;
			buf1[2]=0x09;
			buf1[3]=0x00;
			memset(buf1+4,0x00,12);
			memcpy(buf1+16,Set_Power.Frequency_power,2); //��Task_Upper_Computer.c�и�ֵ����λ��ǰ
      memset(buf1+18,0x00,2);
	    buf1[20]=0x41;buf1[21]=0x41;
      comSendBuf(COM4, buf1, 22);
	    Trans_printf(" ���ù����������·�Set_Power.Frequency_power= %x %x \n",buf1[16],buf1[17]);
}

/* ���ù���ģʽ1~4 */
void Gt_Set_Mode(uint8_t num)
{
		if(num==1)		//����Ϊ��Ƶ,����7B 53 31 7D  ���óɹ���
		{
			uint8_t buf[22];
			buf[0]=0x42;
			buf[1]=0x0A;
			buf[2]=0x0A;
			buf[3]=0x00;
			memset(buf+4,0x00,14);
			buf[18]=0x01;buf[19]=0x00;
	    buf[20]=0x41;buf[21]=0x41;
      comSendBuf(COM4, buf, 22);	
		}
		else if(num==2)		//����Ϊ˫Ƶ
		{
			uint8_t buf[22];
			buf[0]=0x42;
			buf[1]=0x0A;
			buf[2]=0x0A;
			buf[3]=0x00;
			memset(buf+4,0x00,14);
			buf[18]=0x02;buf[19]=0x00;
	    buf[20]=0x41;buf[21]=0x41;
      comSendBuf(COM4, buf, 22);
		}
		else if(num==3)		//����Ϊ��Ƶ
		{
			uint8_t buf[22];
			buf[0]=0x42;
			buf[1]=0x0A;
			buf[2]=0x0A;
			buf[3]=0x00;
			memset(buf+4,0x00,14);
			buf[18]=0x03;buf[19]=0x00;
	    buf[20]=0x41;buf[21]=0x41;
      comSendBuf(COM4, buf, 22);
		}
		else if(num==4)		//����ΪAM
		{
			uint8_t buf[22];
			buf[0]=0x42;
			buf[1]=0x0A;
			buf[2]=0x0A;
			buf[3]=0x00;
			memset(buf+4,0x00,14);
			buf[18]=0x04;buf[19]=0x00;
	    buf[20]=0x41;buf[21]=0x41;
      comSendBuf(COM4, buf, 22);
		}
  Trans_printf(" ���ù���ģʽ%d�������·� \n",num);		
}

/*�����乤����������*/
void GT_Set_Value(GT2000_t Send_CMD)  
{
		
		if(Send_CMD.Gt2000_mode==1)		//��Ƶ
		{
			Gt_Set_Mode(1);   //���ù���ģʽΪ1��Ƶ
			vTaskDelay(1000);  //�ȴ�������ϼ����ʱ������1000
			Gt_Set_Power(Send_CMD);  //���ù���	
			vTaskDelay(1000);
      Gt_Set_Freq(Send_CMD);   //����Ƶ��
			vTaskDelay(1000);			
			Trans_printf(" ��Ƶ����ģʽ�������·���ϣ�\n");
		}
		
		else if(Send_CMD.Gt2000_mode==2) //˫Ƶ
		{
			Gt_Set_Mode(2);   //���ù���ģʽΪ2˫Ƶ
			vTaskDelay(1000);
			Gt_Set_Power(Send_CMD);  //���ù���	
			vTaskDelay(1000);
      Gt_Set_Freq(Send_CMD);   //����Ƶ��
			Trans_printf(" ˫Ƶ����ģʽ�������·���ϣ�\n");
		}
		
		else if(Send_CMD.Gt2000_mode==3) //��Ƶ
		{
			Gt_Set_Mode(3);
			vTaskDelay(1500);
			Gt_Set_Power(Send_CMD);  //���ù���
			vTaskDelay(1500);
      Gt_Set_Freq(Send_CMD);   //����Ƶ��
			Trans_printf(" ��Ƶ����ģʽ�������·���ϣ�\n");
		}
		else if(Send_CMD.Gt2000_mode==4)  //AM
		{
			 Gt_Set_Mode(4);
		}
		else 
		{
			 Trans_printf(" error���޴˹���ģʽ��Send_CMD.Gt2000_mode==  %d \n",Send_CMD.Gt2000_mode);
		}
		Trans_printf(" �����乤�����������������·���ϣ�Send_CMD.Gt2000_mode==  %d \n",Send_CMD.Gt2000_mode);
}


uint8_t check_sum(uint8_t *buf,uint16_t len)		//��У���
{
	uint16_t sum=0;
		for (uint16_t i=0;i<len;i++)
	{
     sum+=buf[i];//��ÿ�������
	}
     if(sum>0xff)
     {
        sum=~sum;
                  
        sum+=1;
 
     }
		return (sum&0xff); 
}


int GT2000_Wait_Ack(uint8_t cmd)
{
    
    uint16_t i = 0;
    uint8_t temp = 0;
    uint8_t read;
    uint16_t num = 0;
    int count = 0;
    int ret;
//		uint8_t Power_1[3],Power_2[3],Power_3[3];
    switch( cmd )
    {
		case MT2000_WAIT_ACK_OK:		num = 20;		break;	//Ӧ��� 1s
		case MT2000_CMD_INQUIRE:		num = 20;		break;	//��ѯ�� 1s
		case MT2000_CMD_CHANNEL:		num = 20;		break;	//���õ� 1s
		case MT2000_CMD_TUNING:			num = 600;		break;	//��г�� 30s
		case MT2000_CMD_EMISSION:		num = 20;		break;	//����� 1s
		case MT2000_CMD_POWER_ADD:		num = 20;		break;	//���ӹ��ʵ� 1s
		case MT2000_CMD_POWER_SUB:		num = 20;		break;	//��С���ʵ� 1s
		case MT2000_CMD_STOP:			num = 20;		break;	//ֹͣ�� 3s
		case MT2000_CMD_POWER_ON:		num = 300;		break;	//������ 15s
		case MT2000_CMD_POWER_OFF:		num = 60;		break;	//�ػ��� 3s
		default:						num = 100;		break;	//����� 5s
    }

    for(i=0; i<num; i++)
    {
        while( comGetChar(COM4,&read) )  //��ͨ����ֻ��һ�������շ�����
        {                                //����4��232������2�ɼ���ʪ������
            if(count<Emis_Size)
            {
                Emission_Rx[count++]=read;
            }
            vTaskDelay(1);
        }
				
			  if( (count>2) && (Emission_Rx[0]==0x42)&&(Emission_Rx[158]==0x41)&&(Emission_Rx[159]==0x00))  //ֻ������ȷ����
        {
//          /*---------�������ݲ���----------*/
//					if (count!=0)  //���������
//					{
//						Trans_printf("\r\n=====================================\r\n");
//						Trans_printf(	"��ȡ��count: %d ���ֽ� = ",count);
//						for(i = 0; i<(count); i++)
//						{
//								Trans_printf("%02X ",Emission_Rx[i]);
//						}
//						Trans_printf("\r\n=====================================\r\n");
//					}
//			  	/*---------�������ݲ���----------*/	
					
					ret = GT2000_Cmd_Analyze(Emission_Rx, cmd);		//���������������ݲ����ؽ��,���б������ٴ��ж�
					
   if (GT2000_Rx.sys_open==0)		//���ݲ�ѯ���Ļ���״̬��������
    {
				System.achieve_add_sub = 0;
				System.modify_power = 0;
				System.cancel_add_sub = 0;
				System_Status_Clean();
				System.Voltage = 0.0;
				System.Electricity = 0.0;
				System.sweep = 0;
				System.open = 0;
				System.close = 0;				//����

				if( get_history_alarm() == 0x01 )   //����ʷ������Ϣ
				{
						System.status = SYSTEM_ALARM;
					  Trans_printf(" ����ʷ������Ϣ�� \n ");
				}
				else
				{
					if(System.status!=SYSTEM_OPENING&&System.status!=SYSTEM_ALARM)		//�������ڿ��������Ǳ���״̬
					{
						if(GT2000_Rx.sys_open==0)
						{
						System.status = SYSTEM_SHUTDOWN;   //�ػ�״̬
						System.Open_Close=0;		//ֻ�����ڲ�ѯ����//0��������ػ���1������
						Trans_printf("�ػ�״̬1�� \n");
						}
					}
				}
       if(GT2000_Rx.sys_open==0)
			 {
				 System.Open_Close=0;		//ֻ�����ڲ�ѯ����//0��������ػ���1������
			 }
       else  
				 System.Open_Close=1; 				 
    }
		
		    /*ϵͳ���ڿ�������δ�����䣬������Ҫ���������������ȥ��*/
		if((GT2000_Rx.sys_open==1)&&(GT2000_Rx.sys_emit_open==0))	//��������δ�����䣬����״̬
		{
			System.Voltage=43;
			if(System.status!=SYSTEM_SCAN&&System.status!=SYSTEM_ALARM&&System.Power_Adjustment!=1)		//��ɨƵ״̬������״̬����������ʱ
			{
				System.status=SYSTEM_STANDBY;  //����״̬
			}
			Trans_printf(" ����״̬1�� \n");
			System.Open_Close=1; //����״̬��־λ
		 
	  }
		
		if((GT2000_Rx.sys_open==1)&&(GT2000_Rx.sys_emit_open==1))  //���ݲ�ѯ����ϵͳ״̬�������������䣬˵�����ڷ���
		{
			if(System.status!=SYSTEM_SCAN&&System.status!=SYSTEM_ALARM)  //������ɨƵ�򱨾�״̬��
			{
				System.status=SYSTEM_EMISSING;	//����״̬	
				System.Voltage=43;   
			}
			Trans_printf("����״̬1�� \n");
			System.Open_Close=1;
		}
	if ((ret==MT2000_ACK_ALARM)&&(GT2000_Rx.sys_open==1)&&(GT2000_Rx.system_status1!=0))		/*��ϸ�����ݲ������ط�Χ����ֻ��ؿ���ʱ����ģ��ı���*/
		{  
				   vTaskDelay(1000);  //��ʱ1s�ٴ��ж�
					 Gt_Inquire_All();		//�ٴη����״̬��ѯ
					ret=GT2000_Wait_Ack(MT2000_CMD_INQUIRE);
			
				if((GT2000_Rx.sys_open==1)&&(GT2000_Rx.system_status1!=0))  //�����й���
				{
					if( System.time_update_flag == 0x01 )  //����ʱ��
					{
							Alarm.emission_time_flag = 0x01;
							Time_Printf( (RTC_GetCounter()+g_stamp_distance), &set_time );
							Alarm_historyBack.emission_time[0]=set_time.tm_year-2000;
							Alarm_historyBack.emission_time[1]=set_time.tm_mon;
							Alarm_historyBack.emission_time[2]=set_time.tm_mday;
							Alarm_historyBack.emission_time[3]=set_time.tm_hour;
							Alarm_historyBack.emission_time[4]=set_time.tm_min;
							Alarm_historyBack.emission_time[5]=set_time.tm_sec;
					}

					System.status = SYSTEM_ALARM;    //����״̬
					Alarm.alarm_history = 1;
					Alarm.emission = 0x01;  //���������������屨������ϸ������Ϣ����־��鿴��
					Trans_printf("emission alarm alarm alarm! GT2000_Rx.system_status1== %02X \r\n",GT2000_Rx.system_status1);
					ret=MT2000_ACK_ALARM;					
				}
					else  //�ٴ��ж�ʱ�޹���
				{	
           if(System.Launch_Switch_state==1)
						{
							GT2000_Rx.sys_emit_open = 1;	//����/ֹͣ״̬
						}
				    else
						{
							GT2000_Rx.sys_emit_open = 0;	//����/ֹͣ״̬
						}
				  System.Electricity=GT2000_Rx.Working_current; //��������  //16~19
					System.Forward_Power=(float)((GT2000_Rx.forward_power[0]<<8)|(GT2000_Rx.forward_power[1])); //��������书��   //6~7
		      System.Reverse_Power=(float)((GT2000_Rx.reverse_power[0]<<8)|(GT2000_Rx.reverse_power[1])); //��������书��   //8~9 
						
					
					System_Status_Update(); //ϵͳ״̬����
				  Alarm.no_respond_count = 0;  //�������					
					ret=MT2000_ACK_OK;
				}
    }
		else
		{
			 if(System.Launch_Switch_state==1)
						{
							GT2000_Rx.sys_emit_open = 1;	//����/ֹͣ״̬
						}
				    else
						{
							GT2000_Rx.sys_emit_open = 0;	//����/ֹͣ״̬
						}
				  System.Electricity=GT2000_Rx.Working_current; //��������  //16~19
					System.Forward_Power=(float)((GT2000_Rx.forward_power[0]<<8)|(GT2000_Rx.forward_power[1])); //��������书��   //6~7
		      System.Reverse_Power=(float)((GT2000_Rx.reverse_power[0]<<8)|(GT2000_Rx.reverse_power[1])); //��������书��   //8~9 
						
					
					System_Status_Update(); //ϵͳ״̬����
				  Alarm.no_respond_count = 0;  //�������					
					ret=MT2000_ACK_OK;
		}
					
					
#if	NO_RESPOND_RECOVER
            if( ret == MT2000_ACK_OK )
            {
                if( Alarm.no_respond == 0x01 )
                {
                    System.stop = 0;
                    System.cancel_sweeping = 0;
                }

                Alarm.no_respond_count = 0;
                Alarm.no_respond = 0;
                if( get_history_alarm() == 0x01 )
                {
                    if( (System.status==SYSTEM_EMISSING) || (System.status==SYSTEM_SCAN) )	//����/ɨƵ״̬��ֻ�����ر���(�������������豸����Ӧ�������޹��ʱ���)���л�Ϊ����״̬
                    {
                        if( (Alarm.emission==0x01) || (Alarm.no_respond==0x01) || (Alarm.no_power==0x01) || (Alarm.swr_alarm==0x01))
                        {
                            System.status = SYSTEM_ALARM;
                        }
                    }
                    else
                    {
                        System.status = SYSTEM_ALARM;
                    }
                }
                else
                {
                    clean_all_alarm_flags();

                    if( System.already_init == 0x00 )
                    {
                        System.status = SYSTEM_UNINITIALIZE;  //δ��ʼ��
                    }
                }
//								Trans_printf("�������ӦOK  \n");
            }
#endif

            return ret;
        }
        count=0;   //����
        vTaskDelay(50);
    }

		if(System.status!=SYSTEM_UNINITIALIZE) //ֻҪ����δ��ʼ��
		{
			if( Alarm.no_respond == 0x00 )
			{
					Alarm.no_respond_count++;
					if( Alarm.no_respond_count >= NO_RESPOND_MAX )  //��ⷢ����Ƿ�ʧ������⴮��4�Ƿ�������
					{
							Alarm.no_respond_count = 0;

							if( System.time_update_flag == 0x01 )
							{
									Alarm.no_respond_time_flag = 0x01;
									Time_Printf( (RTC_GetCounter()+g_stamp_distance), &set_time );
									Alarm_historyBack.no_response_time[0]=set_time.tm_year-2000;
									Alarm_historyBack.no_response_time[1]=set_time.tm_mon;
									Alarm_historyBack.no_response_time[2]=set_time.tm_mday;
									Alarm_historyBack.no_response_time[3]=set_time.tm_hour;
									Alarm_historyBack.no_response_time[4]=set_time.tm_min;
									Alarm_historyBack.no_response_time[5]=set_time.tm_sec;
							}

							System.emission = 0x00;		//�豸����Ӧʱ����������־λ
							Alarm.no_respond = 0x01;
							Trans_printf(" Alarm.no_respond 1 ! ");
							Alarm.alarm_history = 0x01;  //��λ����Ӧ������˵��δ�յ�����ݵ���ȷ���ݡ�

							System.achieve_add_sub = 0x00;
							System.modify_power = 0x00;
							System.cancel_add_sub = 0x00;

							temp = System.emission;
							System.Voltage = 0.0;
							System.Electricity = 0.0;
							System_Status_Clean();		//���壬�ȷ��ظ�PC��ǰ״̬�����Լ�ֹͣ3�Σ�3��ʧ�������	�������������ѹ��������	--20181031 Luonus
							System.emission = temp;		//Ҫֹͣ
					}
				}
    }

    return MT2000_NO_ACK;	//��Ӧ��	

}
	
int GT2000_Cmd_Analyze(uint8_t *msg, uint8_t cmd)
{
	uint16_t Electricity_1[4],Electricity_2[4],Electricity_3[4],Electricity_4[4];		//�洢����һ~�ĵĵ�������
		float Average_E[4];   //���ڴ�ŵ���ƽ��ֵ
		float Electricity_Add_1=0,Electricity_Add_2=0,Electricity_Add_3=0,Electricity_Add_4=0;
	
		/*���յ�������*/
  	Electricity_1[0]=msg[41];   //��̫��ת232���䣬��λ��ǰ
		Electricity_1[1]=msg[42];
		Electricity_1[2]=msg[43];
		Electricity_1[3]=msg[44];
		Electricity_2[0]=msg[49];
		Electricity_2[1]=msg[50];
		Electricity_2[2]=msg[51];
		Electricity_2[3]=msg[52];
	  Electricity_3[0]=msg[57];
		Electricity_3[1]=msg[58];
		Electricity_3[2]=msg[59];
		Electricity_3[3]=msg[60];
		Electricity_4[0]=msg[65];
		Electricity_4[1]=msg[66];
		Electricity_4[2]=msg[67];
		Electricity_4[3]=msg[68];    //��̫��ת232���䣬��λ��ǰ
		
//		Trans_printf("Electricity_����== %d %d %d %d ",Electricity_1[0],\
//		Electricity_1[1],Electricity_1[2],Electricity_1[3]);  //��ӡ2KW������Ϣ
		/*���������ݷ���ϵͳ��Ϣ�ṹ����*/
		for(uint8_t i=0;i<4;i++)
		{ 
			//����֮ǰ��ϵͳ��Ϣ�ṹ�壬������ʱδ�ġ�
			System.Bj_Electricity_1[i]=Electricity_1[i]*10;  //������Ϣ����10��
			System.Bj_Electricity_2[i]=Electricity_2[i]*10;
			System.Bj_Electricity_3[i]=Electricity_3[i]*10;
			System.Bj_Electricity_4[i]=Electricity_4[i]*10;
		}
		
		
		/*ȡ�����������Щ���������ڼ�����������*/	//bit0Ϊ��λ
		Alarm_historyBack.Alarm_Abnormal_Output[0]=((msg[20]&0x01));             //bit0����쳣 0����  1����
		Alarm_historyBack.Alarm_Reverse[0]=((msg[20]&0x02)>>1);                     //bit1�������
		Alarm_historyBack.Alarm_Module_Electricity[0]=((msg[20]&0x08)>>3);          //bit3ģ��������� 
    Alarm_historyBack.Alarm_Absorber_State_1[0]=((msg[20]&0x40)>>6);          //bit6�˲�������
    Alarm_historyBack.Alarm_Switch_Power[0]=((msg[20]&0x80)>>7);                 //bit7���ص�Դ����	
		Trans_printf("Alarm_status1_msg[20]== %x ",msg[20]);  //��ӡ2KW������Ϣ
		
		
		
    GT2000_Rx.system_status1=(msg[20]&0xff);      //�жϹ��������0Ϊ������1Ϊ����
//		Trans_printf("GT2000_Rx.system_status1= %x \n",GT2000_Rx.system_status1);
		
		/*����ֵ*/  //��̫��ͨѶ��λ��ǰ
  	System.Attenuation=msg[14];     //˥��ֵ  //0��3 
    GT2000_Rx.swr[0]=msg[13];       //פ����  //10~13
		GT2000_Rx.swr[1]=msg[12];
		GT2000_Rx.swr[2]=msg[11];
		GT2000_Rx.swr[3]=msg[10];
		GT2000_Rx.sys_open=((msg[21]&0x01));            //bit0��������ػ�״̬ 0�ػ�  1����
		GT2000_Rx.sys_emit_open=((msg[21]&0x02)>>1);    //bit1���������״̬  0�ط���  1������

		if(((msg[21]&0x04)>>2)==0)
		{
			System.Control_Model=1;   //bit2���������ģʽ  1ң��   0����  System.Control_Model  0�Զ�/1�ֶ�
			Trans_printf("msg[21]Bit2== %x ",((msg[21]&0x04)>>2));
		}
		else if(((msg[21]&0x04)>>2)==1)
		{
			System.Control_Model=0;   //bit2���������ģʽ  1ң��   0����
//			Trans_printf("msg[21]== %x ",((msg[21]&0x04)>>2));
		}
		
		GT2000_Rx.Gt2000_mode=msg[30];   //1��Ƶ��2˫Ƶ��3��Ƶ��4AM 
		GT2000_Rx.Gt2000_freq1[0]=msg[23];  //��ǰƵ��1  //22~23
		GT2000_Rx.Gt2000_freq1[1]=msg[22];  //��ǰƵ��1  //22~23
		memset(GT2000_Rx.Gt2000_freq1+2,0x00,2);
		GT2000_Rx.Gt2000_freq2[0]=msg[25];  //��ǰƵ��2  //24~25
		GT2000_Rx.Gt2000_freq2[1]=msg[24];  //��ǰƵ��2  //24~25
		memset(GT2000_Rx.Gt2000_freq2+2,0x00,2);
		GT2000_Rx.Gt2000_freq3[0]=msg[27];  //��ǰƵ��3  //26~27
		GT2000_Rx.Gt2000_freq3[1]=msg[26];  //��ǰƵ��3  //26~27
		memset(GT2000_Rx.Gt2000_freq3+2,0x00,2);
		Trans_printf(" GT2000_Rx.Gt2000_freq1[0]== %x ",GT2000_Rx.Gt2000_freq1[0],GT2000_Rx.Gt2000_freq1[1]);
		
		/* ȡ�������ǰ����ֵ */
		GT2000_Rx.Frequency_power[0]=msg[29];
		GT2000_Rx.Frequency_power[1]=msg[28];
		GT2000_Rx.Now_all_power=((msg[29]<<8)|msg[28]);
		/*ȡ����ģʽ*/
		System.INQUIRE_Mode=msg[30];   //1��Ƶ��2˫Ƶ��3��Ƶ��4AM 
				  
		GT2000_Rx.Working_current=(float)(((msg[19]<<24)+(msg[18]<<16)+(msg[17]<<8)+(msg[16]))/4.0); //��������  //16~19
		GT2000_Rx.forward_power[0]=msg[7];GT2000_Rx.forward_power[1]=msg[6];  //��������书��   //6~7
		GT2000_Rx.reverse_power[0]=msg[9];GT2000_Rx.forward_power[1]=msg[8];  //��������书��   //8~9 
		
		  if(System.Launch_Switch_state==1)
				{
					GT2000_Rx.sys_emit_open = 1;	//����/ֹͣ״̬
				}
				else
				{
					GT2000_Rx.sys_emit_open = 0;	//����/ֹͣ״̬
				}
		
		System.Amplifier_Temperature[0]=(((((msg[48])+(msg[47]))/8.0)-8))*100;//ģ��1�¶�  //45~48
		System.Amplifier_Temperature[1]=(((((msg[56])+(msg[55]))/8.0)-8))*100;//ģ��2�¶�  //56~53
		System.Amplifier_Temperature[2]=(((((msg[64])+(msg[63]))/8.0)-8))*100;//ģ��3�¶�  //64~61
		System.Amplifier_Temperature[3]=(((((msg[72])+(msg[71]))/8.0)-8))*100;//ģ��4�¶�  //72~69
    
//		Trans_printf(" ģ���¶ȣ�%x %x %x %x \n",System.Amplifier_Temperature[0],System.Amplifier_Temperature[0],\
//		System.Amplifier_Temperature[1],System.Amplifier_Temperature[2],System.Amplifier_Temperature[3]);
		
		System.Frequency_band_value=msg[31];   //Ƶ��ֵ
//		Trans_printf("band_value= %d ",System.Frequency_band_value);
		
		System.Launch_Switch_state=((msg[21]&0x02)>>1);    //bit1��������صķ���״̬ 0�ط���  1������
		
		for(uint8_t i=0;i<4;i++)
		{
			Electricity_Add_1 += Electricity_1[i];  //��������Ϊƽ��ֵ
			Electricity_Add_2 += Electricity_2[i];
			Electricity_Add_3 += Electricity_3[i];
			Electricity_Add_4 += Electricity_4[i];
		}
		
		Average_E[0]=Electricity_Add_1/4.0;  //����1����
		Average_E[1]=Electricity_Add_2/4.0;  //����2����
		Average_E[2]=Electricity_Add_3/4.0;  //����3����
		Average_E[3]=Electricity_Add_4/4.0;  //����4����
//		Trans_printf(" ������%f %f %f %f \n",Average_E[0],Average_E[1],Average_E[2],Average_E[3]);    
		
			if ((GT2000_Rx.sys_open==1)&&(GT2000_Rx.system_status1!=0))		/*��ϸ�����ݲ������ط�Χ����ֻ��ؿ���ʱ����ģ��ı���*/
			  {  
				   //���ﲻ������ֹ����
					return MT2000_ACK_ALARM;
        }
			else   //���޹��������˽ṹ
			{					
					return MT2000_ACK_OK;
		  }
}


/* ���� */
uint8_t Bj_BCC_Check_Sum(uint8_t *pdata, uint8_t len)
{
    uint8_t i = 0;
    uint8_t bcc_result = 0;

    bcc_result = pdata[1];
    for(i=1; i<len-1; i++)
        bcc_result ^= pdata[i+1];

    return bcc_result;
}

/*PC�·�������ת��Ϊ����ֵ [100~250],��Ӧ��0~1950W��,1*13*/
void Power_Range(GT2000_t *Value,uint8_t *Data)
{
	uint8_t i;
	for(i=0;i<3;i++)
	{
		if(Data[i]<200)
		{
			if(i==0)
			{   //�·����������̫��ͨѶ����λ��ǰ
				Value->Frequency_power[1]=((((Data[i]-100)*13)&0xff00)>>8);   //ȡ���ʵĸ�λ�ֽ�
				Value->Frequency_power[0]=(((Data[i]-100)*13)&0xff);   //ȡ���ʵĵ�λ�ֽ�
				Value->Now_all_power=((Value->Frequency_power[1]<<8)|Value->Frequency_power[0]);  //ʮ�������ܹ���
			}
		}
		
		else if(Data[i]>=200&&Data[i]<250)
		{
			if(i==0)
			{    //�·����������̫��ͨѶ����λ��ǰ
				Value->Frequency_power[1]=((((Data[i]-100)*13)&0xff00)>>8);   //ȡ���ʵĸ�λ�ֽ�
				Value->Frequency_power[0]=(((Data[i]-100)*13)&0xff);   //ȡ���ʵĵ�λ�ֽ�
				Value->Now_all_power=((Value->Frequency_power[1]<<8)|Value->Frequency_power[0]);  //ʮ�������ܹ���
			}
		}
		
		else		//��������
		{
				if(i==0)
			{
				Value->Frequency_power[0]=0;   //
				Value->Frequency_power[1]=0;   //
			}
		}
	}
}

/* ����ת��Ϊϵͳ����ֵ */
void Range_Power(GT2000_t *Value)
{
	System.power[0]=(Value->Now_all_power/13); 
	/* GT2000�»���ֻ��ʾһ�����ʡ� */	
}

//����΢�� +65
void power_add(uint8_t *data)
{
	uint16_t All_power;
	All_power=((data[1]<<8)|data[0]);
	All_power=All_power+0x41;
	data[1]=((All_power&0xff00)>>8);
	data[0]=(All_power&0xff);
}


//����΢�� -65
void power_sub(uint8_t *data)
{
	uint16_t All_power;
	All_power=((data[0]<<8)|data[1]);
	All_power=All_power-0x41;
	data[0]=((All_power&0xff00)>>8);
	data[1]=(All_power&0xff);
}


