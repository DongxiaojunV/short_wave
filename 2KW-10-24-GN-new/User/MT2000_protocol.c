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

uint8_t Emission_Tx[Emis_Size];		//发送缓冲区
uint8_t Emission_Rx[Emis_Size];		//接收缓冲区


void GT2000_Init(void)
{
    uint8_t i=0;

    /*--------------------------GT2000_Tx------------------------------*/
    GT2000_Tx.Gt2000_mode = 0;		//固频
	  GT2000_Tx.method = 0;		//FM
    Power_Range(&GT2000_Tx,(uint8_t*)(0xAF));  //幅度值转化为功率975W，赋值给Frequency_power
	
	  GT2000_Tx.channel[0] = 0;	//信道01
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
    GT2000_Rx.Gt2000_mode = 1;		//固频
		GT2000_Rx.method= 0;		//FM
		GT2000_Rx.Frequency_power[1] = 0x03;	
    GT2000_Rx.Frequency_power[0] = 0xCF;		//幅度值7.5
    GT2000_Rx.sys_open = 0;		//停止模式
		
		GT2000_Rx.channel[0] = 0;	//信道01
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
    if( ret == MT2000_ACK_ALARM )	//激励器报警
    {
        GT2000_Alarm_Stop();
        return;
    }
    else	if( ret == MT2000_ACK_OK )
    {
        GT2000_Buffer_Clean();
			  vTaskDelay(30);
				Gt2000_Open();   //开机
			  vTaskDelay(1000);
				System.status=SYSTEM_OPENING;
    }
    else						
    {
        Alarm.no_respond = 0x01;
		  	Trans_printf(" Alarm.no_respond 3 ! ");
        Alarm.no_respond_count = 0;		//开机只开一次

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

		Gt_RF_Close();		//先关发射
		vTaskDelay(50);
		Gt2000_Close();   //关机
		vTaskDelay(50);
		ret=MT2000_ACK_OK;		//默认成功

    if( ret == MT2000_ACK_OK )
    {
        Power_offBack.results[0]= 0xFE;	//关机成功
        Power_onBack.results[0]	= 0xFD;	//开机失败#endif

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
        Power_offBack.results[0]=0xFD;	//关机失败
//        Trans_printf("N02:system close fail!\n");

        Alarm.no_respond_locate = 7;
//        Trans_printf("GT2000_Close 7 no respond.\r\n");
    }
}

int GT2000_Emit(void)  //开发射
{
    uint8_t i = 0;
    uint8_t repeat_send_counter = 0;  //重复发送计数器
    int ret=0;

	  System.protect_adjust = 0x00;//1:正在进行保护调节		0:不需要调节或调节结束(完成或失败)		(电流/驻波比 > 80%报警阈值)

    if( System.sweep == 0x01 )  //正在扫频
		{
      System.status = SYSTEM_SCAN;   //扫频状态
		}

    GT2000_Buffer_Clean();  //清除串口缓存
		
	  Trans_printf(" 1由此进入设置工作模式和频率GT2000_Tx.Gt2000_mode== %d \n" ,GT2000_Tx.Gt2000_mode);	
        for(i=0;i<3;i++)	
       {		
		    GT_Set_Value(GT2000_Tx);	 /* 设置工作模式和频率,GT2000_Tx在Task_Upper_Computer.c中赋值 */
			  vTaskDelay(50);   //等待
				Gt_Inquire_All();		//查询一次
				 vTaskDelay(50);   //等待
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
							Trans_printf(" 第 %d 次发射参数设置失败 \n",i);
						}
					}				 
       }
			

		Trans_printf("  设置完工作参数后等待10s! \n");
		vTaskDelay(10000);   //等待10s

    for(repeat_send_counter=0; repeat_send_counter<3; repeat_send_counter++)
    {		
			Gt_Inquire_All();		//查询一次
			ret=GT2000_Wait_Ack(MT2000_CMD_CHANNEL);  

			if( ret == MT2000_ACK_ALARM )
			{
 					Trans_printf("  触发GT2000_Alarm_Stop! \n");
				  GT2000_Alarm_Stop();				//return为直接退出这个函数。
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
        freq_range_judge(GT2000_Tx.Gt2000_freq1);  //核对频段值			
				if( System.stop == 0x01 )
				{
						System.stop = 0x00;
						System.cancel_sweeping = 0x00;

						System.open = 0x02;
						System.achieve_add_sub = 0;
						System.modify_power = 0;
						System.cancel_add_sub = 0;
						System_Status_Clean();

					  Trans_printf(" 开发射判断是否有历史报警 ！ \n");
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
				
				if(System.fbv_c==System.Frequency_band_value)  //检验发射的频段是否在频段内
				{
					Gt_RF_Open();   //开发射
					vTaskDelay(100);
					Trans_printf(" 频段一致！开发射 \n ");
					if(System.status!=SYSTEM_SCAN)		//如果不是扫频，直接更新状态
					{
						System.status=SYSTEM_EMISSING;
					}
				}
				else
				{
					Gt2000_Close(); //关机
					Trans_printf(" 频段不一致！关机System.fbv_c== %d  System.Frequency_band_value== %d \n ",System.fbv_c,System.Frequency_band_value);
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

					//发射成功之后，System.emission=0x01; 有些标志位未置位，导致Task_Hardware_Monitor()会再次判断为需要发射，出现警告

					/* 发射成功之后，不会马上更新数据，在这里等待数据更新(功率会更新的比较慢) */
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
								System.emission = 0x00;		//等待返回的功率不为零，否则清零
						}

							vTaskDelay(200);
					}

					if( ret != MT2000_ACK_OK )
					{
							Alarm.no_respond_locate = 12;
						Trans_printf(" Alarm.no_respond_locate = 12  \n");
							return 0;
					}
					
					if( (System.Forward_Power <= 10.0) && (System.Electricity > 10.0) )	//无功率报警
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
								Trans_printf(" 开发射后检测到无功率报警1！ \n");
							}
					}
		      //发射成功，检查报警
					swr_power_alarm_check();	//驻波比检测
					Trans_printf(" 运行到这里表示发射成功，进行驻波比检测  \n");

					if( Alarm.emission == 0x01 )	//激励器自身报警，立即停止
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

        Trans_printf("发射失败\r\n");

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
	  uint8_t i;  //用于for循环

    System.stop = 0x01;
		System.protect_adjust = 0x00;

    //停止发射,在停止的时候，再发送停止指令，也可以响应
    //查询
    for(repeat_send_counter=0; repeat_send_counter<NO_RESPOND_MAX; repeat_send_counter++)
    {
				Gt_Inquire_All();		//发射机状态查询			
				ret=GT2000_Wait_Ack(MT2000_CMD_INQUIRE);
			  if( (ret == MT2000_ACK_OK) || (ret == MT2000_ACK_ALARM) )
            break;
    }
    if( ret == MT2000_ACK_OK )			//查询成功了，才继续下面的，否则就是报警状态
    {
        if( System.emission != 0x01 )	//如果不是发射状态，不用停止
        {
            System.sweep = 0;
            System.stop = 0;
            System.achieve_add_sub = 0;
            System.modify_power = 0;
            System.cancel_add_sub = 0;
            System_Status_Clean();
            Trans_stopBack.Trans_state[0]=0xFC;	//已经是停止状态
        }
        else
        {
				 for(i=0;i<10;i++)
					{
					Gt_RF_Close();	 //关发射	
				  vTaskDelay(1500);  //各种指令需要间隔1.5s
					Gt_Inquire_All();		//发射机状态查询
				  ret=GT2000_Wait_Ack(MT2000_CMD_INQUIRE);
						if(ret == MT2000_ACK_OK && GT2000_Rx.sys_emit_open==0) //查询是否关发射成功
						{
							Trans_printf(" Gt_RF_Close OK！ ");
							break;  //跳出当前for循环
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
				  Trans_printf(" GT2000_Stop_have_history_alarm ！ \n");
					}
					else
					{
							System.status = SYSTEM_STANDBY;
					Trans_printf(" GT2000_Stop_SYSTEM_STANDBY ！ \n");
					}
					
					System.emission=0;
					Trans_printf(" GT2000_Stop__System.emission=0 Stop_ok！ \n");
					Trans_stopBack.Trans_state[0]=0xFE;		//停止成功
        }
    }
    else     //连查询都不成功，返回停止失败，处于报警状态(这里不修改状态)
    {
        Trans_stopBack.Trans_state[0]=0x02;		//停止失败
        Alarm.no_respond_locate = 13;
			Trans_printf(" GT2000_Stop_Stop_fail ！ \n");
    }

    if( System.emission == 0x00 ) //检查赋值情况,在上面被赋值0
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
				System.status = SYSTEM_SCAN;		//扫下一个频点
			}
		}
		else
		{
			System.status = SYSTEM_STANDBY;
		}
		
        System_Status_Clean();
        App_printf("stop successfully!\r\n");
		    Trans_printf(" GT2000_Stop停止成功！ \n");
    }
    else	//三次停止失败，清除发射位
    {

    }
}

void GT2000_Alarm_Stop(void)
{
	Trans_printf(" 触发GT2000_Alarm_Stop,先关发射，5S后再关机，置位Alarm.emission=1 ");
				Gt_RF_Close();		//先关发射
				vTaskDelay(5000); //等五秒
				Gt2000_Close();   //再关机,无返回数据
			vTaskDelay(50);

			System.status = SYSTEM_ALARM;
	
			Alarm.emission=1;		//激励器报警	
			System.sweep = 0;
			System.stop = 0;
			System.achieve_add_sub = 0;
			System.modify_power = 0;
			System.cancel_add_sub = 0;
			System_Status_Clean();
			Trans_stopBack.Trans_state[0]=0xFE;		//停止成功
	
}

void GT2000_Buffer_Clean(void)  //清零串口fifo队列
{
    comClearRxFifo(COM4);
	  comClearRxFifo(COM1);
}

/*---------------------------------------------------发送------------------------------------------------*/

/* 异或和 */
uint8_t BCC_Check_Sum(uint8_t *pdata, uint8_t len)
{
    uint8_t i = 0;
    uint8_t bcc_result = 0;

    bcc_result = *pdata;
    for(i=1; i<len; i++)
        bcc_result ^= *(pdata+i);

    return bcc_result;
}

/* 开机 */
void Gt2000_Open(void)
{
    uint8_t buf[22] ={0x42,0x0A,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,\
		                  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x41,0x41};
    comSendBuf(COM4, buf, 22);								
}

/* 关机 */
void Gt2000_Close(void)
{
    uint8_t buf[22] ={0x42,0x0A,0x02,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,\
		                  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x41,0x41};
    comSendBuf(COM4, buf, 22);
}

/*开发射*/
void Gt_RF_Open(void)
{
    uint8_t buf[22] ={0x42,0x0A,0x03,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,\
		                  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x41,0x41};
    comSendBuf(COM4, buf, 22);
}

/*关发射*/
void Gt_RF_Close(void)
{
    uint8_t buf[22] ={0x42,0x0A,0x04,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,\
		                  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x41,0x41};
    comSendBuf(COM4, buf, 22);
}

/*功放复位*/
void Gt2000_Reset(void)
{
    uint8_t buf[22] ={0x42,0x0A,0x05,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,\
		                  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x41,0x41};
    comSendBuf(COM4, buf, 22);
}

/*查询所有状态*/
void Gt_Inquire_All(void)
{
    uint8_t buf[22] ={0x42,0x0A,0x14,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,\
		                  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x41,0x41};
    comSendBuf(COM4, buf, 22);
}

/*设置频率1、2、3*/
void Gt_Set_Freq(GT2000_t Set_CMD)
{
	uint8_t i;
		if(Set_CMD.Gt2000_mode==1)		//设置频率1
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
			Gt_Set_Power(Set_CMD);  //设置功率
      vTaskDelay(1000);			
			Trans_printf(" 模式1 buf[0-21]= ");
			for(i=0;i<22;i++)
		{
      Trans_printf(" %02X ",buf[i]);	
		}
      Trans_printf(" \n ");		
		}
		else if(Set_CMD.Gt2000_mode==2)		//先设置频率1，再设置频率2
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
			vTaskDelay(1500);  //等待设置完成			
			GT2000_Buffer_Clean(); //清除串口发送队列
			memset(buf,0x00,22);  //清零buf
			
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
			
      Trans_printf(" 模式2 buf[0-21]= ");
			for(i=0;i<22;i++)
		{
      Trans_printf(" %02X ",buf[i]);	
		}
      Trans_printf(" \n ");		
		}
		else if(Set_CMD.Gt2000_mode==3)		//先设置频率1，再设置频率2，设置频率3
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
			vTaskDelay(1500);  //等待设置完成	
			GT2000_Buffer_Clean();
			memset(buf,0x00,22);  //清零buf
			
			buf[0]=0x42;          //设置频率2
			buf[1]=0x0A;
			buf[2]=0x07;
			buf[3]=0x00;
			memset(buf+4,0x00,4);
			memcpy(buf+8,Set_CMD.Gt2000_freq2,2);
			memset(buf+10,0x00,8);
      buf[20]=0x41;buf[21]=0x41;
      comSendBuf(COM4, buf, 22);
			vTaskDelay(1500);  //等待设置完成
			GT2000_Buffer_Clean();
			memset(buf,0x00,22);  //清零buf
			
			buf[0]=0x42;          //设置频率3
			buf[1]=0x0A;
			buf[2]=0x08;
			buf[3]=0x00;
			memset(buf+4,0x00,8);
			memcpy(buf+12,Set_CMD.Gt2000_freq3,2);
			memset(buf+14,0x00,4);
      buf[20]=0x41;buf[21]=0x41;
      comSendBuf(COM4, buf, 22);
			vTaskDelay(1500);	
			Trans_printf(" 模式3 buf[0-21]= ");
     for(i=0;i<22;i++)
		{
      Trans_printf(" %02X ",buf[i]);	
		}
      Trans_printf(" \n ");				
		
		}		
}

/* 设置功率 */
void Gt_Set_Power(GT2000_t Set_Power)
{	
			uint8_t buf1[22];
			buf1[0]=0x42;
			buf1[1]=0x0A;
			buf1[2]=0x09;
			buf1[3]=0x00;
			memset(buf1+4,0x00,12);
			memcpy(buf1+16,Set_Power.Frequency_power,2); //在Task_Upper_Computer.c中赋值。低位在前
      memset(buf1+18,0x00,2);
	    buf1[20]=0x41;buf1[21]=0x41;
      comSendBuf(COM4, buf1, 22);
	    Trans_printf(" 设置功率命令已下发Set_Power.Frequency_power= %x %x \n",buf1[16],buf1[17]);
}

/* 设置工作模式1~4 */
void Gt_Set_Mode(uint8_t num)
{
		if(num==1)		//设置为单频,返回7B 53 31 7D  设置成功。
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
		else if(num==2)		//设置为双频
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
		else if(num==3)		//设置为三频
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
		else if(num==4)		//设置为AM
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
  Trans_printf(" 设置工作模式%d命令已下发 \n",num);		
}

/*开发射工作参数设置*/
void GT_Set_Value(GT2000_t Send_CMD)  
{
		
		if(Send_CMD.Gt2000_mode==1)		//固频
		{
			Gt_Set_Mode(1);   //设置工作模式为1单频
			vTaskDelay(1000);  //等待设置完毕间隔事时间至少1000
			Gt_Set_Power(Send_CMD);  //设置功率	
			vTaskDelay(1000);
      Gt_Set_Freq(Send_CMD);   //设置频率
			vTaskDelay(1000);			
			Trans_printf(" 固频工作模式参数已下发完毕！\n");
		}
		
		else if(Send_CMD.Gt2000_mode==2) //双频
		{
			Gt_Set_Mode(2);   //设置工作模式为2双频
			vTaskDelay(1000);
			Gt_Set_Power(Send_CMD);  //设置功率	
			vTaskDelay(1000);
      Gt_Set_Freq(Send_CMD);   //设置频率
			Trans_printf(" 双频工作模式参数已下发完毕！\n");
		}
		
		else if(Send_CMD.Gt2000_mode==3) //三频
		{
			Gt_Set_Mode(3);
			vTaskDelay(1500);
			Gt_Set_Power(Send_CMD);  //设置功率
			vTaskDelay(1500);
      Gt_Set_Freq(Send_CMD);   //设置频率
			Trans_printf(" 三频工作模式参数已下发完毕！\n");
		}
		else if(Send_CMD.Gt2000_mode==4)  //AM
		{
			 Gt_Set_Mode(4);
		}
		else 
		{
			 Trans_printf(" error：无此工作模式！Send_CMD.Gt2000_mode==  %d \n",Send_CMD.Gt2000_mode);
		}
		Trans_printf(" 开发射工作参数设置命令已下发完毕！Send_CMD.Gt2000_mode==  %d \n",Send_CMD.Gt2000_mode);
}


uint8_t check_sum(uint8_t *buf,uint16_t len)		//求校验和
{
	uint16_t sum=0;
		for (uint16_t i=0;i<len;i++)
	{
     sum+=buf[i];//将每个数相加
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
		case MT2000_WAIT_ACK_OK:		num = 20;		break;	//应答等 1s
		case MT2000_CMD_INQUIRE:		num = 20;		break;	//查询等 1s
		case MT2000_CMD_CHANNEL:		num = 20;		break;	//设置等 1s
		case MT2000_CMD_TUNING:			num = 600;		break;	//调谐等 30s
		case MT2000_CMD_EMISSION:		num = 20;		break;	//发射等 1s
		case MT2000_CMD_POWER_ADD:		num = 20;		break;	//增加功率等 1s
		case MT2000_CMD_POWER_SUB:		num = 20;		break;	//减小功率等 1s
		case MT2000_CMD_STOP:			num = 20;		break;	//停止等 3s
		case MT2000_CMD_POWER_ON:		num = 300;		break;	//开机等 15s
		case MT2000_CMD_POWER_OFF:		num = 60;		break;	//关机等 3s
		default:						num = 100;		break;	//否则等 5s
    }

    for(i=0; i<num; i++)
    {
        while( comGetChar(COM4,&read) )  //广通机器只从一个串口收发数据
        {                                //串口4是232，串口2采集温湿度数据
            if(count<Emis_Size)
            {
                Emission_Rx[count++]=read;
            }
            vTaskDelay(1);
        }
				
			  if( (count>2) && (Emission_Rx[0]==0x42)&&(Emission_Rx[158]==0x41)&&(Emission_Rx[159]==0x00))  //只解析正确数据
        {
//          /*---------接收数据测试----------*/
//					if (count!=0)  //如果有数据
//					{
//						Trans_printf("\r\n=====================================\r\n");
//						Trans_printf(	"读取到count: %d 个字节 = ",count);
//						for(i = 0; i<(count); i++)
//						{
//								Trans_printf("%02X ",Emission_Rx[i]);
//						}
//						Trans_printf("\r\n=====================================\r\n");
//					}
//			  	/*---------接收数据测试----------*/	
					
					ret = GT2000_Cmd_Analyze(Emission_Rx, cmd);		//解析激励器的数据并返回结果,若有报警，再次判断
					
   if (GT2000_Rx.sys_open==0)		//根据查询到的机器状态，机器关
    {
				System.achieve_add_sub = 0;
				System.modify_power = 0;
				System.cancel_add_sub = 0;
				System_Status_Clean();
				System.Voltage = 0.0;
				System.Electricity = 0.0;
				System.sweep = 0;
				System.open = 0;
				System.close = 0;				//清零

				if( get_history_alarm() == 0x01 )   //有历史报警信息
				{
						System.status = SYSTEM_ALARM;
					  Trans_printf(" 有历史报警消息！ \n ");
				}
				else
				{
					if(System.status!=SYSTEM_OPENING&&System.status!=SYSTEM_ALARM)		//不是正在开机、不是报警状态
					{
						if(GT2000_Rx.sys_open==0)
						{
						System.status = SYSTEM_SHUTDOWN;   //关机状态
						System.Open_Close=0;		//只是用于查询返回//0代表发射机关机，1代表开机
						Trans_printf("关机状态1！ \n");
						}
					}
				}
       if(GT2000_Rx.sys_open==0)
			 {
				 System.Open_Close=0;		//只是用于查询返回//0代表发射机关机，1代表开机
			 }
       else  
				 System.Open_Close=1; 				 
    }
		
		    /*系统处于开机，但未开发射，若不想要电流检测条件，可去掉*/
		if((GT2000_Rx.sys_open==1)&&(GT2000_Rx.sys_emit_open==0))	//开机，但未开发射，待机状态
		{
			System.Voltage=43;
			if(System.status!=SYSTEM_SCAN&&System.status!=SYSTEM_ALARM&&System.Power_Adjustment!=1)		//非扫频状态、报警状态、增减功率时
			{
				System.status=SYSTEM_STANDBY;  //待机状态
			}
			Trans_printf(" 待机状态1！ \n");
			System.Open_Close=1; //开机状态标志位
		 
	  }
		
		if((GT2000_Rx.sys_open==1)&&(GT2000_Rx.sys_emit_open==1))  //根据查询到的系统状态，开机，开发射，说明正在发射
		{
			if(System.status!=SYSTEM_SCAN&&System.status!=SYSTEM_ALARM)  //若不是扫频或报警状态。
			{
				System.status=SYSTEM_EMISSING;	//发射状态	
				System.Voltage=43;   
			}
			Trans_printf("发射状态1！ \n");
			System.Open_Close=1;
		}
	if ((ret==MT2000_ACK_ALARM)&&(GT2000_Rx.sys_open==1)&&(GT2000_Rx.system_status1!=0))		/*详细报警暂不列入监控范围，暂只监控开机时各个模块的报警*/
		{  
				   vTaskDelay(1000);  //延时1s再次判断
					 Gt_Inquire_All();		//再次发射机状态查询
					ret=GT2000_Wait_Ack(MT2000_CMD_INQUIRE);
			
				if((GT2000_Rx.sys_open==1)&&(GT2000_Rx.system_status1!=0))  //依旧有故障
				{
					if( System.time_update_flag == 0x01 )  //更新时间
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

					System.status = SYSTEM_ALARM;    //报警状态
					Alarm.alarm_history = 1;
					Alarm.emission = 0x01;  //激励器报警是总体报警，详细报警信息再日志里查看。
					Trans_printf("emission alarm alarm alarm! GT2000_Rx.system_status1== %02X \r\n",GT2000_Rx.system_status1);
					ret=MT2000_ACK_ALARM;					
				}
					else  //再次判断时无故障
				{	
           if(System.Launch_Switch_state==1)
						{
							GT2000_Rx.sys_emit_open = 1;	//发射/停止状态
						}
				    else
						{
							GT2000_Rx.sys_emit_open = 0;	//发射/停止状态
						}
				  System.Electricity=GT2000_Rx.Working_current; //工作电流  //16~19
					System.Forward_Power=(float)((GT2000_Rx.forward_power[0]<<8)|(GT2000_Rx.forward_power[1])); //发射机入射功率   //6~7
		      System.Reverse_Power=(float)((GT2000_Rx.reverse_power[0]<<8)|(GT2000_Rx.reverse_power[1])); //发射机反射功率   //8~9 
						
					
					System_Status_Update(); //系统状态更新
				  Alarm.no_respond_count = 0;  //新增清除					
					ret=MT2000_ACK_OK;
				}
    }
		else
		{
			 if(System.Launch_Switch_state==1)
						{
							GT2000_Rx.sys_emit_open = 1;	//发射/停止状态
						}
				    else
						{
							GT2000_Rx.sys_emit_open = 0;	//发射/停止状态
						}
				  System.Electricity=GT2000_Rx.Working_current; //工作电流  //16~19
					System.Forward_Power=(float)((GT2000_Rx.forward_power[0]<<8)|(GT2000_Rx.forward_power[1])); //发射机入射功率   //6~7
		      System.Reverse_Power=(float)((GT2000_Rx.reverse_power[0]<<8)|(GT2000_Rx.reverse_power[1])); //发射机反射功率   //8~9 
						
					
					System_Status_Update(); //系统状态更新
				  Alarm.no_respond_count = 0;  //新增清除					
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
                    if( (System.status==SYSTEM_EMISSING) || (System.status==SYSTEM_SCAN) )	//发射/扫频状态，只有严重报警(激励器报警、设备无响应报警和无功率报警)才切换为报警状态
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
                        System.status = SYSTEM_UNINITIALIZE;  //未初始化
                    }
                }
//								Trans_printf("发射机响应OK  \n");
            }
#endif

            return ret;
        }
        count=0;   //清零
        vTaskDelay(50);
    }

		if(System.status!=SYSTEM_UNINITIALIZE) //只要不是未初始化
		{
			if( Alarm.no_respond == 0x00 )
			{
					Alarm.no_respond_count++;
					if( Alarm.no_respond_count >= NO_RESPOND_MAX )  //检测发射机是否失联，检测串口4是否有数据
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

							System.emission = 0x00;		//设备无响应时，清除发射标志位
							Alarm.no_respond = 0x01;
							Trans_printf(" Alarm.no_respond 1 ! ");
							Alarm.alarm_history = 0x01;  //置位无响应报警，说明未收到发射据的正确数据。

							System.achieve_add_sub = 0x00;
							System.modify_power = 0x00;
							System.cancel_add_sub = 0x00;

							temp = System.emission;
							System.Voltage = 0.0;
							System.Electricity = 0.0;
							System_Status_Clean();		//不清，先返回给PC当前状态，再自己停止3次，3次失败再清空	再修正：清除电压电流功率	--20181031 Luonus
							System.emission = temp;		//要停止
					}
				}
    }

    return MT2000_NO_ACK;	//无应答	

}
	
int GT2000_Cmd_Analyze(uint8_t *msg, uint8_t cmd)
{
	uint16_t Electricity_1[4],Electricity_2[4],Electricity_3[4],Electricity_4[4];		//存储功放一~四的电流数据
		float Average_E[4];   //用于存放电流平均值
		float Electricity_Add_1=0,Electricity_Add_2=0,Electricity_Add_3=0,Electricity_Add_4=0;
	
		/*接收电流数据*/
  	Electricity_1[0]=msg[41];   //以太网转232传输，低位在前
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
		Electricity_4[3]=msg[68];    //以太网转232传输，低位在前
		
//		Trans_printf("Electricity_电流== %d %d %d %d ",Electricity_1[0],\
//		Electricity_1[1],Electricity_1[2],Electricity_1[3]);  //打印2KW调试信息
		/*将电流数据放于系统信息结构体中*/
		for(uint8_t i=0;i<4;i++)
		{ 
			//延用之前的系统信息结构体，名字暂时未改。
			System.Bj_Electricity_1[i]=Electricity_1[i]*10;  //电流信息扩大10倍
			System.Bj_Electricity_2[i]=Electricity_2[i]*10;
			System.Bj_Electricity_3[i]=Electricity_3[i]*10;
			System.Bj_Electricity_4[i]=Electricity_4[i]*10;
		}
		
		
		/*取故障情况，这些报警均属于激励器报警！*/	//bit0为低位
		Alarm_historyBack.Alarm_Abnormal_Output[0]=((msg[20]&0x01));             //bit0输出异常 0正常  1故障
		Alarm_historyBack.Alarm_Reverse[0]=((msg[20]&0x02)>>1);                     //bit1反射过大
		Alarm_historyBack.Alarm_Module_Electricity[0]=((msg[20]&0x08)>>3);          //bit3模块电流报警 
    Alarm_historyBack.Alarm_Absorber_State_1[0]=((msg[20]&0x40)>>6);          //bit6滤波器报警
    Alarm_historyBack.Alarm_Switch_Power[0]=((msg[20]&0x80)>>7);                 //bit7开关电源报警	
		Trans_printf("Alarm_status1_msg[20]== %x ",msg[20]);  //打印2KW调试信息
		
		
		
    GT2000_Rx.system_status1=(msg[20]&0xff);      //判断故障情况，0为正常，1为故障
//		Trans_printf("GT2000_Rx.system_status1= %x \n",GT2000_Rx.system_status1);
		
		/*其他值*/  //以太网通讯低位在前
  	System.Attenuation=msg[14];     //衰减值  //0或3 
    GT2000_Rx.swr[0]=msg[13];       //驻波比  //10~13
		GT2000_Rx.swr[1]=msg[12];
		GT2000_Rx.swr[2]=msg[11];
		GT2000_Rx.swr[3]=msg[10];
		GT2000_Rx.sys_open=((msg[21]&0x01));            //bit0发射机开关机状态 0关机  1开机
		GT2000_Rx.sys_emit_open=((msg[21]&0x02)>>1);    //bit1发射机发射状态  0关发射  1开发射

		if(((msg[21]&0x04)>>2)==0)
		{
			System.Control_Model=1;   //bit2发射机控制模式  1遥控   0本控  System.Control_Model  0自动/1手动
			Trans_printf("msg[21]Bit2== %x ",((msg[21]&0x04)>>2));
		}
		else if(((msg[21]&0x04)>>2)==1)
		{
			System.Control_Model=0;   //bit2发射机控制模式  1遥控   0本控
//			Trans_printf("msg[21]== %x ",((msg[21]&0x04)>>2));
		}
		
		GT2000_Rx.Gt2000_mode=msg[30];   //1固频、2双频、3三频、4AM 
		GT2000_Rx.Gt2000_freq1[0]=msg[23];  //当前频率1  //22~23
		GT2000_Rx.Gt2000_freq1[1]=msg[22];  //当前频率1  //22~23
		memset(GT2000_Rx.Gt2000_freq1+2,0x00,2);
		GT2000_Rx.Gt2000_freq2[0]=msg[25];  //当前频率2  //24~25
		GT2000_Rx.Gt2000_freq2[1]=msg[24];  //当前频率2  //24~25
		memset(GT2000_Rx.Gt2000_freq2+2,0x00,2);
		GT2000_Rx.Gt2000_freq3[0]=msg[27];  //当前频率3  //26~27
		GT2000_Rx.Gt2000_freq3[1]=msg[26];  //当前频率3  //26~27
		memset(GT2000_Rx.Gt2000_freq3+2,0x00,2);
		Trans_printf(" GT2000_Rx.Gt2000_freq1[0]== %x ",GT2000_Rx.Gt2000_freq1[0],GT2000_Rx.Gt2000_freq1[1]);
		
		/* 取发射机当前功率值 */
		GT2000_Rx.Frequency_power[0]=msg[29];
		GT2000_Rx.Frequency_power[1]=msg[28];
		GT2000_Rx.Now_all_power=((msg[29]<<8)|msg[28]);
		/*取工作模式*/
		System.INQUIRE_Mode=msg[30];   //1固频、2双频、3三频、4AM 
				  
		GT2000_Rx.Working_current=(float)(((msg[19]<<24)+(msg[18]<<16)+(msg[17]<<8)+(msg[16]))/4.0); //工作电流  //16~19
		GT2000_Rx.forward_power[0]=msg[7];GT2000_Rx.forward_power[1]=msg[6];  //发射机入射功率   //6~7
		GT2000_Rx.reverse_power[0]=msg[9];GT2000_Rx.forward_power[1]=msg[8];  //发射机反射功率   //8~9 
		
		  if(System.Launch_Switch_state==1)
				{
					GT2000_Rx.sys_emit_open = 1;	//发射/停止状态
				}
				else
				{
					GT2000_Rx.sys_emit_open = 0;	//发射/停止状态
				}
		
		System.Amplifier_Temperature[0]=(((((msg[48])+(msg[47]))/8.0)-8))*100;//模块1温度  //45~48
		System.Amplifier_Temperature[1]=(((((msg[56])+(msg[55]))/8.0)-8))*100;//模块2温度  //56~53
		System.Amplifier_Temperature[2]=(((((msg[64])+(msg[63]))/8.0)-8))*100;//模块3温度  //64~61
		System.Amplifier_Temperature[3]=(((((msg[72])+(msg[71]))/8.0)-8))*100;//模块4温度  //72~69
    
//		Trans_printf(" 模块温度：%x %x %x %x \n",System.Amplifier_Temperature[0],System.Amplifier_Temperature[0],\
//		System.Amplifier_Temperature[1],System.Amplifier_Temperature[2],System.Amplifier_Temperature[3]);
		
		System.Frequency_band_value=msg[31];   //频段值
//		Trans_printf("band_value= %d ",System.Frequency_band_value);
		
		System.Launch_Switch_state=((msg[21]&0x02)>>1);    //bit1发射机返回的发射状态 0关发射  1开发射
		
		for(uint8_t i=0;i<4;i++)
		{
			Electricity_Add_1 += Electricity_1[i];  //电流数据为平均值
			Electricity_Add_2 += Electricity_2[i];
			Electricity_Add_3 += Electricity_3[i];
			Electricity_Add_4 += Electricity_4[i];
		}
		
		Average_E[0]=Electricity_Add_1/4.0;  //功放1电流
		Average_E[1]=Electricity_Add_2/4.0;  //功放2电流
		Average_E[2]=Electricity_Add_3/4.0;  //功放3电流
		Average_E[3]=Electricity_Add_4/4.0;  //功放4电流
//		Trans_printf(" 电流：%f %f %f %f \n",Average_E[0],Average_E[1],Average_E[2],Average_E[3]);    
		
			if ((GT2000_Rx.sys_open==1)&&(GT2000_Rx.system_status1!=0))		/*详细报警暂不列入监控范围，暂只监控开机时各个模块的报警*/
			  {  
				   //这里不操作防止误判
					return MT2000_ACK_ALARM;
        }
			else   //若无故障则进入此结构
			{					
					return MT2000_ACK_OK;
		  }
}


/* 异或和 */
uint8_t Bj_BCC_Check_Sum(uint8_t *pdata, uint8_t len)
{
    uint8_t i = 0;
    uint8_t bcc_result = 0;

    bcc_result = pdata[1];
    for(i=1; i<len-1; i++)
        bcc_result ^= pdata[i+1];

    return bcc_result;
}

/*PC下发的数据转换为功率值 [100~250],对应【0~1950W】,1*13*/
void Power_Range(GT2000_t *Value,uint8_t *Data)
{
	uint8_t i;
	for(i=0;i<3;i++)
	{
		if(Data[i]<200)
		{
			if(i==0)
			{   //新发射机采用以太网通讯，低位在前
				Value->Frequency_power[1]=((((Data[i]-100)*13)&0xff00)>>8);   //取功率的高位字节
				Value->Frequency_power[0]=(((Data[i]-100)*13)&0xff);   //取功率的低位字节
				Value->Now_all_power=((Value->Frequency_power[1]<<8)|Value->Frequency_power[0]);  //十六进制总功率
			}
		}
		
		else if(Data[i]>=200&&Data[i]<250)
		{
			if(i==0)
			{    //新发射机采用以太网通讯，低位在前
				Value->Frequency_power[1]=((((Data[i]-100)*13)&0xff00)>>8);   //取功率的高位字节
				Value->Frequency_power[0]=(((Data[i]-100)*13)&0xff);   //取功率的低位字节
				Value->Now_all_power=((Value->Frequency_power[1]<<8)|Value->Frequency_power[0]);  //十六进制总功率
			}
		}
		
		else		//参数错误
		{
				if(i==0)
			{
				Value->Frequency_power[0]=0;   //
				Value->Frequency_power[1]=0;   //
			}
		}
	}
}

/* 功率转化为系统幅度值 */
void Range_Power(GT2000_t *Value)
{
	System.power[0]=(Value->Now_all_power/13); 
	/* GT2000新机器只显示一个功率。 */	
}

//功率微调 +65
void power_add(uint8_t *data)
{
	uint16_t All_power;
	All_power=((data[1]<<8)|data[0]);
	All_power=All_power+0x41;
	data[1]=((All_power&0xff00)>>8);
	data[0]=(All_power&0xff);
}


//功率微减 -65
void power_sub(uint8_t *data)
{
	uint16_t All_power;
	All_power=((data[0]<<8)|data[1]);
	All_power=All_power-0x41;
	data[0]=((All_power&0xff00)>>8);
	data[1]=(All_power&0xff);
}


