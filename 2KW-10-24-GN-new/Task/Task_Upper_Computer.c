#include "Task_Config.h"
#include "bsp_uart_fifo.h"
#include "my_protocol.h"
#include "bsp_adc.h"
#include "alarm.h"
#include "power.h"
#include "W5500.h"
#include "main.h"

extern uint8_t g_fuc_cod[2];			//全局功能码

extern float Scan_Begin;			//范围扫频起始点
extern float Scan_End;				//范围扫频结束点

Buffer_t RxBuf_FromPC;

TaskHandle_t xHandleTask_Upper_Computer;

/* 查询指令0201，直接发送并返回，其他的都是通知Task_MT2000发送，等待PC发0201查询结果 */
void Task_Upper_Computer(void *pvParameters)
{
    uint8_t	  j = 0;				//失联，搜索运行图时使用
    uint8_t		k = 0;					//普通for循环	(停止和关机等待)

    int			count =0;
    int			ret=0;

    static		uint8_t task_upper_inquire_counter = 0;

    Alarm.no_respond_locate = 0;
    Alarm.no_respond_count = 0;

	
    //未初始化，不开机检测
    if( System.already_init == 0x00 )
    {
        System.status = SYSTEM_UNINITIALIZE;
    }
    else
    {		
			Gt_Inquire_All();
			ret=GT2000_Wait_Ack(MT2000_CMD_INQUIRE);

        if( ret == MT2000_ACK_OK )
        {
            if( System.Voltage > 40.0 )
            {
                System.open = 0x02;

                if( System.emission == 0x01 )
                {
                    System.status = SYSTEM_EMISSING;
                }
                else
                {
                    System.status = SYSTEM_STANDBY;
                }			
            }
        }
        else
        {
            if( get_history_alarm() == 0x01 )
            {
                System.status = SYSTEM_ALARM;
            }

            Alarm.no_respond_locate = 15;
        }

        if( System.already_init == 0 )
        {
            System.status = SYSTEM_UNINITIALIZE;
        }
    }

    while(1)
    {
        update_status_without_inquire();		//只更新状态，不查询
        count = W5500_rx_buf_len;
        W5500_rx_buf_len = 0;

        if(count>2)
        {
            RxBuf_FromPC.len = count;//整体数据长度
            ret=CAN_data_analyze(g_fuc_cod, &RxBuf_FromPC);
            count = 0;

            if(ret==1)
            {
                Monitor.no_respond_count=0;	//正常通信则清零
                Monitor.hard_control=0;
                Monitor.need_emit = 0;
                Monitor.need_open = 0;
                Monitor.need_close = 0;

                Run_Diagram_data.mode = 0;	//初始化
                Run_Diagram_data.power[0] = 0;
                Run_Diagram_data.power[1] = 0;
                Run_Diagram_data.power[2] = 0;
                memset(Run_Diagram_data.Freq, 0, sizeof(Run_Diagram_data.Freq));

                if(Monitor.hard_control==1)
                {
                    Monitor.usage_diagram_count=0;
                    System.mode = 0;		//先清零，再查询赋值

                    for(int i=0; i<flash3_Save; i++)
                    {
                        for(j=0; j<10; j++)
                        {
                            Monitor.start[i][j] = 0;
                            Monitor.end[i][j] = 0;
                        }
                    }
                }

                func_code_printf();

                /* 根据功能码，执行相应操作 */
                if(g_fuc_cod[0]==0x01&&g_fuc_cod[1]==0x01)				//初始化
                {
										System.Init_Mark=1;
                    System.CAN_ID[0] = Alarm_threshold.Transmitte_id[0];	//存在flash

                    Alarm_backPC.Transmitte_id[0]=System.CAN_ID[0];
                    g_fuc_codToPC[0]=0x01;
                    g_fuc_codToPC[1]=0x02;

                    Time_Printf( (RTC_GetCounter()+g_stamp_distance), &set_time );

                    if(InternalFlash_SaveData_1(0x01)==0)		/* 写入flash */
                    {
                        System.already_init = 0;
                        flash_1_once_flag = 0;

                        Alarm_backPC.Alarm_paramet[0] = 0xFC;		//初始化失败
                        Alarm_backPC.Alarm_paramet[1] = 0x00;
                    }
                    else
                    {
                        System.already_init = 1;
                        flash_1_once_flag = 1;

                        Alarm_backPC.Alarm_paramet[0] = 0xFE;	//成功初始化并保存在flash
                        Alarm_backPC.Alarm_paramet[1] = 0x00;
                        Flash_to_AcceptAPP();

                        if( System.Voltage > 40.0 )
                        {
                            System.open = 0x02;
                        }
                        else
                        {
                            System.open = 0x00;
                        }
						
												if( get_history_alarm() == 0x01 )
												{
													System.status = SYSTEM_ALARM;
												}
												else	if( System.open == 0x02 )
												{
													System.status = SYSTEM_STANDBY;		//查询时，开机了，不一定是待机，有可能是发射/增减功率/扫频等。
												}
												else
												{
													System.status = SYSTEM_SHUTDOWN;
												}
                    }
										
                    Send_PC(g_fuc_codToPC);
                }
                else if(g_fuc_cod[0]==0x02&&g_fuc_cod[1]==0x01)			//查询
                {
                    g_fuc_codToPC[0]=0x02;
                    g_fuc_codToPC[1]=0x02;			
				 	          g_stamp_distance = g_inquire_stamp - RTC_GetCounter();	//获取本地时间与服务器时间的差值

                    System.time_update_flag = 0x01;//更新了时间标志位

                    //Work_paraBack清零
                    Work_paraBack.Transmitte_id[0]=System.CAN_ID[0];	//CAN_ID
                    Work_paraBack.Mode[0]=0;
                    memset(Work_paraBack.Freq,0,12);
                    Work_paraBack.Power_45_voltage[0]=0;
                    Work_paraBack.Power_45_intensity[0]=0;
                    memset(Work_paraBack.Freq,0,12);			//频率先清零，后赋值
                    Work_paraBack.Forward_power[0] = 0.0;		//功率先清零，后赋值
                    Work_paraBack.Reverse_power[0] = 0.0;
                    Work_paraBack.emission[0]=0x00;
									
										memset(Work_paraBack.Alarm_Reservation,0,2);
										memset(Work_paraBack.System_Version,0,20);
										Work_paraBack.Divider[0]=0xFF;
									  Work_paraBack.Control_Model[0]=0;
									  Work_paraBack.Frequency_band_value[0]=0;
										Work_paraBack.Attenuation[0]=0;
										memset(Work_paraBack.Reservation,0,2);																
										memset(Work_paraBack.Bj_Electricity_1,0,4);
										memset(Work_paraBack.Bj_Electricity_2,0,4);
										memset(Work_paraBack.Bj_Electricity_3,0,4);
										memset(Work_paraBack.Bj_Electricity_4,0,4);
										Work_paraBack.Amplifier_Temperature_1[0]=0;
										Work_paraBack.Amplifier_Temperature_2[0]=0;
										Work_paraBack.Amplifier_Temperature_3[0]=0;
										Work_paraBack.Amplifier_Temperature_4[0]=0;
										Work_paraBack.System_Version[16]=System.Model[0];		//程序默认值，不改变
									
                    Work_paraBack.Channel[0] = PC_Cmd.channel[0];	//发射的信道		(旧机器有3信道，新机器只有1个信道，兼容，直接原样返回)
                    Work_paraBack.Channel[1] = PC_Cmd.channel[1];
                    Work_paraBack.Channel[2] = PC_Cmd.channel[2];

                    Work_paraBack.Power_grade[0] = PC_Cmd.power[0];	//发射的功率等级	(旧机器有3功率等级，新机器只有1个功率等级，兼容，直接原样返回)
                    Work_paraBack.Power_grade[1] = PC_Cmd.power[1];
                    Work_paraBack.Power_grade[2] = PC_Cmd.power[2];
										
										Work_paraBack.Control_Model[0]=System.Control_Model;   //0自动/1手动
										Work_paraBack.Frequency_band_value[0]=System.Frequency_band_value;   //频段值
										Work_paraBack.Attenuation[0]=System.Attenuation;  //衰减值
										
										for(uint8_t n=0;n<4;n++)
										{
											Work_paraBack.Bj_Electricity_1[n]=Convert_byte_order_16(System.Bj_Electricity_1[n]);  
											Work_paraBack.Bj_Electricity_2[n]=Convert_byte_order_16(System.Bj_Electricity_2[n]);
											Work_paraBack.Bj_Electricity_3[n]=Convert_byte_order_16(System.Bj_Electricity_3[n]);
											Work_paraBack.Bj_Electricity_4[n]=Convert_byte_order_16(System.Bj_Electricity_4[n]);
										}
										Work_paraBack.Amplifier_Temperature_1[0]=Convert_byte_order_16(System.Amplifier_Temperature[0]);
										Work_paraBack.Amplifier_Temperature_2[0]=Convert_byte_order_16(System.Amplifier_Temperature[1]);
										Work_paraBack.Amplifier_Temperature_3[0]=Convert_byte_order_16(System.Amplifier_Temperature[2]);
										Work_paraBack.Amplifier_Temperature_4[0]=Convert_byte_order_16(System.Amplifier_Temperature[3]);
										
                    set_alarm_bit();  //置位报警标志位 
                    Work_paraBack.current_alarm_state[0] = Alarm_historyBack.alarm_history[0];
                    Work_paraBack.current_alarm_state[1] = Alarm_historyBack.alarm_history[1];

                    Work_paraBack.temp[0]=Temperature_Humidity.Temperature;		//温度
                    Work_paraBack.humidity[0]=Temperature_Humidity.Humidity;	//湿度


                    if(System.status == SYSTEM_UNINITIALIZE )			//未初始化状态
                    {
                        Work_paraBack.Trans_current_state[0] = SYSTEM_UNINITIALIZE;
                        Work_paraBack.Power_45_voltage[0]	 = System.Voltage;
                        Work_paraBack.Power_45_intensity[0]	 = 0;
												Work_paraBack.Amplifier_Temperature_1[0]=Convert_byte_order_16(System.Amplifier_Temperature[0]);
												Work_paraBack.Amplifier_Temperature_2[0]=Convert_byte_order_16(System.Amplifier_Temperature[1]);
												Work_paraBack.Amplifier_Temperature_3[0]=Convert_byte_order_16(System.Amplifier_Temperature[2]);
												Work_paraBack.Amplifier_Temperature_4[0]=Convert_byte_order_16(System.Amplifier_Temperature[3]);
			
										    for(uint8_t n=0;n<4;n++)
												{
													Work_paraBack.Bj_Electricity_1[n]=Convert_byte_order_16(System.Bj_Electricity_1[n]);
													Work_paraBack.Bj_Electricity_2[n]=Convert_byte_order_16(System.Bj_Electricity_2[n]);
													Work_paraBack.Bj_Electricity_3[n]=Convert_byte_order_16(System.Bj_Electricity_3[n]);
													Work_paraBack.Bj_Electricity_4[n]=Convert_byte_order_16(System.Bj_Electricity_4[n]);
												}
												Trans_printf(" 发射机处于未初始化状态 \n");
                    }
                    else	if( (Alarm.emission==0x01) || (Alarm.no_respond==0x01) || (Alarm.no_power==0x01) ||(Alarm.power_cataclysm==1) )	//激励器自身报警，无应答报警、无功率报警
                    {
                        //激励器自身报警，无应答报警，无功率报警，功率骤变属于严重报警，查询时马上返回，其他报警则先返回操作失败，回到待机/发射/报警状态，等待查询时再报警

                        Work_paraBack.Trans_current_state[0]=SYSTEM_ALARM;			//当前是报警状态
                        Work_paraBack.Mode[0] = System.INQUIRE_Mode;					//模式

//                        freq_str_to_hex(System.freq1,	Work_paraBack.Freq);		//频率
//											  freq_str_to_hex(System.freq2,	Work_paraBack.Freq+4);
//                        freq_str_to_hex(System.freq3,	Work_paraBack.Freq+8);
											  StrToHex(Work_paraBack.Freq, System.freq1, 4); //remarks : 将字符串转化为16进制数
                        StrToHex(Work_paraBack.Freq+4, System.freq2, 4); //remarks : 将字符串转化为16进制数
											  StrToHex(Work_paraBack.Freq+8, System.freq3, 4); //remarks : 将字符串转化为16进制数

                        Work_paraBack.Forward_power[0] = (float)System.Forward_Power;
                        Work_paraBack.Reverse_power[0] = (float)System.Reverse_Power;

                        Work_paraBack.Power_45_voltage[0]	= System.Voltage;
                        Work_paraBack.Power_45_intensity[0]	= 0;
												for(uint8_t n=0;n<4;n++)
												{
													Work_paraBack.Bj_Electricity_1[n]=Convert_byte_order_16(System.Bj_Electricity_1[n]);
													Work_paraBack.Bj_Electricity_2[n]=Convert_byte_order_16(System.Bj_Electricity_2[n]);
													Work_paraBack.Bj_Electricity_3[n]=Convert_byte_order_16(System.Bj_Electricity_3[n]);
													Work_paraBack.Bj_Electricity_4[n]=Convert_byte_order_16(System.Bj_Electricity_4[n]);
												}
                        Work_paraBack.emission[0] = (System.emission&0x01)|(System.Open_Close&0x02<<1);		//待测试

                        if( System.emission == 0x01 )
                        {
                            System.stop = 1;
                            xTaskNotify(xHandleTask_MT2000,				//停止发射
                                        BIT_0,
                                        eSetBits);
                        }
												Trans_printf(" 发射机处于紧急报警状态！停止发射  \n");
                    }
                    else	if( System.status == SYSTEM_OPENING )		//正在开机
                    {
                        Work_paraBack.Trans_current_state[0]=SYSTEM_OPENING;
											  Trans_printf(" 发射机处于正在开机  \n");
                    }
                    else	if( System.status == SYSTEM_SCAN )			//扫频
                    {
												if( (Alarm.swr_alarm==0x01) || (Alarm.over_Electric==0x01) )		//扫频时，查询不返回驻波比报警和过流报警(再次清零)
												{
													Alarm.swr_alarm = 0x00;
													Alarm.over_Electric = 0x00;
												}
						
                        Work_paraBack.Trans_current_state[0]=SYSTEM_SCAN;
                        Work_paraBack.Mode[0]=0x01;
                        Work_paraBack.emission[0]=(System.emission&0x01)|(System.Open_Close&0x02<<1);
												
												
											GT2000_Tx_freq_Hex_to_PChex(GT2000_Tx.Gt2000_freq1, Work_paraBack.Freq);  //查询到的十六进制数据转化为上位机的十六进制
												
                        Trans_printf(" Task_upper查询扫频状态任务中 GT2000_Tx.Gt2000_freq1= %02X %02X %02X %02X \n",\
												GT2000_Tx.Gt2000_freq1[0],GT2000_Tx.Gt2000_freq1[1],GT2000_Tx.Gt2000_freq1[2],GT2000_Tx.Gt2000_freq1[3]);
												Trans_printf(" Task_upper查询扫频状态任务中 Work_paraBack.Freq[1-4]= %02X %02X %02X %02X \n",\
												Work_paraBack.Freq[0],Work_paraBack.Freq[1],Work_paraBack.Freq[2],Work_paraBack.Freq[3]);
												
                        Work_paraBack.Power_grade[0] = 180;		//扫频默认功率等级-8幅度
                        Work_paraBack.Forward_power[0] = (float)System.Forward_Power;
                        Work_paraBack.Reverse_power[0] = (float)System.Reverse_Power;
                        Work_paraBack.Power_45_voltage[0]	= System.Voltage;
                        Work_paraBack.Power_45_intensity[0]	= 0;
											  for(uint8_t n=0;n<4;n++)
												{
													Work_paraBack.Bj_Electricity_1[n]=Convert_byte_order_16(System.Bj_Electricity_1[n]);
													Work_paraBack.Bj_Electricity_2[n]=Convert_byte_order_16(System.Bj_Electricity_2[n]);
													Work_paraBack.Bj_Electricity_3[n]=Convert_byte_order_16(System.Bj_Electricity_3[n]);
													Work_paraBack.Bj_Electricity_4[n]=Convert_byte_order_16(System.Bj_Electricity_4[n]);
												}
												Trans_printf(" 发射机处于扫频状态  \n");
                    }
                    else	if( System.status == SYSTEM_SHUTDOWN )		//关机
                    {
                        Work_paraBack.Trans_current_state[0]=SYSTEM_SHUTDOWN;
											  Trans_printf(" 发射机处于关机状态  \n");
                    }
   
                    else	if( System.status == SYSTEM_STANDBY )		//待机状态
                    {
                        Work_paraBack.Trans_current_state[0] = System.status;
                        Work_paraBack.Power_45_voltage[0]	 = System.Voltage;
                        Work_paraBack.Power_45_intensity[0]  = 0.0;
											  for(uint8_t n=0;n<4;n++)
												{
													Work_paraBack.Bj_Electricity_1[n]=Convert_byte_order_16(System.Bj_Electricity_1[n]);
													Work_paraBack.Bj_Electricity_2[n]=Convert_byte_order_16(System.Bj_Electricity_2[n]);
													Work_paraBack.Bj_Electricity_3[n]=Convert_byte_order_16(System.Bj_Electricity_3[n]);
													Work_paraBack.Bj_Electricity_4[n]=Convert_byte_order_16(System.Bj_Electricity_4[n]);
												}
												Trans_printf(" 发射机处于待机状态  \n");
                    }
										
                    else	if( System.status == SYSTEM_EMISSING )		//发射状态
                    {
                        Work_paraBack.Trans_current_state[0]=SYSTEM_EMISSING;
                        Work_paraBack.Mode[0] = System.INQUIRE_Mode;					//模式

											freq_GT2000Hex_to_PChex(System.freq1, Work_paraBack.Freq);  //查询到的十六进制数据转化为上位机的十六进制
											freq_GT2000Hex_to_PChex(System.freq2, Work_paraBack.Freq+4);
											freq_GT2000Hex_to_PChex(System.freq3, Work_paraBack.Freq+8);
//											memcpy(Work_paraBack.Freq, System.freq1, 4);   //赋值查询到的频率给上位机，System.freq1在System_Status_Update中更新。
//											  memcpy(Work_paraBack.Freq+4, System.freq2, 4);
//											  memcpy(Work_paraBack.Freq+8, System.freq3, 4);
									    Trans_printf(" 查询到发射状态Work_paraBack.Mode[0]=  %d \nWork_paraBack.Freq=  ",Work_paraBack.Mode[0]);
											for(k=0;k<12;k++)
											{
									    Trans_printf("%02x ",Work_paraBack.Freq[k]);
                      }
											Trans_printf(" \n  System.freq1= ");
											for(k=0;k<4;k++)
											{
											Trans_printf("%02x ",System.freq1[k]);
											}
											Trans_printf(" \n");
                        Work_paraBack.Forward_power[0] = (float)System.Forward_Power;
                        Work_paraBack.Reverse_power[0] = (float)System.Reverse_Power;
											Trans_printf("%02x ",System.freq1[k]);
											

                        Work_paraBack.Power_45_voltage[0]	= System.Voltage;
                        Work_paraBack.Power_45_intensity[0]	= 0;
												for(uint8_t n=0;n<4;n++)
												{
													Work_paraBack.Bj_Electricity_1[n]=Convert_byte_order_16(System.Bj_Electricity_1[n]);
													Work_paraBack.Bj_Electricity_2[n]=Convert_byte_order_16(System.Bj_Electricity_2[n]);
													Work_paraBack.Bj_Electricity_3[n]=Convert_byte_order_16(System.Bj_Electricity_3[n]);
													Work_paraBack.Bj_Electricity_4[n]=Convert_byte_order_16(System.Bj_Electricity_4[n]);
												}
												Trans_printf(" Work_paraBack.Bj_Electricity_1_4== ");
												for(uint8_t i=0;i<4;i++)
												{
													Trans_printf(" %x ",Work_paraBack.Bj_Electricity_1[i]);  //打印调试信息
												}
												Trans_printf(" \n ");
                        Work_paraBack.emission[0]=(System.emission&0x01)|(System.Open_Close&0x02<<1);
												Trans_printf(" 发射机处于发射状态  \n");
                    }
                    else	if( System.status == SYSTEM_ALARM )			//报警状态
                    {
                        Work_paraBack.Trans_current_state[0]=SYSTEM_ALARM;			//当前是报警状态
                        Work_paraBack.Mode[0] = System.INQUIRE_Mode;					//模式

                        freq_GT2000Hex_to_PChex(System.freq1, Work_paraBack.Freq);  //查询到的十六进制数据转化为上位机的十六进制
										  	freq_GT2000Hex_to_PChex(System.freq2, Work_paraBack.Freq+4);
										  	freq_GT2000Hex_to_PChex(System.freq3, Work_paraBack.Freq+8); 
//											  memcpy(Work_paraBack.Freq, System.freq1, 4);   //赋值查询到的频率给上位机，System.freq1在System_Status_Update中更新。
//											  memcpy(Work_paraBack.Freq+4, System.freq2, 4);
//											  memcpy(Work_paraBack.Freq+8, System.freq3, 4);
											Trans_printf(" 查询到报警状态状态Work_paraBack.Freq=  ");
											for(k=0;k<12;k++)
											{
									    Trans_printf(" %x  \n",Work_paraBack.Freq[k]);
                      }
											Trans_printf(" System.freq1= %x  ",System.freq1);
											
                        Work_paraBack.Forward_power[0] = (float)System.Forward_Power;
                        Work_paraBack.Reverse_power[0] = (float)System.Reverse_Power;

                        Work_paraBack.Power_45_voltage[0]	= System.Voltage;
                        Work_paraBack.Power_45_intensity[0]	= 0;
											  for(uint8_t n=0;n<4;n++)
												{
													Work_paraBack.Bj_Electricity_1[n]=Convert_byte_order_16(System.Bj_Electricity_1[n]);
													Work_paraBack.Bj_Electricity_2[n]=Convert_byte_order_16(System.Bj_Electricity_2[n]);
													Work_paraBack.Bj_Electricity_3[n]=Convert_byte_order_16(System.Bj_Electricity_3[n]);
													Work_paraBack.Bj_Electricity_4[n]=Convert_byte_order_16(System.Bj_Electricity_4[n]);
												}
                        Work_paraBack.emission[0] = (System.emission&0x01)|(System.Open_Close&0x02<<1);
												Trans_printf(" 发射机处于报警状态  \n");
                    }
                    else												//保留
                    {
                        //App_printf("PC inquire when System.status = ???\r\n");
											Trans_printf(" 未获取到发射机状态？？？？  \n");
                    }
										
                    Send_PC(g_fuc_codToPC);
										
                }
                else	if(g_fuc_cod[0]==0x03&&g_fuc_cod[1]==0x01)		//发射
                {
                    Time_Printf( (RTC_GetCounter()+g_stamp_distance), &set_time );  //时间戳 -> 北京时间并打印

                    Trans_openBack.Transmitte_id[0]=System.CAN_ID[0];
                    g_fuc_codToPC[0]=0x03;
                    g_fuc_codToPC[1]=0x02;

                    if( System.open != 0x02 )					//是否开机//开机标志位	0:关机	1:正在开机	2:已经开机
                    {
											Trans_openBack.Trans_state[0]=0xFB;  //0xFB表示没有开机
                    }
                    else	if( System.emission == 0x01 )		//是否正在发射
										{
											Trans_openBack.Trans_state[0]=0xFC;  //0xFC表示发射失败
										}
										else										//发射
                    {
                        Trans_openBack.Trans_state[0]=0;	//清零	调用BJ2000_Open()开机，收到发射机应答更新Trans_openBack.Trans_state[0]。

                        /* 参数判断，参数正确则赋值给MT2000_Tx，通过MT2000_Cmd_Channel()和MT2000_Cmd_Tune()完成调频调谐，最后MT2000_Cmd_Emit()发射 */
                        switch( PC_Cmd.mode )
                        {
													case 0x01:	GT2000_Tx.Gt2000_mode = 1;					break;  //赋值mode，GT2000_Tx为获取到的上位机下发的数据
													case 0x02:	GT2000_Tx.Gt2000_mode = 2;					break;
													case 0x03:	GT2000_Tx.Gt2000_mode = 3;					break;
													default:	Trans_openBack.Trans_state[0]=0xFE;		break;	//参数错误
                        }
												
											  if(PC_Cmd.power[0]<100&&PC_Cmd.power[0]>=250)
												{
													Trans_openBack.Trans_state[0]=0xFE;		//参数错误
												}
												if(PC_Cmd.power[1]<100&&PC_Cmd.power[1]>=250)
												{
													Trans_openBack.Trans_state[0]=0xFE;		//参数错误
												}
												if(PC_Cmd.power[2]<100&&PC_Cmd.power[2]>=250)
												{
													Trans_openBack.Trans_state[0]=0xFE;		//参数错误
												}        
													
                        if( GT2000_Tx.Gt2000_mode > 0 )
                        {  //判断下发频率必须在3.2~21.85
                            if( (PC_Cmd.freq[0]>0x21 && PC_Cmd.freq[1]>0x85) || (PC_Cmd.freq[0]>=0x22) ||
                                (PC_Cmd.freq[0]<0x03 && PC_Cmd.freq[1]<0x20) || (PC_Cmd.freq[0]<=0x02) )
                                Trans_openBack.Trans_state[0]=0xFE;		//参数错误
                        }

                        if( GT2000_Tx.Gt2000_mode > 1 )
                        {
                            if( (PC_Cmd.freq[4]>0x21 && PC_Cmd.freq[5]>0x85) || (PC_Cmd.freq[4]>=0x22) ||
                                (PC_Cmd.freq[4]<0x03 && PC_Cmd.freq[5]<0x20) || (PC_Cmd.freq[4]<=0x02) )
                                Trans_openBack.Trans_state[0]=0xFE;		//参数错误
                        }

                        if( GT2000_Tx.Gt2000_mode > 2 )
                        {
                            if( (PC_Cmd.freq[8]>0x21 && PC_Cmd.freq[9]>0x85) || (PC_Cmd.freq[8]>=0x22) ||
                                    (PC_Cmd.freq[8]<0x03 && PC_Cmd.freq[9]<0x20) || (PC_Cmd.freq[8]<=0x02) )
                                Trans_openBack.Trans_state[0]=0xFE;		//参数错误
                        }

                        if( (PC_Cmd.freq[3]!=0x00) || (PC_Cmd.freq[7]!=0x00) || (PC_Cmd.freq[11]!=0x00) )
                            Trans_openBack.Trans_state[0]=0xFE;		//参数错误

                        if(Trans_openBack.Trans_state[0]==0xFE)		//参数错误
                        {
                            Trans_printf(" PC下发参数有误！ ");//会到最后，返回给PC
                        }
                        else	//参数正确
                        {
                            /*-----------------------------获取PC设置的数据，存储到MT2000_Tx-----------------------------------*/
                            switch( PC_Cmd.mode )	//工作种类，固频/双频/三频
                            {
																case 0x01:	GT2000_Tx.Gt2000_mode = 1;	break;  //将PC下发的工作模式存储在GT2000_Tx中
																case 0x02:	GT2000_Tx.Gt2000_mode = 2;	break;
																case 0x03:	GT2000_Tx.Gt2000_mode = 3;	break;
																default:	break;
                            }

                            GT2000_Tx.method = 'F';		//F
														Trans_printf(" PC下发的功率PC_Cmd.power= %d %d %d \n",PC_Cmd.power[0],PC_Cmd.power[1],PC_Cmd.power[2]);
                            Power_Range(&GT2000_Tx,PC_Cmd.power);   //将PC下发的幅度值转化为功率，并存储在GT2000_Tx中
                            Trans_printf(" 获取到的power= %x %x \n",GT2000_Tx.Frequency_power[0],GT2000_Tx.Frequency_power[1]);
                           
														/* 赋值频率 */ 
                            freq_hex_to_str(PC_Cmd.freq, 	GT2000_Tx.Gt2000_freq1);   //在此处将PC下发的频率数据赋值到Tx
                            freq_hex_to_str(PC_Cmd.freq+4,	GT2000_Tx.Gt2000_freq2);    //16进制转化为字符串
                            freq_hex_to_str(PC_Cmd.freq+8,	GT2000_Tx.Gt2000_freq3);
														
//														Trans_printf(" 转化成的str1为freq1= %s \n",GT2000_Tx.Gt2000_freq1);
//														Trans_printf(" 转化成的str2为freq2= %s \n",GT2000_Tx.Gt2000_freq2);
//														Trans_printf(" 转化成的str3为freq3= %s \n",GT2000_Tx.Gt2000_freq3);
														freq_PChex_to_GT2000(PC_Cmd.freq,GT2000_Tx.Gt2000_freq1);  //将字符串形式的数据转化为十六进制赋值给Gt2000_freq1
														freq_PChex_to_GT2000(PC_Cmd.freq+4,GT2000_Tx.Gt2000_freq2);  //将字符串形式的转化为十六进制
														freq_PChex_to_GT2000(PC_Cmd.freq+8,GT2000_Tx.Gt2000_freq3);  //将字符串形式的转化为十六进制
											
														
														Trans_printf(" PC频率转化为发射机频率后GT2000_Tx.Gt2000_freq1= %02x %02x freq2=%02x %02x freq3=%02x %02x \n",GT2000_Tx.Gt2000_freq1[0],\
														GT2000_Tx.Gt2000_freq1[1],GT2000_Tx.Gt2000_freq2[0],GT2000_Tx.Gt2000_freq2[1],GT2000_Tx.Gt2000_freq3[0],GT2000_Tx.Gt2000_freq3[1]);

														
                            if( (System.emission == 0x01) || (Alarm.emission==0x01) || (Alarm.no_respond==0x01) || (Alarm.no_power==0x01) || (Alarm.swr_alarm==0x01) || (Alarm.over_Electric==0x01) )
                            {  //正在发射，无响应，无功率，驻波比报警，过流报警。均返回发射失败
																Trans_openBack.Trans_state[0]=0xFC;		//发射失败
															  Trans_printf(" 正在发射，无响应，无功率，驻波比报警，过流报警。均返回发射失败 ");
                            }
                            else
                            {
                                System.stop = 0x00;
                                System.cancel_sweeping = 0x00;
                                Trans_openBack.Trans_state[0]=0xFD;		//表示接收到了指令，但是需要时间去调频，等待再次查询
                                xTaskNotify(xHandleTask_MT2000,			//发射
                                            BIT_3,             
                                            eSetBits);  //创建事件3通知Task_MT2000.c开发射。
                            }
                        }
                    }

                    Send_PC(g_fuc_codToPC);
                }
                else	if(g_fuc_cod[0]==0x04&&g_fuc_cod[1]==0x01)		//停止发射
                {
                    Time_Printf( (RTC_GetCounter()+g_stamp_distance), &set_time );

                    Trans_stopBack.Transmitte_id[0] = System.CAN_ID[0];
                    g_fuc_codToPC[0]=0x04;
                    g_fuc_codToPC[1]=0x02;

                    //没有正在停止的指令，所以，如果是已经停止了，则直接返回，否则等待停止结果再返回
                    if( System.emission != 0x01 )
                    {
                        Trans_stopBack.Trans_state[0]=0xFC;	//已经是停止状态

                        System.achieve_add_sub = 0;
                        System.modify_power = 0;
                        System.cancel_add_sub = 0;
                        System_Status_Clean();
											Trans_printf(" 停止发射__已经是停止状态 ");
                    }
                    else
                    {
                        System.stop = 0x01;

                        //App_printf("Task_Upper_Computer Stop Notify!\r\n");
                        xTaskNotify(xHandleTask_MT2000,		//停止发射
                                    BIT_0,
                                    eSetBits);

                        Trans_stopBack.Trans_state[0]=0x02;//等待前先置位为停止失败
                        for(k=0; k<40; k++)
                        {
                            if( System.emission==0x00 )	//停止发射，退出等待
                                break;
                            else
                                vTaskDelay(50);
                        }
												Trans_printf(" 停止发射__触发BIT_0 ");
                    }
										
                    Send_PC(g_fuc_codToPC);
                }
                else	if(g_fuc_cod[0]==0x05&&g_fuc_cod[1]==0x01)		//解除报警
                {
                    Time_Printf( (RTC_GetCounter()+g_stamp_distance), &set_time );

                    //App_printf("PC DisAlarm\r\n");
                    DisalarmBack.Transmitte_id[0]=System.CAN_ID[0];
                    g_fuc_codToPC[0]=0x05;
                    g_fuc_codToPC[1]=0x02;

										emission_alarm_content_clean();		//解除报警，则具体的内容清零
					
                    GT2000_Buffer_Clean();
                    memset(DisalarmBack.current_alarm_state, 0, 2);
                    if(get_history_alarm()==0)	//没有报警
                    {
                        DisalarmBack.Disalarm_result[0]=0xFD;
                    }
                    else     					//有报警
                    {
											DisalarmBack.Disalarm_result[0]=0xFE;	//解除报警成功
											
											if(Alarm.emission==1)		//若有功放返回的报警		（待测试）
											{
												Gt2000_Reset();		//复位功放
												Trans_printf(" 功放复位 \n");
											}
											
											clean_all_alarm_flags();				//清除所有报警标志位

											GT2000_Buffer_Clean();					//解除报警，清空bj2000的Buffer
                    }

                    if( System.already_init == 0x00 )			//就算报警解除了，也不一定是待机或者关机状态，有可能是未初始化时报的警
                    {
                        System.status = SYSTEM_UNINITIALIZE;
                    }
                    else	if( System.Voltage > 40.0 )
                    {
                        System.open = 0x02;

                        if( get_history_alarm() == 0x01 )
                        {
                            System.status = SYSTEM_ALARM;
                        }
                        else	if( (System.emission == 0x01) && (System.Forward_Power>100.0) )
                        {
                            if( System.sweep == 0x01 )				//正在扫频(扫频时大报警，会清除正在扫频标志位)
                                System.status = SYSTEM_SCAN;
                            else
                                System.status = SYSTEM_EMISSING;	//如果不是扫频，发射功率还不为0，只能是发射状态
                        }
                        else	if( System.status == SYSTEM_EMISSING )
                        {

                        }
                        else
                        {
                            System.emission = 0x00;   //未发射
                            System.status = SYSTEM_STANDBY;			//查询时，开机了，不一定是待机，有可能是发射/增减功率/扫频等。
                        }
                    }
                    else
                    {
                        System.open = 0x00;

                        if( get_history_alarm() == 0x01 )
                        {
                            System.status = SYSTEM_ALARM;
                        }
                        else
                        {
                            System.status = SYSTEM_SHUTDOWN;
                        }
                    }

                    Alarm.no_respond_locate = 0;
                    Alarm.no_respond_count = 0;
            
                    Send_PC(g_fuc_codToPC);
                }
                else	if(g_fuc_cod[0]==0x06&&g_fuc_cod[1]==0x01)		//报警查询
                {
                    Time_Printf( (RTC_GetCounter()+g_stamp_distance), &set_time );
                    Alarm_historyBack.Transmitte_id[0]=	System.CAN_ID[0];
										Alarm_historyBack.Alarm_reservation[0]=0xFF;
								 	  Alarm_historyBack.Alarm_reservation[1]=0xFF;//赋值预留位位0xff，暂时当分割线用
                    g_fuc_codToPC[0]=0x06;
                    g_fuc_codToPC[1]=0x02;
                    set_alarm_bit();
                    Send_PC(g_fuc_codToPC);

                    //直接返回，不查询激励器
                }
                else	if(g_fuc_cod[0]==0x07&&g_fuc_cod[1]==0x01)		//发射机开机
                {
                    Time_Printf( (RTC_GetCounter()+g_stamp_distance), &set_time );

                    Power_onBack.Transmitte_id[0]=System.CAN_ID[0];
                    g_fuc_codToPC[0]=0x07;
                    g_fuc_codToPC[1]=0x02;

					          if( System.open == 0x02 )			//已经开机，再发开机指令(开机了不一定是待机状态，但是开机指令已经开机了，一定是待机，充分不必要)
                    {
                        Power_onBack.results[0]=0xFC;
                        //App_printf("system open alreadly 1!\n");
                    }
                    else	if( System.open == 0x00 )	//关机了，需要执行开机
                    {
                        //开机命令的时候判断开机结果Power_onBack.results[0]，是因为关机之后需要处理，不能马上开机
                        //(除了硬件接管Task_Hardware_Monitor()和Task_MT2000()的关机操作，只在BJ2000_Open()和BJ2000_Close()修改Power_onBack.results[0]的值)
                        if( Power_onBack.results[0] == 0xFD )
                        {
                            Power_onBack.results[0] = 0x00;						
														System.status = SYSTEM_SHUTDOWN;						
                        }
                        else	if( System.Voltage > 40.0 )
                        {
                            System.open = 0x02;
                            Power_onBack.results[0]=0xFC;
														System.status = SYSTEM_STANDBY;                          
                        }
                        else
                        {
                            System.open = 0x01;
                            System.status = SYSTEM_OPENING;	//更新系统状态为正在开机状态(查询时的判据)
                            Power_onBack.results[0]=0xFE;	//正在开机(激励器应答之后会更新，如果失败则赋值为0xFD)
                            xTaskNotify(xHandleTask_MT2000,	//开机
                                        BIT_8,
                                        eSetBits);
                        }
                    }
                    else	if( System.open == 0x01 )	//正在开机时，又发送开机指令
                    {
                        System.status = SYSTEM_OPENING;	//正在开机状态(肯定是开机状态，再赋值，鲁棒性)
                        Power_onBack.results[0]=0xFE;
                    }

										if( get_history_alarm() == 0x01 )				//报警
										{
											System.status = SYSTEM_ALARM;
										}
						
                    Send_PC(g_fuc_codToPC);
                }
                else	if(g_fuc_cod[0]==0x08&&g_fuc_cod[1]==0x01)		//发射机关机
                {
                    Power_offBack.Transmitte_id[0]=System.CAN_ID[0];
                    g_fuc_codToPC[0]=0x08;
                    g_fuc_codToPC[1]=0x02;

                    if( System.emission == 0x01 )
                    {
                        Power_offBack.results[0]=0xFD;				//需要报错，关机失败
                    }
                    else	if( System.status == SYSTEM_SHUTDOWN )	//已经关机
                    {
                        System.open = 0x00;
												System.Voltage = 0.0;
												System.Electricity = 0.0;
						
                        Power_offBack.results[0]=0xFC;	//已经关机
                        //App_printf("system already closed!\n");
                    }
                    else	//执行关机
                    {
                        System.close = 1;
												
                        //App_printf("Task_Upper_Computer Close Notify!\r\n");
                        xTaskNotify(xHandleTask_MT2000,	//关机
                                    BIT_9,
                                    eSetBits);

                        for(k=0; k<20; k++)
                        {
														 if( System.close == 0 )			//关机成功
														{
															System.status = SYSTEM_SHUTDOWN;
															System.sweep = 0x00;
															
															System.open = 0x00;
															System.Voltage = 0.0;
															System.Electricity = 0.0;
															
															System.achieve_add_sub = 0;
															System.modify_power = 0;
															System.cancel_add_sub = 0;
															System_Status_Clean();
																							break;
														}
                            else
                                vTaskDelay(50);
                        }
                    }
                    Send_PC(g_fuc_codToPC);
                }
                else	if(g_fuc_cod[0]==0x09&&g_fuc_cod[1]==0x01)		//扫频
                {
                    Scan_FrqBack.Transmitte_id[0]	= System.CAN_ID[0];
                    Scan_FrqBack1.Transmitte_id[0]	= System.CAN_ID[0];	//3.2-4.9		
                    Scan_FrqBack2.Transmitte_id[0]	= System.CAN_ID[0];	//5.0-6.9
                    Scan_FrqBack3.Transmitte_id[0]	= System.CAN_ID[0];	//7.0-8.9
                    Scan_FrqBack4.Transmitte_id[0]	= System.CAN_ID[0];	//9.0-10.9
                    Scan_FrqBack5.Transmitte_id[0]	= System.CAN_ID[0];	//11.0-12.9
                    Scan_FrqBack6.Transmitte_id[0]	= System.CAN_ID[0];	//13.0-14.9
                    Scan_FrqBack7.Transmitte_id[0]	= System.CAN_ID[0];	//15.0-16.9
                    Scan_FrqBack8.Transmitte_id[0]	= System.CAN_ID[0];	//17.0-18.9
                    Scan_FrqBack9.Transmitte_id[0]	= System.CAN_ID[0];	//19.0-20.9
                    Scan_FrqBack10.Transmitte_id[0]	= System.CAN_ID[0];	//21.0-22.9
                    Scan_FrqBack11.Transmitte_id[0]	= System.CAN_ID[0];	//23.0-24.9
                    Scan_FrqBack12.Transmitte_id[0]	= System.CAN_ID[0];	//25.0-26.1

                    g_fuc_codToPC[0]=0x09;
                    g_fuc_codToPC[1]=0x02;

                    if(Scan_Frq.state[0]==0xFE)					//开始扫频
                    {
												/*范围扫频赋值*/
												Scan_Begin=(Scan_Frq.Fre_Band[0])*10/10.0;
												Scan_End=(Scan_Frq.Fre_Band[1])*10/10.0;											
												/**************/
                        if( (System.emission == 0x01) || (System.status == SYSTEM_SHUTDOWN) || (System.status == SYSTEM_OPENING)|| \
							(Alarm.emission==0x01) || (Alarm.no_respond==0x01) || (Alarm.no_power==0x01) || (Alarm.swr_alarm==0x01) || (Alarm.over_Electric==0x01) )
                        {
                            //App_printf("Task_MT2000_Sweep waring: working or alarm state!\r\n");

                            Scan_FrqBack.results[0]=0xFD;			//扫频失败
                            System.Scan_Freq_flag=0;
                        }
                        else	//不是正在工作且没有报警
                        {
                            //App_printf("Scan execute!\r\n");

                            if( System.open == 0x02 )				//开机状态
                            {
                                if( System.emission == 0x01 )		//发射状态
                                {
                                    Scan_FrqBack.results[0]=0xFC;	//当前处于发射状态
                                }
                                else	if( System.sweep == 0x01 )	//正在扫频，又下发扫频
                                {
                                    Scan_FrqBack.results[0]=0xFE;	//正在扫频
                                    System.status = SYSTEM_SCAN;
                                }
                                else     //第一次扫频
                                {
                                    System.sweep = 0x01;			//正在扫频
                                    System.modify_power = 0x01;
                                    Scan_FrqBack.results[0]=0xFE;

                                    System.stop = 0x00;
                                    System.cancel_sweeping = 0x00;
                                    System.status = SYSTEM_SCAN;

                                    //App_printf("Task_Upper_Computer Sweep Notify!\r\n");
                                    xTaskNotify(xHandleTask_MT2000,	/* 扫频 */
                                                BIT_5,
                                                eSetBits);
                                }

                                System.Scan_Freq_flag=0;
                            }
                            else	//关机状态，需要开机
                            {
                                Scan_FrqBack.results[0]=0xFF;
                                System.Scan_Freq_flag=0;
                            }
                        }
                    }
                    else if(Scan_Frq.state[0]==0xFD)			//查询最佳频段
                    {
                        //App_printf("Scan inquire!\r\n");

                        if( System.sweep == 0x01 )				//正在扫频
                        {
                            System.status = SYSTEM_SCAN;
                            Scan_FrqBack.results[0]=0xFE;

                            System.Scan_Freq_flag=0;
                        }
                        else	/*	if( System.already_swept == 1 )	//有历史最佳频段	*/		//没有完成扫频也返回	--20181014 By Luonus
                        {
                            if( System.sweep == 0x01 )			//正在扫频，说明需要更新最佳频段
                            {
                                System.status = SYSTEM_SCAN;
                                Scan_FrqBack.results[0]=0xFE;	//正在扫频

                                System.Scan_Freq_flag=0;
                            }
                            else	//直接返回给PC
                            {
                                Scan_FrqBack.results[0]=0xFB;	//已经获取最佳频段
                                if(Scan_Frq.Fre_Band[0]==5)
                                {
                                    System.Scan_Freq_flag=2;
                                    Scan_FrqBack2.results[0]=0xFB;
                                    memcpy(Scan_FrqBack2.Fre_Band,Scan_Frq.Fre_Band,8);
                                }
                                else	if(Scan_Frq.Fre_Band[0]==7)
                                {
                                    System.Scan_Freq_flag=3;
                                    Scan_FrqBack3.results[0]=0xFB;
                                    memcpy(Scan_FrqBack3.Fre_Band,Scan_Frq.Fre_Band,8);
                                }
                                else	if(Scan_Frq.Fre_Band[0]==9)
                                {
                                    System.Scan_Freq_flag=4;
                                    Scan_FrqBack4.results[0]=0xFB;
                                    memcpy(Scan_FrqBack4.Fre_Band,Scan_Frq.Fre_Band,8);
                                }
                                else	if(Scan_Frq.Fre_Band[0]==11)
                                {
                                    System.Scan_Freq_flag=5;
                                    Scan_FrqBack5.results[0]=0xFB;
                                    memcpy(Scan_FrqBack5.Fre_Band,Scan_Frq.Fre_Band,8);
                                }
                                else	if(Scan_Frq.Fre_Band[0]==13)
                                {
                                    System.Scan_Freq_flag=6;
                                    Scan_FrqBack6.results[0]=0xFB;
                                    memcpy(Scan_FrqBack6.Fre_Band,Scan_Frq.Fre_Band,8);
                                }
                                else	if(Scan_Frq.Fre_Band[0]==15)
                                {
                                    System.Scan_Freq_flag=7;
                                    Scan_FrqBack7.results[0]=0xFB;
                                    memcpy(Scan_FrqBack7.Fre_Band,Scan_Frq.Fre_Band,8);
                                }
                                else	if(Scan_Frq.Fre_Band[0]==17)
                                {
                                    System.Scan_Freq_flag=8;
                                    Scan_FrqBack8.results[0]=0xFB;
                                    memcpy(Scan_FrqBack8.Fre_Band,Scan_Frq.Fre_Band,8);
                                }
                                else	if(Scan_Frq.Fre_Band[0]==19)
                                {
                                    System.Scan_Freq_flag=9;
                                    Scan_FrqBack9.results[0]=0xFB;
                                    memcpy(Scan_FrqBack9.Fre_Band,Scan_Frq.Fre_Band,8);
                                }
                                else	if(Scan_Frq.Fre_Band[0]==21)
                                {
                                    System.Scan_Freq_flag=10;
                                    Scan_FrqBack10.results[0]=0xFB;
                                    memcpy(Scan_FrqBack10.Fre_Band,Scan_Frq.Fre_Band,8);
                                }
                                else	if(Scan_Frq.Fre_Band[0]==23)
                                {
                                    System.Scan_Freq_flag=11;
                                    Scan_FrqBack11.results[0]=0xFB;
                                    memcpy(Scan_FrqBack11.Fre_Band,Scan_Frq.Fre_Band,8);
                                }
                                else	if(Scan_Frq.Fre_Band[0]==25)
                                {
                                    System.Scan_Freq_flag=12;
                                    Scan_FrqBack12.results[0]=0xFB;
                                    memcpy(Scan_FrqBack12.Fre_Band,Scan_Frq.Fre_Band,8);
                                }
                            }
                        }
                    }

                    Send_PC(g_fuc_codToPC);		//除了查询，其他都是返回 Scan_FrqBack的数据(根据System.Scan_Freq_flag=0;)
                }
                else	if(g_fuc_cod[0]==0x0A&&g_fuc_cod[1]==0x01)		//停止扫频
                {
                    Sacn_stopBack.Transmitte_id[0]=System.CAN_ID[0];
                    g_fuc_codToPC[0]=0x0A;
                    g_fuc_codToPC[1]=0x02;

                    if(Sacn_stop.state[0]==0XFE)   //停止扫频
                    {
                        if(System.sweep == 0x01)   //正在扫频
                        {
                            System.stop = 0x01;

                            System.cancel_sweeping = 0x01;
                            Sacn_stopBack.result[0]=0xFB;	//正在停止
                        }
                        else     //不是扫频状态
                        {
                            Sacn_stopBack.result[0]=0xFA;	//没有扫频，无需停止
                        }
                    }
                    else	if(Sacn_stop.state[0]==0XFD)	//查询停止扫频是否成功
                    {
                        if(System.sweep == 0x01)			//正在扫频
                        {
													System.stop = 0x01;
                            System.cancel_sweeping = 0x01;
                            Sacn_stopBack.result[0]=0xFB;	//正在停止
                        }
                        else	if( System.sweep != 1 )		//没有扫频
                        {
                            if( (Alarm.emission==0x01) || (Alarm.no_respond==0x01) || (Alarm.no_power==0x01) || (Alarm.swr_alarm==0x01) || (Alarm.over_Electric==0x01) )   //无响应或激励器自身报警
                            {
                                Sacn_stopBack.result[0]=0xFC;//停止失败
                            }
                            else
                            {
                                System.sweep = 0;				//清零
                                Sacn_stopBack.result[0]=0xFE;	//停止成功
                            }
                        }
                    }

                    Send_PC(g_fuc_codToPC);
                }
                else	if(g_fuc_cod[0]==0x0B&&g_fuc_cod[1]==0x01)		//功率增加
                {
                    Add_PowerBack.Transmitte_id[0]=System.CAN_ID[0];
                    g_fuc_codToPC[0]=0x0B;
                    g_fuc_codToPC[1]=0x02;

                    if( (System.open != 0x02) || (System.emission != 0x01) )			//没有开机或者不是发射状态
                    {
                        System.achieve_add_sub = 0;
                        System.modify_power = 0;
                        System.cancel_add_sub = 0;
                        Add_PowerBack.results[0]=0xFC;		//增加功率失败
                    }
                    else	//执行增加功率
                    {
												System.Power_Adjustment=1;
                        if( System.modify_power == 0x01 )
                        {
                            System.cancel_add_sub = 1;
                        }
                        else
                        {
                            System.cancel_add_sub = 0;
                            System.modify_power = 0x01;
                        }

                        if( get_history_alarm() == 0x01 )
                        {
                            System.status = SYSTEM_ALARM;
                        }
                        else
                        {
                            System.status = SYSTEM_EMISSING;
                        }

                        System.stop = 0x00;
                        System.cancel_sweeping = 0x00;
                        Add_PowerBack.results[0]=0xFE;		//增加成功

                        //App_printf("Task_Upper_Computer Add Notify!\r\n");
                        xTaskNotify(xHandleTask_MT2000,		/* 增加功率 */
                                    BIT_1,
                                    eSetBits);
                    }

                    Send_PC(g_fuc_codToPC);
                }
                else	if(g_fuc_cod[0]==0x0C&&g_fuc_cod[1]==0x01)		//功率减小
                {
                    Sub_PowerBack.Transmitte_id[0]=System.CAN_ID[0];
                    g_fuc_codToPC[0]=0x0C;
                    g_fuc_codToPC[1]=0x02;

                    if( (System.open != 0x02) || (System.emission != 0x01) )			//没有开机或者不是发射状态
                    {
                        System.achieve_add_sub = 0;
                        System.modify_power = 0;
                        System.cancel_add_sub = 0;
                        Sub_PowerBack.results[0]=0xFC;		//减小功率失败
                    }
                    else
                    {
												System.Power_Adjustment=1;
                        if( System.modify_power == 0x01 )
                        {
                            System.cancel_add_sub = 1;
                        }
                        else
                        {
                            System.cancel_add_sub = 0;
                            System.modify_power = 0x01;
                        }

                        if( get_history_alarm() == 0x01 )
                        {
                            System.status = SYSTEM_ALARM;
                        }
                        else
                        {
                            System.status = SYSTEM_EMISSING;
                        }

                        System.stop = 0x00;
                        System.cancel_sweeping = 0x00;
                        Sub_PowerBack.results[0]=0xFE;		//减小成功

                        //App_printf("Task_Upper_Computer Sub Notify!\r\n");
                        xTaskNotify(xHandleTask_MT2000,	/* 减小功率 */
                                    BIT_2,
                                    eSetBits);
                    }

                    Send_PC(g_fuc_codToPC);
                }
                else	if(g_fuc_cod[0]==0x0D && g_fuc_cod[1]==0x01)	//运行图
                {
                    Run_DiagramBack.Transmitte_id[0]=System.CAN_ID[0];
                    g_fuc_codToPC[0]=0x0D;
                    g_fuc_codToPC[1]=0x02;

                    if(Run_Diagram.Continue[1]==0xFC)   //运行图发送完毕
                    {
                        if(InternalFlash_SaveData_3(Run_Diagram.Continue[0])==0)		//保存的时候会把地址的首位置位，读取时将读取并赋值为flash_3_once_flag
                        {
                            flash_3_once_flag=0;
                            Run_DiagramBack.results[0]=0xFD;
                        }
                        else
                        {
                            Flash3_to_AcceptAPP();		//读取运行图标志位和数量
                        }
                    }
                    else
                    {
                        flash_3_once_flag=0;
                    }

                    switch(Run_Diagram.Continue[0])
                    {
						case 1:
							if(Flash_Save_Run_Diagram(WRITE_START_ADDR_1,WRITE_END_ADDR_1,Run_Diagram.count[0])==0)
							{
								Run_DiagramBack.results[0]=0xFD;
							}
							else
							{
								Run_DiagramBack.results[0]=0xFE;
							}
							break;
						case 2:
							if(Flash_Save_Run_Diagram(WRITE_START_ADDR_2,WRITE_END_ADDR_2,Run_Diagram.count[0])==0)
							{
								Run_DiagramBack.results[0]=0xFD;
							}
							else
							{
								Run_DiagramBack.results[0]=0xFE;
							}
							break;
						case 3:
							if(Flash_Save_Run_Diagram(WRITE_START_ADDR_3,WRITE_END_ADDR_3,Run_Diagram.count[0])==0)
							{
								Run_DiagramBack.results[0]=0xFD;
							}
							else
							{
								Run_DiagramBack.results[0]=0xFE;
							}
							break;
						case 4:
							if(Flash_Save_Run_Diagram(WRITE_START_ADDR_4,WRITE_END_ADDR_4,Run_Diagram.count[0])==0)
							{
								Run_DiagramBack.results[0]=0xFD;
							}
							else
							{
								Run_DiagramBack.results[0]=0xFE;
							}
							break;
						case 5:
							if(Flash_Save_Run_Diagram(WRITE_START_ADDR_5,WRITE_END_ADDR_5,Run_Diagram.count[0])==0)
							{
								Run_DiagramBack.results[0]=0xFD;
							}
							else
							{
								Run_DiagramBack.results[0]=0xFE;
							}
							break;
						case 6:
							if(Flash_Save_Run_Diagram(WRITE_START_ADDR_6,WRITE_END_ADDR_6,Run_Diagram.count[0])==0)
							{
								Run_DiagramBack.results[0]=0xFD;
							}
							else
							{
								Run_DiagramBack.results[0]=0xFE;
							}
							break;
						case 7:
							if(Flash_Save_Run_Diagram(WRITE_START_ADDR_7,WRITE_END_ADDR_7,Run_Diagram.count[0])==0)
							{
								Run_DiagramBack.results[0]=0xFD;
							}
							else
							{
								Run_DiagramBack.results[0]=0xFE;
							}
							break;
						case 8:
							if(Flash_Save_Run_Diagram(WRITE_START_ADDR_8,WRITE_END_ADDR_8,Run_Diagram.count[0])==0)
							{
								Run_DiagramBack.results[0]=0xFD;
							}
							else
							{
								Run_DiagramBack.results[0]=0xFE;
							}
							break;
						case 9:
							if(Flash_Save_Run_Diagram(WRITE_START_ADDR_9,WRITE_END_ADDR_9,Run_Diagram.count[0])==0)
							{
								Run_DiagramBack.results[0]=0xFD;
							}
							else
							{
								Run_DiagramBack.results[0]=0xFE;
							}
							break;
						case 10:
							if(Flash_Save_Run_Diagram(WRITE_START_ADDR_10,WRITE_END_ADDR_10,Run_Diagram.count[0])==0)
							{
								Run_DiagramBack.results[0]=0xFD;
							}
							else
							{
								Run_DiagramBack.results[0]=0xFE;
							}
							break;
						default:
							Run_DiagramBack.results[0]=0xFD;
							break;
                    }
				
					if(flash_3_once_flag==1)
					{
						if(flash3_Save<=10&&flash3_Save>0)
						{
							judg_read_flash(flash3_Save);		//读运行图到数组Run_Diagram_buf[n]，并清空后面未保存运行图的flash
							//App_printf("receive run diagram success!\r\n");
						}
					}
							
                    Send_PC(g_fuc_codToPC);
                }
            }
            else	if(ret==2)		//数据不完整，不是05 02开头，或不是03 04结尾
            {
                //App_printf("It is not a valid CAN data!\r\n");
            }
            else	if(ret==3)		//解析成功，但是发射机system.CAN_ID错误，返回未初始化
            {
                //App_printf("PC//inquire\r\n");
                g_fuc_codToPC[0]=0x02;
                g_fuc_codToPC[1]=0x02;

                Work_paraBack.Trans_current_state[0] = SYSTEM_UNINITIALIZE;

                //Work_paraBack清零
                Work_paraBack.Mode[0]=0;
                memset(Work_paraBack.Freq,0,12);
                Work_paraBack.Power_45_voltage[0]=0;
                Work_paraBack.Power_45_intensity[0]=0;
                memset(Work_paraBack.Freq,0,12);			//频率先清零，后赋值
                Work_paraBack.Forward_power[0] = 0.0;		//功率先清零，后赋值
                Work_paraBack.Reverse_power[0] = 0.0;
                Work_paraBack.emission[0]=0x00;

                Work_paraBack.Channel[0] = PC_Cmd.channel[0];	//发射的信号		(旧机器有3信道，新机器只有1个信道，兼容，直接原样返回)
                Work_paraBack.Channel[1] = PC_Cmd.channel[1];
                Work_paraBack.Channel[2] = PC_Cmd.channel[2];

                Work_paraBack.Power_grade[0] = PC_Cmd.power[0];	//发射的功率等级	(旧机器有3功率等级，新机器只有1个功率等级，兼容，直接原样返回)
                Work_paraBack.Power_grade[1] = PC_Cmd.power[1];
                Work_paraBack.Power_grade[2] = PC_Cmd.power[2];

                set_alarm_bit();
                Work_paraBack.current_alarm_state[0] = Alarm_historyBack.alarm_history[0];
                Work_paraBack.current_alarm_state[0] = Alarm_historyBack.alarm_history[0];

                Work_paraBack.temp[0]=Temperature_Humidity.Temperature;		//温度
                Work_paraBack.humidity[0]=Temperature_Humidity.Humidity;	//湿度

                Send_PC(g_fuc_codToPC);			//Data_Assemble() 里面的CAN_ID 修改为 Work_paraBack.Trans_current_state[0] --20181031 Luonus

                //App_printf("system.CAN_ID math faiule!\r\n");
            }
            else	if(ret==4)		//没有功能码
            {
                //App_printf("no have fuction code!\r\n!\r\n");
            }
            else	if(ret==5)		//CRC校验错误
            {
                //App_printf("CRC fault!\r\n");
            }
        }
        else
        {
            count=0;
        }

        if( Monitor.hard_control == 0x00 )	//硬件接管单独有查询的任务通知
        {
            if( System.already_init == 0x00 )
            {
                System.status = SYSTEM_UNINITIALIZE;
            }
            else
            {
                task_upper_inquire_counter++;
                if( task_upper_inquire_counter > 10 )
                {
                    //正在开机、调频、扫频、调整功率	不查询
                    if( (System.status==SYSTEM_OPENING) || (System.status==SYSTEM_SCAN) || (System.modify_power==0x01) )
                    {
                        task_upper_inquire_counter = 10;
                    }
                    else
                    {
                        task_upper_inquire_counter = 0;

                        //App_printf("Task_Upper_Computer Inquire(1s) Notify!\r\n");
                        xTaskNotify(xHandleTask_MT2000,		//查询
                                    BIT_10,
                                    eSetBits);
                    }
                }
            }
        }

        vTaskDelay(50);
    }
}

/*转换16bit的字节序 高位与低位互换*/
uint16_t Convert_byte_order_16(uint16_t tni2)
{
	tni2=(((tni2>>8)&0xff) | ((tni2&0xff)<<8));
	return tni2;
}
