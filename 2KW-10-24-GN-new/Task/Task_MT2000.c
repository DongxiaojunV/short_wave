#include "Task_Config.h"
#include "bsp_uart_fifo.h"
#include "my_protocol.h"
#include "alarm.h"
#include "stand_wave_rate.h"
#include "power.h"
#include "math.h"

uint8_t	re_scan_flag = 0x00;

float Scan_Begin;			//范围扫频起始点
float Scan_End;				//范围扫频结束点
float Old_Forward_Power=0;		//用于比较上次的功率变化
uint8_t Power_cataclysm_count=0;
TaskHandle_t xHandleTask_MT2000;

void Task_MT2000(void *pvParameters)
{
    uint32_t ulValue;
    BaseType_t xResult;

    uint8_t i = 0;
    uint8_t j = 0;
    uint8_t repeat_send_counter = 0;		//指令失败，重发多次
    uint32_t current_time_stamp = 0;

		uint8_t		freq_begin;
		uint8_t		freq_end;
		
		uint8_t		next_point_int = 0;
		uint8_t		next_point_dec = 0;
		uint16_t	next_point = 0;
		
		GT2000_t Change_Power_Value;
	
    int ret;
    while(1)
    {
        xResult = xTaskNotifyWait(0x00000000,
                                  0xFFFFFFFF,
                                  &ulValue,
                                  portMAX_DELAY); /* 最大允许延迟时间 */

        if( xResult == pdPASS )
        {
            if((ulValue & BIT_0) != 0)			//停止  Bit0在Task_Upper.c中的查询、停止发射、触发。
            {
								System.Power_Adjustment=1;
							  Trans_printf(" BIT_0_to_GT2000_Stop ! ");
                GT2000_Stop();
							  
								vTaskDelay(2000);
								System.Power_Adjustment=0; //增减功率执行标记 1正在
            }
						
            else	if((ulValue & BIT_1) != 0)	//增加功率		//待测试
            {
							if( get_history_alarm() == 0x01 )
                {
                    System.status = SYSTEM_ALARM;
                }
                else
                {
                    System.status = SYSTEM_EMISSING;
                }

                System.modify_power = 0x01;
                if(System.mode==1)		//如果是单频
								{
									  power_add(GT2000_Rx.Frequency_power);  //功率微调增，，一次增加+65W
										Change_Power_Value.Frequency_power[0]=GT2000_Rx.Frequency_power[0];												
								    Change_Power_Value.Frequency_power[1]=GT2000_Rx.Frequency_power[1];										
								}
								
								else if(System.mode==2)
								{							
									  power_add(GT2000_Rx.Frequency_power);  //功率微调增，，一次增加+65W
										Change_Power_Value.Frequency_power[0]=GT2000_Rx.Frequency_power[0];												
								    Change_Power_Value.Frequency_power[1]=GT2000_Rx.Frequency_power[1];		
								}
								
								else if(System.mode==3)
								{
									  power_add(GT2000_Rx.Frequency_power);  //功率微调增，，一次增加+65W
										Change_Power_Value.Frequency_power[0]=GT2000_Rx.Frequency_power[0];												
								    Change_Power_Value.Frequency_power[1]=GT2000_Rx.Frequency_power[1];		
								}
								
								memcpy(Change_Power_Value.Gt2000_freq1,GT2000_Rx.Gt2000_freq1,2);
								memcpy(Change_Power_Value.Gt2000_freq2,GT2000_Rx.Gt2000_freq2,2);
								memcpy(Change_Power_Value.Gt2000_freq3,GT2000_Rx.Gt2000_freq3,2);
								
								Change_Power_Value.Gt2000_mode=System.INQUIRE_Mode;   //从机器上查到的工作模式
//								Power_Range(&Change_Power_Value,Change_Power_Value.Frequency_power);		//转换为幅度值数据
								
				Gt_RF_Close();		//关发射
				vTaskDelay(50);
								
								GT_Set_Value(Change_Power_Value);	 //设置工作参数，前面赋值
								Trans_printf(" 2由此进入设置工作模式和频率22 ");
								vTaskDelay(500);											
					Gt_RF_Open();
					Trans_printf(" 开发射命令已下发22 ");
								
								if(System.status!=SYSTEM_SCAN)		//如果不是扫频，直接更新状态
								{
									System.status=SYSTEM_EMISSING;
								}
								vTaskDelay(100);
								Gt_Inquire_All();		//查询一次，看有无报警
								ret=GT2000_Wait_Ack(MT2000_CMD_INQUIRE);							
								if( ret == MT2000_ACK_ALARM )
								{
										GT2000_Alarm_Stop();
								}
								else	if( ret == MT2000_ACK_OK )
								{
										GT2000_Buffer_Clean();
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
								}
								//发射成功之后，System.emission=0x01; 有些标志位未置位，导致Task_Hardware_Monitor()会再次判断为需要发射，出现警告

								/* 发射成功之后，不会马上更新数据，在这里等待数据更新(功率会更新的比较慢) */
						for(i=0; i<100; i++)
						{
							Gt_Inquire_All();
							vTaskDelay(50);
							ret=GT2000_Wait_Ack(MT2000_CMD_INQUIRE);
							if( ret == MT2000_ACK_ALARM )
							{
									GT2000_Alarm_Stop();
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

								vTaskDelay(100);
						}
	
						if( ret != MT2000_ACK_OK )
						{
								Alarm.no_respond_locate = 12;
						}
					
						if( (System.Forward_Power <= 10.0) && (System.Electricity >= 10.0) )	//无功率报警
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
									Trans_printf(" 增加功率检测到无功率报警2！ \n");
								}
						}
						//发射成功，检查报警
						swr_power_alarm_check();	//驻波比检测

						if( Alarm.emission == 0x01 )	//激励器自身报警，立即停止
						{
								GT2000_Alarm_Stop();
						}
								
						if( (Alarm.emission==0x01) || (Alarm.no_respond==0x01) || (Alarm.no_power==0x01) || (Alarm.swr_alarm==0x01) || (Alarm.over_Electric==0x01) )	//激励器自身报警，无应答报警、无功率报警
						{
								System.status = SYSTEM_ALARM;
						}
						else	if( (System.cancel_sweeping==0x01) || (System.stop==0x01) )			//取消扫频，停止后返回
						{
								if( get_history_alarm() == 0x01 )
								{
										System.status = SYSTEM_ALARM;
								}
								else
								{
										System.status = SYSTEM_STANDBY;
								}
						}

                System.modify_power = 0x00;
								
								vTaskDelay(3000);		//等待功率稳定
						
								System.Power_Adjustment=0;
            }
						
            else 	if((ulValue & BIT_2) != 0)	//减小功率		
            {
                if( get_history_alarm() == 0x01 )
                {
                    System.status = SYSTEM_ALARM;
                }
                else
                {
                    System.status = SYSTEM_EMISSING;
                }

                System.modify_power = 0x01;
                if(System.mode==1)		//如果是单频
								{
									if(GT2000_Rx.Now_all_power>0x41)			//当前功率至少65W
									{
										power_sub(GT2000_Rx.Frequency_power);  //功率微调减，，一次减小-65W
										Change_Power_Value.Frequency_power[0]=GT2000_Rx.Frequency_power[0];												
								    Change_Power_Value.Frequency_power[1]=GT2000_Rx.Frequency_power[1];	
									}
									else
									{
										Sub_PowerBack.results[0]=0xFC;				//失败
									}
								}
								
								else if(System.mode==2)
								{
									if(GT2000_Rx.Now_all_power>0x41)			//当前功率至少65W
									{
										power_sub(GT2000_Rx.Frequency_power);  //功率微调减，，一次减小-65W
										Change_Power_Value.Frequency_power[0]=GT2000_Rx.Frequency_power[0];												
								    Change_Power_Value.Frequency_power[1]=GT2000_Rx.Frequency_power[1];	
									}
									else
									{
										Sub_PowerBack.results[0]=0xFC;				//失败
									}
								}
								
								else if(System.mode==3)
								{
									if(GT2000_Rx.Now_all_power>0x41)			//当前功率至少65W
									{
										power_sub(GT2000_Rx.Frequency_power);  //功率微调减，，一次减小-65W
										Change_Power_Value.Frequency_power[0]=GT2000_Rx.Frequency_power[0];												
								    Change_Power_Value.Frequency_power[1]=GT2000_Rx.Frequency_power[1];	
									}
									else
									{
										Sub_PowerBack.results[0]=0xFC;				//失败
									}
								}
								
								memcpy(Change_Power_Value.Gt2000_freq1,GT2000_Rx.Gt2000_freq1,8);
								memcpy(Change_Power_Value.Gt2000_freq2,GT2000_Rx.Gt2000_freq2,8);
								memcpy(Change_Power_Value.Gt2000_freq3,GT2000_Rx.Gt2000_freq3,8);
								
								Change_Power_Value.Gt2000_mode=System.INQUIRE_Mode;   //从机器上查到的工作模式
//								Power_Range(&Change_Power_Value,Change_Power_Value.power);		//转换为幅度值数据
								
								Gt_RF_Close();		//关发射
								vTaskDelay(50);
								
								GT_Set_Value(Change_Power_Value);	//设置发射机参数
								Trans_printf(" 3由此进入设置工作模式和频率设置 ");
								vTaskDelay(50);
								Gt2000_Open(); //开机
								Trans_printf(" 开机--等待30s--再开发射 ");
								vTaskDelay(30000);				//等待30s									
								Gt_RF_Open();  //开发射Gt_RF_Open
								Trans_printf(" 开发射命令已下发 ");
								
								
								if(System.status!=SYSTEM_SCAN)		//如果不是扫频，直接更新状态
								{
									System.status=SYSTEM_EMISSING;
								}
								vTaskDelay(100);
								Gt_Inquire_All();		//查询一次，看有无报警
								ret=GT2000_Wait_Ack(MT2000_CMD_INQUIRE);						
								if( ret == MT2000_ACK_ALARM )
								{
										GT2000_Alarm_Stop();
								}
								else	if( ret == MT2000_ACK_OK )
								{
										GT2000_Buffer_Clean();
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
								}
								//发射成功之后，System.emission=0x01; 有些标志位未置位，导致Task_Hardware_Monitor()会再次判断为需要发射，出现警告

								/* 发射成功之后，不会马上更新数据，在这里等待数据更新(功率会更新的比较慢) */
						for(i=0; i<100; i++)
						{
							Gt_Inquire_All();
							vTaskDelay(50);
							ret=GT2000_Wait_Ack(MT2000_CMD_INQUIRE);
							if( ret == MT2000_ACK_ALARM )
							{
									GT2000_Alarm_Stop();
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

								vTaskDelay(100);
						}
	
						if( ret != MT2000_ACK_OK )
						{
								Alarm.no_respond_locate = 12;
						}
					
						if( (System.Forward_Power <= 10.0) && (System.Electricity >= 10.0) )	//无功率报警
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
									Trans_printf(" 检测到无功率报警3！ \n");
								}
						}
						//发射成功，检查报警
						swr_power_alarm_check();	//驻波比检测

						if( Alarm.emission == 0x01 )	//激励器自身报警，立即停止
						{
								GT2000_Alarm_Stop();
						}
								
						if( (Alarm.emission==0x01) || (Alarm.no_respond==0x01) || (Alarm.no_power==0x01) || (Alarm.swr_alarm==0x01) || (Alarm.over_Electric==0x01) )	//激励器自身报警，无应答报警、无功率报警
						{
								System.status = SYSTEM_ALARM;
						}
						else	if( (System.cancel_sweeping==0x01) || (System.stop==0x01) )			//取消扫频，停止后返回
						{
								if( get_history_alarm() == 0x01 )
								{
										System.status = SYSTEM_ALARM;
								}
								else
								{
										System.status = SYSTEM_STANDBY;
								}
						}

                System.modify_power = 0x00;
						
								vTaskDelay(3000);		//等待功率稳定
						
								System.Power_Adjustment=0;
            }
						
            else 	if((ulValue & BIT_3) != 0)	//发射
            {
                //App_printf("先查询一下是不是停止状态\r\n");
                     //重复发送计数器
                for(repeat_send_counter=0; repeat_send_counter<NO_RESPOND_MAX; repeat_send_counter++)
                {
									
											Gt_Inquire_All();		//发射机状态查询								
											ret=GT2000_Wait_Ack(MT2000_CMD_INQUIRE);
											if( (ret == MT2000_ACK_OK) || (ret == MT2000_ACK_ALARM) )
													break;

                }
								Trans_printf(" 重复发送先查询一下是不是停止状态 \n");
                if( ret == MT2000_ACK_OK )   //成功查询了，才继续下面的，否则就是报警状态
                {
                    if( GT2000_Rx.sys_open == 0 )
                    {
                        System.open = 0x00;  //开机标志位	0:关机	1:正在开机	2:已经开机
											  Trans_printf(" 发射机未开机，关机 \n");
                    }
                    else	if( (System.emission == 0x01) || (Alarm.emission==0x01) || (Alarm.no_respond==0x01) ||
                                (Alarm.no_power==0x01) || (Alarm.swr_alarm==0x01) || (Alarm.over_Electric==0x01) )
                    {
                            //正在发射，无响应，无功率，驻波比报警，过流报警。不执行操作
											 Trans_printf(" 正在发射，无响应，无功率，驻波比报警，过流报警。不执行操作 \n");
                    }
                    else	//执行发射
                    {
											if(System.Launch_Switch_state!=1)		//查询结果不是发射状态
											{
                        GT2000_Emit(); //开发射
												vTaskDelay(10000);   //等待
												Trans_printf(" 3_T0_GT2000_Emit() \n");
												Trans_printf(" 查询结果不是发射状态，开发射 \n");

                        if( (Alarm.emission==0x01) || (Alarm.no_respond==0x01) || (Alarm.no_power==0x01) || (Alarm.swr_alarm==0x01) )	//激励器自身报警，无应答报警、无功率报警
                        {
                            GT2000_Stop();
													Trans_printf(" 已发送开发射命令，但有紧急报警，停止！ \n");
                        }

                        if( System.emission == 0x01 )
                        {
                            if( Monitor.hard_control == 0x00 )
                            {
                                System.status = SYSTEM_EMISSING;		//不是硬件接管，而且扫频不会在这，所以直接返回发射状态
															  Trans_printf(" 不是硬件接管，而且扫频不会在这，所以直接返回发射状态 \n");
                            }                           
                                if( get_history_alarm() == 0x01 )
                                {
                                    System.status = SYSTEM_ALARM;
																	Trans_printf(" 开发射途中，有历史报警，返回报警状态 \n");
                                }
                                else
                                {
                                    System.status = SYSTEM_EMISSING;
																	Trans_printf(" 开发射成功！返回发射状态 \n");
                                }
                            							
														if( Monitor.hard_control == 0x01 )
														{
															Monitor.need_emit = 0x00; //查询运行图时，模式、频率或功率等级不一致：1需要发射 0不需要
															Trans_printf(" Monitor.hard_control=1，不需要发射 \n");
														}
                        }
                        else
                        {
                            System.sweep = 0;
                            System.stop = 0;
                            System.achieve_add_sub = 0;
                            System.modify_power = 0;
                            System.cancel_add_sub = 0;
                            System_Status_Clean();

														System.status = SYSTEM_STANDBY;
													Trans_printf(" System.emission！=1，待机状态 \n");
                 
                        }
						
											if( get_history_alarm() == 0x01 )
											{
												System.status = SYSTEM_ALARM;
												Trans_printf(" 执行发射过程中有历史报警！ \n");
											}
										}
                    }
                }
                else	//查询失败
                {
                    Alarm.no_respond_locate = 16;
                    //App_printf("Task_MT2000 Emit Inquire 16 no respond.\r\n");
									Trans_printf(" System.emission！=1，待机状态 \n");
                }
							Trans_printf(" 发射事件BIT_3执行完毕 \n");
            }
						
            else 	if((ulValue & BIT_4) != 0)	//保留
            {

            }
            else 	if((ulValue & BIT_5) != 0)	//扫频
            {
				
				/*--------------------------------扫频前先清除过期数据-----------------------------------*/
                for(i=0; (i<10)&&((System.cancel_sweeping == 0x00) && (System.stop == 0x00) ); i++)		
				{                          //取消扫频		1:取消扫频               //紧急停止标志位	1:紧急停止(正在增减功率，但是已经过了工作时间，需要紧急停止)	通知Task_MT2000(),BJ2000_Stop();MT2000_Cmd_Stop();才置位
					current_time_stamp = RTC_GetCounter() + g_stamp_distance;
					
					next_point = Scan_Breakpoint_Read();
					next_point_int = next_point/10;    //整数部分
					next_point_dec = next_point%10;    //小数部分
					     //存储扫频结束的时间戳
					if( (freq_band_time_stamp[i] != 0) && (freq_band_time_stamp[i] != 0xFFFFFFFF) &&	//时间戳数据是有效数据(不一定没过期)
						(current_time_stamp < (freq_band_time_stamp[i] + FREQ_BAND_UPDATE_TIME) ) )		//数据有效，未过期
          {
						if( (next_point == 50) && 						//next point is 5.0MHz
							(freq_band_time_stamp[9] != 0) && 			//.9 is finish
							(freq_band_time_stamp[9] != 0xFFFFFFFF) ) //存储扫频结束的时间戳
						{						
							re_scan_flag = 0x01;  //1:存在有效的扫描数据，扫描按钮已被激活
							break;
						}
						else	if( next_point_dec > i )		//已经扫过的，跳过
						{
							re_scan_flag = 0x00;
							continue;
						}
						else	//部分数据过期，需要重新扫频，先清零(停止扫频之后会保存时间戳，数据不会过期，不会清零)
						{
							re_scan_flag = 0x00;
							freq_band_time_stamp[i] = 0x00; //清零存储扫频结束的时间戳

							if( (i==0) || (i==1) )
							{
								for( j=next_point_int; j<=26; j++)
								{
									Forward_Power_array[ (j-3)*10 + i ] = 0.0;  //从3Mhz开始保存，清零[0]~[23.9]
									Reverse_Power_array[ (j-3)*10 + i ] = 0.0;
									SWR_array[ (j-3)*10 + i ] = 0.0;
								}
							}
							else
							{
								for( j=next_point_int; j<=25; j++)
								{
									Forward_Power_array[ (j-3)*10 + i ] = 0.0;  //从3.2Mhz开始保存，清零[0]~[22.9]
									Reverse_Power_array[ (j-3)*10 + i ] = 0.0;
									SWR_array[ (j-3)*10 + i ] = 0.0;
								}
							}
						}
					}
					else	//该段数据全部无效或者过期，全部清零
					{
						freq_band_time_stamp[i] = 0x00;

						if( (i==0) || (i==1) )
						{
							for(j=5; j<=26; j++)
							{
								Forward_Power_array[ (j-3)*10 + i ] = 0.0;
								Reverse_Power_array[ (j-3)*10 + i ] = 0.0;
								SWR_array[ (j-3)*10 + i ] = 0.0;
							}
						}
						else
						{
							for(j=5; j<=25; j++)
							{
								Forward_Power_array[ (j-3)*10 + i ] = 0.0;
								Reverse_Power_array[ (j-3)*10 + i ] = 0.0;
								SWR_array[ (j-3)*10 + i ] = 0.0;
							}
						}
					}
				}
				
				if(InternalFlash_SaveData_2()==1)   //将正向功率和驻波比保存到flash   1:成功
				{
					//App_printf("flash save successed!\r\n");
				}
				else
				{
					//App_printf("flash save error!\r\n");
				}
				
                Flash2_to_AcceptAPP();		//读取  //读取flash内的正向功率和驻波比到Scan_FrqBackx
				
				if(Scan_Begin==0||Scan_End==0)
				{
	/*---------------------------------------开始扫频----------------------------------------*/
					for(int num=0; (num<10)&&((System.cancel_sweeping == 0x00) && (System.stop == 0x00) ); num++)		
					{
							current_time_stamp = RTC_GetCounter() + g_stamp_distance;

							if( re_scan_flag == 0x01 )
							{
								
							}
							else	if( (freq_band_time_stamp[num] != 0) && (freq_band_time_stamp[num] != 0xFFFFFFFF) )		//有效数据(过期数据之前已经清零了)
							{
								continue;  //跳过本次循环体中余下尚未执行的语句，立即进行下一次的循环条件判定，结束本次循环。
							}
							//甘肃短波设备工资范围5.9-21.8
					switch(num)
					{
						case 0:		freq_begin=6;		freq_end=21;		break;
						case 1:		freq_begin=6;		freq_end=21;		break;
						case 2:		freq_begin=6;		freq_end=21;		break;
						case 3:		freq_begin=6;		freq_end=21;		break;
						case 4:		freq_begin=6;		freq_end=21;		break;
						case 5:		freq_begin=6;		freq_end=21;		break;
						case 6:		freq_begin=6;		freq_end=21;		break;
						case 7:		freq_begin=6;		freq_end=21;		break;
						case 8:		freq_begin=6;		freq_end=21;		break;
						case 9:		freq_begin=5;		freq_end=20;		break;
					}

                    /*------------------------------------------扫频------------------------------------------*/

                    Band_scan(freq_begin, freq_end, num);		
					
					//退出扫频之后就保存，不管退出原因是什么
					freq_band_time_stamp[num] = RTC_GetCounter() + g_stamp_distance;
                    if(InternalFlash_SaveData_2()==1)   //将正向功率和驻波比保存到flash   1:成功
                    {
                        Flash2_to_AcceptAPP();   //读取flash内的正向功率和驻波比到Scan_FrqBackx
                        //App_printf("flash save successed!\r\n");
                    }
                    else
                    {
                        //App_printf("flash save error!\r\n");
                        System.sweep = 0;
                    }
					
                    //无应答报警、无功率报警(驻波比和电流报警，不处理)		--20190415 Luonus
                    if( (Alarm.emission==0x01) || (Alarm.no_respond==0x01) || (Alarm.no_power==0x01) )
                    {
											Trans_printf(" scan1扫频、无应答报警、无功率报警！不处理退出循环 \n");
                        break;  //退出循环
                    }
                    else	if( (System.cancel_sweeping == 0x01) || (System.stop == 0x01)  )		//取消扫频
                    {
											Trans_printf(" scan1扫频、取消扫频！不处理 \n");
                        break;
                    }
                    else	if( System.Voltage < 40.0 )				//关机了
                    {
											Trans_printf(" scan1扫频System.Voltage < 40.0，关机了 \n");
                        break;
                    }

					if( num == 9 )	
					{
						System.already_swept = 1;    //扫频完成标志		//断点扫描之后，该标志位已经无意义
                        System.sweep = 0x02;  //扫频标志位	1:正在扫频，2:扫频完成，更新status标志位，并将sweep清零		最后，保存在flash
						
						Scan_Breakpoint_Save(5, 9);					//扫频完了，下次扫频是重新扫频  //获取扫频断点5.9
						break;
					}

              }    //for循环结束
					}	  //Scan_Begin！=0 && Scan_End！=0
				else
				{
					uint8_t Scan_start,Scan_end;				//不能超过255
					uint8_t clean_int,clean_dec;
					Scan_start=Scan_Begin*10;			//扩大为整数
					Scan_end=Scan_End*10;
				/*---------------------------------------开始扫频2----------------------------------------*/
												
				/*--------------------------------扫频前先清除以往数据-----------------------------------*/
        if((System.cancel_sweeping == 0x00) && (System.stop == 0x00))  //未取消扫频，未急停		1:取消扫频	1:紧急停止
				{		
						for(j=Scan_start; j<=Scan_end; j++)
						{
							clean_int=j/10;
							clean_dec=j%10;
							Forward_Power_array[ (clean_int-3)*10 + clean_dec ] = 0.0;
							Reverse_Power_array[ (clean_int-3)*10 + clean_dec ] = 0.0;
							SWR_array[ (clean_int-3)*10 + clean_dec ] = 0.0;
						}
				}
                 for(int count=Scan_start;(count<=Scan_end)&&(System.cancel_sweeping == 0x00)&&(System.stop == 0x00);count++)
								 {									 
										Trans_printf(" 进入分段扫频函数2！count= %d \n",count);   //为什么I会=0
									 Optional_Band_scan(count,Scan_end);  //分段扫频函数2
								    
									
										//退出扫频之后就保存，不管退出原因是什么
                    if(InternalFlash_SaveData_2()==1) //将正向功率和驻波比保存到flash   1:成功
                    {
                        Flash2_to_AcceptAPP();   //读取flash内的正向功率和驻波比到Scan_FrqBackx
                        //App_printf("flash save successed!\r\n");
                    }
                    else
                    {
                        //App_printf("flash save error!\r\n");
                        System.sweep = 0;
                    }
					
                    //无应答报警、无功率报警(驻波比和电流报警，不处理)		--20190415 Luonus
                    if( (Alarm.emission==0x01) || (Alarm.no_respond==0x01) || (Alarm.no_power==0x01) )
                    {
											Trans_printf(" 扫频、无应答报警、无功率报警！不处理 \n");
                        break;  //退出循环
                    }
                    else	if( (System.cancel_sweeping == 0x01) || (System.stop == 0x01)  )		//取消扫频
                    {
											Trans_printf(" 扫频、取消扫频！不处理 \n");
                        break;
                    }
                    else	if( System.Voltage < 40.0 )				//关机了
                    {
											Trans_printf(" System.Voltage < 40.0，关机了 \n");
                        break;
                    }


										if( i ==  Scan_end)	
										{
											System.already_swept = 1;   //扫频完成标志		//断点扫描之后，该标志位已经无意义
																	System.sweep = 0x02;  //扫频标志位	1:正在扫频，2:扫频完成，更新status标志位，并将sweep清零		最后，保存在flash
											
											Trans_printf(" 已经扫频完了，下次扫频是重新扫频 \n");
											Scan_Breakpoint_Save(5, 9);					//扫频完了，下次扫频是重新扫频
											break;
										}
							}   //for结束
						}	
				
						re_scan_flag = 0x00;   //开扫频标志位，停止扫频
                if( (Alarm.emission==0x01) || (Alarm.no_respond==0x01) || (Alarm.no_power==0x01) )		//报警退出		--20190415 Luonus
                {
									Trans_printf(" 扫频报警退出 \n");
                    System.sweep = 0x00;
                    Scan_FrqBack.results[0] = 0xFD;		//扫频失败
									
				            Gt_RF_Close();		
										vTaskDelay(3000);  //关发射3s后关机
										Gt2000_Close();
										vTaskDelay(1500);   //关机
										ret=MT2000_ACK_OK;


                    if( ret == MT2000_ACK_OK )	//停止成功
                    {
                        System.stop = 0x00;

                        System.achieve_add_sub = 0x00;
                        System.modify_power = 0x00;
                        System.cancel_add_sub = 0x00;
                        System_Status_Clean();

                        System.sweep = 0x00;
                        System.already_swept = 0x00;
                    }
                    else
                    {
                        Alarm.no_respond_locate = 17;
                        //App_printf("Task_MT2000 Sweep stop 17 no respond.\r\n");
                    }
                }
                else	if( (System.cancel_sweeping == 0x01) || (System.stop == 0x01)  )		//如果上面的Band_scan是由于取消扫频退出的
                {       //扫频标志位	1:正在扫频，2:扫频完成
                    Trans_printf(" 如果上面的Band_scan是由于取消扫频退出的、急停退出的 \n");
									if( System.sweep == 0x01 )			//上面的扫频由于强制停止中止了，需要将发射机先停止以保障安全
                    {
											
				            Gt_RF_Close();		
											vTaskDelay(3000);
											Gt2000_Close();
											vTaskDelay(1500);
											ret=MT2000_ACK_OK;

                        if( ret == MT2000_ACK_OK )	//停止成功 ，，默认成功
                        {
                            System.sweep = 0;
                            System.stop = 0;
                            System.achieve_add_sub = 0;
                            System.modify_power = 0;
                            System.cancel_add_sub = 0;
                            System_Status_Clean();

                            if( get_history_alarm() == 0x01 )
                            {
                                System.status = SYSTEM_ALARM;
                            }
                            else
                            {
                                System.status = SYSTEM_STANDBY;
                            }

                            System.already_swept = 0;
                            System.cancel_sweeping = 0;
                            Trans_stopBack.Trans_state[0]=0xFE;		//停止成功
                        }
                        else     //不成功，返回停止失败，且处于报警状态
                        {
                            Trans_stopBack.Trans_state[0]=0x02;//停止失败
//							System.cancel_sweeping = 1;

                            Alarm.no_respond_locate = 19;
                            //App_printf("Task_MT2000 Sweep stop 19 no respond.\r\n");
                        }
                    }
                    else     //取消扫频的时候，正好已经完成扫频，清除标志
                    {
                        System.achieve_add_sub = 0;
                        System.modify_power = 0;
                        System.cancel_add_sub = 0;
                        System_Status_Clean();

                        if( get_history_alarm() == 0x01 )
                        {
                            System.status = SYSTEM_ALARM;
                        }
                        else
                        {
                            System.status = SYSTEM_STANDBY;
                        }

                        System.sweep = 0x02;
                        System.already_swept = 1;
                        System.cancel_sweeping = 0;				//取消扫频标志清零
                        Trans_stopBack.Trans_state[0]=0xFE;		//停止成功
                    }

                    System.sweep = 0x00;
                }
                else	if( System.Voltage < 40.0 )   //关机状态
                {
                    System.achieve_add_sub = 0;
                    System.modify_power = 0;
                    System.cancel_add_sub = 0;
                    System_Status_Clean();

                    System.status = SYSTEM_SHUTDOWN;
                    System.open = 0x00;
                    System.sweep = 0x00;
                    System.already_swept = 0;
									Trans_printf(" 扫频处于关机状态 \n");
                }
            }
            else 	if((ulValue & BIT_6) != 0)	//保留
            {

            }
            else 	if((ulValue & BIT_7) != 0)	//保留
            {

            }
            else 	if((ulValue & BIT_8) != 0)	//开机
            {
                GT2000_Open();
							  Trans_printf(" Task_MT2000中已触发开机事件 \n");
            }
            else 	if((ulValue & BIT_9) != 0)	//关机
            {
								System.Power_Adjustment=1;
							
                GT2000_Close();
								Power_offBack.results[0]= 0xFE;	//关机成功
								Power_onBack.results[0]	= 0xFD;	//开机失败						

								System.close = 0;		//关机成功
								System_Status_Clean();
								System.Voltage = 0.0;
								System.Electricity = 0.0;
								System.achieve_add_sub = 0;
								System.modify_power = 0;
								System.cancel_add_sub = 0;
								System.sweep = 0;
								System.open = 0;

								if( get_history_alarm() == 0x01 )
								{
										System.status = SYSTEM_ALARM;
								}
								else
								{
										System.status = SYSTEM_SHUTDOWN;
								}
                                 

                //如果关机之后马上开机，因为需要等待关机完成，会返回开机失败，所以 if(Power_onBack.results[0]==0xFD) 表示正在关机，清零则表示关机完成
                Power_onBack.results[0] = 0x00;			//关机完成，可以开机了(不管关机有没有成功)
					
								vTaskDelay(2000);
								System.Power_Adjustment=0;

            }
						
            else 	if((ulValue & BIT_10) != 0)	//查询
            {						
								float power_change=0;
								for(repeat_send_counter=0; repeat_send_counter<NO_RESPOND_MAX; repeat_send_counter++)
								{
										Gt_Inquire_All();		//发射机状态查询
										ret=GT2000_Wait_Ack(MT2000_CMD_INQUIRE);
										if( (ret == MT2000_ACK_OK) || (ret == MT2000_ACK_ALARM))
												break;
								}
                
                if( ret == MT2000_ACK_ALARM )	//激励器报警
                {
                    GT2000_Alarm_Stop();
									Trans_printf(" 触发激励器报警！急停！ ");
                }
                else	if( ret == MT2000_ACK_OK )
                {
										if(Old_Forward_Power!=0)															//增加功率骤变监测
										{
											if(System.status==SYSTEM_EMISSING&&System.Power_Adjustment!=1&&System.stop==0&&System.close==0)
											{	
												power_change=fabs(Old_Forward_Power-System.Forward_Power);
												if(System.mode==1)		//如果是单频			//待测试
												{
													if(power_change>=POWER_CHANGE_MAX)
													{
														Power_cataclysm_count++;
														if(Power_cataclysm_count>=3)
														{
															Alarm.power_cataclysm=1;
															Alarm.alarm_history = 1;
															GT2000_Alarm_Stop();
															System.status=SYSTEM_ALARM;	
															Power_cataclysm_count=0;		
															Trans_printf(" 单频时功率骤变大于350！警告！ ");
														}															
													}
													else
													{
														Power_cataclysm_count=0;
													}
												}
												else if(System.mode==2||System.mode==3)		//多频时//待测试
												{
													if(System.Forward_Power<=10)
													{
														Alarm.power_cataclysm=1;
														Alarm.alarm_history = 1;
														GT2000_Alarm_Stop();
														System.status=SYSTEM_ALARM;
														Trans_printf(" 多频时功率骤变大于350！警告！ ");
													}
												}
											}
										}
										
										Old_Forward_Power=System.Forward_Power;
									
                    if( System.already_init == 0x00 )
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
													if(System.Power_Adjustment!=1)
													{
														System.emission = 0x00;
                            System.status = SYSTEM_STANDBY;			//查询时，开机了，不一定是待机，有可能是发射/增减功率/扫频等。
													}
                        }
                    }
                    else
                    {
                        if( (System.status == SYSTEM_OPENING) || (System.open == 0x01) )
                        {

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
                    }
                }
								else if(ret==MT2000_NO_ACK)		//无应答报警-------2021/4/9
								{
									if(System.status!=SYSTEM_UNINITIALIZE)		//未初始化时，不处理
									{
										System.status=SYSTEM_ALARM;	
										Trans_printf(" Task_MT2000查询事件无响应！报警！ ");
									}										
								}

                if( System.emission == 0x01 )
                {
                    System.open = 0x02;
                }
            }
            else								//没有对应事件
            {
                //App_printf("没有对应的事件!!!\r\n");
            }
        }
        else									//等待超时
        {
            //App_printf("Task_MT2000 TIME OUT!!!\r\n");
        }

        vTaskDelay(100);
    }
}
