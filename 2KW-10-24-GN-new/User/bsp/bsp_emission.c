#include "includes.h"

#include "new_protocol.h"

float z[256];//正
uint8_t int_z[256];//正向功率的除以10int值
float f[256];//反
float begain;
float end;
/*
 *外部声明
 */
extern EventGroupHandle_t xCreatedEventGroup;
extern TaskHandle_t xHandleTaskReceiveFormPC;
extern TaskHandle_t xHandleTaskSendToPC;
extern TaskHandle_t xHandleTaskUserIF;
extern uint8_t g_fuc_cod[2];//全局功能码
extern uint8_t g_fuc_codToPC[2];//返回给PC的功能码
// ADC1转换的电压值通过MDA方式传到SRAM
extern __IO uint16_t ADC_ConvertedValue[NOFCHANEL];
void set_alarm_bit(void);
/*
 * 本文件函数声明
 */
static int Is_available_data(const uint8_t * Msg);
static int MT2000_sendCommand(const uint8_t *SendCommand,const int Len,TickType_t xTicksToWait);
static int run_command(TickType_t xTicksToWait);
static int Band_scan(TickType_t xTicksToWait,int index);
static int get_better_SWR(float *array,int index);
//static int judge_fre_segment(void);

volatile uint8_t clean_sweeping;
volatile uint8_t clean_gain;
volatile uint8_t clean_reduction;
volatile uint8_t clean_FM;

uint8_t Emission_Rx[Emis_Size];//接收缓存区
uint8_t Emission_Tx[Emis_Size];//发送缓存区

TaskHandle_t xHandleTaskEmis_Send = NULL;//发送线程
TaskHandle_t xHandleTask_Receive = NULL;//接收线程
TaskHandle_t xHandleTask_hard_control = NULL;//硬件控制
TaskHandle_t xHandleTask_hard_monitor = NULL;//硬件监测控制

EmissionCmd_t EmissionCmd;//发送给激励器数据结构体
Respond_t Respond_Cmd;//激励器返回结构体
Respond_t Respond_Cmd2;//激励器返回结构体，双频
Respond_t Respond_Cmd3;//激励器返回结构体，三频

State_t Sys_State;//系统状态结构体
Show_Parameter_t Show_Parameter;//显示参数
Best_band_t Best_band;//最佳频段
Temperature_Humidity_t Temperature_Humidity;//温湿度
Monitor_t Monitor;//监控系统控制权
Hard_control_t Hard_control;
State_monitor_t State_monitor[10];
float eADC0_buff[100];
float eADC1_buff[100];
float eADC2_buff[100];
float eADC3_buff[100];

float SWR_array[256];//驻波比缓存
//static int SWR_index[256];//驻波比下标寻址

/* 发送指令给发射机(通过控制激励器，从而控制发射机状态)
 * 优先级5
 */
void vTaskTaskTaskEmis_Send(void *pvParameters)
{
    const TickType_t xTicksToWait = 500; /* 最大延迟500ms */
    uint32_t ulValue;
    BaseType_t xResult;

    uint8_t i = 0;

    int ret;
    while(1)
    {
        xResult = xTaskNotifyWait(0x00000000,
                                  0xFFFFFFFF,
                                  &ulValue,
                                  portMAX_DELAY); /* 最大允许延迟时间 */
        if( xResult == pdPASS )
        {
            if((ulValue & BIT_0) != 0)			//若初始化完成，则查询报警状态，否则清零
            {
                //	Work_paraBack.Transmitte_id[0]=CAN_ID;//组包can id 既发射机 id
                if(	Sys_State.Already_init==1)   //已经初始化了的
                {
                    if(get_history_alm()==1)   //历史有报警状态
                    {
                        App_printf("NO1. history state was alarm!\r\n");
                        Work_paraBack.Trans_current_state[0]=0x11;//当前是报警状态
                        if(Sys_State.No_power_level==1&&Sys_State.Emis_working==1)   //无功率报警
                        {
                            Work_paraBack.Hist_alarm[0]=0x01;//正在发射
                        }
                        if(Sys_State.alarm==1)   //激励器报警
                        {
                            Sys_State.Emis_working=0;
                            Work_paraBack.Hist_alarm[0]=0x00;//没有发射
                        }

                        if(Sys_State.Emis_working==0)
                        {
                            memset(Work_paraBack.Channel,0,3);//信道清零
                            memset(Work_paraBack.Freq,0,12);//频率清零
                            memset(Work_paraBack.Power_grade,0,3);//功率等级清零
                            Work_paraBack.Type[0]=0;//工作种类清零
                            Work_paraBack.Forward_power[0]=0;
                            Work_paraBack.Reverse_power[0]=0;
                            Work_paraBack.Power_45_intensity[0]=0;
                            Work_paraBack.Hist_alarm[0]=0x00;//没有发射
                            //电压显示
                            Work_paraBack.Power_45_voltage[0]=(float) ADC_ConvertedValue[2]/4096*3.3*2*Debug_M_45V;
                        }
                        else
                        {
                            Work_paraBack.Hist_alarm[0]=0x01;//正在发射
                            Work_paraBack.Type[0]=Trans_open.Type[0];//工作种类，固频，双频或者三频,把发射包的该状态给查询包
                            Work_paraBack.Channel[0]=Trans_open.Channel[0];//Respond_Cmd.channel;//赋值信道
                            memcpy(Work_paraBack.Freq,Trans_open.Freq,4);//赋值频率
                            Work_paraBack.Power_grade[0]=Trans_open.Power_grade[0];//Respond_Cmd.power_level;//功率等级

                            if(Work_paraBack.Type[0]!=0x01)   //不是固频
                            {
                                Work_paraBack.Channel[1]=Trans_open.Channel[1];//Respond_Cmd2.channel;//赋值信道
                                memcpy((uint8_t *)(Work_paraBack.Freq)+4,(uint8_t*)(Trans_open.Freq)+4,4);//赋值频率
                                Work_paraBack.Power_grade[1]=Trans_open.Power_grade[1];//功率等级
                                if(Work_paraBack.Type[0]!=0x02)   //也不是双频，那么只能是三频
                                {
                                    Work_paraBack.Channel[2]=Trans_open.Channel[2];//Respond_Cmd3.channel;//赋值信道
                                    memcpy((uint8_t *)(Work_paraBack.Freq)+8,(uint8_t*)(Trans_open.Freq)+8,4);//赋值频率
                                    Work_paraBack.Power_grade[2]=Trans_open.Power_grade[2];//功率等级
                                    //App_printf("Respond_Cmd3.power_level=%02x\r\n",Respond_Cmd3.power_level);
                                }
                            }

                            //电流显示
                            if(Show_Parameter.M_45I>1.5)   //2.3-1.5V=0.8V =>对应 30A，步进37.5
                            {
                                Work_paraBack.Power_45_intensity[0]=(Show_Parameter.M_45I-1.5)*Debug_M_45I+60;
                            }
                            else     //小于 1.5V电流系数为40
                            {
                                Work_paraBack.Power_45_intensity[0]=Show_Parameter.M_45I*40;
                            }
                            //电压显示
                            Work_paraBack.Power_45_voltage[0]=Show_Parameter.M_45V*Debug_M_45V;

                            App_printf("Forward_power=%f \r\n",Work_paraBack.Forward_power[0]);
                            App_printf("Reverse_power=%f \r\n",Work_paraBack.Reverse_power[0]);
                            App_printf("Power_45_voltage=%f \r\n",Work_paraBack.Power_45_voltage[0]);
                            App_printf("Power_45_intensity=%f \r\n",Work_paraBack.Power_45_intensity[0]);
                            App_printf("Standing_wave_ratio=%f \r\n",Show_Parameter.Standing_wave_ratio);
                        }
                    }
                    else     //只是AD采集没有当前报警，在下面可能有激励器的当前报警
                    {
                        Work_paraBack.Hist_alarm[0]=0x01;//发射状态
                        //查询
                        Pre_EmissionSend(&EmissionCmd,INQUIRE);//查询组包
                        ret=run_command(xTicksToWait);//内部循环了3次
                        if(ret==1)   //成功状态
                        {
                            if(Respond_Cmd.status==1)   //发射状态
                            {
                                App_printf("working state...\r\n");
                                Sys_State.Emis_working=1;
                                Work_paraBack.Trans_current_state[0]=0x01;//正常发射状态
                                Work_paraBack.Type[0]=Trans_open.Type[0];//工作种类，固频，双频或者三频,把发射包的该状态给查询包
                                Work_paraBack.Channel[0]=Trans_open.Channel[0];//赋值信道
                                memcpy(Work_paraBack.Freq,Trans_open.Freq,4);//赋值频率
                                Work_paraBack.Power_grade[0]=Trans_open.Power_grade[0];//功率等级
                                if(Work_paraBack.Type[0]!=0x01)   //不是固频
                                {
                                    Work_paraBack.Channel[1]=Trans_open.Channel[1];//Respond_Cmd2.channel;//赋值信道
                                    memcpy((uint8_t *)(Work_paraBack.Freq)+4,(uint8_t*)(Trans_open.Freq)+4,4);//赋值频率
                                    Work_paraBack.Power_grade[1]=Trans_open.Power_grade[1];//功率等级
                                    if(Work_paraBack.Type[0]!=0x02)   //也不是双频，那么只能是三频
                                    {
                                        Work_paraBack.Channel[2]=Trans_open.Channel[2];//Respond_Cmd3.channel;//赋值信道
                                        memcpy((uint8_t *)(Work_paraBack.Freq)+8,(uint8_t*)(Trans_open.Freq)+8,4);//赋值频率
                                        Work_paraBack.Power_grade[2]=Trans_open.Power_grade[2];//功率等级
                                    }
                                }
                                //电流显示
                                if(Show_Parameter.M_45I>1.5)   //2.3-1.5V=0.8V =>对应 30A，步进37.5
                                {
                                    Work_paraBack.Power_45_intensity[0]=(Show_Parameter.M_45I-1.5)*Debug_M_45I+60;
                                }
                                else     //小于 1.5V电流系数为40
                                {
                                    Work_paraBack.Power_45_intensity[0]=Show_Parameter.M_45I*40;
                                }
                                //电压显示
                                Work_paraBack.Power_45_voltage[0]=Show_Parameter.M_45V*Debug_M_45V;

                                App_printf("Forward_power=%f \r\n",Work_paraBack.Forward_power[0]);
                                App_printf("Reverse_power=%f \r\n",Work_paraBack.Reverse_power[0]);
                                App_printf("Power_45_voltage=%f \r\n",Work_paraBack.Power_45_voltage[0]);
                                App_printf("Power_45_intensity=%f \r\n",Work_paraBack.Power_45_intensity[0]);
                                App_printf("Standing_wave_ratio=%f \r\n",Show_Parameter.Standing_wave_ratio);
                            }
                            else     //待机状态,Respond_Cmd.status 则为0，因为这个变量只有两个状态
                            {
                                App_printf("waiting state...\r\n");
                                Sys_State.Emis_working=0;
                                Work_paraBack.Trans_current_state[0]=0x02;//停止状态，既待机状态
                                memset(Work_paraBack.Channel,0,3);//信道清零
                                memset(Work_paraBack.Freq,0,12);//频率清零
                                memset(Work_paraBack.Power_grade,0,3);//功率等级清零
                                Work_paraBack.Type[0]=0;//工作种类清零
                                Work_paraBack.Forward_power[0]=0;
                                Work_paraBack.Reverse_power[0]=0;
                                Work_paraBack.Power_45_intensity[0]=0;
                                //电压显示
                                Work_paraBack.Power_45_voltage[0]=(float) ADC_ConvertedValue[2]/4096*3.3*2*Debug_M_45V;
                            }
                        }
                        else     //报警状态
                        {
                            App_printf("NO2.current state is alarm!\r\n");
                            Work_paraBack.Trans_current_state[0]=0x11;//当前是报警状态
                            if(Sys_State.No_power_level==1&&Sys_State.Emis_working==1)   //无功率报警
                            {
                                Work_paraBack.Hist_alarm[0]=0x01;//正在发射
                            }
                            if(Sys_State.alarm==1)   //激励器报警
                            {
                                Sys_State.Emis_working=0;
                                Work_paraBack.Hist_alarm[0]=0x00;//没有发射
                            }
                            if(Sys_State.Emis_working==0)
                            {
                                memset(Work_paraBack.Channel,0,3);//信道清零
                                memset(Work_paraBack.Freq,0,12);//频率清零
                                memset(Work_paraBack.Power_grade,0,3);//功率等级清零
                                Work_paraBack.Type[0]=0;//工作种类清零
                                Work_paraBack.Forward_power[0]=0;
                                Work_paraBack.Reverse_power[0]=0;
                                Work_paraBack.Power_45_intensity[0]=0;
                                //电压显示
                                Work_paraBack.Power_45_voltage[0]=Show_Parameter.M_45V*Debug_M_45V;
                                Work_paraBack.Hist_alarm[0]=0x00;//没有发射
                            }
                            else
                            {
                                Work_paraBack.Hist_alarm[0]=0x01;//正在发射
                                Work_paraBack.Type[0]=Trans_open.Type[0];//工作种类，固频，双频或者三频,把发射包的该状态给查询包
                                Work_paraBack.Channel[0]=Trans_open.Channel[0];//Respond_Cmd.channel;//赋值信道
                                memcpy(Work_paraBack.Freq,Trans_open.Freq,4);//赋值频率
                                Work_paraBack.Power_grade[0]=Trans_open.Power_grade[0];//Respond_Cmd.power_level;//功率等级
                                if(Work_paraBack.Type[0]!=0x01)   //不是固频
                                {
                                    Work_paraBack.Channel[1]=Trans_open.Channel[1];//Respond_Cmd2.channel;//赋值信道
                                    memcpy((uint8_t *)(Work_paraBack.Freq)+4,(uint8_t*)(Trans_open.Freq)+4,4);//赋值频率
                                    Work_paraBack.Power_grade[1]=Trans_open.Power_grade[1];//功率等级
                                    if(Work_paraBack.Type[0]!=0x02)   //也不是双频，那么只能是三频
                                    {
                                        Work_paraBack.Channel[2]=Trans_open.Channel[2];//Respond_Cmd3.channel;//赋值信道
                                        memcpy((uint8_t *)(Work_paraBack.Freq)+8,(uint8_t*)(Trans_open.Freq)+8,4);//赋值频率
                                        Work_paraBack.Power_grade[2]=Trans_open.Power_grade[2];//功率等级
                                        //App_printf("Respond_Cmd3.power_level=%02x\r\n",Respond_Cmd3.power_level);
                                    }
                                }
                                //电流显示
                                if(Show_Parameter.M_45I>1.5)   //2.3-1.5V=0.8V =>对应 30A，步进37.5
                                {
                                    Work_paraBack.Power_45_intensity[0]=(Show_Parameter.M_45I-1.5)*Debug_M_45I+60;
                                }
                                else     //小于 1.5V电流系数为40
                                {
                                    Work_paraBack.Power_45_intensity[0]=Show_Parameter.M_45I*40;
                                }
                                //电压显示
                                Work_paraBack.Power_45_voltage[0]=Show_Parameter.M_45V*Debug_M_45V;

                                App_printf("Forward_power=%f \r\n",Work_paraBack.Forward_power[0]);
                                App_printf("Reverse_power=%f \r\n",Work_paraBack.Reverse_power[0]);
                                App_printf("Power_45_voltage=%f \r\n",Work_paraBack.Power_45_voltage[0]);
                                App_printf("Power_45_intensity=%f \r\n",Work_paraBack.Power_45_intensity[0]);
                                App_printf("Standing_wave_ratio=%f \r\n",Show_Parameter.Standing_wave_ratio);
                            }
                        }

                    }
                }
                else     //没有初始化
                {
                    Work_paraBack.Trans_current_state[0]=0x10;//没有经过初始化
                    memset(Work_paraBack.Channel,0,3);//信道清零
                    memset(Work_paraBack.Freq,0,12);//频率清零
                    memset(Work_paraBack.Power_grade,0,3);//功率等级清零
                    Work_paraBack.Type[0]=0;//工作种类清零
                    Work_paraBack.Forward_power[0]=0;
                    Work_paraBack.Reverse_power[0]=0;
                    Work_paraBack.Power_45_intensity[0]=0;
                    //电压显示
                    Work_paraBack.Power_45_voltage[0]=(float) ADC_ConvertedValue[2]/4096*3.3*2*Debug_M_45V;
                    Work_paraBack.Hist_alarm[0]=0;//没有发射
                }
                //需要触发send to pc线程
                xTaskNotify(xHandleTaskSendToPC, /* 目标任务 */
                            BIT_0, /* 设置目标任务事件标志位 bit0 */
                            eSetBits); /* 将目标任务的事件标志位与 BIT_0 进行或操作，将结果赋值给事件标志位。*/
            }
            else if((ulValue & BIT_1) != 0)		//查询
            {
                /*挂起main函数中的ADC采集*/
                if(xHandleTaskUserIF!=NULL)
                {
                    vTaskSuspend(xHandleTaskUserIF);
                    App_printf("stop task Suspend main->ADC task!!!\r\n");
                }
                //停止发送,在停止的时候，再发送停止指令，也可以响应
                Trans_stopBack.Transmitte_id[0]=CAN_ID;
                Pre_EmissionSend(&EmissionCmd,INQUIRE);//查询组包
                ret=run_command(xTicksToWait);
                if(ret==1)   //成功查询了，才继续下面的，否则就是报警状态
                {
                    if(Respond_Cmd.status==0x00)   //如果是停止状态
                    {
                        Trans_stopBack.Trans_state[0]=0xFC;//已经是停止状态
                        Sys_State.Emis_working=0;
                    }
                    else     //如果不是停止状态，发射停止指令
                    {
                        Pre_EmissionSend(&EmissionCmd,STOP);
                        if(run_command(xTicksToWait)==1)   //停止成功
                        {
                            Trans_stopBack.Trans_state[0]=0xFE;
                            Sys_State.Emis_working=0;
                        }
                        else     //不成功，返回停止失败，且处于报警状态
                        {
                            Trans_stopBack.Trans_state[0]=0x02;//停止失败
                        }
                    }
                }
                else if(ret==2)     //发射机报警
                {
                    Trans_stopBack.Trans_state[0]=0xFC;//有发射机报警时停止发射的
                }
                else     //连查询都不成功，返回停止失败，且处于报警状态
                {
                    Trans_stopBack.Trans_state[0]=0x02;//PC下发的信道停止失败
                }
                if(xHandleTaskUserIF!=NULL)
                {
                    App_printf("stop task Resume main->ADC task!!!\r\n");
                    vTaskResume(xHandleTaskUserIF);
                }
                //然后触发send to PC的查询回包
                xTaskNotify(xHandleTaskSendToPC, /* 目标任务 */
                            BIT_1, /* 设置目标任务事件标志位 bit1 */
                            eSetBits); /* 将目标任务的事件标志位与 BIT_1 进行或操作，将结果赋值给事件标志位。*/
            }

            else 	if((ulValue & BIT_2) != 0)	//保留
            {
//						//解除报警
//						Sys_State.alarm=0;//先用这种清零，之后会写成一个函数
//						App_printf("//解除报警\r\n");
            }
            else 	if((ulValue & BIT_3) != 0)	//发射
            {
                App_printf("Emis//执行操作\r\n");

#if	NEW_PROTOCOL_EN	//新机器协议
                MT2000_Cmd_Channel();			//设置信道、工作模式、工作方式、频率、跳频时间
                ret = MT2000_Wait_Ack();
                if( ret == 1 )
                {
                    MT2000_Cmd_Tune();			//调谐
                    for(i=0; i<10; i++)
                    {
                        vTaskDelay(500);
                        ret = MT2000_Wait_Ack();
                    }

                    if( ret == 1 )
                    {
                        MT2000_Cmd_Emit();		//发射
                        ret = MT2000_Wait_Ack();
                    }
                }
#else
                Pre_EmissionSend(&EmissionCmd,CHANNEL,1);//调用信道
                if(run_command(xTicksToWait)==1)   //内部循环了3次
                {
                    //	App_printf("Respond_Cmd.species=%d\r\n",Respond_Cmd.species);
                    if(Respond_Cmd.species!=0x03)   //如果不是调频状态，先切换到调频状态
                    {
                        Pre_EmissionSend(&EmissionCmd,SPECIES);//调频状态
                        if(run_command(xTicksToWait)!=1)   //失败，goto finish
                        {
                            goto finish;
                        }
                    }

                    //调用信道成功才执行下面的:
                    Pre_EmissionSend(&EmissionCmd,FREQUENCY,1);//固频设置频率
                    if(run_command(xTicksToWait)==1)   //内部循环了3次
                    {
                        if(Trans_open.Power_grade[0]==0x02)   //四分之一功率
                        {
                            Pre_EmissionSend(&EmissionCmd,POWER_LEVEL,4);//先设置功率等级为四分之一
                        }
                        else if(Trans_open.Power_grade[0]==0x00)     //全功率
                        {
                            Pre_EmissionSend(&EmissionCmd,POWER_LEVEL,1);//先设置功率等级为全功率
                        }
                        //else if(Trans_open.Power_grade[0]==0x01)//二分之一功率
                        else   //预防解析问题，当不是四分之一也不是全功率的时候就给二分之一
                        {
                            Pre_EmissionSend(&EmissionCmd,POWER_LEVEL,2);//先设置功率等级为二分之一
                        }
                        if(run_command(xTicksToWait)==1)
                        {
                            Pre_EmissionSend(&EmissionCmd,TUNING);//调谐
                            if(run_command(xTicksToWait*14)==1)   //调谐给7s
                            {
                                Pre_EmissionSend(&EmissionCmd,EMISSION);//发射指令
                                if(run_command(xTicksToWait)!=1)
                                {
                                    goto finish;
                                }
                                else
                                {
                                    Sys_State.Emis_working=1;//系统处于发射状态
                                }
                                if(Trans_open.Type[0]!=0x01)   //不是固频
                                {
                                    Sys_State.Emis_working=0;//系统处于发射状态清零
                                    vTaskDelay(500);
                                    Pre_EmissionSend(&EmissionCmd,STOP);//停止指令
                                    if(run_command(xTicksToWait)==1)   //如果停止成功，执行第二个信道
                                    {
                                        Pre_EmissionSend(&EmissionCmd,CHANNEL,2);//调用信道2
                                        if(run_command(xTicksToWait)==1)   //内部循环了3次
                                        {
                                            if(Respond_Cmd.species!=0x03)   //如果不是调频状态，先切换到调频状态
                                            {
                                                Pre_EmissionSend(&EmissionCmd,SPECIES);//调频状态
                                                if(run_command(xTicksToWait)!=1)   //失败，goto finish
                                                {
                                                    goto finish;
                                                }
                                            }
                                            Pre_EmissionSend(&EmissionCmd,FREQUENCY,2);//双频设置频率
                                            if(run_command(xTicksToWait)==1)   //内部循环了3次
                                            {
                                                if(Trans_open.Power_grade[1]==0x02)   //四分之一功率
                                                {
                                                    Pre_EmissionSend(&EmissionCmd,POWER_LEVEL,4);//先设置功率等级为四分之一
                                                }
                                                else if(Trans_open.Power_grade[1]==0x00)     //全功率
                                                {
                                                    Pre_EmissionSend(&EmissionCmd,POWER_LEVEL,1);//先设置功率等级为全功率
                                                }
                                                //else if(Trans_open.Power_grade[0]==0x01)//二分之一功率
                                                else   //预防解析问题，当不是四分之一也不是全功率的时候就给二分之一
                                                {
                                                    Pre_EmissionSend(&EmissionCmd,POWER_LEVEL,2);//先设置功率等级为二分之一
                                                }
                                                if(run_command(xTicksToWait)==1)   //内部循环了3次
                                                {
                                                    Pre_EmissionSend(&EmissionCmd,TUNING);//调谐
                                                    if(run_command(xTicksToWait*14)==1)   //调谐给7s
                                                    {
                                                        Pre_EmissionSend(&EmissionCmd,EMISSION);//发射指令
                                                        if(run_command(xTicksToWait)==1)
                                                        {
                                                            vTaskDelay(500);
                                                            Pre_EmissionSend(&EmissionCmd,STOP);//停止指令
                                                            if(run_command(xTicksToWait)==1)
                                                            {
                                                                if(Trans_open.Type[0]==0x02)   //是双频
                                                                {
                                                                    Pre_EmissionSend(&EmissionCmd,MODE,2);//开启双频
                                                                    if(run_command(xTicksToWait)==1)   //如果开启成功
                                                                    {
                                                                        Pre_EmissionSend(&EmissionCmd,EMISSION);//发射双频
                                                                        if(run_command(xTicksToWait)!=1)
                                                                        {
                                                                            goto finish;
                                                                        }
                                                                        else
                                                                        {
                                                                            Sys_State.Emis_working=1;//系统正在发射
                                                                        }
                                                                    }
                                                                }
                                                                else if(Trans_open.Type[0]==0x03)     //是三频
                                                                {
                                                                    Pre_EmissionSend(&EmissionCmd,CHANNEL,3);//调用信道3
                                                                    if(run_command(xTicksToWait)==1)   //内部循环了3次
                                                                    {
                                                                        if(Respond_Cmd.species!=0x03)   //如果不是调频状态，先切换到调频状态
                                                                        {
                                                                            Pre_EmissionSend(&EmissionCmd,SPECIES);//调频状态
                                                                            if(run_command(xTicksToWait)!=1)   //失败，goto finish
                                                                            {
                                                                                goto finish;
                                                                            }
                                                                        }
                                                                        //调用信道成功才执行下面的:
                                                                        Pre_EmissionSend(&EmissionCmd,FREQUENCY,3);//三频设置频率
                                                                        if(run_command(xTicksToWait)==1)   //内部循环了3次
                                                                        {
                                                                            if(Trans_open.Power_grade[2]==0x02)   //四分之一功率
                                                                            {
                                                                                Pre_EmissionSend(&EmissionCmd,POWER_LEVEL,4);//先设置功率等级为四分之一
                                                                            }
                                                                            else if(Trans_open.Power_grade[2]==0x01)     //二分之一功率
                                                                            {
                                                                                Pre_EmissionSend(&EmissionCmd,POWER_LEVEL,2);//先设置功率等级为四分之一
                                                                            }
                                                                            else if(Trans_open.Power_grade[2]==0x00)     //全功率
                                                                            {
                                                                                Pre_EmissionSend(&EmissionCmd,POWER_LEVEL,1);//先设置功率等级为四分之一
                                                                            }
                                                                            if(run_command(xTicksToWait)==1)
                                                                            {
                                                                                Pre_EmissionSend(&EmissionCmd,TUNING);//调谐
                                                                                if(run_command(xTicksToWait*14)==1)   //调谐给7s
                                                                                {
                                                                                    Pre_EmissionSend(&EmissionCmd,EMISSION);//发射指令
                                                                                    if(run_command(xTicksToWait)==1)
                                                                                    {
                                                                                        vTaskDelay(500);
                                                                                        Pre_EmissionSend(&EmissionCmd,STOP);//停止指令
                                                                                        if(run_command(xTicksToWait)==1)
                                                                                        {
                                                                                            Pre_EmissionSend(&EmissionCmd,MODE,3);//开启三频
                                                                                            if(run_command(xTicksToWait)==1)   //如果开启成功
                                                                                            {
                                                                                                Pre_EmissionSend(&EmissionCmd,EMISSION);//发射三频
                                                                                                if(run_command(xTicksToWait)!=1)
                                                                                                {
                                                                                                    goto finish;
                                                                                                }
                                                                                                else
                                                                                                {
                                                                                                    Sys_State.Emis_working=1;//系统正在发射
                                                                                                }
                                                                                            }
                                                                                        }
                                                                                    }
                                                                                }
                                                                            }
                                                                        }
                                                                    }
                                                                }
                                                            }
                                                        }
                                                    }
                                                }
                                            }
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
finish:
#endif
                Sys_State.FM_working=0;//正在工作标志清零，不管上面成功与否，这个压制过程宣布结束
            }
            else 	if((ulValue & BIT_4) != 0)	//发射准备判断
            {
                App_printf("先查询一下是不是停止状态\r\n");
                Pre_EmissionSend(&EmissionCmd,INQUIRE);//查询组包
                if(run_command(xTicksToWait)==1)   //内部循环了3次
                {
                    if(Respond_Cmd.status==0x01 || get_history_alm()==1)   //如果是工作状态或者有报警，在没有接收停止状态情况下又接收到工作指令。报错
                    {
                        App_printf("waring: working or alarm state!\r\n");
                        Trans_openBack.Trans_state[0]=0xFC;//向PC报告
                    }
                    else
                    {
                        Trans_openBack.Trans_state[0]=0xFD;//表示接收到了指令，但是需要时间去调频
                        Sys_State.Emis_readay=1;//发射准备	标志
                    }
                }
                else
                {
                    Trans_openBack.Trans_state[0]=0xFC;//向PC报警
                }
                App_printf("Emiss:恢复ToPC线程\r\n");
                vTaskResume(xHandleTaskSendToPC);
            }
            else 	if((ulValue & BIT_5) != 0)	//扫频
            {
                /*挂起main函数中的ADC采集*/
                if(xHandleTaskUserIF!=NULL)
                {
                    vTaskSuspend(xHandleTaskUserIF);
                    App_printf("Band_scan task Suspend main->ADC task!!!\r\n");
                }
                App_printf("Band scaning...\r\n");
                for(int i=0; i<10&&Sys_State.Cancel_Sweeping!=1; i++)
                {
                    switch(i)
                    {
                    case 0:
                    {
                        begain=5;
                        end=26;
                        break;
                    }
                    case 1:
                    {
                        begain=5.1;
                        end=26.1;
                        break;
                    }
                    case 2:
                    {
                        begain=5.2;
                        end=25.2;
                        break;
                    }
                    case 3:
                    {
                        begain=5.3;
                        end=25.3;
                        break;
                    }
                    case 4:
                    {
                        begain=5.4;
                        end=25.4;
                        break;
                    }
                    case 5:
                    {
                        begain=5.5;
                        end=25.5;
                        break;
                    }
                    case 6:
                    {
                        begain=5.6;
                        end=25.6;
                        break;
                    }
                    case 7:
                    {
                        begain=5.7;
                        end=25.7;
                        break;
                    }
                    case 8:
                    {
                        begain=5.8;
                        end=25.8;
                        break;
                    }
                    case 9:
                    {
                        begain=5.9;
                        end=25.9;
                        break;
                    }
                    }
                    Band_scan(xTicksToWait,i);
                    if(get_history_alm()==1)
                    {
                        break;
                    }
//							if(i==1)
//							{
//								Sys_State.Cancel_Sweeping=1;//测试
//								break;
//							}
                    if(Sys_State.Cancel_Sweeping==1)
                    {
                        break;
                    }
                    for(unsigned int j=0; j<180&&Sys_State.Cancel_Sweeping!=1&&i!=9; j++)
                    {
                        vTaskDelay(1000);
                    }

                }
                if(Sys_State.Cancel_Sweeping==1)   //如果上面的Band_scan是由于取消扫频退出的
                {
                    if(Sys_State.Sweeping!=0)   //上面的扫频由于强制停止中止了，需要将发射机先停止以保障安全
                    {
                        Pre_EmissionSend(&EmissionCmd,STOP);//停止指令
                        if(run_command(xTicksToWait)==1)   //如果停止成功
                        {
                            Sys_State.Cancel_Sweeping=0;//取消扫频标志清零
                            clean_sweeping=1;
                        }
                    }
                    else     //已经扫频完成，清除标志
                    {
                        Sys_State.Cancel_Sweeping=0;//取消扫频标志清零
                        clean_sweeping=1;
                    }
                }
//						Sys_State.Sweeping=0;//扫频结束标志清零
                if(xHandleTaskUserIF!=NULL)
                {
                    App_printf("Band_scan task Resume main->ADC task!!!\r\n");
                    vTaskResume(xHandleTaskUserIF);
                }

            }
            else 	if((ulValue & BIT_6) != 0)	//功率增加
            {
                App_printf("Gain_plus...\r\n");
                Sys_State.Sys_gain=1;
                //增加功率
                if(Show_Parameter.M_45I>1.5)   //2.3-1.5V=0.8V =>对应 30A，步进37.5
                {
                    Work_paraBack.Power_45_intensity[0]=(Show_Parameter.M_45I-1.5)*Debug_M_45I+60;
                }
                else     //小于 1.5V电流系数为40
                {
                    Work_paraBack.Power_45_intensity[0]=Show_Parameter.M_45I*40;
                }
                if(Work_paraBack.Forward_power[0]>=1800)
                {
                    Add_PowerBack.results [0]=0xFB;
                }
                if(Work_paraBack.Forward_power[0]>=(float)Add_Power.Power_UP[0]*10.0)   //在增加中发现当前的功率大于需要增加到的功率
                {
                    Add_PowerBack.results [0]=0xFB;//直接返回不能增加
                }
                else if(Work_paraBack.Forward_power[0]<=1800)
                {
                    int Back = 0;
                    float cmp_buffer = 0.0;
                    volatile uint8_t count = 0;
                    Pre_EmissionSend(&EmissionCmd,Gain_plus);//增益加指令组包给激励器的
                    for(int i = 0; i<50&&get_history_alm()==0&&Work_paraBack.Power_45_intensity[0]<100; i++)
                    {
                        cmp_buffer = Work_paraBack.Forward_power[0];//在每次发增加功率的命令之前就先保存当前的值
                        Back = run_command(xTicksToWait*2);//如果加成功，返回-2发射机报警（必须要停），返回-1或是0都是激励器无响应
                        if(Back == -1 || Back == 0)   //如果返回激励器无响应，则再执行一次
                        {
                            Back = run_command(xTicksToWait);//如果加成功，返回-2发射机报警（必须要停），返回-1或是0都是激励器无响应
                        }
                        //这里要加延时，大约200ms
                        vTaskDelay(1000);
                        if(fabs((float)Add_Power.Power_UP[0]*10.0 - Work_paraBack.Forward_power[0])<=30)   //加成功，且加到了预定的频率
                        {
                            //Add_PowerBack.results [0]=0xFE;//返回增加成功
                            break;
                        }
                        if(Back != 1)   //发射机或则发射失败
                        {
                            //Add_PowerBack.results [0]=0xFC;//返回增加失败
                            break;
                        }
                        if(Work_paraBack.Forward_power[0]>=1800.0)   //增加之后大于了1800W
                        {
                            //Add_PowerBack.results [0]=0xFE;//返回增加成功
                            break;
                        }
                        else     //但是需要判断是否增加成功了
                        {
                            if(fabs(cmp_buffer-Work_paraBack.Forward_power[0])<=5)//说明增加前和增加后没有变化
                                count++;
                            else
                            {
                                count = 0;//只要中间成功一次之后，就把计数清0.
                            }
                            if(count>=2)   //大等于2次没有增加功率成功，就返回增加失败
                            {
                                //Add_PowerBack.results [0]=0xFC;//直接返回增加失败
                                count = 0;
                                break;
                            }
                        }
                    }
                }
                clean_gain=1;
//						//然后触发send to PC的查询回包
//						xTaskNotify(xHandleTaskSendToPC, /* 目标任务 */
//												BIT_10, /* 设置目标任务事件标志位 bit10 */
//												eSetBits); /* 将目标任务的事件标志位与 BIT_10 进行或操作，将结果赋值给事件标志位。*/
            }
            else 	if((ulValue & BIT_7) != 0)	//功率减小
            {
                App_printf("Gain_reduction...\r\n");
                Sys_State.Sys_reduction=1;
                //减少功率
                if(Show_Parameter.M_45I>1.5)   //2.3-1.5V=0.8V =>对应 30A，步进37.5
                {
                    Work_paraBack.Power_45_intensity[0]=(Show_Parameter.M_45I-1.5)*Debug_M_45I+60;
                }
                else     //小于 1.5V电流系数为40
                {
                    Work_paraBack.Power_45_intensity[0]=Show_Parameter.M_45I*40;
                }
                if(Work_paraBack.Forward_power[0]<=300)
                {
                    Sub_PowerBack.results [0]=0xFB;//不能减少
                }
                else if(Work_paraBack.Forward_power[0]<=(float)Sub_Power.Power_DOWN[0]*10.0)     //在减少中发现当前的功率小于需要减少到的功率
                {
                    Sub_PowerBack.results [0]=0xFB;//不能减少
                }
                else if(Work_paraBack.Forward_power[0]>=300)
                {
                    int Back = 0;
                    float cmp_buffer = 0.0;
                    volatile uint8_t count = 0;
                    //cmp_buffer = Work_paraBack.Forward_power[0];//首先保存没有增加功率之前的功率
                    Pre_EmissionSend(&EmissionCmd,Gain_reduction);//增益减指令
                    for(int i = 0; i<50&&get_history_alm()==0&&Work_paraBack.Power_45_intensity[0]<100; i++)
                    {
                        cmp_buffer = Work_paraBack.Forward_power[0];//在每次发减少功率的命令之前就先保存当前的值
                        Back = run_command(xTicksToWait*2);//如果加成功，返回-2发射机报警（必须要停），返回-1或是0都是激励器无响应
                        if(Back == -1 || Back == 0)   //如果返回激励器无响应，则再执行一次
                        {
                            Back = run_command(xTicksToWait);//如果加成功，返回-2发射机报警（必须要停），返回-1或是0都是激励器无响应
                        }
                        //这里要加延时，大约200ms
                        vTaskDelay(1000);
                        if(fabs((float)Sub_Power.Power_DOWN[0]*10.0 - Work_paraBack.Forward_power[0])<=30)   //减成功，且减到了预定的功率
                        {
                            //Sub_PowerBack.results [0]=0xFE;//返回减少成功
                            break;
                        }
                        if(Back != 1)   //发射机或则发射失败
                        {
                            //Sub_PowerBack.results [0]=0xFC;//返回减少失败
                            break;
                        }
                        if(Work_paraBack.Forward_power[0]<=300.0)   //减少之后小于300W
                        {
                            //	Sub_PowerBack.results [0]=0xFE;//返回减少成功
                            break;
                        }
                        else     //但是需要判断是否减少成功了
                        {
                            if(fabs(cmp_buffer-Work_paraBack.Forward_power[0])<=5)//说明减少前和增加后没有变化
                                count++;
                            else
                            {
                                count = 0;//只要中间成功一次之后，就把计数清0.
                            }
                            if(count>=2)   //大等于2次没有增加功率成功，就返回减少失败
                            {
                                //Sub_PowerBack.results [0]=0xFC;//直接返回增加失败
                                count = 0;
                                break;
                            }
                        }
                    }
                }
                clean_reduction=1;
//						//然后触发send to PC的查询回包
//						xTaskNotify(xHandleTaskSendToPC, /* 目标任务 */
//												BIT_11, /* 设置目标任务事件标志位 bit11 */
//												eSetBits); /* 将目标任务的事件标志位与 BIT_11 进行或操作，将结果赋值给事件标志位。*/
            }
            else 	if((ulValue & BIT_8) != 0)	//开机
            {
                MT2000_Cmd_Tune();				//调谐
                for(i=0; i<10; i++)
                {
                    MT2000_Cmd_Power_ON();
                    vTaskDelay(100);
                    ret = MT2000_Wait_Ack();
                    if( ret == 1 )
                    {
                        Sys_State.Sys_open = 1;
                    }
                }
            }
            else 	if((ulValue & BIT_9) != 0)	//保留
            {
//						g_fuc_codToPC[0]=0x06;
//						g_fuc_codToPC[1]=0x02;
//						Alarm_historyBack.Transmitte_id[0]=	CAN_ID;
//						Pre_EmissionSend(&EmissionCmd,INQUIRE);//查询组包
//						if(run_command(xTicksToWait)==1)//内部循环了3次
//						{
//							if(Respond_Cmd.status==0x01)
//							{
//								Sys_State.Emis_working=1;
//							}
//							else
//							{
//								Sys_State.Emis_working=0;
//							}
//						}
//						/*等待参数来写*/
//						set_alarm_bit();//主要是组包报警的bit位
//						Send_PC(g_fuc_codToPC);//发送出去，其他参数在报警处就应该填充在需要发送的包里面了
            }
            else								//没有对应事件
            {
                App_printf("没有对应的事件!!!\r\n");
            }

        }
        else									//等待超时
        {
            App_printf("vTaskTaskTaskEmis_Send TIME OUT!!!\r\n");
        }
    }
}
volatile int lock_flag1;
volatile int lock_flag2;
volatile int lock_flag3;

/* 硬件监控任务
 * 优先级8
 */
void vTaskTask_hard_monitor(void *pvParameters)
{
    const TickType_t xTicksToWait = 500; /* 最大延迟500ms */
    volatile uint8_t watch_count=0;

    while(1)
    {
        if(Monitor.hard_control==1&&flash_3_once_flag==1)   //硬件接管
        {
            /*debug*/
//			App_printf("\r\n%d %d %d %d\r\n",Trans_open_Copy.Type[0],lock_flag1,lock_flag2,lock_flag3);
//			vTaskDelay(1000);
            if(Monitor.need_open==1)
            {
                xTaskNotify(xHandleTask_hard_control, /* 目标任务的优先级更低，会先执行完本任务才去执行其他任务 */
                            BIT_1, /* 设置目标任务事件标志位 BIT_1 */
                            eSetBits); /* 将目标任务的事件标志位与 BIT_1 进行或操作，将结果赋值给事件标志位。*/
            }
            if(Trans_open_Copy.Type[0]==0)   //该停止了
            {
                if(Sys_State.Emis_working==1)
                {
                    //停止
                    xTaskNotify(xHandleTask_hard_control, /* 目标任务的优先级更低，会先执行完本任务才去执行其他任务 */
                                BIT_3, /* 设置目标任务事件标志位 BIT_3 */
                                eSetBits); /* 将目标任务的事件标志位与 BIT_3 进行或操作，将结果赋值给事件标志位。*/
                }
            }
            else if(Trans_open_Copy.Type[0]==1&&lock_flag1==0)     //固频  加个标志，让这个函数发射成功之后就不再执行了，只有停止过了，就清除
            {
                if(Sys_State.FM_working!=1)   //不是调频中，即可查询状态
                {
                    Pre_EmissionSend(&EmissionCmd,INQUIRE);//查询组包
                    if(run_command(xTicksToWait)==1)   //内部循环了3次
                    {
                        if(Respond_Cmd.status==0)   //停止状态
                        {
                            //有任务需要发射了
                            memcpy(Trans_open.Type,Trans_open_Copy.Type,19);
                            Sys_State.FM_working=1;//正在工作标志置位
                            xTaskNotify(xHandleTask_hard_control, /* 目标任务的优先级更低，会先执行完本任务才去执行其他任务 */
                                        BIT_0, /* 设置目标任务事件标志位 bit0 */
                                        eSetBits); /* 将目标任务的事件标志位与 BIT_0 进行或操作，将结果赋值给事件标志位。*/
                            Sys_State.FM_working=0;
                        }
                        else
                        {
                            if(memcmp(Trans_open.Freq,Respond_Cmd.frequency,3)!=0||Respond_Cmd.mode!=1)   //NOTE:这里使用Trans_open，可能PC控制发射之后崩掉，所以不是使用Trans_open_Copy
                            {
                                //停止
                                xTaskNotify(xHandleTask_hard_control, /* 目标任务的优先级更低，会先执行完本任务才去执行其他任务 */
                                            BIT_3, /* 设置目标任务事件标志位 BIT_3 */
                                            eSetBits); /* 将目标任务的事件标志位与 BIT_3 进行或操作，将结果赋值给事件标志位。*/
                            }
                            else
                            {
                                lock_flag1=1;
                            }
                        }
                    }
                }
            }
            else if(Trans_open_Copy.Type[0]==2&&lock_flag2==0)     //双频
            {
                if(Sys_State.FM_working!=1)   //不是调频中，即可查询状态
                {
                    Pre_EmissionSend(&EmissionCmd,INQUIRE);//查询组包
                    if(run_command(xTicksToWait)==1)   //内部循环了3次
                    {
                        if(Respond_Cmd.status==0)   //停止状态
                        {
                            //有任务需要发射了
                            memcpy(Trans_open.Type,Trans_open_Copy.Type,19);
                            Sys_State.FM_working=1;//正在工作标志置位
                            xTaskNotify(xHandleTask_hard_control, /* 目标任务的优先级更低，会先执行完本任务才去执行其他任务 */
                                        BIT_0, /* 设置目标任务事件标志位 BIT_0 */
                                        eSetBits); /* 将目标任务的事件标志位与 BIT_0 进行或操作，将结果赋值给事件标志位。*/
                            Sys_State.FM_working=0;
                        }
                        else
                        {
                            /*NOTE ：这里排序为1 2 3,3被踢出之后，下面的判断不会执行停止，bug*/
                            /*NOTE ：这里排序为1 2 3,3被踢出之后，下面的判断不会执行停止，bug*/
                            /*NOTE ：这里排序为1 2 3,3被踢出之后，下面的判断不会执行停止，bug*/
                            if((memcmp(Trans_open.Freq,Respond_Cmd.frequency,3)!=0\
                                    ||memcmp((uint8_t*)Trans_open.Freq+4,Respond_Cmd2.frequency,3)!=0)||Respond_Cmd.mode!=2)   //NOTE:这里使用Trans_open，可能PC控制发射之后崩掉，所以不是使用Trans_open_Copy
                            {
                                //停止
                                xTaskNotify(xHandleTask_hard_control, /* 目标任务的优先级更低，会先执行完本任务才去执行其他任务 */
                                            BIT_3, /* 设置目标任务事件标志位 BIT_3 */
                                            eSetBits); /* 将目标任务的事件标志位与 BIT_3 进行或操作，将结果赋值给事件标志位。*/
                            }
                            else
                            {
                                lock_flag2=1;
                            }
                        }
                    }
                }
            }
            else if(Trans_open_Copy.Type[0]==3&&lock_flag3==0)     //三频
            {
                if(Sys_State.FM_working!=1)   //不是调频中，即可查询状态
                {
                    Pre_EmissionSend(&EmissionCmd,INQUIRE);//查询组包
                    if(run_command(xTicksToWait)==1)   //内部循环了3次
                    {
                        if(Respond_Cmd.status==0)   //停止状态
                        {
                            //有任务需要发射了
                            memcpy(Trans_open.Type,Trans_open_Copy.Type,19);
                            Sys_State.FM_working=1;//正在工作标志置位
                            xTaskNotify(xHandleTask_hard_control, /* 目标任务的优先级更低，会先执行完本任务才去执行其他任务 */
                                        BIT_0, /* 设置目标任务事件标志位 BIT_0 */
                                        eSetBits); /* 将目标任务的事件标志位与 BIT_0 进行或操作，将结果赋值给事件标志位。*/
                            Sys_State.FM_working=0;
                        }
                        else
                        {
                            if((memcmp(Trans_open.Freq,Respond_Cmd.frequency,3)!=0\
                                    ||memcmp((uint8_t*)Trans_open.Freq+4,Respond_Cmd2.frequency,3)!=0\
                                    ||memcmp((uint8_t*)Trans_open.Freq+8,Respond_Cmd3.frequency,3)!=0)||Respond_Cmd.mode!=3)   //NOTE:这里使用Trans_open，可能PC控制发射之后崩掉，所以不是使用Trans_open_Copy
                            {
                                //停止
                                xTaskNotify(xHandleTask_hard_control, /* 目标任务的优先级更低，会先执行完本任务才去执行其他任务 */
                                            BIT_3, /* 设置目标任务事件标志位 BIT_3 */
                                            eSetBits); /* 将目标任务的事件标志位与 BIT_3 进行或操作，将结果赋值给事件标志位。*/
                            }
                            else
                            {
                                lock_flag3=1;
                            }
                        }
                    }
                }
            }
        }
        vTaskDelay(100);
        //	if(watch_count++>=2)
        //	{
        //	watch_count=0;
        //	IWDG_Feed();
        //}

    }
}



/* 硬件控制
 * 任务优先级4
 */
void vTaskTaskTask_hard_control(void *pvParameters)
{
    const TickType_t xTicksToWait = 500; /* 最大延迟500ms */
    uint32_t ulValue;
    BaseType_t xResult;

    uint8_t i = 0;
    uint8_t ret = 0;

    while(1)
    {
        xResult = xTaskNotifyWait(0x00000000,
                                  0xFFFFFFFF,
                                  &ulValue,
                                  portMAX_DELAY); /* 最大允许延迟时间 */
        if( xResult == pdPASS && Monitor.hard_control==1)   //硬件接管
        {
            if(xHandleTask_hard_monitor!=NULL)
            {
                vTaskSuspend(xHandleTask_hard_monitor);
                App_printf("hard_control task Suspend hard_monitor task!!!\r\n");
            }

            if((ulValue & BIT_0) != 0)   		//固频 双频 三频
            {
                App_printf("Hard//执行操作\r\n");

#if	NEW_PROTOCOL_EN	//新机器协议
                MT2000_Cmd_Channel();			//设置信道、工作模式、工作方式、频率、跳频时间
                ret = MT2000_Wait_Ack();
                if( ret == 1 )
                {
                    MT2000_Cmd_Tune();			//调谐
                    for(i=0; i<10; i++)
                    {
                        vTaskDelay(500);
                        ret = MT2000_Wait_Ack();
                    }
                    if( ret == 1 )
                    {
                        MT2000_Cmd_Emit();		//发射
                        ret = MT2000_Wait_Ack();
                    }
                }
#else
                Pre_EmissionSend(&EmissionCmd,CHANNEL,1);//调用信道
                if(run_command(xTicksToWait)==1)   //内部循环了3次
                {
                    if(Respond_Cmd.species!=0x03)   //如果不是调频状态，先切换到调频状态
                    {
                        Pre_EmissionSend(&EmissionCmd,SPECIES);//调频状态
                        if(run_command(xTicksToWait)!=1)   //失败，goto finish
                        {
                            goto option_fish1;
                        }
                    }
                    Pre_EmissionSend(&EmissionCmd,FREQUENCY,1);//固频设置频率
                    if(run_command(xTicksToWait)==1)   //内部循环了3次
                    {
                        if(Trans_open.Power_grade[0]==0x02)   //四分之一功率
                        {
                            Pre_EmissionSend(&EmissionCmd,POWER_LEVEL,4);//先设置功率等级为四分之一
                        }
                        else if(Trans_open.Power_grade[0]==0x00)     //全功率
                        {
                            Pre_EmissionSend(&EmissionCmd,POWER_LEVEL,1);//先设置功率等级为全功率
                        }
                        //else if(Trans_open.Power_grade[0]==0x01)//二分之一功率
                        else   //预防解析问题，当不是四分之一也不是全功率的时候就给二分之一
                        {
                            Pre_EmissionSend(&EmissionCmd,POWER_LEVEL,2);//先设置功率等级为二分之一
                        }
                        if(run_command(xTicksToWait)==1)   //内部循环了3次
                        {
                            Pre_EmissionSend(&EmissionCmd,TUNING);//调谐
                            if(run_command(xTicksToWait*14)==1)   //调谐给7s
                            {
                                Pre_EmissionSend(&EmissionCmd,EMISSION);//发射指令
                                if(run_command(xTicksToWait)!=1)
                                {
                                    goto option_fish1;
                                }
                                else
                                {
                                    Sys_State.Emis_working=1;//系统处于发射状态
                                }
                                if(Trans_open.Type[0]!=0x01)   //不是固频
                                {
                                    Sys_State.Emis_working=0;//系统处于发射状态清零
                                    vTaskDelay(500);
                                    Pre_EmissionSend(&EmissionCmd,STOP);//停止指令
                                    if(run_command(xTicksToWait)==1)   //如果停止成功，执行第二个信道
                                    {
                                        Pre_EmissionSend(&EmissionCmd,CHANNEL,2);//调用信道2
                                        if(run_command(xTicksToWait)==1)   //内部循环了3次
                                        {
                                            if(Respond_Cmd.species!=0x03)   //如果不是调频状态，先切换到调频状态
                                            {
                                                Pre_EmissionSend(&EmissionCmd,SPECIES);//调频状态
                                                if(run_command(xTicksToWait)!=1)   //失败，goto finish
                                                {
                                                    goto option_fish1;
                                                }
                                            }
                                            Pre_EmissionSend(&EmissionCmd,FREQUENCY,2);//双频设置频率
                                            if(run_command(xTicksToWait)==1)   //内部循环了3次
                                            {
                                                if(Trans_open.Power_grade[1]==0x02)   //四分之一功率
                                                {
                                                    Pre_EmissionSend(&EmissionCmd,POWER_LEVEL,4);//先设置功率等级为四分之一
                                                }
                                                else if(Trans_open.Power_grade[1]==0x00)     //全功率
                                                {
                                                    Pre_EmissionSend(&EmissionCmd,POWER_LEVEL,1);//先设置功率等级为全功率
                                                }
                                                //else if(Trans_open.Power_grade[0]==0x01)//二分之一功率
                                                else   //预防解析问题，当不是四分之一也不是全功率的时候就给二分之一
                                                {
                                                    Pre_EmissionSend(&EmissionCmd,POWER_LEVEL,2);//先设置功率等级为二分之一
                                                }
                                                if(run_command(xTicksToWait)==1)   //内部循环了3次
                                                {
                                                    if(run_command(xTicksToWait)==1)   //内部循环了3次
                                                    {
                                                        Pre_EmissionSend(&EmissionCmd,TUNING);//调谐
                                                        if(run_command(xTicksToWait*14)==1)   //调谐给7s
                                                        {
                                                            Pre_EmissionSend(&EmissionCmd,EMISSION);//发射指令
                                                            if(run_command(xTicksToWait)==1)
                                                            {
                                                                vTaskDelay(500);
                                                                Pre_EmissionSend(&EmissionCmd,STOP);//停止指令
                                                                if(run_command(xTicksToWait)==1)
                                                                {
                                                                    if(Trans_open.Type[0]==0x02)   //是双频
                                                                    {
                                                                        Pre_EmissionSend(&EmissionCmd,MODE,2);//开启双频
                                                                        if(run_command(xTicksToWait)==1)   //如果开启成功
                                                                        {
                                                                            Pre_EmissionSend(&EmissionCmd,EMISSION);//发射双频
                                                                            if(run_command(xTicksToWait)!=1)
                                                                            {
                                                                                goto option_fish1;
                                                                            }
                                                                            else
                                                                            {
                                                                                Sys_State.Emis_working=1;//系统正在发射
                                                                            }
                                                                        }
                                                                    }
                                                                    else if(Trans_open.Type[0]==0x03)     //是三频
                                                                    {
                                                                        Pre_EmissionSend(&EmissionCmd,CHANNEL,3);//调用信道3
                                                                        if(run_command(xTicksToWait)==1)   //内部循环了3次
                                                                        {
                                                                            if(Respond_Cmd.species!=0x03)   //如果不是调频状态，先切换到调频状态
                                                                            {
                                                                                Pre_EmissionSend(&EmissionCmd,SPECIES);//调频状态
                                                                                if(run_command(xTicksToWait)!=1)   //失败，goto finish
                                                                                {
                                                                                    goto option_fish1;
                                                                                }
                                                                            }
                                                                            //调用信道成功才执行下面的:
                                                                            Pre_EmissionSend(&EmissionCmd,FREQUENCY,3);//三频设置频率
                                                                            if(run_command(xTicksToWait)==1)   //内部循环了3次
                                                                            {
                                                                                if(Trans_open.Power_grade[2]==0x02)   //四分之一功率
                                                                                {
                                                                                    Pre_EmissionSend(&EmissionCmd,POWER_LEVEL,4);//先设置功率等级为四分之一
                                                                                }
                                                                                else if(Trans_open.Power_grade[2]==0x01)     //二分之一功率
                                                                                {
                                                                                    Pre_EmissionSend(&EmissionCmd,POWER_LEVEL,2);//先设置功率等级为四分之一
                                                                                }
                                                                                else if(Trans_open.Power_grade[2]==0x00)     //全功率
                                                                                {
                                                                                    Pre_EmissionSend(&EmissionCmd,POWER_LEVEL,1);//先设置功率等级为四分之一
                                                                                }
                                                                                if(run_command(xTicksToWait)==1)
                                                                                {
                                                                                    Pre_EmissionSend(&EmissionCmd,TUNING);//调谐
                                                                                    if(run_command(xTicksToWait*14)==1)   //调谐给7s
                                                                                    {
                                                                                        Pre_EmissionSend(&EmissionCmd,EMISSION);//发射指令
                                                                                        if(run_command(xTicksToWait)==1)
                                                                                        {
                                                                                            vTaskDelay(500);
                                                                                            Pre_EmissionSend(&EmissionCmd,STOP);//停止指令
                                                                                            if(run_command(xTicksToWait)==1)
                                                                                            {
                                                                                                Pre_EmissionSend(&EmissionCmd,MODE,3);//开启三频
                                                                                                if(run_command(xTicksToWait)==1)   //如果开启成功
                                                                                                {
                                                                                                    Pre_EmissionSend(&EmissionCmd,EMISSION);//发射三频
                                                                                                    if(run_command(xTicksToWait)!=1)
                                                                                                    {
                                                                                                        goto option_fish1;
                                                                                                    }
                                                                                                    else
                                                                                                    {
                                                                                                        Sys_State.Emis_working=1;//系统正在发射
                                                                                                    }
                                                                                                }
                                                                                            }
                                                                                        }
                                                                                    }
                                                                                }
                                                                            }
                                                                        }
                                                                    }
                                                                }
                                                            }
                                                        }
                                                    }
                                                }
                                            }
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
option_fish1:
#endif
                Sys_State.FM_working=0;
            }
            else if((ulValue & BIT_1) != 0)     //开机
            {
                if((float)ADC_ConvertedValue[2]/4096*3.3*2>4)		//45V电压有电，证明已经是开机状态
                {
                    Monitor.need_open=0;
                    Sys_State.Sys_open=1;
                    App_printf("system already open !\n");
                }
                else
                {

                    if((float)ADC_ConvertedValue[2]/4096*3.3*2>4)   //45V电压有电，证明已经是开机状态
                    {
                        App_printf("system open succeed!\n");
                        Monitor.need_open=0;
                        Sys_State.Sys_open=1;
                        Sys_State.Sys_opening = 0;					//修改7s潜在BUG
                    }
                    else
                    {
                        Sys_State.Sys_close=1;
                        Sys_State.Sys_open=0;
                        Sys_State.Sys_opening = 0;					//修改7s潜在BUG
                        App_printf("system open failure!\n");
                    }
                }
            }
            else if((ulValue & BIT_2) != 0)     //保留
            {

            }
            else if((ulValue & BIT_3) != 0)     //停止
            {
                Pre_EmissionSend(&EmissionCmd,INQUIRE);//查询组包
                if(run_command(xTicksToWait))
                {
                    lock_flag1=0;
                    lock_flag2=0;
                    lock_flag3=0;
                    if(Respond_Cmd.status==0x00)   //如果是停止状态
                    {
                        //已经是停止状态
                        Sys_State.Emis_working=0;
                    }
                    else     //如果不是停止状态，发射停止指令
                    {
                        Pre_EmissionSend(&EmissionCmd,STOP);
                        if(run_command(xTicksToWait)==1)   //停止成功
                        {
//							lock_flag1=0;
//							lock_flag2=0;
//							lock_flag3=0;
                            Sys_State.Emis_working=0;
                        }
                        else     //不成功，返回停止失败，且处于报警状态
                        {
                            App_printf("Stop failure!\r\n");
                        }
                    }
                }
            }
        }
        else
        {
            App_printf("vTaskTaskTask_hard_control TIME OUT!\r\n");
        }
        if(xHandleTask_hard_monitor!=NULL)
        {
            App_printf("hard_control task Resume hard_monitor task!!!\r\n");
            vTaskResume(xHandleTask_hard_monitor);
        }
    }
}




/* 接收发射机应答
 * 任务优先级3
 */
void vTaskTaskTaskEmis_Receive(void *pvParameters)
{
    uint8_t read;

    int count =0;
    int ret=0;

//	MT2000_Cmd_Emit();
//	MT2000_Cmd_Stop();
//	MT2000_Cmd_Channel();
//	MT2000_Cmd_Power_ON();
//	MT2000_Cmd_Power_OFF();
//	MT2000_Cmd_Power_Add();
//	MT2000_Cmd_Power_Sub();
//	MT2000_Cmd_Inquire();
//	MT2000_Cmd_Tune();

    while(1)
    {
        while(comGetChar(COM4,&read))
        {
            if(count<Emis_Size)
            {
                Emission_Rx[count++]=read;
            }

            App_printf("%02x ",Emission_Rx[count-1]);
            vTaskDelay(3);
        }

        if(count>2)
        {
#if	NEW_PROTOCOL_EN
//			Emission_Rx[count] = '\0';

//			/* 检验是否收到回车换行，即命令结束标志位 */
//			if( (Emission_Rx[count-2]=='\r') && (Emission_Rx[count-1]=='\n') )		//不支持连发，即如果收到两条指令，会当成一条指令解析
//			{
            ret = MT2000_Cmd_Analyze(Emission_Rx);		//解析激励器的数据并返回结果
            count=0;
//			}
#else
            count=0;
            App_printf("\r\n");
            ret=Is_available_data(Emission_Rx);		//解析激励器的数据并返回结果
#endif

            if(ret==1)			/* 正常控制激励器 */
            {
                xEventGroupSetBits(xCreatedEventGroup, BIT_0);
                //	uxBits = xEventGroupSetBits(xCreatedEventGroup, BIT_0);
//						if((uxBits & BIT_0) == 0)//该if判断是在本任务优先级低于发射任务优先级时有效
//						{
//							App_printf("BIT_0 notice successed!\r\n");
//						}
            }
            else if(ret==-1)	/* 指令执行失败 */
            {
                /*指令执行失败*/
                App_printf("cmd failure!!!\r\n");
                xEventGroupSetBits(xCreatedEventGroup, BIT_1);
//						uxBits = xEventGroupSetBits(xCreatedEventGroup, BIT_1);
//						if((uxBits & BIT_1) == 0)//该if判断是在本任务优先级低于发射任务优先级时有效
//						{
//							App_printf("BIT_1 cmd not respond!\r\n");
//						}
            }
            else if(ret==-2)	/* 发射机报警，自动停止发射 */
            {
                /*发射机报警，自动停止发射*/
                App_printf("emission alm!!!\r\n");
                xEventGroupSetBits(xCreatedEventGroup, BIT_2);
//						uxBits = xEventGroupSetBits(xCreatedEventGroup, BIT_2);
//						if((uxBits & BIT_2) == 0)//该if判断是在本任务优先级低于发射任务优先级时有效
//						{
//							App_printf("BIT_2 Warning Stop Emit!\r\n");
//						}
            }
            else				/* 返回0，表示激励器数据无效 */
            {
                App_printf("\r\n激励器数据无效!\r\n");
                memset(Emission_Rx,0,Emis_Size);
            }

        }
        else
        {
            count=0;
            vTaskDelay(10);
        }

    }
}
/*
	发送给激励器数据组包函数，把整个需要发送的数据组合成一个数据包
	item：需要整合的结构体
	cmd：指令码,
	uint8_t inquire;//查询状态  CE
	uint8_t frequency;//工作频率 54
	uint8_t species;//工作种类   4D
	uint8_t power_level;//功率等级 50
	uint8_t channel;//信道  43
	uint8_t emission;//发射  0D
	uint8_t stop;//停止  18
	uint8_t gain_plus;//增益加 2B
	uint8_t gain_reduction;//增益减 2D
	uint8_t mode;//工作模式 44
联合体中，只有一个显性，而且是以最后一次操作为显性的，所以任意赋值一个成员都可以改变联合体的值
*/
void Pre_EmissionSend(EmissionCmd_t * item,uint8_t cmd,...)
{
    char p[10];
    uint8_t Hex_buf[6];
    uint8_t temp;
    va_list arg_ptr;
    va_start(arg_ptr,cmd);
    item->head=0x7F; //帧头

#if	NEW_PROTOCOL_EN
    MT2000_Tx.cmd = cmd;		//获取命令(为了兼容，唯一使用的地方就是run_command函数)
#endif

    switch(cmd)
    {
    case TUNING:
    {
        item->cmd.inquire=cmd;
        memset((void *)item->data,0,4);
        App_printf("Emis//调谐\r\n");
        break;
    }
    case INQUIRE:
    {
        item->cmd.inquire=cmd;
        memset((void *)item->data,0,4);
        App_printf("Emis//查询\r\n");
        break;
    }
    case FREQUENCY:
    {
        item->cmd.frequency=cmd;
        if(Sys_State.Sweeping==1)   //正在扫频
        {
            double d_temp=va_arg(arg_ptr, double);

            sprintf(p,"%.6f",d_temp);//把小数转化成字符
            //				for(int i=0;i<9;i++)
            //				{
            //					App_printf("%d ",p[i]);
            //				}
            //				App_printf("\r\n");
            uint8_t Integer;
            sscanf(p,"%02x",(unsigned int *)&Integer);//得到频率的整数部分
            App_printf("%02x \r\n",Integer);
            if(Integer>=16)   //两位数
            {
                for(int i=0; i<2; i++)
                {
                    sscanf(&p[3]+i*2,"%02x",(unsigned int *)(Hex_buf+i));
                    App_printf("%02x ",Hex_buf[i]);//得到小数部分转化成HEX16进制，可以直接发送了
                }
            }
            else     //1位数
            {
                for(int i=0; i<2; i++)
                {
                    sscanf(&p[2]+i*2,"%02x",(unsigned int *)(Hex_buf+i));
                    App_printf("%02x ",Hex_buf[i]);//得到小数部分转化成HEX16进制，可以直接发送了
                }
            }
            App_printf("\r\n");
            item->data[0]=Integer;//给出整数部分
            memcpy((void *)&item->data[1],Hex_buf,2);//小数部分赋值给激励器发送缓存区
            item->data[3]=0;
        }
        else
        {
            temp = va_arg(arg_ptr, int);
            if(temp==1)   //固频
            {
                memcpy((void *)item->data,Trans_open.Freq,4);
            }
            else if(temp==2)     //双频
            {
                memcpy((void *)item->data,Trans_open.Freq+4,4);
            }
            else if(temp==3)     //三频
            {
                memcpy((void *)item->data,Trans_open.Freq+8,4);
            }
        }
        App_printf("Emis//设置频率\r\n");
        break;
    }
    case SPECIES:
    {
        item->cmd.species=cmd;
        item->data[0]=0x03;//调频
        memset((void *)&item->data[1],0,3);
        MT2000_Tx.method = 'F';		//FM
        break;
    }
    case POWER_LEVEL:
    {
        item->cmd.power_level=cmd;
        temp = va_arg(arg_ptr, int);
        if(temp==4)   //4分之1
        {
            item->data[0]=0x02;
        }
        else if(temp==2)     //2分之1
        {
            item->data[0]=0x01;
        }
        else if(temp==1)     //全功率
        {
            item->data[0]=0x00;
        }
        memset((void *)&item->data[1],0,3);
        App_printf("Emis//设置功率为：\r\n");
        break;
    }
    case CHANNEL:
    {
        item->cmd.channel=cmd;
        temp = va_arg(arg_ptr, int);
        //使用那个全局变量中的信道
        //	if(g_fuc_cod[0]==0x02&&g_fuc_cod[1]==0x01)//查询时的功能码
        //	{
        //		item->data[0]=Working_paramet.Channel[0];//查询时的信道
        //		}
        //		else if(g_fuc_cod[0]==0x03&&g_fuc_cod[1]==0x01)//发射时的功能码
        //		{
        if(temp==1)   //固频
        {
            item->data[0]=Trans_open.Channel[0];//查询时的信道
        }
        else if(temp==2)     //双频
        {
            item->data[0]=Trans_open.Channel[1];//查询时的信道
        }
        else if(temp==3)     //三频
        {
            item->data[0]=Trans_open.Channel[2];//查询时的信道
        }
        else     //扫频信道给1
        {
            item->data[0]=0x01;//扫频时的信道，默认1信道
        }
        //	}
        memset((void *)&item->data[1],0,3);
        App_printf("Emis//信道调用\r\n");

        MT2000_Tx.channel[0] = '0';	//信道01
        MT2000_Tx.channel[1] = '1';
        MT2000_Tx.channel[2] = '\0';
        break;
    }
    case EMISSION:
    {
        item->cmd.emission=cmd;
        memset((void *)item->data,0,4);
        App_printf("Emis//发射\r\n");
        break;
    }
    case STOP:
    {
        item->cmd.stop=cmd;
        memset((void *)item->data,0,4);
        App_printf("Emis//停止发射\r\n");
        break;
    }
    case Gain_plus:
    {
        item->cmd.gain_plus=cmd;
        memset((void *)item->data,0,4);
        break;
    }
    case Gain_reduction:
    {
        item->cmd.gain_reduction=cmd;
        memset((void *)item->data,0,4);
        break;
    }
    case MODE:
    {
        item->cmd.mode=cmd;
        temp = va_arg(arg_ptr, int);
        if(temp==1)
        {
            item->data[0]=0x01;
            MT2000_Tx.mode = '1';
        }
        else if(temp==2)
        {
            item->data[0]=0x02;
            MT2000_Tx.mode = '2';
        }
        else if(temp==3)
        {
            item->data[0]=0x03;
            MT2000_Tx.mode = '3';
        }
        memset((void *)&item->data[1],0,3);
        break;
    }
    default :
    {
        item->cmd.inquire=0;//清零
        App_printf("参数错误!\r\n");
        break;
    }
    }
    //帧尾  获取异或值
    item->tail=item->head^item->cmd.inquire^item->data[0]^item->data[1]^item->data[2]^item->data[3];
    memcpy(Emission_Tx,&EmissionCmd,sizeof(EmissionCmd));
    for(int i=0; i<15; i++)   //调试信息
    {
        App_printf("%02x ",Emission_Tx[i]);
    }
    App_printf("\r\n");
    va_end(arg_ptr);
}

/*
 *串口4 发送
 *SendCommand:需要发送的指令  Len:发送长度  xTicksToWait：超时等待时间
 *const uint8_t *SendCommand本可以在调用该函数的时候把  &EmissionCmd 强制转换成uint8_t *的也可以，但是还是使用了上面的memcopy，使用Emission_Tx
 *return：1 成功，-1 激励器返回错误 -2 发射机报警
 */
static int MT2000_sendCommand(const uint8_t *SendCommand,const int Len,TickType_t xTicksToWait)
{
    EventBits_t uxBits;
    memset(Emission_Rx,0,60);
    comSendBuf(COM4,(uint8_t *)SendCommand,Len);
    uxBits = xEventGroupWaitBits(
                 xCreatedEventGroup,   /* 事件标志组句柄 */
                 BIT_0|BIT_1|BIT_2,   /* 等待bit0或bit1或bit2被设置 */
                 pdTRUE,             /* 退出前bit0和bit1被清除，这里是bit0和bit1都被设置才表示“退出”*/
                 pdFALSE ,          /* 设置为pdFALSE 表示等待任一被设置都返回*/
                 xTicksToWait); 	 /* 等待延迟时间 */

    if((uxBits & BIT_0) == BIT_0)
    {
        return 1;
    }
    else if((uxBits & BIT_1) == BIT_1)
    {
        return -1;
    }
    else if((uxBits & BIT_2) == BIT_2)
    {
        return -2;
    }
    else
    {
        App_printf("\r\n TIME OUT \r\n");
        return 0;
    }
}

/* MT2000_sendCommand	发送并且等待应答
 * MT2000_Wait_Ack		只等待实际标志组，MT2000_sendCommand的兼容版
 */
int MT2000_Wait_Ack(void)
{
    EventBits_t uxBits;
    const TickType_t delay_time = 500; /* 最大延迟500ms */

    uxBits = xEventGroupWaitBits(
                 xCreatedEventGroup,	/* 事件标志组句柄 */
                 BIT_0|BIT_1|BIT_2,		/* 等待bit0或bit1或bit2被设置 */
                 pdTRUE,				/* 退出前bit0和bit1被清除，这里是bit0和bit1都被设置才表示“退出”*/
                 pdFALSE ,				/* 设置为pdFALSE 表示等待任一被设置都返回*/
                 delay_time);			/* 等待延迟时间 */

    if((uxBits & BIT_0) == BIT_0)
    {
        return 1;
    }
    else if((uxBits & BIT_1) == BIT_1)
    {
        return -1;
    }
    else if((uxBits & BIT_2) == BIT_2)
    {
        return -2;
    }
    else
    {
        App_printf("\r\n TIME OUT \r\n");
        return 0;
    }
}

/*
  * return: 0表示数据不可用，返回1 表示正常控制激励器 返回 -1表示激励器设置失败，返回 -2表示发射机报警，自动停止发射
 */
static int Is_available_data(const uint8_t * Msg)
{
    volatile unsigned int j = 0;
    uint8_t check;
    for( j=0; j<46; j++)   //j+1 所以j<31
    {
        if(Msg[j]==0x7F && Msg[j+1]==0xFE)
        {
            break;
        }
    }
    if(j>=30)   //19
    {
        //如果有7F FE，那么后面还将有13个字节，所以j不能大于等于19，这样的话后面的数据肯定是不全或者不对的
        //所以直接返回0 代表失败
        //正常情况是一来就有数据，为了防止有时电平或者其他不可控原因，前面出现的\0,这样也有冗余作用
        //可以做到数据即使不是一开始就来真实的帧头，只要在缓存区长度内程序就可以正常运行
        return 0;
    }
    /*运行到这里证明找到了帧头*/
    /*此时 j下标= 0xFE*///错了，这里下标是0X7F
    if(Msg[j+14]==0xFD)
    {
        /*运行到这里证明找到了帧尾*/

        //数据异或校验
        check=Msg[j];
        for(int i=j+1; i<j+13; i++)
        {
            check ^=Msg[i];
        }
        if(check==Msg[j+13])
        {
            /*运行到这里已经校验通过了*/
            j=j+2;//移动给两个字节的帧头数据,此时是需要的数据位了
            memcpy((char *)&Respond_Cmd+2,Msg+j,11);
            memcpy((char *)&Respond_Cmd2,Msg+j+11+2,13);
            memcpy((char *)&Respond_Cmd3,Msg+j+11+2+13+2,13);
            return 1;
        }
    }
    if(Msg[j+4]==0xFD)
    {
        if(Msg[j+3]==0x6F && Msg[j+2]==0xEE)
        {
            //设置出错
            return -1;
        }
    }
    if(Msg[j+5]==0xFD)
    {
        if(Msg[j+4]==0xEF && Msg[j+3]==0xEE && Msg[j+2]==0xEE)
        {
            //发射机报警，自动停止发射
            return -2;
        }
    }
    return 0;
}

/*
 *扫频函数
 */
static int Band_scan(TickType_t xTicksToWait,int index)
{
//	//八段，17个点测试
//	float swr_compare[17]={3.2,3.6999,4.2,4.8499,5.5,6.3499,7.2,8.3499,9.5,10.9999,12.5,14.4999,16.5,\
//	19.1499,21.8,23.95,26.1};
    //八段
    //float swr_compare[8]={3.6999,4.8499,6.3499,8.3499,10.9999,14.4999,19.1499,23.95};

#if defined(Eight_seg)
    float swr_compare[8]= {3.6999,4.8499,6.3499,8.3499,10.9999,14.4999,19.1499,23.95};
    int SIZE =8;
#else
    int SIZE = ceil((end-begain))+1;
    float temp_begain=begain;
#endif
    float temp_Forward_power;
    float temp_Reverse_power;
    float temp_Standing_wave_ratio;

    Pre_EmissionSend(&EmissionCmd,CHANNEL,0);//调用扫频专用信道
    if(Advance_run_command(xTicksToWait)==1)
    {
        if(Respond_Cmd.species!=0x03)   //如果不是调频状态，先切换到调频状态
        {
            Pre_EmissionSend(&EmissionCmd,SPECIES);//调频状态
            if(Advance_run_command(xTicksToWait)!=1)   //失败，goto finish
            {
                goto finish;
            }
        }
        if(Respond_Cmd.power_level!=0x01)   //如果不是二分之一功率
        {
            /*先采取全部二分之一功率调整*/
            Pre_EmissionSend(&EmissionCmd,POWER_LEVEL,2);//先设置功率等级为二分之一
            if(Advance_run_command(xTicksToWait)!=1)   //失败，goto finish
            {
                goto finish;
            }
        }

        for(int i=0; i<SIZE; i++)
        {
            if(temp_begain<3.2)
            {
                temp_begain=3.2;
            }
            else if(temp_begain>26.1)
            {
                temp_begain=26.1;
            }
#if defined(Eight_seg)
            Pre_EmissionSend(&EmissionCmd,FREQUENCY,swr_compare[i]);//选择第一频段的最低值
#else
            Pre_EmissionSend(&EmissionCmd,FREQUENCY,temp_begain);//选择第一频段的最低值
            temp_begain +=1;
#endif
            if(Advance_run_command(xTicksToWait)==1)
            {
                Pre_EmissionSend(&EmissionCmd,TUNING);//调谐
                if(Advance_run_command(xTicksToWait*12)==1)   //最大给给6s
                {
                    Pre_EmissionSend(&EmissionCmd,EMISSION);//发射指令
                    if(Advance_run_command(xTicksToWait)!=1)
                    {
                        goto finish;
                    }
                    else     //ADC电压采集并判断驻波比
                    {
                        vTaskDelay(3000);//发射3s之后开始采集
                        for(int i=0; i<100; i++)
                        {
                            eADC0_buff[i] =(float) ADC_ConvertedValue[0]/4096*3.3*3;
                            eADC1_buff[i] =(float) ADC_ConvertedValue[1]/4096*3.3;
                            //	eADC2_buff[i] =(float) ADC_ConvertedValue[2]/4096*3.3*2;
                            eADC3_buff[i] =(float) ADC_ConvertedValue[3]/4096*3.3*2;
                            vTaskDelay(1);
                        }
                        for(int i=1; i<100; i++)   //100次数据求和
                        {
                            eADC0_buff[0]+=eADC0_buff[i];
                            eADC1_buff[0]+=eADC1_buff[i];
                            //	eADC2_buff[0]+=eADC2_buff[i];
                            eADC3_buff[0]+=eADC3_buff[i];
                        }
                        //取100次平均值
                        Show_Parameter.Forward_Power=eADC0_buff[0]/100;//正向功率
                        Show_Parameter.Reverse_Power=eADC1_buff[0]/100;//反向功率
                        //Show_Parameter.M_45V=eADC2_buff[0]/100;//电压
                        Show_Parameter.M_45I=eADC3_buff[0]/100;//电流

                        /*觉得应该使用功率而不是使用电压的比值，因为正反向功率比值不是线性的，如下可见不同段位驻波比不同*/
                        /*但是直接使用电压比值似乎更精确？*/
                        if(Show_Parameter.Forward_Power>5)   //正向功率的电压大于5V时，400的步进
                        {
                            Work_paraBack.Forward_power[0]=(Show_Parameter.Forward_Power-5)*Forward_P+1000+20;
                            //Work_paraBack.Reverse_power[0]=Show_Parameter.Reverse_Power*Forward_P;//反向功率倍数待现场考察
                            Work_paraBack.Reverse_power[0]=Show_Parameter.Reverse_Power*Work_paraBack.Forward_power[0]/Show_Parameter.Forward_Power;
                        }
                        else if(Show_Parameter.Forward_Power<=5&&Show_Parameter.Forward_Power>3.2)     //小于5V时，200倍的步进
                        {
                            Work_paraBack.Forward_power[0]=(Show_Parameter.Forward_Power-3.2)*277.8+500+20;
                            //Work_paraBack.Reverse_power[0]=Show_Parameter.Reverse_Power*277.8;//反向功率倍数待现场考察
                            Work_paraBack.Reverse_power[0]=Show_Parameter.Reverse_Power*Work_paraBack.Forward_power[0]/Show_Parameter.Forward_Power;
                        }
                        else     //小于3.2v
                        {
                            Work_paraBack.Forward_power[0]=(Show_Parameter.Forward_Power)*500/3.2+20;
                            //Work_paraBack.Reverse_power[0]=Show_Parameter.Reverse_Power*500/3.2;//反向功率倍数待现场考察
                            Work_paraBack.Reverse_power[0]=Show_Parameter.Reverse_Power*Work_paraBack.Forward_power[0]/Show_Parameter.Forward_Power;
                        }
                        Show_Parameter.Standing_wave_ratio =get_Standing_wave_ratio(Work_paraBack.Forward_power[0],Work_paraBack.Reverse_power[0]);

                        if(GPIO_ReadInputDataBit(In_GPIO_PORT,In_GPIO_PIN)==1)   //PF0
                        {
                            App_printf("Sys_State.alarm\r\n");
                            Sys_State.alarm=1;
                            App_printf("the first alarm,needing save RTC time!!!\r\n");
                            Time_Display( RTC_GetCounter(),&set_time);
                            Alarm_historyBack.Exciter_alarm[0]=set_time.tm_year-2000;
                            Alarm_historyBack.Exciter_alarm[1]=set_time.tm_mon;
                            Alarm_historyBack.Exciter_alarm[2]=set_time.tm_mday;
                            Alarm_historyBack.Exciter_alarm[3]=set_time.tm_hour;
                            Alarm_historyBack.Exciter_alarm[4]=set_time.tm_min;
                            Alarm_historyBack.Exciter_alarm[5]=set_time.tm_sec;
                        }
                        else
                        {
                            if(Show_Parameter.Standing_wave_ratio>=2.0||Show_Parameter.Standing_wave_ratio<1)
                            {
                                //需要立即停止
                                //SWR_array[i]=Show_Parameter.Standing_wave_ratio;
                            }
                            else
                            {
                                for(int count=0; count<7; count++)
                                {
                                    //延时10s等待机器稳定再采集
                                    vTaskDelay(900);//发射1s之后再次开始采集
                                    for(int i=0; i<100; i++)
                                    {
                                        eADC0_buff[i] =(float) ADC_ConvertedValue[0]/4096*3.3*3;
                                        eADC1_buff[i] =(float) ADC_ConvertedValue[1]/4096*3.3;
                                        //	eADC2_buff[i] =(float) ADC_ConvertedValue[2]/4096*3.3*2;
                                        eADC3_buff[i] =(float) ADC_ConvertedValue[3]/4096*3.3*2;
                                        vTaskDelay(1);
                                    }
                                    for(int i=1; i<100; i++)   //100次数据求和
                                    {
                                        eADC0_buff[0]+=eADC0_buff[i];
                                        eADC1_buff[0]+=eADC1_buff[i];
                                        //	eADC2_buff[0]+=eADC2_buff[i];
                                        eADC3_buff[0]+=eADC3_buff[i];
                                    }
                                    //取100次平均值
                                    Show_Parameter.Forward_Power=eADC0_buff[0]/100;//正向功率
                                    Show_Parameter.Reverse_Power=eADC1_buff[0]/100;//反向功率
                                    //Show_Parameter.M_45V=eADC2_buff[0]/100;//电压
                                    Show_Parameter.M_45I=eADC3_buff[0]/100;//电流
//							Show_Parameter.M_45V=(float) ADC_ConvertedValue[2]/4096*3.3;
//							Show_Parameter.M_45I =(float) ADC_ConvertedValue[3]/4096*3.3;
                                    /*觉得应该使用功率而不是使用电压的比值，因为正反向功率比值不是线性的，如下可见不同段位驻波比不同*/
                                    /*但是直接使用电压比值似乎更精确？*/
                                    if(Show_Parameter.M_45I>1.5)   //2.3-1.5V=0.8V =>对应 30A，步进37.5
                                    {
                                        Work_paraBack.Power_45_intensity[0]=(Show_Parameter.M_45I-1.5)*Debug_M_45I+60;
                                    }
                                    else     //小于 1.5V电流系数为40
                                    {
                                        Work_paraBack.Power_45_intensity[0]=Show_Parameter.M_45I*40;
                                    }
                                    if(Show_Parameter.Forward_Power>5)   //正向功率的电压大于5V时，400的步进
                                    {
                                        Work_paraBack.Forward_power[0]=(Show_Parameter.Forward_Power-5)*Forward_P+1000+20;
                                        //Work_paraBack.Reverse_power[0]=Show_Parameter.Reverse_Power*Forward_P;//反向功率倍数待现场考察
                                        Work_paraBack.Reverse_power[0]=Show_Parameter.Reverse_Power*Work_paraBack.Forward_power[0]/Show_Parameter.Forward_Power;
                                    }
                                    else if(Show_Parameter.Forward_Power<=5&&Show_Parameter.Forward_Power>3.2)     //小于5V时，200倍的步进
                                    {
                                        Work_paraBack.Forward_power[0]=(Show_Parameter.Forward_Power-3.2)*277.8+500+20;
                                        //Work_paraBack.Reverse_power[0]=Show_Parameter.Reverse_Power*277.8;//反向功率倍数待现场考察
                                        Work_paraBack.Reverse_power[0]=Show_Parameter.Reverse_Power*Work_paraBack.Forward_power[0]/Show_Parameter.Forward_Power;
                                    }
                                    else     //小于3.2v
                                    {
                                        Work_paraBack.Forward_power[0]=(Show_Parameter.Forward_Power)*500/3.2+20;
                                        //Work_paraBack.Reverse_power[0]=Show_Parameter.Reverse_Power*500/3.2;//反向功率倍数待现场考察
                                        Work_paraBack.Reverse_power[0]=Show_Parameter.Reverse_Power*Work_paraBack.Forward_power[0]/Show_Parameter.Forward_Power;
                                    }
                                    Show_Parameter.Standing_wave_ratio =get_Standing_wave_ratio(Work_paraBack.Forward_power[0],Work_paraBack.Reverse_power[0]);
                                    if(Show_Parameter.Standing_wave_ratio>=1.8||Show_Parameter.Standing_wave_ratio<1 || Work_paraBack.Power_45_intensity[0]>=100)
                                    {
//								z[i]=temp_Forward_power;
//								f[i]=temp_Reverse_power;
                                        SWR_array[i*10+index]=Show_Parameter.Standing_wave_ratio;
                                        goto s_finish;
                                    }
                                    else
                                    {
//							 	z[i]=temp_Forward_power;
//								f[i]=temp_Reverse_power;
                                        SWR_array[i*10+index]=Show_Parameter.Standing_wave_ratio;
                                    }
                                }
                                for(int count=0; count<30; count++)   //最多加30次
                                {
                                    Pre_EmissionSend(&EmissionCmd,Gain_plus);//增益加指令
                                    if(run_command(xTicksToWait*2)==1)   //如果加成功
                                    {
                                        //延时10s等待机器稳定再采集
                                        vTaskDelay(300);//发射300ms之后再次开始采集
                                        for(int i=0; i<100; i++)
                                        {
                                            eADC0_buff[i] =(float) ADC_ConvertedValue[0]/4096*3.3*3;
                                            eADC1_buff[i] =(float) ADC_ConvertedValue[1]/4096*3.3;
                                            //	eADC2_buff[i] =(float) ADC_ConvertedValue[2]/4096*3.3*2;
                                            eADC3_buff[i] =(float) ADC_ConvertedValue[3]/4096*3.3*2;
                                            vTaskDelay(1);
                                        }
                                        for(int i=1; i<100; i++)   //100次数据求和
                                        {
                                            eADC0_buff[0]+=eADC0_buff[i];
                                            eADC1_buff[0]+=eADC1_buff[i];
                                            //	eADC2_buff[0]+=eADC2_buff[i];
                                            eADC3_buff[0]+=eADC3_buff[i];
                                        }
                                        //取100次平均值
                                        Show_Parameter.Forward_Power=eADC0_buff[0]/100;//正向功率
                                        Show_Parameter.Reverse_Power=eADC1_buff[0]/100;//反向功率
                                        //Show_Parameter.M_45V=eADC2_buff[0]/100;//电压
                                        Show_Parameter.M_45I=eADC3_buff[0]/100;//电流
                                        //电流显示
                                        if(Show_Parameter.M_45I>1.5)   //2.3-1.5V=0.8V =>对应 30A，步进37.5
                                        {
                                            Work_paraBack.Power_45_intensity[0]=(Show_Parameter.M_45I-1.5)*Debug_M_45I+60;
                                        }
                                        else     //小于 1.5V电流系数为40
                                        {
                                            Work_paraBack.Power_45_intensity[0]=Show_Parameter.M_45I*40;
                                        }
                                        if(Show_Parameter.Forward_Power>5)   //正向功率的电压大于5V时，400的步进
                                        {
                                            temp_Forward_power=Work_paraBack.Forward_power[0];
                                            temp_Reverse_power=Work_paraBack.Reverse_power[0];
                                            Work_paraBack.Forward_power[0]=(Show_Parameter.Forward_Power-5)*Forward_P+1000+20;
                                            //Work_paraBack.Reverse_power[0]=Show_Parameter.Reverse_Power*Forward_P;//反向功率倍数待现场考察
                                            Work_paraBack.Reverse_power[0]=Show_Parameter.Reverse_Power*Work_paraBack.Forward_power[0]/Show_Parameter.Forward_Power;
                                        }
                                        else if(Show_Parameter.Forward_Power<=5&&Show_Parameter.Forward_Power>3.2)     //小于5V时，200倍的步进
                                        {
                                            temp_Forward_power=Work_paraBack.Forward_power[0];
                                            temp_Reverse_power=Work_paraBack.Reverse_power[0];
                                            Work_paraBack.Forward_power[0]=(Show_Parameter.Forward_Power-3.2)*277.8+500+20;
                                            //Work_paraBack.Reverse_power[0]=Show_Parameter.Reverse_Power*277.8;//反向功率倍数待现场考察
                                            Work_paraBack.Reverse_power[0]=Show_Parameter.Reverse_Power*Work_paraBack.Forward_power[0]/Show_Parameter.Forward_Power;
                                        }
                                        else     //小于3.2v
                                        {
                                            temp_Forward_power=Work_paraBack.Forward_power[0];
                                            temp_Reverse_power=Work_paraBack.Reverse_power[0];
                                            Work_paraBack.Forward_power[0]=(Show_Parameter.Forward_Power)*500/3.2+20;
                                            //Work_paraBack.Reverse_power[0]=Show_Parameter.Reverse_Power*500/3.2;//反向功率倍数待现场考察
                                            Work_paraBack.Reverse_power[0]=Show_Parameter.Reverse_Power*Work_paraBack.Forward_power[0]/Show_Parameter.Forward_Power;
                                        }
//							temp_Standing_wave_ratio=Show_Parameter.Standing_wave_ratio ;
                                        temp_Standing_wave_ratio =get_Standing_wave_ratio(Work_paraBack.Forward_power[0],Work_paraBack.Reverse_power[0]);

                                        //	if(Work_paraBack.Forward_power[0]>1500||(fabs(temp_Forward_power-Work_paraBack.Forward_power[0])<=5)\//
                                        if(Work_paraBack.Forward_power[0]>1500 ||temp_Standing_wave_ratio>=1.8||temp_Standing_wave_ratio<1 || Work_paraBack.Power_45_intensity[0]>=100)
                                        {
                                            z[i*10+index]=temp_Forward_power;
                                            f[i*10+index]=temp_Reverse_power;
//								SWR_array[i]=temp_Standing_wave_ratio;
                                            goto s_finish;
                                        }
                                        else
                                        {
                                            z[i*10+index]=Work_paraBack.Forward_power[0];
                                            f[i*10+index]=Work_paraBack.Reverse_power[0];
//								SWR_array[i]=Show_Parameter.Standing_wave_ratio;
                                        }
                                    }
                                }
                            }
                        }
s_finish:
                        z[i*10+index]=Work_paraBack.Forward_power[0];//212
                        f[i*10+index]=Work_paraBack.Reverse_power[0];
                        SWR_array[i*10+index]=Show_Parameter.Standing_wave_ratio;
                        App_printf("Forward_power=%f \r\n",z[i*10+index]);
                        App_printf("Reverse_power=%f \r\n",f[i*10+index]);
                        App_printf("Standing_wave_ratio=%f \r\n",SWR_array[i*10+index]);
                        Pre_EmissionSend(&EmissionCmd,STOP);//停止指令
                        if(Advance_run_command(xTicksToWait)!=1)
                        {
                            goto finish;
                        }
                        else
                        {
                            if(i!=SIZE-1)
                            {
                                vTaskDelay(10000);//延时10s之后开机
                            }
                            if(i<SIZE-1)
                            {
                                Sys_State.Sweeping=1;//正在扫频
                            }
                            else if(i==SIZE-1)
                            {
                                get_better_SWR(SWR_array,index);//获取更佳驻波比，置位不合理驻波比包的字段
                                if(index==9)   //2
                                {
                                    //judge_fre_segment();//置位驻波比包的字段
                                    if(InternalFlash_SaveData_2()==1)
                                    {
                                        Flash2_to_AcceptAPP();
                                        App_printf("flash save successed!\r\n");
                                        clean_sweeping=1;
                                    }
                                    else
                                    {
                                        App_printf("flash save error!\r\n");
                                        clean_sweeping=1;
                                    }
//									Sys_State.Sweeping=0;
//									Sys_State.Already_Swept=1;//扫频结束
                                }
                            }
                        }
                    }
                }
                else
                {
                    goto finish;
                }
            }
            else
            {
                goto finish;
            }
        }
    }
finish:


    return 0;
}


/*
 * return: 1 正常，-2 发射机报警 ，其余情况归于激励器无响应
 */
static int run_command(TickType_t xTicksToWait)
{
    int ret=0;

    //循环三次无响应，就报警？？？
    for(int i=0; i<2; i++)
    {
#if	NEW_PROTOCOL_EN
        switch( MT2000_Tx.cmd )
        {
        case EMISSION:		//发射
            MT2000_Cmd_Emit();
            ret = MT2000_Wait_Ack();
            break;

        case STOP:			//停止
            MT2000_Cmd_Stop();
            ret = MT2000_Wait_Ack();
            break;

        case CHANNEL:		//调用信道
            MT2000_Cmd_Channel();
            ret = MT2000_Wait_Ack();
            break;

        case POWER_ADD:		//功率增加(微调)
            MT2000_Cmd_Power_Add();
            ret = MT2000_Wait_Ack();
            break;

        case POWER_SUB:		//功率减小(微调)
            MT2000_Cmd_Power_Sub();
            ret = MT2000_Wait_Ack();
            break;

        case INQUIRE:		//查询
            MT2000_Cmd_Inquire();
            ret = MT2000_Wait_Ack();
            break;

        case TUNING:		//调谐
            MT2000_Cmd_Tune();
            ret = MT2000_Wait_Ack();
            break;

            /* 以下几个，不处理		没有相应命令，只在设置信道的时候进行设置 */
        case FREQUENCY:		//频率-------------------------------------------
            ret = 1;
            break;

        case SPECIES:		//工作种类---------------------------------------
            ret = 1;
            break;

        case POWER_LEVEL:	//功率等级---------------------------------------
            ret = 1;
            break;

        case MODE:			//工作模式---------------------------------------
            ret = 1;
            break;
        }
#else
        ret=MT2000_sendCommand(Emission_Tx,sizeof(EmissionCmd),xTicksToWait);
#endif

        if(ret==1)
        {
            //正常时需要取消历史的报警标志
            Sys_State.no_respond=0;
            Alarm_historyBack.alarm_history[0] &=~(1<<7);
            //Sys_State.alarm=0;上面的无响应可以取消。但是激励器报警状态必须是PC取消
            /*这个应该不打印*/
            //App_printf("//查询 goto finish;\r\n");
            return ret;
        }
        if(ret==-2)
        {
            if(Sys_State.alarm==0)
            {
                Sys_State.alarm=1;
                App_printf("the first alarm,needing save RTC time!!!\r\n");
                Time_Display( RTC_GetCounter(),&set_time);
                Alarm_historyBack.Exciter_alarm[0]=set_time.tm_year-2000;
                Alarm_historyBack.Exciter_alarm[1]=set_time.tm_mon;
                Alarm_historyBack.Exciter_alarm[2]=set_time.tm_mday;
                Alarm_historyBack.Exciter_alarm[3]=set_time.tm_hour;
                Alarm_historyBack.Exciter_alarm[4]=set_time.tm_min;
                Alarm_historyBack.Exciter_alarm[5]=set_time.tm_sec;
                /*这里需要获取相对应的RTC时间，然后赋值给0602包里面对应的字段*/
                /*这里需要获取相对应的RTC时间，然后赋值给0602包里面对应的字段*/
                /*这里需要获取相对应的RTC时间，然后赋值给0602包里面对应的字段*/
            }
            App_printf("alarm alarm alarm!!!\r\n");
            return ret;
        }
        if(ret==-1 || ret==0)
        {
            if(i==2)
            {
//					Sys_State.no_respond=1;
//					App_printf("Sys_State.no_respond =%d\r\n",Sys_State.no_respond);
//					return ret;
                break;
            }
        }
    }

    if(Sys_State.no_respond==0)
    {
        Sys_State.no_respond=1;
        App_printf("the first no_respond,needing save RTC time!!!\r\n");
        Time_Display( RTC_GetCounter(),&set_time);
        Alarm_historyBack.No_response_time[0]=set_time.tm_year-2000;
        Alarm_historyBack.No_response_time[1]=set_time.tm_mon;
        Alarm_historyBack.No_response_time[2]=set_time.tm_mday;
        Alarm_historyBack.No_response_time[3]=set_time.tm_hour;
        Alarm_historyBack.No_response_time[4]=set_time.tm_min;
        Alarm_historyBack.No_response_time[5]=set_time.tm_sec;
    }
//	Sys_State.no_respond=1;
//	App_printf("Sys_State.no_respond =%d\r\n",Sys_State.no_respond);
    return ret;		//为了不使用这个函数屏蔽了的模式，以前那样最后return 0是没有意义的
//	return 0;	//正常情况下，不可能执行到这里
}

/*
 *run_command函数的加强版，主要是为了能够在扫频的时候加一个判断，能迅速推出扫描
 */
int Advance_run_command(TickType_t xTicksToWait)
{
    int ret=0;
    if(Sys_State.Cancel_Sweeping!=1)   //PC没有强制取消扫频
    {
        ret=run_command(xTicksToWait);
    }
    return ret;
}
/*返回1表示有历史报警消息*/
int get_history_alm(void)
{
    volatile uint8_t* check=&Sys_State.SWR_alarm;
    for(int i=0; i<9; i++)
    {
        if(*check++==1)
        {
            Sys_State.History_alarm=1;//清除只能由PC清除
            return 1;
        }

    }
    return 0;
//	if(Sys_State.alarm==1 || Sys_State.no_respond==1)
//	{
//		//还需要加上ADC采集的数据进行判断，这里暂时先不写
//		//还需要加上ADC采集的数据进行判断，这里暂时先不写
//		//还需要加上ADC采集的数据进行判断，这里暂时先不写
//		Sys_State.History_alarm=1;//清除只能由PC清除
//		return 1;
//	}
//	return 0;
}

/*返回1表示当前有报警消息*/
int get_current_alm(void)
{
    if(Sys_State.alarm==1 || Sys_State.no_respond==1)
    {
        /*感觉上面的if判断激励器相关的应该不使用，而只使用AD采集的*/
        /*感觉上面的if判断激励器相关的应该不使用，而只使用AD采集的*/
        /*感觉上面的if判断激励器相关的应该不使用，而只使用AD采集的*/
        return 1;
    }
    return 0;
}

void clean_all_flags(void)
{
    Sys_State.History_alarm=0;//历史报警标志

    Sys_State.alarm=0;//发射机自身报警

    Sys_State.no_respond=0;//激励器无响应报警

    Sys_State.SWR_alarm=0;//驻波比报警

    Sys_State.low_Voltage=0;//欠压报警

    Sys_State.over_Voltage=0;//过压报警

    Sys_State.over_Electric=0;//过流报警

    Sys_State.humidity_alarm=0;//湿度报警

    Sys_State.temperature_alarm=0;//温度报警

    Sys_State.No_power_level=0;//无功率输出报警

//	Alarm_historyBack.alarm_history[0]=0;//清零
//	Alarm_historyBack.alarm_history[1]=0;
    //历史报警包全部数据清零
    memset(&Alarm_historyBack,0,sizeof(Alarm_historyBack));

}

void clean_alarm_bit_flags(void)
{
    Alarm_historyBack.alarm_history[0]=DisalarmBack.current_alarm_state[0];
    Alarm_historyBack.alarm_history[1]=DisalarmBack.current_alarm_state[1];
    if((Alarm_historyBack.alarm_history[0]&(1<<0))==0)
    {
        Sys_State.No_power_level=0;
    }
    if((Alarm_historyBack.alarm_history[0]&(1<<1))==0)
    {
        Sys_State.temperature_alarm=0;
    }
    if((Alarm_historyBack.alarm_history[0]&(1<<2))==0)
    {
        Sys_State.humidity_alarm=0;
    }
    if((Alarm_historyBack.alarm_history[0]&(1<<3))==0)
    {
        Sys_State.over_Electric=0;
    }
    if((Alarm_historyBack.alarm_history[0]&(1<<4))==0)
    {
        Sys_State.over_Voltage=0;
    }
    if((Alarm_historyBack.alarm_history[0]&(1<<5))==0)
    {
        Sys_State.low_Voltage=0;
    }
    if((Alarm_historyBack.alarm_history[0]&(1<<6))==0)
    {
        Sys_State.SWR_alarm=0;
    }
    if((Alarm_historyBack.alarm_history[0]&(1<<7))==0)
    {
        Sys_State.no_respond=0;
    }
    if((Alarm_historyBack.alarm_history[1]&(1<<0))==0)
    {
        Sys_State.alarm=0;
    }
}
/*获取驻波比函数，返回驻波比
 *Forward_Power:正向功率(其实是电压值)   Reverse_Power：反向功率
 */
float get_Standing_wave_ratio(float Forward_Power,float Reverse_Power)
{
    float temp;
    if(Forward_Power==0 || Forward_Power==Reverse_Power)
    {
        return -1;
    }
    temp=(float)sqrt(Reverse_Power/Forward_Power);
    return (1+temp)/(1-temp);
}
/*
 *获取最佳驻波比,置位不合理的驻波比频段，并返回最佳频段个数
 */
static int get_better_SWR(float *array,int index)
{
    //float *temp=array;
    //float min=0;
    int min_count=0;
#if defined(Eight_seg)
    int SIZE =8;
#else
    int SIZE = ceil((end-begain))+1;
#endif
    App_printf("SIZE:%d\r\n",SIZE);
    App_printf("swr:");
    for(int i=0; i<SIZE; i++)
    {
        App_printf("%f ",array[i*10+index]);
    }
    App_printf("\r\n\r\n");

    App_printf("Forward_Power:");
    for(int i=0; i<SIZE; i++)
    {
        App_printf("%f ",z[i*10+index]);
        if(z[i*10+index]<1100)
        {
            z[i*10+index]=0;
        }
        else if(z[i*10+index]>=1550)
        {
            z[i*10+index]=1550;
        }

    }
    App_printf("\r\n\r\n");

//App_printf("Reverse_Power:");
//	for(int i=0;i<SIZE;i++)
//	{
//		App_printf("%f ",f[i]);
//	}
//	App_printf("\r\n\r\n");

    return min_count;
}
/*
 *判断最佳频段是第几段
 */
//static int judge_fre_segment(void)
//{
//	return 0;
//}

/*
 *Save_count:实参为flash3_Save，即运行图存储了flash几次
 */
int find_hard_control(uint8_t Save_count)
{
    //Hard_control.usage_diagram_count=0;//进入清零
    for(int i=0; i<Save_count; i++)
    {
        //for(int j=0;j<10;j++)
        //{
        //第一个
        if(State_monitor[i].start[0]!=1)
        {
            if(Run_Diagram_buff[i].Start_Time1[0]!=0xFF&&((set_time.tm_hour*60+set_time.tm_min)>=(Run_Diagram_buff[i].Start_Time1[0]*60+Run_Diagram_buff[i].Start_Time1[1]))\
                    &&((set_time.tm_hour*60+set_time.tm_min)<(Run_Diagram_buff[i].End_Timep1[0]*60+Run_Diagram_buff[i].End_Timep1[1])))
            {
                //	if(set_time.tm_min== Run_Diagram_buff[i].Start_Time1[1]||(set_time.tm_hour*60+set_time.tm_min)\
                //										<(Run_Diagram_buff[i].End_Timep1[0]*60+Run_Diagram_buff[i].End_Timep1[1]))
                //	{
                App_printf("\r\nthe 1 start");
                State_monitor[i].start[0]=1;
                State_monitor[i].end[0]=0;
                Hard_control.usage_diagram_count++;
                if(Hard_control.usage_diagram_count==1)
                {
                    Trans_open_Copy.Type[0]=1;
                    Trans_open_Copy.Channel[0]=1;
                    Trans_open_Copy.Power_grade[0]=Run_Diagram_buff[i].Power1[0];
                    memcpy(Trans_open_Copy.Freq,Run_Diagram_buff[i].Frq1,4);
                }
                else if(Hard_control.usage_diagram_count==2)
                {
                    Trans_open_Copy.Type[0]=2;
                    Trans_open_Copy.Channel[1]=2;
                    Trans_open_Copy.Power_grade[1]=Run_Diagram_buff[i].Power1[0];
                    memcpy((uint8_t *)(Trans_open_Copy.Freq)+4,Run_Diagram_buff[i].Frq1,4);
                }
                else if(Hard_control.usage_diagram_count==3)
                {
                    Trans_open_Copy.Type[0]=3;
                    Trans_open_Copy.Channel[2]=3;
                    Trans_open_Copy.Power_grade[2]=Run_Diagram_buff[i].Power1[0];
                    memcpy((uint8_t *)(Trans_open_Copy.Freq)+8,Run_Diagram_buff[i].Frq1,4);
                }
                //}
            }

        }

        if(State_monitor[i].end[0]!=1)   //结束
        {
            if(set_time.tm_hour== Run_Diagram_buff[i].End_Timep1[0])
            {
                if(set_time.tm_min== Run_Diagram_buff[i].End_Timep1[1])
                {
                    App_printf("\r\nthe 1  end");
                    Hard_control.usage_diagram_count--;
                    State_monitor[i].end[0]=1;
                    State_monitor[i].start[0]=0;//清除标志
                    if(Trans_open_Copy.Type[0]==1)
                    {
                        Trans_open_Copy.Type[0]=0;
                        //固频时，时间到，停止即可
                        memset(Trans_open_Copy.Freq,0,12);//清零
                    }
                    if(Trans_open_Copy.Type[0]==2)
                    {
                        Trans_open_Copy.Type[0]=1;
                        if(memcmp(Trans_open_Copy.Freq,Run_Diagram_buff[i].Frq1,4)==0)   //踢出第一个
                        {
                            Trans_open_Copy.Freq[0]=Trans_open_Copy.Freq[4];
                            Trans_open_Copy.Freq[1]=Trans_open_Copy.Freq[5];
                            Trans_open_Copy.Freq[2]=Trans_open_Copy.Freq[6];
                            //最后1byte都为0，所以不处理
                        }
                        //如果是踢出第二个，不需要做处理了
                    }
                    if(Trans_open_Copy.Type[0]==3)
                    {
                        Trans_open_Copy.Type[0]=2;
                        if(memcmp(Trans_open_Copy.Freq,Run_Diagram_buff[i].Frq1,4)==0)   //踢出第一个
                        {
                            Trans_open_Copy.Freq[0]=Trans_open_Copy.Freq[4];
                            Trans_open_Copy.Freq[1]=Trans_open_Copy.Freq[5];
                            Trans_open_Copy.Freq[2]=Trans_open_Copy.Freq[6];
                            //最后1byte都为0，所以不处理
                            Trans_open_Copy.Freq[4]=Trans_open_Copy.Freq[8];
                            Trans_open_Copy.Freq[5]=Trans_open_Copy.Freq[9];
                            Trans_open_Copy.Freq[6]=Trans_open_Copy.Freq[10];
                        }
                        else if(memcmp((uint8_t *)(Trans_open_Copy.Freq)+4,Run_Diagram_buff[i].Frq1,4)==0)     //踢出第二个
                        {
                            //最后1byte都为0，所以不处理
                            Trans_open_Copy.Freq[4]=Trans_open_Copy.Freq[8];
                            Trans_open_Copy.Freq[5]=Trans_open_Copy.Freq[9];
                            Trans_open_Copy.Freq[6]=Trans_open_Copy.Freq[10];
                        }
                        //如果是踢出第三个，不需要做处理了
                    }
                }
            }
        }
        //第二个
        if(State_monitor[i].start[1]!=1)
        {
            if(Run_Diagram_buff[i].Start_Time2[0]!=0xFF&&((set_time.tm_hour*60+set_time.tm_min)>=(Run_Diagram_buff[i].Start_Time2[0]*60+Run_Diagram_buff[i].Start_Time2[1]))\
                    &&((set_time.tm_hour*60+set_time.tm_min)<(Run_Diagram_buff[i].End_Timep2[0]*60+Run_Diagram_buff[i].End_Timep2[1])))
            {
//					if(set_time.tm_min== Run_Diagram_buff[i].Start_Time2[1]||(set_time.tm_hour*60+set_time.tm_min)\
//														<(Run_Diagram_buff[i].End_Timep2[0]*60+Run_Diagram_buff[i].End_Timep2[1]))
//					{
                App_printf("\r\nthe 2 start");
                State_monitor[i].start[1]=1;
                State_monitor[i].end[1]=0;
                Hard_control.usage_diagram_count++;
                if(Hard_control.usage_diagram_count==1)
                {
                    Trans_open_Copy.Type[0]=1;
                    Trans_open_Copy.Channel[0]=1;
                    Trans_open_Copy.Power_grade[0]=Run_Diagram_buff[i].Power2[0];
                    memcpy(Trans_open_Copy.Freq,Run_Diagram_buff[i].Frq2,4);
                }
                else if(Hard_control.usage_diagram_count==2)
                {
                    Trans_open_Copy.Type[0]=2;
                    Trans_open_Copy.Channel[1]=2;
                    Trans_open_Copy.Power_grade[1]=Run_Diagram_buff[i].Power2[0];
                    memcpy((uint8_t *)(Trans_open_Copy.Freq)+4,Run_Diagram_buff[i].Frq2,4);
                }
                else if(Hard_control.usage_diagram_count==3)
                {
                    Trans_open_Copy.Type[0]=3;
                    Trans_open_Copy.Channel[2]=3;
                    Trans_open_Copy.Power_grade[2]=Run_Diagram_buff[i].Power2[0];
                    memcpy((uint8_t *)(Trans_open_Copy.Freq)+8,Run_Diagram_buff[i].Frq2,4);
                }
                //	}
            }

        }

        if(State_monitor[i].end[1]!=1)   //结束
        {
            if(set_time.tm_hour== Run_Diagram_buff[i].End_Timep2[0])
            {
                if(set_time.tm_min== Run_Diagram_buff[i].End_Timep2[1])
                {
                    App_printf("\r\nthe 2 end");
                    Hard_control.usage_diagram_count--;
                    State_monitor[i].end[1]=1;
                    State_monitor[i].start[1]=0;
                    if(Trans_open_Copy.Type[0]==1)
                    {
                        Trans_open_Copy.Type[0]=0;
                        //固频时，时间到，停止即可
                        memset(Trans_open_Copy.Freq,0,12);//清零
                    }
                    if(Trans_open_Copy.Type[0]==2)
                    {
                        Trans_open_Copy.Type[0]=1;
                        if(memcmp(Trans_open_Copy.Freq,Run_Diagram_buff[i].Frq2,4)==0)   //踢出第一个
                        {
                            Trans_open_Copy.Freq[0]=Trans_open_Copy.Freq[4];
                            Trans_open_Copy.Freq[1]=Trans_open_Copy.Freq[5];
                            Trans_open_Copy.Freq[2]=Trans_open_Copy.Freq[6];
                            //最后1byte都为0，所以不处理
                        }
                        //如果是踢出第二个，不需要做处理了
                    }
                    if(Trans_open_Copy.Type[0]==3)
                    {
                        Trans_open_Copy.Type[0]=2;
                        if(memcmp(Trans_open_Copy.Freq,Run_Diagram_buff[i].Frq2,4)==0)   //踢出第一个
                        {
                            Trans_open_Copy.Freq[0]=Trans_open_Copy.Freq[4];
                            Trans_open_Copy.Freq[1]=Trans_open_Copy.Freq[5];
                            Trans_open_Copy.Freq[2]=Trans_open_Copy.Freq[6];
                            //最后1byte都为0，所以不处理
                            Trans_open_Copy.Freq[4]=Trans_open_Copy.Freq[8];
                            Trans_open_Copy.Freq[5]=Trans_open_Copy.Freq[9];
                            Trans_open_Copy.Freq[6]=Trans_open_Copy.Freq[10];
                        }
                        else if(memcmp((uint8_t *)(Trans_open_Copy.Freq)+4,Run_Diagram_buff[i].Frq2,4)==0)     //踢出第二个
                        {
                            //最后1byte都为0，所以不处理
                            Trans_open_Copy.Freq[4]=Trans_open_Copy.Freq[8];
                            Trans_open_Copy.Freq[5]=Trans_open_Copy.Freq[9];
                            Trans_open_Copy.Freq[6]=Trans_open_Copy.Freq[10];
                        }
                        //如果是踢出第三个，不需要做处理了
                    }
                }
            }
        }
        //第三个
        if(State_monitor[i].start[2]!=1)
        {
            if(Run_Diagram_buff[i].Start_Time3[0]!=0xFF&&((set_time.tm_hour*60+set_time.tm_min)>=(Run_Diagram_buff[i].Start_Time3[0]*60+Run_Diagram_buff[i].Start_Time3[1]))\
                    &&((set_time.tm_hour*60+set_time.tm_min)<(Run_Diagram_buff[i].End_Timep3[0]*60+Run_Diagram_buff[i].End_Timep3[1])))
            {
//					if(set_time.tm_min== Run_Diagram_buff[i].Start_Time3[1]||(set_time.tm_hour*60+set_time.tm_min)\
//														<(Run_Diagram_buff[i].End_Timep3[0]*60+Run_Diagram_buff[i].End_Timep3[1]))
//					{
                App_printf("\r\nthe 3");
                State_monitor[i].start[2]=1;
                State_monitor[i].end[2]=0;
                Hard_control.usage_diagram_count++;
                if(Hard_control.usage_diagram_count==1)
                {
                    Trans_open_Copy.Type[0]=1;
                    Trans_open_Copy.Channel[0]=1;
                    Trans_open_Copy.Power_grade[0]=Run_Diagram_buff[i].Power3[0];
                    memcpy(Trans_open_Copy.Freq,Run_Diagram_buff[i].Frq3,4);
                }
                else if(Hard_control.usage_diagram_count==2)
                {
                    Trans_open_Copy.Type[0]=2;
                    Trans_open_Copy.Channel[1]=2;
                    Trans_open_Copy.Power_grade[1]=Run_Diagram_buff[i].Power3[0];
                    memcpy((uint8_t *)(Trans_open_Copy.Freq)+4,Run_Diagram_buff[i].Frq3,4);
                }
                else if(Hard_control.usage_diagram_count==3)
                {
                    Trans_open_Copy.Type[0]=3;
                    Trans_open_Copy.Channel[2]=3;
                    Trans_open_Copy.Power_grade[2]=Run_Diagram_buff[i].Power3[0];
                    memcpy((uint8_t *)(Trans_open_Copy.Freq)+8,Run_Diagram_buff[i].Frq3,4);
                }
                //	}
            }

        }
        if(State_monitor[i].end[2]!=1)   //结束
        {
            if(set_time.tm_hour== Run_Diagram_buff[i].End_Timep3[0])
            {
                if(set_time.tm_min== Run_Diagram_buff[i].End_Timep3[1])
                {
                    App_printf("\r\nthe 3 end\r\n");
                    Hard_control.usage_diagram_count--;
                    State_monitor[i].end[2]=1;
                    State_monitor[i].start[2]=0;
                    if(Trans_open_Copy.Type[0]==1)
                    {
                        Trans_open_Copy.Type[0]=0;
                        //固频时，时间到，停止即可
                        memset(Trans_open_Copy.Freq,0,12);//清零
                    }
                    if(Trans_open_Copy.Type[0]==2)
                    {
                        Trans_open_Copy.Type[0]=1;
                        if(memcmp(Trans_open_Copy.Freq,Run_Diagram_buff[i].Frq3,4)==0)   //踢出第一个
                        {
                            Trans_open_Copy.Freq[0]=Trans_open_Copy.Freq[4];
                            Trans_open_Copy.Freq[1]=Trans_open_Copy.Freq[5];
                            Trans_open_Copy.Freq[2]=Trans_open_Copy.Freq[6];
                            //最后1byte都为0，所以不处理
                        }
                        //如果是踢出第二个，不需要做处理了
                    }
                    if(Trans_open_Copy.Type[0]==3)
                    {
                        Trans_open_Copy.Type[0]=2;
                        if(memcmp(Trans_open_Copy.Freq,Run_Diagram_buff[i].Frq3,4)==0)   //踢出第一个
                        {
                            Trans_open_Copy.Freq[0]=Trans_open_Copy.Freq[4];
                            Trans_open_Copy.Freq[1]=Trans_open_Copy.Freq[5];
                            Trans_open_Copy.Freq[2]=Trans_open_Copy.Freq[6];
                            //最后1byte都为0，所以不处理
                            Trans_open_Copy.Freq[4]=Trans_open_Copy.Freq[8];
                            Trans_open_Copy.Freq[5]=Trans_open_Copy.Freq[9];
                            Trans_open_Copy.Freq[6]=Trans_open_Copy.Freq[10];
                        }
                        else if(memcmp((uint8_t *)(Trans_open_Copy.Freq)+4,Run_Diagram_buff[i].Frq3,4)==0)     //踢出第二个
                        {
                            //最后1byte都为0，所以不处理
                            Trans_open_Copy.Freq[4]=Trans_open_Copy.Freq[8];
                            Trans_open_Copy.Freq[5]=Trans_open_Copy.Freq[9];
                            Trans_open_Copy.Freq[6]=Trans_open_Copy.Freq[10];
                        }
                        //如果是踢出第三个，不需要做处理了
                    }
                }
            }


        }
        //第四个
        if(State_monitor[i].start[3]!=1)
        {
            if(Run_Diagram_buff[i].Start_Time4[0]!=0xFF&&((set_time.tm_hour*60+set_time.tm_min)>=(Run_Diagram_buff[i].Start_Time4[0]*60+Run_Diagram_buff[i].Start_Time4[1]))\
                    &&((set_time.tm_hour*60+set_time.tm_min)<(Run_Diagram_buff[i].End_Timep4[0]*60+Run_Diagram_buff[i].End_Timep4[1])))
            {
//				if(set_time.tm_min== Run_Diagram_buff[i].Start_Time4[1]||(set_time.tm_hour*60+set_time.tm_min)\
//														<(Run_Diagram_buff[i].End_Timep4[0]*60+Run_Diagram_buff[i].End_Timep4[1]))
//				{
                App_printf("\r\nthe 4");
                State_monitor[i].start[3]=1;
                State_monitor[i].end[3]=0;
                Hard_control.usage_diagram_count++;
                if(Hard_control.usage_diagram_count==1)
                {
                    Trans_open_Copy.Type[0]=1;
                    Trans_open_Copy.Channel[0]=1;
                    Trans_open_Copy.Power_grade[0]=Run_Diagram_buff[i].Power4[0];
                    memcpy(Trans_open_Copy.Freq,Run_Diagram_buff[i].Frq4,4);
                }
                else if(Hard_control.usage_diagram_count==2)
                {
                    Trans_open_Copy.Type[0]=2;
                    Trans_open_Copy.Channel[1]=2;
                    Trans_open_Copy.Power_grade[1]=Run_Diagram_buff[i].Power4[0];
                    memcpy((uint8_t *)(Trans_open_Copy.Freq)+4,Run_Diagram_buff[i].Frq4,4);
                }
                else if(Hard_control.usage_diagram_count==3)
                {
                    Trans_open_Copy.Type[0]=3;
                    Trans_open_Copy.Channel[2]=3;
                    Trans_open_Copy.Power_grade[2]=Run_Diagram_buff[i].Power4[0];
                    memcpy((uint8_t *)(Trans_open_Copy.Freq)+8,Run_Diagram_buff[i].Frq4,4);
                }
                //}
            }
        }
        if(State_monitor[i].end[3]!=1)   //结束
        {
            if(set_time.tm_hour== Run_Diagram_buff[i].End_Timep4[0])
            {
                if(set_time.tm_min== Run_Diagram_buff[i].End_Timep4[1])
                {
                    App_printf("\r\nthe 4 end");
                    Hard_control.usage_diagram_count--;
                    State_monitor[i].end[3]=1;
                    State_monitor[i].start[3]=0;
                    if(Trans_open_Copy.Type[0]==1)
                    {
                        Trans_open_Copy.Type[0]=0;
                        //固频时，时间到，停止即可
                        memset(Trans_open_Copy.Freq,0,12);//清零
                    }
                    if(Trans_open_Copy.Type[0]==2)
                    {
                        Trans_open_Copy.Type[0]=1;
                        if(memcmp(Trans_open_Copy.Freq,Run_Diagram_buff[i].Frq4,4)==0)   //踢出第一个
                        {
                            Trans_open_Copy.Freq[0]=Trans_open_Copy.Freq[4];
                            Trans_open_Copy.Freq[1]=Trans_open_Copy.Freq[5];
                            Trans_open_Copy.Freq[2]=Trans_open_Copy.Freq[6];
                            //最后1byte都为0，所以不处理
                        }
                        //如果是踢出第二个，不需要做处理了
                    }
                    if(Trans_open_Copy.Type[0]==3)
                    {
                        Trans_open_Copy.Type[0]=2;
                        if(memcmp(Trans_open_Copy.Freq,Run_Diagram_buff[i].Frq4,4)==0)   //踢出第一个
                        {
                            Trans_open_Copy.Freq[0]=Trans_open_Copy.Freq[4];
                            Trans_open_Copy.Freq[1]=Trans_open_Copy.Freq[5];
                            Trans_open_Copy.Freq[2]=Trans_open_Copy.Freq[6];
                            //最后1byte都为0，所以不处理
                            Trans_open_Copy.Freq[4]=Trans_open_Copy.Freq[8];
                            Trans_open_Copy.Freq[5]=Trans_open_Copy.Freq[9];
                            Trans_open_Copy.Freq[6]=Trans_open_Copy.Freq[10];
                        }
                        else if(memcmp((uint8_t *)(Trans_open_Copy.Freq)+4,Run_Diagram_buff[i].Frq4,4)==0)     //踢出第二个
                        {
                            //最后1byte都为0，所以不处理
                            Trans_open_Copy.Freq[4]=Trans_open_Copy.Freq[8];
                            Trans_open_Copy.Freq[5]=Trans_open_Copy.Freq[9];
                            Trans_open_Copy.Freq[6]=Trans_open_Copy.Freq[10];
                        }
                        //如果是踢出第三个，不需要做处理了
                    }
                }
            }
        }
        //第五个
        if(State_monitor[i].start[4]!=1)
        {
            if(Run_Diagram_buff[i].Start_Time5[0]!=0xFF&&((set_time.tm_hour*60+set_time.tm_min)>=(Run_Diagram_buff[i].Start_Time5[0]*60+Run_Diagram_buff[i].Start_Time5[1]))\
                    &&((set_time.tm_hour*60+set_time.tm_min)<(Run_Diagram_buff[i].End_Timep5[0]*60+Run_Diagram_buff[i].End_Timep5[1])))
            {
//					if(set_time.tm_min== Run_Diagram_buff[i].Start_Time5[1]||(set_time.tm_hour*60+set_time.tm_min)\
//														<(Run_Diagram_buff[i].End_Timep5[0]*60+Run_Diagram_buff[i].End_Timep5[1]))
//					{
                App_printf("\r\nthe 5");
                State_monitor[i].start[4]=1;
                State_monitor[i].end[4]=0;
                Hard_control.usage_diagram_count++;
                if(Hard_control.usage_diagram_count==1)
                {
                    Trans_open_Copy.Type[0]=1;
                    Trans_open_Copy.Channel[0]=1;
                    Trans_open_Copy.Power_grade[0]=Run_Diagram_buff[i].Power5[0];
                    memcpy(Trans_open_Copy.Freq,Run_Diagram_buff[i].Frq5,4);
                }
                else if(Hard_control.usage_diagram_count==2)
                {
                    Trans_open_Copy.Type[0]=2;
                    Trans_open_Copy.Channel[1]=2;
                    Trans_open_Copy.Power_grade[1]=Run_Diagram_buff[i].Power5[0];
                    memcpy((uint8_t *)(Trans_open_Copy.Freq)+4,Run_Diagram_buff[i].Frq5,4);
                }
                else if(Hard_control.usage_diagram_count==3)
                {
                    Trans_open_Copy.Type[0]=3;
                    Trans_open_Copy.Channel[2]=3;
                    Trans_open_Copy.Power_grade[2]=Run_Diagram_buff[i].Power5[0];
                    memcpy((uint8_t *)(Trans_open_Copy.Freq)+8,Run_Diagram_buff[i].Frq5,4);
                }
                //	}
            }
        }
        if(State_monitor[i].end[4]!=1)   //结束
        {
            if(set_time.tm_hour== Run_Diagram_buff[i].Start_Time5[0])
            {
                if(set_time.tm_min== Run_Diagram_buff[i].End_Timep5[1])
                {
                    App_printf("\r\nthe 5 end");
                    Hard_control.usage_diagram_count--;
                    State_monitor[i].end[4]=1;
                    State_monitor[i].start[4]=0;
                    if(Trans_open_Copy.Type[0]==1)
                    {
                        Trans_open_Copy.Type[0]=0;
                        //固频时，时间到，停止即可
                        memset(Trans_open_Copy.Freq,0,12);//清零
                    }
                    if(Trans_open_Copy.Type[0]==2)
                    {
                        Trans_open_Copy.Type[0]=1;
                        if(memcmp(Trans_open_Copy.Freq,Run_Diagram_buff[i].Frq5,4)==0)   //踢出第一个
                        {
                            Trans_open_Copy.Freq[0]=Trans_open_Copy.Freq[4];
                            Trans_open_Copy.Freq[1]=Trans_open_Copy.Freq[5];
                            Trans_open_Copy.Freq[2]=Trans_open_Copy.Freq[6];
                            //最后1byte都为0，所以不处理
                        }
                        //如果是踢出第二个，不需要做处理了
                    }
                    if(Trans_open_Copy.Type[0]==3)
                    {
                        Trans_open_Copy.Type[0]=2;
                        if(memcmp(Trans_open_Copy.Freq,Run_Diagram_buff[i].Frq5,4)==0)   //踢出第一个
                        {
                            Trans_open_Copy.Freq[0]=Trans_open_Copy.Freq[4];
                            Trans_open_Copy.Freq[1]=Trans_open_Copy.Freq[5];
                            Trans_open_Copy.Freq[2]=Trans_open_Copy.Freq[6];
                            //最后1byte都为0，所以不处理
                            Trans_open_Copy.Freq[4]=Trans_open_Copy.Freq[8];
                            Trans_open_Copy.Freq[5]=Trans_open_Copy.Freq[9];
                            Trans_open_Copy.Freq[6]=Trans_open_Copy.Freq[10];
                        }
                        else if(memcmp((uint8_t *)(Trans_open_Copy.Freq)+4,Run_Diagram_buff[i].Frq5,4)==0)     //踢出第二个
                        {
                            //最后1byte都为0，所以不处理
                            Trans_open_Copy.Freq[4]=Trans_open_Copy.Freq[8];
                            Trans_open_Copy.Freq[5]=Trans_open_Copy.Freq[9];
                            Trans_open_Copy.Freq[6]=Trans_open_Copy.Freq[10];
                        }
                        //如果是踢出第三个，不需要做处理了
                    }
                }
            }
        }

        //第六个
        if(State_monitor[i].start[5]!=1)
        {
            if(Run_Diagram_buff[i].Start_Time6[0]!=0xFF&&((set_time.tm_hour*60+set_time.tm_min)>=(Run_Diagram_buff[i].Start_Time6[0]*60+Run_Diagram_buff[i].Start_Time6[1]))\
                    &&((set_time.tm_hour*60+set_time.tm_min)<(Run_Diagram_buff[i].End_Timep6[0]*60+Run_Diagram_buff[i].End_Timep6[1])))
            {
//					if(set_time.tm_min== Run_Diagram_buff[i].Start_Time6[1]||(set_time.tm_hour*60+set_time.tm_min)\
//														<(Run_Diagram_buff[i].End_Timep6[0]*60+Run_Diagram_buff[i].End_Timep6[1]))
//					{
                App_printf("\r\nthe 6");
                State_monitor[i].start[5]=1;
                State_monitor[i].end[5]=0;
                Hard_control.usage_diagram_count++;
                if(Hard_control.usage_diagram_count==1)
                {
                    Trans_open_Copy.Type[0]=1;
                    Trans_open_Copy.Channel[0]=1;
                    Trans_open_Copy.Power_grade[0]=Run_Diagram_buff[i].Power6[0];
                    memcpy(Trans_open_Copy.Freq,Run_Diagram_buff[i].Frq6,4);
                }
                else if(Hard_control.usage_diagram_count==2)
                {
                    Trans_open_Copy.Type[0]=2;
                    Trans_open_Copy.Channel[1]=2;
                    Trans_open_Copy.Power_grade[1]=Run_Diagram_buff[i].Power6[0];
                    memcpy((uint8_t *)(Trans_open_Copy.Freq)+4,Run_Diagram_buff[i].Frq6,4);
                }
                else if(Hard_control.usage_diagram_count==3)
                {
                    Trans_open_Copy.Type[0]=3;
                    Trans_open_Copy.Channel[2]=3;
                    Trans_open_Copy.Power_grade[2]=Run_Diagram_buff[i].Power6[0];
                    memcpy((uint8_t *)(Trans_open_Copy.Freq)+8,Run_Diagram_buff[i].Frq6,4);
                }
                //}
            }
        }
        if(State_monitor[i].end[5]!=1)   //结束
        {
            if(set_time.tm_hour== Run_Diagram_buff[i].End_Timep6[0])
            {
                if(set_time.tm_min== Run_Diagram_buff[i].End_Timep6[1])
                {
                    App_printf("\r\nthe 6 end");
                    Hard_control.usage_diagram_count--;
                    State_monitor[i].end[5]=1;
                    State_monitor[i].start[5]=0;
                    if(Trans_open_Copy.Type[0]==1)
                    {
                        Trans_open_Copy.Type[0]=0;
                        //固频时，时间到，停止即可
                        memset(Trans_open_Copy.Freq,0,12);//清零
                    }
                    if(Trans_open_Copy.Type[0]==2)
                    {
                        Trans_open_Copy.Type[0]=1;
                        if(memcmp(Trans_open_Copy.Freq,Run_Diagram_buff[i].Frq6,4)==0)   //踢出第一个
                        {
                            Trans_open_Copy.Freq[0]=Trans_open_Copy.Freq[4];
                            Trans_open_Copy.Freq[1]=Trans_open_Copy.Freq[5];
                            Trans_open_Copy.Freq[2]=Trans_open_Copy.Freq[6];
                            //最后1byte都为0，所以不处理
                        }
                        //如果是踢出第二个，不需要做处理了
                    }
                    if(Trans_open_Copy.Type[0]==3)
                    {
                        Trans_open_Copy.Type[0]=2;
                        if(memcmp(Trans_open_Copy.Freq,Run_Diagram_buff[i].Frq6,4)==0)   //踢出第一个
                        {
                            Trans_open_Copy.Freq[0]=Trans_open_Copy.Freq[4];
                            Trans_open_Copy.Freq[1]=Trans_open_Copy.Freq[5];
                            Trans_open_Copy.Freq[2]=Trans_open_Copy.Freq[6];
                            //最后1byte都为0，所以不处理
                            Trans_open_Copy.Freq[4]=Trans_open_Copy.Freq[8];
                            Trans_open_Copy.Freq[5]=Trans_open_Copy.Freq[9];
                            Trans_open_Copy.Freq[6]=Trans_open_Copy.Freq[10];
                        }
                        else if(memcmp((uint8_t *)(Trans_open_Copy.Freq)+4,Run_Diagram_buff[i].Frq6,4)==0)     //踢出第二个
                        {
                            //最后1byte都为0，所以不处理
                            Trans_open_Copy.Freq[4]=Trans_open_Copy.Freq[8];
                            Trans_open_Copy.Freq[5]=Trans_open_Copy.Freq[9];
                            Trans_open_Copy.Freq[6]=Trans_open_Copy.Freq[10];
                        }
                        //如果是踢出第三个，不需要做处理了
                    }
                }
            }
        }

        //第七个
        if(State_monitor[i].start[6]!=1)
        {
            if(Run_Diagram_buff[i].Start_Time7[0]!=0xFF&&((set_time.tm_hour*60+set_time.tm_min)>=(Run_Diagram_buff[i].Start_Time7[0]*60+Run_Diagram_buff[i].Start_Time7[1]))\
                    &&((set_time.tm_hour*60+set_time.tm_min)<(Run_Diagram_buff[i].End_Timep7[0]*60+Run_Diagram_buff[i].End_Timep7[1])))
            {
//					if(set_time.tm_min== Run_Diagram_buff[i].Start_Time7[1]||(set_time.tm_hour*60+set_time.tm_min)\
//														<(Run_Diagram_buff[i].End_Timep7[0]*60+Run_Diagram_buff[i].End_Timep7[1]))
//					{
                App_printf("\r\nthe 7");
                State_monitor[i].start[6]=1;
                State_monitor[i].end[6]=0;
                Hard_control.usage_diagram_count++;
                if(Hard_control.usage_diagram_count==1)
                {
                    Trans_open_Copy.Type[0]=1;
                    Trans_open_Copy.Channel[0]=1;
                    Trans_open_Copy.Power_grade[0]=Run_Diagram_buff[i].Power7[0];
                    memcpy(Trans_open_Copy.Freq,Run_Diagram_buff[i].Frq7,4);
                }
                else if(Hard_control.usage_diagram_count==2)
                {
                    Trans_open_Copy.Type[0]=2;
                    Trans_open_Copy.Channel[1]=2;
                    Trans_open_Copy.Power_grade[1]=Run_Diagram_buff[i].Power7[0];
                    memcpy((uint8_t *)(Trans_open_Copy.Freq)+4,Run_Diagram_buff[i].Frq7,4);
                }
                else if(Hard_control.usage_diagram_count==3)
                {
                    Trans_open_Copy.Type[0]=3;
                    Trans_open_Copy.Channel[2]=3;
                    Trans_open_Copy.Power_grade[2]=Run_Diagram_buff[i].Power7[0];
                    memcpy((uint8_t *)(Trans_open_Copy.Freq)+8,Run_Diagram_buff[i].Frq7,4);
                }
                //}
            }
        }
        if(State_monitor[i].end[6]!=1)   //结束
        {
            if(set_time.tm_hour== Run_Diagram_buff[i].End_Timep7[0])
            {
                if(set_time.tm_min== Run_Diagram_buff[i].End_Timep7[1])
                {
                    App_printf("\r\nthe 7 end");
                    Hard_control.usage_diagram_count--;
                    State_monitor[i].end[6]=1;
                    State_monitor[i].start[6]=0;
                    if(Trans_open_Copy.Type[0]==1)
                    {
                        Trans_open_Copy.Type[0]=0;
                        //固频时，时间到，停止即可
                        memset(Trans_open_Copy.Freq,0,12);//清零
                    }
                    if(Trans_open_Copy.Type[0]==2)
                    {
                        Trans_open_Copy.Type[0]=1;
                        if(memcmp(Trans_open_Copy.Freq,Run_Diagram_buff[i].Frq7,4)==0)   //踢出第一个
                        {
                            Trans_open_Copy.Freq[0]=Trans_open_Copy.Freq[4];
                            Trans_open_Copy.Freq[1]=Trans_open_Copy.Freq[5];
                            Trans_open_Copy.Freq[2]=Trans_open_Copy.Freq[6];
                            //最后1byte都为0，所以不处理
                        }
                        //如果是踢出第二个，不需要做处理了
                    }
                    if(Trans_open_Copy.Type[0]==3)
                    {
                        Trans_open_Copy.Type[0]=2;
                        if(memcmp(Trans_open_Copy.Freq,Run_Diagram_buff[i].Frq7,4)==0)   //踢出第一个
                        {
                            Trans_open_Copy.Freq[0]=Trans_open_Copy.Freq[4];
                            Trans_open_Copy.Freq[1]=Trans_open_Copy.Freq[5];
                            Trans_open_Copy.Freq[2]=Trans_open_Copy.Freq[6];
                            //最后1byte都为0，所以不处理
                            Trans_open_Copy.Freq[4]=Trans_open_Copy.Freq[8];
                            Trans_open_Copy.Freq[5]=Trans_open_Copy.Freq[9];
                            Trans_open_Copy.Freq[6]=Trans_open_Copy.Freq[10];
                        }
                        else if(memcmp((uint8_t *)(Trans_open_Copy.Freq)+4,Run_Diagram_buff[i].Frq7,4)==0)     //踢出第二个
                        {
                            //最后1byte都为0，所以不处理
                            Trans_open_Copy.Freq[4]=Trans_open_Copy.Freq[8];
                            Trans_open_Copy.Freq[5]=Trans_open_Copy.Freq[9];
                            Trans_open_Copy.Freq[6]=Trans_open_Copy.Freq[10];
                        }
                        //如果是踢出第三个，不需要做处理了
                    }
                }
            }
        }
        //第八个
        if(State_monitor[i].start[7]!=1)
        {
            if(Run_Diagram_buff[i].Start_Time8[0]!=0xFF&&((set_time.tm_hour*60+set_time.tm_min)>=(Run_Diagram_buff[i].Start_Time8[0]*60+Run_Diagram_buff[i].Start_Time8[1]))\
                    &&((set_time.tm_hour*60+set_time.tm_min)<(Run_Diagram_buff[i].End_Timep8[0]*60+Run_Diagram_buff[i].End_Timep8[1])))
            {
//						if(set_time.tm_min== Run_Diagram_buff[i].Start_Time8[1]||(set_time.tm_hour*60+set_time.tm_min)\
//														<(Run_Diagram_buff[i].End_Timep8[0]*60+Run_Diagram_buff[i].End_Timep8[1]))
//						{
                App_printf("\r\nthe 8");
                State_monitor[i].start[7]=1;
                State_monitor[i].end[7]=0;
                Hard_control.usage_diagram_count++;
                if(Hard_control.usage_diagram_count==1)
                {
                    Trans_open_Copy.Type[0]=1;
                    Trans_open_Copy.Channel[0]=1;
                    Trans_open_Copy.Power_grade[0]=Run_Diagram_buff[i].Power8[0];
                    memcpy(Trans_open_Copy.Freq,Run_Diagram_buff[i].Frq8,4);
                }
                else if(Hard_control.usage_diagram_count==2)
                {
                    Trans_open_Copy.Type[0]=2;
                    Trans_open_Copy.Channel[1]=2;
                    Trans_open_Copy.Power_grade[1]=Run_Diagram_buff[i].Power8[0];
                    memcpy((uint8_t *)(Trans_open_Copy.Freq)+4,Run_Diagram_buff[i].Frq8,4);
                }
                else if(Hard_control.usage_diagram_count==3)
                {
                    Trans_open_Copy.Type[0]=3;
                    Trans_open_Copy.Channel[2]=3;
                    Trans_open_Copy.Power_grade[2]=Run_Diagram_buff[i].Power8[0];
                    memcpy((uint8_t *)(Trans_open_Copy.Freq)+8,Run_Diagram_buff[i].Frq8,4);
                }
                //}
            }
        }
        if(State_monitor[i].end[7]!=1)   //结束
        {
            if(set_time.tm_hour== Run_Diagram_buff[i].End_Timep8[0])
            {
                if(set_time.tm_min== Run_Diagram_buff[i].End_Timep8[1])
                {
                    App_printf("\r\nthe 8 end");
                    Hard_control.usage_diagram_count--;
                    State_monitor[i].end[7]=1;
                    State_monitor[i].start[7]=0;
                    if(Trans_open_Copy.Type[0]==1)
                    {
                        Trans_open_Copy.Type[0]=0;
                        //固频时，时间到，停止即可
                        memset(Trans_open_Copy.Freq,0,12);//清零
                    }
                    if(Trans_open_Copy.Type[0]==2)
                    {
                        Trans_open_Copy.Type[0]=1;
                        if(memcmp(Trans_open_Copy.Freq,Run_Diagram_buff[i].Frq8,4)==0)   //踢出第一个
                        {
                            Trans_open_Copy.Freq[0]=Trans_open_Copy.Freq[4];
                            Trans_open_Copy.Freq[1]=Trans_open_Copy.Freq[5];
                            Trans_open_Copy.Freq[2]=Trans_open_Copy.Freq[6];
                            //最后1byte都为0，所以不处理
                        }
                        //如果是踢出第二个，不需要做处理了
                    }
                    if(Trans_open_Copy.Type[0]==3)
                    {
                        Trans_open_Copy.Type[0]=2;
                        if(memcmp(Trans_open_Copy.Freq,Run_Diagram_buff[i].Frq8,4)==0)   //踢出第一个
                        {
                            Trans_open_Copy.Freq[0]=Trans_open_Copy.Freq[4];
                            Trans_open_Copy.Freq[1]=Trans_open_Copy.Freq[5];
                            Trans_open_Copy.Freq[2]=Trans_open_Copy.Freq[6];
                            //最后1byte都为0，所以不处理
                            Trans_open_Copy.Freq[4]=Trans_open_Copy.Freq[8];
                            Trans_open_Copy.Freq[5]=Trans_open_Copy.Freq[9];
                            Trans_open_Copy.Freq[6]=Trans_open_Copy.Freq[10];
                        }
                        else if(memcmp((uint8_t *)(Trans_open_Copy.Freq)+4,Run_Diagram_buff[i].Frq8,4)==0)     //踢出第二个
                        {
                            //最后1byte都为0，所以不处理
                            Trans_open_Copy.Freq[4]=Trans_open_Copy.Freq[8];
                            Trans_open_Copy.Freq[5]=Trans_open_Copy.Freq[9];
                            Trans_open_Copy.Freq[6]=Trans_open_Copy.Freq[10];
                        }
                        //如果是踢出第三个，不需要做处理了
                    }
                }
            }
        }

        //第九个
        if(State_monitor[i].start[8]!=1)
        {
            if(Run_Diagram_buff[i].Start_Time9[0]!=0xFF&&((set_time.tm_hour*60+set_time.tm_min)>=(Run_Diagram_buff[i].Start_Time9[0]*60+Run_Diagram_buff[i].Start_Time9[1]))\
                    &&((set_time.tm_hour*60+set_time.tm_min)<(Run_Diagram_buff[i].End_Timep9[0]*60+Run_Diagram_buff[i].End_Timep9[1])))
            {
//					if(set_time.tm_min== Run_Diagram_buff[i].Start_Time9[1]||(set_time.tm_hour*60+set_time.tm_min)\
//														<(Run_Diagram_buff[i].End_Timep9[0]*60+Run_Diagram_buff[i].End_Timep9[1]))
//					{
                App_printf("\r\nthe 9");
                State_monitor[i].start[8]=1;
                State_monitor[i].end[8]=0;
                Hard_control.usage_diagram_count++;
                if(Hard_control.usage_diagram_count==1)
                {
                    Trans_open_Copy.Type[0]=1;
                    Trans_open_Copy.Channel[0]=1;
                    Trans_open_Copy.Power_grade[0]=Run_Diagram_buff[i].Power9[0];
                    memcpy(Trans_open_Copy.Freq,Run_Diagram_buff[i].Frq9,4);
                }
                else if(Hard_control.usage_diagram_count==2)
                {
                    Trans_open_Copy.Type[0]=2;
                    Trans_open_Copy.Channel[1]=2;
                    Trans_open_Copy.Power_grade[1]=Run_Diagram_buff[i].Power9[0];
                    memcpy((uint8_t *)(Trans_open_Copy.Freq)+4,Run_Diagram_buff[i].Frq9,4);
                }
                else if(Hard_control.usage_diagram_count==3)
                {
                    Trans_open_Copy.Type[0]=3;
                    Trans_open_Copy.Channel[2]=3;
                    Trans_open_Copy.Power_grade[2]=Run_Diagram_buff[i].Power9[0];
                    memcpy((uint8_t *)(Trans_open_Copy.Freq)+8,Run_Diagram_buff[i].Frq9,4);
                }
                //	}
            }
        }

        if(State_monitor[i].end[8]!=1)   //结束
        {
            if(set_time.tm_hour== Run_Diagram_buff[i].End_Timep9[0])
            {
                if(set_time.tm_min== Run_Diagram_buff[i].End_Timep9[1])
                {
                    App_printf("\r\nthe 9 end");
                    Hard_control.usage_diagram_count--;
                    State_monitor[i].end[8]=1;
                    State_monitor[i].start[8]=0;
                    if(Trans_open_Copy.Type[0]==1)
                    {
                        Trans_open_Copy.Type[0]=0;
                        //固频时，时间到，停止即可
                        memset(Trans_open_Copy.Freq,0,12);//清零
                    }
                    if(Trans_open_Copy.Type[0]==2)
                    {
                        Trans_open_Copy.Type[0]=1;
                        if(memcmp(Trans_open_Copy.Freq,Run_Diagram_buff[i].Frq9,4)==0)   //踢出第一个
                        {
                            Trans_open_Copy.Freq[0]=Trans_open_Copy.Freq[4];
                            Trans_open_Copy.Freq[1]=Trans_open_Copy.Freq[5];
                            Trans_open_Copy.Freq[2]=Trans_open_Copy.Freq[6];
                            //最后1byte都为0，所以不处理
                        }
                        //如果是踢出第二个，不需要做处理了
                    }
                    if(Trans_open_Copy.Type[0]==3)
                    {
                        Trans_open_Copy.Type[0]=2;
                        if(memcmp(Trans_open_Copy.Freq,Run_Diagram_buff[i].Frq9,4)==0)   //踢出第一个
                        {
                            Trans_open_Copy.Freq[0]=Trans_open_Copy.Freq[4];
                            Trans_open_Copy.Freq[1]=Trans_open_Copy.Freq[5];
                            Trans_open_Copy.Freq[2]=Trans_open_Copy.Freq[6];
                            //最后1byte都为0，所以不处理
                            Trans_open_Copy.Freq[4]=Trans_open_Copy.Freq[8];
                            Trans_open_Copy.Freq[5]=Trans_open_Copy.Freq[9];
                            Trans_open_Copy.Freq[6]=Trans_open_Copy.Freq[10];
                        }
                        else if(memcmp((uint8_t *)(Trans_open_Copy.Freq)+4,Run_Diagram_buff[i].Frq9,4)==0)     //踢出第二个
                        {
                            //最后1byte都为0，所以不处理
                            Trans_open_Copy.Freq[4]=Trans_open_Copy.Freq[8];
                            Trans_open_Copy.Freq[5]=Trans_open_Copy.Freq[9];
                            Trans_open_Copy.Freq[6]=Trans_open_Copy.Freq[10];
                        }
                        //如果是踢出第三个，不需要做处理了
                    }
                }
            }
        }

        //第十个
        if(State_monitor[i].start[9]!=1)
        {
            if(Run_Diagram_buff[i].Start_Time10[0]!=0xFF&&((set_time.tm_hour*60+set_time.tm_min)>=(Run_Diagram_buff[i].Start_Time10[0]*60+Run_Diagram_buff[i].Start_Time10[1]))\
                    &&((set_time.tm_hour*60+set_time.tm_min)<(Run_Diagram_buff[i].End_Timep10[0]*60+Run_Diagram_buff[i].End_Timep10[1])))
            {
//					if(set_time.tm_min== Run_Diagram_buff[i].Start_Time10[1]||(set_time.tm_hour*60+set_time.tm_min)\
//														<(Run_Diagram_buff[i].End_Timep10[0]*60+Run_Diagram_buff[i].End_Timep10[1]))
//					{
                App_printf("\r\nthe 10 ");
                State_monitor[i].start[9]=1;
                State_monitor[i].end[9]=0;
                Hard_control.usage_diagram_count++;
                if(Hard_control.usage_diagram_count==1)
                {
                    Trans_open_Copy.Type[0]=1;
                    Trans_open_Copy.Channel[0]=1;
                    Trans_open_Copy.Power_grade[0]=Run_Diagram_buff[i].Power10[0];
                    memcpy(Trans_open_Copy.Freq,Run_Diagram_buff[i].Frq10,4);
                }
                else if(Hard_control.usage_diagram_count==2)
                {
                    Trans_open_Copy.Type[0]=2;
                    Trans_open_Copy.Channel[1]=2;
                    Trans_open_Copy.Power_grade[1]=Run_Diagram_buff[i].Power10[0];
                    memcpy((uint8_t *)(Trans_open_Copy.Freq)+4,Run_Diagram_buff[i].Frq10,4);
                }
                else if(Hard_control.usage_diagram_count==3)
                {
                    Trans_open_Copy.Type[0]=3;
                    Trans_open_Copy.Channel[2]=3;
                    Trans_open_Copy.Power_grade[2]=Run_Diagram_buff[i].Power10[0];
                    memcpy((uint8_t *)(Trans_open_Copy.Freq)+8,Run_Diagram_buff[i].Frq10,4);
                }
                //	}
            }
        }
        if(State_monitor[i].end[9]!=1)   //结束
        {
            if(set_time.tm_hour== Run_Diagram_buff[i].End_Timep10[0])
            {
                if(set_time.tm_min== Run_Diagram_buff[i].End_Timep10[1])
                {
                    App_printf("\r\nthe 10 end");
                    Hard_control.usage_diagram_count--;
                    State_monitor[i].end[9]=1;
                    State_monitor[i].start[9]=0;
                    if(Trans_open_Copy.Type[0]==1)
                    {
                        Trans_open_Copy.Type[0]=0;
                        //固频时，时间到，停止即可
                        memset(Trans_open_Copy.Freq,0,12);//清零
                    }
                    if(Trans_open_Copy.Type[0]==2)
                    {
                        Trans_open_Copy.Type[0]=1;
                        if(memcmp(Trans_open_Copy.Freq,Run_Diagram_buff[i].Frq10,4)==0)   //踢出第一个
                        {
                            Trans_open_Copy.Freq[0]=Trans_open_Copy.Freq[4];
                            Trans_open_Copy.Freq[1]=Trans_open_Copy.Freq[5];
                            Trans_open_Copy.Freq[2]=Trans_open_Copy.Freq[6];
                            //最后1byte都为0，所以不处理
                        }
                        //如果是踢出第二个，不需要做处理了
                    }
                    if(Trans_open_Copy.Type[0]==3)
                    {
                        Trans_open_Copy.Type[0]=2;
                        if(memcmp(Trans_open_Copy.Freq,Run_Diagram_buff[i].Frq10,4)==0)   //踢出第一个
                        {
                            Trans_open_Copy.Freq[0]=Trans_open_Copy.Freq[4];
                            Trans_open_Copy.Freq[1]=Trans_open_Copy.Freq[5];
                            Trans_open_Copy.Freq[2]=Trans_open_Copy.Freq[6];
                            //最后1byte都为0，所以不处理
                            Trans_open_Copy.Freq[4]=Trans_open_Copy.Freq[8];
                            Trans_open_Copy.Freq[5]=Trans_open_Copy.Freq[9];
                            Trans_open_Copy.Freq[6]=Trans_open_Copy.Freq[10];
                        }
                        else if(memcmp((uint8_t *)(Trans_open_Copy.Freq)+4,Run_Diagram_buff[i].Frq10,4)==0)     //踢出第二个
                        {
                            //最后1byte都为0，所以不处理
                            Trans_open_Copy.Freq[4]=Trans_open_Copy.Freq[8];
                            Trans_open_Copy.Freq[5]=Trans_open_Copy.Freq[9];
                            Trans_open_Copy.Freq[6]=Trans_open_Copy.Freq[10];
                        }
                        //如果是踢出第三个，不需要做处理了
                    }
                }
            }
        }
        //}

    }
    return 0;
}

/*判断是否需要开机*/
void juge_isOpen(uint8_t Save_count)
{
    uint32_t min;//分钟
    min=set_time.tm_hour*60+set_time.tm_min;
    for(int i=0; i<Save_count; i++)
    {
        for(int j=0; j<10; j++)
        {
            if(Sys_State.Sys_open==1)
            {
                Monitor.need_open=0;
                return ;
            }
            if(Run_Diagram_buff[i].Start_Time1[0]*60+Run_Diagram_buff[i].Start_Time1[1]-min<=10)
            {
                /*需要开机*/
                Monitor.need_open=1;
            }
            if(Run_Diagram_buff[i].Start_Time2[0]*60+Run_Diagram_buff[i].Start_Time2[1]-min<=10)
            {
                /*需要开机*/
                Monitor.need_open=1;
            }

            if(Run_Diagram_buff[i].Start_Time3[0]*60+Run_Diagram_buff[i].Start_Time3[1]-min<=10)
            {
                /*需要开机*/
                Monitor.need_open=1;
            }
            if(Run_Diagram_buff[i].Start_Time4[0]*60+Run_Diagram_buff[i].Start_Time4[1]-min<=10)
            {
                /*需要开机*/
                Monitor.need_open=1;
            }
            if(Run_Diagram_buff[i].Start_Time5[0]*60+Run_Diagram_buff[i].Start_Time5[1]-min<=10)
            {
                /*需要开机*/
                Monitor.need_open=1;
            }
            if(Run_Diagram_buff[i].Start_Time6[0]*60+Run_Diagram_buff[i].Start_Time6[1]-min<=10)
            {
                /*需要开机*/
                Monitor.need_open=1;
            }
            if(Run_Diagram_buff[i].Start_Time7[0]*60+Run_Diagram_buff[i].Start_Time7[1]-min<=10)
            {
                /*需要开机*/
                Monitor.need_open=1;
            }
            if(Run_Diagram_buff[i].Start_Time8[0]*60+Run_Diagram_buff[i].Start_Time8[1]-min<=10)
            {
                /*需要开机*/
                Monitor.need_open=1;
            }
            if(Run_Diagram_buff[i].Start_Time9[0]*60+Run_Diagram_buff[i].Start_Time9[1]-min<=10)
            {
                /*需要开机*/
                Monitor.need_open=1;
            }
            if(Run_Diagram_buff[i].Start_Time10[0]*60+Run_Diagram_buff[i].Start_Time10[1]-min<=10)
            {
                /*需要开机*/
                Monitor.need_open=1;
            }

        }
    }
}

