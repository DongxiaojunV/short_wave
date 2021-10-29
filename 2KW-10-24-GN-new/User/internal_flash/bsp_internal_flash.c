/**
  ******************************************************************************
  * @file    bsp_internalFlash.c
  * @author  fire
  * @version V1.0
  * @date    2015-xx-xx
  * @brief   �ڲ�FLASH��д���Է���
  ******************************************************************************
  * @attention
  *
  * ʵ��ƽ̨:����  STM32 �Ե� ������
  * ��̳    :http://www.firebbs.cn
  * �Ա�    :https://fire-stm32.taobao.com
  *
  ******************************************************************************
  */
#include "bsp.h"
#include "bsp_internal_flash.h"
#include "main.h"

volatile uint32_t flash_1_once_flag;
volatile uint32_t flash_2_once_flag;
volatile uint32_t flash_3_once_flag;

F_I_t  flash1_arry[9];
F_I_t  flash2_arry[480];		//Ϊ�˱���ʱת��float��int����
uint8_t flash1_IP[4];
uint8_t flash1_Port[2];
uint8_t flash_run_diagram[100];
volatile uint8_t flash3_Save;


uint8_t  *Flash_p1=(uint8_t*)&Run_Diagram_buf[0];
uint8_t  *Flash_p2=(uint8_t*)&Run_Diagram_buf[1];
uint8_t  *Flash_p3=(uint8_t*)&Run_Diagram_buf[2];
uint8_t  *Flash_p4=(uint8_t*)&Run_Diagram_buf[3];
uint8_t  *Flash_p5=(uint8_t*)&Run_Diagram_buf[4];
uint8_t  *Flash_p6=(uint8_t*)&Run_Diagram_buf[5];
uint8_t  *Flash_p7=(uint8_t*)&Run_Diagram_buf[6];
uint8_t  *Flash_p8=(uint8_t*)&Run_Diagram_buf[7];
uint8_t  *Flash_p9=(uint8_t*)&Run_Diagram_buf[8];
uint8_t  *Flash_p10=(uint8_t*)&Run_Diagram_buf[9];

//void my_delay_us(uint32_t nTimer)
//{
//	uint32_t i=0;
//	for(i=0;i<nTimer;i++){
//		__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
//		__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
//		__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
//		__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
//		__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
//	}
//}

//void my_delay_ms(uint32_t nTimer)
//{
//	u32 i=1000*nTimer;
//	my_delay_us(i);
//}

void clear_flash_save(uint8_t num)
{
    uint8_t  *Flash_p=Flash_p1;
    switch(num)
    {
		case 2:
		{
			for(int i=100; i<100*10; i++)
			{
				Flash_p[i]=0xFF;
			}
			break;
		}
		case 3:
		{
			for(int i=200; i<100*10; i++)
			{
				Flash_p[i]=0xFF;
			}
			break;
		}
		case 4:
		{
			for(int i=300; i<100*10; i++)
			{
				Flash_p[i]=0xFF;
			}
			break;
		}
		case 5:
		{
			for(int i=400; i<100*10; i++)
			{
				Flash_p[i]=0xFF;
			}
			break;
		}
		case 6:
		{
			for(int i=500; i<100*10; i++)
			{
				Flash_p[i]=0xFF;
			}
			break;
		}
		case 7:
		{
			for(int i=600; i<100*10; i++)
			{
				Flash_p[i]=0xFF;
			}
			break;
		}
		case 8:
		{
			for(int i=700; i<100*10; i++)
			{
				Flash_p[i]=0xFF;
			}
			break;
		}
		case 9:
		{
			for(int i=800; i<100*10; i++)
			{
				Flash_p[i]=0xFF;
			}
			break;
		}
		case 10:
		{
			for(int i=900; i<100*10; i++)
			{
				Flash_p[i]=0xFF;
			}
			break;
		}
		default:
		{
			App_printf("num not match!\r\n");
			break;
		}
    }
}

/*	��ȡ����ͼ����10������ͼ����ӦRun_Diagram_buf[n]����պ���δ��������ͼ��flash
 *	save_cout:һ��д�˶��ٴ�flash ����ʵ��Ϊ flash3_Save��ֵ
 */
void judg_read_flash(uint32_t save_cout)
{
    volatile uint8_t count=1;
	
    Flash_Read_Run_Diagram(WRITE_START_ADDR_1,WRITE_END_ADDR_1,count);
    ++count;
    if(count>save_cout)
    {
        clear_flash_save(count);
        return;
    }
	
    Flash_Read_Run_Diagram(WRITE_START_ADDR_2,WRITE_END_ADDR_2,count);
    ++count;
    if(count>save_cout)
    {
        clear_flash_save(count);
        return;
    }
	
    Flash_Read_Run_Diagram(WRITE_START_ADDR_3,WRITE_END_ADDR_3,count);
    ++count;
    if(count>save_cout)
    {
        clear_flash_save(count);
        return;
    }
	
    Flash_Read_Run_Diagram(WRITE_START_ADDR_4,WRITE_END_ADDR_4,count);
    ++count;
    if(count>save_cout)
    {
        clear_flash_save(count);
        return;
    }
	
    Flash_Read_Run_Diagram(WRITE_START_ADDR_5,WRITE_END_ADDR_5,count);
    ++count;
    if(count>save_cout)
    {
        clear_flash_save(count);
        return;
    }
	
    Flash_Read_Run_Diagram(WRITE_START_ADDR_6,WRITE_END_ADDR_6,count);
    ++count;
    if(count>save_cout)
    {
        clear_flash_save(count);
        return;
    }
	
    Flash_Read_Run_Diagram(WRITE_START_ADDR_7,WRITE_END_ADDR_7,count);
    ++count;
    if(count>save_cout)
    {
        clear_flash_save(count);
        return;
    }
	
    Flash_Read_Run_Diagram(WRITE_START_ADDR_8,WRITE_END_ADDR_8,count);
    ++count;
    if(count>save_cout)
    {
        clear_flash_save(count);
        return;
    }
	
    Flash_Read_Run_Diagram(WRITE_START_ADDR_9,WRITE_END_ADDR_9,count);
    ++count;
    if(count>save_cout)
    {
        clear_flash_save(count);
        return;
    }
	
    Flash_Read_Run_Diagram(WRITE_START_ADDR_10,WRITE_END_ADDR_10,count);
//		++count;
//	if(count>save_cout) {return;}
}

int System_InternalFlash_Init(uint32_t init_flag)
{
    uint32_t EraseCounter = 0x00;		//��¼Ҫ��������ҳ
    uint32_t Address = 0x00;			//��¼д��ĵ�ַ
    uint32_t NbrOfPage = 0x00;			//��¼д�����ҳ

    FLASH_Status FLASHStatus = FLASH_COMPLETE; //��¼ÿ�β����Ľ��
    TestStatus MemoryProgramStatus = PASSED;//��¼�������Խ��
    flash1_arry[0].i_data=Alarm_threshold.Transmitte_id[0];
    float *Alarm_threshold_p =(float *)&(Alarm_threshold.Low_temp_limit);
    for(int i=1; i<=8; i++)
    {
        flash1_arry[i].f_data=*Alarm_threshold_p;
//        App_printf("%f ",flash1_arry[i].f_data);
        Alarm_threshold_p +=1;
    }
//    App_printf("....\r\n");
//	flash1_arry[13].i_data=Alarm_threshold.Type[0];//��������ֶβ��������壬Ҳû����ʹ��
    /* ���� */
    FLASH_Unlock();

    /* ����Ҫ��������ҳ */
    NbrOfPage = (WRITE_END_ADDR_first - WRITE_START_ADDR_first+1) / FLASH_PAGE_SIZE;

    /* ������б�־λ */
    FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);

    /* ��ҳ����*/
    for(EraseCounter = 0; (EraseCounter < NbrOfPage) && (FLASHStatus == FLASH_COMPLETE); EraseCounter++)
    {
        FLASHStatus = FLASH_ErasePage(WRITE_START_ADDR_first + (FLASH_PAGE_SIZE * EraseCounter));

    }

    /* ���ڲ�FLASHд������ */
    Address = WRITE_START_ADDR_first;

    FLASH_ProgramWord(Address,flash1_arry[0].i_data);//�����ID
    Address = Address + 4;
    for(int i=1; i<=8 && Address < WRITE_END_ADDR_first && (FLASHStatus == FLASH_COMPLETE) ; i++)
    {
        FLASH_ProgramWord(Address,flash1_arry[i].i_data);
        Address = Address + 4;
    }
//	FLASH_ProgramWord(Address,flash1_arry[13].i_data);//����
//	Address = Address + 4;

//д��flash_1_once_flag�̻�һ�α�־
    FLASH_ProgramWord(Address,init_flag);
    FLASH_Lock();//flash����

    /* ���д��������Ƿ���ȷ */
    Address = WRITE_START_ADDR_first;
    for(int i=0; i<9 && Address < WRITE_END_ADDR_first && (MemoryProgramStatus != FAILED); i++)
    {
        if((*(__IO uint32_t*) Address) != flash1_arry[i].i_data)
        {
            //uint32_t hi=*(__IO uint32_t*) Address;
            MemoryProgramStatus = FAILED;
        }
        Address = Address + 4;
    }
    if((*(__IO uint32_t*) Address) != init_flag)
    {
        MemoryProgramStatus = FAILED;
    }
    return MemoryProgramStatus;
}

int InternalFlash_SaveData_1(uint8_t init)
{
    uint32_t EraseCounter = 0x00;		//��¼Ҫ��������ҳ
    uint32_t Address = 0x00;			//��¼д��ĵ�ַ
    uint32_t NbrOfPage = 0x00;			//��¼д�����ҳ
    FLASH_Status FLASHStatus = FLASH_COMPLETE; //��¼ÿ�β����Ľ��
    TestStatus MemoryProgramStatus = PASSED;//��¼�������Խ��
    flash1_arry[0].i_data=Alarm_threshold.Transmitte_id[0];

		flash1_arry[1].f_data=Alarm_threshold.Low_temp_limit[0];
		flash1_arry[2].f_data=Alarm_threshold.Upp_temp_limit[0];
		flash1_arry[3].f_data=Alarm_threshold.Low_humidity_limit[0];
		flash1_arry[4].f_data=Alarm_threshold.Upp_humidity_limit[0];
		flash1_arry[5].f_data=Alarm_threshold.Low_45I_limit[0];
		flash1_arry[6].f_data=Alarm_threshold.Upp_45I_limit[0];
		flash1_arry[7].f_data=Alarm_threshold.Low_45V_limit[0];
		flash1_arry[8].f_data=Alarm_threshold.Upp_45V_limit[0];
	  
		for(int i=0;i<4;i++)
		{
			flash1_IP[i]=System.Ip[i];
		}
		flash1_Port[0]=System.Port[0];
		flash1_Port[1]=System.Port[1];
    /* ���� */
    FLASH_Unlock();

    /* ����Ҫ��������ҳ */
    NbrOfPage = (WRITE_END_ADDR_first - WRITE_START_ADDR_first+1) / FLASH_PAGE_SIZE;

    /* ������б�־λ */
    FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);

    /* ��ҳ����*/
    for(EraseCounter = 0; (EraseCounter < NbrOfPage) && (FLASHStatus == FLASH_COMPLETE); EraseCounter++)
    {
        FLASHStatus = FLASH_ErasePage(WRITE_START_ADDR_first + (FLASH_PAGE_SIZE * EraseCounter));

    }

    /* ���ڲ�FLASHд������ */
    Address = WRITE_START_ADDR_first;

    FLASH_ProgramWord(Address,flash1_arry[0].i_data);//�����ID
    Address = Address + 4;
    for(int i=1; i<=8 && Address < WRITE_END_ADDR_first && (FLASHStatus == FLASH_COMPLETE) ; i++)
    {
        FLASH_ProgramWord(Address,flash1_arry[i].i_data);
        Address = Address + 4;
    }
//	FLASH_ProgramWord(Address,flash1_arry[13].i_data);//����
//	Address = Address + 4;
	
//    uint32_t one=1;
//д��flash_1_once_flag�̻�һ�α�־
    FLASH_ProgramWord(Address,init);
		if(System.Init_Mark!=1)		//�����ʼ��ʱ��дIP��Port
		{
			FLASHStatus=FLASH_ErasePage(Ip_Port_Addr);		//�Ȳ�����һҳ��д
			Address = Ip_Port_Addr;
			/*��flashдIP*/
			for(int i=0; i<4 && Address < Ip_Port_Addr+26 && (FLASHStatus == FLASH_COMPLETE) ; i++)
			{
					FLASH_ProgramWord(Address,flash1_IP[i]);
					Address = Address + 4;
			}
			/*��flashдPORT*/
			FLASH_ProgramWord(Address,flash1_Port[0]);
			Address = Address + 4;
			FLASH_ProgramWord(Address,flash1_Port[1]);
		}
    FLASH_Lock();//flash����

    /* ���д��������Ƿ���ȷ */
    Address = WRITE_START_ADDR_first;
    for(int i=0; i<9 && Address < WRITE_END_ADDR_first && (MemoryProgramStatus != FAILED); i++)
    {

        if((*(__IO uint32_t*) Address) != flash1_arry[i].i_data)
        {
            //uint32_t hi=*(__IO uint32_t*) Address;
            MemoryProgramStatus = FAILED;
        }
        Address = Address + 4;
    }
    if((*(__IO uint32_t*) Address) != init)
    {
        MemoryProgramStatus = FAILED;
    }
		
	if(System.Init_Mark!=1)		//�����ʼ��ʱ��дIP��Port
	{	
		Address= Ip_Port_Addr;//��ַָ��IP[0]
		
		for(int i=0; i<4 && Address < Ip_Port_Addr+26 && (MemoryProgramStatus != FAILED); i++)
    {

        if((*(__IO uint32_t*) Address) != flash1_IP[i])
        {
            MemoryProgramStatus = FAILED;
        }
        Address = Address + 4;
    }
		
		if((*(__IO uint32_t*) Address) != flash1_Port[0]||(*(__IO uint32_t*)(Address+4)) != flash1_Port[1])
		{
				MemoryProgramStatus = FAILED;
		}
	}
    return MemoryProgramStatus;
}


//�������ʺ�פ���ȱ��浽flash
int InternalFlash_SaveData_2(void)
{
    uint32_t EraseCounter = 0x00; 			//��¼Ҫ��������ҳ
    uint32_t Address = 0x00;				//��¼д��ĵ�ַ
    uint32_t NbrOfPage = 0x00;				//��¼д�����ҳ

    FLASH_Status FLASHStatus = FLASH_COMPLETE; //��¼ÿ�β����Ľ��
    TestStatus MemoryProgramStatus = PASSED;//��¼�������Խ��

    memcpy(flash2_arry,		SWR_array,				233*4);
    memcpy(flash2_arry+233,	Forward_Power_array,	233*4);

    FLASH_Unlock();	/* ���� */
    NbrOfPage = (WRITE_END_ADDR_second-WRITE_START_ADDR_second+1) / FLASH_PAGE_SIZE;	/* ����Ҫ��������ҳ */
    FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);			/* ������б�־λ */
    for(EraseCounter = 0; (EraseCounter < NbrOfPage) && (FLASHStatus == FLASH_COMPLETE); EraseCounter++)	/* ��ҳ���� */
    {
        FLASHStatus = FLASH_ErasePage(WRITE_START_ADDR_second + (FLASH_PAGE_SIZE * EraseCounter));
    }

    Address = WRITE_START_ADDR_second;			/* ���ڲ�FLASHд������ */

    //д��flash_2_once_flag�̻�һ�α�־
	if( System.sweep == 0x02 )
		FLASH_ProgramWord(Address, 1);		//�Ѿ�ɨƵ����
	else
		FLASH_ProgramWord(Address, 0);		//������ɨ����
	
    Address = Address + 4;
    for(int i=0; i<466 && Address < WRITE_END_ADDR_second && (FLASHStatus == FLASH_COMPLETE) ; i++)
    {
        FLASH_ProgramWord(Address,flash2_arry[i].i_data);
        Address = Address + 4;
    }

    for(int i=0; i<10 && Address < WRITE_END_ADDR_second && (FLASHStatus == FLASH_COMPLETE) ; i++)
    {
        FLASH_ProgramWord(Address, freq_band_time_stamp[i]);
        Address = Address + 4;
    }

    FLASH_Lock();	/* ���� */

    /* ���д��������Ƿ���ȷ */
    Address = WRITE_START_ADDR_second;
    if((*(__IO uint32_t*) Address) != 1)
    {
//        MemoryProgramStatus = FAILED;
    }

    Address = Address + 4;
    for(int i=0; i<466 && Address < WRITE_END_ADDR_second && (MemoryProgramStatus != FAILED); i++)	//У��
    {
        if((*(__IO uint32_t*) Address) != flash2_arry[i].i_data)
        {
            MemoryProgramStatus = FAILED;
        }
        Address = Address + 4;
    }

    for(int i=0; i<10 && Address < WRITE_END_ADDR_second && (MemoryProgramStatus != FAILED); i++)	//У��
    {
        if((*(__IO uint32_t*) Address) != freq_band_time_stamp[i])
        {
            MemoryProgramStatus = FAILED;  //�ڴ�洢״̬��FAILEDʧ��   
        }
        Address = Address + 4;
    }

    return MemoryProgramStatus;
}


int Flash_Save_Run_Diagram(uint32_t start, uint32_t end, uint8_t run_diagram_count)
{

    uint32_t EraseCounter = 0x00; 	//��¼Ҫ��������ҳ
    uint32_t Address = 0x00;				//��¼д��ĵ�ַ
    uint32_t NbrOfPage = 0x00;			//��¼д�����ҳ

    FLASH_Status FLASHStatus = FLASH_COMPLETE; //��¼ÿ�β����Ľ��
    TestStatus MemoryProgramStatus = PASSED;//��¼�������Խ��
    switch(run_diagram_count)
    {
		case 1:
		{
			memcpy(flash_run_diagram, &Run_Diagram.Start_Time1[0], 10*run_diagram_count);
			break;
		}
		case 2:
		{
			memcpy(flash_run_diagram, &Run_Diagram.Start_Time1[0], 10*run_diagram_count);
			break;
		}
		case 3:
		{
			memcpy(flash_run_diagram, &Run_Diagram.Start_Time1[0], 10*run_diagram_count);
			break;
		}
		case 4:
		{
			memcpy(flash_run_diagram, &Run_Diagram.Start_Time1[0], 10*run_diagram_count);
			break;
		}
		case 5:
		{
			memcpy(flash_run_diagram, &Run_Diagram.Start_Time1[0], 10*run_diagram_count);
			break;
		}
		case 6:
		{
			memcpy(flash_run_diagram, &Run_Diagram.Start_Time1[0], 10*run_diagram_count);
			break;
		}
		case 7:
		{
			memcpy(flash_run_diagram, &Run_Diagram.Start_Time1[0], 10*run_diagram_count);
			break;
		}
		case 8:
		{
			memcpy(flash_run_diagram, &Run_Diagram.Start_Time1[0], 10*run_diagram_count);
			break;
		}
		case 9:
		{
			memcpy(flash_run_diagram, &Run_Diagram.Start_Time1[0], 10*run_diagram_count);
			break;
		}
		case 10:
		{
			memcpy(flash_run_diagram, &Run_Diagram.Start_Time1[0], 10*run_diagram_count);
			break;
		}
    }

    /* ���� */
    FLASH_Unlock();

    /* ����Ҫ��������ҳ */
    NbrOfPage = (end-start+1) / FLASH_PAGE_SIZE;

    /* ������б�־λ */
    FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);

    /* ��ҳ����*/
    for(EraseCounter = 0; (EraseCounter < NbrOfPage) && (FLASHStatus == FLASH_COMPLETE); EraseCounter++)
    {
        FLASHStatus = FLASH_ErasePage(start + (FLASH_PAGE_SIZE * EraseCounter));
    }
    /* ���ڲ�FLASHд������ */
    Address = start;
    for(int i=0; i<10*run_diagram_count && Address < end && (FLASHStatus == FLASH_COMPLETE) ; i++)
    {
        FLASH_ProgramWord(Address,flash_run_diagram[i]);
        Address = Address + 4;
    }

    FLASH_Lock();

    /* ���д��������Ƿ���ȷ */
    Address = start;
    for(int i=0; i<10*run_diagram_count && Address < end && (MemoryProgramStatus != FAILED); i++)
    {
        if((*(__IO uint32_t*) Address) !=flash_run_diagram[i])
        {
            //	uint32_t hi=*(__IO uint32_t*) Address;
            MemoryProgramStatus = FAILED;
        }
        Address = Address + 4;
    }

    return MemoryProgramStatus;
}

/**
  * @brief  InternalFlash_Test,���ڲ�FLASH���ж�д����
  * @param  num:PC�·��˼�������ͼ����Ҳ�Ǻ�����Ҫ��ȡflash�����Ĵ���
  * @retval None
  */
int InternalFlash_SaveData_3(uint8_t num)
{

    uint32_t EraseCounter = 0x00; 	//��¼Ҫ��������ҳ
    uint32_t Address = 0x00;				//��¼д��ĵ�ַ
    uint32_t NbrOfPage = 0x00;			//��¼д�����ҳ

    FLASH_Status FLASHStatus = FLASH_COMPLETE; //��¼ÿ�β����Ľ��
    TestStatus MemoryProgramStatus = PASSED;//��¼�������Խ��
    /* ���� */
    FLASH_Unlock();
    /* ����Ҫ��������ҳ */
    NbrOfPage = (WRITE_END_ADDR_third - WRITE_START_ADDR_third+1) / FLASH_PAGE_SIZE;
    /* ������б�־λ */
    FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);

    /* ��ҳ����*/
    for(EraseCounter = 0; (EraseCounter < NbrOfPage) && (FLASHStatus == FLASH_COMPLETE); EraseCounter++)
    {
        FLASHStatus = FLASH_ErasePage(WRITE_START_ADDR_third + (FLASH_PAGE_SIZE * EraseCounter));
    }
    /* ���ڲ�FLASHд������ */
    Address = WRITE_START_ADDR_third;
    FLASH_ProgramWord(Address,1);
    Address=Address+4;
    FLASH_ProgramWord(Address,num);
    FLASH_Lock();//flash����

    /* ���д��������Ƿ���ȷ */
    Address = WRITE_START_ADDR_third;
    if((*(__IO uint32_t*) Address) !=1)
    {
        MemoryProgramStatus = FAILED;
    }
    Address=Address+4;
    if((*(__IO uint32_t*) Address) !=num)
    {
        MemoryProgramStatus = FAILED;
    }
    return MemoryProgramStatus;
}

//��ȡ��������
_Bool Flash_to_AcceptAPP(void)
{
    uint32_t Address = 0x00;				//��¼д��ĵ�ַ
    TestStatus MemoryProgramStatus = PASSED;//��¼�������Խ��

    /* ���д��������Ƿ���ȷ */
    Address = WRITE_START_ADDR_first;
    for(int i=0; i<9 && Address < WRITE_END_ADDR_first && (MemoryProgramStatus != FAILED); i++)
    {
        flash1_arry[i].i_data=*(__IO uint32_t*) Address;
//        if(i!=0)
//            App_printf("%f ",flash1_arry[i].f_data);
//        else
//            App_printf("%d ",flash1_arry[i].i_data);
        Address = Address + 4;
    }

    flash_1_once_flag=*(__IO uint32_t*) Address;				//flash�̻���־д1,���Ϊ65535��Ϊû�г�ʼ��
    System.already_init = flash_1_once_flag;					//�Ѿ���ʼ����־
    Alarm_threshold.Transmitte_id[0] = flash1_arry[0].i_data;
    System.CAN_ID[0] = Alarm_threshold.Transmitte_id[0];		//�̻�CAN_ID��
    Alarm_threshold.Low_temp_limit[0] = flash1_arry[1].f_data;
    float *	Alarm_threshold_p =(float *)&(Alarm_threshold.Low_temp_limit);
//    float *temp=Alarm_threshold_p;

    for(int i=2; i<=8; i++)
    {
        Alarm_threshold_p +=1;
        *Alarm_threshold_p=flash1_arry[i].f_data;
    }

    return 1;
}

//��ȡflash�ڵ������ʺ�פ���ȵ�Scan_FrqBackx
_Bool Flash2_to_AcceptAPP(void)
{
    uint32_t i = 0;
    uint32_t Address = 0x00;//��¼д��ĵ�ַ
	
	uint8_t forward_power[240];
//	uint32_t temp[424];

    /* ���д��������Ƿ���ȷ */
    Address = WRITE_START_ADDR_second;
    flash_2_once_flag=*(__IO uint32_t*) Address;
    System.already_swept = flash_2_once_flag;

//    if( System.already_swept != 1 )
//    {
//        return 0;
//    }

    Address = Address + 4;
    for(i=0; i<466 && Address < WRITE_END_ADDR_second ; i++)
    {
        flash2_arry[i].i_data=(*(__IO uint32_t*) Address);
        Address = Address + 4;
    }

    for(i=0; i<10 && Address < WRITE_END_ADDR_second ; i++)
    {
        freq_band_time_stamp[i]=(*(__IO uint32_t*) Address);
        Address = Address + 4;
    }

    memcpy(SWR_array, flash2_arry, 233*4);						//פ����
    memcpy(Forward_Power_array, flash2_arry+233, 233*4);		//������

	for(int i=0; i<233; i++)
	{
		forward_power[i] = (uint8_t)Forward_Power_array[i];
	}
		memcpy(Scan_FrqBack1.swr,SWR_array,20*4);			  //3.2-4.9
    memcpy(Scan_FrqBack2.swr,SWR_array+20,20*4);		//5-6.9
    memcpy(Scan_FrqBack3.swr,	SWR_array+40,	20*4);	//7-8.9
    memcpy(Scan_FrqBack4.swr,	SWR_array+60,	20*4);	//9-10.9
    memcpy(Scan_FrqBack5.swr,	SWR_array+80,	20*4);	//11-12.9
    memcpy(Scan_FrqBack6.swr,	SWR_array+100,	20*4);	//13-14.9
    memcpy(Scan_FrqBack7.swr,	SWR_array+120,	20*4);	//15-16.9
    memcpy(Scan_FrqBack8.swr,	SWR_array+140,	20*4);	//17-18.9
    memcpy(Scan_FrqBack9.swr,	SWR_array+160,	20*4);	//19-20.9
    memcpy(Scan_FrqBack10.swr,	SWR_array+180,	20*4);	//21-22.9
    memcpy(Scan_FrqBack11.swr,	SWR_array+200,	20*4);	//23-24.9
    memcpy(Scan_FrqBack12.swr,	SWR_array+220,	12*4);	//25-26.1

	  memcpy(Scan_FrqBack1.Power_UP,	forward_power,		20);	//3.2-4.9
    memcpy(Scan_FrqBack2.Power_UP,	forward_power+20,		20);	//5-6.9
    memcpy(Scan_FrqBack3.Power_UP,	forward_power+40,		20);	//7-8.9
    memcpy(Scan_FrqBack4.Power_UP,	forward_power+60,		20);	//9-10.9
    memcpy(Scan_FrqBack5.Power_UP,	forward_power+80,		20);	//11-12.9
    memcpy(Scan_FrqBack6.Power_UP,	forward_power+100,	20);	//13-14.9
    memcpy(Scan_FrqBack7.Power_UP,	forward_power+120,	20);	//15-16.9
    memcpy(Scan_FrqBack8.Power_UP,	forward_power+140,	20);	//17-18.9
    memcpy(Scan_FrqBack9.Power_UP,	forward_power+160,	20);	//19-20.9
    memcpy(Scan_FrqBack10.Power_UP,	forward_power+180,	20);	//21-22.9
    memcpy(Scan_FrqBack11.Power_UP,	forward_power+200,	20);	//23-24.9
    memcpy(Scan_FrqBack12.Power_UP,	forward_power+220,	12);	//25-26.1
    return 1;
}

/**
  * @brief  ��ȡfalsh��������10������ͼ����ӦRun_Diagram_buf[n]
  * @param  judge_read_diagram����ȡ��һ������������ͼ
  * @retval None
  */
int Flash_Read_Run_Diagram(uint32_t start,uint32_t end,uint8_t judge_read_diagram)
{
    uint32_t Address = 0x00;				//��¼д��ĵ�ַ
    uint8_t *Flash_p=NULL;

    switch(judge_read_diagram)
    {
		case 1:			Flash_p=Flash_p1;			break;
		case 2:			Flash_p=Flash_p2;			break;
		case 3:			Flash_p=Flash_p3;			break;
		case 4:			Flash_p=Flash_p4;			break;
		case 5:			Flash_p=Flash_p5;			break;
		case 6:			Flash_p=Flash_p6;			break;
		case 7:			Flash_p=Flash_p7;			break;
		case 8:			Flash_p=Flash_p8;			break;
		case 9:			Flash_p=Flash_p9;			break;
		case 10:		Flash_p=Flash_p10;			break;
		default:		App_printf("No match parameter!\r\n");		break;
    }

    if(Flash_p==NULL)
    {
        return -1;
    }

    Address = start;
    for(int i=0; i<100 && Address < end ; i++)
    {
        *Flash_p++=*(__IO uint32_t*) Address;	//sorry,flash 4 byteȫ��1��ֵ��uint8_t �����Ϊ255�������ٸ��ӵ��ж��ˣ�ѡ���������ʽ
        Address = Address + 4;
    }

    return 0;
}


//��ȡ����ͼ��־λ������
_Bool Flash3_to_AcceptAPP(void)
{
    uint32_t Address = 0x00;				//��¼д��ĵ�ַ

    /* ���д��������Ƿ���ȷ */
    Address = WRITE_START_ADDR_third;
    flash_3_once_flag=*(__IO uint32_t*) Address;
    Address=Address+4;

    flash3_Save=*(__IO uint32_t*) Address;

//    App_printf("flash_3_once_flag=%d, flash3_Save=%d\r\n",flash_3_once_flag,flash3_Save);
//    if(flash_3_once_flag==1)
//    {
//        for(int i=0; i<100; i++)
//        {
//            if(i<4||(i>=5&&i<=7)||(i<14&&i>=10)||(i>=15&&i<=17)||(i<24&&i>=20)||(i>=25&&i<=27)||(i<34&&i>=30)\
//                    ||(i>=35&&i<=37)||(i<44&&i>=40)||(i>=45&&i<=47)||(i<54&&i>=50)||(i>=55&&i<=57)||(i<64&&i>=60)||\
//                    (i>=65&&i<=67)||(i<74&&i>=70)||(i>=75&&i<=77)||(i<84&&i>=80)||(i>=85&&i<=87)||(i<94&&i>=90))
//                App_printf("%02x ",Flash_p1[i]);
//        }
//        App_printf("\r\n");
//    }
//
    return 0;
}


