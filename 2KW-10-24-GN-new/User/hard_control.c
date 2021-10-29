#include "hard_control.h"
#include "bsp_uart_fifo.h"
#include "my_protocol.h"

#include "main.h"

void juge_need_open_close(uint8_t Save_count)
{
	if( System.emission == 0x01 )		//���ڷ��䣬�϶��Ѿ�����������Ҫ���������ܹػ�
    {
        Monitor.need_open=0;
		    Monitor.need_close=0;
        return ;
    }
	else	if( System.open == 0x01 )	//���ڿ���
	{
		return;
	}
	else	if( Save_count == 0x00 )	//û����ʱ���ػ�
	{
		Monitor.need_open = 0x00;
		Monitor.need_close = 0x01;
		return;
	}

	juge_need_open(Save_count);
	
	if( (Monitor.need_open==0) && (Monitor.need_close==0) )
	{
		juge_need_close(Save_count);
	}
	
	if( System.open == 0x00 )			//�Ѿ��ػ�
	{
		Monitor.need_close=0;			//����Ҫ�ػ�
	}
	else	if( System.open == 0x02 )	//�Ѿ�����
	{
		Monitor.need_open=0;			//����Ҫ����
	}
}

/*�ж��Ƿ���Ҫ����*/
void juge_need_open(uint8_t Save_count)
{
	uint8_t i = 0;
    uint8_t j = 0;
    uint32_t min;					//ʵʱʱ��
    uint32_t run_diagram_min_start;	//����ͼ��ʼʱ��
	uint32_t run_diagram_min_end;	//����ͼ����ʱ��
	
    update_RTCtime( (RTC_GetCounter()+g_stamp_distance), &set_time );	//��ʱ���ת��Ϊ������set_time
    min=set_time.tm_hour*60+set_time.tm_min;
	if( min < BEFOREHAND_OPEN_TIME )
		min += 24*60;
	
    /*------------------------------------------����(�ѿ���24��ǰ��)------------------------------------------*/
	for(i=0; i<Save_count; i++)
	{
		for(j=0; j<10; j++)
		{
			run_diagram_min_start	= Run_Diagram_buf[i].Start_Time1[0]*60+Run_Diagram_buf[i].Start_Time1[1];	//�������Ч���ݣ���0xFF�������ܻῪ��
			run_diagram_min_end		= Run_Diagram_buf[i].End_Time1[0]*60+Run_Diagram_buf[i].End_Time1[1];		//00:00������24:00
			if( run_diagram_min_start < BEFOREHAND_OPEN_TIME )
				run_diagram_min_start += 24*60;
			if( run_diagram_min_end < BEFOREHAND_OPEN_TIME )
				run_diagram_min_end += 24*60;
			if( (min < run_diagram_min_start) && ( (min + BEFOREHAND_OPEN_TIME) >= run_diagram_min_start) )		//����ʱ��С������ͼ��ʼʱ�䣬��������ʱ��+��ǰ����ʱ���������ͼ��ʼʱ��
			{
				Monitor.need_open=1;	/*��Ҫ����*/
				Monitor.need_close=0;	/*���ܹػ�*/
				return;
			}
			else	if( (run_diagram_min_start <= min) && (min < run_diagram_min_end) )							//����δ����
			{
				Monitor.need_open=1;	/*��Ҫ����*/
				Monitor.need_close=0;	/*���ܹػ�*/
				return;
			}

			run_diagram_min_start	= Run_Diagram_buf[i].Start_Time2[0]*60+Run_Diagram_buf[i].Start_Time2[1];
			run_diagram_min_end		= Run_Diagram_buf[i].End_Time2[0]*60+Run_Diagram_buf[i].End_Time2[1];
			if( run_diagram_min_start < BEFOREHAND_OPEN_TIME )
				run_diagram_min_start += 24*60;
			if( run_diagram_min_end < BEFOREHAND_OPEN_TIME )
				run_diagram_min_end += 24*60;
			if( (min < run_diagram_min_start) && ( (min + BEFOREHAND_OPEN_TIME) >= run_diagram_min_start) )		//����ʱ��С������ͼ��ʼʱ�䣬��������ʱ��+��ǰ����ʱ���������ͼ��ʼʱ��
			{
				Monitor.need_open=1;	/*��Ҫ����*/
				Monitor.need_close=0;	/*���ܹػ�*/
				return;
			}
			else	if( (run_diagram_min_start <= min) && (min < run_diagram_min_end) )							//����δ����
			{
				Monitor.need_open=1;	/*��Ҫ����*/
				Monitor.need_close=0;	/*���ܹػ�*/
				return;
			}

			run_diagram_min_start	= Run_Diagram_buf[i].Start_Time3[0]*60+Run_Diagram_buf[i].Start_Time3[1];
			run_diagram_min_end		= Run_Diagram_buf[i].End_Time3[0]*60+Run_Diagram_buf[i].End_Time3[1];
			if( run_diagram_min_start < BEFOREHAND_OPEN_TIME )
				run_diagram_min_start += 24*60;
			if( run_diagram_min_end < BEFOREHAND_OPEN_TIME )
				run_diagram_min_end += 24*60;
			if( (min < run_diagram_min_start) && ( (min + BEFOREHAND_OPEN_TIME) >= run_diagram_min_start) )		//����ʱ��С������ͼ��ʼʱ�䣬��������ʱ��+��ǰ����ʱ���������ͼ��ʼʱ��
			{
				Monitor.need_open=1;	/*��Ҫ����*/
				Monitor.need_close=0;	/*���ܹػ�*/
				return;
			}
			else	if( (run_diagram_min_start <= min) && (min < run_diagram_min_end) )							//����δ����
			{
				Monitor.need_open=1;	/*��Ҫ����*/
				Monitor.need_close=0;	/*���ܹػ�*/
				return;
			}

			run_diagram_min_start	= Run_Diagram_buf[i].Start_Time4[0]*60+Run_Diagram_buf[i].Start_Time4[1];
			run_diagram_min_end		= Run_Diagram_buf[i].End_Time4[0]*60+Run_Diagram_buf[i].End_Time4[1];
			if( run_diagram_min_start < BEFOREHAND_OPEN_TIME )
				run_diagram_min_start += 24*60;
			if( run_diagram_min_end < BEFOREHAND_OPEN_TIME )
				run_diagram_min_end += 24*60;
			if( (min < run_diagram_min_start) && ( (min + BEFOREHAND_OPEN_TIME) >= run_diagram_min_start) )		//����ʱ��С������ͼ��ʼʱ�䣬��������ʱ��+��ǰ����ʱ���������ͼ��ʼʱ��
			{
				Monitor.need_open=1;	/*��Ҫ����*/
				Monitor.need_close=0;	/*���ܹػ�*/
				return;
			}
			else	if( (run_diagram_min_start <= min) && (min < run_diagram_min_end) )							//����δ����
			{
				Monitor.need_open=1;	/*��Ҫ����*/
				Monitor.need_close=0;	/*���ܹػ�*/
				return;
			}

			run_diagram_min_start	= Run_Diagram_buf[i].Start_Time5[0]*60+Run_Diagram_buf[i].Start_Time5[1];
			run_diagram_min_end		= Run_Diagram_buf[i].End_Time5[0]*60+Run_Diagram_buf[i].End_Time5[1];
			if( run_diagram_min_start < BEFOREHAND_OPEN_TIME )
				run_diagram_min_start += 24*60;
			if( run_diagram_min_end < BEFOREHAND_OPEN_TIME )
				run_diagram_min_end += 24*60;
			if( (min < run_diagram_min_start) && ( (min + BEFOREHAND_OPEN_TIME) >= run_diagram_min_start) )		//����ʱ��С������ͼ��ʼʱ�䣬��������ʱ��+��ǰ����ʱ���������ͼ��ʼʱ��
			{
				Monitor.need_open=1;	/*��Ҫ����*/
				Monitor.need_close=0;	/*���ܹػ�*/
				return;
			}
			else	if( (run_diagram_min_start <= min) && (min < run_diagram_min_end) )							//����δ����
			{
				Monitor.need_open=1;	/*��Ҫ����*/
				Monitor.need_close=0;	/*���ܹػ�*/
				return;
			}

			run_diagram_min_start	= Run_Diagram_buf[i].Start_Time6[0]*60+Run_Diagram_buf[i].Start_Time6[1];
			run_diagram_min_end		= Run_Diagram_buf[i].End_Time6[0]*60+Run_Diagram_buf[i].End_Time6[1];
			if( run_diagram_min_start < BEFOREHAND_OPEN_TIME )
				run_diagram_min_start += 24*60;
			if( run_diagram_min_end < BEFOREHAND_OPEN_TIME )
				run_diagram_min_end += 24*60;
			if( (min < run_diagram_min_start) && ( (min + BEFOREHAND_OPEN_TIME) >= run_diagram_min_start) )		//����ʱ��С������ͼ��ʼʱ�䣬��������ʱ��+��ǰ����ʱ���������ͼ��ʼʱ��
			{
				Monitor.need_open=1;	/*��Ҫ����*/
				Monitor.need_close=0;	/*���ܹػ�*/
				return;
			}
			else	if( (run_diagram_min_start <= min) && (min < run_diagram_min_end) )							//����δ����
			{
				Monitor.need_open=1;	/*��Ҫ����*/
				Monitor.need_close=0;	/*���ܹػ�*/
				return;
			}

			run_diagram_min_start	= Run_Diagram_buf[i].Start_Time7[0]*60+Run_Diagram_buf[i].Start_Time7[1];
			run_diagram_min_end		= Run_Diagram_buf[i].End_Time7[0]*60+Run_Diagram_buf[i].End_Time7[1];
			if( run_diagram_min_start < BEFOREHAND_OPEN_TIME )
				run_diagram_min_start += 24*60;
			if( run_diagram_min_end < BEFOREHAND_OPEN_TIME )
				run_diagram_min_end += 24*60;
			if( (min < run_diagram_min_start) && ( (min + BEFOREHAND_OPEN_TIME) >= run_diagram_min_start) )		//����ʱ��С������ͼ��ʼʱ�䣬��������ʱ��+��ǰ����ʱ���������ͼ��ʼʱ��
			{
				Monitor.need_open=1;	/*��Ҫ����*/
				Monitor.need_close=0;	/*���ܹػ�*/
				return;
			}
			else	if( (run_diagram_min_start <= min) && (min < run_diagram_min_end) )							//����δ����
			{
				Monitor.need_open=1;	/*��Ҫ����*/
				Monitor.need_close=0;	/*���ܹػ�*/
				return;
			}

			run_diagram_min_start	= Run_Diagram_buf[i].Start_Time8[0]*60+Run_Diagram_buf[i].Start_Time8[1];
			run_diagram_min_end		= Run_Diagram_buf[i].End_Time8[0]*60+Run_Diagram_buf[i].End_Time8[1];
			if( run_diagram_min_start < BEFOREHAND_OPEN_TIME )
				run_diagram_min_start += 24*60;
			if( run_diagram_min_end < BEFOREHAND_OPEN_TIME )
				run_diagram_min_end += 24*60;
			if( (min < run_diagram_min_start) && ( (min + BEFOREHAND_OPEN_TIME) >= run_diagram_min_start) )		//����ʱ��С������ͼ��ʼʱ�䣬��������ʱ��+��ǰ����ʱ���������ͼ��ʼʱ��
			{
				Monitor.need_open=1;	/*��Ҫ����*/
				Monitor.need_close=0;	/*���ܹػ�*/
				return;
			}
			else	if( (run_diagram_min_start <= min) && (min < run_diagram_min_end) )							//����δ����
			{
				Monitor.need_open=1;	/*��Ҫ����*/
				Monitor.need_close=0;	/*���ܹػ�*/
				return;
			}

			run_diagram_min_start	= Run_Diagram_buf[i].Start_Time9[0]*60+Run_Diagram_buf[i].Start_Time9[1];
			run_diagram_min_end		= Run_Diagram_buf[i].End_Time9[0]*60+Run_Diagram_buf[i].End_Time9[1];
			if( run_diagram_min_start < BEFOREHAND_OPEN_TIME )
				run_diagram_min_start += 24*60;
			if( run_diagram_min_end < BEFOREHAND_OPEN_TIME )
				run_diagram_min_end += 24*60;
			if( (min < run_diagram_min_start) && ( (min + BEFOREHAND_OPEN_TIME) >= run_diagram_min_start) )		//����ʱ��С������ͼ��ʼʱ�䣬��������ʱ��+��ǰ����ʱ���������ͼ��ʼʱ��
			{
				Monitor.need_open=1;	/*��Ҫ����*/
				Monitor.need_close=0;	/*���ܹػ�*/
				return;
			}
			else	if( (run_diagram_min_start <= min) && (min < run_diagram_min_end) )							//����δ����
			{
				Monitor.need_open=1;	/*��Ҫ����*/
				Monitor.need_close=0;	/*���ܹػ�*/
				return;
			}

			run_diagram_min_start	= Run_Diagram_buf[i].Start_Time10[0]*60+Run_Diagram_buf[i].Start_Time10[1];
			run_diagram_min_end		= Run_Diagram_buf[i].End_Time10[0]*60+Run_Diagram_buf[i].End_Time10[1];
			if( run_diagram_min_start < BEFOREHAND_OPEN_TIME )
				run_diagram_min_start += 24*60;
			if( run_diagram_min_end < BEFOREHAND_OPEN_TIME )
				run_diagram_min_end += 24*60;
			if( (min < run_diagram_min_start) && ( (min + BEFOREHAND_OPEN_TIME) >= run_diagram_min_start) )		//����ʱ��С������ͼ��ʼʱ�䣬��������ʱ��+��ǰ����ʱ���������ͼ��ʼʱ��
			{
				Monitor.need_open=1;	/*��Ҫ����*/
				Monitor.need_close=0;	/*���ܹػ�*/
				return;
			}
			else	if( (run_diagram_min_start <= min) && (min < run_diagram_min_end) )							//����δ����
			{
				Monitor.need_open=1;	/*��Ҫ����*/
				Monitor.need_close=0;	/*���ܹػ�*/
				return;
			}
		}
	}
}

/*�ж��Ƿ���Ҫ�ػ�*/
void juge_need_close(uint8_t Save_count)
{
    uint8_t i = 0;
    uint8_t j = 0;
    uint32_t min;					//ʵʱʱ��
    uint32_t run_diagram_min_start;	//����ͼ��ʼʱ��
	
    update_RTCtime( (RTC_GetCounter()+g_stamp_distance), &set_time );	//��ʱ���ת��Ϊ������set_time
    min=set_time.tm_hour*60+set_time.tm_min;
	if( min < BEFOREHAND_OPEN_TIME )
		min += 24*60;
	
    /*------------------------------------------�Ƿ���Ҫ�г���Ϣ--�ػ�------------------------------------------*/
	for(i=0; i<Save_count; i++)
	{
		for(j=0; j<10; j++)
		{
			run_diagram_min_start	= Run_Diagram_buf[i].Start_Time1[0]*60+Run_Diagram_buf[i].Start_Time1[1];	//�������Ч���ݣ���0xFF�������ܻῪ��
			if( run_diagram_min_start < BEFOREHAND_OPEN_TIME )
				run_diagram_min_start += 24*60;
			if( (run_diagram_min_start <= BEFOREHAND_OPEN_TIME + 24*60) && ( (min + BEFOREHAND_OPEN_TIME) <= run_diagram_min_start) )		//����ͼ��ʼʱ��С����ǰ����ʱ��+24*60������Ч���ݣ���������ʱ��+��ǰ����ʱ��С������ͼ��ʼʱ��
			{
				Monitor.need_open=0;
				Monitor.need_close=1;	/*��Ҫ�ػ�*/
				return;
			}

			run_diagram_min_start	= Run_Diagram_buf[i].Start_Time2[0]*60+Run_Diagram_buf[i].Start_Time2[1];
			if( run_diagram_min_start < BEFOREHAND_OPEN_TIME )
				run_diagram_min_start += 24*60;
			if( (run_diagram_min_start <= BEFOREHAND_OPEN_TIME + 24*60) && ( (min + BEFOREHAND_OPEN_TIME) <= run_diagram_min_start) )		//����ͼ��ʼʱ��С����ǰ����ʱ��+24*60������Ч���ݣ���������ʱ��+��ǰ����ʱ��С������ͼ��ʼʱ��
			{
				Monitor.need_open=0;
				Monitor.need_close=1;	/*��Ҫ�ػ�*/
				return;
			}

			run_diagram_min_start	= Run_Diagram_buf[i].Start_Time3[0]*60+Run_Diagram_buf[i].Start_Time3[1];
			if( run_diagram_min_start < BEFOREHAND_OPEN_TIME )
				run_diagram_min_start += 24*60;
			if( (run_diagram_min_start <= BEFOREHAND_OPEN_TIME + 24*60) && ( (min + BEFOREHAND_OPEN_TIME) <= run_diagram_min_start) )		//����ͼ��ʼʱ��С����ǰ����ʱ��+24*60������Ч���ݣ���������ʱ��+��ǰ����ʱ��С������ͼ��ʼʱ��
			{
				Monitor.need_open=0;
				Monitor.need_close=1;	/*��Ҫ�ػ�*/
				return;
			}

			run_diagram_min_start	= Run_Diagram_buf[i].Start_Time4[0]*60+Run_Diagram_buf[i].Start_Time4[1];
			if( run_diagram_min_start < BEFOREHAND_OPEN_TIME )
				run_diagram_min_start += 24*60;
			if( (run_diagram_min_start <= BEFOREHAND_OPEN_TIME + 24*60) && ( (min + BEFOREHAND_OPEN_TIME) <= run_diagram_min_start) )		//����ͼ��ʼʱ��С����ǰ����ʱ��+24*60������Ч���ݣ���������ʱ��+��ǰ����ʱ��С������ͼ��ʼʱ��
			{
				Monitor.need_open=0;
				Monitor.need_close=1;	/*��Ҫ�ػ�*/
				return;
			}

			run_diagram_min_start	= Run_Diagram_buf[i].Start_Time5[0]*60+Run_Diagram_buf[i].Start_Time5[1];
			if( run_diagram_min_start < BEFOREHAND_OPEN_TIME )
				run_diagram_min_start += 24*60;
			if( (run_diagram_min_start <= BEFOREHAND_OPEN_TIME + 24*60) && ( (min + BEFOREHAND_OPEN_TIME) <= run_diagram_min_start) )		//����ͼ��ʼʱ��С����ǰ����ʱ��+24*60������Ч���ݣ���������ʱ��+��ǰ����ʱ��С������ͼ��ʼʱ��
			{
				Monitor.need_open=0;
				Monitor.need_close=1;	/*��Ҫ�ػ�*/
				return;
			}

			run_diagram_min_start	= Run_Diagram_buf[i].Start_Time6[0]*60+Run_Diagram_buf[i].Start_Time6[1];
			if( run_diagram_min_start < BEFOREHAND_OPEN_TIME )
				run_diagram_min_start += 24*60;
			if( (run_diagram_min_start <= BEFOREHAND_OPEN_TIME + 24*60) && ( (min + BEFOREHAND_OPEN_TIME) <= run_diagram_min_start) )		//����ͼ��ʼʱ��С����ǰ����ʱ��+24*60������Ч���ݣ���������ʱ��+��ǰ����ʱ��С������ͼ��ʼʱ��
			{
				Monitor.need_open=0;
				Monitor.need_close=1;	/*��Ҫ�ػ�*/
				return;
			}

			run_diagram_min_start	= Run_Diagram_buf[i].Start_Time7[0]*60+Run_Diagram_buf[i].Start_Time7[1];
			if( run_diagram_min_start < BEFOREHAND_OPEN_TIME )
				run_diagram_min_start += 24*60;
			if( (run_diagram_min_start <= BEFOREHAND_OPEN_TIME + 24*60) && ( (min + BEFOREHAND_OPEN_TIME) <= run_diagram_min_start) )		//����ͼ��ʼʱ��С����ǰ����ʱ��+24*60������Ч���ݣ���������ʱ��+��ǰ����ʱ��С������ͼ��ʼʱ��
			{
				Monitor.need_open=0;
				Monitor.need_close=1;	/*��Ҫ�ػ�*/
				return;
			}

			run_diagram_min_start	= Run_Diagram_buf[i].Start_Time8[0]*60+Run_Diagram_buf[i].Start_Time8[1];
			if( run_diagram_min_start < BEFOREHAND_OPEN_TIME )
				run_diagram_min_start += 24*60;
			if( (run_diagram_min_start <= BEFOREHAND_OPEN_TIME + 24*60) && ( (min + BEFOREHAND_OPEN_TIME) <= run_diagram_min_start) )		//����ͼ��ʼʱ��С����ǰ����ʱ��+24*60������Ч���ݣ���������ʱ��+��ǰ����ʱ��С������ͼ��ʼʱ��
			{
				Monitor.need_open=0;
				Monitor.need_close=1;	/*��Ҫ�ػ�*/
				return;
			}

			run_diagram_min_start	= Run_Diagram_buf[i].Start_Time9[0]*60+Run_Diagram_buf[i].Start_Time9[1];
			if( run_diagram_min_start < BEFOREHAND_OPEN_TIME )
				run_diagram_min_start += 24*60;
			if( (run_diagram_min_start <= BEFOREHAND_OPEN_TIME + 24*60) && ( (min + BEFOREHAND_OPEN_TIME) <= run_diagram_min_start) )		//����ͼ��ʼʱ��С����ǰ����ʱ��+24*60������Ч���ݣ���������ʱ��+��ǰ����ʱ��С������ͼ��ʼʱ��
			{
				Monitor.need_open=0;
				Monitor.need_close=1;	/*��Ҫ�ػ�*/
				return;
			}

			run_diagram_min_start	= Run_Diagram_buf[i].Start_Time10[0]*60+Run_Diagram_buf[i].Start_Time10[1];
			if( run_diagram_min_start < BEFOREHAND_OPEN_TIME )
				run_diagram_min_start += 24*60;
			if( (run_diagram_min_start <= BEFOREHAND_OPEN_TIME + 24*60) && ( (min + BEFOREHAND_OPEN_TIME) <= run_diagram_min_start) )		//����ͼ��ʼʱ��С����ǰ����ʱ��+24*60������Ч���ݣ���������ʱ��+��ǰ����ʱ��С������ͼ��ʼʱ��
			{
				Monitor.need_open=0;
				Monitor.need_close=1;	/*��Ҫ�ػ�*/
				return;
			}
		}
	}

    /*-------------------------------------------�Ƿ�û������--�ػ�---------------------------------------------*/
    if( Monitor.need_close == 0x00 )
    {
        for(i=0; i<Save_count; i++)
        {
            for(j=0; j<10; j++)
            {
                run_diagram_min_start	= Run_Diagram_buf[i].Start_Time1[0]*60+Run_Diagram_buf[i].Start_Time1[1];	//�������Ч���ݣ���0xFF�������ܻῪ��
				if( run_diagram_min_start < BEFOREHAND_OPEN_TIME )
					run_diagram_min_start += 24*60;
				if( (run_diagram_min_start <= BEFOREHAND_OPEN_TIME + 24*60) )										//��Ч����
				{
					Monitor.need_open=0;
					Monitor.need_close=1;	/*��Ҫ�ػ�*/
					return;
				}

				run_diagram_min_start	= Run_Diagram_buf[i].Start_Time2[0]*60+Run_Diagram_buf[i].Start_Time2[1];
				if( run_diagram_min_start < BEFOREHAND_OPEN_TIME )
					run_diagram_min_start += 24*60;
				if( (run_diagram_min_start <= BEFOREHAND_OPEN_TIME + 24*60) )										//��Ч����
				{
					Monitor.need_open=0;
					Monitor.need_close=1;	/*��Ҫ�ػ�*/
					return;
				}

				run_diagram_min_start	= Run_Diagram_buf[i].Start_Time3[0]*60+Run_Diagram_buf[i].Start_Time3[1];
				if( run_diagram_min_start < BEFOREHAND_OPEN_TIME )
					run_diagram_min_start += 24*60;
				if( (run_diagram_min_start <= BEFOREHAND_OPEN_TIME + 24*60) )										//��Ч����
				{
					Monitor.need_open=0;
					Monitor.need_close=1;	/*��Ҫ�ػ�*/
					return;
				}

				run_diagram_min_start	= Run_Diagram_buf[i].Start_Time4[0]*60+Run_Diagram_buf[i].Start_Time4[1];
				if( run_diagram_min_start < BEFOREHAND_OPEN_TIME )
					run_diagram_min_start += 24*60;
				if( (run_diagram_min_start <= BEFOREHAND_OPEN_TIME + 24*60) )										//��Ч����
				{
					Monitor.need_open=0;
					Monitor.need_close=1;	/*��Ҫ�ػ�*/
					return;
				}

				run_diagram_min_start	= Run_Diagram_buf[i].Start_Time5[0]*60+Run_Diagram_buf[i].Start_Time5[1];
				if( run_diagram_min_start < BEFOREHAND_OPEN_TIME )
					run_diagram_min_start += 24*60;
				if( (run_diagram_min_start <= BEFOREHAND_OPEN_TIME + 24*60) )										//��Ч����
				{
					Monitor.need_open=0;
					Monitor.need_close=1;	/*��Ҫ�ػ�*/
					return;
				}

				run_diagram_min_start	= Run_Diagram_buf[i].Start_Time6[0]*60+Run_Diagram_buf[i].Start_Time6[1];
				if( run_diagram_min_start < BEFOREHAND_OPEN_TIME )
					run_diagram_min_start += 24*60;
				if( (run_diagram_min_start <= BEFOREHAND_OPEN_TIME + 24*60) )										//��Ч����
				{
					Monitor.need_open=0;
					Monitor.need_close=1;	/*��Ҫ�ػ�*/
					return;
				}

				run_diagram_min_start	= Run_Diagram_buf[i].Start_Time7[0]*60+Run_Diagram_buf[i].Start_Time7[1];
				if( run_diagram_min_start < BEFOREHAND_OPEN_TIME )
					run_diagram_min_start += 24*60;
				if( (run_diagram_min_start <= BEFOREHAND_OPEN_TIME + 24*60) )										//��Ч����
				{
					Monitor.need_open=0;
					Monitor.need_close=1;	/*��Ҫ�ػ�*/
					return;
				}

				run_diagram_min_start	= Run_Diagram_buf[i].Start_Time8[0]*60+Run_Diagram_buf[i].Start_Time8[1];
				if( run_diagram_min_start < BEFOREHAND_OPEN_TIME )
					run_diagram_min_start += 24*60;
				if( (run_diagram_min_start <= BEFOREHAND_OPEN_TIME + 24*60) )										//��Ч����
				{
					Monitor.need_open=0;
					Monitor.need_close=1;	/*��Ҫ�ػ�*/
					return;
				}

				run_diagram_min_start	= Run_Diagram_buf[i].Start_Time9[0]*60+Run_Diagram_buf[i].Start_Time9[1];
				if( run_diagram_min_start < BEFOREHAND_OPEN_TIME )
					run_diagram_min_start += 24*60;
				if( (run_diagram_min_start <= BEFOREHAND_OPEN_TIME + 24*60) )										//��Ч����
				{
					Monitor.need_open=0;
					Monitor.need_close=1;	/*��Ҫ�ػ�*/
					return;
				}

				run_diagram_min_start	= Run_Diagram_buf[i].Start_Time10[0]*60+Run_Diagram_buf[i].Start_Time10[1];
				if( run_diagram_min_start < BEFOREHAND_OPEN_TIME )
					run_diagram_min_start += 24*60;
				if( (run_diagram_min_start <= BEFOREHAND_OPEN_TIME + 24*60) )										//��Ч����
				{
					Monitor.need_open=0;
					Monitor.need_close=1;	/*��Ҫ�ػ�*/
					return;
				}
            }
        }
    }

}

/* ɨ��flash�ڵ�����ͼ����ʱ�����ͽ��� */
int find_hard_control(uint8_t Save_count)
{
    for(int i=0; i<Save_count; i++)
    {
        if(Monitor.start[i][0]!=1)	//��һ��
        {
            if(Run_Diagram_buf[i].Start_Time1[0]!=0xFF&&((set_time.tm_hour*60+set_time.tm_min)>=(Run_Diagram_buf[i].Start_Time1[0]*60+Run_Diagram_buf[i].Start_Time1[1]))\
                    &&((set_time.tm_hour*60+set_time.tm_min)<(Run_Diagram_buf[i].End_Time1[0]*60+Run_Diagram_buf[i].End_Time1[1])))
            {
                App_printf("\r\nthe %d1 start\r\n", i);
                Monitor.start[i][0]=1;
                Monitor.end[i][0]=0;
                Monitor.usage_diagram_count++;

                if(Monitor.usage_diagram_count==1)
                {
                    Run_Diagram_data.mode='1';
                    set_run_diagram_new_power_level(Run_Diagram_data.mode, Run_Diagram_buf[i].Power1[0]);
                    memcpy(Run_Diagram_data.Freq,Run_Diagram_buf[i].Frq1,4);
                }
                else if(Monitor.usage_diagram_count==2)
                {
                    Run_Diagram_data.mode='2';
                    set_run_diagram_new_power_level(Run_Diagram_data.mode, Run_Diagram_buf[i].Power1[0]);
                    memcpy(Run_Diagram_data.Freq+4,Run_Diagram_buf[i].Frq1,4);
                }
                else if(Monitor.usage_diagram_count==3)
                {
                    Run_Diagram_data.mode='3';
                    set_run_diagram_new_power_level(Run_Diagram_data.mode, Run_Diagram_buf[i].Power1[0]);
                    memcpy(Run_Diagram_data.Freq+8,Run_Diagram_buf[i].Frq1,4);
                }
            }
        }

        if(Monitor.end[i][0]!=1)	//����
        {
            if( ( (set_time.tm_hour == Run_Diagram_buf[i].End_Time1[0]) && (set_time.tm_min == Run_Diagram_buf[i].End_Time1[1]) ) ||
                    ( (set_time.tm_hour == 0) && (set_time.tm_min == 0 ) && (Run_Diagram_buf[i].End_Time1[0] == 24) ) )
            {
                App_printf("\r\nthe %d1 end\r\n", i);
                Monitor.usage_diagram_count--;
                Monitor.end[i][0]=1;
                Monitor.start[i][0]=0;//�����־
                if(Run_Diagram_data.mode=='1')
                {
                    Run_Diagram_data.mode='0';
                    //��Ƶʱ��ʱ�䵽��ֹͣ����
                    memset(Run_Diagram_data.Freq,0,12);//����
                }
                if(Run_Diagram_data.mode=='2')
                {
                    Run_Diagram_data.mode='1';
                    if(memcmp(Run_Diagram_data.Freq,Run_Diagram_buf[i].Frq1,4)==0)   //Run_Diagram_data.Freq��ǰ4λ������ͼ��Ƶ����ͬ���߳���һ��
                    {
                        Run_Diagram_data.Freq[0]=Run_Diagram_data.Freq[4];
                        Run_Diagram_data.Freq[1]=Run_Diagram_data.Freq[5];
                        Run_Diagram_data.Freq[2]=Run_Diagram_data.Freq[6];
                        //���1byte��Ϊ0�����Բ�����
                    }
                    //������߳��ڶ���������Ҫ��������
                }
                if(Run_Diagram_data.mode=='3')
                {
                    Run_Diagram_data.mode='2';
                    if(memcmp(Run_Diagram_data.Freq,Run_Diagram_buf[i].Frq1,4)==0)   //�߳���һ��
                    {
                        Run_Diagram_data.Freq[0]=Run_Diagram_data.Freq[4];
                        Run_Diagram_data.Freq[1]=Run_Diagram_data.Freq[5];
                        Run_Diagram_data.Freq[2]=Run_Diagram_data.Freq[6];
                        //���1byte��Ϊ0�����Բ�����
                        Run_Diagram_data.Freq[4]=Run_Diagram_data.Freq[8];
                        Run_Diagram_data.Freq[5]=Run_Diagram_data.Freq[9];
                        Run_Diagram_data.Freq[6]=Run_Diagram_data.Freq[10];
                    }
                    else if(memcmp((uint8_t *)(Run_Diagram_data.Freq)+4,Run_Diagram_buf[i].Frq1,4)==0)     //�߳��ڶ���
                    {
                        //���1byte��Ϊ0�����Բ�����
                        Run_Diagram_data.Freq[4]=Run_Diagram_data.Freq[8];
                        Run_Diagram_data.Freq[5]=Run_Diagram_data.Freq[9];
                        Run_Diagram_data.Freq[6]=Run_Diagram_data.Freq[10];
                    }
                    //������߳�������������Ҫ��������
                }
            }
        }


        if(Monitor.start[i][1]!=1)	//�ڶ���
        {
            if(Run_Diagram_buf[i].Start_Time2[0]!=0xFF&&((set_time.tm_hour*60+set_time.tm_min)>=(Run_Diagram_buf[i].Start_Time2[0]*60+Run_Diagram_buf[i].Start_Time2[1]))\
                    &&((set_time.tm_hour*60+set_time.tm_min)<(Run_Diagram_buf[i].End_Time2[0]*60+Run_Diagram_buf[i].End_Time2[1])))
            {
                App_printf("\r\nthe %d2 start\r\n", i);
                Monitor.start[i][1]=1;
                Monitor.end[i][1]=0;
                Monitor.usage_diagram_count++;
                if(Monitor.usage_diagram_count==1)
                {
                    Run_Diagram_data.mode='1';
                    set_run_diagram_new_power_level(Run_Diagram_data.mode, Run_Diagram_buf[i].Power2[0]);
                    memcpy(Run_Diagram_data.Freq,Run_Diagram_buf[i].Frq2,4);
                }
                else if(Monitor.usage_diagram_count==2)
                {
                    Run_Diagram_data.mode='2';
                    set_run_diagram_new_power_level(Run_Diagram_data.mode, Run_Diagram_buf[i].Power2[0]);
                    memcpy((uint8_t *)(Run_Diagram_data.Freq)+4,Run_Diagram_buf[i].Frq2,4);
                }
                else if(Monitor.usage_diagram_count==3)
                {
                    Run_Diagram_data.mode='3';
                    set_run_diagram_new_power_level(Run_Diagram_data.mode, Run_Diagram_buf[i].Power2[0]);
                    memcpy((uint8_t *)(Run_Diagram_data.Freq)+8,Run_Diagram_buf[i].Frq2,4);
                }
            }
        }

        if(Monitor.end[i][1]!=1)	//����
        {
            if( ( (set_time.tm_hour == Run_Diagram_buf[i].End_Time2[0]) && (set_time.tm_min == Run_Diagram_buf[i].End_Time2[1]) ) ||
                    ( (set_time.tm_hour == 0) && (set_time.tm_min == 0 ) && (Run_Diagram_buf[i].End_Time2[0] == 24) ) )
            {
                App_printf("\r\nthe %d2 end\r\n", i);
                Monitor.usage_diagram_count--;
                Monitor.end[i][1]=1;
                Monitor.start[i][1]=0;
                if(Run_Diagram_data.mode=='1')
                {
                    Run_Diagram_data.mode='0';
                    //��Ƶʱ��ʱ�䵽��ֹͣ����
                    memset(Run_Diagram_data.Freq,0,12);//����
                }
                if(Run_Diagram_data.mode=='2')
                {
                    Run_Diagram_data.mode='1';
                    if(memcmp(Run_Diagram_data.Freq,Run_Diagram_buf[i].Frq2,4)==0)   //�߳���һ��
                    {
                        Run_Diagram_data.Freq[0]=Run_Diagram_data.Freq[4];
                        Run_Diagram_data.Freq[1]=Run_Diagram_data.Freq[5];
                        Run_Diagram_data.Freq[2]=Run_Diagram_data.Freq[6];
                        //���1byte��Ϊ0�����Բ�����
                    }
                    //������߳��ڶ���������Ҫ��������
                }
                if(Run_Diagram_data.mode=='3')
                {
                    Run_Diagram_data.mode='2';
                    if(memcmp(Run_Diagram_data.Freq,Run_Diagram_buf[i].Frq2,4)==0)   //�߳���һ��
                    {
                        Run_Diagram_data.Freq[0]=Run_Diagram_data.Freq[4];
                        Run_Diagram_data.Freq[1]=Run_Diagram_data.Freq[5];
                        Run_Diagram_data.Freq[2]=Run_Diagram_data.Freq[6];
                        //���1byte��Ϊ0�����Բ�����
                        Run_Diagram_data.Freq[4]=Run_Diagram_data.Freq[8];
                        Run_Diagram_data.Freq[5]=Run_Diagram_data.Freq[9];
                        Run_Diagram_data.Freq[6]=Run_Diagram_data.Freq[10];
                    }
                    else if(memcmp((uint8_t *)(Run_Diagram_data.Freq)+4,Run_Diagram_buf[i].Frq2,4)==0)     //�߳��ڶ���
                    {
                        //���1byte��Ϊ0�����Բ�����
                        Run_Diagram_data.Freq[4]=Run_Diagram_data.Freq[8];
                        Run_Diagram_data.Freq[5]=Run_Diagram_data.Freq[9];
                        Run_Diagram_data.Freq[6]=Run_Diagram_data.Freq[10];
                    }
                    //������߳�������������Ҫ��������
                }
            }
        }

        if(Monitor.start[i][2]!=1)	//������
        {
            if(Run_Diagram_buf[i].Start_Time3[0]!=0xFF&&((set_time.tm_hour*60+set_time.tm_min)>=(Run_Diagram_buf[i].Start_Time3[0]*60+Run_Diagram_buf[i].Start_Time3[1]))\
                    &&((set_time.tm_hour*60+set_time.tm_min)<(Run_Diagram_buf[i].End_Time3[0]*60+Run_Diagram_buf[i].End_Time3[1])))
            {
                App_printf("\r\nthe %d3 start\r\n", i);
                Monitor.start[i][2]=1;
                Monitor.end[i][2]=0;
                Monitor.usage_diagram_count++;
                if(Monitor.usage_diagram_count==1)
                {
                    Run_Diagram_data.mode='1';
                    set_run_diagram_new_power_level(Run_Diagram_data.mode, Run_Diagram_buf[i].Power3[0]);
                    memcpy(Run_Diagram_data.Freq,Run_Diagram_buf[i].Frq3,4);
                }
                else if(Monitor.usage_diagram_count==2)
                {
                    Run_Diagram_data.mode='2';
                    set_run_diagram_new_power_level(Run_Diagram_data.mode, Run_Diagram_buf[i].Power3[0]);
                    memcpy((uint8_t *)(Run_Diagram_data.Freq)+4,Run_Diagram_buf[i].Frq3,4);
                }
                else if(Monitor.usage_diagram_count==3)
                {
                    Run_Diagram_data.mode='3';
                    set_run_diagram_new_power_level(Run_Diagram_data.mode, Run_Diagram_buf[i].Power3[0]);
                    memcpy((uint8_t *)(Run_Diagram_data.Freq)+8,Run_Diagram_buf[i].Frq3,4);
                }
            }
        }
        if(Monitor.end[i][2]!=1)	//����
        {
            if( ( (set_time.tm_hour == Run_Diagram_buf[i].End_Time3[0]) && (set_time.tm_min == Run_Diagram_buf[i].End_Time3[1]) ) ||
                    ( (set_time.tm_hour == 0) && (set_time.tm_min == 0 ) && (Run_Diagram_buf[i].End_Time3[0] == 24) ) )
            {
                App_printf("\r\nthe %d3 end\r\n", i);
                Monitor.usage_diagram_count--;
                Monitor.end[i][2]=1;
                Monitor.start[i][2]=0;
                if(Run_Diagram_data.mode=='1')
                {
                    Run_Diagram_data.mode='0';
                    //��Ƶʱ��ʱ�䵽��ֹͣ����
                    memset(Run_Diagram_data.Freq,0,12);//����
                }
                if(Run_Diagram_data.mode=='2')
                {
                    Run_Diagram_data.mode='1';
                    if(memcmp(Run_Diagram_data.Freq,Run_Diagram_buf[i].Frq3,4)==0)   //�߳���һ��
                    {
                        Run_Diagram_data.Freq[0]=Run_Diagram_data.Freq[4];
                        Run_Diagram_data.Freq[1]=Run_Diagram_data.Freq[5];
                        Run_Diagram_data.Freq[2]=Run_Diagram_data.Freq[6];
                        //���1byte��Ϊ0�����Բ�����
                    }
                    //������߳��ڶ���������Ҫ��������
                }
                if(Run_Diagram_data.mode=='3')
                {
                    Run_Diagram_data.mode='2';
                    if(memcmp(Run_Diagram_data.Freq,Run_Diagram_buf[i].Frq3,4)==0)   //�߳���һ��
                    {
                        Run_Diagram_data.Freq[0]=Run_Diagram_data.Freq[4];
                        Run_Diagram_data.Freq[1]=Run_Diagram_data.Freq[5];
                        Run_Diagram_data.Freq[2]=Run_Diagram_data.Freq[6];
                        //���1byte��Ϊ0�����Բ�����
                        Run_Diagram_data.Freq[4]=Run_Diagram_data.Freq[8];
                        Run_Diagram_data.Freq[5]=Run_Diagram_data.Freq[9];
                        Run_Diagram_data.Freq[6]=Run_Diagram_data.Freq[10];
                    }
                    else if(memcmp((uint8_t *)(Run_Diagram_data.Freq)+4,Run_Diagram_buf[i].Frq3,4)==0)     //�߳��ڶ���
                    {
                        //���1byte��Ϊ0�����Բ�����
                        Run_Diagram_data.Freq[4]=Run_Diagram_data.Freq[8];
                        Run_Diagram_data.Freq[5]=Run_Diagram_data.Freq[9];
                        Run_Diagram_data.Freq[6]=Run_Diagram_data.Freq[10];
                    }
                    //������߳�������������Ҫ��������
                }
            }
        }


        if(Monitor.start[i][3]!=1)	//���ĸ�
        {
            if(Run_Diagram_buf[i].Start_Time4[0]!=0xFF&&((set_time.tm_hour*60+set_time.tm_min)>=(Run_Diagram_buf[i].Start_Time4[0]*60+Run_Diagram_buf[i].Start_Time4[1]))\
                    &&((set_time.tm_hour*60+set_time.tm_min)<(Run_Diagram_buf[i].End_Time4[0]*60+Run_Diagram_buf[i].End_Time4[1])))
            {
                App_printf("\r\nthe %d4 start\r\n", i);
                Monitor.start[i][3]=1;
                Monitor.end[i][3]=0;
                Monitor.usage_diagram_count++;
                if(Monitor.usage_diagram_count==1)
                {
                    Run_Diagram_data.mode='1';
                    set_run_diagram_new_power_level(Run_Diagram_data.mode, Run_Diagram_buf[i].Power4[0]);
                    memcpy(Run_Diagram_data.Freq,Run_Diagram_buf[i].Frq4,4);
                }
                else if(Monitor.usage_diagram_count==2)
                {
                    Run_Diagram_data.mode='2';
                    set_run_diagram_new_power_level(Run_Diagram_data.mode, Run_Diagram_buf[i].Power4[0]);
                    memcpy((uint8_t *)(Run_Diagram_data.Freq)+4,Run_Diagram_buf[i].Frq4,4);
                }
                else if(Monitor.usage_diagram_count==3)
                {
                    Run_Diagram_data.mode='3';
                    set_run_diagram_new_power_level(Run_Diagram_data.mode, Run_Diagram_buf[i].Power4[0]);
                    memcpy((uint8_t *)(Run_Diagram_data.Freq)+8,Run_Diagram_buf[i].Frq4,4);
                }
            }
        }

        if(Monitor.end[i][3]!=1)	//����
        {
            if( ( (set_time.tm_hour == Run_Diagram_buf[i].End_Time4[0]) && (set_time.tm_min == Run_Diagram_buf[i].End_Time4[1]) ) ||
                    ( (set_time.tm_hour == 0) && (set_time.tm_min == 0 ) && (Run_Diagram_buf[i].End_Time4[0] == 24) ) )
            {
                App_printf("\r\nthe %d4 end\r\n", i);
                Monitor.usage_diagram_count--;
                Monitor.end[i][3]=1;
                Monitor.start[i][3]=0;
                if(Run_Diagram_data.mode=='1')
                {
                    Run_Diagram_data.mode='0';
                    //��Ƶʱ��ʱ�䵽��ֹͣ����
                    memset(Run_Diagram_data.Freq,0,12);//����
                }
                if(Run_Diagram_data.mode=='2')
                {
                    Run_Diagram_data.mode='1';
                    if(memcmp(Run_Diagram_data.Freq,Run_Diagram_buf[i].Frq4,4)==0)   //�߳���һ��
                    {
                        Run_Diagram_data.Freq[0]=Run_Diagram_data.Freq[4];
                        Run_Diagram_data.Freq[1]=Run_Diagram_data.Freq[5];
                        Run_Diagram_data.Freq[2]=Run_Diagram_data.Freq[6];
                        //���1byte��Ϊ0�����Բ�����
                    }
                    //������߳��ڶ���������Ҫ��������
                }
                if(Run_Diagram_data.mode=='3')
                {
                    Run_Diagram_data.mode='2';
                    if(memcmp(Run_Diagram_data.Freq,Run_Diagram_buf[i].Frq4,4)==0)   //�߳���һ��
                    {
                        Run_Diagram_data.Freq[0]=Run_Diagram_data.Freq[4];
                        Run_Diagram_data.Freq[1]=Run_Diagram_data.Freq[5];
                        Run_Diagram_data.Freq[2]=Run_Diagram_data.Freq[6];
                        //���1byte��Ϊ0�����Բ�����
                        Run_Diagram_data.Freq[4]=Run_Diagram_data.Freq[8];
                        Run_Diagram_data.Freq[5]=Run_Diagram_data.Freq[9];
                        Run_Diagram_data.Freq[6]=Run_Diagram_data.Freq[10];
                    }
                    else if(memcmp((uint8_t *)(Run_Diagram_data.Freq)+4,Run_Diagram_buf[i].Frq4,4)==0)     //�߳��ڶ���
                    {
                        //���1byte��Ϊ0�����Բ�����
                        Run_Diagram_data.Freq[4]=Run_Diagram_data.Freq[8];
                        Run_Diagram_data.Freq[5]=Run_Diagram_data.Freq[9];
                        Run_Diagram_data.Freq[6]=Run_Diagram_data.Freq[10];
                    }
                    //������߳�������������Ҫ��������
                }
            }
        }

        if(Monitor.start[i][4]!=1)	//�����
        {
            if(Run_Diagram_buf[i].Start_Time5[0]!=0xFF&&((set_time.tm_hour*60+set_time.tm_min)>=(Run_Diagram_buf[i].Start_Time5[0]*60+Run_Diagram_buf[i].Start_Time5[1]))\
                    &&((set_time.tm_hour*60+set_time.tm_min)<(Run_Diagram_buf[i].End_Time5[0]*60+Run_Diagram_buf[i].End_Time5[1])))
            {
                App_printf("\r\nthe %d5 start\r\n", i);
                Monitor.start[i][4]=1;
                Monitor.end[i][4]=0;
                Monitor.usage_diagram_count++;
                if(Monitor.usage_diagram_count==1)
                {
                    Run_Diagram_data.mode='1';
                    set_run_diagram_new_power_level(Run_Diagram_data.mode, Run_Diagram_buf[i].Power5[0]);
                    memcpy(Run_Diagram_data.Freq,Run_Diagram_buf[i].Frq5,4);
                }
                else if(Monitor.usage_diagram_count==2)
                {
                    Run_Diagram_data.mode='2';
                    set_run_diagram_new_power_level(Run_Diagram_data.mode, Run_Diagram_buf[i].Power5[0]);
                    memcpy((uint8_t *)(Run_Diagram_data.Freq)+4,Run_Diagram_buf[i].Frq5,4);
                }
                else if(Monitor.usage_diagram_count==3)
                {
                    Run_Diagram_data.mode='3';
                    set_run_diagram_new_power_level(Run_Diagram_data.mode, Run_Diagram_buf[i].Power5[0]);
                    memcpy((uint8_t *)(Run_Diagram_data.Freq)+8,Run_Diagram_buf[i].Frq5,4);
                }
            }
        }

        if(Monitor.end[i][4]!=1)	//����
        {
            if( ( (set_time.tm_hour == Run_Diagram_buf[i].End_Time5[0]) && (set_time.tm_min == Run_Diagram_buf[i].End_Time5[1]) ) ||
                    ( (set_time.tm_hour == 0) && (set_time.tm_min == 0 ) && (Run_Diagram_buf[i].End_Time5[0] == 24) ) )
            {
                App_printf("\r\nthe %d5 end\r\n", i);
                Monitor.usage_diagram_count--;
                Monitor.end[i][4]=1;
                Monitor.start[i][4]=0;
                if(Run_Diagram_data.mode=='1')
                {
                    Run_Diagram_data.mode='0';
                    //��Ƶʱ��ʱ�䵽��ֹͣ����
                    memset(Run_Diagram_data.Freq,0,12);//����
                }
                if(Run_Diagram_data.mode=='2')
                {
                    Run_Diagram_data.mode='1';
                    if(memcmp(Run_Diagram_data.Freq,Run_Diagram_buf[i].Frq5,4)==0)   //�߳���һ��
                    {
                        Run_Diagram_data.Freq[0]=Run_Diagram_data.Freq[4];
                        Run_Diagram_data.Freq[1]=Run_Diagram_data.Freq[5];
                        Run_Diagram_data.Freq[2]=Run_Diagram_data.Freq[6];
                        //���1byte��Ϊ0�����Բ�����
                    }
                    //������߳��ڶ���������Ҫ��������
                }
                if(Run_Diagram_data.mode=='3')
                {
                    Run_Diagram_data.mode='2';
                    if(memcmp(Run_Diagram_data.Freq,Run_Diagram_buf[i].Frq5,4)==0)   //�߳���һ��
                    {
                        Run_Diagram_data.Freq[0]=Run_Diagram_data.Freq[4];
                        Run_Diagram_data.Freq[1]=Run_Diagram_data.Freq[5];
                        Run_Diagram_data.Freq[2]=Run_Diagram_data.Freq[6];
                        //���1byte��Ϊ0�����Բ�����
                        Run_Diagram_data.Freq[4]=Run_Diagram_data.Freq[8];
                        Run_Diagram_data.Freq[5]=Run_Diagram_data.Freq[9];
                        Run_Diagram_data.Freq[6]=Run_Diagram_data.Freq[10];
                    }
                    else if(memcmp((uint8_t *)(Run_Diagram_data.Freq)+4,Run_Diagram_buf[i].Frq5,4)==0)     //�߳��ڶ���
                    {
                        //���1byte��Ϊ0�����Բ�����
                        Run_Diagram_data.Freq[4]=Run_Diagram_data.Freq[8];
                        Run_Diagram_data.Freq[5]=Run_Diagram_data.Freq[9];
                        Run_Diagram_data.Freq[6]=Run_Diagram_data.Freq[10];
                    }
                    //������߳�������������Ҫ��������
                }
            }
        }

        if(Monitor.start[i][5]!=1)	//������
        {
            if(Run_Diagram_buf[i].Start_Time6[0]!=0xFF&&((set_time.tm_hour*60+set_time.tm_min)>=(Run_Diagram_buf[i].Start_Time6[0]*60+Run_Diagram_buf[i].Start_Time6[1]))\
                    &&((set_time.tm_hour*60+set_time.tm_min)<(Run_Diagram_buf[i].End_Time6[0]*60+Run_Diagram_buf[i].End_Time6[1])))
            {
                App_printf("\r\nthe %d6 start\r\n", i);
                Monitor.start[i][5]=1;
                Monitor.end[i][5]=0;
                Monitor.usage_diagram_count++;
                if(Monitor.usage_diagram_count==1)
                {
                    Run_Diagram_data.mode='1';
                    set_run_diagram_new_power_level(Run_Diagram_data.mode, Run_Diagram_buf[i].Power6[0]);
                    memcpy(Run_Diagram_data.Freq,Run_Diagram_buf[i].Frq6,4);
                }
                else if(Monitor.usage_diagram_count==2)
                {
                    Run_Diagram_data.mode='2';
                    set_run_diagram_new_power_level(Run_Diagram_data.mode, Run_Diagram_buf[i].Power6[0]);
                    memcpy((uint8_t *)(Run_Diagram_data.Freq)+4,Run_Diagram_buf[i].Frq6,4);
                }
                else if(Monitor.usage_diagram_count==3)
                {
                    Run_Diagram_data.mode='3';
                    set_run_diagram_new_power_level(Run_Diagram_data.mode, Run_Diagram_buf[i].Power6[0]);
                    memcpy((uint8_t *)(Run_Diagram_data.Freq)+8,Run_Diagram_buf[i].Frq6,4);
                }
            }
        }

        if(Monitor.end[i][5]!=1)	//����
        {
            if( ( (set_time.tm_hour == Run_Diagram_buf[i].End_Time6[0]) && (set_time.tm_min == Run_Diagram_buf[i].End_Time6[1]) ) ||
                    ( (set_time.tm_hour == 0) && (set_time.tm_min == 0 ) && (Run_Diagram_buf[i].End_Time6[0] == 24) ) )
            {
                App_printf("\r\nthe %d6 end\r\n", i);
                Monitor.usage_diagram_count--;
                Monitor.end[i][5]=1;
                Monitor.start[i][5]=0;
                if(Run_Diagram_data.mode=='1')
                {
                    Run_Diagram_data.mode='0';
                    //��Ƶʱ��ʱ�䵽��ֹͣ����
                    memset(Run_Diagram_data.Freq,0,12);//����
                }
                if(Run_Diagram_data.mode=='2')
                {
                    Run_Diagram_data.mode='1';
                    if(memcmp(Run_Diagram_data.Freq,Run_Diagram_buf[i].Frq6,4)==0)   //�߳���һ��
                    {
                        Run_Diagram_data.Freq[0]=Run_Diagram_data.Freq[4];
                        Run_Diagram_data.Freq[1]=Run_Diagram_data.Freq[5];
                        Run_Diagram_data.Freq[2]=Run_Diagram_data.Freq[6];
                        //���1byte��Ϊ0�����Բ�����
                    }
                    //������߳��ڶ���������Ҫ��������
                }
                if(Run_Diagram_data.mode=='3')
                {
                    Run_Diagram_data.mode='2';
                    if(memcmp(Run_Diagram_data.Freq,Run_Diagram_buf[i].Frq6,4)==0)   //�߳���һ��
                    {
                        Run_Diagram_data.Freq[0]=Run_Diagram_data.Freq[4];
                        Run_Diagram_data.Freq[1]=Run_Diagram_data.Freq[5];
                        Run_Diagram_data.Freq[2]=Run_Diagram_data.Freq[6];
                        //���1byte��Ϊ0�����Բ�����
                        Run_Diagram_data.Freq[4]=Run_Diagram_data.Freq[8];
                        Run_Diagram_data.Freq[5]=Run_Diagram_data.Freq[9];
                        Run_Diagram_data.Freq[6]=Run_Diagram_data.Freq[10];
                    }
                    else if(memcmp((uint8_t *)(Run_Diagram_data.Freq)+4,Run_Diagram_buf[i].Frq6,4)==0)     //�߳��ڶ���
                    {
                        //���1byte��Ϊ0�����Բ�����
                        Run_Diagram_data.Freq[4]=Run_Diagram_data.Freq[8];
                        Run_Diagram_data.Freq[5]=Run_Diagram_data.Freq[9];
                        Run_Diagram_data.Freq[6]=Run_Diagram_data.Freq[10];
                    }
                    //������߳�������������Ҫ��������
                }
            }
        }

        if(Monitor.start[i][6]!=1)	//���߸�
        {
            if(Run_Diagram_buf[i].Start_Time7[0]!=0xFF&&((set_time.tm_hour*60+set_time.tm_min)>=(Run_Diagram_buf[i].Start_Time7[0]*60+Run_Diagram_buf[i].Start_Time7[1]))\
                    &&((set_time.tm_hour*60+set_time.tm_min)<(Run_Diagram_buf[i].End_Time7[0]*60+Run_Diagram_buf[i].End_Time7[1])))
            {
                App_printf("\r\nthe %d7 start\r\n", i);
                Monitor.start[i][6]=1;
                Monitor.end[i][6]=0;
                Monitor.usage_diagram_count++;
                if(Monitor.usage_diagram_count==1)
                {
                    Run_Diagram_data.mode='1';
                    set_run_diagram_new_power_level(Run_Diagram_data.mode, Run_Diagram_buf[i].Power7[0]);
                    memcpy(Run_Diagram_data.Freq,Run_Diagram_buf[i].Frq7,4);
                }
                else if(Monitor.usage_diagram_count==2)
                {
                    Run_Diagram_data.mode='2';
                    set_run_diagram_new_power_level(Run_Diagram_data.mode, Run_Diagram_buf[i].Power7[0]);
                    memcpy((uint8_t *)(Run_Diagram_data.Freq)+4,Run_Diagram_buf[i].Frq7,4);
                }
                else if(Monitor.usage_diagram_count==3)
                {
                    Run_Diagram_data.mode='3';
                    set_run_diagram_new_power_level(Run_Diagram_data.mode, Run_Diagram_buf[i].Power7[0]);
                    memcpy((uint8_t *)(Run_Diagram_data.Freq)+8,Run_Diagram_buf[i].Frq7,4);
                }
            }
        }
        if(Monitor.end[i][6]!=1)	//����
        {
            if( ( (set_time.tm_hour == Run_Diagram_buf[i].End_Time7[0]) && (set_time.tm_min == Run_Diagram_buf[i].End_Time7[1]) ) ||
                    ( (set_time.tm_hour == 0) && (set_time.tm_min == 0 ) && (Run_Diagram_buf[i].End_Time7[0] == 24) ) )
            {
                App_printf("\r\nthe %d7 end\r\n", i);
                Monitor.usage_diagram_count--;
                Monitor.end[i][6]=1;
                Monitor.start[i][6]=0;
                if(Run_Diagram_data.mode=='1')
                {
                    Run_Diagram_data.mode='0';
                    //��Ƶʱ��ʱ�䵽��ֹͣ����
                    memset(Run_Diagram_data.Freq,0,12);//����
                }
                if(Run_Diagram_data.mode=='2')
                {
                    Run_Diagram_data.mode='1';
                    if(memcmp(Run_Diagram_data.Freq,Run_Diagram_buf[i].Frq7,4)==0)   //�߳���һ��
                    {
                        Run_Diagram_data.Freq[0]=Run_Diagram_data.Freq[4];
                        Run_Diagram_data.Freq[1]=Run_Diagram_data.Freq[5];
                        Run_Diagram_data.Freq[2]=Run_Diagram_data.Freq[6];
                        //���1byte��Ϊ0�����Բ�����
                    }
                    //������߳��ڶ���������Ҫ��������
                }
                if(Run_Diagram_data.mode=='3')
                {
                    Run_Diagram_data.mode='2';
                    if(memcmp(Run_Diagram_data.Freq,Run_Diagram_buf[i].Frq7,4)==0)   //�߳���һ��
                    {
                        Run_Diagram_data.Freq[0]=Run_Diagram_data.Freq[4];
                        Run_Diagram_data.Freq[1]=Run_Diagram_data.Freq[5];
                        Run_Diagram_data.Freq[2]=Run_Diagram_data.Freq[6];
                        //���1byte��Ϊ0�����Բ�����
                        Run_Diagram_data.Freq[4]=Run_Diagram_data.Freq[8];
                        Run_Diagram_data.Freq[5]=Run_Diagram_data.Freq[9];
                        Run_Diagram_data.Freq[6]=Run_Diagram_data.Freq[10];
                    }
                    else if(memcmp((uint8_t *)(Run_Diagram_data.Freq)+4,Run_Diagram_buf[i].Frq7,4)==0)     //�߳��ڶ���
                    {
                        //���1byte��Ϊ0�����Բ�����
                        Run_Diagram_data.Freq[4]=Run_Diagram_data.Freq[8];
                        Run_Diagram_data.Freq[5]=Run_Diagram_data.Freq[9];
                        Run_Diagram_data.Freq[6]=Run_Diagram_data.Freq[10];
                    }
                    //������߳�������������Ҫ��������
                }
            }
        }

        if(Monitor.start[i][7]!=1)	//�ڰ˸�
        {
            if(Run_Diagram_buf[i].Start_Time8[0]!=0xFF&&((set_time.tm_hour*60+set_time.tm_min)>=(Run_Diagram_buf[i].Start_Time8[0]*60+Run_Diagram_buf[i].Start_Time8[1]))\
                    &&((set_time.tm_hour*60+set_time.tm_min)<(Run_Diagram_buf[i].End_Time8[0]*60+Run_Diagram_buf[i].End_Time8[1])))
            {
                App_printf("\r\nthe %d8 start\r\n", i);
                Monitor.start[i][7]=1;
                Monitor.end[i][7]=0;
                Monitor.usage_diagram_count++;
                if(Monitor.usage_diagram_count==1)
                {
                    Run_Diagram_data.mode='1';
                    set_run_diagram_new_power_level(Run_Diagram_data.mode, Run_Diagram_buf[i].Power8[0]);
                    memcpy(Run_Diagram_data.Freq,Run_Diagram_buf[i].Frq8,4);
                }
                else if(Monitor.usage_diagram_count==2)
                {
                    Run_Diagram_data.mode='2';
                    set_run_diagram_new_power_level(Run_Diagram_data.mode, Run_Diagram_buf[i].Power8[0]);
                    memcpy((uint8_t *)(Run_Diagram_data.Freq)+4,Run_Diagram_buf[i].Frq8,4);
                }
                else if(Monitor.usage_diagram_count==3)
                {
                    Run_Diagram_data.mode='3';
                    set_run_diagram_new_power_level(Run_Diagram_data.mode, Run_Diagram_buf[i].Power8[0]);
                    memcpy((uint8_t *)(Run_Diagram_data.Freq)+8,Run_Diagram_buf[i].Frq8,4);
                }
            }
        }
        if(Monitor.end[i][7]!=1)	//����
        {
            if( ( (set_time.tm_hour == Run_Diagram_buf[i].End_Time8[0]) && (set_time.tm_min == Run_Diagram_buf[i].End_Time8[1]) ) ||
                    ( (set_time.tm_hour == 0) && (set_time.tm_min == 0 ) && (Run_Diagram_buf[i].End_Time8[0] == 24) ) )
            {
                App_printf("\r\nthe %d8 end\r\n", i);
                Monitor.usage_diagram_count--;
                Monitor.end[i][7]=1;
                Monitor.start[i][7]=0;
                if(Run_Diagram_data.mode=='1')
                {
                    Run_Diagram_data.mode='0';
                    //��Ƶʱ��ʱ�䵽��ֹͣ����
                    memset(Run_Diagram_data.Freq,0,12);//����
                }
                if(Run_Diagram_data.mode=='2')
                {
                    Run_Diagram_data.mode='1';
                    if(memcmp(Run_Diagram_data.Freq,Run_Diagram_buf[i].Frq8,4)==0)   //�߳���һ��
                    {
                        Run_Diagram_data.Freq[0]=Run_Diagram_data.Freq[4];
                        Run_Diagram_data.Freq[1]=Run_Diagram_data.Freq[5];
                        Run_Diagram_data.Freq[2]=Run_Diagram_data.Freq[6];
                        //���1byte��Ϊ0�����Բ�����
                    }
                    //������߳��ڶ���������Ҫ��������
                }
                if(Run_Diagram_data.mode=='3')
                {
                    Run_Diagram_data.mode='2';
                    if(memcmp(Run_Diagram_data.Freq,Run_Diagram_buf[i].Frq8,4)==0)   //�߳���һ��
                    {
                        Run_Diagram_data.Freq[0]=Run_Diagram_data.Freq[4];
                        Run_Diagram_data.Freq[1]=Run_Diagram_data.Freq[5];
                        Run_Diagram_data.Freq[2]=Run_Diagram_data.Freq[6];
                        //���1byte��Ϊ0�����Բ�����
                        Run_Diagram_data.Freq[4]=Run_Diagram_data.Freq[8];
                        Run_Diagram_data.Freq[5]=Run_Diagram_data.Freq[9];
                        Run_Diagram_data.Freq[6]=Run_Diagram_data.Freq[10];
                    }
                    else if(memcmp((uint8_t *)(Run_Diagram_data.Freq)+4,Run_Diagram_buf[i].Frq8,4)==0)     //�߳��ڶ���
                    {
                        //���1byte��Ϊ0�����Բ�����
                        Run_Diagram_data.Freq[4]=Run_Diagram_data.Freq[8];
                        Run_Diagram_data.Freq[5]=Run_Diagram_data.Freq[9];
                        Run_Diagram_data.Freq[6]=Run_Diagram_data.Freq[10];
                    }
                    //������߳�������������Ҫ��������
                }
            }
        }


        if(Monitor.start[i][8]!=1)	//�ھŸ�
        {
            if(Run_Diagram_buf[i].Start_Time9[0]!=0xFF&&((set_time.tm_hour*60+set_time.tm_min)>=(Run_Diagram_buf[i].Start_Time9[0]*60+Run_Diagram_buf[i].Start_Time9[1]))\
                    &&((set_time.tm_hour*60+set_time.tm_min)<(Run_Diagram_buf[i].End_Time9[0]*60+Run_Diagram_buf[i].End_Time9[1])))
            {
                App_printf("\r\nthe %d9 start\r\n", i);
                Monitor.start[i][8]=1;
                Monitor.end[i][8]=0;
                Monitor.usage_diagram_count++;
                if(Monitor.usage_diagram_count==1)
                {
                    Run_Diagram_data.mode='1';
                    set_run_diagram_new_power_level(Run_Diagram_data.mode, Run_Diagram_buf[i].Power9[0]);
                    memcpy(Run_Diagram_data.Freq,Run_Diagram_buf[i].Frq9,4);
                }
                else if(Monitor.usage_diagram_count==2)
                {
                    Run_Diagram_data.mode='2';
                    set_run_diagram_new_power_level(Run_Diagram_data.mode, Run_Diagram_buf[i].Power9[0]);
                    memcpy((uint8_t *)(Run_Diagram_data.Freq)+4,Run_Diagram_buf[i].Frq9,4);
                }
                else if(Monitor.usage_diagram_count==3)
                {
                    Run_Diagram_data.mode='3';
                    set_run_diagram_new_power_level(Run_Diagram_data.mode, Run_Diagram_buf[i].Power9[0]);
                    memcpy((uint8_t *)(Run_Diagram_data.Freq)+8,Run_Diagram_buf[i].Frq9,4);
                }
            }
        }

        if(Monitor.end[i][8]!=1)	//����
        {
            if( ( (set_time.tm_hour == Run_Diagram_buf[i].End_Time9[0]) && (set_time.tm_min == Run_Diagram_buf[i].End_Time9[1]) ) ||
                    ( (set_time.tm_hour == 0) && (set_time.tm_min == 0 ) && (Run_Diagram_buf[i].End_Time9[0] == 24) ) )
            {
                App_printf("\r\nthe %d9 end\r\n", i);
                Monitor.usage_diagram_count--;
                Monitor.end[i][8]=1;
                Monitor.start[i][8]=0;
                if(Run_Diagram_data.mode=='1')
                {
                    Run_Diagram_data.mode='0';
                    //��Ƶʱ��ʱ�䵽��ֹͣ����
                    memset(Run_Diagram_data.Freq,0,12);//����
                }
                if(Run_Diagram_data.mode=='2')
                {
                    Run_Diagram_data.mode='1';
                    if(memcmp(Run_Diagram_data.Freq,Run_Diagram_buf[i].Frq9,4)==0)   //�߳���һ��
                    {
                        Run_Diagram_data.Freq[0]=Run_Diagram_data.Freq[4];
                        Run_Diagram_data.Freq[1]=Run_Diagram_data.Freq[5];
                        Run_Diagram_data.Freq[2]=Run_Diagram_data.Freq[6];
                        //���1byte��Ϊ0�����Բ�����
                    }
                    //������߳��ڶ���������Ҫ��������
                }
                if(Run_Diagram_data.mode=='3')
                {
                    Run_Diagram_data.mode='2';
                    if(memcmp(Run_Diagram_data.Freq,Run_Diagram_buf[i].Frq9,4)==0)   //�߳���һ��
                    {
                        Run_Diagram_data.Freq[0]=Run_Diagram_data.Freq[4];
                        Run_Diagram_data.Freq[1]=Run_Diagram_data.Freq[5];
                        Run_Diagram_data.Freq[2]=Run_Diagram_data.Freq[6];
                        //���1byte��Ϊ0�����Բ�����
                        Run_Diagram_data.Freq[4]=Run_Diagram_data.Freq[8];
                        Run_Diagram_data.Freq[5]=Run_Diagram_data.Freq[9];
                        Run_Diagram_data.Freq[6]=Run_Diagram_data.Freq[10];
                    }
                    else if(memcmp((uint8_t *)(Run_Diagram_data.Freq)+4,Run_Diagram_buf[i].Frq9,4)==0)     //�߳��ڶ���
                    {
                        //���1byte��Ϊ0�����Բ�����
                        Run_Diagram_data.Freq[4]=Run_Diagram_data.Freq[8];
                        Run_Diagram_data.Freq[5]=Run_Diagram_data.Freq[9];
                        Run_Diagram_data.Freq[6]=Run_Diagram_data.Freq[10];
                    }
                    //������߳�������������Ҫ��������
                }
            }
        }


        if(Monitor.start[i][9]!=1)	//��ʮ��
        {
            if(Run_Diagram_buf[i].Start_Time10[0]!=0xFF&&((set_time.tm_hour*60+set_time.tm_min)>=(Run_Diagram_buf[i].Start_Time10[0]*60+Run_Diagram_buf[i].Start_Time10[1]))\
                    &&((set_time.tm_hour*60+set_time.tm_min)<(Run_Diagram_buf[i].End_Time10[0]*60+Run_Diagram_buf[i].End_Time10[1])))
            {
                App_printf("\r\nthe %d10 start\r\n", i);
                Monitor.start[i][9]=1;
                Monitor.end[i][9]=0;
                Monitor.usage_diagram_count++;
                if(Monitor.usage_diagram_count==1)
                {
                    Run_Diagram_data.mode='1';
                    set_run_diagram_new_power_level(Run_Diagram_data.mode, Run_Diagram_buf[i].Power10[0]);
                    memcpy(Run_Diagram_data.Freq,Run_Diagram_buf[i].Frq10,4);
                }
                else if(Monitor.usage_diagram_count==2)
                {
                    Run_Diagram_data.mode='2';
                    set_run_diagram_new_power_level(Run_Diagram_data.mode, Run_Diagram_buf[i].Power10[0]);
                    memcpy((uint8_t *)(Run_Diagram_data.Freq)+4,Run_Diagram_buf[i].Frq10,4);
                }
                else if(Monitor.usage_diagram_count==3)
                {
                    Run_Diagram_data.mode='3';
                    set_run_diagram_new_power_level(Run_Diagram_data.mode, Run_Diagram_buf[i].Power10[0]);
                    memcpy((uint8_t *)(Run_Diagram_data.Freq)+8,Run_Diagram_buf[i].Frq10,4);
                }
            }
        }

        if(Monitor.end[i][9]!=1)	//����
        {
            if( ( (set_time.tm_hour == Run_Diagram_buf[i].End_Time10[0]) && (set_time.tm_min == Run_Diagram_buf[i].End_Time10[1]) ) ||
                    ( (set_time.tm_hour == 0) && (set_time.tm_min == 0 ) && (Run_Diagram_buf[i].End_Time10[0] == 24) ) )
            {
                App_printf("\r\nthe %d10 end\r\n", i);
                Monitor.usage_diagram_count--;
                Monitor.end[i][9]=1;
                Monitor.start[i][9]=0;
                if(Run_Diagram_data.mode=='1')
                {
                    Run_Diagram_data.mode='0';
                    //��Ƶʱ��ʱ�䵽��ֹͣ����
                    memset(Run_Diagram_data.Freq,0,12);//����
                }
                if(Run_Diagram_data.mode=='2')
                {
                    Run_Diagram_data.mode='1';
                    if(memcmp(Run_Diagram_data.Freq,Run_Diagram_buf[i].Frq10,4)==0)   //�߳���һ��
                    {
                        Run_Diagram_data.Freq[0]=Run_Diagram_data.Freq[4];
                        Run_Diagram_data.Freq[1]=Run_Diagram_data.Freq[5];
                        Run_Diagram_data.Freq[2]=Run_Diagram_data.Freq[6];
                        //���1byte��Ϊ0�����Բ�����
                    }
                    //������߳��ڶ���������Ҫ��������
                }
                if(Run_Diagram_data.mode=='3')
                {
                    Run_Diagram_data.mode='2';
                    if(memcmp(Run_Diagram_data.Freq,Run_Diagram_buf[i].Frq10,4)==0)   //�߳���һ��
                    {
                        Run_Diagram_data.Freq[0]=Run_Diagram_data.Freq[4];
                        Run_Diagram_data.Freq[1]=Run_Diagram_data.Freq[5];
                        Run_Diagram_data.Freq[2]=Run_Diagram_data.Freq[6];
                        //���1byte��Ϊ0�����Բ�����
                        Run_Diagram_data.Freq[4]=Run_Diagram_data.Freq[8];
                        Run_Diagram_data.Freq[5]=Run_Diagram_data.Freq[9];
                        Run_Diagram_data.Freq[6]=Run_Diagram_data.Freq[10];
                    }
                    else if(memcmp((uint8_t *)(Run_Diagram_data.Freq)+4,Run_Diagram_buf[i].Frq10,4)==0)     //�߳��ڶ���
                    {
                        //���1byte��Ϊ0�����Բ�����
                        Run_Diagram_data.Freq[4]=Run_Diagram_data.Freq[8];
                        Run_Diagram_data.Freq[5]=Run_Diagram_data.Freq[9];
                        Run_Diagram_data.Freq[6]=Run_Diagram_data.Freq[10];
                    }
                    //������߳�������������Ҫ��������
                }
            }
        }
    }

    return 0;
}
