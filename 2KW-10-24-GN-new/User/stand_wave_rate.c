#include "stand_wave_rate.h"
#include "bsp_uart_fifo.h"

#include "main.h"
#include <string.h>
#include <math.h>


/*��ȡפ���Ⱥ���������פ����
 *Forward_Power:������(��ʵ�ǵ�ѹֵ)   Reverse_Power��������
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

///*
// *��ȡ���פ����,��λ�������פ����Ƶ�Σ����������Ƶ�θ���
// */
//int get_better_SWR(float *array,int index)
//{
//    int min_count=0;
//    int SIZE = ceil((freq_end-freq_begin))+1;

//    App_printf("SIZE:%d\r\n",SIZE);
//    App_printf("swr:");
//    for(int i=0; i<SIZE; i++)
//    {
//        App_printf("%f ",array[i*10+index]);
//    }
//    App_printf("\r\n\r\n");

//    App_printf("Forward_Power:");
//    for(int i=0; i<SIZE; i++)
//    {
//        App_printf("%f ",Forward_Power_array[i*10+index]);
//        if(Forward_Power_array[i*10+index]<1100)
//        {
//            Forward_Power_array[i*10+index]=0;
//        }
//        else if(Forward_Power_array[i*10+index]>=1550)
//        {
//            Forward_Power_array[i*10+index]=1550;
//        }

//    }
//    App_printf("\r\n\r\n");

//    return min_count;
//}
