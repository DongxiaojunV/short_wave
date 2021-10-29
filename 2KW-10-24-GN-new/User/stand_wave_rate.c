#include "stand_wave_rate.h"
#include "bsp_uart_fifo.h"

#include "main.h"
#include <string.h>
#include <math.h>


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

///*
// *获取最佳驻波比,置位不合理的驻波比频段，并返回最佳频段个数
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
