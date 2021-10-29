#include "power.h"
#include "bsp_uart_fifo.h"
#include "alarm.h"
#include "stand_wave_rate.h"
#include "MT2000_protocol.h"
#include "includes.h"

#include "main.h"
//#include <math.h>

//更新45V下的电压电流显示，返回给查询应答包0602(旧版本，暂保留)
void update_System_Voltage_Electricity(void)
{
    System.Voltage = System.ADC_Sum_V * Debug_M_45V - 0.5;
    if( System.Voltage < 0.001 )
        System.Voltage = 0.0;

    if( System.ADC_Sum_A < 0.01 )
    {
        System.Electricity = 0.0;
    }
    else
    {
        System.Electricity = System.ADC_Sum_A * Debug_M_45I;
    }
}

