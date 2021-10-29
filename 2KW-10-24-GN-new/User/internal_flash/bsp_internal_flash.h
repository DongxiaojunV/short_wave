#ifndef __INTERNAL_FLASH_H
#define	__INTERNAL_FLASH_H

#include "bsp.h"

/* STM32大容量产品每页大小2KByte，中、小容量产品每页大小1KByte */
#if defined (STM32F10X_HD) || defined (STM32F10X_HD_VL) || defined (STM32F10X_CL) || defined (STM32F10X_XL)
#define FLASH_PAGE_SIZE    ((uint16_t)0x800)	//2048
#else
#define FLASH_PAGE_SIZE    ((uint16_t)0x400)	//1024
#endif

//写入的起始地址与结束地址   起始126 k-130k共计4k 126-127-128 2k+ 128-129-130 2k共计 4k
//0x0801F800前面有126k了
//0x080207FF增长4096 4k

///*大容量 512k 内部falsh，选取200页处开始存储*/
////第一页2k内存
//#define WRITE_START_ADDR_first  ((uint32_t)0x08064000)
//#define WRITE_END_ADDR_first    ((uint32_t)0x080647FF)

////第二页2k内存
//#define WRITE_START_ADDR_second ((uint32_t)0x08064800)
//#define WRITE_END_ADDR_second   ((uint32_t)0x08064FFF)

////第三页2k内存
//#define WRITE_START_ADDR_third  ((uint32_t)0x08065000)
//#define WRITE_END_ADDR_third    ((uint32_t)0x080657FF)


//#define WRITE_START_ADDR_1 ((uint32_t)0x08070800)
//#define WRITE_END_ADDR_1   ((uint32_t)0x08070FFF)

//#define WRITE_START_ADDR_2 ((uint32_t)0x08071000)
//#define WRITE_END_ADDR_2   ((uint32_t)0x080717FF)

//#define WRITE_START_ADDR_3 ((uint32_t)0x08071800)
//#define WRITE_END_ADDR_3   ((uint32_t)0x08071FFF)

//#define WRITE_START_ADDR_4 ((uint32_t)0x08072000)
//#define WRITE_END_ADDR_4   ((uint32_t)0x080727FF)

//#define WRITE_START_ADDR_5 ((uint32_t)0x08072800)
//#define WRITE_END_ADDR_5   ((uint32_t)0x08072FFF)

//#define WRITE_START_ADDR_6 ((uint32_t)0x08073000)
//#define WRITE_END_ADDR_6   ((uint32_t)0x080737FF)

//#define WRITE_START_ADDR_7 ((uint32_t)0x08073800)
//#define WRITE_END_ADDR_7   ((uint32_t)0x08073FFF)

//#define WRITE_START_ADDR_8 ((uint32_t)0x08074000)
//#define WRITE_END_ADDR_8   ((uint32_t)0x080747FF)

//#define WRITE_START_ADDR_9 ((uint32_t)0x08074800)
//#define WRITE_END_ADDR_9   ((uint32_t)0x08074FFF)

//#define WRITE_START_ADDR_10 ((uint32_t)0x08075000)
//#define WRITE_END_ADDR_10   ((uint32_t)0x080757FF)

/*大容量 256k 内部falsh，选取100页处开始存储*/
//第一页2k内存
#define WRITE_START_ADDR_first  ((uint32_t)0x08032000)
#define WRITE_END_ADDR_first    ((uint32_t)0x080327FF)

//第二页2k内存
#define WRITE_START_ADDR_second ((uint32_t)0x08032800)
#define WRITE_END_ADDR_second   ((uint32_t)0x08032FFF)

//第三页2k内存
#define WRITE_START_ADDR_third  ((uint32_t)0x08033000)
#define WRITE_END_ADDR_third    ((uint32_t)0x080337FF)


#define WRITE_START_ADDR_1 ((uint32_t)0x08033800)
#define WRITE_END_ADDR_1   ((uint32_t)0x08033FFF)

#define WRITE_START_ADDR_2 ((uint32_t)0x08034000)
#define WRITE_END_ADDR_2   ((uint32_t)0x080347FF)

#define WRITE_START_ADDR_3 ((uint32_t)0x08034800)
#define WRITE_END_ADDR_3   ((uint32_t)0x08034FFF)

#define WRITE_START_ADDR_4 ((uint32_t)0x08035000)
#define WRITE_END_ADDR_4   ((uint32_t)0x080357FF)

#define WRITE_START_ADDR_5 ((uint32_t)0x08035800)
#define WRITE_END_ADDR_5   ((uint32_t)0x08035FFF)

#define WRITE_START_ADDR_6 ((uint32_t)0x08036000)
#define WRITE_END_ADDR_6   ((uint32_t)0x080367FF)

#define WRITE_START_ADDR_7 ((uint32_t)0x08036800)
#define WRITE_END_ADDR_7   ((uint32_t)0x08036FFF)

#define WRITE_START_ADDR_8 ((uint32_t)0x08037000)
#define WRITE_END_ADDR_8   ((uint32_t)0x080377FF)

#define WRITE_START_ADDR_9 ((uint32_t)0x08037800)
#define WRITE_END_ADDR_9   ((uint32_t)0x08037FFF)

#define WRITE_START_ADDR_10 ((uint32_t)0x08038000)
#define WRITE_END_ADDR_10   ((uint32_t)0x080387FF)
/*0x08038800地址区域存放版本号,0x08039000存放扫频断电数据*/
#define Ip_Port_Addr ((uint32_t)0x08039800)

typedef enum
{
    FAILED = 0,
    PASSED = !FAILED
} TestStatus;

typedef union float_int
{
    float f_data;
    uint32_t i_data;
} F_I_t; //float和int互转

extern volatile uint8_t		flash3_Save;
extern volatile uint32_t	flash_1_once_flag;
extern volatile uint32_t	flash_3_once_flag;


int System_InternalFlash_Init(uint32_t init_flag);	//初始化报警参数(可以设置为已初始化或未初始化)
int InternalFlash_SaveData_1(uint8_t init);			//写入报警参数
int InternalFlash_SaveData_2(void);					//写入正向功率和驻波比(扫频)
int InternalFlash_SaveData_3(uint8_t num);			//运行图


_Bool Flash_to_AcceptAPP(void);					//读取报警参数
_Bool Flash2_to_AcceptAPP(void);				//读取正向功率和驻波比(扫频)
_Bool Flash3_to_AcceptAPP(void);				//运行图


int Flash_Save_Run_Diagram(uint32_t start,uint32_t end,uint8_t run_diagram_count);
int Flash_Read_Run_Diagram(uint32_t start,uint32_t end,uint8_t judge_read_diagram);

void judg_read_flash(uint32_t save_cout);
#endif /* __INTERNAL_FLASH_H */

