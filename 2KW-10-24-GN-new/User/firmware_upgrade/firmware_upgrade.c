#include "firmware_upgrade.h"
#include <string.h>

uint8_t	APP_Version_1[APPLICATION_VERSION_LEN];
uint8_t	APP_Version_2[APPLICATION_VERSION_LEN];

typedef  void (*pFunction)(void);		//函数指针

#if	BOOTLOADER

#else
//设置堆栈指针
__asm void MSR_MSP(u32 addr)
{
    MSR MSP, r0
    BX r14
}
#endif

//设置中断向量表偏移地址
void Set_Vector_Table(void)
{
	if( Get_APP_Version() )
	{
		NVIC_SetVectorTable(NVIC_VectTab_FLASH, APPLICATION_ADDRESS_OFFSET_2);	//设置中断向量表偏移地址	0x1A000
	}
	else
	{
		NVIC_SetVectorTable(NVIC_VectTab_FLASH, APPLICATION_ADDRESS_OFFSET_1);	//设置中断向量表偏移地址	0x2000
	}
}

//BootLoader 跳转
void BootLoad_Jump(void)
{
	u32 Application_Address;
    u32 JumpAddress;
    pFunction  Jump_To_Application;
    /* Check Vector Table: Test if user code is programmed starting from address 
    "APPLICATION_ADDRESS" */
	
	Application_Address = Get_Application_Address();
	
    //d_printfhex32((*(__IO uint32_t*)APPLICATION_ADDRESS));d_printf("\n");
    if (((*(__IO uint32_t*)Application_Address) & 0x2FFE0000 ) == 0x20000000)		//判断栈顶地址是否在0x2000 0000 ~ 0x2000 2000之间
    {
		__disable_irq();															//失能中断
	
		JumpAddress = *(__IO uint32_t*) (Application_Address +4);					//中断向量表的第二项 复位中断入口向量地址(第一项为堆栈指针)
		//d_printfhex32(JumpAddress);d_printf("\n");
	
		Jump_To_Application = (pFunction)JumpAddress;								//指向复位函数所在地址
		
		/* Initialize user application's Stack Pointer */
		MSR_MSP(*(__IO uint32_t*) Application_Address);								//设置主函数栈指针
		Jump_To_Application();														//执行复位函数
    }
}

//BootLoader 跳转恢复
void BootLoad_Jump_Recover(void)
{
	u32 Application_Address;
    u32 JumpAddress;
    pFunction  Jump_To_Application;
    /* Check Vector Table: Test if user code is programmed starting from address 
    "APPLICATION_ADDRESS" */
	
	Application_Address = Get_Old_Application_Address();
	
    //d_printfhex32((*(__IO uint32_t*)APPLICATION_ADDRESS));d_printf("\n");
    if (((*(__IO uint32_t*)Application_Address) & 0x2FFE0000 ) == 0x20000000)		//判断栈顶地址是否在0x2000 0000 ~ 0x2000 2000之间
    {
		__disable_irq();															//失能中断
	
		JumpAddress = *(__IO uint32_t*) (Application_Address +4);					//中断向量表的第二项 复位中断入口向量地址(第一项为堆栈指针)
		//d_printfhex32(JumpAddress);d_printf("\n");
	
		Jump_To_Application = (pFunction)JumpAddress;								//指向复位函数所在地址
		
		/* Initialize user application's Stack Pointer */
		MSR_MSP(*(__IO uint32_t*) Application_Address);								//设置主函数栈指针
		Jump_To_Application();														//执行复位函数
    }
}

//获取最新的APP版本号	0:APP1	1:APP2
uint8_t	Get_APP_Version(void)
{
	APP_Version_Read(APPLICATION_VERSION_ADDRESS, APP_Version_1, APP_Version_2);
	
	//如果都是有效数据，
	if( ( (APP_Version_1[0]<0x9A) && (APP_Version_1[1]<0x9A) && (APP_Version_1[2]<0x32) && (APP_Version_1[3]<0x32) ) &&		//有效数据
		( (APP_Version_2[0]<0x9A) && (APP_Version_2[1]<0x9A) && (APP_Version_2[2]<0x32) && (APP_Version_2[3]<0x32) ) )		//有效数据
	{
		if( (APP_Version_1[0]<APP_Version_2[0]) || (APP_Version_1[1]<APP_Version_2[1]) || 	//APP2版本号大于APP1
			(APP_Version_1[2]<APP_Version_2[2]) || (APP_Version_1[3]<APP_Version_2[3]) )
		{
			return 1;
		}
		else
		{
			return 0;
		}
	}
	else	if( ( (APP_Version_1[0]>=0x9A) && (APP_Version_1[1]>=0x9A) && (APP_Version_1[2]>=0x32) && (APP_Version_1[3]>=0x32) ) &&		//无效数据
				( (APP_Version_2[0]<0x9A) && (APP_Version_2[1]<0x9A) && (APP_Version_2[2]<0x32) && (APP_Version_2[3]<0x32) ) )			//有效数据
	{
		return 1;
	}
	else	if( ( (APP_Version_1[0]<0x9A) && (APP_Version_1[1]<0x9A) && (APP_Version_1[2]<0x32) && (APP_Version_1[3]<0x32) ) &&			//有效数据
				( (APP_Version_2[0]>=0x9A) && (APP_Version_2[1]>=0x9A) && (APP_Version_2[2]>=0x32) && (APP_Version_2[3]>=0x32) ) )		//无效数据
	{
		return 0;
	}
	
	return 0;
}

//获取最新版本APP地址
uint32_t Get_Application_Address(void)
{
	u32 Application_Address;
	
	if( Get_APP_Version() )
	{
		Application_Address = APPLICATION_ADDRESS_BASE + APPLICATION_ADDRESS_OFFSET_2;
	}
	else
	{
		Application_Address = APPLICATION_ADDRESS_BASE + APPLICATION_ADDRESS_OFFSET_1;
	}
	
	return Application_Address;
}

//获取旧版本APP地址
uint32_t Get_Old_Application_Address(void)
{
	u32 Application_Address;
	
	if( Get_APP_Version() )
	{
		Application_Address = APPLICATION_ADDRESS_BASE + APPLICATION_ADDRESS_OFFSET_1;
	}
	else
	{
		Application_Address = APPLICATION_ADDRESS_BASE + APPLICATION_ADDRESS_OFFSET_2;
	}
	
	return Application_Address;
}

//读固件版本号
void APP_Version_Read(uint32_t Address, uint8_t *pBuffer_1, uint8_t *pBuffer_2)
{
	FLASH_Read(Address,		pBuffer_1,	APPLICATION_VERSION_LEN);
	FLASH_Read(Address+8,	pBuffer_2,	APPLICATION_VERSION_LEN);
}

//写固件版本号
void APP_Version_Write(uint32_t Address, uint8_t *p_data_1, uint8_t *p_data_2)
{
	uint8_t p_data[2*APPLICATION_VERSION_LEN];
	
	memcpy(p_data,		p_data_1,	APPLICATION_VERSION_LEN);
	memcpy(p_data+4,	p_data_2,	APPLICATION_VERSION_LEN);
	Flash_Write(Address, p_data, 2*APPLICATION_VERSION_LEN);
}

//读flash
uint16_t FLASH_ReadHalfWord(uint32_t ReadAddr)
{
	return *(__IO uint32_t*)ReadAddr; 
}

//读flash
void FLASH_Read(uint32_t ReadAddr, uint8_t *pBuffer, uint16_t len) 	
{
	uint16_t i;
	
	for(i=0; i<len; i++)
	{
		pBuffer[i] = FLASH_ReadHalfWord(ReadAddr);	//读取2个字节.
		ReadAddr += 2;	//偏移2个字节.	
	}
}

//写flash(半字)		擦除整页再写，没做保存数据处理
uint8_t Flash_Write(uint32_t baseAddress, uint8_t *p_data, uint32_t len)
{
	uint8_t i = 0;
	
    uint32_t EraseCounter = 0x00;		//记录要擦除多少页
    uint32_t Address = 0x00;			//记录写入的地址
    uint32_t NbrOfPage = 0x00;			//记录写入多少页

    FLASH_Status FLASHStatus = FLASH_COMPLETE;		//记录每次擦除的结果
    uint8_t MemoryProgramStatus = 0x01;				//记录整个测试结果

	//flash解锁
    FLASH_Unlock();

    /* 计算要擦除多少页 */
    NbrOfPage = len/FLASH_PAGE_SIZE + 1;

    /* 清空所有标志位 */
	FLASH_ClearFlag(FLASH_FLAG_BSY | FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);

    /* 按页擦除*/
    for(EraseCounter = 0; (EraseCounter < NbrOfPage) && (FLASHStatus == FLASH_COMPLETE); EraseCounter++)
    {
        FLASHStatus = FLASH_ErasePage(baseAddress + (FLASH_PAGE_SIZE * EraseCounter));
    }

    /* 向内部FLASH写入数据 */
    Address = baseAddress;
    for(i=0; (i<len)&&(FLASHStatus == FLASH_COMPLETE); i++)
    {
        FLASH_ProgramHalfWord(Address, *(p_data+i) );
        Address += 2;
    }

	//flash上锁
    FLASH_Lock();

    /* 检查写入的数据是否正确 */
    Address = baseAddress;
    for(i=0; (i<len)&&(MemoryProgramStatus != 0x00); i++)
    {
        if( (*(__IO uint32_t*) Address) != (*(p_data+i)) )
        {
            MemoryProgramStatus = 0x00;
        }
		
        Address += 2;
    }
	
    return MemoryProgramStatus;
}

#if	BOOTLOADER

#else
//appxaddr:	应用程序的起始地址
//appbuf:	应用程序CODE
//appsize:	应用程序大小(字节)
void iap_write_appbin(u32 appxaddr,u8 *appbuf,u32 appsize)
{
	u16 t;
	u16 i=0;
	u16 temp;
	u16 iapbuf[1024];
	
	u32 fwaddr=appxaddr;		//当前写入的地址
	u8 *dfu=appbuf;
	
	for(t=0;t<appsize;t+=2)
	{						    
		temp=(u16)dfu[1]<<8;
		temp+=(u16)dfu[0];	  
		dfu+=2;					//偏移2个字节
		
		iapbuf[i++]=temp;	    
		if(i==1024)
		{
			i=0;
			STMFLASH_Write(fwaddr, iapbuf, 1024);	
			fwaddr+=2048;		//偏移2048  16=2*8.所以要乘以2.
		}
	}
	
	if( i )
		STMFLASH_Write(fwaddr, iapbuf, i);	//将最后的一些内容字节写进去.  
}
#endif
