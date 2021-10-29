#ifndef	__FIRMWARE_UPGRADE_H_

#define		APP_VERSION		(0x20211024)

#define		BOOTLOADER		0		//是否是BootLoader

#if	BOOTLOADER
	#include "stm32f10x.h"
	#include "stm32f10x_flash.h"

	/*-------------------------------------------------------------------------------*/
	/* STM32大容量产品每页大小2KByte，中、小容量产品每页大小1KByte */
	#if defined (STM32F10X_HD) || defined (STM32F10X_HD_VL) || defined (STM32F10X_CL) || defined (STM32F10X_XL)
	#define FLASH_PAGE_SIZE    ((uint16_t)0x800)	//2048
	#else
	#define FLASH_PAGE_SIZE    ((uint16_t)0x400)	//1024
	#endif
#else
	#include "main.h"
	#include "bsp.h"
	#include "stmflash.h"
#endif

/*-------------------------------------------------------------------------------*/
//固件更新
#define	APPLICATION_ADDRESS_BASE			0x08000000		//基地址

#define APPLICATION_ADDRESS_OFFSET_1		0x00002000		//起始地址			8K
#define APPLICATION_ADDRESS_OFFSET_2		0x0001A000		//起始地址			104K

#define	APPLICATION_SIZE					0x00018000		//APP大小			96K
#define	APPLICATION_PACK_SIZE				0x00000800		//APP包大小			2K

#define	APPLICATION_VERSION_ADDRESS			0x08038800		//APP版本地址		226K	前4个半字是APP1版本，后4个半字是APP2版本(保存的是日期0x20181211)，接运行图后面

#define	APPLICATION_VERSION_LEN				4				//APP版本长度		4Byte

/*-------------------------------------------------------------------------------*/
extern	uint8_t	APP_Version_1[4];
extern	uint8_t	APP_Version_2[4];

/*-------------------------------------------------------------------------------*/
extern	__asm void MSR_MSP(uint32_t addr);					//设置堆栈指针
extern	void Set_Vector_Table(void);						//设置中断向量表偏移地址
extern	void BootLoad_Jump(void);							//BootLoad跳转
extern	void BootLoad_Jump_Recover(void);					//BootLoad跳转恢复

extern	uint8_t	Get_APP_Version(void);						//获取最新的APP版本号	0:APP1	1:APP2
extern	uint32_t Get_Application_Address(void);				//获取最新版本APP地址
extern	uint32_t Get_Old_Application_Address(void);			//获取旧版本APP地址

/*-------------------------------------------------------------------------------*/
extern	void APP_Version_Read(uint32_t Address, uint8_t *pBuffer_1, uint8_t *pBuffer_2);	//读固件版本号
extern	void APP_Version_Write(uint32_t Address, uint8_t *p_data_1, uint8_t *p_data_2);		//写固件版本号

extern	uint16_t FLASH_ReadHalfWord(uint32_t flash_addr);							//读flash	半字
extern	void FLASH_Read(uint32_t ReadAddr, uint8_t *pBuffer, uint16_t len);			//读flash	字符串
extern	uint8_t Flash_Write(uint32_t baseAddress, uint8_t *p_data, uint32_t len);	//写flash	字符串	擦除整页再写，没做保存数据处理

#endif
