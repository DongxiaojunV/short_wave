#ifndef	__FIRMWARE_UPGRADE_H_

#define		APP_VERSION		(0x20211024)

#define		BOOTLOADER		0		//�Ƿ���BootLoader

#if	BOOTLOADER
	#include "stm32f10x.h"
	#include "stm32f10x_flash.h"

	/*-------------------------------------------------------------------------------*/
	/* STM32��������Ʒÿҳ��С2KByte���С�С������Ʒÿҳ��С1KByte */
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
//�̼�����
#define	APPLICATION_ADDRESS_BASE			0x08000000		//����ַ

#define APPLICATION_ADDRESS_OFFSET_1		0x00002000		//��ʼ��ַ			8K
#define APPLICATION_ADDRESS_OFFSET_2		0x0001A000		//��ʼ��ַ			104K

#define	APPLICATION_SIZE					0x00018000		//APP��С			96K
#define	APPLICATION_PACK_SIZE				0x00000800		//APP����С			2K

#define	APPLICATION_VERSION_ADDRESS			0x08038800		//APP�汾��ַ		226K	ǰ4��������APP1�汾����4��������APP2�汾(�����������0x20181211)��������ͼ����

#define	APPLICATION_VERSION_LEN				4				//APP�汾����		4Byte

/*-------------------------------------------------------------------------------*/
extern	uint8_t	APP_Version_1[4];
extern	uint8_t	APP_Version_2[4];

/*-------------------------------------------------------------------------------*/
extern	__asm void MSR_MSP(uint32_t addr);					//���ö�ջָ��
extern	void Set_Vector_Table(void);						//�����ж�������ƫ�Ƶ�ַ
extern	void BootLoad_Jump(void);							//BootLoad��ת
extern	void BootLoad_Jump_Recover(void);					//BootLoad��ת�ָ�

extern	uint8_t	Get_APP_Version(void);						//��ȡ���µ�APP�汾��	0:APP1	1:APP2
extern	uint32_t Get_Application_Address(void);				//��ȡ���°汾APP��ַ
extern	uint32_t Get_Old_Application_Address(void);			//��ȡ�ɰ汾APP��ַ

/*-------------------------------------------------------------------------------*/
extern	void APP_Version_Read(uint32_t Address, uint8_t *pBuffer_1, uint8_t *pBuffer_2);	//���̼��汾��
extern	void APP_Version_Write(uint32_t Address, uint8_t *p_data_1, uint8_t *p_data_2);		//д�̼��汾��

extern	uint16_t FLASH_ReadHalfWord(uint32_t flash_addr);							//��flash	����
extern	void FLASH_Read(uint32_t ReadAddr, uint8_t *pBuffer, uint16_t len);			//��flash	�ַ���
extern	uint8_t Flash_Write(uint32_t baseAddress, uint8_t *p_data, uint32_t len);	//дflash	�ַ���	������ҳ��д��û���������ݴ���

#endif
