#include "firmware_upgrade.h"
#include <string.h>

uint8_t	APP_Version_1[APPLICATION_VERSION_LEN];
uint8_t	APP_Version_2[APPLICATION_VERSION_LEN];

typedef  void (*pFunction)(void);		//����ָ��

#if	BOOTLOADER

#else
//���ö�ջָ��
__asm void MSR_MSP(u32 addr)
{
    MSR MSP, r0
    BX r14
}
#endif

//�����ж�������ƫ�Ƶ�ַ
void Set_Vector_Table(void)
{
	if( Get_APP_Version() )
	{
		NVIC_SetVectorTable(NVIC_VectTab_FLASH, APPLICATION_ADDRESS_OFFSET_2);	//�����ж�������ƫ�Ƶ�ַ	0x1A000
	}
	else
	{
		NVIC_SetVectorTable(NVIC_VectTab_FLASH, APPLICATION_ADDRESS_OFFSET_1);	//�����ж�������ƫ�Ƶ�ַ	0x2000
	}
}

//BootLoader ��ת
void BootLoad_Jump(void)
{
	u32 Application_Address;
    u32 JumpAddress;
    pFunction  Jump_To_Application;
    /* Check Vector Table: Test if user code is programmed starting from address 
    "APPLICATION_ADDRESS" */
	
	Application_Address = Get_Application_Address();
	
    //d_printfhex32((*(__IO uint32_t*)APPLICATION_ADDRESS));d_printf("\n");
    if (((*(__IO uint32_t*)Application_Address) & 0x2FFE0000 ) == 0x20000000)		//�ж�ջ����ַ�Ƿ���0x2000 0000 ~ 0x2000 2000֮��
    {
		__disable_irq();															//ʧ���ж�
	
		JumpAddress = *(__IO uint32_t*) (Application_Address +4);					//�ж�������ĵڶ��� ��λ�ж����������ַ(��һ��Ϊ��ջָ��)
		//d_printfhex32(JumpAddress);d_printf("\n");
	
		Jump_To_Application = (pFunction)JumpAddress;								//ָ��λ�������ڵ�ַ
		
		/* Initialize user application's Stack Pointer */
		MSR_MSP(*(__IO uint32_t*) Application_Address);								//����������ջָ��
		Jump_To_Application();														//ִ�и�λ����
    }
}

//BootLoader ��ת�ָ�
void BootLoad_Jump_Recover(void)
{
	u32 Application_Address;
    u32 JumpAddress;
    pFunction  Jump_To_Application;
    /* Check Vector Table: Test if user code is programmed starting from address 
    "APPLICATION_ADDRESS" */
	
	Application_Address = Get_Old_Application_Address();
	
    //d_printfhex32((*(__IO uint32_t*)APPLICATION_ADDRESS));d_printf("\n");
    if (((*(__IO uint32_t*)Application_Address) & 0x2FFE0000 ) == 0x20000000)		//�ж�ջ����ַ�Ƿ���0x2000 0000 ~ 0x2000 2000֮��
    {
		__disable_irq();															//ʧ���ж�
	
		JumpAddress = *(__IO uint32_t*) (Application_Address +4);					//�ж�������ĵڶ��� ��λ�ж����������ַ(��һ��Ϊ��ջָ��)
		//d_printfhex32(JumpAddress);d_printf("\n");
	
		Jump_To_Application = (pFunction)JumpAddress;								//ָ��λ�������ڵ�ַ
		
		/* Initialize user application's Stack Pointer */
		MSR_MSP(*(__IO uint32_t*) Application_Address);								//����������ջָ��
		Jump_To_Application();														//ִ�и�λ����
    }
}

//��ȡ���µ�APP�汾��	0:APP1	1:APP2
uint8_t	Get_APP_Version(void)
{
	APP_Version_Read(APPLICATION_VERSION_ADDRESS, APP_Version_1, APP_Version_2);
	
	//���������Ч���ݣ�
	if( ( (APP_Version_1[0]<0x9A) && (APP_Version_1[1]<0x9A) && (APP_Version_1[2]<0x32) && (APP_Version_1[3]<0x32) ) &&		//��Ч����
		( (APP_Version_2[0]<0x9A) && (APP_Version_2[1]<0x9A) && (APP_Version_2[2]<0x32) && (APP_Version_2[3]<0x32) ) )		//��Ч����
	{
		if( (APP_Version_1[0]<APP_Version_2[0]) || (APP_Version_1[1]<APP_Version_2[1]) || 	//APP2�汾�Ŵ���APP1
			(APP_Version_1[2]<APP_Version_2[2]) || (APP_Version_1[3]<APP_Version_2[3]) )
		{
			return 1;
		}
		else
		{
			return 0;
		}
	}
	else	if( ( (APP_Version_1[0]>=0x9A) && (APP_Version_1[1]>=0x9A) && (APP_Version_1[2]>=0x32) && (APP_Version_1[3]>=0x32) ) &&		//��Ч����
				( (APP_Version_2[0]<0x9A) && (APP_Version_2[1]<0x9A) && (APP_Version_2[2]<0x32) && (APP_Version_2[3]<0x32) ) )			//��Ч����
	{
		return 1;
	}
	else	if( ( (APP_Version_1[0]<0x9A) && (APP_Version_1[1]<0x9A) && (APP_Version_1[2]<0x32) && (APP_Version_1[3]<0x32) ) &&			//��Ч����
				( (APP_Version_2[0]>=0x9A) && (APP_Version_2[1]>=0x9A) && (APP_Version_2[2]>=0x32) && (APP_Version_2[3]>=0x32) ) )		//��Ч����
	{
		return 0;
	}
	
	return 0;
}

//��ȡ���°汾APP��ַ
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

//��ȡ�ɰ汾APP��ַ
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

//���̼��汾��
void APP_Version_Read(uint32_t Address, uint8_t *pBuffer_1, uint8_t *pBuffer_2)
{
	FLASH_Read(Address,		pBuffer_1,	APPLICATION_VERSION_LEN);
	FLASH_Read(Address+8,	pBuffer_2,	APPLICATION_VERSION_LEN);
}

//д�̼��汾��
void APP_Version_Write(uint32_t Address, uint8_t *p_data_1, uint8_t *p_data_2)
{
	uint8_t p_data[2*APPLICATION_VERSION_LEN];
	
	memcpy(p_data,		p_data_1,	APPLICATION_VERSION_LEN);
	memcpy(p_data+4,	p_data_2,	APPLICATION_VERSION_LEN);
	Flash_Write(Address, p_data, 2*APPLICATION_VERSION_LEN);
}

//��flash
uint16_t FLASH_ReadHalfWord(uint32_t ReadAddr)
{
	return *(__IO uint32_t*)ReadAddr; 
}

//��flash
void FLASH_Read(uint32_t ReadAddr, uint8_t *pBuffer, uint16_t len) 	
{
	uint16_t i;
	
	for(i=0; i<len; i++)
	{
		pBuffer[i] = FLASH_ReadHalfWord(ReadAddr);	//��ȡ2���ֽ�.
		ReadAddr += 2;	//ƫ��2���ֽ�.	
	}
}

//дflash(����)		������ҳ��д��û���������ݴ���
uint8_t Flash_Write(uint32_t baseAddress, uint8_t *p_data, uint32_t len)
{
	uint8_t i = 0;
	
    uint32_t EraseCounter = 0x00;		//��¼Ҫ��������ҳ
    uint32_t Address = 0x00;			//��¼д��ĵ�ַ
    uint32_t NbrOfPage = 0x00;			//��¼д�����ҳ

    FLASH_Status FLASHStatus = FLASH_COMPLETE;		//��¼ÿ�β����Ľ��
    uint8_t MemoryProgramStatus = 0x01;				//��¼�������Խ��

	//flash����
    FLASH_Unlock();

    /* ����Ҫ��������ҳ */
    NbrOfPage = len/FLASH_PAGE_SIZE + 1;

    /* ������б�־λ */
	FLASH_ClearFlag(FLASH_FLAG_BSY | FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);

    /* ��ҳ����*/
    for(EraseCounter = 0; (EraseCounter < NbrOfPage) && (FLASHStatus == FLASH_COMPLETE); EraseCounter++)
    {
        FLASHStatus = FLASH_ErasePage(baseAddress + (FLASH_PAGE_SIZE * EraseCounter));
    }

    /* ���ڲ�FLASHд������ */
    Address = baseAddress;
    for(i=0; (i<len)&&(FLASHStatus == FLASH_COMPLETE); i++)
    {
        FLASH_ProgramHalfWord(Address, *(p_data+i) );
        Address += 2;
    }

	//flash����
    FLASH_Lock();

    /* ���д��������Ƿ���ȷ */
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
//appxaddr:	Ӧ�ó������ʼ��ַ
//appbuf:	Ӧ�ó���CODE
//appsize:	Ӧ�ó����С(�ֽ�)
void iap_write_appbin(u32 appxaddr,u8 *appbuf,u32 appsize)
{
	u16 t;
	u16 i=0;
	u16 temp;
	u16 iapbuf[1024];
	
	u32 fwaddr=appxaddr;		//��ǰд��ĵ�ַ
	u8 *dfu=appbuf;
	
	for(t=0;t<appsize;t+=2)
	{						    
		temp=(u16)dfu[1]<<8;
		temp+=(u16)dfu[0];	  
		dfu+=2;					//ƫ��2���ֽ�
		
		iapbuf[i++]=temp;	    
		if(i==1024)
		{
			i=0;
			STMFLASH_Write(fwaddr, iapbuf, 1024);	
			fwaddr+=2048;		//ƫ��2048  16=2*8.����Ҫ����2.
		}
	}
	
	if( i )
		STMFLASH_Write(fwaddr, iapbuf, i);	//������һЩ�����ֽ�д��ȥ.  
}
#endif
