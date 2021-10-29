#include "int_char_change.h"

/*�������ã�IP�ŵ�char-int*/
uint32_t IP_CCI(char *ch)  
{  
    const char *ip ="192.168.34.232";   
    char str_ip_index[4]={'\0'};  
    uint32_t ip_int,ip_add=0;  
    uint8_t j =0,a=3;     
    for(uint16_t i =0;i<=strlen(ip);i++)//Ҫ�õ�'\0'  
    {  
        if (ip[i]=='\0'||ip[i]=='.')  
        {             
            ip_int =atoi(str_ip_index);   
            if (ip_int==0||ip_int>255)  
            {  
                return 0;                 
            }  
  
            ip_add+=(ip_int*((unsigned int)pow(256.0,a)));            
            a--;  
            memset(str_ip_index,0,sizeof(str_ip_index));  
  
            j=0;  
            continue;  
        }  
  
        str_ip_index[j]=ip[i];  
        j++;  
    }     
    
			return ip_add; 
}  

/*IP�ŵ�����ת�ַ���*/
char *IP_ICC(int num)
{
	  char *ipstr=NULL; 
		uint32_t ip_int_index[4],ip_temp_numbr=24;
    for(uint16_t j=0;j<4;j++)  
    {  
        ip_int_index[j]=(num>>ip_temp_numbr)&0xFF;  
        ip_temp_numbr-=8;  
    }  
  
    if ((ipstr=(char *)malloc(17*sizeof(char)))==NULL)  
    {  
        return 0;  
    }  
      
    sprintf(ipstr,"%d.%d.%d.%d",ip_int_index[0],ip_int_index[1],ip_int_index[2],ip_int_index[3]);  
  
    return ipstr;  
}	

/*IP��u8����ת��Ϊ�ַ���,���鳤��ֻ��4*/
char *AP_ICC(uint8_t *Array)
{
		char *ipstr=NULL; 
 
    if ((ipstr=(char *)malloc(17*sizeof(char)))==NULL)  
    {  
        return 0;  
    }  
      
    sprintf(ipstr,"%d.%d.%d.%d",Array[0],Array[1],Array[2],Array[3]);  
  
    return ipstr;  

}

/*�˿�����ת�ַ���*/
char *P_ICC(uint8_t *Array)
{
		char *ipstr=NULL; 

		uint16_t num;
	  num=Array[0]<<8|Array[1];
    if ((ipstr=(char *)malloc(17*sizeof(char)))==NULL)  
    {  
        return 0;  
    }  
      
    sprintf(ipstr,"%d",num);  
  
    return ipstr;  
}
/*����ת��Ϊ�ַ���*/
char *Array_ch(uint8_t *Array,uint8_t len)
{
		char *ipstr=NULL; 
		char *str=NULL;
    if ((ipstr=(char *)malloc(30*sizeof(char)))==NULL||(str=(char *)malloc(17*sizeof(char)))==NULL)  
    {  
        return 0;  
    }  
		
		for(uint8_t i=0;i<len;i++)
		{
			sprintf(str,"%d",Array[i]);
			strcat(ipstr,str);
		}
    return ipstr;  
}

