#ifndef __INT_CHAR_H
#define	__INT_CHAR_H

#include <stdio.h>  
#include <stdlib.h>  
#include <string.h>   
#include <math.h> 
#include <stm32f10x.h>

extern uint32_t IP_CCI(char *ch);
extern char *IP_ICC(int num);
extern char *AP_ICC(uint8_t *Array);
extern char *P_ICC(uint8_t *Array);
extern char *Array_ch(uint8_t *Array,uint8_t len);
#endif
