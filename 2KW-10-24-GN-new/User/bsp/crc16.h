
#ifndef __CRC16_H__
#define __CRC16_H__

#include "bsp.h"
#ifdef __cplusplus
#if __cplusplus
extern "C" {
#endif
#endif /* __cplusplus */

    /* Header File */


    /* Macro Define */


    /* Global variable and struct/enum/union define */


    /* Function List*/
    uint32_t CRC32(uint8_t * Data ,uint32_t Len);
    uint16_t CRC16_XMODEM(uint8_t * Data ,uint32_t Len);
    uint16_t CRC_CheckSum(uint8_t * Data ,uint32_t Len);


#ifdef __cplusplus
#if __cplusplus
}
#endif
#endif /* __cplusplus */

#endif /* __CRC16_H__ */
