#ifndef	_HARD_CONTROL_H_
#define	_HARD_CONTROL_H_

#include <stdint.h>

extern void juge_need_open_close(uint8_t Save_count);
extern void juge_need_open(uint8_t Save_count);
extern void juge_need_close(uint8_t Save_count);

extern int find_hard_control(uint8_t Save_count);

#endif
