#ifndef	_ALARM_H_
#define	_ALARM_H_

/*-------------------------------�������----------------------------------*/
extern void VA_alarm_check(void);			/* ��ѹ����������� */
extern void swr_power_alarm_check(void);	/* פ���Ⱥ��޹��ʱ������ */

extern void Read_Sum_AD(void);
extern void alarm_printf(void);
extern int  get_history_alarm(void);
extern void clean_all_alarm_flags(void);
extern void set_alarm_bit(void);

extern void emission_alarm_content_clean(void);
#endif
