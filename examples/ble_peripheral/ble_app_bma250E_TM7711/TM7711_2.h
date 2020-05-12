#ifndef TM7711_2_H_
#define TM7711_2_H_

#define DOUT_7711_2   31//sbit DOUT_7711=P2^1;
#define PD_SCK_7711_2 29//sbit PD_SCK_7711=P2^0;

extern void GPIO_tm7711_2_init(void);
extern unsigned long Read_TM7711_2(unsigned char next_select);
#endif
