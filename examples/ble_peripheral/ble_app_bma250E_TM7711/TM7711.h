#ifndef TM7711_H_
#define TM7711_H_

#define DOUT_7711   6//sbit DOUT_7711=P2^1;
#define PD_SCK_7711 8//sbit PD_SCK_7711=P2^0;

extern void GPIO_tm7711_init(void);
extern unsigned long Read_TM7711(unsigned char next_select);
#endif
