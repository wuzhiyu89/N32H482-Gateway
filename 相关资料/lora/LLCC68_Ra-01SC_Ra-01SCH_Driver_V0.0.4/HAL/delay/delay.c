#include "delay.h"
#include "stm32f10x_it.h"

void delay_us(u32 nUs){
	uint32_t i=0,j=0;
	for(i=0;i<nUs;i++){
		for(j=0;j<9;j++){
			;
		}
	}
}

//简单的延时函数
void delay_ms(u16 nms)
{
	delay_us(nms*1000);
} 









































