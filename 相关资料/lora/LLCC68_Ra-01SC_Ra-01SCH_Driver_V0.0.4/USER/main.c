#include "stm32f10x.h"
#include "delay.h"
#include "HAL_uart.h"
#include "stdio.h"
#include "stm32f10x_it.h"
#include "project_config.h"
#include "LLCC68_example_send.h"
#include "LLCC68_example_recive.h"

/*
wiring setting
spi bus:
	LoRa modules		  STM32
	NSS_PIN				    PA4
	MOSI_PIN      		PA7
	MISO_PIN     		  PA6
	SCK_PIN       		PA5
	RESET_PIN    		  PA12
	DIO1_PIN      		PB11
	DIO4_BUSY_PIN     PA0

UART:
	USB to TTL			STM32
	Tx					    PA_9
	Rx					    PA_10
*/
static void LORA_LED_init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Pin = GREEN_LED_PIN;
	GPIO_Init(GREEN_LED_PORT, &GPIO_InitStructure);

	GPIO_WriteBit(GREEN_LED_PORT, GREEN_LED_PIN, Bit_RESET);
}

//硬件初始化
void SysInit(void){
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);	//NVIC(中断优先级管理)分组配置,注意:这个分组整个程序只能有一次,配置后不要修改,否则会出现很多问题 这里直接用4,0~15优先级
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);	    //JTAG复用为GPIO需要使用 RCC_APB2Periph_AFIO 时钟
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable,ENABLE);	//关闭JTAG功能(需要打开 RCC_APB2Periph_AFIO 时钟)

	//led指示灯
	LORA_LED_init();
	
	HALUart1Init();
	SysTick_Config(SystemCoreClock/1000);
}

int main(void){
	SysInit();	//硬件初始化
	
	printf("SysInit OK，version:%s\r\n",SOFT_VERSION);

	//测试demo，一个程序只能打开一条测试demo，进入测试demo后将进入死循环，不会返回了
	
	#if LORA_DATA_SEND_OR_RECEIV_MODE
	ExampleLLCC68SendDemo();	//循环接收demo
	#else
	ExampleLLCC68ReciveDemo();	//定时发送demo
	#endif
	
	//开启测试demo后代码就执行不到这里了
	while(1){
		printf("systick=%d\r\n",Get_SysTick());
		GPIO_ResetBits(GREEN_LED_PORT,GREEN_LED_PIN);
		delay_ms(500);
		GPIO_SetBits(GREEN_LED_PORT,GREEN_LED_PIN);
		delay_ms(500);
	}
}
