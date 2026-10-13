/*!
 * \file      sx1262mbxcas-board.c
 *
 * \brief     Target board SX1262MBXCAS shield driver implementation
 *
 * \copyright Revised BSD License, see section \ref LICENSE.
 *
 * \code
 *                ______                              _
 *               / _____)             _              | |
 *              ( (____  _____ ____ _| |_ _____  ____| |__
 *               \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 *               _____) ) ____| | | || |_| ____( (___| | | |
 *              (______/|_____)_|_|_| \__)_____)\____)_| |_|
 *              (C)2013-2017 Semtech
 *
 * \endcode
 *
 * \author    Miguel Luis ( Semtech )
 *
 * \author    Gregory Cristian ( Semtech )
 */
#include <stdlib.h>
#include "project_config.h"
#include "delay.h"
#include "radio.h"
#include "LLCC68-board.h"
#include "HAL_SPI.h"
#include "stdio.h"

typedef struct __softTimer_s{
	uint8_t isRun;		//0Í£Ö¹£¬1ÔËĞĞ
	uint32_t delayMs;	//¶¨Ê±Æ÷ĞèÒª¶à³¤Ê±¼äºó»½ĞÑ
	uint32_t startMs;	//¶¨Ê±Æ÷¿ªÊ¼Ê±¼ä
}SoftTimer_t;

static DioIrqHandler *dio1IrqCallback=NULL;	//¼ÇÂ¼DIO1µÄ»Øµ÷º¯Êı¾ä±ú
static uint32_t timer_count=0;
static SoftTimer_t txTimerHandle,rxTimerHandle;

/* ³õÊ¼»¯LLCC68ĞèÒªÓÃµ½µÄGPIO³õÊ¼»¯£¬½« BUSY Òı½ÅÉèÖÃÎªÊäÈëÄ£Ê½
 */
void LLCC68IoInit( void )
{
	uint8_t u8_ret=255;
	GPIO_InitTypeDef  GPIO_InitStructure;

	//³õÊ¼»¯BUSYÎªÉÏÀ­ÊäÈëÄ£Ê½
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB, ENABLE);	    //Ê¹ÄÜÖ¸¶¨¶Ë¿ÚÊ±ÖÓ
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	 //IO¿ÚËÙ¶ÈÎª50MHz£¬ÕâÀï²»ÓÃ´«²Î£¬Ö±½ÓĞ´ËÀÓÃ×î´óËÙ¶È50MHZ
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IPU;//ÉÏÀ­ÊäÈë
	GPIO_InitStructure.GPIO_Pin = RADIO_DIO4_BUSY_PIN;
	GPIO_Init(RADIO_DIO4_BUSY_PORT, &GPIO_InitStructure);	//³õÊ¼»¯GPIO
	
	//ÏÂÃæÕâ¼¸¸öÒı½ÅÃ»ÓÃÓÃµ½£¬ÉèÖÃÎª¸¡¿ÕÊäÈëÄ£Ê½
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IN_FLOATING;//¸¡¿ÕÊäÈë
	
	GPIO_InitStructure.GPIO_Pin = RADIO_DIO0_TXEN_PIN;
	GPIO_Init(RADIO_DIO0_TXEN_PORT, &GPIO_InitStructure);	//³õÊ¼»¯GPIO

	GPIO_InitStructure.GPIO_Pin = RADIO_DIO2_PIN;
	GPIO_Init(RADIO_DIO2_PORT, &GPIO_InitStructure);	//³õÊ¼»¯GPIO
	
	GPIO_InitStructure.GPIO_Pin = RADIO_DIO3_PIN;
	GPIO_Init(RADIO_DIO3_PORT, &GPIO_InitStructure);	//³õÊ¼»¯GPIO
	
	GPIO_InitStructure.GPIO_Pin = RADIO_DIO5_RXEN_PIN;
	GPIO_Init(RADIO_DIO5_RXEN_PORT, &GPIO_InitStructure);	//³õÊ¼»¯GPIO
	
	u8_ret=Spi1Init();
	if(u8_ret){
		printf("spi1 init error\r\n");
	}else{
		printf("LLCC68IoInit init sucess\r\n");
	}
}

/* ³õÊ¼»¯ DIO1
 * ½«DIO1ÉèÖÃÎªÍâ²¿ÖĞ¶Ï(ÉÏÉıÑØ´¥·¢)£¬²¢ÇÒ»Øµ÷º¯ÊıÎª dioIrq º¯ÊıÔ­ĞÍ void RadioOnDioIrq( void* context )
 */
void LLCC68IoIrqInit( DioIrqHandler dioIrq )
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	EXTI_InitTypeDef EXTI_InitStructure;
 	NVIC_InitTypeDef NVIC_InitStructure;
	
	dio1IrqCallback=dioIrq;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IPD;
	GPIO_InitStructure.GPIO_Pin = RADIO_DIO1_PIN;
	GPIO_Init(RADIO_DIO1_PORT, &GPIO_InitStructure);
	
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);

  GPIO_EXTILineConfig(GPIO_PortSourceGPIOB,GPIO_PinSource1);
	EXTI_InitStructure.EXTI_Line=EXTI_Line1;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;	
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);

	//Priority config
	NVIC_InitStructure.NVIC_IRQChannel = EXTI1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x02;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x03;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	EXTI_ClearITPendingBit(EXTI_Line11);
}

//void LLCC68IoIrqInit( DioIrqHandler dioIrq )
void EXTI1_IRQHandler(void){
	if(GPIO_ReadInputDataBit(RADIO_DIO1_PORT,RADIO_DIO1_PIN)){
		printf("DIO1 irq\r\n");
		if(NULL!=dio1IrqCallback){
			dio1IrqCallback(NULL);
		}
		EXTI_ClearITPendingBit(EXTI_Line1);
	}
}

void LLCC68IoDeInit( void )
{
    //GPIOÈ¥³õÊ¼»¯´úÂë
}

//Reset Pin initï¼ˆPA12ï¼‰
void LLCC68ResetInit( void )
{
	GPIO_InitTypeDef  GPIO_InitStructure;

	LLCC68DelayMs( 10 );
	
	//Â½Â«RSTÊ¤Â³Ã¶ÂµÍµç†½
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);	    //Ê¹ÄœÖ¸Â¶Â¨Â¶Ë¿ÚŠÂ±Ö“
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	 //IOÂ¿Ú‹Ù¶ÈÂª50MHzÂ£Â¬Õ¢Ã€ï²»ÓƒÂ´Â«Â²Î£Â¬Ö±Â½ÓÂ´Ë€Óƒ×®Â´ó‹™¶Èµ0MHZ
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_PP;	//Í†Í¬Ê¤Â³Ã¶
	GPIO_InitStructure.GPIO_Pin = RADIO_nRESET_PIN;
	GPIO_Init(RADIO_nRESET_PORT, &GPIO_InitStructure);	//Â³ÃµÊ¼Â»Â¯GPIO
	GPIO_WriteBit( RADIO_nRESET_PORT, RADIO_nRESET_PIN,Bit_RESET);	//RSTÉ¨ÎªÂµÍµç†½Ê¤Â³Ã¶
	
	LLCC68DelayMs( 20 );
	
	//Â½Â«RSTÉ¨ÖƒÎªÉÃ€Â­Ê¤È«Ä£Ê½
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IPU;
	GPIO_Init(RADIO_nRESET_PORT, &GPIO_InitStructure);	//Â³ÃµÊ¼Â»Â¯GPIO
	
	LLCC68DelayMs( 10 );
}

//¶ÁÈ¡BusyÒı½ÅµçÆ½×´Ì¬
void LLCC68WaitOnBusy( void )
{
	uint32_t u32_count=0;
	
	while( GPIO_ReadInputDataBit(RADIO_DIO4_BUSY_PORT,RADIO_DIO4_BUSY_PIN) == 1 ){
		if(u32_count++>1000){
			printf("wait busy pin timeout\r\n");
			u32_count=0;
		}
		LLCC68DelayMs(1);
	}
}

//ÉèÖÃSPIµÄNSSÒı½ÅµçÆ½£¬0µÍµçÆ½£¬·ÇÁã¸ßµçÆ½
void LLCC68SetNss(uint8_t lev ){
	if(lev){
		GPIO_SetBits(RADIO_NSS_PORT,RADIO_NSS_PIN);	//Êä³ö¸ßµçÆ½
	}else{
		GPIO_ResetBits(RADIO_NSS_PORT,RADIO_NSS_PIN);	//Êä³öµÍµçÆ½
	}
}

//spi´«ÊäÒ»¸ö×Ö½ÚµÄÊı¾İ
uint8_t LLCC68SpiInOut(uint8_t data){
	return HALSpi1InOut(data);
}

//¼ì²éÆµÂÊÊÇ·ñ·ûºÏÒªÇó£¬Èç¹û²»ĞèÒªÅĞ¶ÏÔò¿ÉÒÔÖ±½Ó·µ»Øtrue
bool LLCC68CheckRfFrequency( uint32_t frequency )
{
    // Implement check. Currently all frequencies are supported
    return true;
}

//ºÁÃëÑÓÊ±
void LLCC68DelayMs(uint32_t ms){
	delay_ms(ms);
}

//tx/rx¶¨Ê±Æ÷²Ù×÷

//¶¨Ê±Æ÷3ÖĞ¶Ï·şÎñ³ÌĞò
//tx¶¨Ê±½áÊøĞèÒª»Øµ÷ RadioOnTxTimeoutIrq
//rx¶¨Ê±½áÊøĞèÒª»Øµ÷ RadioOnRxTimeoutIrq
void TIM3_IRQHandler(void)   //TIM3ÖĞ¶Ï
{
	uint32_t diffMs=0;
	
	if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET){
		timer_count++;
		TIM_ClearITPendingBit(TIM3, TIM_IT_Update  );  //Çå³ıTIMx¸üĞÂÖĞ¶Ï±êÖ¾
		
		//printf("timer_count=%d\r\n",timer_count);
		
		//´¦Àítx¶¨Ê±Æ÷
		if(txTimerHandle.isRun){
			if(timer_count>=txTimerHandle.startMs){
				diffMs=timer_count-txTimerHandle.startMs;
			}else{//Òç³öÁË
				diffMs=0xffffffff - txTimerHandle.startMs + timer_count;
			}
			if(diffMs>txTimerHandle.delayMs){
				//¼ÆÊ±½áÊø
				//printf("----------------- tx timer irq ---------------\r\n");
				LLCC68TxTimerStop();
				txTimerHandle.startMs=timer_count;
				RadioOnTxTimeoutIrq(NULL);	//tx¶¨Ê±½áÊøĞèÒª»Øµ÷ RadioOnTxTimeoutIrq
			}
		}
		
		//´¦Àírx¶¨Ê±Æ÷
		if(rxTimerHandle.isRun){
			if(timer_count>=rxTimerHandle.startMs){
				diffMs=timer_count-rxTimerHandle.startMs;
			}else{//Òç³öÁË
				diffMs=0xffffffff - rxTimerHandle.startMs + timer_count;
			}
			if(diffMs>rxTimerHandle.delayMs){
				//¼ÆÊ±½áÊø
				//printf("----------------- rx timer irq ---------------\r\n");
				LLCC68RxTimerStop();
				rxTimerHandle.startMs=timer_count;
				RadioOnRxTimeoutIrq(NULL);	//rx¶¨Ê±½áÊøĞèÒª»Øµ÷ RadioOnRxTimeoutIrq
			}
		}
	}
}

//³õÊ¼»¯¶¨Ê±Æ÷(ÕâÀïÓÃTIM3¶¨Ê±Æ÷Ä£Äâ³öÁËÁ½¸öms¶¨Ê±Æ÷)
//tx¶¨Ê±½áÊøĞèÒª»Øµ÷ RadioOnTxTimeoutIrq
//rx¶¨Ê±½áÊøĞèÒª»Øµ÷ RadioOnRxTimeoutIrq
void LLCC68TimerInit(void){
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	//Ê¹ÄÜTIM3
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE); //Ê±ÖÓÊ¹ÄÜ
 
	//TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE ); //Ê¹ÄÜÖ¸¶¨µÄTIM3ÖĞ¶Ï,ÔÊĞí¸üĞÂÖĞ¶Ï

	//ÖĞ¶ÏÓÅÏÈ¼¶NVICÉèÖÃ
	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;  //TIM3ÖĞ¶Ï
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;  //ÏÈÕ¼ÓÅÏÈ¼¶0¼¶
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;  //´ÓÓÅÏÈ¼¶3¼¶
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQÍ¨µÀ±»Ê¹ÄÜ
	NVIC_Init(&NVIC_InitStructure);  //³õÊ¼»¯NVIC¼Ä´æÆ÷
	
	//¶¨Ê±Æ÷TIM3³õÊ¼»¯
	TIM_TimeBaseStructure.TIM_Period = 10; //ÉèÖÃÔÚÏÂÒ»¸ö¸üĞÂÊÂ¼ş×°Èë»î¶¯µÄ×Ô¶¯ÖØ×°ÔØ¼Ä´æÆ÷ÖÜÆÚµÄÖµ	
	TIM_TimeBaseStructure.TIM_Prescaler =SystemCoreClock/10000; //ÉèÖÃÓÃÀ´×÷ÎªTIMxÊ±ÖÓÆµÂÊ³ıÊıµÄÔ¤·ÖÆµÖµ(1s10000´Î£¬Ò²¾ÍÊÇ1´Î0.1ms)
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //ÉèÖÃÊ±ÖÓ·Ö¸î:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIMÏòÉÏ¼ÆÊıÄ£Ê½
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure); //¸ù¾İÖ¸¶¨µÄ²ÎÊı³õÊ¼»¯TIMxµÄÊ±¼ä»ùÊıµ¥Î»
	
	//¹Ø±ÕÈí¼ş¶¨Ê±Æ÷
	txTimerHandle.isRun=0;
	rxTimerHandle.isRun=0;
	
	TIM_Cmd(TIM3, ENABLE);  //Ê¹ÄÜTIMx
	TIM_ClearITPendingBit(TIM3, TIM_IT_Update  );  //Çå³ıTIMx¸üĞÂÖĞ¶Ï±êÖ¾
	TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE ); //Ê¹ÄÜÖ¸¶¨µÄTIM2ÖĞ¶Ï,ÔÊĞí¸üĞÂÖĞ¶Ï
}

void LLCC68SetTxTimerValue(uint32_t nMs){
	printf("[%s()-%d]set timer out %d ms\r\n",__func__,__LINE__,nMs);
	txTimerHandle.delayMs=nMs;
}

void LLCC68TxTimerStart(void){
	printf("[%s()-%d]start timer\r\n",__func__,__LINE__);
	txTimerHandle.startMs=timer_count;
	txTimerHandle.isRun=1;
}

void LLCC68TxTimerStop(void){
	printf("[%s()-%d]stop timer\r\n",__func__,__LINE__);
	txTimerHandle.isRun=0;
}

void LLCC68SetRxTimerValue(uint32_t nMs){
	printf("[%s()-%d]set timer out %d ms\r\n",__func__,__LINE__,nMs);
	rxTimerHandle.delayMs=nMs;
}

void LLCC68RxTimerStart(void){
	printf("[%s()-%d]start timer\r\n",__func__,__LINE__);
	rxTimerHandle.startMs=timer_count;
	rxTimerHandle.isRun=1;
}

void LLCC68RxTimerStop(void){
	printf("[%s()-%d]stop timer\r\n",__func__,__LINE__);
	rxTimerHandle.isRun=0;
}
