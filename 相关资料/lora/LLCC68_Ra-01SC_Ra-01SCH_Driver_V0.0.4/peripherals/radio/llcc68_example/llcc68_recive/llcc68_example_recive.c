#include "LLCC68_example_recive.h"
#include "radio.h"
#include "project_config.h"
#include "stdio.h"
#include "stm32f10x_it.h"
#include "delay.h"
#include "string.h"

/*!
 * Radio events function pointer
 * 这个是传参进入其他函数中了，所以用全局变量(局部变量使用完了内存释放可能导致异常)
 */
static RadioEvents_t LLCC68RadioEvents;

static void LLCC68OnTxDone( void );
static void LLCC68OnRxDone( uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr );
static void LLCC68OnTxTimeout( void );
static void LLCC68OnRxTimeout( void );
static void LLCC68OnRxError( void );

static void LLCC68ReciveReconfig(void){
	LLCC68RadioEvents.TxDone = LLCC68OnTxDone;
	LLCC68RadioEvents.RxDone = LLCC68OnRxDone;
	LLCC68RadioEvents.TxTimeout = LLCC68OnTxTimeout;
	LLCC68RadioEvents.RxTimeout = LLCC68OnRxTimeout;
	LLCC68RadioEvents.RxError = LLCC68OnRxError;
	Radio.Init( &LLCC68RadioEvents );
	Radio.SetChannel(LORA_FRE);
	Radio.SetRxConfig( MODEM_LORA, LORA_BANDWIDTH, LORA_SPREADING_FACTOR,
                     LORA_CODINGRATE, 0, LORA_PREAMBLE_LENGTH,
                     LORA_LLCC68_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON,
                     0, true, 0, 0, LORA_IQ_INVERSION_ON, false );
	printf("LLCC68ReciveReconfig\r\n");
}

//开启一个定时发送任务，每隔1S发送一条数据
void ExampleLLCC68ReciveDemo(void){
	uint8_t OCP_Value = 0;
	printf("start %s() example\r\n",__func__);
	
	LLCC68RadioEvents.TxDone = LLCC68OnTxDone;
	LLCC68RadioEvents.RxDone = LLCC68OnRxDone;
	LLCC68RadioEvents.TxTimeout = LLCC68OnTxTimeout;
	LLCC68RadioEvents.RxTimeout = LLCC68OnRxTimeout;
	LLCC68RadioEvents.RxError = LLCC68OnRxError;

	Radio.Init( &LLCC68RadioEvents );
	Radio.SetChannel(LORA_FRE);

	OCP_Value = Radio.Read(REG_OCP);
	printf("[%s()-%d]read OCP register value:0x%04X\r\n",__func__,__LINE__,OCP_Value);
	
	Radio.SetRxConfig( MODEM_LORA, LORA_BANDWIDTH, LORA_SPREADING_FACTOR,
                     LORA_CODINGRATE, 0, LORA_PREAMBLE_LENGTH,
                     LORA_LLCC68_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON,
                     0, true, 0, 0, LORA_IQ_INVERSION_ON, false );

	printf("start enter rx mode\r\n");
	printf("all setting\r\n");
	printf("freq: %d\r\n Tx power: %d\r\n band width: %d\r\n FS: %d\r\n CODINGRATE: %d\r\n PREAMBLE_LENGTH: %d\r\n",LORA_FRE,LORA_TX_OUTPUT_POWER,LORA_BANDWIDTH,LORA_SPREADING_FACTOR,LORA_CODINGRATE,LORA_PREAMBLE_LENGTH);

	Radio.Rx( LORA_RX_TIMEOUT_VALUE );	//进入接收模式
	while(1){
		Radio.IrqProcess( ); // Process Radio IRQ
		delay_ms(1);
	}
}

static void LLCC68OnTxDone( void )
{
	printf("TxDone\r\n");
	Radio.Standby();
}

static void LLCC68OnRxDone( uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr )
{
	Radio.Standby();
	Radio.Rx( LORA_RX_TIMEOUT_VALUE );
	printf("RxDone\r\nsize:%d\r\nrssi:%d\r\nsnr:%d\r\n",size,rssi,snr);
	printf("recive data: %s\r\n",(char*)payload);
	GPIO_SetBits(GREEN_LED_PORT,GREEN_LED_PIN);
	delay_ms(100);
	GPIO_ResetBits(GREEN_LED_PORT,GREEN_LED_PIN);
}

static void LLCC68OnTxTimeout( void )
{
	printf("TxTimeout\r\n");
}

static void LLCC68OnRxTimeout( void )
{
	static uint32_t RxTimeoutCount = 0;
	Radio.Standby();
	printf("RxTimeout retry recive\r\n");
	RxTimeoutCount ++;
	if(RxTimeoutCount > 10){
		LLCC68ReciveReconfig();
	}	
	Radio.Rx(LORA_RX_TIMEOUT_VALUE);
}

static void LLCC68OnRxError( void )
{
	Radio.Standby();
	printf("RxError retry recive\r\n");
	Radio.Rx(LORA_RX_TIMEOUT_VALUE);  
}
