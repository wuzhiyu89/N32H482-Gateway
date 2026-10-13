#include "LLCC68_example_recive.h"
#include "radio.h"
#include "project_config.h"
#include "stdio.h"
#include "n32h47x_48x_it.h"
#include "delay.h"
#include "string.h"

static RadioEvents_t LLCC68RadioEvents;

static void LLCC68OnTxDone( void );
static void LLCC68OnRxDone( uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr );
static void LLCC68OnTxTimeout( void );
static void LLCC68OnRxTimeout( void );
static void LLCC68OnRxError( void );

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
	Radio.SetTxConfig( MODEM_LORA, LORA_TX_OUTPUT_POWER, 0, LORA_BANDWIDTH,
                     LORA_SPREADING_FACTOR, LORA_CODINGRATE,
                     LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
                     true, 0, 0, LORA_IQ_INVERSION_ON, 3000 );//������loraģʽ,���书��,fsk�õ�lora����Ϊ0�Ϳ��ԣ����������������ʣ�ǰ���볤�ȣ��̶��������ݰ�(һ���ǲ��̶�������ѡfalse)��crcУ�飬0��ʾ�ر���Ƶ����Ƶ֮��ķ�����(�ر���Ƶ�������û������)�����Ӧ���Ǳ�ʾ�Ƿ�Ҫ��ת�жϵ�ƽ�ģ���ʱʱ��

	OCP_Value = Radio.Read(REG_OCP);
	printf("[%s()-%d]read OCP register value:0x%04X\r\n",__func__,__LINE__,OCP_Value);
	
	Radio.SetRxConfig( MODEM_LORA, LORA_BANDWIDTH, LORA_SPREADING_FACTOR,
                     LORA_CODINGRATE, 0, LORA_PREAMBLE_LENGTH,
                     LORA_LLCC68_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON,
                     0, true, 0, 0, LORA_IQ_INVERSION_ON, false );

	printf("start enter rx mode\r\n");
	Radio.Rx( 0 );	//�������ģʽ
	printf("all setting\r\n");
	printf("freq: %d\r\n Tx power: %d\r\n band width: %d\r\n FS: %d\r\n CODINGRATE: %d\r\n PREAMBLE_LENGTH: %d\r\n",LORA_FRE,LORA_TX_OUTPUT_POWER,LORA_BANDWIDTH,LORA_SPREADING_FACTOR,LORA_CODINGRATE,LORA_PREAMBLE_LENGTH);
	while(1){
		Radio.IrqProcess( ); // Process Radio IRQ
		systick_delay_ms(1);
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
	printf("RxDone size=%d rssi=%d snr=%d\r\n", size, rssi, snr);
	Radio.Rx( 0 );
	
	if(size == 0)
	{
		printf("size=0, ignore\r\n");
	}
	else if(size == 4)
	{
		uint32_t n;
		memcpy(&n, payload, 4);
		printf("recive u32 data=%u\r\n", n);
	}
	else
	{
		char tmp[64] = {0};
		u16 cp = size < sizeof(tmp) - 1 ? size : sizeof(tmp) - 1;
		memcpy(tmp, payload, cp);
		printf("recive text: %s\r\n", tmp);
	}
	
}

static void LLCC68OnTxTimeout( void )
{
	printf("TxTimeout\r\n");
}

static void LLCC68OnRxTimeout( void )
{
	Radio.Standby();
	printf("RxTimeout retry recive\r\n");
	Radio.Rx( LORA_RX_TIMEOUT_VALUE ); 
}

static void LLCC68OnRxError( void )
{
	Radio.Standby();
	printf("RxError retry recive\r\n");
	Radio.Rx(LORA_RX_TIMEOUT_VALUE); 
}
