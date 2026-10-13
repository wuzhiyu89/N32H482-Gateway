#include "LLCC68_example_send.h"
#include "radio.h"
#include "project_config.h"
#include "stdio.h"

#include "delay.h"
#include "string.h"

/*!
 * Radio events function pointer
 * ����Ǵ��ν��������������ˣ�������ȫ�ֱ���(�ֲ�����ʹ�������ڴ��ͷſ��ܵ����쳣)
 */
static RadioEvents_t LLCC68RadioEvents;

// static const uint8_t sendData[] = {
//     0xAA, 0x55, 0x01, 0x00, 0x00, 0x00, 0x21, 0x00,
//     0x01, 0x03, 0x0F, 0x01, 0x01, 0x05, 0x01, 0x08,
//     0x03, 0x00, 0x07, 0xCB, 0xAE, 0xC9, 0xEE, 0x32,
//     0x30, 0x6D, 0x2C, 0xFA
// };

static const uint8_t sendData[] = {
    0xAA, 0x55, 0x01, 0x00, 0x00, 0x00, 0x21, 0x00,
    0x01, 0x03, 0x0F, 0x01, 0x01, 0x05, 0x01, 0x08,
 
	
};


#define SEND_DATA_LEN   (sizeof(sendData))

//const char sendData[20] = "aithinker";

static void LLCC68OnTxDone( void );
static void LLCC68OnRxDone( uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr );
static void LLCC68OnTxTimeout( void );
static void LLCC68OnRxTimeout( void );
static void LLCC68OnRxError( void );

//����һ����ʱ��������ÿ��1S����һ������
void ExampleLLCC68SendDemo(void)
{
	uint8_t OCP_Value = 0;
	
	printf("start %s() example\r\n",__func__);
	
	LLCC68RadioEvents.TxDone 	= LLCC68OnTxDone;
	LLCC68RadioEvents.RxDone 	= LLCC68OnRxDone;
	LLCC68RadioEvents.TxTimeout = LLCC68OnTxTimeout;
	LLCC68RadioEvents.RxTimeout = LLCC68OnRxTimeout;
	LLCC68RadioEvents.RxError 	= LLCC68OnRxError;

	Radio.Init(&LLCC68RadioEvents);
	Radio.SetChannel(LORA_FRE);
	Radio.SetTxConfig(MODEM_LORA, LORA_TX_OUTPUT_POWER, 0, LORA_BANDWIDTH,
                      LORA_SPREADING_FACTOR, LORA_CODINGRATE,
                      LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
                      true, 0, 0, LORA_IQ_INVERSION_ON, 3000 );//������loraģʽ,���书��,fsk�õ�lora����Ϊ0�Ϳ��ԣ����������������ʣ�ǰ���볤�ȣ��̶��������ݰ�(һ���ǲ��̶�������ѡfalse)��crcУ�飬0��ʾ�ر���Ƶ����Ƶ֮��ķ�����(�ر���Ƶ�������û������)�����Ӧ���Ǳ�ʾ�Ƿ�Ҫ��ת�жϵ�ƽ�ģ���ʱʱ��

	OCP_Value = Radio.Read(REG_OCP);
	
	printf("[%s()-%d]read OCP register value:0x%04X\r\n",__func__,__LINE__,OCP_Value);
	
	Radio.SetRxConfig(MODEM_LORA, LORA_BANDWIDTH, LORA_SPREADING_FACTOR,
                      LORA_CODINGRATE, 0, LORA_PREAMBLE_LENGTH,
                      LORA_LLCC68_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON,
                      0, true, 0, 0, LORA_IQ_INVERSION_ON, false );
	
	printf("all setting\r\n");
	printf("freq: %d\r\n Tx power: %d\r\n band width: %d\r\n FS: %d\r\n CODINGRATE: %d\r\n PREAMBLE_LENGTH: %d\r\n",LORA_FRE,LORA_TX_OUTPUT_POWER,LORA_BANDWIDTH,LORA_SPREADING_FACTOR,LORA_CODINGRATE,LORA_PREAMBLE_LENGTH);
	while(1)
	{
		Radio.IrqProcess( ); // Process Radio IRQ
		
		/* ÿ 5 �뷢һ�Σ��� tick �������� delay_ms æ�ȣ� */
		static uint32_t tick = 0;
		if (++tick >= 5000) {
			tick = 0;
			Radio.Send((uint8_t *)sendData, SEND_DATA_LEN);
			printf("send %d bytes\r\n", (int)SEND_DATA_LEN);
		}

		systick_delay_ms(1);       // ֻ��ʱ 1ms���� IrqProcess Ƶ����
	}
}

static void LLCC68OnTxDone( void )
{
	printf("TxDone\r\n");
	Radio.Standby();
	

	/* �� ���������������ģʽ���Ƚ����� ACK */
    Radio.Rx(LORA_RX_TIMEOUT_VALUE);   // ���� Radio.Rx(0) ���ý���
}

static void LLCC68OnRxDone( uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr )
{
	Radio.Standby();
	printf("RxDone size=%d rssi=%d snr=%d\r\n", size, rssi, snr);

	/* �� ��ӡ���� payload���� ACK �Բ��� */
    printf("payload: ");
    for (u16 i = 0; i < size; i++) printf("%02X ", payload[i]);
    printf("\r\n");

    /* �ж��Ƿ��� ACK��size=12��payload[0]=0xAA, [1]=0x55, [9]=0x80 */
    if (size >= 12 && payload[0] == 0xAA && payload[1] == 0x55) {
        printf("ACK received!\r\n");
    } else {
        printf("not an ACK, size=%d\r\n", size);
    }

    /* �� ����������գ���� Standby ����һ�� Send�� */
    // Radio.Rx(0);   // ��һֱ�վʹ�
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
