#include "bcomdef.h"
#include "OSAL.h"
#include "OSAL_PwrMgr.h"

#include "OnBoard.h"
#include "hal_adc.h"
#include "hal_led.h"
#include "hal_key.h"

#include "hal_lcd.h"

#include "hal_uart.h"


#include "gatt.h"

#include "gapgattserver.h"
#include "gattservapp.h"
#include "simpleGATTprofile.h"


#include "SerialApp.h"

void serialAppInitTransport( );
void sbpSerialAppCallback(uint8 port, uint8 event);

extern void SendNotify(uint8 *pBuffer,uint16 length);

/*�ú����������������ĳ�ʼ�������е���*/
void SerialApp_Init( uint8 taskID )
{
  //����uart��ʼ������
  serialAppInitTransport();

}

/*uart��ʼ�����룬���ô��ڵĲ����ʡ������Ƶ�*/
void serialAppInitTransport( )
{
  halUARTCfg_t uartConfig;

  // configure UART
  uartConfig.configured           = TRUE;
  uartConfig.baudRate             = SBP_UART_BR;//������
  uartConfig.flowControl          = SBP_UART_FC;//������
  uartConfig.flowControlThreshold = SBP_UART_FC_THRESHOLD;//��������ֵ��������flowControlʱ����������Ч
  uartConfig.rx.maxBufSize        = SBP_UART_RX_BUF_SIZE;//uart���ջ�������С
  uartConfig.tx.maxBufSize        = SBP_UART_TX_BUF_SIZE;//uart���ͻ�������С
  uartConfig.idleTimeout          = SBP_UART_IDLE_TIMEOUT;
  uartConfig.intEnable            = SBP_UART_INT_ENABLE;//�Ƿ����ж�
  uartConfig.callBackFunc         = sbpSerialAppCallback;//uart���ջص��������ڸú����ж�ȡ����uart����

  // start UART
  // Note: Assumes no issue opening UART port.
  (void)HalUARTOpen( SBP_UART_PORT, &uartConfig );

  return;
}
  uint16 numBytes;
/*uart���ջص�����*/
void sbpSerialAppCallback(uint8 port, uint8 event)
{
  uint8  pktBuffer[SBP_UART_RX_BUF_SIZE];
  // unused input parameter; PC-Lint error 715.
  (void)event;
  HalLcdWriteString("Data from my UART:", HAL_LCD_LINE_4 );
  HalLcdWriteString("Data from my UART:", HAL_LCD_LINE_4 );
  //���ؿɶ����ֽ�
  if ( (numBytes = Hal_UART_RxBufLen(port)) > 0 ){
  	//��ȡȫ����Ч�����ݣ��������һ��һ����ȡ���Խ����ض�������
	(void)HalUARTRead (port, pktBuffer, numBytes);
	uint8 data[20]={0};
	osal_memcpy(data,pktBuffer,numBytes);
	SendNotify(data,20);
	HalLcdWriteString((char*)pktBuffer, HAL_LCD_LINE_5 );
  }
  
}
void sbpSerialAppWrite(uint8 *pBuffer, uint16 length)
{
	HalUARTWrite (SBP_UART_PORT, pBuffer, length);
}

/*******************************************************************************
 ******************************************************************************/
//add by ghostyu
void UART_Print(uint8 *str,uint8 len)
{
    sbpSerialAppWrite(str, len);
}
/*
��ӡһ���ַ���
str�����԰���0x00�����ǽ�β
*/
void UART_PrintString(uint8 *str)
{
    sbpSerialAppWrite(str, osal_strlen((char*)str));
}
/*
��ӡָ���ĸ�ʽ����ֵ
����
title,ǰ׺�ַ���
value,��Ҫ��ʾ����ֵ
format,��Ҫ��ʾ�Ľ��ƣ�ʮ����Ϊ10,ʮ������Ϊ16
*/
void UART_PrintValue(char *title, uint16 value, uint8 format)
{
  uint8 tmpLen;
  uint8 buf[128];
  uint32 err;

  tmpLen = (uint8)osal_strlen( (char*)title );
  osal_memcpy( buf, title, tmpLen );
  buf[tmpLen] = ' ';
  err = (uint32)(value);
  _ltoa( err, &buf[tmpLen+1], format );
  UART_PrintString(buf);		
}
