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

/*该函数将会在任务函数的初始化函数中调用*/
void SerialApp_Init( uint8 taskID )
{
  //调用uart初始化代码
  serialAppInitTransport();

}

/*uart初始化代码，配置串口的波特率、流控制等*/
void serialAppInitTransport( )
{
  halUARTCfg_t uartConfig;

  // configure UART
  uartConfig.configured           = TRUE;
  uartConfig.baudRate             = SBP_UART_BR;//波特率
  uartConfig.flowControl          = SBP_UART_FC;//流控制
  uartConfig.flowControlThreshold = SBP_UART_FC_THRESHOLD;//流控制阈值，当开启flowControl时，该设置有效
  uartConfig.rx.maxBufSize        = SBP_UART_RX_BUF_SIZE;//uart接收缓冲区大小
  uartConfig.tx.maxBufSize        = SBP_UART_TX_BUF_SIZE;//uart发送缓冲区大小
  uartConfig.idleTimeout          = SBP_UART_IDLE_TIMEOUT;
  uartConfig.intEnable            = SBP_UART_INT_ENABLE;//是否开启中断
  uartConfig.callBackFunc         = sbpSerialAppCallback;//uart接收回调函数，在该函数中读取可用uart数据

  // start UART
  // Note: Assumes no issue opening UART port.
  (void)HalUARTOpen( SBP_UART_PORT, &uartConfig );

  return;
}
  uint16 numBytes;
/*uart接收回调函数*/
void sbpSerialAppCallback(uint8 port, uint8 event)
{
  uint8  pktBuffer[SBP_UART_RX_BUF_SIZE];
  // unused input parameter; PC-Lint error 715.
  (void)event;
  HalLcdWriteString("Data from my UART:", HAL_LCD_LINE_4 );
  HalLcdWriteString("Data from my UART:", HAL_LCD_LINE_4 );
  //返回可读的字节
  if ( (numBytes = Hal_UART_RxBufLen(port)) > 0 ){
  	//读取全部有效的数据，这里可以一个一个读取，以解析特定的命令
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
打印一个字符串
str不可以包含0x00，除非结尾
*/
void UART_PrintString(uint8 *str)
{
    sbpSerialAppWrite(str, osal_strlen((char*)str));
}
/*
打印指定的格式的数值
参数
title,前缀字符串
value,需要显示的数值
format,需要显示的进制，十进制为10,十六进制为16
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
