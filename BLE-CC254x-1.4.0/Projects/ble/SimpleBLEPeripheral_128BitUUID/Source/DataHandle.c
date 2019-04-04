#include "bcomdef.h"
#include "osal.h"
#include "DataHandle.h"
#include "SerialApp.h"

extern void SendNotify(uint8 *pBuffer,uint16 length);


void GhostyuProfileHandle(uint8 *data,uint8 len)
{
    UART_Print(data,len);
}



