#include "bcomdef.h"
#include "OSAL.h"
#include "OSAL_PwrMgr.h"
#include "OnBoard.h"
#include "hal_adc.h"
//#include "hal_led.h"
#include "hal_key.h"
//#include "hal_lcd.h"
#include "gatt.h"
#include "hci.h"
#include "gapgattserver.h"
#include "gattservapp.h"
#include "devinfoservice.h"
#include "simpleGATTprofile.h"

#include "peripheral.h"
#include "gapbondmgr.h"
#include "simpleBLEPeripheral.h"

#define motoA   P1_0
#define motoB   P1_1
#define limte_key P1_3
uint8 lock_flag = 0;
static uint8 macaddr[6]={0}; //   mac 地址 
uint8 sofe_ver = 0;
uint8 hard_ver = 0;
uint8 power_value = 0;

//uart print
#include "SerialApp.h"
#include "DataHandle.h"

// How often to perform periodic event
#define SBP_PERIODIC_EVT_PERIOD                   500   //700

// What is the advertising interval when device is discoverable (units of 625us, 160=100ms)
#define DEFAULT_ADVERTISING_INTERVAL          160

// Limited discoverable mode advertises for 30.72s, and then stops
// General discoverable mode advertises indefinitely

#if defined ( CC2540_MINIDK )
#define DEFAULT_DISCOVERABLE_MODE             GAP_ADTYPE_FLAGS_LIMITED
#else
#define DEFAULT_DISCOVERABLE_MODE             GAP_ADTYPE_FLAGS_GENERAL
#endif  // defined ( CC2540_MINIDK )

// Minimum connection interval (units of 1.25ms, 80=100ms) if automatic parameter update request is enabled
#define DEFAULT_DESIRED_MIN_CONN_INTERVAL     80

// Maximum connection interval (units of 1.25ms, 800=1000ms) if automatic parameter update request is enabled
#define DEFAULT_DESIRED_MAX_CONN_INTERVAL     800

// Slave latency to use if automatic parameter update request is enabled
#define DEFAULT_DESIRED_SLAVE_LATENCY         0

// Supervision timeout value (units of 10ms, 1000=10s) if automatic parameter update request is enabled
#define DEFAULT_DESIRED_CONN_TIMEOUT          1000

// Whether to enable automatic parameter update request when a connection is formed
#define DEFAULT_ENABLE_UPDATE_REQUEST         TRUE

// Connection Pause Peripheral time value (in seconds)
#define DEFAULT_CONN_PAUSE_PERIPHERAL         6

// Company Identifier: Texas Instruments Inc. (13)
#define TI_COMPANY_ID                         0x000D

#define INVALID_CONNHANDLE                    0xFFFF

// Length of bd addr as a string
#define B_ADDR_STR_LEN                        15

static uint8 simpleBLEPeripheral_TaskID;   // Task ID for internal task/event processing

static gaprole_States_t gapProfileState = GAPROLE_INIT;


unsigned char adcres[1] = {0};


// GAP - SCAN RSP data (max size = 31 bytes)
static uint8 scanRspData[] =
{
  // complete name
  0x07,   // length of this data
  GAP_ADTYPE_LOCAL_NAME_COMPLETE,
 
  0x45,   // 'E'
  0x5F,   // '_'
  0x4C,   // 'L'
  0x6F,   // 'o'
  0x63,   // 'c'
  0x4B,   // 'K'
 
  // connection interval range
  0x05,   // length of this data
  GAP_ADTYPE_SLAVE_CONN_INTERVAL_RANGE,
  LO_UINT16( DEFAULT_DESIRED_MIN_CONN_INTERVAL ),   // 100ms
  HI_UINT16( DEFAULT_DESIRED_MIN_CONN_INTERVAL ),
  LO_UINT16( DEFAULT_DESIRED_MAX_CONN_INTERVAL ),   // 1s
  HI_UINT16( DEFAULT_DESIRED_MAX_CONN_INTERVAL ),

  // Tx power level
  0x02,   // length of this data
  GAP_ADTYPE_POWER_LEVEL,
  0       // 0dBm
};

// GAP - Advertisement data (max size = 31 bytes, though this is
// best kept short to conserve power while advertisting)
static uint8 advertData[] =
{
  // Flags; this sets the device to use limited discoverable
  // mode (advertises for 30 seconds at a time) instead of general
  // discoverable mode (advertises indefinitely)
  0x02,   // length of this data
  GAP_ADTYPE_FLAGS,
  DEFAULT_DISCOVERABLE_MODE | GAP_ADTYPE_FLAGS_BREDR_NOT_SUPPORTED,

  // service UUID, to notify central devices what services are included
  // in this peripheral
  0x03,   // length of this data
  GAP_ADTYPE_16BIT_MORE,      // some of the UUID's, but not all
  LO_UINT16( GHOSTYU_DEVICE_SERV_UUID ),
  HI_UINT16( GHOSTYU_DEVICE_SERV_UUID ),

  0x08,
  GAP_ADTYPE_MANUFACTURER_SPECIFIC,
  0x9C,0xD2,0x1E,0xF3,0xE9,0xC9

};

// GAP GATT Attributes
static uint8 attDeviceName[GAP_DEVICE_NAME_LEN] = "E_Lock";


/*********************************************************************
 * LOCAL FUNCTIONS
 */
static void simpleBLEPeripheral_ProcessOSALMsg( osal_event_hdr_t *pMsg );
static void peripheralStateNotificationCB( gaprole_States_t newState );
static void performPeriodicTask( void );
static void simpleProfileChangeCB( uint8 paramID );

#if defined( CC2540_MINIDK )
static void simpleBLEPeripheral_HandleKeys( uint8 shift, uint8 keys );
#endif

void SendNotify(uint8 *pBuffer,uint16 length);

// GAP Role Callbacks
static gapRolesCBs_t simpleBLEPeripheral_PeripheralCBs =
{
  peripheralStateNotificationCB,  // Profile State Change Callbacks
  NULL                            // When a valid RSSI is read from controller (not used by application)
};

// GAP Bond Manager Callbacks
static gapBondCBs_t simpleBLEPeripheral_BondMgrCBs =
{
  NULL,                     // Passcode callback (not used by application)
  NULL                      // Pairing / Bonding state Callback (not used by application)
};

// Simple GATT Profile Callbacks
static simpleProfileCBs_t simpleBLEPeripheral_SimpleProfileCBs =
{
  simpleProfileChangeCB    // Charactersitic value change callback
};

void Aes128EncryptAndDecrypTest(void)
{
  int i = 0;
  uint8 key[16] = {1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16};
  uint8 source_buf[16] = {1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16};
  uint8 encrypted_buf[16];
  uint8 deccrypted_buf[16];

  LL_Encrypt( key, source_buf, encrypted_buf );
  LL_EXT_Decrypt( key, encrypted_buf, deccrypted_buf);

  //tx_printf("source:");
  for( i=0; i<16; i++)
  {
    UART_PrintValue(" ", source_buf[i],16);
  }
  //tx_printf("");
  //tx_printf("deccrypte:");
  UART_Print("****************",16);
  for(i=0; i<16; i++)
  {
    UART_PrintValue(" ", deccrypted_buf[i],16);
 // 	txprintf("0x%02x ", deccrypted_buf[i]);
  }
  //tx_printf("");
}

void SimpleBLEPeripheral_Init( uint8 task_id )
{
  simpleBLEPeripheral_TaskID = task_id;
  
  SerialApp_Init(task_id);
  UART_PrintString("JDprofile Start\r\n");     //串口打印

  // Setup the GAP
  VOID GAP_SetParamValue( TGAP_CONN_PAUSE_PERIPHERAL, DEFAULT_CONN_PAUSE_PERIPHERAL );
  
  // Setup the GAP Peripheral Role Profile
  {
    #if defined( CC2540_MINIDK )
      // For the CC2540DK-MINI keyfob, device doesn't start advertising until button is pressed
      uint8 initial_advertising_enable = FALSE;
    #else
      // For other hardware platforms, device starts advertising upon initialization
      uint8 initial_advertising_enable = TRUE;
    #endif

    // By setting this to zero, the device will go into the waiting state after
    // being discoverable for 30.72 second, and will not being advertising again
    // until the enabler is set back to TRUE
    uint16 gapRole_AdvertOffTime = 0;

    uint8 enable_update_request = DEFAULT_ENABLE_UPDATE_REQUEST;
    uint16 desired_min_interval = DEFAULT_DESIRED_MIN_CONN_INTERVAL;
    uint16 desired_max_interval = DEFAULT_DESIRED_MAX_CONN_INTERVAL;
    uint16 desired_slave_latency = DEFAULT_DESIRED_SLAVE_LATENCY;
    uint16 desired_conn_timeout = DEFAULT_DESIRED_CONN_TIMEOUT;

    // Set the GAP Role Parameters
    GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &initial_advertising_enable );
    GAPRole_SetParameter( GAPROLE_ADVERT_OFF_TIME, sizeof( uint16 ), &gapRole_AdvertOffTime );

    GAPRole_SetParameter( GAPROLE_SCAN_RSP_DATA, sizeof ( scanRspData ), scanRspData );
    GAPRole_SetParameter( GAPROLE_ADVERT_DATA, sizeof( advertData ), advertData );

    GAPRole_SetParameter( GAPROLE_PARAM_UPDATE_ENABLE, sizeof( uint8 ), &enable_update_request );
    GAPRole_SetParameter( GAPROLE_MIN_CONN_INTERVAL, sizeof( uint16 ), &desired_min_interval );
    GAPRole_SetParameter( GAPROLE_MAX_CONN_INTERVAL, sizeof( uint16 ), &desired_max_interval );
    GAPRole_SetParameter( GAPROLE_SLAVE_LATENCY, sizeof( uint16 ), &desired_slave_latency );
    GAPRole_SetParameter( GAPROLE_TIMEOUT_MULTIPLIER, sizeof( uint16 ), &desired_conn_timeout );
  }

  // Set the GAP Characteristics
  GGS_SetParameter( GGS_DEVICE_NAME_ATT, GAP_DEVICE_NAME_LEN, attDeviceName );

  // Set advertising interval
  {
    uint16 advInt = DEFAULT_ADVERTISING_INTERVAL;

    GAP_SetParamValue( TGAP_LIM_DISC_ADV_INT_MIN, advInt );
    GAP_SetParamValue( TGAP_LIM_DISC_ADV_INT_MAX, advInt );
    GAP_SetParamValue( TGAP_GEN_DISC_ADV_INT_MIN, advInt );
    GAP_SetParamValue( TGAP_GEN_DISC_ADV_INT_MAX, advInt );
  }

  // Setup the GAP Bond Manager
  {
    uint32 passkey = 0; // passkey "000000"
    uint8 pairMode = GAPBOND_PAIRING_MODE_WAIT_FOR_REQ;
    uint8 mitm = TRUE;
    uint8 ioCap = GAPBOND_IO_CAP_DISPLAY_ONLY;
    uint8 bonding = TRUE;
    GAPBondMgr_SetParameter( GAPBOND_DEFAULT_PASSCODE, sizeof ( uint32 ), &passkey );
    GAPBondMgr_SetParameter( GAPBOND_PAIRING_MODE, sizeof ( uint8 ), &pairMode );
    GAPBondMgr_SetParameter( GAPBOND_MITM_PROTECTION, sizeof ( uint8 ), &mitm );
    GAPBondMgr_SetParameter( GAPBOND_IO_CAPABILITIES, sizeof ( uint8 ), &ioCap );
    GAPBondMgr_SetParameter( GAPBOND_BONDING_ENABLED, sizeof ( uint8 ), &bonding );
  }

  // Initialize GATT attributes
  GGS_AddService( GATT_ALL_SERVICES );            // GAP
  GATTServApp_AddService( GATT_ALL_SERVICES );    // GATT attributes
  DevInfo_AddService();                           // Device Information Service
  SimpleProfile_AddService( GATT_ALL_SERVICES );  // Simple GATT Profile
  // Setup the SimpleProfile Characteristic Values
  {
    uint8 charValue1[20] = {1,2,3,4,5,6,7,8,9,0,1,2,3,4,5,6,7,8,9,0xA};
   // SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR1, 20, charValue1 );
  //  
  }  
//init gpio
//  P1SEL  = 0x00;
//  P1DIR |= 0x03;
//  P1DIR &= ~0x08;
//  P1INP &= ~0x08;

  RegisterForKeys( simpleBLEPeripheral_TaskID );  // 注册按键，
  
  // Register callback with SimpleGATTprofile
  VOID SimpleProfile_RegisterAppCBs( &simpleBLEPeripheral_SimpleProfileCBs );

  // Enable clock divide on halt
  // This reduces active current while radio is active and CC254x MCU
  // is halted
  HCI_EXT_ClkDivOnHaltCmd( HCI_EXT_ENABLE_CLK_DIVIDE_ON_HALT );

#if defined ( DC_DC_P0_7 )
  // Enable stack to toggle bypass control on TPS62730 (DC/DC converter)
  HCI_EXT_MapPmIoPortCmd( HCI_EXT_PM_IO_PORT_P0, HCI_EXT_PM_IO_PORT_PIN7 );
#endif // defined ( DC_DC_P0_7 )
  
  Aes128EncryptAndDecrypTest();
  // Setup a delayed profile startup
  osal_set_event( simpleBLEPeripheral_TaskID, SBP_START_DEVICE_EVT );

}

uint16 SimpleBLEPeripheral_ProcessEvent( uint8 task_id, uint16 events )
{

  VOID task_id; // OSAL required parameter that isn't used in this function

  if ( events & SYS_EVENT_MSG )
  {
    uint8 *pMsg;

    if ( (pMsg = osal_msg_receive( simpleBLEPeripheral_TaskID )) != NULL )   //系统消息
    {
      simpleBLEPeripheral_ProcessOSALMsg( (osal_event_hdr_t *)pMsg );      //消息处理

      // Release the OSAL message
      VOID osal_msg_deallocate( pMsg );
    }
    // return unprocessed events
    return (events ^ SYS_EVENT_MSG);
  }

  if ( events & SBP_START_DEVICE_EVT )
  {
    // Start the Device
    VOID GAPRole_StartDevice( &simpleBLEPeripheral_PeripheralCBs );

    // Start Bond Manager
    VOID GAPBondMgr_Register( &simpleBLEPeripheral_BondMgrCBs );

    // Set timer for first periodic event
    osal_start_timerEx( simpleBLEPeripheral_TaskID, SBP_PERIODIC_EVT, SBP_PERIODIC_EVT_PERIOD );

    return ( events ^ SBP_START_DEVICE_EVT );
  }

  if ( events & SBP_PERIODIC_EVT )
  {
    // Restart timer
    if ( SBP_PERIODIC_EVT_PERIOD )
    {
      osal_start_timerEx( simpleBLEPeripheral_TaskID, SBP_PERIODIC_EVT, SBP_PERIODIC_EVT_PERIOD );
    }

    // Perform periodic application task
    //adc
    //adcres[0] = HalAdcRead (HAL_ADC_CHANNEL_0, HAL_ADC_RESOLUTION_8);
    
   // Getmac
    SendNotify(macaddr,6);

    return (events ^ SBP_PERIODIC_EVT);
  }

  // Discard unknown events
  return 0;
}

/*********************************************************************
*系统消息处理函数
判断是什么消息， pMsg->event 的值。
 */

// 按键处理函数
static void simpleBLEPeripheral_HandleKeys( uint8 shift, uint8 keys )
{
  VOID shift;  // Intentionally unreferenced parameter

  if ( keys & HAL_KEY_SW_6 )
  {
    P1_0=0;
    P1_1=0;
  //  UART_PrintString("HAL_KEY_SW_6 Start DOWN \r\n"); ;
  }
  if ( keys == 0 )
  {
    P1_0=0;
    P1_1=0;
  //  UART_PrintString("HAL_KEY_SW_6 Start UP \r\n"); ;
  }
}

static void simpleBLEPeripheral_ProcessOSALMsg( osal_event_hdr_t *pMsg )
{
  switch ( pMsg->event )
  {
      case KEY_CHANGE:
        simpleBLEPeripheral_HandleKeys( ((keyChange_t *)pMsg)->state, ((keyChange_t *)pMsg)->keys );
        break;
      default:
      // do nothing
        break;
  }
}

static void peripheralStateNotificationCB( gaprole_States_t newState )
{
#ifdef PLUS_BROADCASTER
  static uint8 first_conn_flag = 0;
#endif // PLUS_BROADCASTER
  
  switch ( newState )
  {
    case GAPROLE_STARTED:
      {
        uint8 ownAddress[B_ADDR_LEN];
        uint8 systemId[DEVINFO_SYSTEM_ID_LEN];

        GAPRole_GetParameter(GAPROLE_BD_ADDR, ownAddress);

        // use 6 bytes of device address for 8 bytes of system ID value
        systemId[0] = ownAddress[0];
        systemId[1] = ownAddress[1];
        systemId[2] = ownAddress[2];

        // set middle bytes to zero
        systemId[4] = 0x00;
        systemId[3] = 0x00;

        // shift three bytes up
        systemId[7] = ownAddress[5];
        systemId[6] = ownAddress[4];
        systemId[5] = ownAddress[3];
        
        
        macaddr[5] = ownAddress[0];
        macaddr[4] = ownAddress[1];
        macaddr[3] = ownAddress[2];
        macaddr[2] = ownAddress[3];
        macaddr[1] = ownAddress[4];
        macaddr[0] = ownAddress[5];
         
//3E E8 9C 19 FD C8  
//C8 FD 19 9C E8 3E
        
        DevInfo_SetParameter(DEVINFO_SYSTEM_ID, DEVINFO_SYSTEM_ID_LEN, systemId);
      }
      break;

    case GAPROLE_ADVERTISING:
      {
      //    HalLcdWriteString( "Advertising",  HAL_LCD_LINE_3 );
      }
      break;

    case GAPROLE_CONNECTED:
      {        
      //    HalLcdWriteString( "Connected",  HAL_LCD_LINE_3 );
      
#ifdef PLUS_BROADCASTER
        // Only turn advertising on for this state when we first connect
        // otherwise, when we go from connected_advertising back to this state
        // we will be turning advertising back on.
        if ( first_conn_flag == 0 ) 
        {
          uint8 adv_enabled_status = 1;
          GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8), &adv_enabled_status); // Turn on Advertising
          first_conn_flag = 1;
        }
#endif // PLUS_BROADCASTER
      }
      break;

    case GAPROLE_CONNECTED_ADV:
      {
      //    HalLcdWriteString( "Connected Advertising",  HAL_LCD_LINE_3 );
      }
      break;      
    case GAPROLE_WAITING:
      {

      //    HalLcdWriteString( "Disconnected",  HAL_LCD_LINE_3 );

      }
      break;

    case GAPROLE_WAITING_AFTER_TIMEOUT:
      {

       //   HalLcdWriteString( "Timed Out",  HAL_LCD_LINE_3 );

          
#ifdef PLUS_BROADCASTER
        // Reset flag for next connection.
        first_conn_flag = 0;
#endif //#ifdef (PLUS_BROADCASTER)
      }
      break;

    case GAPROLE_ERROR:
      {
        //  HalLcdWriteString( "Error",  HAL_LCD_LINE_3 );
      }
      break;

    default:
      {
      //    HalLcdWriteString( "",  HAL_LCD_LINE_3 );
      }
      break;
  }
  gapProfileState = newState;
}

static void performPeriodicTask( void )
{
  //uint8 value[20]={0,1,2,3};
  //SendNotify(value,20);

}

uint8 * Getmac(void)
{

    GAPRole_GetParameter(GAPROLE_BD_ADDR, macaddr);
    
    return macaddr;
}
   
uint8 crcdate[27]={0xef,0x01,0x13,0xc4,0x4a,0x19,0x68,0x02,0x01,0x13,0x2e,0x00,0x0a,0x13,0x04,0x03,0x16,0x0e,0x34,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
//uint8 crcdate[20]={0xef,0x01,0x13,0xc4,0x4a,0x19,0x68,0x02,0x01,0x13,0x2e,0x00,0x0a,0x13,0x04,0x03,0x16,0x0e,0x34,0x00};

unsigned short ModBusCRC (unsigned char *ptr,unsigned char size)
{
    unsigned short a,b,tmp,CRC16,V;
    CRC16=0xffff; //CRC 寄存器初始值
    for (a=0;a<size;a++) //N 个字节
    {
        CRC16=*ptr^CRC16;
        for (b=0;b<8;b++) //8 位数据
        {
            tmp=CRC16 & 0x0001;
            CRC16 =CRC16 >>1; // 右移一位
            if (tmp)
            CRC16=CRC16 ^ 0xa001; // 异或多项式
        }
        *ptr++;
    }
    V = ((CRC16 & 0x00FF) << 8) | ((CRC16 & 0xFF00) >> 8) ;// 高低字节转换
    return V;
}

/*********************************************************************
从机接收数据函数，当有数据时，通知用户层。系统回调/
*/
static uint8 rcv_data[29]={0};
static void simpleProfileChangeCB( uint8 paramID )
{
  uint8 newValue[20];
  uint8 phonemac[6];
  uint8 i = 0;
  uint8 send_par_1[20]={0};
  uint8 send_par_2[20]={0};
  uint8 send_crc[31];

  switch( paramID )
  {
    case SIMPLEPROFILE_CHAR1:  //CHAR1 的数据接收，当主机通过writechar发送数据后，从机接收。
      SimpleProfile_GetParameter( SIMPLEPROFILE_CHAR1, newValue );
      
      if((newValue[0] == 0xef)&&(newValue[1]==0x01))
      {
      	for(uint8 i = 0;i < 20; i++)
	      rcv_data[i] = newValue[i];     	
      }
      else
      {
      	for(uint8 i = 0;i < 9; i++)
	      rcv_data[i+20] = newValue[i]; 
      }
	
	uint16 crcres = ModBusCRC(rcv_data, 27);

	if(((crcres&0x00FF)==rcv_data[28])&&(crcres>>8 &&0x00FF)==rcv_data[27])
	{
		UART_PrintString("Ok");
	//2e
		if( rcv_data[10]==0x2e )
		{
			send_par_1[0] = 0xef;
			send_par_1[1] = 0x01;
		    send_par_1[2] = macaddr[0];
		    send_par_1[3] = macaddr[1];
		    send_par_1[4] = macaddr[2];
		    send_par_1[5] = macaddr[3];
		    send_par_1[6] = macaddr[4];
		    send_par_1[7] = macaddr[5];
		    send_par_1[8] = 0x07;
		    send_par_1[9] = 0x15;
		    send_par_1[10]= 0x2e;
		    send_par_1[11]= 0x00;
		    send_par_1[12]= 0x0a;
		    send_par_1[13]=	macaddr[0];
		    send_par_1[14]=	macaddr[1];
		    send_par_1[15]=	macaddr[2];
		    send_par_1[16]=	macaddr[3];
		    send_par_1[17]=	macaddr[4];
		    send_par_1[18]=	macaddr[5];
		    send_par_1[19]= 0x00;
		    send_par_2[0]= 0x00;
		    send_par_2[1]= 0x00;
		    send_par_2[2]= 0x00;
		    send_par_2[3]= 0x00;
		    send_par_2[4]= 0x00;
		    send_par_2[5]= 0x00;
		    send_par_2[6]= 0x00;
		    send_par_2[7]= 0x00;
		    send_par_2[8]= 0x00;

		    for (int i = 0; i < 20; ++i)
	    	{
	    		send_crc[i] = send_par_1[i];
	    	}
		    for (int i = 20; i < 29; ++i)
	    	{
	    		send_crc[i] = send_par_2[i];
	    	}
	    	uint16 sendcrcres = ModBusCRC(send_crc, 31);	    		
		    send_par_2[9]= (sendcrcres>>8&0x00FF);
		    send_par_2[10]= sendcrcres&0x00FF;
		
		SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR1, 20, send_par_1 );
	  	SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR1, 20, send_par_2 );

	    }
        }
      default:
        motoA = 0;
        motoB = 0;
      // should not reach here!
      break;
  }
}

void SendNotify(uint8 *pBuffer,uint16 length)
{
  uint8 len;
  if(length > 20)
    len = 20;
  else
    len = length;
  static attHandleValueNoti_t pReport;
  pReport.handle=0x28;
  pReport.len = len;
  osal_memcpy(pReport.value, pBuffer, len);
  GATT_Notification( 0, &pReport, FALSE );
}

/*********************************************************************
void ReadMac(unsigned char *TempMacAddress,int len)  // Len 一定是6  
{  
   TempMacAddress[5]=XREG(0x780E); // 直接指向指针内容  
   TempMacAddress[4]=XREG(0x780F);  
   TempMacAddress[3]=XREG(0x7810);  
   TempMacAddress[2]=XREG(0x7811);                // define 函数直接读出数据  
   TempMacAddress[1]=XREG(0x7812);  
   TempMacAddress[0]=XREG(0x7813);   
}

#define XREG(addr)       ((unsigned char volatile __xdata *) 0)[addr]





//随机数

uint16 osal_rand( void );





void InvertUint8(unsigned char *dBuf,unsigned char *srcBuf)
{
    int i;
    unsigned char tmp[4];
    tmp[0] = 0;
    for(i=0;i< 8;i++)
    {
      if(srcBuf[0]& (1 << i))
        tmp[0]|=1<<(7-i);
    }
    dBuf[0] = tmp[0];
    
}
void InvertUint16(unsigned short *dBuf,unsigned short *srcBuf)
{
    int i;
    unsigned short tmp[4];
    tmp[0] = 0;
    for(i=0;i< 16;i++)
    {
      if(srcBuf[0]& (1 << i))
        tmp[0]|=1<<(15 - i);
    }
    dBuf[0] = tmp[0];
}

--------------------- 
作者：leumber 
来源：CSDN 
原文：https://blog.csdn.net/leumber/article/details/54311811 
版权声明：本文为博主原创文章，转载请附上博文链接！


unsigned short CRC16_MODBUS(unsigned char *puchMsg, unsigned int usDataLen)
{
  unsigned short wCRCin = 0xFFFF;
  unsigned short wCPoly = 0x8005;
  unsigned char wChar = 0;
  
  while (usDataLen--) 	
  {
        wChar = *(puchMsg++);
        InvertUint8(&wChar,&wChar);
        wCRCin ^= (wChar << 8);
        for(int i = 0;i < 8;i++)
        {
          if(wCRCin & 0x8000)
            wCRCin = (wCRCin << 1) ^ wCPoly;
          else
            wCRCin = wCRCin << 1;
        }
  }
  InvertUint16(&wCRCin,&wCRCin);
  return (wCRCin) ;
}


#define POLYNOMIAL          0x8005 
#define INITIAL_REMAINDER   0x0000 
#define FINAL_XOR_VALUE     0x0000 

typedef unsigned short width_t;  
#define WIDTH (8 * sizeof(width_t))  
#define TOPBIT (1 << (WIDTH - 1))  
uint8 crcdate[27]={0xef,0x01,0x13,0xc4,0x4a,0x19,0x68,0x02,0x01,0x13,0x2e,0x00,0x0a,0x13,0x04,0x03,0x16,0x0e,0x34,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
uint16 crcTable[27];
void crcInit(void)  
{  
    uint16 remainder;  
    uint16 dividend;  
    uint8 bit;  
    for(dividend = 0; dividend < 256; dividend++)  
    {   
        remainder = dividend << (WIDTH - 8);   
        for(bit = 0; bit < 8; bit++)  
        {  
            if(remainder & TOPBIT)   
                remainder = (remainder << 1) ^ POLYNOMIAL;   
            else   
                remainder = remainder << 1;  
        }   
        crcTable[dividend] = remainder; 
    }  
}
uint16 crcCompute(uint8 * message, uint8 nBytes)  
{  
    uint8 i;  
    uint8 byte;  
    uint16 remainder = 0x0000;  
  
    for( i = 0; i < nBytes; i++)  
    {  
        byte = (remainder >> (WIDTH - 8)) ^ message[i];  
        remainder = crcTable[byte] ^ (remainder << 8);  
    }   
    return (remainder ^ 0x0000);  
}

*********************************************************************/
