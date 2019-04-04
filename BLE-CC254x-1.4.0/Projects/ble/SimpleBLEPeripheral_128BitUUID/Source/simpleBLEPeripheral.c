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
  0x06,   // length of this data
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
static uint8 attDeviceName[GAP_DEVICE_NAME_LEN] = "E-Lock";

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
  //  SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR1, 20, charValue1 );
  //  UART_PrintString("reset********************init*******************\r\n"); 
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
  
  GAPRole_GetParameter(GAPROLE_BD_ADDR, macaddr);
  UART_PrintValue("mac= ",macaddr[0],16);
  UART_PrintValue("mac= ",macaddr[1],16);
  UART_PrintValue("mac= ",macaddr[2],16);
  UART_PrintValue("mac= ",macaddr[3],16);
  UART_PrintValue("mac= ",macaddr[4],16);
  UART_PrintValue("mac= ",macaddr[5],16);

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
    UART_PrintString("HAL_KEY_SW_6 Start DOWN \r\n"); ;
  }
  if ( keys == 0 )
  {
    P1_0=0;
    P1_1=0;
    UART_PrintString("HAL_KEY_SW_6 Start UP \r\n"); ;
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


/*********************************************************************
从机接收数据函数，当有数据时，通知用户层。系统回调/
*/

static void simpleProfileChangeCB( uint8 paramID )
{
  uint8 newValue[20];
  uint8 charValue1[20];
  uint8 phonemac[6];
  uint8 i = 0;
  switch( paramID )
  {
    case SIMPLEPROFILE_CHAR1:  //CHAR1 的数据接收，当主机通过writechar发送数据后，从机接收。
      SimpleProfile_GetParameter( SIMPLEPROFILE_CHAR1, newValue );
     
      for(i=0; i<20; i++)
        UART_PrintValue("newValu=",newValue[i],16);
 

      
      if( newValue[0] == 0x55 )
      {
          charValue1[11] = 0xaa;         
          SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR1, 20, charValue1 );
      }  
      else if(newValue[0] == 0x22)
      {
          motoA = 0;
          motoB = 1; 
          
          charValue1[0] = 0xbb;
          charValue1[1] = 0xbb;
          charValue1[2] = 0xab;
          charValue1[3] = 0xab;
          charValue1[4] = 0xab;
          charValue1[5] = 0xab;
          charValue1[6] = 0xab;
          charValue1[7] = 0xab;
          charValue1[8] = 0xab;
          charValue1[9] = 0xab;
          charValue1[10] = 0xba;
          charValue1[11] = 0xba;
          charValue1[12] = 0xba;
          charValue1[13] = 0xba;
          charValue1[14] = 0xba;
          charValue1[15] = 0xba;
          charValue1[16] = 0xba;
          charValue1[17] = 0xba;
          charValue1[18] = 0xba;
          charValue1[19] = 0xcc; 
          SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR1, 20, charValue1 );          
      }
      else if(newValue[0] == 0x33)
      { 
        motoA = 0;
        motoB = 0;      
      }     
      break;
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
*********************************************************************/
