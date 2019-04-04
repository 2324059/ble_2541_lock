#include "hal_mcu.h"
#include "hal_defs.h"
#include "hal_types.h"
#include "hal_drivers.h"
#include "hal_adc.h"
#include "hal_key.h"
#include "osal.h"

#define HAL_KEY_RISING_EDGE   0
#define HAL_KEY_FALLING_EDGE  1
#define HAL_KEY_DEBOUNCE_VALUE  25
/* CPU port interrupt */
#define HAL_KEY_CPU_PORT_0_IF P0IF
#define HAL_KEY_CPU_PORT_1_IF P1IF
#define HAL_KEY_CPU_PORT_2_IF P2IF
/* SW_6 is at P0.1 */
#define HAL_KEY_SW_6_PORT   P1//P0
#define HAL_KEY_SW_6_BIT    BV(3)//BV(1)
#define HAL_KEY_SW_6_SEL    P1SEL//P0SEL
#define HAL_KEY_SW_6_DIR    P1DIR//P0DIR
/* edge interrupt */
#define HAL_KEY_SW_6_EDGEBIT  BV(1)//BV(0)
#define HAL_KEY_SW_6_EDGE     HAL_KEY_RISING_EDGE//HAL_KEY_FALLING_EDGE
/* SW_6 interrupts */
#define HAL_KEY_SW_6_IEN      IEN2//IEN1  /* CPU interrupt mask register */
#define HAL_KEY_SW_6_IENBIT   BV(4)//BV(5) /* Mask bit for all of Port_0 */
#define HAL_KEY_SW_6_ICTL     P1IEN//P0IEN /* Port Interrupt Control register */
#define HAL_KEY_SW_6_ICTLBIT  BV(3)//BV(1) /* P0IEN - P0.1 enable/disable bit */
#define HAL_KEY_SW_6_PXIFG    P1IFG//P0IFG /* Interrupt flag at source */
static uint8 halKeySavedKeys;     /* used to store previous key state in polling mode */
static halKeyCBack_t pHalKeyProcessFunction;
static uint8 HalKeyConfigured;
bool Hal_KeyIntEnable;            /* interrupt enable/disable flag */
void halProcessKeyInterrupt(void);

/**************************************************************************************************
 *                                        FUNCTIONS - API
 **************************************************************************************************/
void HalKeyInit( void )
{
  halKeySavedKeys = 0;  // Initialize previous key to 0.

  P1SEL &= ~0x08; 
//  HAL_KEY_SW_6_SEL &= ~(HAL_KEY_SW_6_BIT);    /* Set pin function to GPIO */
  P1DIR &= ~0x08; 
//  HAL_KEY_SW_6_DIR &= ~(HAL_KEY_SW_6_BIT);    /* Set pin direction to Input */

//  P2INP |= PUSH2_BV;  /* Configure GPIO tri-state. */
  /* Initialize callback function */
  pHalKeyProcessFunction  = NULL;

  /* Start with key is not configured */
  HalKeyConfigured = FALSE;
}

void HalKeyConfig (bool interruptEnable, halKeyCBack_t cback)
{
  /* Enable/Disable Interrupt or */
  Hal_KeyIntEnable = interruptEnable;
  /* Register the callback fucntion */
  pHalKeyProcessFunction = cback;
  /* Determine if interrupt is enable or not */
  if (Hal_KeyIntEnable)
  {
    PICTL &= ~0x02;    
    // PICTL &= ~(HAL_KEY_SW_6_EDGEBIT);   // 端口1 3-0为 上升沿产生中断。
    P1IEN &= ~0x03;    //保证P1.0 P1.1 禁止中断；
    P1IEN |= 0x08;     
   //  HAL_KEY_SW_6_ICTL |= HAL_KEY_SW_6_ICTLBIT;  // p1.3 中断使能
    IEN2 |= 0x10;     //
   //  HAL_KEY_SW_6_IEN |= HAL_KEY_SW_6_IENBIT;
    P1IFG &= 0x08;       
   //  HAL_KEY_SW_6_PXIFG = ~(HAL_KEY_SW_6_BIT);

    if (HalKeyConfigured == TRUE)
    {
      osal_stop_timerEx(Hal_TaskID, HAL_KEY_EVENT);  /* Cancel polling if active */
    }
  }
  else    /* Interrupts NOT enabled */
  {
    P1IEN &= ~0x08;     
    //HAL_KEY_SW_6_ICTL &= ~(HAL_KEY_SW_6_ICTLBIT); /* don't generate interrupt *///  P1.3禁用中断
    IEN2 &= ~0x10;     
    //  HAL_KEY_SW_6_IEN &= ~(HAL_KEY_SW_6_IENBIT);   /* Clear interrupt enable bit */

    osal_set_event(Hal_TaskID, HAL_KEY_EVENT);
  }

  HalKeyConfigured = TRUE;
}

uint8 HalKeyRead ( void )
{
  uint8 keys = 0;

//  if ( (HAL_KEY_SW_6_PORT & HAL_KEY_SW_6_BIT))    /* Key is active high */
//  if (!(HAL_KEY_SW_6_PORT & HAL_KEY_SW_6_BIT))    /* Key is active low */
  if( P1_3 == 1 )
  {
    keys |= 0x08;     //HAL_KEY_SW_6;
  }

  return keys;
}

void HalKeyPoll (void)
{
  uint8 keys = 0;
  uint8 notify = 0;

//  if (!(HAL_KEY_SW_6_PORT & HAL_KEY_SW_6_BIT))    /* Key is active low */
  if(P1_3 == 1)
  {
    keys |= 0x08;  //HAL_KEY_SW_6;
  }

  /* If interrupts are not enabled, previous key status and current key status
   * are compared to find out if a key has changed status.
   */
  if (!Hal_KeyIntEnable)
  {
    if (keys == halKeySavedKeys)
    {
      /* Exit - since no keys have changed */
      return;
    }
    else
    {
      notify = 1;
    }
  }
  else
  {
    /* Key interrupt handled here */
    if (keys)
    {
      notify = 1;
    }
  }

  /* Store the current keys for comparation next time */
  halKeySavedKeys = keys;

  /* Invoke Callback if new keys were depressed */
  if (notify && (pHalKeyProcessFunction))
  {
    (pHalKeyProcessFunction) (keys, HAL_KEY_STATE_NORMAL);

  }
}

void halProcessKeyInterrupt (void)
{
  bool valid=0;
  //if (HAL_KEY_SW_6_PXIFG & HAL_KEY_SW_6_BIT)  /* Interrupt Flag has been set */
  if( P1IFG & 0x08 )
  {
   // HAL_KEY_SW_6_PXIFG = ~(HAL_KEY_SW_6_BIT); /* Clear Interrupt Flag */
    P1IFG &= ~0x08;   // 清楚P1.3中断标志位；
    valid = 1;
  }

  if (valid == 1)
  {
    osal_start_timerEx (Hal_TaskID, HAL_KEY_EVENT, HAL_KEY_DEBOUNCE_VALUE);
  }
}

void HalKeyEnterSleep ( void )
{
}

uint8 HalKeyExitSleep ( void )
{
  /* Wake up and read keys */
  return ( HalKeyRead () );
}

HAL_ISR_FUNCTION( halKeyPort1Isr, P1INT_VECTOR )
{
  HAL_ENTER_ISR();
  if (HAL_KEY_SW_6_PXIFG & HAL_KEY_SW_6_BIT)
  // if( (P1IFG & 0x08) == 1 )
  {
    halProcessKeyInterrupt();
  }
  /*
    Clear the CPU interrupt flag for Port_0
    PxIFG has to be cleared before PxIF
  */
  P1IFG = 0;
  P1IF = 0;
  CLEAR_SLEEP_MODE();
  HAL_EXIT_ISR();
  return;
}


