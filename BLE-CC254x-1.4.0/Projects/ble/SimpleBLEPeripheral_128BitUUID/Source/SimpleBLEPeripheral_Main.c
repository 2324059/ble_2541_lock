/* Hal Drivers */
#include "hal_types.h"
#include "hal_key.h"
#include "hal_timer.h"
#include "hal_drivers.h"
#include "hal_led.h"

/* OSAL */
#include "OSAL.h"
#include "OSAL_Tasks.h"
#include "OSAL_PwrMgr.h"
#include "osal_snv.h"
#include "OnBoard.h"

void MSA_Main_KeyCallback(uint8 keys, uint8 state);

int main(void)
{
  HAL_BOARD_INIT();  /* Initialize hardware */
  InitBoard( OB_COLD );  // Initialize board I/O
  HalDriverInit();  /* Initialze the HAL driver */
  osal_snv_init();  /* Initialize NV system */
  /* Initialize LL */
  osal_init_system();  /* Initialize the operating system */
  HAL_ENABLE_INTERRUPTS();  /* Enable interrupts */
  InitBoard( OB_READY );  // Final board initialization
  #if defined ( POWER_SAVING )
    osal_pwrmgr_device( PWRMGR_BATTERY );
  #endif
  /* Start OSAL */
  osal_start_system(); // No Return from here

  return 0;
}

