/**
  @file       wmbus_semaphore.c
  @copyright  STACKFORCE GmbH, Heitersheim, Germany, http://www.stackforce.de
  @author     STACKFORCE
  @brief      General module to create a wmbus semaphore
*/

/*==============================================================================
                            INCLUDE FILES
==============================================================================*/
/* XDCtools Header files */
#include <xdc/std.h>
#include <xdc/cfg/global.h>
#include <xdc/runtime/Error.h>
#include <xdc/runtime/System.h>

/* BIOS Header files */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Semaphore.h>
/*==============================================================================
                            DEFINES
==============================================================================*/

/*==============================================================================
                            VARIABLES
==============================================================================*/
/* Semaphore hanfle */
Semaphore_Handle gs_wmbusSemaphore;
/*==============================================================================
                            FUNCTION PROTOTYPES
==============================================================================*/

/*==============================================================================
                            LOCAL FUNCTIONS
==============================================================================*/

/*==============================================================================
                            FUNCTIONS
==============================================================================*/
/*============================================================================*/
/*! wmbus_semaphore_init() */
/*============================================================================*/
void semaphore_init(Error_Block* ps_eb)
{
  /* Parameter for creating the semaphore */
  Semaphore_Params s_semParams;

  Semaphore_Params_init(&s_semParams);
  s_semParams.mode = Semaphore_Mode_BINARY;
  gs_wmbusSemaphore =Semaphore_create(0, &s_semParams, ps_eb);

}/* wmbus_semaphore_init() */

/*============================================================================*/
/*! wmbus_semaphore_pend() */
/*============================================================================*/
void semaphore_pend(void)
{
  /* Ensure that the stack task will be blocked here */
  while(Semaphore_getCount(gs_wmbusSemaphore) > 0x00U)
  {
    Semaphore_pend(gs_wmbusSemaphore, BIOS_WAIT_FOREVER);
  }/* while */

  /* Block the wmbus_task until an event occured */
  Semaphore_pend(gs_wmbusSemaphore, BIOS_WAIT_FOREVER);
}/* wmbus_semaphore_pend */

/*============================================================================*/
/*! wmbus_semaphore_post() */
/*============================================================================*/
void semaphore_post(void)
{
  Semaphore_post(gs_wmbusSemaphore);
}/* wmbus_semaphore_post */
