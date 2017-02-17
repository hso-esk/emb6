/**
  @file       wmbus_task.c
  @copyright  STACKFORCE GmbH, Heitersheim, Germany, http://www.stackforce.de
  @author     STACKFORCE
  @brief      General module to create a wmbus task
*/

/*==============================================================================
                            INCLUDE FILES
==============================================================================*/
/* XDCtools Header files */
#include <xdc/std.h>
#include <xdc/cfg/global.h>
#include <xdc/runtime/Error.h>
#include <xdc/runtime/System.h>
#include <ti/sysbios/knl/Task.h>
/*==============================================================================
                            DEFINES
==============================================================================*/
/* Size of the wmbus task */
#define WMBUS_TASK_STACK_SIZE 4096U
/* Priority of the wmbus task */
#define WMBUS_TASK_PRIORITY   3U
/*==============================================================================
                            VARIABLES
==============================================================================*/
/* RAM used by the wmbus task */
static uint8_t ac_wmbusTaskStack[WMBUS_TASK_STACK_SIZE];
/* Parameters for the wmbus task */
static Task_Params taskParamWmbus;
/* Task struct for the wmbus */
static Task_Struct taskWmbus;
/*==============================================================================
                            FUNCTION PROTOTYPES
==============================================================================*/

/*==============================================================================
                            LOCAL FUNCTIONS
==============================================================================*/


/*==============================================================================
                            FUNCTIONS
==============================================================================*/
void task_init(ti_sysbios_knl_Task_FuncPtr fp_fxn, Error_Block* ps_eb)
{
  /* Create task for the wmbus handling */
  Task_Params_init(&taskParamWmbus);
  taskParamWmbus.stackSize = WMBUS_TASK_STACK_SIZE;
  taskParamWmbus.priority = WMBUS_TASK_PRIORITY;
  taskParamWmbus.stack = &ac_wmbusTaskStack;
  taskParamWmbus.arg0 = (UInt)1000000;
  Task_construct(&taskWmbus, fp_fxn, &taskParamWmbus, ps_eb);

}/* wmbus_task_init() */
