/**
  @file       emb6_task.c
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
/* Size of the emb6 task */
#define EMB6_TASK_STACK_SIZE   2048U //4096U
/* Priority of the wmbus task */
#define EMB6_TASK_PRIORITY   3U
/*==============================================================================
                            VARIABLES
==============================================================================*/
/* RAM used by the wmbus task */
static uint8_t ac_emb6TaskStack[EMB6_TASK_STACK_SIZE];
/* Parameters for the wmbus task */
static Task_Params taskParamEmb6;
/* Task struct for the wmbus */
static Task_Struct taskEmb6;
/*==============================================================================
                            FUNCTION PROTOTYPES
==============================================================================*/

/*==============================================================================
                            LOCAL FUNCTIONS
==============================================================================*/


/*==============================================================================
                            FUNCTIONS
==============================================================================*/
void emb6_task_init(ti_sysbios_knl_Task_FuncPtr fp_fxn, Error_Block* ps_eb)
{
  /* Create task for the wmbus handling */
  Task_Params_init(&taskParamEmb6);
  taskParamEmb6.stackSize = EMB6_TASK_STACK_SIZE;
  taskParamEmb6.priority = EMB6_TASK_PRIORITY;
  taskParamEmb6.stack = &ac_emb6TaskStack;
  taskParamEmb6.arg0 = (UInt)1000000;
  Task_construct(&taskEmb6, fp_fxn, &taskParamEmb6, ps_eb);

}/* wmbus_task_init() */
