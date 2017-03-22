#ifndef __EMB6_TASK_H__
#define __EMB6_TASK_H__
#ifndef __DECL_WMBUS_TASK_H__
#define __DECL_WMBUS_TASK_H__ extern
#else
#define __DECL_WMBUS_TASK_H__
#endif /* __WMBUS_TASK_H__ */

/**
  @file       emb6_task.h
  @copyright  STACKFORCE GmbH, Heitersheim, Germany, http://www.stackforce.de
  @author     STACKFORCE
  @brief      Wmbus task module.

              Include before:
              - wmbus_typedefs.h
              - <ti/sysbios/knl/Task.h>
*/


/*==============================================================================
                            DEFINES
==============================================================================*/

/*==============================================================================
                            FUNCTIONS
==============================================================================*/
/*============================================================================*/
/*!
 * @brief  Creates a emb6 task for the specified function pointer
 *
 * @param fp_fxn  Function which sould be used as emb6 task
 * @param ps_eb   Pointer to the error handler
 */
/*============================================================================*/
void emb6_task_init(ti_sysbios_knl_Task_FuncPtr fp_fxn, Error_Block* ps_eb);

#endif /* __WMBUS_TASK_H__ */
