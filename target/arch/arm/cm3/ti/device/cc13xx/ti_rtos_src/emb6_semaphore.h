#ifndef __WMBUS_SEMAPHORE_H__
#define __WMBUS_SEMAPHORE_H__
#ifndef __DECL_WMBUS_SEMAPHORE_H__
#define __DECL_WMBUS_SEMAPHORE_H__ extern
#else
#define __DECL_WMBUS_SEMAPHORE_H__
#endif /* __WMBUS_SEMAPHORE_H__ */

/**
  @file       wmbus_semaphore.h
  @copyright  STACKFORCE GmbH, Heitersheim, Germany, http://www.stackforce.de
  @author     STACKFORCE
  @brief      Wmbus semaphore module.

              Include before:
              - wmbus_typedefs.h
              - <ti/sysbios/knl/Semaphore.h>
*/


/*==============================================================================
                            DEFINES
==============================================================================*/

/*==============================================================================
                            FUNCTIONS
==============================================================================*/
/*============================================================================*/
/*!
 * @brief  Creates a wmbus Semaphore
 *
 * @param  ps_eb Pointer to the error handler
 */
/*============================================================================*/
void semaphore_init(Error_Block* ps_eb);

/*============================================================================*/
/*!
 * @brief  Wait until resource is available
 *
 */
/*============================================================================*/
void semaphore_pend(void);

/*============================================================================*/
/*!
 * @brief  Make the resource available
 *
 */
/*============================================================================*/
void semaphore_post(void);

#endif /* __WMBUS_SEMAPHORE_H__ */
