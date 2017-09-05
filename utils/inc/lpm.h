#ifndef __LPM_H__
#define __LPM_H__
#ifndef __DECL_LPM_H__
#define __DECL_LPM_H__
#endif /* #ifndef __DECL_LPM_H__ */

/*============================================================================*/
/**
 * \file    lpm.h
 *
 * \author  Manuel Schappacher
 *
 * \brief   Low Power Manager.
 *
 */
/*============================================================================*/


/*============================================================================*/
/*                                INCLUDES                                    */
/*============================================================================*/


/*============================================================================*/
/*                                DEFINES                                     */
/*============================================================================*/

/** \brief  Successful operation */
#define LPM_SUCCESS                         (0)
/** \brief  General error */
#define LPM_ERROR                           (-1)

/** \brief  General error */
#define LPM_MOD_BUSY                        (-1)
/** \brief  General error */
#define LPM_MOD_IDLE                        (0)


/*============================================================================*/
/*                               TYPEDEFS                                     */
/*============================================================================*/

/**
 * \brief       Prototype of a registerd module
 *
 *              This function prototype is used by the LPM to check if all the
 *              registered modules are idle. Therefore this function must return
 *              \ref LPM_MOD_BUSY if the module is busy, \ref LPM_MOD_IDLE if
 *              the module is completly idle or a value greater 0 if the
 *              module is idle bus has some action sheduled after a specific
 *              time identifyable by the value
 *
 */
typedef int32_t (*pf_lpm_check)( void );


/*============================================================================*/
/*                    FUNCTION PROTOTYPES OF THE API                          */
/*============================================================================*/


/*============================================================================*/
/**
 * \brief   Initialize LPM.
 *
 * \return  The status of the request as follows:
 *          \ref LPM_SUCCESS on success
 *          \ref LPM_ERROR on general error
 */
/*============================================================================*/
int8_t lpm_init( void );


/*============================================================================*/
/**
 * \brief   Entry function of the LPM module.
 *
 *          This function must be called periodically as often as possible
 *          to provide a proper behavior of the LPM module.
 *
 * \return  \ref LPM_MOD_BUSY if the LPM did not sleep or the time otherwise
 *           the sleep duration.
 */
/*============================================================================*/
int32_t lpm_entry( void );


/*============================================================================*/
/**
 * \brief   Register a module at the LPM.
 *
 *          This function must be used to register a module at the LPM. After
 *          registration the LPM checks the module before it enters a
 *          lowpower mode.
 *
 * \param   pf_check    Function pointer to use to check if the module
 *                      is idle or not.
 *
 * \return  Id of the registered module or \ref LPM_ERROR on error e.g. (to
 *          much modules are registered)
 */
/*============================================================================*/
int8_t lpm_register( pf_lpm_check pf_check );


/*============================================================================*/
/**
 * \brief   Unregister a module at the LPM.
 *
 *          This function must be used to unregister a module at the LPM. After
 *          unregistration the LPM does not check the module before it enters a
 *          lowpower mode.
 *
 * \param   c_id        ID of the module to remove.
 *
 * \return  Id of the registered module or \ref LPM_ERROR on error e.g. (to
 *          much modules are registered)
 */
/*============================================================================*/
int8_t lpm_unregister( int8_t c_id );

#endif /* #ifndef __LPM_H__ */


