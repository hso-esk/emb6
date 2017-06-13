/*============================================================================*/
/**
 * \file    lpm.c
 *
 * \author  Manuel Schappacher
 *
 * \brief   Low Power Manager.
 *
 */
/*============================================================================*/

#ifdef __cplusplus
extern "C"
{
#endif

/*============================================================================*/
/*                                INCLUDES                                    */
/*============================================================================*/
#include "emb6.h"
#include "bsp.h"
#include "lpm.h"

#if (HAL_SUPPORT_MCU_SLEEP == TRUE)
/*============================================================================*/
/*                                DEFINES                                     */
/*============================================================================*/

/** Maximum number of modules to register */
#define LMP_MODULES_MAX                     (5)

/** Maximum time to sleep */
#if !defined(LPM_SLEEP_MAX)
#define LPM_SLEEP_MAX                       (5000)
#endif

/*============================================================================*/
/*                        STRUCTURES AND OTHER TYPEDEFS                       */
/*============================================================================*/

/**
 * Structure of an LPM element.
 */
typedef struct S_LPM_ENTRY_T
{
  /** entry is free */
  uint8_t uc_free;

  /** registered check function*/
  pf_lpm_check pf_check;
} s_lmp_entry_t;


/*============================================================================*/
/*                              LOCAL VARIABLES                               */
/*============================================================================*/

/** table of LPM entries */
s_lmp_entry_t gs_lpm_entry[LMP_MODULES_MAX];



/*============================================================================*/
/*                          FUNCTIONS OF THE API                              */
/*============================================================================*/


/*=============================================================================
 *  lpm_init()
 *============================================================================*/
int8_t lpm_init( void )
{
  int i_ret = LPM_ERROR;
  int i = 0;

  for( i = 0; i < LMP_MODULES_MAX; i++ )
  {
    /* reset all entries */
    gs_lpm_entry[i].uc_free = 1;
    gs_lpm_entry[i].pf_check =0;
  }

  i_ret = LPM_SUCCESS;
  return i_ret;
} /* lpm_init() */


/*=============================================================================
 *  lpm_entry()
 *============================================================================*/
int32_t lpm_entry( void )
{
  int32_t l_ret = LPM_MOD_BUSY;
  int32_t l_lpmRet = 0;
  int32_t l_tot = LPM_SLEEP_MAX;
  int i = 0;

  for( i = 0; i < LMP_MODULES_MAX; i++ )
  {
    /* check for a valid entry */
    if( (gs_lpm_entry[i].uc_free == 0) && (gs_lpm_entry[i].pf_check) != NULL )
    {
      /* Entry is valid. Check for the status */
      l_lpmRet = gs_lpm_entry[i].pf_check();
      if( l_lpmRet == LPM_MOD_BUSY )
        /* Module is busy and we can stop checking further modules */
        break;
      else
      {
        /* Module is idle. Use return value to set the time
         to sleep */
        if( l_lpmRet != LPM_MOD_IDLE )
          /* use lower value */
          l_tot = (l_tot < l_lpmRet) ? l_tot : l_lpmRet;
      }
    }
  }

  if( i >= LMP_MODULES_MAX )
  {
    /* enter sleep mode */
    bsp_sleepEnter(l_tot);

    /* Woke up. Now the timers must be adjusted. */
    l_ret = bsp_sleepDuration();
    bsp_adjustTick(l_ret);
  }

  return l_ret;
} /* lpm_entry() */


/*=============================================================================
 *  lpm_register()
 *============================================================================*/
int8_t lpm_register( pf_lpm_check pf_check )
{
  int i_ret = LPM_ERROR;
  int i = 0;

  if( pf_check != NULL )
  {
    for( i = 0; i < LMP_MODULES_MAX; i++ )
    {
      if( gs_lpm_entry[i].uc_free == 1 )
        break;
    }

    if( i < LMP_MODULES_MAX )
    {
      gs_lpm_entry[i].uc_free = 0;
      gs_lpm_entry[i].pf_check = pf_check;
      i_ret = i;
    }
  }
  return i_ret;
} /* lpm_register() */


/*=============================================================================
 *  lpm_unregister()
 *============================================================================*/
int8_t lpm_unregister( int8_t c_id )
{
  int i_ret = LPM_ERROR;;

  if( c_id < LMP_MODULES_MAX )
  {
    gs_lpm_entry[c_id].uc_free = 1;
    gs_lpm_entry[c_id].pf_check = NULL;
    i_ret = LPM_SUCCESS;
  }
  return i_ret;
} /* lpm_unregister() */

#ifdef __cplusplus
}
#endif

#endif /* #if (HAL_SUPPORT_MCU_SLEEP == TRUE) */
