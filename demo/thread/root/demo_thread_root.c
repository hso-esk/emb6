/*
 * demo_mcast_root.c
 *
 *  Created on: 4 Apr 2016
 *      Author: osboxes
 */



#include "emb6.h"
#include "demo_thread_root.h"
#include "bsp.h"
#include "etimer.h"
#include "evproc.h"
#include "tcpip.h"
#include "rpl.h"
#include "udp-socket.h"
#include "mle_management.h"

#define		DEBUG		DEBUG_PRINT

#include "uip-debug.h"	// For debugging terminal output.

/*==============================================================================
                                         MACROS
 =============================================================================*/
#define     LOGGER_ENABLE        LOGGER_DEMO_MCAST
#include    "logger.h"


/*==============================================================================
                          LOCAL VARIABLE DECLARATIONS
 =============================================================================*/


/*==============================================================================
                               LOCAL FUNCTION PROTOTYPES
 =============================================================================*/



/*==============================================================================
                                    LOCAL FUNCTIONS
 =============================================================================*/




/*=============================================================================
                                         API FUNCTIONS
 ============================================================================*/

/*---------------------------------------------------------------------------*/
/*  demo_threadRootConf()                                                      */
/*---------------------------------------------------------------------------*/
uint8_t demo_threadRootConf(s_ns_t* pst_netStack)
{
	uint8_t c_ret = 1;

	/*
	 * By default stack
	 */
	if (pst_netStack != NULL) {
		if (!pst_netStack->c_configured) {

			pst_netStack->hc     = &sicslowpan_driver;
			pst_netStack->dllsec  = &nullsec_driver;
			pst_netStack->frame  = &framer_802154;
			pst_netStack->c_configured = 1;

		} else {
			if ((pst_netStack->hc == &sicslowpan_driver)   &&
					(pst_netStack->dllsec == &nullsec_driver)   &&
					(pst_netStack->frame == &framer_802154)) {
				/* right configuration */
			}
			else {
				pst_netStack = NULL;
				c_ret = 0;
			}
		}
	}

	return (c_ret);
}/* demo_threadRootConf */

/*---------------------------------------------------------------------------*/
/*    demo_threadRootInit()                                                    */
/*---------------------------------------------------------------------------*/
int8_t demo_threadRootInit(void)
{

	if(!mle_init()){return 0; }
	//mle_set_parent_mode();
	//mle_set_child_mode();
	PRINTF("Thread root demo initialized ... \n");


	return 1;
}/* demo_threadRootInit()  */

