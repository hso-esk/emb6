/*
 * demo_mcast_sink.c
 *
 *  Created on: 4 Apr 2016
 *      Author: osboxes
 */



#include "emb6.h"

#include "demo_thread_sink.h"

#include "bsp.h"
#include "evproc.h"
#include "tcpip.h"
#include "uip.h"
#include "rpl.h"
#include "udp-socket.h"
#include "mle_management.h"

#define		DEBUG		DEBUG_PRINT

#include "uip-debug.h"	// For debugging terminal output.


/*==============================================================================
                                         MACROS
 =============================================================================*/

#define     LOGGER_ENABLE       	LOGGER_DEMO_MCAST
#include    "logger.h"


/*==============================================================================
                          LOCAL VARIABLE DECLARATIONS
 =============================================================================*/




/*==============================================================================
                                    LOCAL FUNCTIONS
 =============================================================================*/




/*=============================================================================
                                         API FUNCTIONS
 ============================================================================*/

/*---------------------------------------------------------------------------*/
/*  demo_mcastSinkInit()                                                      */
/*---------------------------------------------------------------------------*/
uint8_t demo_threadSinkConf(s_ns_t* pst_netStack)
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
}/* demo_mcastSinkConf */

/*---------------------------------------------------------------------------*/
/*    demo_mcastInit()                                                    */
/*---------------------------------------------------------------------------*/
int8_t demo_threadSinkInit(void)
{

	if(!mle_init()){return 0; }

	PRINTF("Thread sink demo initialized ... \n");





	return 1;
}/* demo_mcastInit()  */

