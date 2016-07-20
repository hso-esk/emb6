/*
 * thrd-network-data-leader.c
 *
 * Created on: 13 Jul 2016
 * Author: Lukas Zimmermann <lzimmer1@stud.hs-offenburg.de>
 *
 * Thread Network Data managed by the Thread Leader.
 */

/*
 ********************************************************************************
 *                                   INCLUDES
 ********************************************************************************
 */

#include "emb6.h"
#include "stdlib.h"
#include "bsp.h"
#include "clist.h"
#include "memb.h"
#include "rip.h"
#include "thread_conf.h"

#include "er-coap.h"
#include "er-coap-engine.h"
#include "rest-engine.h"

#include "thrd-network-data.h"
#include "thrd-network-data-leader.h"

#define DEBUG DEBUG_PRINT
#include "uip-debug.h"


/*
 ********************************************************************************
 *                          LOCAL FUNCTION DECLARATIONS
 ********************************************************************************
 */

static void coap_init();

/*
 ********************************************************************************
 *                               LOCAL VARIABLES
 ********************************************************************************
 */

/*
 ********************************************************************************
 *                               GLOBAL VARIABLES
 ********************************************************************************
 */

/*
 * CoAP Resources to be activated need to be imported through the extern keyword.
 */
extern resource_t thrd_res_n_sd;

/*
 ********************************************************************************
 *                           LOCAL FUNCTION DEFINITIONS
 ********************************************************************************
 */

static void
coap_init()
{
	PRINTF("Starting Thread Network Data (CoAP).\n\r");
	/* Receives all CoAP messages */
	// coap_init_engine();
	/* Initialize the REST engine. */
	// rest_init_engine();

	// Bind the resources to their Uri-Path.
	rest_activate_resource(&thrd_res_n_sd, "n/sd");
}

/* --------------------------------------------------------------------------- */

/*
 ********************************************************************************
 *                           API FUNCTION DEFINITIONS
 ********************************************************************************
 */

void
thrd_leader_network_data_init()
{
	coap_init();
}

/* --------------------------------------------------------------------------- */

void
thrd_leader_network_data_reset(void)
{

}

/* --------------------------------------------------------------------------- */

/*
 ********************************************************************************
 *                               END OF FILE
 ********************************************************************************
 */
