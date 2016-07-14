/*
 * thrd-dev.c
 *
 *  Created on: 15 Jun 2016
 *  Author: Lukas Zimmermann <lzimmer1@stud.hs-offenburg.de>
 *  Thread Device.
 */

#include "thread_conf.h"
#include "thrd-dev.h"
#include "thrd-iface.h"
#include "thrd-leader-db.h"

#include "er-coap.h"
#include "er-coap-engine.h"
#include "rest-engine.h"

/** Thread default Device Type Configuration. */
thrd_dev_t thrd_dev = {
		.type = THRD_DEV_TYPE,
		.isFFD = THRD_DEV_FUNC,
};

/*==============================================================================
                          LOCAL VARIABLE DECLARATIONS
 =============================================================================*/

/*==============================================================================
                               LOCAL FUNCTION PROTOTYPES
 =============================================================================*/

/*==============================================================================
                                    LOCAL FUNCTIONS
 =============================================================================*/

/* --------------------------------------------------------------------------- */

/*==============================================================================
                                     API FUNCTIONS
 =============================================================================*/

void
thrd_dev_init(void)
{
#ifdef THRD_DEV_TYPE == THRD_DEV_TYPE_REED || THRD_DEV_TYPE == THRD_DEV_TYPE_ROUTER
	/* Receives all CoAP messages */
	coap_init_engine();
	/* Initialize the REST engine. */
	rest_init_engine();
#endif
}

/* --------------------------------------------------------------------------- */

/*==============================================================================
                                    DEBUG FUNCTIONS
 =============================================================================*/

