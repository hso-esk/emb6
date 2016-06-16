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

/* --------------------------------------------------------------------------- */

/*==============================================================================
                                    DEBUG FUNCTIONS
 =============================================================================*/
