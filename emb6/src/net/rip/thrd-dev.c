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

#include "thrd-iface.h"
#include "thrd-partition.h"

#define     LOGGER_ENABLE                 LOGGER_THRD_NET
#include    "logger.h"

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
#if (THRD_DEV_TYPE == THRD_DEV_TYPE_REED) || (THRD_DEV_TYPE == THRD_DEV_TYPE_ROUTER)
	/* Receives all CoAP messages */
	coap_init_engine();
	/* Initialize the REST engine. */
	rest_init_engine();
#endif
}

/* --------------------------------------------------------------------------- */

thrd_error_t
thrd_dev_set_type(thrd_dev_type_t type)
{
	switch(type) {
	case THRD_DEV_TYPE_NONE:
		thrd_dev.type = THRD_DEV_TYPE_NONE;
		break;
	case THRD_DEV_TYPE_END:
		thrd_dev.type = THRD_DEV_TYPE_END;
		break;
	case THRD_DEV_TYPE_REED:
		thrd_dev.type = THRD_DEV_TYPE_REED;
		break;
	case THRD_DEV_TYPE_ROUTER:
		thrd_dev.type = THRD_DEV_TYPE_ROUTER;
		break;
	case THRD_DEV_TYPE_LEADER:
		thrd_dev.type = THRD_DEV_TYPE_LEADER;
		break;
	default:
		thrd_dev.type = THRD_DEV_TYPE_NONE;
		return THRD_ERROR_INVALID_ARGS;
	}
	return THRD_ERROR_NONE;
}

/* --------------------------------------------------------------------------- */

/*==============================================================================
                                    DEBUG FUNCTIONS
 =============================================================================*/

void
thrd_dev_print_dev_info()
{
	LOG_RAW("|==================================== THREAD DEVICE ====================================|\n\r");
	LOG_RAW("| Device Type = ");
	switch(thrd_dev.type) {
	case THRD_DEV_TYPE_NONE:
		LOG_RAW("THRD_DEV_TYPE_NONE");
		break;
	case THRD_DEV_TYPE_END:
		LOG_RAW("THRD_DEV_TYPE_END");
		break;
	case THRD_DEV_TYPE_REED:
		LOG_RAW("THRD_DEV_TYPE_REED");
		break;
	case THRD_DEV_TYPE_ROUTER:
		LOG_RAW("THRD_DEV_TYPE_ROUTER");
		break;
	case THRD_DEV_TYPE_LEADER:
		LOG_RAW("THRD_DEV_TYPE_LEADER");
		break;
	default:
		LOG_RAW("THRD_DEV_TYPE_NONE");
		break;
	}
	LOG_RAW("\n");
	// Print interface.
	thrd_iface_print();
	// Print Thread Partition data.
	thrd_print_partition_data();
	LOG_RAW("|=======================================================================================|\n\r");
}

