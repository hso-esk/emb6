/*
 * thrd-res-an.c
 *
 *  Created on: 7 Jun 2016
 *  Author: Lukas Zimmermann <lzimmer1@stud.hs-offenburg.de>
 *  CoAP Resource - (Proactive) Address Notification /a/an.
 */


#include "emb6.h"
#include "bsp.h"
#include "er-coap.h"
#include "tlv.h"
#include "net_tlv.h"

#include "thrd-addr-query.h"
#include "thrd-eid-rloc.h"
#include "thrd-iface.h"

#define     LOGGER_ENABLE                 LOGGER_THRD_NET
#include    "logger.h"

/*
 ********************************************************************************
 *                          LOCAL FUNCTION DECLARATIONS
 ********************************************************************************
 */

static void res_post_handler(void *request, void *response, uint8_t *buffer, uint16_t preferred_size, int32_t *offset);

static void print_ml_eid(net_tlv_ml_eid_t *ml_eid_tlv);

/*
 ********************************************************************************
 *                               LOCAL VARIABLES
 ********************************************************************************
 */

static size_t len = 0;						// CoAP payload length.
static tlv_t *tlv;
static net_tlv_target_eid_t *target_eid_tlv;
static net_tlv_rloc16_t *rloc16_tlv;
static net_tlv_ml_eid_t *ml_eid_tlv;
static net_tlv_last_transaction_t *last_transaction_tlv;

/**
 * (Proactive) Address Notification CoAP Resource (/a/an).
 */
RESOURCE(thrd_res_a_an,
		"title=\"Address Notification: POST\";rt=\"Text\"",
		NULL,
		res_post_handler,
		NULL,
		NULL);

/*
 ********************************************************************************
 *                               GLOBAL VARIABLES
 ********************************************************************************
 */

/*
 ********************************************************************************
 *                           LOCAL FUNCTION DEFINITIONS
 ********************************************************************************
 */

static void
res_post_handler(void *request, void *response, uint8_t *buffer, uint16_t preferred_size, int32_t *offset)
{
	const uint8_t *chunk;
	LOG_RAW("========================== CoAP ==========================\n");
	LOG_RAW("Receiving CoAP packet! (Res: a/an)\n");

	if ( (len = coap_get_payload(request, &chunk)) >= 32 ) {
		tlv = (tlv_t*) &chunk[0];
		if ( tlv->type == NET_TLV_TARGET_EID && tlv->length == 16 ) {
			target_eid_tlv = (net_tlv_target_eid_t*) tlv->value;
			LOG_RAW("Target EID = ");
			PRINT6ADDR(&target_eid_tlv->target_eid);
			LOG_RAW("\n");
		}
		tlv = (tlv_t*) &chunk[18];
		if ( tlv->type == NET_TLV_RLOC16 && tlv->length == 2 ) {
			rloc16_tlv = (net_tlv_rloc16_t*) tlv->value;
			LOG_RAW("RLOC16 = %04x \n", rloc16_tlv->rloc16);
		}
		tlv = (tlv_t*) &chunk[22];
		if ( tlv->type == NET_TLV_ML_EID && tlv->length == 8 ) {
			ml_eid_tlv = (net_tlv_ml_eid_t*) tlv->value;
			LOG_RAW("ML-EID = ");
			print_ml_eid(ml_eid_tlv);
		}
		if ( len == 38 ) {
			tlv = (tlv_t*) &chunk[32];
			if ( tlv->type == NET_TLV_LAST_TRANSACTION_TIME && tlv->length == 4 ) {
				last_transaction_tlv = (net_tlv_last_transaction_t*) tlv->value;
				LOG_RAW("Last Transaction Time = %08x\n", last_transaction_tlv->last_transaction_time);
			}
		}
		// Receipt of Address Notification Messages.
		thrd_addr_qry_t *addr_qry;
		addr_qry = thrd_addr_qry_lookup(target_eid_tlv->target_eid);
		if ( addr_qry != NULL ) {
			// Check whether the response has been received within valid time.
			if ( ctimer_expired(&addr_qry->timer) == 0 ) {
				// Stop timer.
				ctimer_stop(&addr_qry->timer);
				addr_qry->AQ_Timeout = 0;
				addr_qry->AQ_Retry_Delay = 0;
				// Update EID-to-RLOC Set (Map Cache).
				uip_ipaddr_t rloc_addr;
				thrd_create_meshlocal_prefix(&rloc_addr);
				thrd_create_rloc_iid(&rloc_addr, rloc16_tlv->rloc16);
				thrd_eid_rloc_cache_update(target_eid_tlv->target_eid, rloc_addr);
				thrd_eid_rloc_cache_print();
			}
		}
	}
	LOG_RAW("==========================================================\n");
}

/* --------------------------------------------------------------------------- */

static void
print_ml_eid(net_tlv_ml_eid_t *ml_eid_tlv)
{
	for (uint8_t i = 0; i < 8; i++) {
		LOG_RAW("%02x", ml_eid_tlv->ml_eid[i]);
		if ( i < 7 && (i % 2) == 1 )
			LOG_RAW(":");
		if ( i == 7 )
			LOG_RAW("\n");
	}
}

/* --------------------------------------------------------------------------- */
