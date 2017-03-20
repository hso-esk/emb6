/**
 * \file thrd-res-ae.c
 * \author Institute for reliable Embedded Systems and Communication Electronics
 * \date 2016/06/10
 * \version 1.0
 *
 * \brief CoAP resource - (proactive) address notification /a/ae
 */

#include "emb6.h"
#include "bsp.h"
#include "er-coap.h"
#include "tlv.h"
#include "net_tlv.h"
#include "thrd-addr-query.h"
#include "thrd-eid-rloc.h"

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
static net_tlv_ml_eid_t *ml_eid_tlv;

/**
 * (Proactive) Address Notification CoAP Resource (/a/an).
 */
RESOURCE(thrd_res_a_ae,
         "title=\"Address Query: POST\";rt=\"Text\"",
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
	LOG_RAW("========================== CoAP ==========================\n\r");
	LOG_RAW("Receiving CoAP packet (Res: a/ae)!\n\r");

	if ( (len = coap_get_payload(request, &chunk)) >= 32 ) {
		// TODO Process payload -> Receipt of Address Query Request.
		tlv = (tlv_t*) &chunk[0];
		if ( tlv->type == NET_TLV_TARGET_EID && tlv->length == 16 ) {
			target_eid_tlv = (net_tlv_target_eid_t*) tlv->value;
			LOG_RAW("Target EID = ");
			LOG_IP6ADDR(&target_eid_tlv->target_eid);
			LOG_RAW("\n\r");
		}
		tlv = (tlv_t*) &chunk[18];
		if ( tlv->type == NET_TLV_ML_EID && tlv->length == 8 ) {
			ml_eid_tlv = (net_tlv_ml_eid_t*) tlv->value;
			LOG_RAW("ML-EID = ");
			print_ml_eid(ml_eid_tlv);
		}
		// Check Target EID.
		thrd_local_addr_t *local_addr;
		// TODO Also lookup RFD Child's addresses.
		local_addr = thrd_local_addr_lookup(target_eid_tlv->target_eid);
		if ( local_addr != NULL ) {
			thrd_local_addr_rm(local_addr);
		}
	}
	LOG_RAW("==========================================================\n\r");
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
			LOG_RAW("\n\r");
	}
}

