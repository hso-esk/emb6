/**
 * \file thrd-res-aq.c
 * \author Institute for reliable Embedded Systems and Communication Electronics
 * \date 2016/06/07
 * \version 1.0
 *
 * \brief CoAP resource - address query /a/aq
 */

#include "emb6.h"
#include "bsp.h"
#include "er-coap.h"
#include "tlv.h"
#include "net_tlv.h"
#include "uip.h"
#include "thrd-addr-query.h"

#define     LOGGER_ENABLE                 LOGGER_THRD_NET
#include    "logger.h"

/*
 ********************************************************************************
 *                          LOCAL FUNCTION DECLARATIONS
 ********************************************************************************
 */

static void res_post_handler(void *request, void *response, uint8_t *buffer,
    uint16_t preferred_size, int32_t *offset, void* p_user);

/*
 ********************************************************************************
 *                               LOCAL VARIABLES
 ********************************************************************************
 */

static size_t len = 0;						// CoAP payload length.
static tlv_t *tlv;
static net_tlv_target_eid_t *target_eid_tlv;

/**
 * Address Query CoAP Resource (/a/aq).
 */
RESOURCE(thrd_res_a_aq,
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
res_post_handler(void *request, void *response, uint8_t *buffer,
    uint16_t preferred_size, int32_t *offset, void* p_user)
{
	const uint8_t *chunk;
	LOG_RAW("========================== CoAP ==========================\n\r");
	LOG_RAW("Receiving CoAP packet (Res: a/aq)!\n");

	if ( (len = coap_get_payload(request, &chunk)) == 18 ) {
		tlv = (tlv_t*) chunk;
		if ( tlv->type == NET_TLV_TARGET_EID && tlv->length == 16 ) {
			target_eid_tlv = (net_tlv_target_eid_t*) tlv->value;
			LOG_RAW("Target EID = ");
			LOG_IP6ADDR(&target_eid_tlv->target_eid);
			LOG_RAW("\n\r");

			// Receipt of Address Query Messages.
			thrd_local_addr_t *local_addr;
			thrd_rfd_child_addr_t *rfd_addr;
			local_addr = thrd_local_addr_lookup(target_eid_tlv->target_eid);
			rfd_addr = thrd_rfd_child_addr_lookup(target_eid_tlv->target_eid);

			if ( local_addr != NULL || rfd_addr != NULL ) {
				// Sending Address Notification.
				uint16_t rloc16 = 0x1010;	// TODO Including the RLOC of itself.
				uint8_t ml_eid[8] = { 0xF0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0F };	// TODO
				clock_time_t time = 0x80000008;	// TODO
				// uip_ipaddr_t addr;
				uip_ipaddr_t addr = UIP_IP_BUF->srcipaddr;
				LOG_RAW("Responding to Address Query Request received from IP Address: ");
				LOG_IP6ADDR(&addr);
				LOG_RAW("\n\r");
				thrd_addr_ntf_response(&addr, &target_eid_tlv->target_eid, &rloc16, ml_eid, &time);
			}
		}
	}
	LOG_RAW("==========================================================\n\r");
}

/* --------------------------------------------------------------------------- */
