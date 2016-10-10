/*
 * thrd-a-as.c
 *
 *  Author: Lukas Zimmermann <lzimmer1@stud.hs-offenburg.de>
 *  CoAP Resource - Address Solicit Request /a/as.
 */

#include "emb6.h"
#include "bsp.h"
#include "er-coap.h"
#include "tlv.h"
#include "net_tlv.h"
#include "uip.h"

#include "thrd-partition.h"
#include "thrd-router-id.h"
#include "thrd-leader-db.h"
#include "thrd-route.h"

#define     LOGGER_ENABLE                 LOGGER_THRD_NET
#include    "logger.h"

/*
 ********************************************************************************
 *                          LOCAL FUNCTION DECLARATIONS
 ********************************************************************************
 */

static void res_post_handler(void *request, void *response, uint8_t *buffer,
		uint16_t preferred_size, int32_t *offset);

static size_t create_response_payload(uint8_t *buf, uint8_t status,
		uint16_t *rloc16, uint8_t *id_sequence, uint64_t *router_mask);

static void print_ml_eid(net_tlv_ml_eid_t *ml_eid_tlv);

/*
 ********************************************************************************
 *                               LOCAL VARIABLES
 ********************************************************************************
 */

static uint8_t payload_buf[18] = { 0 };

static size_t len = 0;						// CoAP payload length.
static tlv_t *tlv;
static net_tlv_ml_eid_t *ml_eid_tlv;
static net_tlv_rloc16_t *rloc16_tlv;

uint8_t mac_ext_addr_dummy[8] = { 0 };

/**
 * Address Query CoAP Resource (/a/aq).
 */
RESOURCE(thrd_res_a_as,
         "title=\"Address Solicit Request: POST\";rt=\"Text\"",
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
	LOG_RAW("Receiving CoAP packet! (Res: a/as)\n");

	if ( (len = coap_get_payload(request, &chunk)) >= 20 ) {
		tlv = (tlv_t*) &chunk[0];
		uint8_t router_id = 63;	// Invalid Router ID.
		if ( tlv->type == NET_TLV_ML_EID && tlv->length == 8 ) {
			ml_eid_tlv = (net_tlv_ml_eid_t*) tlv->value;
			LOG_RAW("ML-EID = ");
			print_ml_eid(ml_eid_tlv);
		}
		if ( len == 24 ) {
			tlv = (tlv_t*) &chunk[10];
			if ( tlv->type == NET_TLV_RLOC16 && tlv->length == 2 ) {
				rloc16_tlv = (net_tlv_rloc16_t*) tlv->value;
				LOG_RAW("RLOC16 = %04x \n", rloc16_tlv->rloc16);
				router_id = (uint8_t) (rloc16_tlv->rloc16 >> 10);
			}
		}
		// TODO Assign a Router ID to the REED if one is available.
		thrd_ldb_ida_t *ida;
		if ( router_id == 63 ) {
			ida = thrd_leader_assign_rid(NULL, mac_ext_addr_dummy);	// TODO Add IEEE 802.15.4 Extended Address.
		} else {
			ida = thrd_leader_assign_rid(&router_id, mac_ext_addr_dummy);	// TODO Add IEEE 802.15.4 Extended Address.
		}
		if ( ida != NULL ) {
			// Success.

			// Increment ID sequence number.
			thrd_partition.ID_sequence_number++;

			// Update router ID set.
			thrd_rdb_id_t *rid;
			rid = thrd_rdb_rid_add(ida->ID_id);
			if ( rid == NULL ) {
				// This should not happen.
				return;
			}

			uint64_t router_id_mask = thrd_create_router_id_mask();
			uint16_t rloc16 = THRD_CREATE_RLOC16(ida->ID_id, 0);
			LOG_RAW("rloc16 = %04x\n\r", rloc16);
			len = create_response_payload(payload_buf, THRD_ADDR_SOL_STATUS_SUCCESS, &rloc16, &thrd_partition.ID_sequence_number, &router_id_mask);
			LOG_RAW("len = %d bytes\n\r", len);
			REST.set_response_status(response, REST.status.CHANGED);
			REST.set_response_payload(response, payload_buf, len);
		} else {
			// Could not assign a Router ID.
			len = create_response_payload(payload_buf, THRD_ADDR_SOL_STATUS_FAIL, NULL, NULL, NULL);
			LOG_RAW("len = %d bytes\n\r", len);
			REST.set_response_status(response, REST.status.CHANGED);
			REST.set_response_payload(response, payload_buf, len);
		}
		thrd_ldb_print_leader_database();
	}
	LOG_RAW("==========================================================\n");
}

/* --------------------------------------------------------------------------- */

static size_t
create_response_payload(uint8_t *buf, uint8_t status, uint16_t *rloc16, uint8_t *id_sequence, uint64_t *router_mask)
{
	if ( buf != NULL ) {
		// Create Status TLV.
		buf[0] = NET_TLV_STATUS;
		buf[1] = 1;
		buf[2] = status;
		if ( rloc16 != NULL && id_sequence != NULL && router_mask != NULL ) {
			// Create RLOC16 TLV.
			buf[3] = NET_TLV_RLOC16;
			buf[4] = 2;
			memcpy(&buf[5], rloc16, 2);
			buf[7] = NET_TLV_ROUTER_MASK;
			buf[8] = 9;
			memcpy(&buf[9], id_sequence, 1);
			memcpy(&buf[10], router_mask, 8);
			return 18;
		}
		return 3;
	}
	return 0;
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
