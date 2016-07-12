/*
 * thrd_network_data_tlv.h
 *
 *  Created on: 12 Jul 2016
 * Author: Lukas Zimmermann <lzimmer1@stud.hs-offenburg.de>
 *
 * Thread Network Data Message Format.
 */

#ifndef EMB6_INC_NET_RIP_THRD_NETWORK_DATA_TLV_H_
#define EMB6_INC_NET_RIP_THRD_NETWORK_DATA_TLV_H_

#include "uip.h"

#define MAX_HAS_ROUTE_TLV_SIZE			5		// Maximum number of Has Route TLV entries.
#define MAX_PREFIX_TLV_DATA_SIZE		5		// Maximum number of data entries in Prefix TLV (Prefix + Sub-TLVs).
#define MAX_BORDER_ROUTER_TLV_SIZE		5		// Maximum number of Border Router TLV entries.
#define MAX_SERVICE_TLV_SIZE			5		// Maximum number of Service TLV entries (S_data).

/**
 * Network Layer TLV types.
 */
typedef enum
{
	NET_DATA_MSG_HAS_ROUTE_TLV = 0,
	NET_DATA_MSG_PREFIX_TLV = 1,
	NET_DATA_MSG_BORDER_ROUTER_TLV = 2,
	NET_DATA_MSG_SICSLOWPAN_ID_TLV = 3,
	NET_DATA_MSG_COM_DATA_TLV = 4,
	NET_DATA_MSG_SERVICE_TLV = 5,
	NET_DATA_MSG_SERVER_TLV = 6,
} net_data_msg_tlv_type_t;

/**
 * Has Route TLV.
 */
typedef struct __attribute__((__packed__)) {
	uint32_t data[MAX_HAS_ROUTE_TLV_SIZE];
}  net_data_msg_has_route_tlv_t;

/**
 * Prefix TLV.
 */
typedef struct __attribute__((__packed__)) {
	uint8_t domain_id;
	uint8_t prefix_length;
	uint8_t data[MAX_PREFIX_TLV_DATA_SIZE];
}  net_data_msg_prefix_tlv_t;

/**
 * Border Router TLV.
 */
typedef struct __attribute__((__packed__)) {
	uint32_t data[MAX_BORDER_ROUTER_TLV_SIZE];
}  net_data_msg_border_router_tlv_t;

/**
 * 6LoWPAN ID TLV.
 */
typedef struct __attribute__((__packed__)) {
	uint8_t cid;
	uint8_t contect_length;
}  net_data_msg_sicslowpan_id_tlv_t;

/**
 * Commissioning Data TLV.
 */
typedef struct __attribute__((__packed__)) {
	uint8_t COM_length;
	uint8_t COM_data;
}  net_data_msg_com_data_tlv_t;

/**
 * Service TLV.
 */
typedef struct __attribute__((__packed__)) {
	uint8_t S_id;
	uint32_t S_enterprise_number;
	uint8_t S_data_length;
	uint8_t data[MAX_SERVICE_TLV_SIZE];
}  net_data_msg_service_tlv_t;

/**
 * Server TLV.
 */
typedef struct __attribute__((__packed__)) {
	uint16_t S_server_16;
	uint8_t S_server_data[6];
}  net_data_msg_server_tlv_t;

#endif /* EMB6_INC_NET_RIP_THRD_NETWORK_DATA_TLV_H_ */
