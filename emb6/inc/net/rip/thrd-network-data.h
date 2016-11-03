/**
 * \file thrd-network-data.h
 * \author Institute for reliable Embedded Systems and Communication Electronics
 * \date 2016/07/11
 * \version 1.0
 *
 * \brief Thread network data manipulation
 */

#include "bsp.h"
#include "thrd-network-data-tlv.h"

#ifndef EMB6_INC_NET_RIP_THRD_NETWORK_DATA_H_
#define EMB6_INC_NET_RIP_THRD_NETWORK_DATA_H_

/*
 ********************************************************************************
 *                               GLOBAL VARIABLES
 ********************************************************************************
 */

uint8_t networkDataBuffer[MAX_NETWORK_DATA_SIZE];		// Network Data Buffer.

/**
 * Version Number Set.
 */
typedef struct {
	uint8_t VN_version;
	uint8_t VN_stable_version;
	bool VN_stable_only;
} thrd_vn_number_set_t;

extern thrd_vn_number_set_t thrd_vn_number_set;

/**
 * On-Mesh Prefix Set.
 */
typedef struct thrd_on_mesh_prefix_set {
	struct thrd_on_mesh_prefix_set *next;

	uint8_t P_prefix[8];
	uint8_t P_domain_id;
	uint16_t P_border_router_16;
	bool P_stable;
	bool P_slaac_preferred;
	bool P_slaac_valid;
	bool P_dhcp;
	bool P_configure;
	bool P_default;
	uint8_t P_preference;
} thrd_on_mesh_prefix_set_t;

/**
 * External Router Set.
 */
typedef struct thrd_ext_route_set {
	struct thrd_ext_route_set *next;

	uint8_t R_domain_id;
	uint16_t R_border_router_16;
	uint8_t R_prefix[8];
	bool R_stable;
	uint8_t R_preference;
} thrd_ext_route_set_t;

/**
 * 6LoWPAN Context ID Set.
 */
typedef struct thrd_sicslowpan_ctx_id_set {
	struct thrd_sicslowpan_ctx_id_set *next;

	uint8_t CID_id;			// TODO Datatype?
	uint8_t CID_prefix[8];	// TODO Datatype?
	bool CID_compress;
	bool CID_stable;
} thrd_sicslowpan_ctx_id_set_t;

/**
 * Server Set.
 */
typedef struct thrd_server_set {
	struct thrd_server_set *next;

	uint16_t S_enterprise_number;	// TODO Datatype?
	uint8_t S_service_data[1];		// TODO Datatype? --> Defined in a later revision of this specification.
	uint16_t S_server_16;
	uint8_t S_server_data[1];		// TODO Datatype? --> Defined in a later revision of this specification.
	bool S_stable;
	uint8_t S_id;
} thrd_server_set_t;

/**
 * Commissioning Data Set.
 */
typedef struct {
	uint8_t COM_length;
	uint8_t COM_data[32];			// TODO Maximum length?
} thrd_com_data_set_t;

extern thrd_com_data_set_t thrd_com_data_set;

/*
 ********************************************************************************
 *                           API FUNCTION DEFINITIONS
 ********************************************************************************
 */

void thrd_network_data_init(void);

/**
 * Get a full or stable copy of the Thread Network Data.
 * @param stable	TRUE when copying the stable version, FALSE when copying the full version.
 * @param data		A pointer ro the data buffer.
 * @param len		On entry, size of the data buffer pointed to by @p data.
 * 					On extrit, number of copied bytes.
 */
void thrd_network_data_getNetworkData(bool stable, uint8_t data, uint8_t len);

/**
 * Get a pointer to the Border Router TLV within a given Prefix TLV.
 * @param prefix_tlv	A reference to the Prefrix TLV.
 * @return				A pointer to the Border Router TLV if one is found.
 * 						NULL else.
 */
net_data_msg_border_router_tlv_t* thrd_network_data_getBorderRouterTLV(net_data_msg_prefix_tlv_t *prefix_tlv);

/**
 * Get a pointer to the Has Router TLV within a given Prefix TLV.
 * @param prefix_tlv	A reference to the Prefix TLV.
 * @return				A pointer to the Has Route TLV if one found.
 * 						NULL else.
 */
net_data_msg_has_route_tlv_t* thrd_network_data_getHasRouteTLV(net_data_msg_prefix_tlv_t *prefix_tlv);

/**
 * Get a pointer to the 6LoWPAN Context ID TLV within a given Prefix TLV.
 * @param prefix_tlv	A reference to the Prefix TLV.
 * @return				A pointer to the 6LoWPAN Context ID TLV if one is found.
 * 						NULL else.
 */
net_data_msg_sicslowpan_id_tlv_t* thrd_network_data_getContextIdTLV(net_data_msg_prefix_tlv_t *prefix_tlv);

/**
 * Get a pointer to a Prefix TLV.
 * @param prefix	A pointer to an IPv6 prefix.
 * @param length	The prefix length pointed to by @p prefix.
 * @return
 */
net_data_msg_prefix_tlv_t* thrd_network_data_getPrefixTLV(uint8_t *prefix, uint8_t length);

/**
 * Insert bytes into the Network Data.
 * @param data		A pointer to the beginning of the insertion.
 * @param length	The number of bytes to insert.
 * @retval THRD_ERROR_NONE		Successfully inserted bytes.
 * @retval THRD_ERROR_NO_BUF	Insufficient space to insert bytes.
 */
thrd_error_t thrd_network_data_insert(uint8_t data, uint8_t length);

/**
 * Remove bytes from the Network Data.
 * @param data		A pointer to the beginning of the removal.
 * @param length	The number of bytes to remove.
 * @retval THRD_ERROR_NONE	Successfully removed bytes.
 */
thrd_error_t thrd_network_data_remove(uint8_t data, uint8_t length);

/**
 * Remove non-stable data from the Thread Network Data.
 * @param data		A pointer ro the Network Data to modify.
 * @param length	On entry, the size of the Network Data in bytes.
 * 					On exit, the size of the resulting Network Data in bytes.
 * @return
 */
thrd_error_t thrd_network_data_removeTempData(uint8_t data, uint8_t length);

#endif /* EMB6_INC_NET_RIP_THRD_NETWORK_DATA_H_ */
