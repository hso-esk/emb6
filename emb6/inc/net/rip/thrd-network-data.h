/*
 * thrd-network-data.h
 *
 * Created on: 11 Jul 2016
 * Author: Lukas Zimmermann <lzimmer1@stud.hs-offenburg.de>
 *
 * Thread Network Data.
 */

#include "bsp.h"

#ifndef EMB6_INC_NET_RIP_THRD_NETWORK_DATA_H_
#define EMB6_INC_NET_RIP_THRD_NETWORK_DATA_H_

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

#endif /* EMB6_INC_NET_RIP_THRD_NETWORK_DATA_H_ */
