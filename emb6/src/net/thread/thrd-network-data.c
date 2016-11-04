/**
 * \file thrd-network-data.c
 * \author Institute for reliable Embedded Systems and Communication Electronics
 * \date 2016/07/11
 * \version 1.0
 *
 * \brief Thread network data manipulation
 */

/*
 ********************************************************************************
 *                                   INCLUDES
 ********************************************************************************
 */

#include "emb6.h"
#include "stdlib.h"
#include "bsp.h"
#include "clist.h"
#include "memb.h"
#include "rip.h"
#include "thread_conf.h"
#include "er-coap.h"
#include "er-coap-engine.h"
#include "rest-engine.h"
#include "thrd-network-data.h"

/*
 ********************************************************************************
 *                          LOCAL FUNCTION DECLARATIONS
 ********************************************************************************
 */

static uint8_t thrd_get_num_on_mesh_prefixes(void);
static uint8_t thrd_get_num_ext_routes(void);
static uint8_t thrd_get_num_sicslowpan_ctx_ids(void);
static uint8_t thrd_get_num_servers(void);

static thrd_on_mesh_prefix_set_t* thrd_on_mesh_prefix_set_head(void);
static thrd_ext_route_set_t* thrd_ext_route_set_head(void);
static thrd_sicslowpan_ctx_id_set_t* thrd_sicslowpan_ctx_id_set_head(void);
static thrd_server_set_t* thrd_server_set_head(void);

static thrd_on_mesh_prefix_set_t* thrd_on_mesh_prefix_set_next(thrd_on_mesh_prefix_set_t *r);
static thrd_ext_route_set_t* thrd_ext_route_set_next(thrd_ext_route_set_t *r);
static thrd_sicslowpan_ctx_id_set_t* thrd_sicslowpan_ctx_id_set_next(thrd_sicslowpan_ctx_id_set_t *r);
static thrd_server_set_t* thrd_server_set_next(thrd_server_set_t *r);

static thrd_on_mesh_prefix_set_t* thrd_on_mesh_prefix_set_lookup(uint8_t *on_mesh_prefix);
static thrd_ext_route_set_t* thrd_ext_route_set_lookup(uint8_t domain_id);
static thrd_sicslowpan_ctx_id_set_t* thrd_sicslowpan_ctx_id_set_lookup(uint8_t sicslowpan_ctx_id);
static thrd_server_set_t* thrd_server_set_lookup(uint16_t enterprise_number);

static thrd_on_mesh_prefix_set_t* thrd_on_mesh_prefix_set_add(uint8_t *prefix, uint8_t domain_id,
		uint16_t border_router_16, bool stable, bool slaac_preferred, bool slaac_valid, bool dhcp,
		bool configure, bool def, bool preference);
static thrd_ext_route_set_t* thrd_ext_route_set_add(uint8_t domain_id, uint16_t border_router_16,
		uint8_t *prefix, bool stable, uint8_t preference);
static thrd_sicslowpan_ctx_id_set_t* thrd_sicslowpan_ctx_id_set_add(uint8_t id, uint8_t *prefix,
		bool compress, bool stable);
static thrd_server_set_t* thrd_server_set_add(uint16_t enterprise_number, uint8_t *service_data,
		uint16_t server_16, uint8_t *server_data, bool stable, uint8_t id);

static void thrd_on_mesh_prefix_set_rm(thrd_on_mesh_prefix_set_t *pref);
static void thrd_ext_route_set_rm(thrd_ext_route_set_t *ext_route);
static void thrd_sicslowpan_ctx_id_set_rm(thrd_sicslowpan_ctx_id_set_t *ctx_id);
static void thrd_server_set_rm(thrd_server_set_t *server);

/*
 ********************************************************************************
 *                               LOCAL VARIABLES
 ********************************************************************************
 */

/**
 * On-Mesh Prefix Set memory allocation.
 */
LIST(on_mesh_prefix_list);
MEMB(on_mesh_prefix_memb, thrd_on_mesh_prefix_set_t, THRD_MAX_ON_MESH_PREFIX_SET_ENTRIES);

/**
 * External Route Set memory allocation.
 */
LIST(ext_route_list);
MEMB(ext_route_memb, thrd_ext_route_set_t, THRD_MAX_EXT_ROUTE_SET_ENTRIES);

/**
 * 6LoWPAN Context ID Set memory allocation.
 */
LIST(sicslowpan_ctx_id_list);
MEMB(sicslowpan_ctx_id_memb, thrd_sicslowpan_ctx_id_set_t, THRD_MAX_SICSLOWPAN_CTX_ID_SET_ENTRIES);

/**
 * Server Set memory allocation.
 */
LIST(server_list);
MEMB(server_memb, thrd_server_set_t, THRD_MAX_SERVER_SET_ENTRIES);

/* Number of currently stored On-Mesh Prefixes in the On-Mesh Prefix Set. */
static uint8_t num_on_mesh_prefixes = 0;
/* Number of currently stored External Routers in the External Route Set. */
static uint8_t num_ext_routes = 0;
/* Number of currently stored 6LoWPAN Context IDs in the 6LoWPAN Context ID Set. */
static uint8_t num_sicslowpan_ctx_ids = 0;
/* Number of currently stored Servers in the Server Set. */
static uint8_t num_servers = 0;

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

static uint8_t
thrd_get_num_on_mesh_prefixes(void)
{
	return num_on_mesh_prefixes;
}

/* --------------------------------------------------------------------------- */

static uint8_t
thrd_get_num_ext_routes(void)
{
	return num_ext_routes;
}

/* --------------------------------------------------------------------------- */

static uint8_t
thrd_get_num_sicslowpan_ctx_ids(void)
{
	return num_sicslowpan_ctx_ids;
}

/* --------------------------------------------------------------------------- */

static uint8_t
thrd_get_num_servers(void)
{
	return num_servers;
}

/* --------------------------------------------------------------------------- */

static thrd_on_mesh_prefix_set_t*
thrd_on_mesh_prefix_set_head(void)
{
	return list_head(on_mesh_prefix_list);
}

/* --------------------------------------------------------------------------- */

static thrd_ext_route_set_t
*thrd_ext_route_set_head(void)
{
	return list_head(ext_route_list);
}

/* --------------------------------------------------------------------------- */

static thrd_sicslowpan_ctx_id_set_t*
thrd_sicslowpan_ctx_id_set_head(void)
{
	return list_head(sicslowpan_ctx_id_list);
}

/* --------------------------------------------------------------------------- */

static thrd_server_set_t*
thrd_server_set_head(void)
{
	return list_head(server_list);
}

/* --------------------------------------------------------------------------- */

static thrd_on_mesh_prefix_set_t*
thrd_on_mesh_prefix_set_next(thrd_on_mesh_prefix_set_t *r)
{
	if ( r != NULL ) {
		thrd_on_mesh_prefix_set_t *n = list_item_next(r);
		return n;
	}
	return NULL;
}

/* --------------------------------------------------------------------------- */

static thrd_ext_route_set_t*
thrd_ext_route_set_next(thrd_ext_route_set_t *r)
{
	if ( r != NULL ) {
		thrd_ext_route_set_t *n = list_item_next(r);
		return n;
	}
	return NULL;
}

/* --------------------------------------------------------------------------- */

static thrd_sicslowpan_ctx_id_set_t*
thrd_sicslowpan_ctx_id_set_next(thrd_sicslowpan_ctx_id_set_t *r)
{
	if ( r != NULL ) {
		thrd_sicslowpan_ctx_id_set_t *n = list_item_next(r);
		return n;
	}
	return NULL;
}

/* --------------------------------------------------------------------------- */

static thrd_server_set_t*
thrd_server_set_next(thrd_server_set_t *r)
{
	if ( r != NULL ) {
		thrd_server_set_t *n = list_item_next(r);
		return n;
	}
	return NULL;
}

/* --------------------------------------------------------------------------- */

static thrd_on_mesh_prefix_set_t*
thrd_on_mesh_prefix_set_lookup(uint8_t *on_mesh_prefix)
{
	thrd_on_mesh_prefix_set_t *prefix;
	thrd_on_mesh_prefix_set_t *found_prefix;

	PRINTF("thrd_on_mesh_prefix_set_lookup: Looking up On-Mesh Prefix.\n\r");

	found_prefix = NULL;
	for ( prefix = thrd_on_mesh_prefix_set_head(); prefix != NULL; prefix = thrd_on_mesh_prefix_set_next(prefix) ) {
		if ( memcmp(on_mesh_prefix, prefix->P_prefix, 8) == 0 ) {
			found_prefix = prefix;
			break;
		}
	}
	if ( found_prefix != NULL ) {
		PRINTF("thrd_on_mesh_prefix_set_lookup: Found On-Mesh Prefix.\n\r");
	} else {
		PRINTF("thrd_on_mesh_prefix_set_lookup: No On-Mesh Prefix found.\n\r");
	}
	return found_prefix;
}

/* --------------------------------------------------------------------------- */

static thrd_ext_route_set_t*
thrd_ext_route_set_lookup(uint8_t domain_id)
{
	thrd_ext_route_set_t *ext_route;
	thrd_ext_route_set_t *found_ext_route;

	PRINTF("thrd_ext_route_set_lookup: Looking up External Route Set for Domain ID: ");
	PRINTF("%d.", domain_id);
	PRINTF("\n\r");

	found_ext_route = NULL;
	for ( ext_route = thrd_ext_route_set_head(); ext_route != NULL; ext_route = thrd_ext_route_set_next(ext_route) ) {
		if ( ext_route->R_domain_id == domain_id ) {
			found_ext_route = ext_route;
			break;
		}
	}
	if ( found_ext_route != NULL ) {
		PRINTF("thrd_ext_route_set_lookup: Found Domain ID: %d.\n\r", domain_id);
	} else {
		PRINTF("thrd_ext_route_set_lookup: No Domain ID found.\n\r");
	}
	return found_ext_route;
}

/* --------------------------------------------------------------------------- */

static thrd_sicslowpan_ctx_id_set_t*
thrd_sicslowpan_ctx_id_set_lookup(uint8_t sicslowpan_ctx_id)
{
	thrd_sicslowpan_ctx_id_set_t *ctx_id;
	thrd_sicslowpan_ctx_id_set_t *found_ctx_id;

	PRINTF("thrd_sicslowpan_ctx_id_set_lookup: Looking up 6LoWPAN Context ID: ");
	PRINTF("%d.", sicslowpan_ctx_id);
	PRINTF("\n\r");

	found_ctx_id = NULL;
	for ( ctx_id = thrd_sicslowpan_ctx_id_set_head(); ctx_id != NULL; ctx_id = thrd_sicslowpan_ctx_id_set_next(ctx_id) ) {
		if ( ctx_id->CID_id == sicslowpan_ctx_id ) {
			found_ctx_id = ctx_id;
			break;
		}
	}
	if ( found_ctx_id != NULL ) {
		PRINTF("thrd_sicslowpan_ctx_id_set_lookup: Found 6LoWPAN Context ID: %d.\n\r", sicslowpan_ctx_id);
	} else {
		PRINTF("thrd_sicslowpan_ctx_id_set_lookup: No 6LoWPAN Context ID found.\n\r");
	}
	return found_ctx_id;
}

/* --------------------------------------------------------------------------- */

static thrd_server_set_t*
thrd_server_set_lookup(uint16_t enterprise_number)
{
	thrd_server_set_t *server;
	thrd_server_set_t *found_server;

	PRINTF("thrd_server_set_lookup: Looking up Server Set.\n\r");

	found_server = NULL;
	for ( server = thrd_server_set_head(); server != NULL; server = thrd_server_set_next(server) ) {
		if ( server->S_enterprise_number == enterprise_number ) {
			found_server = server;
			break;
		}
	}
	if ( found_server != NULL ) {
		PRINTF("thrd_server_set_lookup: Found Server.\n\r");
	} else {
		PRINTF("thrd_server_set_lookup: No Server found.\n\r");
	}
	return found_server;
}

/* --------------------------------------------------------------------------- */

static thrd_on_mesh_prefix_set_t*
thrd_on_mesh_prefix_set_add(uint8_t *prefix, uint8_t domain_id,
		uint16_t border_router_16, bool stable, bool slaac_preferred,
		bool slaac_valid, bool dhcp, bool configure, bool def,
		bool preference)
{
	thrd_on_mesh_prefix_set_t *pref;
	pref = thrd_on_mesh_prefix_set_lookup(prefix);

	if ( pref == NULL ) {
		PRINTF("thrd_on_mesh_prefix_set_add: On-Mesh Prefix unknown.\n\r");

		if ( thrd_get_num_on_mesh_prefixes() == THRD_MAX_ON_MESH_PREFIX_SET_ENTRIES ) {
			/* Removing the last entry from the On-Mesh Prefix Set. */
			thrd_on_mesh_prefix_set_t *last;

			last = list_tail(on_mesh_prefix_list);
			thrd_on_mesh_prefix_set_rm(last);
		}

		/* Allocate a router id entry and populate it. */
		pref = memb_alloc(&on_mesh_prefix_memb);

		if ( pref == NULL ) {
			/* This should not happen, as we explicitly deallocated one
			 * link set entry above. */
			PRINTF("thrd_on_mesh_prefix_set_add: Could not allocate On-Mesh Prefix.\n\r");
			return NULL;
		}
		list_push(on_mesh_prefix_list, pref);

		memcpy(&(pref->P_prefix[0]), prefix, 8);
		pref->P_domain_id = domain_id;
		pref->P_border_router_16 = border_router_16;
		pref->P_stable = stable;
		pref->P_slaac_preferred = slaac_preferred;
		pref->P_slaac_valid = slaac_valid;
		pref->P_dhcp = dhcp;
		pref->P_configure = configure;
		pref->P_default = def;
		pref->P_preference = preference;
		PRINTF("thrd_on_mesh_prefix_set_add: Added On-Mesh Prefix.\n\r");
		num_on_mesh_prefixes++;
		PRINTF("thrd_on_mesh_prefix_set_add: num_on_mesh_prefixes %d\n\r", num_on_mesh_prefixes);
	} else {

		PRINTF(ANSI_COLOR_RED "thrd_on_mesh_prefix_set_add: On-Mesh Prefix is already known.\n\r");
		PRINTF("thrd_on_mesh_prefix_set_add: num_on_mesh_prefixes %d\n\r", num_on_mesh_prefixes);
		PRINTF("-----------------------------------------------------\n\r");
		return NULL;
	}
	PRINTF("-----------------------------------------------------\n\r");
	return pref;
}

/* --------------------------------------------------------------------------- */

static thrd_ext_route_set_t*
thrd_ext_route_set_add(uint8_t domain_id, uint16_t border_router_16,
		uint8_t *prefix, bool stable, uint8_t preference)
{
	thrd_ext_route_set_t *ext_route;
	ext_route = thrd_ext_route_set_lookup(domain_id);

	if ( ext_route == NULL ) {
		PRINTF("thrd_ext_route_set_add: External Route unknown.\n\r");

		if ( thrd_get_num_ext_routes() == THRD_MAX_EXT_ROUTE_SET_ENTRIES ) {
			/* Removing the last entry from the External Route Set. */
			thrd_ext_route_set_t *last;

			last = list_tail(ext_route_list);
			thrd_ext_route_set_rm(last);
		}

		/* Allocate a router id entry and populate it. */
		ext_route = memb_alloc(&ext_route_memb);

		if ( ext_route == NULL ) {
			/* This should not happen, as we explicitly deallocated one
			 * link set entry above. */
			PRINTF("thrd_ext_route_set_add: Could not allocate External Route.\n\r");
			return NULL;
		}
		list_push(ext_route_list, ext_route);

		ext_route->R_domain_id = domain_id;
		ext_route->R_border_router_16 = border_router_16;
		memcpy(&(ext_route->R_prefix[0]), prefix, 8);
		ext_route->R_stable = stable;
		ext_route->R_preference = preference;
		PRINTF("thrd_ext_route_set_add: Added External Route.\n\r");
		num_ext_routes++;
		PRINTF("thrd_ext_route_set_add: num_ext_routes %d\n\r", num_ext_routes);
	} else {

		PRINTF(ANSI_COLOR_RED "thrd_ext_route_set_add: External Route is already known.\n\r");
		PRINTF("thrd_ext_route_set_add: num_ext_routes %d\n\r", num_ext_routes);
		PRINTF("-----------------------------------------------------\n\r");
		return NULL;
	}
	PRINTF("-----------------------------------------------------\n\r");
	return ext_route;
}

/* --------------------------------------------------------------------------- */

static thrd_sicslowpan_ctx_id_set_t*
thrd_sicslowpan_ctx_id_set_add(uint8_t id, uint8_t *prefix, bool compress,
		bool stable)
{
	thrd_sicslowpan_ctx_id_set_t *ctx_id;
	ctx_id = thrd_sicslowpan_ctx_id_set_lookup(id);

	if ( ctx_id == NULL ) {
		PRINTF("thrd_sicslowpan_ctx_id_set_add: External Route unknown.\n\r");

		if ( thrd_get_num_sicslowpan_ctx_ids() == THRD_MAX_SICSLOWPAN_CTX_ID_SET_ENTRIES ) {
			/* Removing the last entry from the 6LoWPAN Context ID Set. */
			thrd_sicslowpan_ctx_id_set_t *last;

			last = list_tail(sicslowpan_ctx_id_list);
			thrd_sicslowpan_ctx_id_set_rm(last);
		}

		/* Allocate a router id entry and populate it. */
		ctx_id = memb_alloc(&sicslowpan_ctx_id_memb);

		if ( ctx_id == NULL ) {
			/* This should not happen, as we explicitly deallocated one
			 * link set entry above. */
			PRINTF("thrd_sicslowpan_ctx_id_set_add: Could not allocate 6LoWPAN Context ID.\n\r");
			return NULL;
		}
		list_push(ext_route_list, ctx_id);

		ctx_id->CID_id = id;
		memcpy(&(ctx_id->CID_prefix[0]), prefix, 8);
		ctx_id->CID_compress = compress;
		ctx_id->CID_stable = stable;
		PRINTF("thrd_sicslowpan_ctx_id_set_add: Added 6LoWPAN Context ID.\n\r");
		num_sicslowpan_ctx_ids++;
		PRINTF("thrd_sicslowpan_ctx_id_set_add: num_sicslowpan_ctx_ids %d\n\r", num_sicslowpan_ctx_ids);
	} else {

		PRINTF(ANSI_COLOR_RED "thrd_sicslowpan_ctx_id_set_add: 6LoWPAN Context ID is already known.\n\r");
		PRINTF("thrd_sicslowpan_ctx_id_set_add: num_sicslowpan_ctx_ids %d\n\r", num_sicslowpan_ctx_ids);
		PRINTF("-----------------------------------------------------\n\r");
		return NULL;
	}
	PRINTF("-----------------------------------------------------\n\r");
	return ctx_id;
}

/* --------------------------------------------------------------------------- */

static thrd_server_set_t*
thrd_server_set_add(uint16_t enterprise_number, uint8_t *service_data,
		uint16_t server_16, uint8_t *server_data, bool stable, uint8_t id)
{
	thrd_server_set_t *server;
	server = thrd_server_set_lookup(enterprise_number);

	if ( server == NULL ) {
		PRINTF("thrd_server_set_add: External Route unknown.\n\r");

		if ( thrd_get_num_servers() == THRD_MAX_SERVER_SET_ENTRIES ) {
			/* Removing the last entry from the 6LoWPAN Context ID Set. */
			thrd_server_set_t *last;

			last = list_tail(server_list);
			thrd_server_set_rm(last);
		}

		/* Allocate a router id entry and populate it. */
		server = memb_alloc(&server_memb);

		if ( server == NULL ) {
			/* This should not happen, as we explicitly deallocated one
			 * link set entry above. */
			PRINTF("thrd_server_set_add: Could not allocate 6LoWPAN Context ID.\n\r");
			return NULL;
		}
		list_push(ext_route_list, server);

		server->S_enterprise_number = enterprise_number;
		memcpy(&(server->S_service_data[0]), service_data, 8);
		server->S_server_16 = server_16;
		memcpy(&(server->S_server_data[0]), server_data, 1);		// TODO Length of server data?
		server->S_stable = stable;
		server->S_id = id;
		PRINTF("thrd_server_set_add: Added 6LoWPAN Context ID.\n\r");
		num_servers++;
		PRINTF("thrd_server_set_add: num_servers %d\n\r", num_servers);
	} else {

		PRINTF(ANSI_COLOR_RED "thrd_server_set_add: 6LoWPAN Context ID is already known.\n\r");
		PRINTF("thrd_server_set_add: num_servers %d\n\r", num_servers);
		PRINTF("-----------------------------------------------------\n\r");
		return NULL;
	}
	PRINTF("-----------------------------------------------------\n\r");
	return server;
}

/* --------------------------------------------------------------------------- */

static void
thrd_on_mesh_prefix_set_rm(thrd_on_mesh_prefix_set_t *pref)
{
	if ( pref != NULL ) {
		PRINTF("thrd_on_mesh_prefix_set_rm: Removing On-Mesh Prefix from On-Mesh Prefix Set.\n\r");
		list_remove(on_mesh_prefix_list, pref);
		memb_free(&on_mesh_prefix_memb, pref);
		num_on_mesh_prefixes--;
		PRINTF("thrd_on_mesh_prefix_set_rm: num_on_mesh_prefixes %d\n\r", num_on_mesh_prefixes);
	}
}

/* --------------------------------------------------------------------------- */

static void
thrd_ext_route_set_rm(thrd_ext_route_set_t *ext_route)
{
	if ( ext_route != NULL ) {
		PRINTF("thrd_ext_route_set_rm: Removing External Route from External Route Set.\n\r");
		list_remove(ext_route_list, ext_route);
		memb_free(&ext_route_memb, ext_route);
		num_ext_routes--;
		PRINTF("thrd_ext_route_set_rm: num_ext_routes %d\n\r", num_ext_routes);
	}
}

/* --------------------------------------------------------------------------- */

static void
thrd_sicslowpan_ctx_id_set_rm(thrd_sicslowpan_ctx_id_set_t *ctx_id)
{
	if ( ctx_id != NULL ) {
		PRINTF("thrd_sicslowpan_ctx_id_set_rm: Removing 6LoWPAN Context ID from 6LoWPAN Context ID Set.\n\r");
		list_remove(sicslowpan_ctx_id_list, ctx_id);
		memb_free(&sicslowpan_ctx_id_memb, ctx_id);
		num_sicslowpan_ctx_ids--;
		PRINTF("thrd_sicslowpan_ctx_id_set_rm: num_sicslowpan_ctx_ids %d\n\r", num_sicslowpan_ctx_ids);
	}
}

/* --------------------------------------------------------------------------- */

static void
thrd_server_set_rm(thrd_server_set_t *server)
{
	if ( server != NULL ) {
		PRINTF("thrd_server_set_rm: Removing Server from Server Set.\n\r");
		list_remove(server_list, server);
		memb_free(&server_memb, server);
		num_servers--;
		PRINTF("thrd_server_set_rm: num_servers %d\n\r", num_servers);
	}
}

/*
 ********************************************************************************
 *                           API FUNCTION DEFINITIONS
 ********************************************************************************
 */

void
thrd_network_data_init(void)
{
	PRINTF("Initializing Thread Network Data.\n\r");
	memb_init(&on_mesh_prefix_memb);
	list_init(on_mesh_prefix_list);

	memb_init(&ext_route_memb);
	list_init(ext_route_list);

	memb_init(&sicslowpan_ctx_id_memb);
	list_init(sicslowpan_ctx_id_list);

	memb_init(&server_memb);
	list_init(server_list);
}

/* --------------------------------------------------------------------------- */

void
thrd_network_data_getNetworkData(bool stable, uint8_t data, uint8_t len)
{

}

/* --------------------------------------------------------------------------- */

net_data_msg_border_router_tlv_t*
thrd_network_data_getBorderRouterTLV(net_data_msg_prefix_tlv_t *prefix_tlv)
{
	return NULL;
}

/* --------------------------------------------------------------------------- */

net_data_msg_has_route_tlv_t*
thrd_network_data_getHasRouteTLV(net_data_msg_prefix_tlv_t *prefix_tlv)
{
	return NULL;
}

/* --------------------------------------------------------------------------- */

net_data_msg_sicslowpan_id_tlv_t*
thrd_network_data_getContextIdTLV(net_data_msg_prefix_tlv_t *prefix_tlv)
{
	return NULL;
}

/* --------------------------------------------------------------------------- */

net_data_msg_prefix_tlv_t*
thrd_network_data_getPrefixTLV(uint8_t *prefix, uint8_t length)
{
	return NULL;
}

/* --------------------------------------------------------------------------- */

thrd_error_t
thrd_network_data_insert(uint8_t data, uint8_t length)
{
	return THRD_ERROR_NONE;
}

/* --------------------------------------------------------------------------- */

thrd_error_t
thrd_network_data_remove(uint8_t data, uint8_t length)
{
	return THRD_ERROR_NONE;
}

/* --------------------------------------------------------------------------- */

thrd_error_t
thrd_network_data_removeTempData(uint8_t data, uint8_t length)
{
	return THRD_ERROR_NONE;
}

/* --------------------------------------------------------------------------- */

/* --------------------------------------------------------------------------- */

/* --------------------------------------------------------------------------- */

/* --------------------------------------------------------------------------- */

/* --------------------------------------------------------------------------- */

/*
 ********************************************************************************
 *                               END OF FILE
 ********************************************************************************
 */
