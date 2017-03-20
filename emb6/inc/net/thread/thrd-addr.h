/**
 * \file thrd-addr.h
 * \author Institute for reliable Embedded Systems and Communication Electronics
 * \date 2016/07/25
 * \version 1.0
 *
 * \brief Thread IPv6 addressing tools
 */

#ifndef EMB6_INC_NET_RIP_THRD_ADDR_H_
#define EMB6_INC_NET_RIP_THRD_ADDR_H_

#define IPV6_UNIVERSAL_LOCAL_BIT				0x01
#define IPV6_UNIVERSAL_LOCAL_BIT_INVERT_MASK	0xfd

/**
 * Create Link-Local Prefix.
 */
// TODO Add Mesh-Local Prefix (e.g. 0xfd00).
#define thrd_create_linklocal_prefix(addr) do { 					\
		(addr)->u16[0] = UIP_HTONS(0xfe80);         \
		(addr)->u16[1] = 0x00;                     					\
		(addr)->u16[2] = 0x00;                      				\
		(addr)->u16[3] = 0x00;                      				\
} while(0)
/**
 * Create Mesh-Local Prefix.
 */
// TODO Add Mesh-Local Prefix (e.g. 0xfd00).
#define thrd_create_meshlocal_prefix(addr) do { 					\
		(addr)->u16[0] = UIP_HTONS(THRD_MESH_LOCAL_PREFIX);         \
		(addr)->u16[1] = 0x00;                     					\
		(addr)->u16[2] = 0x00;                      				\
		(addr)->u16[3] = 0x00;                      				\
} while(0)
/**
 * Create a random Interface Identifier (IID).
 */
#define thrd_create_random_iid(addr) do {			\
		(addr)->u16[4] = bsp_getrand(0, 0);			\
		(addr)->u16[5] = bsp_getrand(0, 0);			\
		(addr)->u16[6] = bsp_getrand(0, 0);			\
		(addr)->u16[7] = bsp_getrand(0, 0);			\
} while(0)
/**
 * Check whether the given IPv6 address's IID is valid (Not allowed: 0000:00ff:fe00:xxxx).
 */
#define thrd_ml_eid_iid_is_valid(addr) (( (addr)->u16[4] == 0x0000 && (addr)->u16[5] == 0x00ff && (addr)->u16[6] == 0xfe00 ) ? FALSE : TRUE)
/**
 * Create IID of a given RLOC address.
 */
#define thrd_create_rloc_iid(addr, rloc16) do {		\
		(addr)->u16[4] = UIP_HTONS(0x0000);			\
		(addr)->u16[5] = UIP_HTONS(0x00ff);			\
		(addr)->u16[6] = UIP_HTONS(0xfe00);			\
		(addr)->u16[7] = UIP_HTONS(rloc16);			\
} while(0)
/**
 * Create IID of a EUI-64 Bit IPv6 Address.
 */
#define thrd_create_eui_64_bit_iid(addr, mac) do {		\
		(addr)->u8[8] = mac[0];							\
		(addr)->u8[8] |= IPV6_UNIVERSAL_LOCAL_BIT << 1;	\
		(addr)->u8[9] = mac[1];							\
		(addr)->u8[10] = mac[2];						\
		(addr)->u8[11] = mac[3];						\
		(addr)->u8[12] = mac[4];						\
		(addr)->u8[13] = mac[5];						\
		(addr)->u8[14] = mac[6];						\
		(addr)->u8[15] = mac[7];						\
} while(0)

/**
 * Check whether the given RLOC16 belongs to a router (child id == 0).
 * @param rloc16 A RLOC16.
 * @retval TRUE If the given RLOC16 belongs to a router.
 * @retval FALSE If the given RLOC16 does not belong to a router.
 */
bool thrd_rloc16_belongs_to_router(uint16_t rloc16);

/**
 * Check whether the given IPv6 address is a Link-Local RLOC address.
 * @param addr A IPv6 address,
 * @retval TRUE  If the given IPv6 address is a Link-Local RLOC address.
 * @retval FALSE If the given IPv6 address is not a Link-Local RLOC address.
 */
extern bool thrd_is_addr_ll_rloc(uip_ipaddr_t *addr);

/**
 * Check whether the given IPv6 address is a Mesh-Local RLOC address.
 * @param addr A IPv6 address.
 * @retval TRUE If the given IPv6 address is a Mesh-Local RLOC address.
 * @retval FALSE If the given IPv6 address is not a Mesh-Local RLOC address.
 */
extern bool thrd_is_addr_ml_rloc(uip_ipaddr_t *addr);

/**
 * Check whether the given IPv6 address is a RLOC address.
 * @param addr A IPv6 address.
 * @retval TRUE If the given IPv6 address is a RLOC address.
 * @retval FALSE If the given IPv6 address is not a RLOC address.
 */
extern bool thrd_is_rloc_addr(uip_ipaddr_t *addr);

/**
 * Invert the universal/local bit of a given EUI-64 based link address.
 */
void thrd_eui_64_invert_universal_local_bit(linkaddr_t *link_addr);

/**
 * Check whether the given Long Address (MAC) is from type RLOC.
 * @param lladdr A Long Address (MAC).
 * @retval TRUE If the given Long Address (MAC) is from type RLOC.
 * @retval FALSE If the given Long Address (MAC) is not from type RLOC.
 */
extern bool thrd_is_linkaddr_rloc(linkaddr_t *link_addr);

/**
 * Extract the Router ID from a Thread IPv6 RLOC address.
 * @param rloc_addr A RLOC address.
 * @return The extracted Router ID.
 */
extern uint8_t thrd_extract_router_id_from_rloc_addr(uip_ipaddr_t *rloc_addr);

/**
 * Extract the child ID from a Thread IPv6 RLOC address.
 * @param rloc_addr A RLOC address.
 * @return The extracted child ID.
 */
extern uint16_t thrd_extract_child_id_from_rloc_addr(uip_ipaddr_t *rloc_addr);

/**
 * Extract the Router ID from a Long Address (MAC) of type RLOC.
 * @param lladdr The Long Address (MAC) of type RLOC.
 * @return The extracted Router ID.
 */
extern uint8_t thrd_extract_router_id_from_rloc_linkaddr(linkaddr_t *link_addr);

/**
 * Extract the RLOC16 from a Thread IPv6 address (htons).
 * @return The RLOC16 (htons).
 */
uint16_t thrd_extract_rloc16_from_rloc_address_htons(uip_ipaddr_t *rloc_addr);

/**
 * Extract the RLOC16 from a Thread IPv6 RLOC address.
 * @return The RLOC16.
 */
extern uint16_t thrd_extract_rloc16_from_rloc_address(uip_ipaddr_t *rloc_addr);

/**
 * Extract the RLOC16 from a Long Address (MAC) of type RLOC.
 * @param lladdr The Long Address (MAC) of type RLOC.
 * @return The RLOC16.
 */
extern uint16_t thrd_extract_rloc16_from_rloc_linkaddr(linkaddr_t *link_addr);

/**
 * Check whether the given RLOC16 is valid (Router ID within [0..62].
 * @param rloc16 A RLOC16.
 * @retval TRUE If the given RLOC16 is valid.
 * @retval FALSE If the given RLOC16 is invalid (==63).
 */
extern bool thrd_is_rloc16_valid(uint16_t rloc16);

/**
 * Check whether the given router ID is valid (Router ID within [0..62].
 * @param router_id A router ID.
 * @retval TRUE If the given router ID is valid.
 * @retval FALSE If the given router ID is invalid (>=63).
 */
extern bool thrd_is_rid_valid(uint8_t router_id);

/**
 * Create the next hop address for based on the RLOC16.
 * @param addr A pointer to the IPv6 address.
 * @param rloc16 The RLOC16.
 */
extern void thrd_create_next_hop_addr(uip_ipaddr_t *addr, uint8_t rloc16);

/**
 * Create RLOC based link layer address.
 * @param link_addr A link layer addr.
 * @param router_id The router id.
 * @param child_id The child id.
 */
extern void thrd_create_rloc_linkaddr(linkaddr_t *link_addr, uint8_t router_id, uint8_t child_id);

/**
 * Invert the universal/local bit of a MAC Extended Address based IID.
 * @param link_addr The MAC Extended Address based IID.
 */
extern void thrd_invert_universal_local_bit(uip_lladdr_t *lladdr);

#endif /* EMB6_INC_NET_RIP_THRD_ADDR_H_ */
