/*
 * net_tlv.h
 *
 *  Created on: 22 Apr 2016
 *  Author: Lukas Zimmermann <lzimmer1@stud.hs-offenburg.de>
 *
 *  Network Layer TLVs.
 */

#ifndef EMB6_INC_NET_RIP_NET_TLV_H_
#define EMB6_INC_NET_RIP_NET_TLV_H_

/**
 * Network Layer TLV types.
 */
typedef enum
{
	NET_TLV_TARGET_EID = 0,           //!< NET_TLV_TARGET_EID
	NET_TLV_RLOC16 = 2,               //!< NET_TLV_RLOC16
	NET_TLV_ML_EID = 3,               //!< NET_TLV_ML_EID
	NET_TLV_STATUS = 4,               //!< NET_TLV_STATUS
	NET_TLV_LAST_TRANSACTION_TIME = 6,//!< NET_TLV_LAST_TRANSACTION_TIME
	NET_TLV_ROUTER_MASK = 7,          //!< NET_TLV_ROUTER_MASK
	NET_TLV_NONE,				  	  //!< NET_TLV_NONE
} net_tlv_type_t;

/**
 * \brief Network Layer TLV.
 */
typedef struct __attribute__((__packed__)) {
	net_tlv_type_t type;    // type
	uint8_t length;   // length
	uint8_t *value; // pointer to the value
} net_tlv_t;

/**
 * Target EID TLV.
 */
typedef struct __attribute__((__packed__)) {
	uint8_t target_eid;
}  net_tlv_target_eid_t;

/**
 * RLOC16 TLV.
 */
typedef struct __attribute__((__packed__)) {
	uint8_t rloc16;
}  net_tlv_rloc16_t;

/**
 * ML-EID TLV.
 */
typedef struct __attribute__((__packed__)) {
	uint8_t ml_eid;
}  net_tlv_ml_eid_t;

/**
 * Status TLV.
 */
typedef struct __attribute__((__packed__)) {
	uint8_t status;
}  net_tlv_status_t;

/**
 * Last Transaction Time TLV.
 */
typedef struct __attribute__((__packed__)) {
	uint32_t last_transaction_time;
}  net_tlv_last_transaction_t   ;

/**
 * Router Mask TLV.
 */
typedef struct __attribute__((__packed__)) {
	uint64_t router_id_mask;
}  net_tlv_router_mask_t;

/*
 ********************************************************************************
 *                           LOCAL FUNCTION DEFINITIONS
 ********************************************************************************
 */

net_tlv_t *net_tlv_write(net_tlv_t *net_tlv, net_tlv_type_t type, uint8_t length, uint8_t *value,
		uint8_t *ptr_to_buffer);

#endif /* EMB6_INC_NET_RIP_NET_TLV_H_ */
