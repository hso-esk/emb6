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
typedef struct {
	net_tlv_type_t type;    // type
	uint8_t length;   // length
	uint8_t *value; // pointer to the value
} net_tlv_t;

/*
 ********************************************************************************
 *                           LOCAL FUNCTION DEFINITIONS
 ********************************************************************************
 */

net_tlv_t *net_tlv_write(net_tlv_t *net_tlv, net_tlv_type_t type, uint8_t length, uint8_t *value,
		uint8_t *ptr_to_buffer);

#endif /* EMB6_INC_NET_RIP_NET_TLV_H_ */
