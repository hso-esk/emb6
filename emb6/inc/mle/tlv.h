/*
 * Copyright (c) 2016 .... All rights reserved.
 *
 *
 *
 */
#ifndef  __TLV_H_
#define  __TLV_H_

/*==============================================================================
                                 INCLUDE FILES
 =============================================================================*/
#include "emb6.h"



/*==============================================================================
									TYPEDEFS
 =============================================================================*/

/* MLE TLV TYPE  */
typedef enum {
	TLV_SOURCE_ADDRESS,
	TLV_MODE,
	TLV_TIME_OUT,
	TLV_CHALLENGE,
	TLV_RESPONSE,
	TLV_LINK_LAYER_FRAME_COUNTER,
	TLV_LINK_QUALITY,  // Not used in Thread Network
	TLV_NETWORK_PARAMETER,  // Not used in Thread Network
	TLV_MLE_FRAME_COUNTER,
	TLV_ROUTER64,
	TLV_ADDRESS16,
	TLV_LEADER_DATA,
	TLV_NETWORK_DATA,
	TLV_TLV_REQUEST,
	TLV_SCAN_MASK,
	TLV_CONNECTIVITY,
	TLV_LINK_MARGIN,
	TLV_STATUS,
	TLV_VERSION,
	TLV_ADDRESS_REGISTRATION,
	TLV_NONE,
} tlv_type_t ;


typedef struct {
	tlv_type_t type;    // type
	int8_t length;   // length
	uint8_t * value; // pointer to the value
}tlv_t;

/*==============================================================================
								LOCAL FUNCTION
 =============================================================================*/

/**
 * @brief  write on tlv object
 *
 * @param  tlv	  			pointer to tlv object to write on
 * @param  type	  			type of tlv object
 * @param  length	 		length of the value
 * @param  value	 		pointer to the value to write on the value buffer
 * @param  ptr_to_buffer	pointer to buffer to write value (cause no malloc function used)
 *
 * @return
         -  1 sucess
         -  0 error
 */
int8_t tlv_write(tlv_t *tlv, tlv_type_t type, int8_t length, uint8_t * value, uint8_t * ptr_to_buffer);

/**
 * @brief  print a tlv object
 *
 * @param  tlv	  tlv opbject to print

 * @return
         -  1 sucess
         -  0 error
 */
int8_t tlv_print(tlv_t tlv);


/*==============================================================================
                                 API FUNCTIONS
 =============================================================================*/


#endif /* __TLV_H_ */
