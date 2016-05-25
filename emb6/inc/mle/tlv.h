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
#define		DEBUG		DEBUG_PRINT
#include "uip-debug.h"	// For debugging terminal output.


/*==============================================================================
									TYPEDEFS
 =============================================================================*/

/* MLE TLV TYPE  */
typedef enum  {
	TLV_SOURCE_ADDRESS,
	TLV_MODE,
	TLV_TIME_OUT,
	TLV_CHALLENGE,
	TLV_RESPONSE,
	TLV_LINK_LAYER_FRAME_COUNTER,
	TLV_LINK_QUALITY,  // Not used in Thread Network
	TLV_NETWORK_PARAMETER,  // Not used in Thread Network
	TLV_MLE_FRAME_COUNTER,
	TLV_ROUTE64,
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
}  tlv_type_t  ;


typedef struct __attribute__((__packed__)) {
	uint8_t  type    ;    // type
	uint8_t	length  ;   // length
	uint8_t  value[1]   ; // pointer to the value
}  tlv_t   ;


typedef struct __attribute__((__packed__)) {
	uint8_t  id_seq    ;
	uint64_t router_id_mask  ;
	uint8_t  lq[1]   ;
}  tlv_route64_t   ;


typedef struct __attribute__((__packed__)) {
	uint32_t  partition_id  ;
	uint8_t	  weight  ;
	uint8_t   data_version  ;
	uint8_t   stable_data_version  ;
	uint8_t   leader_router_id  ;
}  tlv_leader_t   ;


typedef struct __attribute__((__packed__)) {
	uint8_t   maxChild  ;
	uint8_t	  child_count  ;
	uint8_t   LQ3  ;
	uint8_t   LQ2  ;
	uint8_t   LQ1  ;
	uint8_t   LeaderCost  ;
	uint8_t   id_seq  ;
}  tlv_connectivity_t   ;

/*==============================================================================
								API FUNCTION
 =============================================================================*/

int8_t tlv_init(tlv_t **tlv, uint8_t * ptr );

int8_t tlv_leader_init(tlv_leader_t **tlv, uint8_t * ptr );

int8_t tlv_route64_init(tlv_route64_t **tlv, uint8_t * ptr );

int8_t tlv_connectivity_init(tlv_connectivity_t **tlv, uint8_t * ptr );


/**
 * @brief  write on tlv object
 *
 * @param  tlv	  			pointer to tlv object to write on
 * @param  type	  			type of tlv object
 * @param  length	 		length of the value
 * @param  value	 		pointer to the value
 *
 * @return
         -  1 sucess
         -  0 error
 */
int8_t tlv_write(tlv_t *tlv , tlv_type_t type, int8_t length, uint8_t * value );

/**
 * @brief  print a tlv object
 *
 * @param  tlv	  tlv opbject to print

 * @return
         -  1 sucess
         -  0 error
 */
int8_t tlv_print(tlv_t * tlv);

/**
 * @brief  find tlv type in buffer
 *
 * @param  buf	          buffer
 * @param  buf_length	  buffer length
 * @param  type	  		  type of tlv to looking for
*
 * @return pointer to the tlv if exist
 *         otherwise NULL
 */
tlv_t* tlv_find(uint8_t * buf, uint8_t buf_length, const tlv_type_t type);

/*==============================================================================
                                 useful macros
 =============================================================================*/

/* a=variable, b=bit number */
#define BIT_SET(a,b) ((a) |= (1<<(b)))
#define BIT_RESET(a,b) ((a) &= ~(1<<(b)))
#define BIT_CHECK(a,b) ((a) & (1<<(b)))

#endif /* __TLV_H_ */
