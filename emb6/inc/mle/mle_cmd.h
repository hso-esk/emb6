#ifndef  __MLE_CMD_H_
#define  __MLE_CMD_H_

/*==============================================================================
                                 INCLUDE FILES
 =============================================================================*/
#include "emb6.h"
#include "tlv.h"




/*==============================================================================
                                     MACROS
 =============================================================================*/


#define MAX_TLV_DATA_SIZE         255


/*==============================================================================
									TYPEDEFS
 =============================================================================*/

typedef enum 
{
	LINK_REQUEST,
	LINK_ACCEPT,
	LINK_ACCEPT_AND_REQUEST,
	LINK_REJECT,
	ADVERTISEMENT,
	UPDATE,                    // Not used in Thread Network
	UPDATE_REQUEST,            // Not used in Thread Network
	DATA_REQUEST,
	DATA_RESPONSE,
	PARENT_REQUEST,
	PARENT_RESPONSE,
	CHILD_ID_REQUEST,
	CHILD_ID_RESPONSE,
	CHILD_UPDATE,
	CHILD_UPDATE_RESPONSE,
	CMD_NONE,
} mle_cmd_type_t;


typedef struct __attribute__((__packed__)) {
	uint8_t   		   type;                                  /**< command type */
	uint8_t            tlv_data[MAX_TLV_DATA_SIZE];           /**< tlv buffer */
	uint8_t            used_data;
}mle_cmd_t ;

/*==============================================================================
                                 API FUNCTIONS
 =============================================================================*/



/**
 * @brief  initialize mle command
 *
 * @param  mle_cmd	  pointer to mle_cmd structure
 * @param  type	      type of  mle command
 */

void  mle_init_cmd(mle_cmd_t* mle_cmd , const mle_cmd_type_t type );


/**
 * @brief  create  mle command from buffer
 *
 * @param  mle_cmd	  pointer to mle_cmd structure
 * @param  data	      pointer to the data
 * @param  datalen	  data length
 */
void  mle_create_cmd_from_buff(mle_cmd_t** cmd , uint8_t* data , uint16_t datalen);



/**
 * @brief  convert buffer to mle_cmd structure

 * @param  ptr	      pointer to the buffer to convert.
 * @param  mle_cmd	      pointer to the mle_cmd structure to parse in

 * @return
        -  1 sucess
        -  0 error
 */
uint8_t mle_deserialize_cmd(const uint8_t* ptr , mle_cmd_t* mle_cmd  , int16_t cmd_length );

/**
 * @brief  convert mle_cmd structure to buffer
 *
 * @param  mle_cmd	  mle_cmd structure to serialize
 * @param  ptr	      pointer to the buffer.

 * @return
         -  1 sucess
         -  0 error
 */
uint8_t mle_serialize_cmd(const mle_cmd_t mle_cmd , uint8_t* ptr , uint16_t* result_length  );


/**
 * @brief  add tlv to mle command
 *
 * @param  mle_cmd	  pointer to mle_cmd structure

 * @return
         -  1 sucess
         -  0 error
 */


uint8_t mle_add_tlv_to_cmd(mle_cmd_t * mle_cmd , const tlv_type_t type, const int8_t length,  uint8_t * value);





/**
 * @brief  print buffer
 *
 * @param  buffer	  pointer to buffer

 */
void print_buffer(uint8_t* buffer  ,uint8_t length);



/**
 * @brief  print mle command
 *
 * @param  mle_cmd	  mle_cmd structure

 * @return
         -  1 sucess
         -  0 error
 */
uint8_t mle_print_cmd(  mle_cmd_t mle_cmd );




/*==============================================================================
								LOCAL FUNCTION 
 =============================================================================*/











#endif /* __MLE_CMD_H_ */
