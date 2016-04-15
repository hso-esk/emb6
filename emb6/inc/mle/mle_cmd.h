#ifndef  __MLE_CMD_H_
#define  __MLE_CMD_H_

/*==============================================================================
                                 INCLUDE FILES
 =============================================================================*/
 #include "emb6.h"




/*==============================================================================
                                     MACROS
 =============================================================================*/

#define MAX_TLV_BUFFER_SIZE                 50


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
} mle_cmd_type_t;


typedef struct {
mle_cmd_type_t     type;                               /**< command type */
uint8_t            TLV_Buffer[MAX_TLV_BUFFER_SIZE];        /**< tlv buffer contain a series of TLV */
uint8_t            command_length ;                   /**< or tlv_counter (to think about ) */
}mle_cmd_t ;

/*==============================================================================
                                 API FUNCTIONS
 =============================================================================*/

 
 uint8_t mle_build_cmd( mle_cmd_type_t  cmd_type , mle_cmd_t* mle_cmd);
 uint8_t mle_parse_in_cmd(uint8_t* ptr , mle_cmd_t* mle_cmd  );

/*==============================================================================
								LOCAL FUNCTION 
 =============================================================================*/
static uint8_t mle_build_link_request_cmd(mle_cmd_t* mle_cmd);
static uint8_t mle_build_link_accept_cmd(mle_cmd_t* mle_cmd);
static uint8_t mle_build_link_accept_request_cmd(mle_cmd_t* mle_cmd);
static uint8_t mle_build_link_reject_cmd(mle_cmd_t* mle_cmd);

static uint8_t mle_build_advertisement_cmd(mle_cmd_t* mle_cmd);

static uint8_t mle_build_data_request_cmd(mle_cmd_t* mle_cmd);
static uint8_t mle_build_data_response_cmd(mle_cmd_t* mle_cmd);

static uint8_t mle_build_parent_request_cmd(mle_cmd_t* mle_cmd);
static uint8_t mle_build_parent_response_cmd(mle_cmd_t* mle_cmd);

static uint8_t mle_build_child_id_request_cmd(mle_cmd_t* mle_cmd);
static uint8_t mle_build_child_id_response_cmd(mle_cmd_t* mle_cmd);

static uint8_t mle_build_child_update_cmd(mle_cmd_t* mle_cmd);
static uint8_t mle_build_child_update_response_cmd(mle_cmd_t* mle_cmd);

#endif /* __MLE_CMD_H_ */
