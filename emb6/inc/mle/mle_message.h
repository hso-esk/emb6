#ifndef  __MLE_MSG_H_
#define  __MLE_MSG_H_

#include "emb6.h"
#include "mle_cmd.h"



typedef struct {
//uint8_t          aux_header;     /**< Aux Header */
mle_cmd_t          command;        /**< mle command */
uint8_t            mic ;           /**< MIC */
}mle_msg_t ;




void process_incoming_mle_msg();

uint8_t mle_link_request_to_routers(int8_t pan_id);

uint8_t mle_parent_request(int8_t nwk_id , uint8_t Scan_Mask);


#endif /* __MLE_MSG_H_ */
