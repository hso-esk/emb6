#ifndef  __MLE_MANAGEMENT_H_
#define  __MLE_MANAGEMENT_H_

/*==============================================================================
                                 INCLUDE FILES
 =============================================================================*/

#include "mle_message.h"



/*==============================================================================
                                     MACROS
 =============================================================================*/

#define		MLE_UDP_LPORT		         19788 /* MLE UDP Port  */
#define		MLE_UDP_RPORT		         19788 /* MLE UDP Port  */
#define     MAX_MLE_UDP_PAYLOAD_LEN      200    /* MLE UDP Payload length  */

/** macro for a half of ipv6 address */
#define     HALFIP6ADDR                 8
/** full of ipv6 address */
#define     IP6ADDRSIZE                 16


/*==============================================================================
									TYPEDEFS
 =============================================================================*/

typedef enum
{
	NOT_LINKED,
	CHILD,
	PARENT,
} mle_device_mode_t;


typedef struct {
	mle_device_mode_t     mode;                               /**< device mode */
	//uip_ipaddr_t          my_address;
}mle_node_t ;

/*==============================================================================
                                 API FUNCTIONS
 =============================================================================*/

uint8_t mle_init(void);



/*==============================================================================
								LOCAL FUNCTION
 =============================================================================*/
/*
static uint8_t mle_link_request_to_routers(int8_t pan_id);

static uint8_t mle_parent_request(int8_t nwk_id , uint8_t Scan_Mask);*/

#endif /* __MLE_MANAGEMENT_H_ */
