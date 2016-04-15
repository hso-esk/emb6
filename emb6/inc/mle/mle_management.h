#ifndef  __MLE_MANAGEMENT_H_
#define  __MLE_MANAGEMENT_H_

/*==============================================================================
                                 INCLUDE FILES
 =============================================================================*/








/*==============================================================================
                                     MACROS
 =============================================================================*/

#define		MLE_UDP_LPORT		         19788 /* MLE UDP Port  */
#define		MLE_UDP_RPORT		         19788 /* MLE UDP Port  */
#define     MAX_MLE_UDP_PAYLOAD_LEN      40    /* MLE UDP Payload length  */

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

}mle_node_t ;

/*==============================================================================
                                 API FUNCTIONS
 =============================================================================*/

uint8_t mle_init(void);


/*==============================================================================
								LOCAL FUNCTION
 =============================================================================*/


#endif /* __MLE_MANAGEMENT_H_ */
