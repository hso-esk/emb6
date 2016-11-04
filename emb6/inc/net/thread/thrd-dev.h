/**
 * \file thrd-dev.h
 * \author Institute for reliable Embedded Systems and Communication Electronics
 * \date 2016/06/15
 * \version 1.0
 *
 * \brief Thread device
 */

#ifndef EMB6_INC_NET_RIP_THRD_DEV_H_
#define EMB6_INC_NET_RIP_THRD_DEV_H_

#include "thread_conf.h"

/**
 * Thread Device Types.
 */
typedef enum
{
	THRD_DEV_NETTYPE_ROUTER = 0x00,		// Active router.
	THRD_DEV_NETTYPE_REED = 0x01,  		// Router-eligible end device.
	THRD_DEV_NETTYPE_PED = 0x02,		// Powered end devices, also referred as non-sleepy end devices.
	THRD_DEV_NETTYPE_SED = 0x03,		// Sleepy end devices.
	THRD_DEV_NETTYPE_LEADER = 0x04 		// Leader (router with extended capabilities).	// TODO Remove this?
} thrd_dev_type_t;

/**
 * Thread link layer device functionalty (FFD or RFD).
 */
typedef enum
{
	THRD_DEV_LLTYPE_FFD = 0,					// Full function device.
	THRD_DEV_LLTYPE_RX_ON_WHEN_IDLE_RFD = 1,	// Reduced function device.
	THRD_DEV_LLTYPE_RX_OFF_WHEN_IDLE_RFD = 2	// Reduced function device.
} thrd_dev_llfunct_t;

/**
 * Thread Device Types and RFD/FFD.
 */
typedef struct
{
	uint8_t net_type;	// Network layer type.
	uint8_t ll_type;	// Link layer type.
#if ( (THRD_DEV_TYPE == THRD_DEV_TYPE_ROUTER) || (THRD_DEV_TYPE == THRD_DEV_TYPE_REED) )
	// uint8_t Router_ID;	// Router ID.
#endif
} thrd_dev_t;

/*! Thread Device Type Configuration. */
extern thrd_dev_t thrd_dev;

extern void thrd_dev_init(void);

/**
 * Get the device type (network layer) of the Thread device.
 * @retval The Thread device type (thrd_dev_type_t).
 */
extern thrd_dev_type_t thrd_get_dev_net_type(void);

/**
 * Set the device type (network layer) of the Thread device.
 * @param type The Thread device type thrd_dev_type_t.
 * @retval THRD_ERROR_NONE Device type successfully set.
 * @retval THRD_ERROR_INVALID_ARGS Invalid arguments.
 */
extern thrd_error_t thrd_set_dev_net_type(thrd_dev_type_t type);

/**
 * Print Thread device information.
 */
void thrd_dev_print_dev_info();

#endif /* EMB6_INC_NET_RIP_THRD_DEV_H_ */
