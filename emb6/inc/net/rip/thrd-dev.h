/*
 * thrd-dev.h
 *
 *  Created on: 15 Jun 2016
 *  Author: Lukas Zimmermann <lzimmer1@stud.hs-offenburg.de>
 *  Thread Device.
 */

#ifndef EMB6_INC_NET_RIP_THRD_DEV_H_
#define EMB6_INC_NET_RIP_THRD_DEV_H_

#include "thread_conf.h"

/**
 * Thread Device Types.
 */
typedef enum
{
	THRD_DEV_TYPE_NONE = 0x00,	 //!< DEV_TYPE_NONE
	THRD_DEV_TYPE_END = 0x01,   //!< DEV_TYPE_END
	THRD_DEV_TYPE_REED = 0x02,  //!< DEV_TYPE_REED
	THRD_DEV_TYPE_ROUTER = 0x03,//!< DEV_TYPE_ROUTER
	THRD_DEV_TYPE_LEADER = 0x04 //!< DEV_TYPE_LEADER
} thrd_dev_type_t;

/**
 * Thread Device Functionalty (RFD or FFD).
 */
typedef enum
{
	THRD_DEV_FUNC_RFD = 0,//!< THRD_DEV_RFD
	THRD_DEV_FUNC_FFD = 1 //!< THRD_DEV_FFD
} thrd_dev_funct_t;

/**
 * Thread Device Types and RFD/FFD.
 */
typedef struct
{
	uint8_t type;	// thrd_dev_type_t.
	uint8_t isFFD;
	uint8_t isRX_off_when_idle;
#if ( (THRD_DEV_TYPE == THRD_DEV_TYPE_ROUTER) || (THRD_DEV_TYPE == THRD_DEV_TYPE_REED) )
	// uint8_t Router_ID;	// Router ID.
#endif
} thrd_dev_t;

/*! Thread Device Type Configuration. */
extern thrd_dev_t thrd_dev;

extern void thrd_dev_init(void);

/**
 * Set the device type of the Thread Device.
 * @param type The Thread device type thrd_dev_type_t.
 * @retval THRD_ERROR_NONE Device type successfully set.
 * @retval THRD_ERROR_INVALID_ARGS Invalid arguments.
 */
extern thrd_error_t thrd_dev_set_type(thrd_dev_type_t type);

/**
 * Print Thread device information.
 */
void thrd_dev_print_dev_info();

#endif /* EMB6_INC_NET_RIP_THRD_DEV_H_ */
