/**
 * \addtogroup bsp
 * @{
 * \addtogroup if    PHY interfaces
 * @{ */

/**
 * \defgroup fake_radio PC based fake radio transceiver library
 *
 * The PC based radio library provides function for sending
 * and receiving packets via UDP ports on a loopback interface
 * of linux.
 *
 * @{
 */
/*
 * tcpip.h
 *
 *  Created on: Aug, 2015
 *      Author: Artem Yushev artem.yushev@hs-offenburg.de
 */

#ifndef TCPIP_RADIO_H_
#define TCPIP_RADIO_H_

#include "emb6.h"
#include "evproc.h"

#define FRADIO_S_IP                            "127.0.0.1"
#define FRADIO_C_IP                            "127.0.0.1"
#define FRADIO_OUTPORT                        "40001"
#define FRADIO_INPORT                        "40002"
#define FRADIO_OUTPORT_CLIENT                FRADIO_OUTPORT
#define FRADIO_INPORT_CLIENT                FRADIO_INPORT
#define UDPDEV_LLADDR_CLIENT                "1"
#define FRADIO_OUTPORT_SERVER                FRADIO_INPORT
#define FRADIO_INPORT_SERVER                FRADIO_OUTPORT
#define UDPDEV_LLADDR_SERVER                "2"


#endif /* TCPIP_RADIO_H_ */
/** @} */
/** @} */
/** @} */
