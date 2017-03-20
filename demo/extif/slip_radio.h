/*
 * slip_radio.h
 *
 *  Created on: Jul 3, 2014
 *      Author: yushev
 */
/**
 * \file
 *         A brief description of what this file is
 */
#ifndef SLIP_RADIO_H_
#define SLIP_RADIO_H_


int8_t demo_extifInit(void);

/*============================================================================*/
/*!
    \brief Configuration of the CoAP client application.

    \return  0 on success, otherwise -1
*/
/*============================================================================*/
int8_t demo_extifConf(s_ns_t* pst_netStack);

#endif /* SLIP_RADIO_H_ */
