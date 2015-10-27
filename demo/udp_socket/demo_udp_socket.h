/**
 * \file    demo_udp_socket.h
 * \author  Phuong Nguyen
 * \version 0.0.2
 *
 * \brief   UDP demo application using emb6 stack
 */

#ifndef __ADDON_DEMO_UDP_ALIVE_H__
#define __ADDON_DEMO_UDP_ALIVE_H__

/**
 * \brief   Initializes UDP demo application
 *
 * \return  0 when success, otherwise -1
 */
int8_t demo_udpSocketInit(void);


/**
 * \brief   Configures UDP demo application
 *
 * \param   p_netStack  Pointer to NET stack
 *
 * \return  0 when success, otherwise -1
 */
int8_t demo_udpSocketCfg(s_ns_t *p_netStack);

#endif /* __ADDON_DEMO_UDP_ALIVE_H__ */
