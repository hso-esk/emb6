/*
 * emb6 is licensed under the 3-clause BSD license. This license gives everyone
 * the right to use and distribute the code, either in binary or source code
 * format, as long as the copyright license is retained in the source code.
 *
 * The emb6 is derived from the Contiki OS platform with the explicit approval
 * from Adam Dunkels. However, emb6 is made independent from the OS through the
 * removal of protothreads. In addition, APIs are made more flexible to gain
 * more adaptivity during run-time.
 *
 * The license text is:
 *
 * Copyright (c) 2015,
 * Hochschule Offenburg, University of Applied Sciences
 * Laboratory Embedded Systems and Communications Electronics.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO
 * EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 * ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
/*============================================================================*/
/**
 *      \addtogroup emb6
 *      @{
 *   \addtogroup stack_API Stack API
 *   @{
 *   \addtogroup compile_config Compile Time Configuration
 *   @{
*/
/*! \file   emb6_conf.h

    \author Peter Lehmann peter.lehmann@hs-offenburg.de
            Phuong Nguyen minh.nguyen@hs-offenburg.de

    \brief  emb6 compile-time parameter configuration

    \version 0.0.2
*/

#ifndef EMB6_CONF_H_
#define EMB6_CONF_H_

#define EMB6_TEST_CFG_CONT_TX_EN                      FALSE
#define EMB6_TEST_CFG_CONT_RX_EN                      FALSE
#define EMB6_TEST_CFG_WOR_EN                          FALSE



/*=============================================================================
                                APPLICATION LAYER SECTION
===============================================================================*/
#ifndef EMB6_INIT_ROOT
/** Define node act as DAG root */
#define     EMB6_INIT_ROOT                     FALSE
#endif

/** Define a network prefix for dag root */
#define NETWORK_PREFIX_DODAG                   0xaaaa, 0x0000, 0x0000, 0x0000

#define UIP_CONF_DS6_DEFAULT_PREFIX             0xaaaa

/*=============================================================================
                                TRANSPORT LAYER SECTION
===============================================================================*/

/** Define using of UDP (should not be undefined) */
#define UIP_CONF_UDP                          TRUE

/** The amount of concurrent UDP connections. */
#define UIP_CONF_UDP_CONNS                    4

/** Define using of TCP (should not be undefined) */
#define UIP_CONF_TCP                          FALSE

/*=============================================================================
                                NETWORK LAYER SECTION
===============================================================================*/

/** Define IPv6 as based protocol (should not be undefined) */
#define UIP_CONF_IPV6                        TRUE

/* NETSTACK_CONF_WITH_IPV6 specifies whether or not IPv6 should be used. If IPv6
    is not used, IPv4 is used instead. */
#define NETSTACK_CONF_WITH_IPV6              TRUE

/** Define using of ICMP6 (should not be undefined) */
#define UIP_CONF_ICMP6                       TRUE

/** Define router functionality TRUE by default */
#ifndef UIP_CONF_ROUTER
#define UIP_CONF_ROUTER                      TRUE
#endif

/** Neighbor table size */
#define NBR_TABLE_CONF_MAX_NEIGHBORS         10

/** Routing table */
#define UIP_CONF_MAX_ROUTES                  10

/** Unicast address list */
#define UIP_CONF_DS6_ADDR_NBU                3

/** Should we use LinkLayer acks in NUD ? */
#define UIP_CONF_DS6_LL_NUD                  TRUE

/** Force acknowledge from sender (test hardware autoacks) */
#define SICSLOWPAN_CONF_ACK_ALL              TRUE

/** Do we support 6lowpan fragmentation */
#define SICSLOWPAN_CONF_FRAG                 TRUE

#define SICSLOWPAN_CONF_FRAGMENT_BUFFERS     4

/** Most browsers reissue GETs after 3 seconds which stops frag reassembly, longer MAXAGE does no good */
#define SICSLOWPAN_CONF_MAXAGE               3

/** Do we compress the IP header or not */
#define SICSLOWPAN_CONF_COMPRESSION          SICSLOWPAN_COMPRESSION_HC06

/*==============================================================================

                        Security Configuration

 =============================================================================*/



#ifdef LLSEC802154_CONF_ENABLED
#define LLSEC802154_ENABLED                       LLSEC802154_CONF_ENABLED
#else
#define LLSEC802154_ENABLED                       FALSE
#endif /* LLSEC802154_CONF_ENABLED */

#if LLSEC802154_ENABLED
/* Auxiliary Header must be enabled for security */
#define LLSEC802154_USES_AUX_HEADER               TRUE

#ifdef LLSEC802154_CONF_SECURITY_LEVEL
#define LLSEC802154_SECURITY_LEVEL                LLSEC802154_CONF_SECURITY_LEVEL
#else
#define LLSEC802154_SECURITY_LEVEL                0
#endif /* LLSEC802154_CONF_SECURITY_LEVEL */

#ifdef LLSEC802154_CONF_USES_EXPLICIT_KEYS
#define LLSEC802154_USES_EXPLICIT_KEYS            LLSEC802154_CONF_USES_EXPLICIT_KEYS
#else
#define LLSEC802154_USES_EXPLICIT_KEYS            0
#endif /* LLSEC802154_CONF_USES_EXPLICIT_KEYS */

#ifdef LLSEC802154_CONF_USES_FRAME_COUNTER
#define LLSEC802154_USES_FRAME_COUNTER            LLSEC802154_CONF_USES_FRAME_COUNTER
#else
#define LLSEC802154_USES_FRAME_COUNTER            0
#endif /* LLSEC802154_CONF_USES_FRAME_COUNTER */

#else
/* Auxiliary Header is not required */
#define LLSEC802154_USES_AUX_HEADER               FALSE
#endif /* #if LLSEC802154_ENABLED */


/*==============================================================================
                        Neighbor Discovery Configuration
 =============================================================================*/

/*! \name RFC4861 (Neighbor Discovery for IPv6) Router constants
 *     set to 1 if ad hoc network is used */
/** enable/disable RA sending, not needed when using RPL */
#ifndef UIP_CONF_ND6_SEND_RA
#define UIP_ND6_SEND_RA                     TRUE
#else
#define UIP_ND6_SEND_RA UIP_CONF_ND6_SEND_RA
#endif

/** enable/disable NA sending, not needed when using RPL */
#ifndef UIP_CONF_ND6_SEND_NA
#define UIP_ND6_SEND_NA                     TRUE
#else
#define UIP_ND6_SEND_NA UIP_CONF_ND6_SEND_NA
#endif


/*=============================================================================
                                  RPL SECTION
===============================================================================*/

/**
 * This value decides if this node must stay as a leaf or not
 * leaf => reduced function device
 */
#ifdef RPL_CONF_LEAF_ONLY
#define RPL_LEAF_ONLY                       RPL_CONF_LEAF_ONLY
#else
#define RPL_LEAF_ONLY                       0
#endif

/** UIP_CONF_IPV6_RPL     RPL motes use the uip.c link layer address
 *    or optionally the harded coded address (but without the prefix!).
 *     different instances can be made by changing the link layer addresses
 *     For multihop testing, configure intermediate notes as routers.
 */
#if NET_USE_RPL == TRUE
#define UIP_CONF_IPV6_RPL                   TRUE
#endif

/* If RPL is enabled also enable the RPL NBR Policy */
#if UIP_CONF_IPV6_RPL
#ifndef NBR_TABLE_FIND_REMOVABLE
#define NBR_TABLE_FIND_REMOVABLE rpl_nbr_policy_find_removable
#endif /* NBR_TABLE_FIND_REMOVABLE */
#endif /* UIP_CONF_IPV6_RPL */

/** Set to 1 to enable RPL statistics */
#define    RPL_CONF_STATS                   FALSE
#define    RPL_CONF_DAO_LATENCY             bsp_getTRes()
#define    RPL_CONF_DAG_MC                  RPL_DAG_MC_ETX
/*
 * The objective function (OF) used by a RPL root is configurable through
 * the RPL_CONF_OF_OCP parameter. This is defined as the objective code
 * point (OCP) of the OF, RPL_OCP_OF0 or RPL_OCP_MRHOF. This flag is of
 * no relevance to non-root nodes, which run the OF advertised in the
 * instance they join.
 * Make sure the selected of is inRPL_SUPPORTED_OFS.
 */
#ifdef RPL_CONF_OF_OCP
#define RPL_OF_OCP RPL_CONF_OF_OCP
#else /* RPL_CONF_OF_OCP */
#define RPL_OF_OCP RPL_OCP_MRHOF
#endif /* RPL_CONF_OF_OCP */

/*
 * The set of objective functions supported at runtime. Nodes are only
 * able to join instances that advertise an OF in this set. To include
 * both OF0 and MRHOF, use {&rpl_of0, &rpl_mrhof}.
 */
#ifdef RPL_CONF_SUPPORTED_OFS
#define RPL_SUPPORTED_OFS RPL_CONF_SUPPORTED_OFS
#else /* RPL_CONF_SUPPORTED_OFS */
#define RPL_SUPPORTED_OFS {&rpl_of0, &rpl_mrhof}
#endif /* RPL_CONF_SUPPORTED_OFS */

/*
 * Enable/disable RPL Metric Containers (MC). The actual MC in use
 * for a given DODAG is decided at runtime, when joining. Note that
 * OF0 (RFC6552) operates without MC, and so does MRHOF (RFC6719) when
 * used with ETX as a metric (the rank is the metric). We disable MC
 * by default, but note it must be enabled to support joining a DODAG
 * that requires MC (e.g., MRHOF with a metric other than ETX).
 */
#ifdef RPL_CONF_WITH_MC
#define RPL_WITH_MC RPL_CONF_WITH_MC
#else /* RPL_CONF_WITH_MC */
#define RPL_WITH_MC 0
#endif /* RPL_CONF_WITH_MC */

/* The MC advertised in DIOs and propagating from the root */
#ifdef RPL_CONF_DAG_MC
#define RPL_DAG_MC                          RPL_CONF_DAG_MC
#else
#define RPL_DAG_MC                          RPL_DAG_MC_NONE
#endif /* RPL_CONF_DAG_MC */

/** This value decides which DAG instance we should participate in by default. */
#ifdef RPL_CONF_DEFAULT_INSTANCE
#define RPL_DEFAULT_INSTANCE                RPL_CONF_DEFAULT_INSTANCE
#else
#define RPL_DEFAULT_INSTANCE                rpl_config.defInst
#endif /* RPL_CONF_DEFAULT_INSTANCE */

/**
 * The DIO interval (n) represents 2^n ms.
 *
 * According to the specification, the default value is 3 which
 * means 8 milliseconds. That is far too low when using duty cycling
 * with wake-up intervals that are typically hundreds of milliseconds.
 * ContikiRPL thus sets the default to 2^12 ms = 4.096 s.
 */
#ifdef RPL_CONF_DIO_INTERVAL_MIN
#define RPL_DIO_INTERVAL_MIN                RPL_CONF_DIO_INTERVAL_MIN
#else
#define RPL_DIO_INTERVAL_MIN                rpl_config.DIOintmin
#endif

/**
 * Maximum amount of timer doublings.
 *
 * The maximum interval will by default be 2^(12+8) ms = 1048.576 s.
 * RFC 6550 suggests a default value of 20, which of course would be
 * unsuitable when we start with a minimum interval of 2^12.
 */
#ifdef RPL_CONF_DIO_INTERVAL_DOUBLINGS
#define RPL_DIO_INTERVAL_DOUBLINGS          RPL_CONF_DIO_INTERVAL_DOUBLINGS
#else
#define RPL_DIO_INTERVAL_DOUBLINGS          rpl_config.DIOintdoub
#endif

/**
 * DIO redundancy. To learn more about this, see RFC 6206.
 *
 * RFC 6550 suggests a default value of 10. It is unclear what the basis
 * of this suggestion is. Network operators might attain more efficient
 * operation by tuning this parameter for specific deployments.
 */
#ifdef RPL_CONF_DIO_REDUNDANCY
#define RPL_DIO_REDUNDANCY                  RPL_CONF_DIO_REDUNDANCY
#else
#define RPL_DIO_REDUNDANCY                  10
#endif

/**
 * Maximum of concurent RPL instances.
 */
#ifdef RPL_CONF_MAX_INSTANCES
#define RPL_MAX_INSTANCES                   RPL_CONF_MAX_INSTANCES
#else
#define RPL_MAX_INSTANCES                   1
#endif /* RPL_CONF_MAX_INSTANCES */

/**
 * Maximum number of DAGs within an instance.
 */
#ifdef RPL_CONF_MAX_DAG_PER_INSTANCE
#define RPL_MAX_DAG_PER_INSTANCE            RPL_CONF_MAX_DAG_PER_INSTANCE
#else
#define RPL_MAX_DAG_PER_INSTANCE     2
#endif /* RPL_CONF_MAX_DAG_PER_INSTANCE */

/*
 * RPL Default route lifetime
 * The RPL route lifetime is used for the downward routes and for the default
 * route. In a high density network with DIO suppression activated it may happen
 * that a node will never send a DIO once the DIO interval becomes high as it
 * has heard DIO from many neighbors already. As the default route to the
 * preferred parent has a lifetime reset by receiving DIO from the parent, it
 * means that the default route can be destroyed after a while. Setting the
 * default route with infinite lifetime secures the upstream route.
 */
#ifdef RPL_CONF_DEFAULT_ROUTE_INFINITE_LIFETIME
#define RPL_DEFAULT_ROUTE_INFINITE_LIFETIME                    RPL_CONF_DEFAULT_ROUTE_INFINITE_LIFETIME
#else
#define RPL_DEFAULT_ROUTE_INFINITE_LIFETIME                    0
#endif /* RPL_CONF_DEFAULT_ROUTE_INFINITE_LIFETIME */

/*
 * Maximum lifetime of a DAG
 * When a DODAG is not updated since RPL_CONF_DAG_LIFETIME times the DODAG
 * maximum DIO interval the DODAG is removed from the list of DODAGS of the
 * related instance, except if it is the currently joined DODAG.
 */
#ifdef RPL_CONF_DAG_LIFETIME
#define RPL_DAG_LIFETIME                    RPL_CONF_DAG_LIFETIME
#else
#define RPL_DAG_LIFETIME                    3
#endif /* RPL_CONF_DAG_LIFETIME */

/*
 *
 */
#ifndef RPL_CONF_DAO_SPECIFY_DAG
  #if RPL_MAX_DAG_PER_INSTANCE > 1
    #define RPL_DAO_SPECIFY_DAG             1
  #else
    #define RPL_DAO_SPECIFY_DAG             0
  #endif /* RPL_MAX_DAG_PER_INSTANCE > 1 */
#else
  #define RPL_DAO_SPECIFY_DAG               RPL_CONF_DAO_SPECIFY_DAG
#endif /* RPL_CONF_DAO_SPECIFY_DAG */

/**
 * Default route lifetime unit. This is the granularity of time
 * used in RPL lifetime values, in seconds.
 */
#ifndef RPL_CONF_DEFAULT_LIFETIME_UNIT
#define RPL_DEFAULT_LIFETIME_UNIT           rpl_config.defRouteTimeUnit
#else
#define RPL_DEFAULT_LIFETIME_UNIT           RPL_CONF_DEFAULT_LIFETIME_UNIT
#endif

/**
 * Default route lifetime as a multiple of the lifetime unit.
 */
#ifndef RPL_CONF_DEFAULT_LIFETIME
#define RPL_DEFAULT_LIFETIME                rpl_config.defRouteTime
#else
#define RPL_DEFAULT_LIFETIME                RPL_CONF_DEFAULT_LIFETIME
#endif

/*
 * DAG preference field
 */
#ifdef RPL_CONF_PREFERENCE
#define RPL_PREFERENCE                      RPL_CONF_PREFERENCE
#else
#define RPL_PREFERENCE                      0
#endif

/*
 * RPL DAO ACK support. When enabled, DAO ACK will be sent and requested.
 * This will also enable retransmission of DAO when no ack is received.
 * */
#ifdef RPL_CONF_WITH_DAO_ACK
#define RPL_WITH_DAO_ACK RPL_CONF_WITH_DAO_ACK
#else
#define RPL_WITH_DAO_ACK 1
#endif /* RPL_CONF_WITH_DAO_ACK */

/*
 * RPL REPAIR ON DAO NACK. When enabled, DAO NACK will trigger a local
 * repair in order to quickly find a new parent to send DAO's to.
 * NOTE: this is too agressive in some cases so use with care.
 * */
#ifdef RPL_CONF_RPL_REPAIR_ON_DAO_NACK
#define RPL_REPAIR_ON_DAO_NACK RPL_CONF_RPL_REPAIR_ON_DAO_NACK
#else
#define RPL_REPAIR_ON_DAO_NACK 0
#endif /* RPL_CONF_RPL_REPAIR_ON_DAO_NACK */

/*
 * Setting the DIO_REFRESH_DAO_ROUTES will make the RPL root always
 * increase the DTSN (Destination Advertisement Trigger Sequence Number)
 * when sending multicast DIO. This is to get all children to re-register
 * their DAO route. This is needed when DAO-ACK is not enabled to add
 * reliability to route maintenance.
 * */
#ifdef RPL_CONF_DIO_REFRESH_DAO_ROUTES
#define RPL_DIO_REFRESH_DAO_ROUTES RPL_CONF_DIO_REFRESH_DAO_ROUTES
#else
#define RPL_DIO_REFRESH_DAO_ROUTES 1
#endif /* RPL_CONF_DIO_REFRESH_DAO_ROUTES */

/*
 * RPL probing. When enabled, probes will be sent periodically to keep
 * parent link estimates up to date.
 * */
#ifdef RPL_CONF_WITH_PROBING
#define RPL_WITH_PROBING RPL_CONF_WITH_PROBING
#else
#define RPL_WITH_PROBING 1
#endif

/*
 * RPL probing interval.
 * */
#ifdef RPL_CONF_PROBING_INTERVAL
#define RPL_PROBING_INTERVAL RPL_CONF_PROBING_INTERVAL
#else
#define RPL_PROBING_INTERVAL (120 * bsp_getTRes())
#endif

/*
 * Function used to select the next parent to be probed.
 * */
#ifdef RPL_CONF_PROBING_SELECT_FUNC
#define RPL_PROBING_SELECT_FUNC RPL_CONF_PROBING_SELECT_FUNC
#else
#define RPL_PROBING_SELECT_FUNC get_probing_target
#endif

/*
 * Function used to send RPL probes.
 * To probe with DIO, use:
 * #define RPL_CONF_PROBING_SEND_FUNC(instance, addr) dio_output((instance), (addr))
 * To probe with DIS, use:
 * #define RPL_CONF_PROBING_SEND_FUNC(instance, addr) dis_output((addr))
 * Any other custom probing function is also acceptable.
 * */
#ifdef RPL_CONF_PROBING_SEND_FUNC
#define RPL_PROBING_SEND_FUNC RPL_CONF_PROBING_SEND_FUNC
#else
#define RPL_PROBING_SEND_FUNC(instance, addr) dio_output((instance), (addr))
#endif

/*
 * Function used to calculate next RPL probing interval
 * */
#ifdef RPL_CONF_PROBING_DELAY_FUNC
#define RPL_PROBING_DELAY_FUNC RPL_CONF_PROBING_DELAY_FUNC
#else
#define RPL_PROBING_DELAY_FUNC get_probing_delay
#endif

/*
 * Interval of DIS transmission
 */
#ifdef  RPL_CONF_DIS_INTERVAL
#define RPL_DIS_INTERVAL                RPL_CONF_DIS_INTERVAL
#else
#define RPL_DIS_INTERVAL                60
#endif

/*
 * Added delay of first DIS transmission after boot
 */
#ifdef  RPL_CONF_DIS_START_DELAY
#define RPL_DIS_START_DELAY             RPL_CONF_DIS_START_DELAY
#else
#define RPL_DIS_START_DELAY             5
#endif

/* RPL_CONF_MOP specifies the RPL mode of operation that will be
 * advertised by the RPL root. Possible values: RPL_MOP_NO_DOWNWARD_ROUTES,
 * RPL_MOP_NON_STORING, RPL_MOP_STORING_NO_MULTICAST, RPL_MOP_STORING_MULTICAST */
#ifndef RPL_CONF_MOP
#define RPL_CONF_MOP RPL_MOP_STORING_NO_MULTICAST
#endif /* RPL_CONF_MOP */

/* RPL_NS_CONF_LINK_NUM specifies the maximum number of links a RPL root
 * will maintain in non-storing mode. */
#ifndef RPL_NS_CONF_LINK_NUM
#define RPL_NS_CONF_LINK_NUM 20
#endif /* RPL_NS_CONF_LINK_NUM */


/*=============================================================================
                                  uIP SECTION
===============================================================================*/


/** Should we use LinkLayer acks in NUD ?*/
#ifndef UIP_CONF_DS6_LL_NUD
#define UIP_DS6_LL_NUD                      TRUE
#else
#define UIP_DS6_LL_NUD UIP_CONF_DS6_LL_NUD
#endif

#ifndef UIP_CONF_IPV6_QUEUE_PKT
/** Do we do per %neighbor queuing during address resolution */
#define UIP_CONF_IPV6_QUEUE_PKT             TRUE
#endif

/** Default uip_aligned_buf and sicslowpan_aligned_buf sizes of 1280 overflows RAM */
#ifndef UIP_CONF_BUFFER_SIZE
#define UIP_CONF_BUFFER_SIZE                350
#endif

/**
 * The maximum number of simultaneously open TCP connections.
 *
 * Since the TCP connections are statically allocated, turning this
 * configuration knob down results in less RAM used. Each TCP
 * connection requires approximately 30 bytes of memory.
 *
 * \hideinitializer
 */
#ifndef UIP_CONF_MAX_CONNECTIONS
#define UIP_CONNS                           2
#else /* UIP_CONF_MAX_CONNECTIONS */
#define UIP_CONNS (UIP_CONF_MAX_CONNECTIONS)
#endif /* UIP_CONF_MAX_CONNECTIONS */



#ifndef UIP_CONF_DS6_PREFIX_NBU
/** Default number of IPv6 prefixes associated to the node's interface */
#define UIP_CONF_DS6_PREFIX_NBU             2
#endif

#ifndef UIP_CONF_DS6_DEFRT_NBU
/** Minimum number of default routers */
#define UIP_CONF_DS6_DEFRT_NBU              1
#endif

/**
 * Timeout for packet reassembly at the 6lowpan layer
 * (should be < 60s)
 */
#ifdef SICSLOWPAN_CONF_MAXAGE
#define SICSLOWPAN_REASS_MAXAGE (SICSLOWPAN_CONF_MAXAGE)
#else
#define SICSLOWPAN_REASS_MAXAGE             20
#endif

/**
 * Determines if uIP should use a fixed IP address or not.
 *
 * If uIP should use a fixed IP address, the settings are set in the
 * uipopt.h file. If not, the macros uip_sethostaddr(),
 * uip_setdraddr() and uip_setnetmask() should be used instead.
 *
 * \hideinitializer
 */
#define UIP_FIXEDADDR                       0

/**
 * Ping IP address assignment.
 *
 * uIP uses a "ping" packets for setting its own IP address if this
 * option is set. If so, uIP will start with an empty IP address and
 * the destination IP address of the first incoming "ping" (ICMP echo)
 * packet will be used for setting the hosts IP address.
 *
 * \note This works only if UIP_FIXEDADDR is 0.
 *
 * \hideinitializer
 */
#ifdef UIP_CONF_PINGADDRCONF
#define UIP_PINGADDRCONF (UIP_CONF_PINGADDRCONF)
#else /* UIP_CONF_PINGADDRCONF */
#define UIP_PINGADDRCONF 0
#endif /* UIP_CONF_PINGADDRCONF */


/**
 * Specifies if the uIP ARP module should be compiled with a fixed
 * Ethernet MAC address or not.
 *
 * If this configuration option is 0, the macro uip_setethaddr() can
 * be used to specify the Ethernet address at run-time.
 *
 * \hideinitializer
 */
#define UIP_FIXEDETHADDR                    0


/*------------------------------------------------------------------------------*/
/**
 * The IP TTL (time to live) of IP packets sent by uIP.
 *
 * This should normally not be changed.
 */
#ifdef UIP_CONF_TTL
#define UIP_TTL         UIP_CONF_TTL
#else /* UIP_CONF_TTL */
#define UIP_TTL                             64
#endif /* UIP_CONF_TTL */

/**
 * The maximum time an IP fragment should wait in the reassembly
 * buffer before it is dropped.
 *
 */
#define UIP_REASS_MAXAGE                    60 /*60s*/

/**
 * Turn on support for IP packet reassembly.
 *
 * uIP supports reassembly of fragmented IP packets. This features
 * requires an additional amount of RAM to hold the reassembly buffer
 * and the reassembly code size is approximately 700 bytes.  The
 * reassembly buffer is of the same size as the uip_buf buffer
 * (configured by UIP_BUFSIZE).
 *
 * \note IP packet reassembly is not heavily tested.
 *
 * \hideinitializer
 */
#ifdef UIP_CONF_REASSEMBLY
#define UIP_REASSEMBLY                      (UIP_CONF_REASSEMBLY)
#else /* UIP_CONF_REASSEMBLY */
#define UIP_REASSEMBLY                      0
#endif /* UIP_CONF_REASSEMBLY */
/** @} */

/*------------------------------------------------------------------------------*/


/** The maximum transmission unit at the IP Layer*/
#define UIP_LINK_MTU                        1280

#ifndef UIP_CONF_IPV6_CHECKS
/** Do we do IPv6 consistency checks (highly recommended, default: yes) */
#define UIP_CONF_IPV6_CHECKS                1
#endif

#ifndef UIP_CONF_IPV6_REASSEMBLY
/** Do we do IPv6 fragmentation (default: no) */
#define UIP_CONF_IPV6_REASSEMBLY            0
#endif

#ifndef UIP_CONF_NETIF_MAX_ADDRESSES
/** Default number of IPv6 addresses associated to the node's interface */
#define UIP_CONF_NETIF_MAX_ADDRESSES        3
#endif

#ifndef UIP_CONF_DS6_PREFIX_NBU
/** Default number of IPv6 prefixes associated to the node's interface */
#define UIP_CONF_DS6_PREFIX_NBU             2
#endif


/*------------------------------------------------------------------------------*/
/**
 *
 * \note The UDP support in uIP is still not entirely complete; there
 * is no support for sending or receiving broadcast or multicast
 * packets, but it works well enough to support a number of vital
 * applications such as DNS queries, though
 */

/**
 * Toggles whether UDP support should be compiled in or not.
 *
 * \hideinitializer
 */
#ifdef UIP_CONF_UDP
#define UIP_UDP UIP_CONF_UDP
#else /* UIP_CONF_UDP */
#define UIP_UDP                             1
#endif /* UIP_CONF_UDP */

/**
 * Toggles if UDP checksums should be used or not.
 *
 * \note Support for UDP checksums is currently not included in uIP,
 * so this option has no function.
 *
 * \hideinitializer
 */
#ifdef UIP_CONF_UDP_CHECKSUMS
#define UIP_UDP_CHECKSUMS                   (UIP_CONF_UDP_CHECKSUMS)
#else
#define UIP_UDP_CHECKSUMS                   (UIP_CONF_IPV6)
#endif

/**
 * The maximum amount of concurrent UDP connections.
 *
 * \hideinitializer
 */
#ifdef UIP_CONF_UDP_CONNS
#define UIP_UDP_CONNS                       (UIP_CONF_UDP_CONNS)
#else /* UIP_CONF_UDP_CONNS */
#define UIP_UDP_CONNS                       10
#endif /* UIP_CONF_UDP_CONNS */


/**
 * Toggles whether TCP support should be compiled in or not.
 *
 * \hideinitializer
 */
#ifdef UIP_CONF_TCP
#define UIP_TCP (UIP_CONF_TCP)
#else /* UIP_CONF_TCP */
#define UIP_TCP                             1
#endif /* UIP_CONF_TCP */

/**
 * Determines if support for opening connections from uIP should be
 * compiled in.
 *
 * If the applications that are running on top of uIP for this project
 * do not need to open outgoing TCP connections, this configuration
 * option can be turned off to reduce the code size of uIP.
 *
 * \hideinitializer
 */
#ifndef UIP_CONF_ACTIVE_OPEN
#define UIP_ACTIVE_OPEN                     1
#else /* UIP_CONF_ACTIVE_OPEN */
#define UIP_ACTIVE_OPEN                     (UIP_CONF_ACTIVE_OPEN)
#endif /* UIP_CONF_ACTIVE_OPEN */


/**
 * The maximum number of simultaneously listening TCP ports.
 *
 * Each listening TCP port requires 2 bytes of memory.
 *
 * \hideinitializer
 */
#ifndef UIP_CONF_MAX_LISTENPORTS
#define UIP_LISTENPORTS                     20
#else /* UIP_CONF_MAX_LISTENPORTS */
#define UIP_LISTENPORTS                     (UIP_CONF_MAX_LISTENPORTS)
#endif /* UIP_CONF_MAX_LISTENPORTS */

/**
 * Determines if support for TCP urgent data notification should be
 * compiled in.
 *
 * Urgent data (out-of-band data) is a rarely used TCP feature that
 * very seldom would be required.
 *
 * \hideinitializer
 */
#define UIP_URGDATA                         0

/**
 * The initial retransmission timeout counted in timer pulses.
 *
 * This should not be changed.
 */
#define UIP_RTO                             3

/**
 * The maximum number of times a segment should be retransmitted
 * before the connection should be aborted.
 *
 * This should not be changed.
 */
#define UIP_MAXRTX                          8

/**
 * The maximum number of times a SYN segment should be retransmitted
 * before a connection request should be deemed to have been
 * unsuccessful.
 *
 * This should not need to be changed.
 */
#define UIP_MAXSYNRTX                       5

/**
 * The TCP maximum segment size.
 *
 * This is should not be to set to more than
 * UIP_BUFSIZE - UIP_LLH_LEN - UIP_TCPIP_HLEN.
 */
#ifdef UIP_CONF_TCP_MSS
#if UIP_CONF_TCP_MSS > (UIP_BUFSIZE - UIP_LLH_LEN - UIP_TCPIP_HLEN)
#error UIP_CONF_TCP_MSS is too large for the current UIP_BUFSIZE
#endif /* UIP_CONF_TCP_MSS > (UIP_BUFSIZE - UIP_LLH_LEN - UIP_TCPIP_HLEN) */
#define UIP_TCP_MSS                         (UIP_CONF_TCP_MSS)
#else /* UIP_CONF_TCP_MSS */
#define UIP_TCP_MSS                         (UIP_BUFSIZE - \
                                             UIP_LLH_LEN - \
                                             UIP_TCPIP_HLEN)
#endif /* UIP_CONF_TCP_MSS */

/**
 * The size of the advertised receiver's window.
 *
 * Should be set low (i.e., to the size of the uip_buf buffer) if the
 * application is slow to process incoming data, or high (32768 bytes)
 * if the application processes data quickly.
 *
 * \hideinitializer
 */
#ifndef UIP_CONF_RECEIVE_WINDOW
#define UIP_RECEIVE_WINDOW                  (UIP_TCP_MSS)
#else
#define UIP_RECEIVE_WINDOW                  (UIP_CONF_RECEIVE_WINDOW)
#endif

/**
 * How long a connection should stay in the E_TIME_WAIT state.
 *
 * This can be reduced for faster entry into power saving modes.
 */
#ifndef UIP_CONF_WAIT_TIMEOUT
#define UIP_E_TIME_WAIT_TIMEOUT             120
#else
#define UIP_E_TIME_WAIT_TIMEOUT             UIP_CONF_WAIT_TIMEOUT
#endif


/*------------------------------------------------------------------------------*/


/**
 * The size of the ARP table.
 *
 * This option should be set to a larger value if this uIP node will
 * have many connections from the local network.
 *
 * \hideinitializer
 */
#ifdef UIP_CONF_ARPTAB_SIZE
#define UIP_ARPTAB_SIZE                     (UIP_CONF_ARPTAB_SIZE)
#else
#define UIP_ARPTAB_SIZE                     8
#endif

/**
 * The maximum age of ARP table entries measured in 10ths of seconds.
 *
 * An UIP_ARP_MAXAGE of 120 corresponds to 20 minutes (BSD
 * default).
 */
#define UIP_ARP_MAXAGE                      120


/*------------------------------------------------------------------------------*/



#define UIP_DEFAULT_PREFIX_LEN              64

/**
 * If we use IPHC compression, how many address contexts do we support
 */
#ifndef SICSLOWPAN_CONF_MAX_ADDR_CONTEXTS
#define SICSLOWPAN_CONF_MAX_ADDR_CONTEXTS 1
#endif




/**
 * The size of the uIP packet buffer.
 *
 * The uIP packet buffer should not be smaller than 60 bytes, and does
 * not need to be larger than 1514 bytes. Lower size results in lower
 * TCP throughput, larger size results in higher TCP throughput.
 *
 * \hideinitializer
 */
#ifndef UIP_CONF_BUFFER_SIZE
#define UIP_BUFSIZE                         (UIP_LINK_MTU + UIP_LLH_LEN)
#else /* UIP_CONF_BUFFER_SIZE */
#define UIP_BUFSIZE                         (UIP_CONF_BUFFER_SIZE)
#endif /* UIP_CONF_BUFFER_SIZE */


/**
 * Determines if statistics support should be compiled in.
 *
 * The statistics is useful for debugging and to show the user.
 *
 * \hideinitializer
 */
#ifndef UIP_CONF_STATISTICS
#define UIP_STATISTICS                      0
#else /* UIP_CONF_STATISTICS */
#define UIP_STATISTICS                      (UIP_CONF_STATISTICS)
#endif /* UIP_CONF_STATISTICS */

/**
 * Determines if logging of certain events should be compiled in.
 *
 * This is useful mostly for debugging. The function uip_log()
 * must be implemented to suit the architecture of the project, if
 * logging is turned on.
 *
 * \hideinitializer
 */
#ifndef UIP_CONF_LOGGING
#define UIP_LOGGING                         0
#else /* UIP_CONF_LOGGING */
#define UIP_LOGGING                         (UIP_CONF_LOGGING)
#endif /* UIP_CONF_LOGGING */

/**
 * Broadcast support.
 *
 * This flag configures IP broadcast support. This is useful only
 * together with UDP.
 *
 * \hideinitializer
 *
 */
#ifndef UIP_CONF_BROADCAST
#define UIP_BROADCAST                       0
#else /* UIP_CONF_BROADCAST */
#define UIP_BROADCAST                       (UIP_CONF_BROADCAST)
#endif /* UIP_CONF_BROADCAST */

/**
 * Print out a uIP log message.
 *
 * This function must be implemented by the module that uses uIP, and
 * is called by uIP whenever a log message is generated.
 */
void uip_log(char *msg);

/**
 * The link level header length.
 *
 * This is the offset into the uip_buf where the IP header can be
 * found. For Ethernet, this should be set to 14. For SLIP, this
 * should be set to 0.
 *
 * \note we probably won't use this constant for other link layers than
 * ethernet as they have variable header length (this is due to variable
 * number and type of address fields and to optional security features)
 * E.g.: 802.15.4 -> 2 + (1/2*4/8) + 0/5/6/10/14
 *       802.11 -> 4 + (6*3/4) + 2
 * \hideinitializer
 */
#ifdef UIP_CONF_LLH_LEN
#define UIP_LLH_LEN                         (UIP_CONF_LLH_LEN)
#else /* UIP_LLH_LEN */
#define UIP_LLH_LEN                         0UL
#endif /* UIP_CONF_LLH_LEN */


/**
 * The byte order of the CPU architecture on which uIP is to be run.
 *
 * This option can be either UIP_BIG_ENDIAN (Motorola byte order) or
 * UIP_LITTLE_ENDIAN (Intel byte order).
 *
 * \hideinitializer
 */
#ifdef UIP_CONF_BYTE_ORDER
#define UIP_BYTE_ORDER                      (UIP_CONF_BYTE_ORDER)
#else /* UIP_CONF_BYTE_ORDER */
#define UIP_BYTE_ORDER                      (UIP_LITTLE_ENDIAN)
#endif /* UIP_CONF_BYTE_ORDER */



 /*=============================================================================
                                   UTILS SECTION
 ===============================================================================*/

 /* QUEUEBUF_NUM is the total number of queuebuf */
 #ifndef QUEUEBUF_CONF_NUM
 #define QUEUEBUF_CONF_NUM                  4
 #endif /* QUEUEBUF_CONF_NUM */


 /*=============================================================================
                                 DEBUG ENABLER SECTION
 =============================================================================*/
 /** Define a deepness of the logger helper (see logger.h)*/
#ifndef LOGGER_LEVEL
#define LOGGER_LEVEL                        0
#endif
/** Core logging, should be TRUE for almost all cases except for production
* (see emb6.c) */
#ifndef LOGGER_CORE
#define LOGGER_CORE                         FALSE
#endif

/** Hardware abstraction layer functions    (see target.c) */
#ifndef LOGGER_HAL
#define LOGGER_HAL                          FALSE
#endif

/** Board support package                   (see bsp.c) */
#ifndef LOGGER_BSP
#define LOGGER_BSP                          FALSE
#endif

/** Main functions                          (see emb6_main.c) */
#ifndef LOGGER_MAIN
#define LOGGER_MAIN                         FALSE
#endif

/** Event process functions                 (see llc_xxx.c) */
#ifndef LOGGER_LLC
#define LOGGER_LLC                          FALSE
#endif

/** Event process functions                (see llsec_xxx.c) */

#ifndef LOGGER_LLCSEC
#define LOGGER_LLCSEC                       FALSE
#endif


/** Event process functions                 (see mac_xxx.c) */
#ifndef LOGGER_MAC
#define LOGGER_MAC                          FALSE
#endif

/** Event process functions                 (see phy_xxx.c) */
#ifndef LOGGER_PHY
#define LOGGER_PHY                          FALSE
#endif

/** Radio functions                         (see $(IF).c) */
#ifndef LOGGER_RADIO
#define LOGGER_RADIO                        FALSE
#endif

/** DEMO 6tisch example                 (see demo_6tisch.c) */
#ifndef LOGGER_DEMO_6TISCH
#define LOGGER_DEMO_6TISCH                  FALSE
#endif

/** DEMO UDP socket example                 (see demo_udp_socket.c) */
#ifndef LOGGER_DEMO_UDP_SOCKET
#define LOGGER_DEMO_UDP_SOCKET              FALSE
#endif

 /** DEMO UDP socket simple example        (see demo_udp_socket_simple.c) */
 #ifndef LOGGER_DEMO_UDP_SOCKET_SIMPLE
 #define LOGGER_DEMO_UDP_SOCKET_SIMPLE      FALSE
 #endif

/** DEMO UDP example                        (see demo_exudp.c) */
#ifndef LOGGER_DEMO_UDPIAA
#define LOGGER_DEMO_UDPIAA                  FALSE
#endif

/** DEMO APTB example                       (see demo_aptb_xxx.c) */
#ifndef LOGGER_DEMO_APTB
#define LOGGER_DEMO_APTB                    FALSE
#endif

/** DEMO COAP example                       (see demo_coap_*.c) */
#ifndef LOGGER_DEMO_COAP
#define LOGGER_DEMO_COAP                    FALSE
#endif

/** DEMO LWM2M example                      (see demo_lwm2m_*.c) */
#ifndef LOGGER_DEMO_LWM2M
#define LOGGER_DEMO_LWM2M                   FALSE
#endif

/** DEMO DTLS example                       (see demo_dtls_*.c) */
#ifndef LOGGER_DEMO_DTLS
#define LOGGER_DEMO_DTLS                    FALSE
#endif

/** DEMO MDNS example                       (see demo_mdns_*.c) */
#ifndef LOGGER_DEMO_MDNS
#define LOGGER_DEMO_MDNS                    FALSE
#endif

 /** DEMO Serial API example                (see demo_serialapi_*.c) */
 #ifndef LOGGER_DEMO_SERIALAPI
 #define LOGGER_DEMO_SERIALAPI              FALSE
 #endif

/** DEMO SNIFFER                            (see demo_sniffer.c) */
#ifndef LOGGER_DEMO_SNIFFER
#define LOGGER_DEMO_SNIFFER                 FALSE
#endif

/** DEMO TESTSUITE                          (see tessuite.c) */
#ifndef LOGGER_DEMO_TESTSUITE
#define LOGGER_DEMO_TESTSUITE               FALSE
#endif

/** Event timer functions                   (see etimer.c) */
#ifndef LOGGER_ETIMER
#define LOGGER_ETIMER                       FALSE
#endif

/** Callback timer functions                (see ctimer.c) */
#ifndef LOGGER_CTIMER
#define LOGGER_CTIMER                       FALSE
#endif

/** Event process functions                 (see evproc.c) */
#ifndef LOGGER_EVPROC
#define LOGGER_EVPROC                       FALSE
#endif


/*
********************************************************************************
*                          NETSTACK CONFIGURATIONS
********************************************************************************
*/
 /*!< Enable/Disable support for IEEE Std. 802.15.4G */
#ifndef NETSTK_CFG_IEEE_802154G_EN
#define NETSTK_CFG_IEEE_802154G_EN                          FALSE
#endif

#if (NETSTK_CFG_IEEE_802154G_EN == TRUE)
 /*!< Enable/Disable 2k-length frame support in 802.15.4G mode */
  #ifndef NETSTK_CFG_2K_FRAME_EN
    #define NETSTK_CFG_2K_FRAME_EN                          TRUE
  #endif

  /*!< Enable/Disable MR-FSK PHY, see IEEE Std. 802.15.4g-2012 */
  #ifndef NETSTK_CFG_MR_FSK_PHY_EN
    #define NETSTK_CFG_MR_FSK_PHY_EN                        TRUE
  #endif
#endif

/*!< Turn SW auto-ACK support on MAC by default */
#ifndef NETSTK_SUPPORT_SW_MAC_AUTOACK
#define NETSTK_SUPPORT_SW_MAC_AUTOACK                       FALSE
#endif

/*!< Enable/Disable automatic on/off to netstack
 * If the netstack receives transmission request in sleep mode, it shall
 * go back to the sleep mode upon complete transmission process
 */
#ifndef NETSTK_CFG_AUTO_ONOFF_EN
#define NETSTK_CFG_AUTO_ONOFF_EN                            FALSE
#endif

/*!< Enable/Disable PHY MR-FSK operation mode, see IEEE Std. 802.15.4g-2012 table 68d */
#define NETSTK_CFG_PHY_OP_MODE_1_EN                         TRUE
#define NETSTK_CFG_PHY_OP_MODE_2_EN                         FALSE
#define NETSTK_CFG_PHY_OP_MODE_3_EN                         FALSE

/*!< Enable/Disable low power mode */
#ifndef NETSTK_CFG_LOW_POWER_MODE_EN
#define NETSTK_CFG_LOW_POWER_MODE_EN                        FALSE
#endif

/*!< Enable/Disable loosely-sync feature in low power mode */
#if (NETSTK_CFG_LOW_POWER_MODE_EN == TRUE)
  #ifndef NETSTK_CFG_LOOSELY_SYNC_EN
    #define NETSTK_CFG_LOOSELY_SYNC_EN                      FALSE
  #endif
#endif

/*!< Enable/Disable Data Whitening */
#ifndef NETSTK_CFG_DATA_WHITENING_EN
#define NETSTK_CFG_DATA_WHITENING_EN                        FALSE
#endif

/*!< Enable/Disable eWOR support */
#ifndef NETSTK_CFG_WOR_EN
#define NETSTK_CFG_WOR_EN                                   FALSE
#endif

/*!< CSMA configuration */
#define NETSTK_CFG_CSMA_MIN_BE                    (uint8_t )(   3u )
#define NETSTK_CFG_CSMA_MAX_BE                    (uint8_t )(   5u )
#define NETSTK_CFG_CSMA_MAX_BACKOFF               (uint8_t )(   4u )
#define NETSTK_CFG_CSMA_UNIT_BACKOFF_US           (uint32_t)( 400u )  /* @50kbps 2FSK */

/*!< Default transceiver's transmission power */
#ifndef TX_POWER
#define TX_POWER              0
#endif

/*!< Default transceiver's receive sensitivity */
#ifndef RX_SENSITIVITY
#define RX_SENSITIVITY      -100
#endif

/*!< Default transceiver's transmission power */
#ifndef MAC_ADDR_WORD
#define MAC_ADDR_WORD       0xffff
#endif

/*
********************************************************************************
*                           PARAMETER ASSERTION
********************************************************************************
*/
#define NETSTK_CFG_ARG_CHK_EN               ( 1u )


#endif /* EMB6_CONF_H_ */

/** @} */
/** @} */
/** @} */
