#ifndef _DEMO_ROUTE_DEMO_ROUTE_SINK_H_
#define _DEMO_ROUTE_DEMO_ROUTE_SINK_H_

/*==============================================================================
                         FUNCTION PROTOTYPES OF THE API
==============================================================================*/

/*----------------------------------------------------------------------------*/
/*!
   \brief Initialization of the simple multicast example application.

*/
/*----------------------------------------------------------------------------*/
int8_t demo_routeSinkInit(void);

/*----------------------------------------------------------------------------*/
/*!
    \brief Configuration of the simple multicast example application.

    \return 0 - error, 1 - success
*/
/*----------------------------------------------------------------------------*/
uint8_t demo_routeSinkConf(s_ns_t* pst_netStack);



#endif /* _DEMO_ROUTE_DEMO_MCAST_ROUTE_H_ */
