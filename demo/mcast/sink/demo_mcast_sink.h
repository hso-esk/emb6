#ifndef _DEMO_MCAST_DEMO_MCAST_SINK_H_
#define _DEMO_MCAST_DEMO_MCAST_SINK_H_

/*==============================================================================
                         FUNCTION PROTOTYPES OF THE API
==============================================================================*/

/*----------------------------------------------------------------------------*/
/*!
   \brief Initialization of the simple multicast example application.

*/
/*----------------------------------------------------------------------------*/
int8_t demo_mcastSinkInit(void);

/*----------------------------------------------------------------------------*/
/*!
    \brief Configuration of the simple multicast example application.

    \return 0 - error, 1 - success
*/
/*----------------------------------------------------------------------------*/
uint8_t demo_mcastSinkConf(s_ns_t* pst_netStack);



#endif /* _DEMO_MCAST_DEMO_MCAST_SINK_H_ */
