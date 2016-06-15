#ifndef _DEMO_MCAST_DEMO_MCAST_ROOT_H_
#define _DEMO_MCAST_DEMO_MCAST_ROOT_H_

/*==============================================================================
                         FUNCTION PROTOTYPES OF THE API
==============================================================================*/

/*----------------------------------------------------------------------------*/
/*!
   \brief Initialization of the simple multicast example application.

*/
/*----------------------------------------------------------------------------*/
int8_t demo_threadRootInit(void);

/*----------------------------------------------------------------------------*/
/*!
    \brief Configuration of the simple multicast example application.

    \return 0 - error, 1 - success
*/
/*----------------------------------------------------------------------------*/
uint8_t demo_threadRootConf(s_ns_t* pst_netStack);


#endif /* _DEMO_MCAST_DEMO_MCAST_ROOT_H_ */
