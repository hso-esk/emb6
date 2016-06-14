#ifndef _DEMO_ROUTE_DEMO_ROUTE_ROOT_H_
#define _DEMO_ROUTE_DEMO_ROUTE_ROOT_H_

/*==============================================================================
                         FUNCTION PROTOTYPES OF THE API
==============================================================================*/

/*----------------------------------------------------------------------------*/
/*!
   \brief Initialization of the simple multicast example application.

*/
/*----------------------------------------------------------------------------*/
int8_t demo_routeRootInit(void);

/*----------------------------------------------------------------------------*/
/*!
    \brief Configuration of the simple multicast example application.

    \return 0 - error, 1 - success
*/
/*----------------------------------------------------------------------------*/
uint8_t demo_routeRootConf(s_ns_t* pst_netStack);


#endif /* _DEMO_ROUTE_DEMO_ROUTE_ROOT_H_ */
