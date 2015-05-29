/**
 * \addtogroup fake_radio
 * @{
 */
/*============================================================================*/
/*! \file   fake_radio.c

    \author Artem Yushev artem.yushev@hs-offenburg.de

    \brief  Fake radio transceiver based on UDP loopback.

   \version 0.0.1
*/
/*============================================================================*/

/*==============================================================================
                                 INCLUDE FILES
==============================================================================*/
#include "emb6.h"
#include "emb6_conf.h"
#include "fake_radio.h"
#include "evproc.h"
#include "etimer.h"
#include <fcntl.h>
#include <errno.h>
/*==============================================================================
                                     MACROS
==============================================================================*/
#define 	LOGGER_ENABLE		LOGGER_RADIO
#if			LOGGER_ENABLE 	== 	TRUE
#define 	LOGGER_SUBSYSTEM	"fradio"
#endif
#include	"logger.h"

/*==============================================================================
                                     ENUMS
==============================================================================*/

/*==============================================================================
                          VARIABLE DECLARATIONS
==============================================================================*/
		static	int 					sockfd;
		static	struct 	sockaddr_in 	servaddr;
		static	struct 	sockaddr_in 	cliaddr;
		static	uint8_t 				tmp_buf[PACKETBUF_SIZE];
		static 	struct 	etimer 			tmr;

/*==============================================================================
                                GLOBAL CONSTANTS
==============================================================================*/
const uint8_t mac_address[8] = {0x00, 0x00, 0x00, 0x00, 0x00, FAKE_MAC_2_BIT, 0x00, 0x00};
/*==============================================================================
                           LOCAL FUNCTION PROTOTYPES
==============================================================================*/
/* Radio transceiver local functions */
		static	int8_t 					_fradio_on(void);
		static	int8_t 					_fradio_off(void);
		static	int8_t 					_fradio_init(void);
		static	int8_t 					_fradio_send(const void *pr_payload, uint8_t c_len);
		static	void					_fradio_handler(c_event_t ev, p_data_t data);

/*==============================================================================
						 STRUCTURES AND OTHER TYPEDEFS
==============================================================================*/
const struct s_nsIf_t fradio_driver = {
		_fradio_init,
		_fradio_send,
		_fradio_on,
		_fradio_off,
};
/*==============================================================================
                                LOCAL FUNCTIONS
==============================================================================*/
static int8_t _fradio_init(void)
{
	uint32_t	env_s_ip;
	uint32_t	env_s_port;
	uint32_t	env_c_ip;
	uint32_t 	env_c_port;
	linkaddr_t 	un_addr;

	env_s_ip = inet_addr(FRADIO_S_IP);
	env_c_ip = inet_addr(FRADIO_C_IP);
	if (!env_s_ip || !env_c_ip){
		LOG_ERR("%s\n\r","FRADIO_S_IP or FRADIO_C_IP not found");
		exit(1);
	}
	LOG_INFO("serv ip is: %s\n", FRADIO_S_IP);
	LOG_INFO("cli ip is: %s\n", FRADIO_C_IP);

#if CLIENT & !SERVER
	env_s_port = atoi(FRADIO_OUTPORT_CLIENT);
	env_c_port = atoi(FRADIO_INPORT_CLIENT);
#elif SERVER & !CLIENT
	env_s_port = atoi(FRADIO_OUTPORT_SERVER);
	env_c_port = atoi(FRADIO_INPORT_SERVER);
#endif
	if (!env_s_port || !env_c_port){
		LOG_ERR("%s\n\r","OUTPORT or INPORT not found");
		exit(1);
	}
	LOG_INFO("serv port is: %d\n", env_s_port);
	LOG_INFO("cli port is: %d\n", env_c_port);

//	LOG_INFO("initial ll addr = %02X:%02X:%02X:%02X:%02X:%02X\n",uip_lladdr.addr[0],uip_lladdr.addr[1],uip_lladdr.addr[2],uip_lladdr.addr[3],uip_lladdr.addr[4],uip_lladdr.addr[5]);
	sockfd=socket(AF_INET,SOCK_DGRAM,0);

	memset(&servaddr,0,sizeof(servaddr));
	servaddr.sin_family = AF_INET;
	servaddr.sin_addr.s_addr=env_s_ip;
	servaddr.sin_port=htons(env_s_port);
	if (bind(sockfd,(struct sockaddr *)&servaddr,sizeof(servaddr)) < 0) {
		LOG_ERR("fail to create a socket: error %d\n",errno);
		exit(1);
	}
	memset(&cliaddr,0,sizeof(cliaddr));
	cliaddr.sin_family = AF_INET;
	cliaddr.sin_addr.s_addr=env_c_ip;
	cliaddr.sin_port=htons(env_c_port);
	int flags = fcntl(sockfd, F_GETFL, 0);
	if (flags < 0){
		LOG_ERR("%s\n\r","fcntl error");
		exit(1);
	}
	flags = (flags|O_NONBLOCK);
	fcntl(sockfd, F_SETFL, flags);
	LOG_INFO("%s\n\r","fake_radio driver was initialized");
	memcpy((void *)&un_addr.u8,  &mac_address, 8);
	memcpy(&uip_lladdr.addr, &un_addr.u8, 8);
	linkaddr_set_node_addr(&un_addr);
	LOG_INFO("MAC address %x:%x:%x:%x:%x:%x:%x:%x\n",	\
							un_addr.u8[0],un_addr.u8[1],\
							un_addr.u8[2],un_addr.u8[3],\
							un_addr.u8[4],un_addr.u8[5],\
							un_addr.u8[6],un_addr.u8[7]);
	/* Start the packet receive process */
	etimer_set(&tmr, 10, _fradio_handler);
	printf("set %p for %p callback\n\r",&tmr, &_fradio_handler);
  return 0;
} /* _fradio_init() */

/*---------------------------------------------------------------------------*/
static int8_t _fradio_send(const void *pr_payload, uint8_t c_len)
{
  int ret = sendto(sockfd, pr_payload, c_len, 0,
		  (struct sockaddr *)&cliaddr, sizeof(cliaddr));
  if(ret == -1){
	  LOG_ERR("%s\n\r","sendto");
	  return RADIO_TX_ERR;
  }
  else {
		uint8_t _ret = ret;
		uint8_t i=0;
		LOG_INFO("send msg with len = %d\n",ret);
//		while(_ret--){
//			printf("%02X", (uint8_t)(*((uint8_t *)pr_payload + i)));
//			i++;
//		}
//		LOG_RAW("\n");

		return RADIO_TX_OK;
  }
} /* _fradio_send() */

/*---------------------------------------------------------------------------*/
static int _fradio_read(void *buf, unsigned short buf_len)
{
	int ret = recv(sockfd, tmp_buf, buf_len, 0);
	if((ret == -1) && (errno != 11)){
		LOG_ERR("%s\n\r","receive error");
		return 0;
	}
	if(ret > 0){
			LOG_INFO("receive msg len = %d\n",ret);
			memcpy(buf,tmp_buf,ret);
			uint8_t _ret = ret;
			uint8_t i=0;
//			while(_ret--){
//				LOG_RAW("%02X", (uint8_t)(*((uint8_t *)tmp_buf + i)));
//				i++;
//			}
//			LOG_RAW("\r\n");
		return ret;
	}
	return 0;
} /* _fradio_read() */

/*---------------------------------------------------------------------------*/
static int8_t _fradio_on(void)
{
  return 0;
} /* _fradio_on() */

/*---------------------------------------------------------------------------*/
static int8_t _fradio_off(void)
{
  return 0;
}  /* _fradio_off() */

/*---------------------------------------------------------------------------*/
static void _fradio_handler(c_event_t ev, p_data_t data)
{
	uint8_t len = 0;
	if (etimer_expired(&tmr)) {
		if ((len = _fradio_read(packetbuf_dataptr(), PACKETBUF_SIZE)) > 0)	{
			packetbuf_set_datalen(len);
			//p_ns->lmac->input();
		}
		etimer_restart(&tmr);
	}
}



/*==============================================================================
                                API FUNCTIONS
==============================================================================*/
/** @} */
