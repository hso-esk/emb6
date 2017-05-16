#include "tsch_null_adaptive.h"
#include "packetbuf.h"

static s_ns_t *pdllsec_netstk;

/*---------------------------------------------------------------------------*/
static void init(s_ns_t *p_netstk)
{
#if NETSTK_CFG_ARG_CHK_EN
  if (p_netstk == NULL) {
    return;
  }
#endif

  pdllsec_netstk = p_netstk;
  //pdllsec_netstk->dllc->ioctrl(NETSTK_CMD_RX_CBFNT_SET, (void *) dllsec_input, &err);
}
/*---------------------------------------------------------------------------*/
static void
send(mac_callback_t sent, void *ptr)
{

  packetbuf_set_attr(PACKETBUF_ATTR_FRAME_TYPE, FRAME802154_DATAFRAME);

  /*
   * Issue next lower layer to transmit the prepared packet
   */
//  pdllsec_netstk->dllc->send(sent, ptr);
  //NETSTACK_MAC.send(sent, ptr);


  /* inform upper layer of the TX status */
 // dllsec_cbTx(p_arg, &err);

/****************************/

}
/*---------------------------------------------------------------------------*/
static void input(void)
{
   pdllsec_netstk->hc->input();
}

/*---------------------------------------------------------------------------*/
static int  dllsec_onFrameCreated(void)
{
  return 0;
}

/*---------------------------------------------------------------------------*/
static uint8_t dllsec_getOverhead(void)
{
  return 0;
}

/*---------------------------------------------------------------------------*/

const struct s_nsDllsec dllsec_tsch_adaptive_driver = {
  "nullsec",
  init,
  send,
  dllsec_onFrameCreated,
  input,
  dllsec_getOverhead
};

/*-----------------------------------------------------------------------------------*/

static void init_dll(void *p_netstk, e_nsErr_t *p_err)
{
#if NETSTK_CFG_ARG_CHK_EN
  if (p_netstk == NULL) {
    return;
  }
#endif

  pdllsec_netstk = p_netstk;
}

const s_nsDLLC_t dll_tsch_adaptive_driver = {
  "nullsec",
  init_dll,
  NULL,
  NULL,
  NULL,
  NULL,
  NULL
};

const s_nsMAC_t mac_tsch_adaptive_driver = {
  "nullsec",
  init_dll,
  NULL,
  NULL,
  NULL,
  NULL,
  NULL
};
/*---------------------------------------------------------------------------*/

/** @} */
