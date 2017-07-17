#include "tsch_null_adaptive.h"
#include "packetbuf.h"
#include "tsch.h"

static s_ns_t *pdllsec_netstk;

/*---------------------------------------------------------------------------*/
static void init(s_ns_t *p_netstk)
{
  e_nsErr_t err = NETSTK_ERR_NONE;

#if NETSTK_CFG_ARG_CHK_EN
  if (p_netstk == NULL) {
    return;
  }
#endif

  pdllsec_netstk = p_netstk;
  /* TODO to remove if we adapt tsch struct with mac driver struct */
  tschmac_driver.init(p_netstk,&err);
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
   //NETSTACK_MAC.send(sent, ptr);
  tschmac_driver.send(sent,ptr);

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


/**
 * @brief   Turn driver on
 *
 * @param   p_err       Pointer to a variable storing returned error code
 */
static void dllc_on(e_nsErr_t *p_err)
{
#if NETSTK_CFG_ARG_CHK_EN
  if (p_err == NULL) {
    return;
  }
#endif

  pdllsec_netstk->mac->on(p_err);
#if (NETSTK_CFG_AUTO_ONOFF_EN == TRUE)
  dllc_isOn = TRUE;
#endif
}

static void mac_on(e_nsErr_t *p_err)
{
#if NETSTK_CFG_ARG_CHK_EN
  if (p_err == NULL) {
    return;
  }
#endif

  pdllsec_netstk->phy->on(p_err);
#if (NETSTK_CFG_AUTO_ONOFF_EN == TRUE)
  dllc_isOn = TRUE;
#endif
}

const s_nsDLLC_t dll_tsch_adaptive_driver = {
  "nullsec",
  init_dll,
  dllc_on,
  NULL,
  NULL,
  NULL,
  NULL
};

const s_nsMAC_t mac_tsch_adaptive_driver = {
  "nullsec",
  init_dll,
  mac_on,
  NULL,
  NULL,
  NULL,
  NULL
};
/*---------------------------------------------------------------------------*/

/** @} */
