/*
 * cc1120.c
 *
 *  Created on: Aug 18, 2015
 *      Author: phuongnguyen
 */

#include <radio.h>
#include <cc112x.h>


void cc112x_Init ();
void cc112x_On ();
void cc112x_Off ();
void cc112x_Send ();
void cc112x_Recv ();
void cc112x_Ioctl ();

const radio_drv_api CC112x_DRV = {
    "CC112x",
    cc112x_Init,
    cc112x_On,
    cc112x_Off,
    cc112x_Send,
    cc112x_Recv,
    cc112x_Ioctl,
};


void cc112x_Init ()
{
    /* use existing corresponding code segments */
}


void cc112x_On ()
{
    /* use existing corresponding code segments */
}


void cc112x_Off ()
{
    /* use existing corresponding code segments */
}


void cc112x_Send ()
{
    /* use existing corresponding code segments */
}


void cc112x_Recv ()
{
    /* use existing corresponding code segments */
}


void cc112x_Ioctl (IOCTRL cmd, RADIO_VAL *p_val, RADIO_ERR *p_err)
{
    switch (cmd) {
        case IOC_CMD_TXPOWER_SET:
            /* jump to existing corresponding function */
            break;

        case IOC_CMD_TXPOWER_GET:
            break;

        case IOC_CMD_SENS_SET:
            break;

        case IOC_CMD_SENS_GET:
            break;

        case IOC_CMD_RSSI_GET:
            break;

        case IOC_CMD_CCA_GET:
            break;

        case IOC_CMD_ANT_DIV_SET:
            break;

        case IOC_CMD_RF_SWITCH:
            break;

        case IOC_CMD_SYNC_SET:
            break;

        case IOC_CMD_SYNC_GET:
            break;

        default:
            /* unsupported commands are treated in same way */
            *p_err = RADIO_ERR_CMD_UNSUPPORTED;
            break;
    }
}
