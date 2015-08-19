/**
 * @file    smartmac.c
 *
 * @author  PN
 *
 * @brief   X-MAC module
 */

#include <stddef.h>
#include <include.h>

typedef uint8_t SMARTMAC_STATE;

#define SMARTMAC_STATE_SLEEP        (SMARTMAC_STATE) ( 0u )
#define SMARTMAC_STATE_IDLE         (SMARTMAC_STATE) ( 1u )
#define SMARTMAC_STATE_SNIFF        (SMARTMAC_STATE) ( 2u )
#define SMARTMAC_STATE_RX           (SMARTMAC_STATE) ( 3u )
#define SMARTMAC_STATE_TX           (SMARTMAC_STATE) ( 4u )

static SMARTMAC_STATE State;

void SmartMac_Init()
{
    /* TODO Initialization code */

    /* TODO Register a periodic timer callback function handler. In this case,
     * SmartMac_Powerup acts as the timer interrupt handler */
}

void SmartMac_Powerup ()
{
    /* This function handles periodic channel scans.
     * To be specifically, the device wakes up periodically, listening to any
     * incoming SmartPreambles. If a valid SmartPreamble is not received during
     * this time, the device is set to sleep mode again and the radio is turned
     * off completely.
     * If a valid SmartPreamble is received, the device should process the frame
     * and determine if it should go to sleep until end of smart preamble
     * transmission or stay awake to reply with an ACK.
     * It should be noted that SYNC word of the radio shall be set in accordance
     * with SmartMac state.
     **/

    switch (State) {
        case SMARTMAC_STATE_SLEEP:
            /* Step1: turn radio on */
            RADIO_DRV->On (NULL);


            break;

        case SMARTMAC_STATE_IDLE:
            break;

        case SMARTMAC_STATE_SNIFF:
            break;

        case SMARTMAC_STATE_RX:
            break;

        case SMARTMAC_STATE_TX:
            break;

        default:
            break;
    }

}

void SmartMac_Send()
{
    /* TODO Step1: send timestamp/SmartPreamble to wake the recipient up */

    /* TODO Step2: wait for ACK in response to the SmartPreamble */

    /* TODO Step3: transmit data packet */

    /* TODO Step4: wait for ACK in response to the data packet */
}


void SmartMac_SendList()
{

}


void SmartMac_Input()
{

}


void SmartMac_On()
{
    RADIO_DRV->Off (NULL);
}


void SmartMac_Off()
{
    RADIO_DRV->On (NULL);
}
