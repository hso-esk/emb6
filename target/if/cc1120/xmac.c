/**
 * @file    xmac.c
 *
 * @author  PN
 *
 * @brief   X-MAC module
 */

#include <include.h>

void Xmac_Init()
{

}

void Xmac_Send()
{

}

void Xmac_SendList()
{

}

void Xmac_Input()
{

}

void Xmac_On()
{
    RADIO_DRV->Off (NULL);
}

void Xmac_Off()
{
    RADIO_DRV->On (NULL);
}
