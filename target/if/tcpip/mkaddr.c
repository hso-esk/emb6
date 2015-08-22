/*
 * emb6 is licensed under the 3-clause BSD license. This license gives everyone
 * the right to use and distribute the code, either in binary or source code
 * format, as long as the copyright license is retained in the source code.
 *
 * The emb6 is derived from the Contiki OS platform with the explicit approval
 * from Adam Dunkels. However, emb6 is made independent from the OS through the
 * removal of protothreads. In addition, APIs are made more flexible to gain
 * more adaptivity during run-time.
 *
 * The license text is:
 *
 * Copyright (c) 2015,
 * Hochschule Offenburg, University of Applied Sciences
 * Laboratory Embedded Systems and Communications Electronics.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO
 * EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 * ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
/*
 * mkaddr.c
 *
 *\addtogroup tcpip_radio
 * @{
/*============================================================================*/
/*! \file   mkaddr.c

    \author Artem Yushev artem.yushev@hs-offenburg.de

    \brief  Originally derived from
            http://www.ccplusplus.com/2011/09/mkaddr-c-code.html

   \version 0.0.1
 */
/*============================================================================*/
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <ctype.h>
#include <errno.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>
#include "mkaddr.h"

char *strdup (const char *s)
{
    char *d = malloc (strlen (s) + 1);   // Space for length plus nul
    if (d == NULL) return NULL;          // No memory
    strcpy (d,s);                        // Copy the characters
    return d;                            // Return the new string
}

int mkaddr(void *p_addr,int *pi_addrlen, char *pc_strAddr, char *pc_proto)
{

    char*               pc_inpAddr  = strdup(pc_strAddr);
    char*               pc_hostPart = strtok(pc_inpAddr, ":" );
    char*               pc_portPart = strtok(NULL, "\n" );
    struct sockaddr_in* ps_ap       = (struct sockaddr_in *) p_addr;
    struct hostent*     ps_hp       = NULL;
    struct servent*     ps_sp       = NULL;
    char*               pc_cp;
    long                l_tmp;

    /*
     * Set input defaults:
     */
    if ( !pc_hostPart ) {
        pc_hostPart =  "*" ;
    }
    if ( !pc_portPart ) {
        pc_portPart =  "*" ;
    }
    if ( !pc_proto ) {
        pc_proto =  "tcp";
    }

    /*
     * Initialize the address structure:
     */
    memset(ps_ap,0,*pi_addrlen);
    ps_ap->sin_family = AF_INET;
    ps_ap->sin_port = 0;
    ps_ap->sin_addr.s_addr = INADDR_ANY;

    /*
     * Fill in the host address:
     */
    if ( strcmp(pc_hostPart, "*" ) == 0 ) {
        ; /* Leave as INADDR_ANY */
    }
    else if ( isdigit(*pc_hostPart) ) {
        /*
         * Numeric IP address:
         */
        ps_ap->sin_addr.s_addr =
                inet_addr(pc_hostPart);
        // if ( ap->sin_addr.s_addr == INADDR_NONE ) {
        if ( inet_aton(pc_hostPart,&ps_ap->sin_addr) != 1 ) {
            printf("222inet_aton failed\n\r");
            printf("pc_hostAddr %s\n\r",pc_hostPart);
            return -1;
        }
    }
    else {
        /*
         * Assume a hostname:
         */
        ps_hp = gethostbyname(pc_hostPart);
        if ( !ps_hp ) { 
            printf("gethostbyname\n\r");
            return -1;
        }
        if ( ps_hp->h_addrtype != AF_INET ) { 
            printf("ah_init failed\n\r");
            return -1;
        }
        ps_ap->sin_addr = * (struct in_addr *)ps_hp->h_addr_list[0];
    }

    /*
     * Process an optional port #:
     */
    if ( !strcmp(pc_portPart, "*" ) ) {
        /* Leave as wild (zero) */
    }
    else if ( isdigit(*pc_portPart) ) {
        /*
         * Process numeric port #:
         */
        l_tmp = strtol(pc_portPart,&pc_cp,10);
        if ( pc_cp != NULL && *pc_cp ) {
            return -2;
        }
        if ( l_tmp < 0L || l_tmp >= 32768 ) {
            return -2;
        }
        ps_ap->sin_port = htons( (short)l_tmp);
    }
    else {
        /*
         * Lookup the service:
         */
        ps_sp = getservbyname( pc_portPart, pc_proto);
        if ( !ps_sp ) {
            return -2;
        }
        ps_ap->sin_port = (short) ps_sp->s_port;
    }

    /*
     * Return address length
     */
    *pi_addrlen = sizeof *ps_ap;

    free(pc_inpAddr);
    return 0;
}
/** @} */
