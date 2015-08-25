/*
 * hexdump.c
 *
 *  Created on: Aug 25, 2015
 *      Author: grapsus Link: http://grapsus.net/blog/post/Hexadecimal-dump-in-C
 */

#include <stdio.h>
#include <stdint.h>
#include <ctype.h>
#include "hexdump.h"

#ifndef HEXDUMP_COLS
#define HEXDUMP_COLS 8
#endif

void hexdump(const void* p_buf, uint32_t l_len)
{
    unsigned int i, j;

    for( i = 0; i < l_len
            + ( ( l_len % HEXDUMP_COLS ) ? ( HEXDUMP_COLS
            - l_len % HEXDUMP_COLS ) : 0 ); i++ )
    {
        /* print offset */
        if( i % HEXDUMP_COLS == 0 )
        {
            printf( "0x%06x: ", i );
        }

        /* print hex data */
        if( i < l_len )
        {
            printf( "%02x ", 0xFF & ( (char*)p_buf )[i] );
        }
        else /* end of block, just aligning for ASCII dump */
        {
            printf( "   " );
        }

        /* print ASCII dump */
        if( i % HEXDUMP_COLS == ( HEXDUMP_COLS - 1 ) )
        {
            for( j = i - ( HEXDUMP_COLS - 1 ); j <= i; j++ )
            {
                if( j >= l_len ) /* end of block, not really printing */
                {
                    putchar( ' ' );
                }
                else if( isprint( ( (char*)p_buf )[j] ) ) /* printable char */
                {
                    putchar( 0xFF & ( (char*)p_buf )[j] );
                }
                else /* other char */
                {
                    putchar( '.' );
                }
            }
            putchar( '\n' );
        }
    }
}
