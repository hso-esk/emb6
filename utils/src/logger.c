/**
 * \file   logger.c
 *
 * \author Artem Yushev
 *
 * \brief  There are 3 levels of LOG macros. Please try to follow advices
 *         LOG_XXXX  - to output "MUST HAVE" information. For example errors.
 *         LOG1_XXXX - to output supplementary information which is usefull
 *                     for debugging; e.g. entry points to the functions,
 *                     report about function process execution
 *         LOG2_XXXX - to output data which normally consumes a lot of space;
 *                     e.g., to printout packets or raw data and to annotate
 *                     this output
 *
 *         You can control logger behavior combining two methods
 *         INDIVIDUAL module:
 *              Following construction for your SOURCE file should be defined:
 *              #define     LOGGER_ENABLE        LOGGER_MODULENAME
 *              #include    "logger.h"
 *              Where <LOGGER_MODULENAME> can be defined externally
 *              Moreover in that external include you may specify LOGGER_LEVEL,
 *              which instructs logger
 *         GLOBAL:
 *              LOGGER_LEVEL specify up to what level messages will be shown
 *              for example if in your module you've written
 *              LOG2_HEXDUMP(<data to output>) and global LOGGER_LEVEL is set
 *              to 2 then you data won't be shown, but if you used
 *              LOG1_HEXDUMP(<data to output>) instead data will be printed out
 */

#define IPV6ADDR_LEN    16
#ifndef HEXDUMP_COLS
#define HEXDUMP_COLS    8
#endif

#include "logger.h"

void log_ip6addr( const uint8_t* pc_addr)
{
    uint16_t    i_addr_byte;
    uint16_t    i;
    uint32_t    f;
    for( i = 0, f = 0; i < IPV6ADDR_LEN; i += 2 )
    {
        i_addr_byte = ( pc_addr[i] << 8 ) + pc_addr[i + 1];
        if( i_addr_byte == 0 && f >= 0 )
        {
            if( f++ == 0 )
            {
                printf( "::" );
            }
        }
        else
        {
            if( f > 0 )
            {
                f = -1;
            }
            else if( i > 0 )
            {
                printf( ":" );
            }
            printf( "%x", i_addr_byte );
        }
    }
}

/* We need a printable char, so it's not an warning or error
 * "else if( isprint( ( (char*)p_buf )[j] ) )"
 *  */
#pragma GCC diagnostic ignored "-Wchar-subscripts"
void log_hexdump(const void* p_buf, uint32_t l_len)
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
