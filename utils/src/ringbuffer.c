/*============================================================================*/
/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2014 Anders Kalør
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */
/*============================================================================*/

#include "ringbuffer.h"
/**
 * @file
 * Implementation of ring ac_buf functions.
 */

void ringb_init( s_ringb_t *ps_rb )
{
    ps_rb->sz_tail = 0;
    ps_rb->sz_head = 0;
}

void ringb_pusha( s_ringb_t *ps_rb, ringb_atom_t data )
{
    /* Is ac_buf full? */
    if( ringb_full( ps_rb ) )
    {
        /* Is going to overwrite the oldest byte */
        /* Increase tail index */
        ps_rb->sz_tail = ( ( ps_rb->sz_tail + 1 ) & RB_MASK );
    }

    /* Place data in ac_buf */
    ps_rb->a_buf[ps_rb->sz_head] = data;
    ps_rb->sz_head = ( ( ps_rb->sz_head + 1 ) & RB_MASK );
}

void ringb_push( s_ringb_t *ps_rb, const ringb_atom_t* p_data,
        ringb_size_t size )
{
    /* Add bytes; one by one */
    ringb_size_t i;
    for( i = 0; i < size; i++ )
    {
        ringb_pusha( ps_rb, p_data[i] );
    }
}

uint8_t ringb_pulla( s_ringb_t *ps_rb, ringb_atom_t* p_data )
{
    if( ringb_empty( ps_rb ) )
    {
        /* No items */
        return 0;
    }

    *p_data = ps_rb->a_buf[ps_rb->sz_tail];
    ps_rb->sz_tail = ( ( ps_rb->sz_tail + 1 ) & RB_MASK );
    return 1;
}

ringb_size_t ringb_pull( s_ringb_t *ps_rb, ringb_atom_t* p_data,
                         ringb_size_t len )
{
    if( ringb_empty( ps_rb ) )
    {
        /* No items */
        return 0;
    }

    ringb_atom_t *data_ptr = p_data;
    ringb_size_t cnt = 0;
    while( ringb_pulla( ps_rb, data_ptr ) && cnt < len )
    {
        cnt++;
        data_ptr++;
    }
    return cnt;
}

uint8_t ringb_peek( s_ringb_t *ps_rb, ringb_atom_t *data,
                         ringb_size_t index )
{
    if( index >= ringb_items( ps_rb ) )
    {
        /* No items at index */
        return 0;
    }

    /* Add index to pointer */
    ringb_size_t data_index = ( ( ps_rb->sz_tail + index ) & RB_MASK );
    *data = ps_rb->a_buf[data_index];
    return 1;
}

extern inline uint8_t ringb_empty( s_ringb_t *ps_rb );
extern inline uint8_t ringb_full( s_ringb_t *ps_rb );
extern inline uint8_t ringb_items( s_ringb_t *ps_rb );

