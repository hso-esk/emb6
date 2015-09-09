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

void ringb_init( s_ringb_t *ac_buf )
{
    ac_buf->sz_tail = 0;
    ac_buf->sz_head = 0;
}

void ringb_pusha( s_ringb_t *ac_buf, ringb_atom_t data )
{
    /* Is ac_buf full? */
    if( ringb_full( ac_buf ) )
    {
        /* Is going to overwrite the oldest byte */
        /* Increase tail index */
        ac_buf->sz_tail = ( ( ac_buf->sz_tail + 1 ) & RB_MASK );
    }

    /* Place data in ac_buf */
    ac_buf->a_buf[ac_buf->sz_head] = data;
    ac_buf->sz_head = ( ( ac_buf->sz_head + 1 ) & RB_MASK );
}

void ringb_push( s_ringb_t *ac_buf, const ringb_atom_t* p_data,
        ringb_size_t size )
{
    /* Add bytes; one by one */
    ringb_size_t i;
    for( i = 0; i < size; i++ )
    {
        ringb_pusha( ac_buf, p_data[i] );
    }
}

ringb_size_t ringb_pulla( s_ringb_t *ac_buf, ringb_atom_t* p_data )
{
    if( ringb_empty( ac_buf ) )
    {
        /* No items */
        return 0;
    }

    *p_data = ac_buf->a_buf[ac_buf->sz_tail];
    ac_buf->sz_tail = ( ( ac_buf->sz_tail + 1 ) & RB_MASK );
    return 1;
}

ringb_size_t ringb_pull( s_ringb_t *ac_buf, ringb_atom_t* p_data,
        ringb_size_t len )
{
    if( ringb_empty( ac_buf ) )
    {
        /* No items */
        return 0;
    }

    char *data_ptr = p_data;
    ringb_size_t cnt = 0;
    while( ringb_pulla( ac_buf, data_ptr ) && cnt < len )
    {
        cnt++;
        data_ptr++;
    }
    return cnt;
}

ringb_size_t ringb_peek( s_ringb_t *ac_buf, ringb_atom_t *data,
        ringb_size_t index )
{
    if( index >= ringb_items( ac_buf ) )
    {
        /* No items at index */
        return 0;
    }

    /* Add index to pointer */
    ringb_size_t data_index = ( ( ac_buf->sz_tail + index ) & RB_MASK );
    *data = ac_buf->a_buf[data_index];
    return 1;
}

extern inline uint8_t ringb_empty( s_ringb_t *ac_buf );
extern inline uint8_t ringb_full( s_ringb_t *ac_buf );
extern inline uint8_t ringb_items( s_ringb_t *ac_buf );

