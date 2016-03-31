//*****************************************************************************
//! @file       lcd_dogm128_6_alphabet.c
//! @brief      Font array for the DOGM128-6 LCD display.
//!
//! Revised     $Date: 2013-04-11 20:12:56 +0200 (to, 11 apr 2013) $
//! Revision    $Revision: 9713 $
//
//  Copyright (C) 2013 Texas Instruments Incorporated - http://www.ti.com/
//
//
//  Redistribution and use in source and binary forms, with or without
//  modification, are permitted provided that the following conditions
//  are met:
//
//    Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//
//    Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
//
//    Neither the name of Texas Instruments Incorporated nor the names of
//    its contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
//  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
//  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
//  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
//  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
//  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
//  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
//  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
//  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
//  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
//  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
//  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//****************************************************************************/
#ifndef LCD_EXCLUDE


/******************************************************************************
* INCLUDES
*/
#include <stdint.h>


//! Font data for 5x7 font
const uint8_t lcd_alphabet[] =
{
    0x00, //
    0x00, //
    0x5F, //  # #####
    0x00, //
    0x00, //

    0x00, //
    0x07, //      ###
    0x00, //
    0x07, //      ###
    0x00, //

    0x14, //    # #
    0x7F, //  #######
    0x14, //    # #
    0x7F, //  #######
    0x14, //    # #

    0x24, //   #  #
    0x2A, //   # # #
    0x7F, //  #######
    0x2A, //   # # #
    0x12, //    #  #

    0x23, //   #   ##
    0x13, //    #  ##
    0x08, //     #
    0x64, //  ##  #
    0x62, //  ##   #

    0x36, //   ## ##
    0x49, //  #  #  #
    0x55, //  # # # #
    0x22, //   #   #
    0x50, //  # #

    0x00, //
    0x05, //      # #
    0x03, //       ##
    0x00, //
    0x00, //

    0x00, //
    0x1C, //    ###
    0x22, //   #   #
    0x41, //  #     #
    0x00, //

    0x00, //
    0x41, //  #     #
    0x22, //   #   #
    0x1C, //    ###
    0x00, //

    0x08, //     #
    0x2A, //   # # #
    0x1C, //    ###
    0x2A, //   # # #
    0x08, //     #

    0x08, //     #
    0x08, //     #
    0x3E, //   #####
    0x08, //     #
    0x08, //     #

    0x00, //
    0x50, //  # #
    0x30, //   ##
    0x00, //
    0x00, //

    0x08, //     #
    0x08, //     #
    0x08, //     #
    0x08, //     #
    0x08, //     #

    0x00, //
    0x60, //  ##
    0x60, //  ##
    0x00, //
    0x00, //

    0x20, //   #
    0x10, //    #
    0x08, //     #
    0x04, //      #
    0x02, //       #

    0x3E, //   #####
    0x51, //  # #   #
    0x49, //  #  #  #
    0x45, //  #   # #
    0x3E, //   #####

    0x00, //
    0x42, //  #    #
    0x7F, //  #######
    0x40, //  #
    0x00, //

    0x42, //  #    #
    0x61, //  ##    #
    0x51, //  # #   #
    0x49, //  #  #  #
    0x46, //  #   ##

    0x21, //   #    #
    0x41, //  #     #
    0x45, //  #   # #
    0x4B, //  #  # ##
    0x31, //   ##   #

    0x18, //    ##
    0x14, //    # #
    0x12, //    #  #
    0x7F, //  #######
    0x10, //    #

    0x27, //   #  ###
    0x45, //  #   # #
    0x45, //  #   # #
    0x45, //  #   # #
    0x39, //   ###  #

    0x3C, //   ####
    0x4A, //  #  # #
    0x49, //  #  #  #
    0x49, //  #  #  #
    0x30, //   ##

    0x01, //        #
    0x71, //  ###   #
    0x09, //     #  #
    0x05, //      # #
    0x03, //       ##

    0x36, //   ## ##
    0x49, //  #  #  #
    0x49, //  #  #  #
    0x49, //  #  #  #
    0x36, //   ## ##

    0x06, //      ##
    0x49, //  #  #  #
    0x49, //  #  #  #
    0x29, //   # #  #
    0x1E, //    ####

    0x00, //
    0x36, //   ## ##
    0x36, //   ## ##
    0x00, //
    0x00, //

    0x00, //
    0x56, //  # # ##
    0x36, //   ## ##
    0x00, //
    0x00, //

    0x00, //
    0x08, //     #
    0x14, //    # #
    0x22, //   #   #
    0x41, //  #     #

    0x14, //    # #
    0x14, //    # #
    0x14, //    # #
    0x14, //    # #
    0x14, //    # #

    0x41, //  #     #
    0x22, //   #   #
    0x14, //    # #
    0x08, //     #
    0x00, //

    0x02, //       #
    0x01, //        #
    0x51, //  # #   #
    0x09, //     #  #
    0x06, //      ##

    0x32, //   ##  #
    0x49, //  #  #  #
    0x79, //  ####  #
    0x41, //  #     #
    0x3E, //   #####

    0x7E, //  ######
    0x11, //    #   #
    0x11, //    #   #
    0x11, //    #   #
    0x7E, //  ######

    0x7F, //  #######
    0x49, //  #  #  #
    0x49, //  #  #  #
    0x49, //  #  #  #
    0x36, //   ## ##

    0x3E, //   #####
    0x41, //  #     #
    0x41, //  #     #
    0x41, //  #     #
    0x22, //   #   #

    0x7F, //  #######
    0x41, //  #     #
    0x41, //  #     #
    0x22, //   #   #
    0x1C, //    ###

    0x7F, //  #######
    0x49, //  #  #  #
    0x49, //  #  #  #
    0x49, //  #  #  #
    0x41, //  #     #

    0x7F, //  #######
    0x09, //     #  #
    0x09, //     #  #
    0x01, //        #
    0x01, //        #

    0x3E, //   #####
    0x41, //  #     #
    0x41, //  #     #
    0x51, //  # #   #
    0x32, //   ##  #

    0x7F, //  #######
    0x08, //     #
    0x08, //     #
    0x08, //     #
    0x7F, //  #######

    0x00, //
    0x41, //  #     #
    0x7F, //  #######
    0x41, //  #     #
    0x00, //

    0x20, //   #
    0x40, //  #
    0x41, //  #     #
    0x3F, //   ######
    0x01, //        #

    0x7F, //  #######
    0x08, //     #
    0x14, //    # #
    0x22, //   #   #
    0x41, //  #     #

    0x7F, //  #######
    0x40, //  #
    0x40, //  #
    0x40, //  #
    0x40, //  #

    0x7F, //  #######
    0x02, //       #
    0x04, //      #
    0x02, //       #
    0x7F, //  #######

    0x7F, //  #######
    0x04, //      #
    0x08, //     #
    0x10, //    #
    0x7F, //  #######

    0x3E, //   #####
    0x41, //  #     #
    0x41, //  #     #
    0x41, //  #     #
    0x3E, //   #####

    0x7F, //  #######
    0x09, //     #  #
    0x09, //     #  #
    0x09, //     #  #
    0x06, //      ##

    0x3E, //   #####
    0x41, //  #     #
    0x51, //  # #   #
    0x21, //   #    #
    0x5E, //  # ####

    0x7F, //  #######
    0x09, //     #  #
    0x19, //    ##  #
    0x29, //   # #  #
    0x46, //  #   ##

    0x46, //  #   ##
    0x49, //  #  #  #
    0x49, //  #  #  #
    0x49, //  #  #  #
    0x31, //   ##   #

    0x01, //        #
    0x01, //        #
    0x7F, //  #######
    0x01, //        #
    0x01, //        #

    0x3F, //   ######
    0x40, //  #
    0x40, //  #
    0x40, //  #
    0x3F, //   ######

    0x1F, //    #####
    0x20, //   #
    0x40, //  #
    0x20, //   #
    0x1F, //    #####

    0x7F, //  #######
    0x20, //   #
    0x18, //    ##
    0x20, //   #
    0x7F, //  #######

    0x63, //  ##   ##
    0x14, //    # #
    0x08, //     #
    0x14, //    # #
    0x63, //  ##   ##

    0x03, //       ##
    0x04, //      #
    0x78, //  ####
    0x04, //      #
    0x03, //       ##

    0x61, //  ##    #
    0x51, //  # #   #
    0x49, //  #  #  #
    0x45, //  #   # #
    0x43, //  #    ##

    0x00, //
    0x00, //
    0x7F, //  #######
    0x41, //  #     #
    0x41, //  #     #

    0x02, //       #
    0x04, //      #
    0x08, //     #
    0x10, //    #
    0x20, //   #

    0x41, //  #     #
    0x41, //  #     #
    0x7F, //  #######
    0x00, //
    0x00, //

    0x04, //      #
    0x02, //       #
    0x01, //        #
    0x02, //       #
    0x04, //      #

    0x40, //  #
    0x40, //  #
    0x40, //  #
    0x40, //  #
    0x40, //  #

    0x00, //
    0x01, //        #
    0x02, //       #
    0x04, //      #
    0x00, //

    0x20, //   #
    0x54, //  # # #
    0x54, //  # # #
    0x54, //  # # #
    0x78, //  ####

    0x7F, //  #######
    0x48, //  #  #
    0x44, //  #   #
    0x44, //  #   #
    0x38, //   ###

    0x38, //   ###
    0x44, //  #   #
    0x44, //  #   #
    0x44, //  #   #
    0x20, //   #

    0x38, //   ###
    0x44, //  #   #
    0x44, //  #   #
    0x48, //  #  #
    0x7F, //  #######

    0x38, //   ###
    0x54, //  # # #
    0x54, //  # # #
    0x54, //  # # #
    0x18, //    ##

    0x08, //     #
    0x7E, //  ######
    0x09, //     #  #
    0x01, //        #
    0x02, //       #

    0x08, //     #
    0x14, //    # #
    0x54, //  # # #
    0x54, //  # # #
    0x3C, //   ####

    0x7F, //  #######
    0x08, //     #
    0x04, //      #
    0x04, //      #
    0x78, //  ####

    0x00, //
    0x44, //  #   #
    0x7D, //  ##### #
    0x40, //  #
    0x00, //

    0x20, //   #
    0x40, //  #
    0x44, //  #   #
    0x3D, //   #### #
    0x00, //

    0x00, //
    0x7F, //  #######
    0x10, //    #
    0x28, //   # #
    0x44, //  #   #

    0x00, //
    0x41, //  #     #
    0x7F, //  #######
    0x40, //  #
    0x00, //

    0x7C, //  #####
    0x04, //      #
    0x18, //    ##
    0x04, //      #
    0x78, //  ####

    0x7C, //  #####
    0x08, //     #
    0x04, //      #
    0x04, //      #
    0x78, //  ####

    0x38, //   ###
    0x44, //  #   #
    0x44, //  #   #
    0x44, //  #   #
    0x38, //   ###

    0x7C, //  #####
    0x14, //    # #
    0x14, //    # #
    0x14, //    # #
    0x08, //     #

    0x08, //     #
    0x14, //    # #
    0x14, //    # #
    0x18, //    ##
    0x7C, //  #####

    0x7C, //  #####
    0x08, //     #
    0x04, //      #
    0x04, //      #
    0x08, //     #

    0x48, //  #  #
    0x54, //  # # #
    0x54, //  # # #
    0x54, //  # # #
    0x20, //   #

    0x04, //      #
    0x3F, //   ######
    0x44, //  #   #
    0x40, //  #
    0x20, //   #

    0x3C, //   ####
    0x40, //  #
    0x40, //  #
    0x20, //   #
    0x7C, //  #####

    0x1C, //    ###
    0x20, //   #
    0x40, //  #
    0x20, //   #
    0x1C, //    ###

    0x3C, //   ####
    0x40, //  #
    0x30, //   ##
    0x40, //  #
    0x3C, //   ####

    0x44, //  #   #
    0x28, //   # #
    0x10, //    #
    0x28, //   # #
    0x44, //  #   #

    0x0C, //     ##
    0x50, //  # #
    0x50, //  # #
    0x50, //  # #
    0x3C, //   ####

    0x44, //  #   #
    0x64, //  ##  #
    0x54, //  # # #
    0x4C, //  #  ##
    0x44, //  #   #

    0x00, //
    0x08, //     #
    0x36, //   ## ##
    0x41, //  #     #
    0x00, //

    0x00, //
    0x00, //
    0x7F, //  #######
    0x00, //
    0x00, //

    0x00, //
    0x41, //  #     #
    0x36, //   ## ##
    0x08, //     #
    0x00, //

    // SPECIAL NON-ASCII SYMBOLS

    0x00, //
    0x3E, //   #####
    0x1C, //    ###
    0x08, //     #
    0x00  //

};
#endif // #ifndef LCD_EXCLUDE

