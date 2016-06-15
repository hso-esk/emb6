//------------------------------------------------------------------------------
// TI Confidential - NDA Restrictions
//
// Copyright (c) 2014 Texas Instruments, Inc.
//
//    This is an unpublished work created in the year stated above.
//    Texas Instruments owns all rights in and to this work and
//    intends to maintain and protect it as an unpublished copyright.
//    In the event of either inadvertent or deliberate publication,
//    the above stated date shall be treated as the year of first
//    publication. In the event of such publication, Texas Instruments
//    intends to enforce its rights in the work under the copyright
//    laws as a published work.
//
//------------------------------------------------------------------------------
//
// This file is auto-generated and should not be edited 
// Generated Fri Mar  6 12:13:39 2015
//

// This file implements patches for the MCE on CC26xx
// It should only be included from ONE source file to avoid duplicated constant arrays


#ifndef _APPLY_WMBUS_TMODE_MCE_PATCH_H
#define _APPLY_WMBUS_TMODE_MCE_PATCH_H

#include <stdint.h>

#ifndef MCE_PATCH_TYPE
#define MCE_PATCH_TYPE static const uint32_t
#endif

#ifndef PATCH_FUN_SPEC
#define PATCH_FUN_SPEC static inline
#endif

#ifndef RFC_MCERAM_BASE
#define RFC_MCERAM_BASE 0x21008000
#endif

MCE_PATCH_TYPE patchWmbusTmodeMce[512] = { 
   0x2fcf60b5,
   0x04073f9d,
   0x00000000,
   0x000c000f,
   0x00040401,
   0x003f0fff,
   0x002c5555,
   0x00180004,
   0x00470031,
   0x007a0061,
   0x00a80092,
   0x00c700ba,
   0x00d400d0,
   0x004b00d6,
   0x00d60037,
   0x00d000d4,
   0x00ba00c7,
   0x009200a8,
   0x0061007a,
   0x00310047,
   0x00040018,
   0x0037004b,
   0x071f070a,
   0x07480733,
   0x0771075c,
   0x079a0785,
   0x07c307ae,
   0x07ec07d7,
   0x08140800,
   0x083d0829,
   0x08660852,
   0x088f087b,
   0x08b808a4,
   0x08e108cd,
   0x001608f6,
   0x001c002c,
   0x000e0034,
   0x001a0026,
   0x000d0032,
   0x00190025,
   0x000b0031,
   0x00130023,
   0x00060029,
   0x0006000e,
   0x0002000e,
   0x00040008,
   0x0006000e,
   0x0006000a,
   0x0002000c,
   0x00040008,
   0x0006000c,
   0x0006000e,
   0x0002000e,
   0x0000000e,
   0x0006000e,
   0x0006000a,
   0x0002000e,
   0x0006000a,
   0x0007000e,
   0x0007000f,
   0x0003000d,
   0x00050009,
   0x0001000d,
   0x0007000f,
   0x0001000f,
   0x0005000f,
   0x0007000f,
   0x0007000b,
   0x0003000b,
   0x0007000b,
   0x0007000b,
   0x0007000f,
   0x0003000f,
   0x0007000f,
   0x0003000f,
   0x3d1f0007,
   0x00000000,
   0x000f0400,
   0x03840000,
   0x40f4000b,
   0x80000043,
   0x06700801,
   0x091e0000,
   0x00040514,
   0x02000000,
   0x00613e10,
   0x842f0000,
   0x027f3030,
   0xaaaaaaaa,
   0xaaaaaaaa,
   0x72200000,
   0xa32d7248,
   0x73057303,
   0x73047203,
   0x72047306,
   0x72767376,
   0x8001c7c0,
   0x90010001,
   0x08019010,
   0x720c9001,
   0x720e720d,
   0x7100b0c0,
   0xa0c0b0f0,
   0x81327218,
   0x39521020,
   0x00200670,
   0x11011630,
   0x6c011401,
   0x60ed60ea,
   0x619f615e,
   0x60ea60ea,
   0x60eb60ea,
   0x72201210,
   0x7310730f,
   0x81817311,
   0x91800010,
   0x60cab070,
   0x67ef60e0,
   0xc03060e0,
   0xc9516720,
   0xc470c282,
   0x6f131820,
   0x16116e23,
   0x68f31612,
   0x9ab07830,
   0x9ac07840,
   0x9ad07850,
   0x9ae08300,
   0xc5a0c482,
   0x41081820,
   0x6e231203,
   0x69051612,
   0x816060e0,
   0x81409490,
   0x2a703980,
   0x16111001,
   0x84448432,
   0xc0f5c0f3,
   0x1c01c200,
   0xc100412d,
   0x41231c10,
   0x10134d25,
   0x18301803,
   0x1a101a13,
   0x69203912,
   0x13f3612d,
   0x13f3612d,
   0xc1001015,
   0x1a151850,
   0x39141a10,
   0xb0d8692b,
   0xb1087100,
   0xb200a0d8,
   0xb003b480,
   0xb002b013,
   0x7229b012,
   0x7100b0d0,
   0x8140b100,
   0x71009290,
   0x8140b100,
   0x453d22f0,
   0x1c0313f0,
   0x92934149,
   0x71009492,
   0x9295b100,
   0x71009494,
   0xb0d0b100,
   0x7000a480,
   0xc030a0d1,
   0xc0409760,
   0xb0f19780,
   0x7100b0c1,
   0xa0c1b0f1,
   0xa0037276,
   0x7000a002,
   0x7310730f,
   0x6720c040,
   0x91c0c100,
   0xb4836509,
   0xb0c3b0f3,
   0xc030b484,
   0xb10191c0,
   0x8ad5b0d1,
   0x39553985,
   0x41730615,
   0x809012f5,
   0x45882230,
   0x085081a0,
   0x1401c451,
   0xc0636f12,
   0x41802252,
   0x61811211,
   0x92c113f1,
   0x7100b101,
   0x1a133112,
   0x6173457c,
   0xb101a0d1,
   0x6550a0c3,
   0xb88c60e0,
   0x89a18180,
   0x31813921,
   0x91810001,
   0x6720c050,
   0x72767376,
   0x72067248,
   0x72047202,
   0x73067305,
   0x123060e0,
   0xb32d91b0,
   0x6720c060,
   0xb006b0f8,
   0xb004b016,
   0xb002b014,
   0x7810b012,
   0x90509030,
   0x90407820,
   0xb2059060,
   0xb0f2b486,
   0xc000b0c2,
   0x97609780,
   0x93008ae0,
   0x9310c000,
   0x93b0c030,
   0xa0c1b069,
   0xb0f2b0f1,
   0x12008441,
   0x08011a10,
   0x1012c0f0,
   0x311e0612,
   0x3911142e,
   0x78cc69c7,
   0x6720c070,
   0x6706120d,
   0x120f120a,
   0x120b1a1f,
   0xc0f01a1b,
   0x97801660,
   0x7100b0f2,
   0x8090b88d,
   0x45fc2210,
   0x45e91e2a,
   0x39708af0,
   0x140b311b,
   0x41fc1ccb,
   0x821061f1,
   0x41ef2220,
   0x9760c030,
   0xc00061f1,
   0x8b109760,
   0x08018b21,
   0x41f82271,
   0x61fb120a,
   0x41f61e3a,
   0x61da161a,
   0x46012210,
   0x6720c080,
   0xc0906203,
   0xc0006720,
   0x97609780,
   0xb0f1a0c1,
   0x93b0c070,
   0x6720c0a0,
   0xc0097860,
   0xc00bc00a,
   0x66edc00c,
   0x3d5a3d59,
   0x8940b889,
   0x3d803180,
   0x14107891,
   0x8ae83d30,
   0x93081808,
   0x7880b069,
   0xc02093b0,
   0x1cbc9310,
   0x79b04e32,
   0x4a3f18b0,
   0x18b01aa0,
   0x10bd4e4f,
   0x66e510c3,
   0x1c0579c0,
   0x624f4e3f,
   0x18c07aa0,
   0x1aa04a47,
   0x4e4f18c0,
   0x10c310bd,
   0x7ab066e5,
   0x4e471c05,
   0xc0ec624f,
   0x66d0c0d0,
   0x1ac510d5,
   0x185d120d,
   0x10cb6251,
   0xc0d0c1dc,
   0x10d566d0,
   0x185d120d,
   0xc00d6251,
   0xd0b06251,
   0x6720986d,
   0x120b6706,
   0xc00a1a1b,
   0xc007c006,
   0x8ad0c009,
   0x39503980,
   0x46610610,
   0x78a878b9,
   0x7872048e,
   0xb0f2c0f0,
   0x6a657100,
   0x93b0c030,
   0x78d0b069,
   0xc0309780,
   0xb0f29760,
   0x80907100,
   0x46902210,
   0x1e2ab88d,
   0x8af0467f,
   0x311b3970,
   0x048b140b,
   0x42951ceb,
   0x1e026285,
   0x1a124283,
   0xc0706285,
   0x8b1093b0,
   0x08018b21,
   0x428c2271,
   0x628f120a,
   0x428a1e3a,
   0x626f161a,
   0xd0c08308,
   0x67209868,
   0x161a61b5,
   0xc00ec00b,
   0xb074b201,
   0x7100b0f2,
   0x1e2ab88d,
   0x8af046a9,
   0x311b3970,
   0x161e140b,
   0x46bf1e6e,
   0x120e1216,
   0x106662bf,
   0xc55042b4,
   0x14b0089b,
   0x10116f01,
   0xc0069191,
   0x62bfc017,
   0x42bf1077,
   0xc00b164f,
   0x81501207,
   0x42bf1e00,
   0x4ebf1cf0,
   0x8b1062ca,
   0x08018b21,
   0x42c62271,
   0x62c9120a,
   0x42c41e3a,
   0x629a161a,
   0xa0c2a205,
   0xa0c1b0f2,
   0x618db0f1,
   0xc0011a10,
   0x6fcdc0c2,
   0x10b3161c,
   0x105166e5,
   0x6fcd1a10,
   0x66e5161c,
   0x4ae01c15,
   0x62e36ad9,
   0x10021051,
   0x102d6ad9,
   0x1cd37000,
   0x10354aea,
   0x700018d5,
   0x183510d5,
   0xb0f27000,
   0xb88e7100,
   0x10128911,
   0x12014ef5,
   0x31311821,
   0x1c1b1419,
   0x101b4efa,
   0x10128921,
   0x12014eff,
   0x31311821,
   0x1c1c141a,
   0x101c4f04,
   0x70006aed,
   0x16c1c2c1,
   0x6fd1141d,
   0x9721101d,
   0x3150c1b0,
   0x10218332,
   0x39520671,
   0x14210632,
   0x43171611,
   0x97303010,
   0x6b191270,
   0x874092dd,
   0x875094a0,
   0x700094b0,
   0x88409850,
   0x47212200,
   0x7000b830,
   0x00000000,
   0x00000000,
   0x00000000,
   0x00000000,
   0x00000000,
   0x00000000,
   0x00000000,
   0x00000000,
   0x00000000,
   0x00000000,
   0x00000000,
   0x00000000,
   0x00000000,
   0x00000000,
   0x00000000,
   0x00000000,
   0x00000000,
   0x00000000,
   0x00000000,
   0x00000000,
   0x00000000,
   0x00000000,
   0x00000000,
   0x00000000,
   0x00000000,
   0x00000000,
   0x00000000,
   0x00000000,
   0x00000000,
   0x00000000,
   0x00000000,
   0x00000000,
   0x00000000,
   0x00000000,
   0x00000000,
   0x00000000,
   0x00000000,
   0x00000000,
   0x00000000,
   0x00000000,
   0x00000000,
   0x00000000,
   0x00000000,
   0x00000000,
   0x00000000,
   0x00000000,
   0x00000000,
   0x00000000,
   0x00000000,
   0x00000000,
   0x00000000,
   0x00000000,
   0x00000000,
   0x00000000,
   0x00000000,
   0x00000000,
   0x00000000,
   0x00000000,
   0x00000000,
   0x00000000,
   0x00000000,
   0x00000000,
   0x00000000,
   0x00000000,
   0x00000000,
   0x00000000,
   0x00000000,
   0x00000000,
   0x00000000,
   0x00000000,
   0x00000000,
   0x00000000,
   0x00000000,
   0x00000000,
   0x00000000,
   0x00000000,
   0x00000000,
   0x00000000,
   0x00000000,
   0x00000000,
   0x00000000,
   0x00000000,
   0x00000000,
   0x00000000,
   0x00000000,
   0x00000000,
   0x00000000,
   0x00000000,
   0x00000000,
   0x00000000,
   0x00000000,
   0x00000000,
   0x00000000,
   0x00000000,
   0x00000000,
   0x00000000,
   0x00000000,
   0x00000000,
   0x00000000,
   0x00000000,
   0xc1020000,
   0xc0013162,
   0x1e008150,
   0x1a1043f7,
   0x102063f9,
   0x6f131a10,
   0x16116e23,
   0x6bf91612,
   0x00007000
};

PATCH_FUN_SPEC void enterWmbusTmodeMcePatch(void)
{
#ifdef __PATCH_NO_UNROLLING
   uint32_t i;
   for (i = 0; i < 512; i++) {
      HWREG(RFC_MCERAM_BASE + 4 * i) = patchWmbusTmodeMce[i];
   }
#else
   const uint32_t *pS = patchWmbusTmodeMce;
   volatile unsigned long *pD = &HWREG(RFC_MCERAM_BASE);
   uint32_t t1, t2, t3, t4, t5, t6, t7, t8;
   uint32_t nIterations = 64;

   do {
      t1 = *pS++;
      t2 = *pS++;
      t3 = *pS++;
      t4 = *pS++;
      t5 = *pS++;
      t6 = *pS++;
      t7 = *pS++;
      t8 = *pS++;
      *pD++ = t1;
      *pD++ = t2;
      *pD++ = t3;
      *pD++ = t4;
      *pD++ = t5;
      *pD++ = t6;
      *pD++ = t7;
      *pD++ = t8;
   } while (--nIterations);
#endif
}
/* -------- ASSEMBLY CODE ----------------------------------------------
;;; --------------------------------------------------------------------------------
;;; MCE RAM BANK 
;;;
;;; Contains SOFT TX/RX Mode
;;; WMBUS TMODE Only
;;; - Fixed 100kbps BaudRate (patch is controlling MDMBAUD registers)
;;; - Fixed 50 kHz Deviation 
;;; - Fixed 220/260 kHz RX BW (patch is controlling CHFIBW)
;;; - Fixed 500kHz RX IF because using DEMFIFE to calculate 
;;; - Fixed 3 out of 6 coding implemented.
;;; - Support only MSB first. Defined by TMODE protocol.
;;;
;;; API setting:	 
;;; The total preamble (header + synchronisation) chips sequence for this mode shall be n*(01) 0000111101 with n >=19.
;;;     -> Number of preambles bits are 38 and number of synchronisation bits are 10 which in total gives 48 bits or 6 bytes header.
;;; The user shall program 4 byte preambles and 2 byte SyncWord.
;;; The user shall program 2 byte SyncWord, meaning that 6 preamble bits must be part of the sync word.
;;; The user shall MSB bit first. 3 out of 6 coding is not supported with LSB first. Then bits in FIFO is swapped and Encoding/Decoding table must be updated.
;;; The user shall program SyncWord = 0x0000543D;   
;;; 
;;; Testvector for 3 out of 6 coding
;;; Hex string to code (from sn en 13757-4:2005):	
;;;	0F 44 AE 0C	010110 101001 011100 011100 100110 110010 010110 110100
;;;	78 56 34 12	010011 101100 011001 011010 001011 011100 001101 001110
;;;	01 07 44 47	010110 001101 010110 010011 011100 011100 011100 010011
;;;	78 0B 13 43	010011 101100 010110 100011 001101 001011 011100 001011
;;;	65 87 1E 6D	011010 011001 101100 010011 001101 110010 011010 110001
;;; --------------------------------------------------------------------------------
        
__DBG__ .assign 1
.include "dbg.asm"

.DEFINE CMD_OK 1
.DEFINE CMD_ERR 2

;;; --------------------------------------------------------------------------------
;;; Includes & Defines
;;; 
.INCLUDE "../inc/mdm_regs.asm"
.INCLUDE "../inc/mdmconf_wmbus_tmode_rom.asm"
        
;;; --------------------------------------------------------------------------------
;;; Memory Map
;;; 
.ORG 0
	jmp START_PROCESS
        
.DEFINE LMD_DATA_SPACE      1           ;; LMD Data Space (Tables)
.DEFINE MDMCONF_WMBUS_TMODE 149         ;; Configuration Information
.DEFINE MAIN                181         ;; Main Program
.DEFINE COMMONLIBS	   1007         ;; Common Libraries        

.ORG COMMONLIBS        
.include "mce_commonlib.asm"

;;; --------------------------------------------------------------------------------
;;; GenFSK Data
;;; 
.ORG LMD_DATA_SPACE
	
DEMENABLE0_RX_GENFSK:
        .DATA 0x2FCF            ; DEMENABLE0 settings (felp, frac,fidc,chfi,bdec,iqmc,mge2,codc,cmix)
DEMENABLE1_RX_GENFSK:
        .DATA 0x3F9D            ; DEMENABLE1 settings (do not start stim)
        
MDMSPAR0_REG_DEFAULT:		; Default value for MDMSPAR0, Use Register Override to configure FB2PLL
	.DATA 0x0407            ; MDMSPAR0[15] Enable preamble detect duty cycle mode
                                ; MDMSPAR0[14] Enable carrier sense duty cycle mode
                                ; MDMSPAR0[13] Enable feedback to pll mode
                                ; MDMSPAR0[12] Disable strick sync word check
                                ; MDMSPAR0[11:8] is FOC gain, 
                                ; MDMSPAR0[7:0] is FOC limit
				; FOC limit is has equal unit as DEMMISC0.CMIXN (i.e. 10 means ~20 kHz)
MDMSPAR1_REG_DEFAULT:		; Default value for MDMSPAR1, Use Register Override to configure
	.DATA 0x0000            ; MDMSPAR1[15:12] Active time in cs mode
                                ; MDMSPAR1[11:7]  Active time in sniff mode and pqt mode, in symbol periods
                                ; MDMSPAR1[6:0]  Sleep time in sniff mode and pqt mode, in symbol periods
MDMSPAR2_REG_DEFAULT:		; Default value for MDMSPAR2, Use Register Override to configure
	.DATA 0x0000            ; MDMSPAR2[15]   PQT SNIFF MODE ENABLE
                                ; MDMSPAR2[14]   
                                ; MDMSPAR2[13]   TX/RX: Invert data
                                ; MDMSPAR2[11:8] Not in use
                                ; MDMSPAR2[7:0]  Not in use

SYMBOLRATE_ESTIMATION_TIME:	; Symbol Rate Estimation Time
	.DATA 0x000F		; Unit [4Baud Sample Period], 4Baud tick, 4 symbols period
		                             
DEMFIFE0_SETTLE_TIME:		; Digital Filter Settle time for DEMFIFE0 after updating RX IF / CMIXN and cleared DEMFIFE0 
	.DATA 0x000C		; Unit [3Baud Sample Period]... Due to MIPS challenge does not count in sample phase 2, so 32 means ~5.33 Symbol period 

DEMFIFE0_BYPASS:		; TBD:Maybe be removed as long as DEMFIFE0 does a reset after DEMFIFE0 settle timeout
	.DATA 0x0401		; 

FB2PLL_ROUNDING:		; Magic value for calculating CMIXN update. May be caused by CMIXN resolution
	.DATA 0x0004		; +4
	
SYNCWORD_DETECT_MASK:		; Program 16 bit sync, but Sync Detect algorithm use 12 bit sync detect 
	.DATA 0x0FFF		; 12 bit syncword MASK.....

CHIP_SIGNBIT_INVERT_MASK:	; MASK for 6 bit invert chip invert....
	.DATA 0x003F

PREAMBLE_PATTERN:		; Preamble Pattern
	.DATA 0x5555		; 16 bit

SYNCSEARCH_TIMEOUT_PERIOD:	; Restart Preamble search if no sync word present
	.DATA 0x002C		; Unit [Symbol Period] 32 symbol timeout

;;; EARLY_CORR Table look up for finding symbol rate error 
;;; The table is made from Simulation (MAX PEAK algorithm is used) 
EARLY_CORR_COMP_LUT:
	.DATA    4 		 ;  ...   0%, Integration length is 32 symbols ...
	.DATA   24 		 ;  ...   1%, Integration length is 32 symbols ...
	.DATA   49 		 ;  ...   2%, Integration length is 32 symbols ...
	.DATA   71 		 ;  ...   3%, Integration length is 32 symbols ...
	.DATA   97 		 ;  ...   4%, Integration length is 32 symbols ...
	.DATA  122 		 ;  ...   5%, Integration length is 32 symbols ...
	.DATA  146 		 ;  ...   6%, Integration length is 32 symbols ...
	.DATA  168 		 ;  ...   7%, Integration length is 32 symbols ...
	.DATA  186 		 ;  ...   8%, Integration length is 32 symbols ...
	.DATA  199 		 ;  ...   9%, Integration length is 32 symbols ...
	.DATA  208 		 ;  ...  10%, Integration length is 32 symbols ...
	.DATA  212 		 ;  ...  11%, Integration length is 32 symbols ...
	.DATA  214 		 ;  ...  12%, Integration length is 32 symbols ...


;;; Threshold for setting symbol rate estimate to zero. Receiver handle 3% data error, but not more.
;;; Early / Late algorithm is not able to distinguish between +/- data rate when data rate error is less than 3.
;;; (Positive error)
EARLY_COARSE_THRESHOLD:		; EARLY_CORR > EARLY_COARSE_THRESHOLD is reliable
	.DATA 75		;
EARLY_DISTANCE_THRESHOLD:	; Problem area: use distance treshold: ABS(EARLY_CORR-LATE_CORR) > EARLY_DISTANCE_THRESHOLD ?
	.DATA 55		;

;;; LATE_CORR Table look up for finding symbol rate error 
;;; The table is made from Simulation (MAX PEAK algorithm is used) 
LATE_CORR_COMP_LUT:
	.DATA  214 		 ;  ...  12%, Integration length is 32 symbols ...
	.DATA  212 		 ;  ...  11%, Integration length is 32 symbols ...
	.DATA  208 		 ;  ...  10%, Integration length is 32 symbols ...
	.DATA  199 		 ;  ...   9%, Integration length is 32 symbols ...
	.DATA  186 		 ;  ...   8%, Integration length is 32 symbols ...
	.DATA  168 		 ;  ...   7%, Integration length is 32 symbols ...
	.DATA  146 		 ;  ...   6%, Integration length is 32 symbols ...
	.DATA  122 		 ;  ...   5%, Integration length is 32 symbols ...
	.DATA   97 		 ;  ...   4%, Integration length is 32 symbols ...
	.DATA   71 		 ;  ...   3%, Integration length is 32 symbols ...
	.DATA   49 		 ;  ...   2%, Integration length is 32 symbols ...
	.DATA   24 		 ;  ...   1%, Integration length is 32 symbols ...
	.DATA    4 		 ;  ...   0%, Integration length is 32 symbols ...


;;; Threshold for continuing in earlylate mode after 14 baud periods due to a high LATE_THRESHOLD value
;;; Early / Late algorithm is not able to distinguish between +/- data rate when data rate error is less than 3.
;;; (Negative error)
LATE_COARSE_THRESHOLD:		; LATE_CORR > LATE_COARSE_THRESHOLD is reliable
	.DATA 75		;
LATE_DISTANCE_THRESHOLD:	; Problem area: use distance treshold: ABS(LATE_CORR-LATE_CORR) > LATE_DISTANCE_THRESHOLD ?
	.DATA 55		;

;;; The Baud Rate look up table for data rate error from -12% up to 12%
;;  Rs=rw/2^20*24e6/pre or rw = Rs*2^20/24e6*pre
MDMBAUD_VALUES_LUT:
	.DATA  0x070A 		 ;  ... -12%, rw=57664 ...
	.DATA  0x071F 		 ;  ... -11%, rw=58336 ...
	.DATA  0x0733 		 ;  ... -10%, rw=58976 ...
	.DATA  0x0748 		 ;  ...  -9%, rw=59648 ...
	.DATA  0x075C 		 ;  ...  -8%, rw=60288 ...
	.DATA  0x0771 		 ;  ...  -7%, rw=60960 ...
	.DATA  0x0785 		 ;  ...  -6%, rw=61600 ...
	.DATA  0x079A 		 ;  ...  -5%, rw=62272 ...
	.DATA  0x07AE 		 ;  ...  -4%, rw=62912 ...
	.DATA  0x07C3 		 ;  ...  -3%, rw=63584 ...
	.DATA  0x07D7 		 ;  ...  -2%, rw=64224 ...
	.DATA  0x07EC 		 ;  ...  -1%, rw=64896 ...
	.DATA  0x0800 		 ;  ...   0%, rw=65536 ...
	.DATA  0x0814 		 ;  ...   1%, rw=66176 ...
	.DATA  0x0829 		 ;  ...   2%, rw=66848 ...
	.DATA  0x083D 		 ;  ...   3%, rw=67488 ...
	.DATA  0x0852 		 ;  ...   4%, rw=68160 ...
	.DATA  0x0866 		 ;  ...   5%, rw=68800 ...
	.DATA  0x087B 		 ;  ...   6%, rw=69472 ...
	.DATA  0x088F 		 ;  ...   7%, rw=70112 ...
	.DATA  0x08A4 		 ;  ...   8%, rw=70784 ...
	.DATA  0x08B8 		 ;  ...   9%, rw=71424 ...
	.DATA  0x08CD 		 ;  ...  10%, rw=72096 ...
	.DATA  0x08E1 		 ;  ...  11%, rw=72736 ...
	.DATA  0x08F6 		 ;  ...  12%, rw=73408 ...

	
;;; 3OUTOF6 Encoding Table Look Up
;;; 	WMBUS Tmode use MSB first, which is placed in LSB bit in TXFIFO when MSB first is set in Prop. API....
;;;     This table swappes MSB and LSB automatically (therefor it is a bit strange....)
;;;     So reading 0001b from FIFO actually means 1000b or 8 decimal. 
TMODE_3OUTOF6_ENCODING_LUT:	
	.DATA  0x0016 		 ;  ... 22 (decimal) and with 0 bit sent first  ...
	.DATA  0x002C 		 ;  ... 44 (decimal) and with 1 bit sent first  ...
	.DATA  0x001C 		 ;  ... 28 (decimal) and with 0 bit sent first  ...
	.DATA  0x0034 		 ;  ... 52 (decimal) and with 1 bit sent first  ...
	.DATA  0x000E 		 ;  ... 14 (decimal) and with 0 bit sent first  ...
	.DATA  0x0026 		 ;  ... 38 (decimal) and with 1 bit sent first  ...
	.DATA  0x001A 		 ;  ... 26 (decimal) and with 0 bit sent first  ...
	.DATA  0x0032 		 ;  ... 50 (decimal) and with 1 bit sent first  ...
	.DATA  0x000D 		 ;  ... 13 (decimal) and with 0 bit sent first  ...
	.DATA  0x0025 		 ;  ... 37 (decimal) and with 1 bit sent first  ...
	.DATA  0x0019 		 ;  ... 25 (decimal) and with 0 bit sent first  ...
	.DATA  0x0031 		 ;  ... 49 (decimal) and with 1 bit sent first  ...
	.DATA  0x000B 		 ;  ... 11 (decimal) and with 0 bit sent first  ...
	.DATA  0x0023 		 ;  ... 35 (decimal) and with 1 bit sent first  ...
	.DATA  0x0013 		 ;  ... 19 (decimal) and with 0 bit sent first  ...
	.DATA  0x0029 		 ;  ... 41 (decimal) and with 1 bit sent first  ...
	
	
;;; 3OUTOF6 Decoding Table Look Up 
;;; 	WMBUS Tmode use MSB first, which is placed in LSB bit in RXFIFO when MSB first is set in Prop. API....
;;;     This table swappes MSB and LSB automatically (therefor it is a bit strange....)
;;;     So reading 0001b actually means 1000b or 8 decimal. 
TMODE_3OUTOF6_DECODING_LUT:
	.DATA  0x0006 		 ;... 6 .... 
	.DATA  0x000E 		 ;... 14 .... 
	.DATA  0x0006 		 ;... 6 .... 
	.DATA  0x000E 		 ;... 14 .... 
	.DATA  0x0002 		 ;... 2 .... 
	.DATA  0x0008 		 ;... 8 .... 
	.DATA  0x0004 		 ;... 4 .... 
	.DATA  0x000E 		 ;... 14 .... 
	.DATA  0x0006 		 ;... 6 .... 
	.DATA  0x000A 		 ;... 10 .... 
	.DATA  0x0006 		 ;... 6 .... 
	.DATA  0x000C 		 ;... 12 .... 
	.DATA  0x0002 		 ;... 2 .... 
	.DATA  0x0008 		 ;... 8 .... 
	.DATA  0x0004 		 ;... 4 .... 
	.DATA  0x000C 		 ;... 12 .... 
	.DATA  0x0006 		 ;... 6 .... 
	.DATA  0x000E 		 ;... 14 .... 
	.DATA  0x0006 		 ;... 6 .... 
	.DATA  0x000E 		 ;... 14 .... 
	.DATA  0x0002 		 ;... 2 .... 
	.DATA  0x000E 		 ;... 14 .... 
	.DATA  0x0000 		 ;... 0 .... 
	.DATA  0x000E 		 ;... 14 .... 
	.DATA  0x0006 		 ;... 6 .... 
	.DATA  0x000A 		 ;... 10 .... 
	.DATA  0x0006 		 ;... 6 .... 
	.DATA  0x000E 		 ;... 14 .... 
	.DATA  0x0002 		 ;... 2 .... 
	.DATA  0x000A 		 ;... 10 .... 
	.DATA  0x0006 		 ;... 6 .... 
	.DATA  0x000E 		 ;... 14 .... 
	.DATA  0x0007 		 ;... 7 .... 
	.DATA  0x000F 		 ;... 15 .... 
	.DATA  0x0007 		 ;... 7 .... 
	.DATA  0x000D 		 ;... 13 .... 
	.DATA  0x0003 		 ;... 3 .... 
	.DATA  0x0009 		 ;... 9 .... 
	.DATA  0x0005 		 ;... 5 .... 
	.DATA  0x000D 		 ;... 13 .... 
	.DATA  0x0001 		 ;... 1 .... 
	.DATA  0x000F 		 ;... 15 .... 
	.DATA  0x0007 		 ;... 7 .... 
	.DATA  0x000F 		 ;... 15 .... 
	.DATA  0x0001 		 ;... 1 .... 
	.DATA  0x000F 		 ;... 15 .... 
	.DATA  0x0005 		 ;... 5 .... 
	.DATA  0x000F 		 ;... 15 .... 
	.DATA  0x0007 		 ;... 7 .... 
	.DATA  0x000B 		 ;... 11 .... 
	.DATA  0x0007 		 ;... 7 .... 
	.DATA  0x000B 		 ;... 11 .... 
	.DATA  0x0003 		 ;... 3 .... 
	.DATA  0x000B 		 ;... 11 .... 
	.DATA  0x0007 		 ;... 7 .... 
	.DATA  0x000B 		 ;... 11 .... 
	.DATA  0x0007 		 ;... 7 .... 
	.DATA  0x000F 		 ;... 15 .... 
	.DATA  0x0007 		 ;... 7 .... 
	.DATA  0x000F 		 ;... 15 .... 
	.DATA  0x0003 		 ;... 3 .... 
	.DATA  0x000F 		 ;... 15 .... 
	.DATA  0x0007 		 ;... 7 .... 
	.DATA  0x000F 		 ;... 15 .... 

	
;;; --------------------------------------------------------------------------------
;;; Start of the Program itself
.ORG MAIN
        
START_PROCESS:
        ;; Do hard initialization of all submodules of the modem
	outclr RFESEND		; Clear RFE send
        outclr MODCTRL          ; 
        outbclr DEMMISC2_MLSERUN,DEMMISC2 ;  Make sure MLSE can be stopped
        outset DEMENABLE0       ;
        outset DEMINIT0         ;
        outclr DEMENABLE0       ;
        outset DEMENABLE1       ;
        outset DEMINIT1         ;
        outclr DEMENABLE1       ;
        outset TIMCTRL          ;
        outclr TIMCTRL          ;
        lli    0x7C, r0         ; dont't touch fifo and topsm
        input  MDMENABLE, r1    ;
        or     r0, r1           ; 
        output r1, MDMENABLE    ;
        output r0, MDMINIT      ;
        xor    r0, r1           ;
        output r1,MDMENABLE     ;   
        outclr MCEEVENTMSK0     ;
        outclr MCEEVENTMSK1     ;
        outclr MCEEVENTMSK2     ;   
                  
;;; Move into Command processing
;;; --------------------------------------------------------------------------------
;;; Step 1. Main Command Processing Loop
        
CMD_PROC:
        outbset MCEEVENT0_MDMAPI_WR, MCEEVENTMSK0  ; Enable event 0 = MDMAPI write
        wait                             ; Waits until a command arrives
        outbset MCEEVENT0_MDMAPI_WR, MCEEVENTCLR0  ; Clear the event 0
        outbclr MCEEVENT0_MDMAPI_WR, MCEEVENTMSK0  ; Clear the event 0        
        outclr  MDMSTATUS       ; 
        input   MDMAPI, r2      ; Protocol ID [15:8] API [7:0] put together as [4:3][2:0]
        mov     r2, r0          ; Copy first into R0
        sr0     5, r2           ; Shift 5 positions R2
        and     7, r0           ; Clear all above bit 2 in R0
        or      r2, r0          ; Put them together
	add     3, r0           ; offset +3
	mov     pc, r1
	add     r0, r1
	jmp     (r1)
        jmp     MNOP_Entry        ; Jump tables Prot 0 Call 0 (MNOP)
        jmp     MCFG_GenFSK_Entry ; Jump tables Prot 0 Call 1 (MCFG) 
        jmp     MTX_WMBUS_TMODE   ; Jump tables Prot 0 Call 2 (MTX) 
        jmp     MRX_WMBUS_TMODE   ; Jump tables Prot 0 Call 3 (MRX) 
        jmp     MNOP_Entry        ; Jump tables Prot 0 Call 4 
        jmp     MNOP_Entry        ; Jump tables Prot 0 Call 5 
        jmp     MNOP_Entry        ; Jump tables Prot 0 Call 6 
        jmp     MROM_Dump_Entry   ; Jump tables Prot 0 Call 7 (MROMD)
;;; End of Command Processing, start of the commands themselves

;;; --------------------------------------------------------------------------------
;;; General end of command routine
;;; 
CMD_OK_END:
	mov     CMD_OK, r0	; Normal end of command
CPE_NOTIFY:        
	outclr	RFESEND		; Clear RFE send
        outset  MCEEVENTCLR0    ; Clear all events: NOTE, this will BREAK command pipelining!
        outset  MCEEVENTCLR1    ; Clear all events
        outset  MCEEVENTCLR2    ; Clear all events
        input   MDMSTATUS, r1   ;
        or      r1, r0          ; 
	output  r0, MDMSTATUS
	outbset 0, MCESTROBES0	; Notify CPE of completion
	jmp     CMD_PROC	; Ready for another command

;;; --------------------------------------------------------------------------------
;;; MNOP
;;; 
       
MNOP_Entry:
        jmp     CMD_OK_END

;;; --------------------------------------------------------------------------------
;;; ROM Dump to RAM
;;;
MROM_Dump_Entry:
        jsr MROM_Dump           ; Defined in mce_commonlib.asm
        jmp CMD_OK_END

;;; --------------------------------------------------------------------------------
;;; 
;;; 
;;; --------------------------------------------------------------------------------

;;; --------------------------------------------------------------------------------
;;; MCFG
;;; --------------------------------------------------------------------------------
        
MCFG_GenFSK_Entry:
        DBG_PRINT0 "MCFG - MCE ram bank (WMBUS T-MODE)"
        lli     MDMCONF_WMBUS_TMODE, r1              ; Points R1 to the WMBUS TMODE data
        lli     MDMCONF_WMBUS_TMODE_FIRST_REG, r2    ; Points to the first IO address for configiration
        lli     MDMCONF_WMBUS_TMODE_LAST_REG, r0     ; Points to the last of the IO address
        sub     r2,r0           ; R0 now holds number of iterations for CONF_LOOP, r0 implitly used by loop instruction
MCFG_GenFSK_Loop:
        lmd     (r1), r3        ; Load ROM data
        output  r3, (r2)        ; Send to register bank
        add     1, r1           ; Increase memory pointer
        add     1, r2           ; Increase regbank pointer
        loop    MCFG_GenFSK_Loop   ;

	;; Load default value outside above loop to reduce ROM size
	lmd	MDMSPAR0_REG_DEFAULT,r0
	output  r0,MDMSPAR0        
	lmd	MDMSPAR1_REG_DEFAULT,r0
	output  r0,MDMSPAR1
	lmd	MDMSPAR2_REG_DEFAULT,r0
	output  r0,MDMSPAR2             
        
	input	DEMMISC0, r0				; Equal default value for DEMMISC0/MDMSPAR3 
	output	r0, MDMSPAR3				

	lli     MODCTRL, r2     ; First register to clear
        lli     VITCTRL, r0     ; Last register to clear
        sub     r2,r0           ; R0 now holds number of iterations for first zero loop, r0 implitly used by loop instruction
        beq     ZERO_DONE       ; No values to write
        mov     0, r3           ; Set all regs to 0
ZERO_LOOP:      
	output  r3, (r2)	; Send to register bank
	add     1, r2		; Increase regbank pointer
        loop    ZERO_LOOP       ;
ZERO_DONE:        
        jmp     CMD_OK_END      ; Done

;;; --------------------------------------------------------------------------------
;;; MTX Common Entry
;;; --------------------------------------------------------------------------------

;;; --------------------------------------------------------------------------
;;; MTX GenericFSK Send Preamble and Sync Word function
;;; 
MTX_Common_Preamble:       
        ;; Fetch information from MDMPAR0|2: Preamble settings
        input   MDMCMDPAR2, r0
        output  r0, MODPREAMBLE                       ; Send directly to MODPREAMBLE
        input   MDMCMDPAR0, r0
        sr0     8, r0                                 ; [15:8] => Sync Word Length
        bclr    7, r0
        mov     r0, r1                                ; R1 = SyncWordLength
        add     1, r1                                 ; In [32..1] format
        ;; We are now going to process the SyncWord, and prepare it for later
        ;; In general this is where we are putting things:
        ;; R2 = MDMSYNC0, R3 = 15, R4 = MDMSYNC1, R5 = 15
        ;; This would apply to a full 32-bit sync word. If the syncword is
        ;; between 32 to 17 bits, we would transmit both. If is 16 or less
        ;; only the second one (R4) would be transmitted.
        ;; Also, those syncwords come left-aligned. We need to bit-shift to be
        ;; right aligned.
        ;; Step 0. General Case (32-bit case)
        input   MDMSYNC0, r2
        input   MDMSYNC1, r4
        lli     15, r3
        lli     15, r5
        ;; Step 1. Check if SyncWord is 32 bits
        lli     32, r0
        cmp     r0, r1                                ; Special case, 32 bits 
        beq     MTX_Common_Preamble_RFESEND    ; if yes, we are done
        ;; Step 2. Check if in [31..17] bits range
        lli     16, r0
        cmp     r1, r0              
        beq     MTX_Common_Preamble_16bitSyncWord
        bpl     MTX_Common_Preamble_ShortSyncWord
	;; R2 = MDMSYNC0-right-aligned, R3 = num of bits, R4 = MDMSYNC1, R5 = 15
MTX_Common_Preamble_LongSyncWord:
        mov     r1, r3                                ; R3 = [31..17]
	sub	r0, r3		                      ; R3 = R3-16 
        sub     r3, r0                                ; R0 = 16-R3
        sub     1, r3                                 ; R3 = # bits to send
        sub     1, r0                                 ; R0 = # bits to shift down
MTX_Common_Preamble_SyncWord_ShiftLoop1:
	sr0	1, r2                                 ; MDMSYNC0-right-aligned
	loop	MTX_Common_Preamble_SyncWord_ShiftLoop1
        jmp MTX_Common_Preamble_RFESEND
        ;; Step 3. Must be in [16..1] bits range
MTX_Common_Preamble_16bitSyncWord:
        mov	0x1F, r3                              ; Illegal value to indicate 16 bit or shorter sw
        jmp     MTX_Common_Preamble_RFESEND
MTX_Common_Preamble_ShortSyncWord:       
	;; Results, R2 = XXX, R3 = 0, R4 = MDMSYNC1, R5 = num of bits
        mov	0x1F, r3                              ; Illegal value to indicate 16 bit or shorter sw        
        mov     r1, r5                                ; R5 = [16..1]
        lli     16, r0                                ; R0 = 16
        sub     r5, r0                                ; R0 = 16-R5
        sub     1, r5                                 ; R5 = # bits to send
	sub	1, r0	                              ; R0 = # bits to shift down
MTX_Common_Preamble_SyncWord_ShiftLoop2:
	sr0	1, r4                                 ; MDMSYNC0-right-aligned
	loop	MTX_Common_Preamble_SyncWord_ShiftLoop2
	;; and done with SyncWordProcessing
MTX_Common_Preamble_RFESEND:     
        ;; ============================================
        ;; Wait for RAT event
        outbset MCEEVENT1_RAT_EVENT0, MCEEVENTMSK1    ; enable wait for RAT
        wait                                       
	outbset MCEEVENT1_RAT_EVENT0, MCEEVENTCLR1    ; Clear event again
        outbclr MCEEVENT1_RAT_EVENT0, MCEEVENTMSK1    ; disable wait for RAT
        ;; Send message to RFE that modulation is starting
        outbset  0,RFESEND
        ;; First, send
        ;; Enable modulator and timebase now
        ;; Step 2. Go to Preamble insert, Send MDMSYNC0, MDMSYNC1
        outbset MODCTRL_PREAMBLEINSERT, MODCTRL       ; Go to preamble insert mode
        outbset MDMENABLE_MODULATOR, MDMENABLE
        outbset MDMINIT_MODULATOR, MDMINIT
        outbset MDMENABLE_TIMEBASE, MDMENABLE
        outbset MDMINIT_TIMEBASE, MDMINIT
        ;; Preamble insert mode from start
MTX_FirstBit:
        outclr  MODPRECTRL                            ; Send ONE bit        
        outbset MCEEVENT1_PREAMBLE_DONE, MCEEVENTMSK1 ; wait for preamble done
        wait
        outbset MCEEVENT1_PREAMBLE_DONE, MCEEVENTCLR1 ; Clear Preamble Done
        ;; Reload MOCPRECTRL with proper value now
        input   MDMCMDPAR0, r0
        output  r0, MODPRECTRL                        ; Send directly to MODPRECTRL again
MTX_Common_Preamble_Loop:        
        wait
        outbset MCEEVENT1_PREAMBLE_DONE, MCEEVENTCLR1 ; Clear Preamble Done
        ;; Infinite preamble? Indicated by MSB of MDMCMDPAR0
        input   MDMCMDPAR0, r0
        btst    15, r0
        bne     MTX_Common_Preamble_Loop
        ;; Preamble is done now, start with SyncWord. Use the R2, R3, R4, R5 values from above
        mov     0x1F, r0        ; 
	cmp	r0, r3          ;  check if less than 16 bit SW        
	beq	MTX_Common_Preamble_Send_One_SW
MTX_Common_Preamble_Send_Two_SW:
	output	r3, MODPRECTRL
	output	r2, MODPREAMBLE
	wait
        outbset MCEEVENT1_PREAMBLE_DONE, MCEEVENTCLR1 ; Clear Preamble Done
MTX_Common_Preamble_Send_One_SW:
	output	r5, MODPRECTRL
	output	r4, MODPREAMBLE
	wait
        outbset MCEEVENT1_PREAMBLE_DONE, MCEEVENTCLR1 ; Clear Preamble Done
        outbset MCEEVENT1_PREAMBLE_DONE, MCEEVENTMSK1 ; Disable PREAMBLE_DONE from now on
        outbclr MODCTRL_PREAMBLEINSERT, MODCTRL       ; No more preamble insert
        rts

;;; --------------------------------------------------------------------------
;;; MTX GenericFSK Termination
;;; 
MTX_Termination:
        ;; Termination, Step 2. wait 4 clock baud periods before switching off
        outbclr MCEEVENT1_CLKEN_BAUD, MCEEVENTMSK1   ; Disable CLKEN_BAUD events
        lli     0x03,r0                              ; Wait for 4 baud periods before switching
        output  r0, TIMCTRL                          ; the modulator off
        lli     0x04, r0                             ; Four periods
        output  r0, TIMPERIOD
        outbset MCEEVENT0_TIMER_IRQ, MCEEVENTCLR0    ; Clear IRQ
        outbset MCEEVENT0_TIMER_IRQ, MCEEVENTMSK0    ; Enable the mask!
        wait
        outbset MCEEVENT0_TIMER_IRQ, MCEEVENTCLR0    ; Clear IRQ
        outbclr MCEEVENT0_TIMER_IRQ, MCEEVENTMSK0    ; Disable Mask
        outclr  TIMCTRL                              ; Switch off counter
        outbclr MDMENABLE_MODULATOR, MDMENABLE       ; Disable modulator, timebase
        outbclr MDMENABLE_TIMEBASE, MDMENABLE
        rts
        


;;; --------------------------------------------------------------------------
;;; Soft TX / Manchester coding, TopSM read data from FIFO, Manchester coding and send out. 
;;;           
MTX_WMBUS_TMODE:

	outset  MCEEVENTCLR0    
        outset  MCEEVENTCLR1
	DBG_PRINT0 "WMBUS T-MODE, TX Started...."
        lli     0x10, r0                                   ; MDMFIFORDCTRL, 1 bit read, from modem
        output  r0, MDMFIFORDCTRL
        jsr     MTX_Common_Preamble			   ; Send Preamble
        outbset MODCTRL_FECENABLE, MODCTRL                 ; No FEC
        outbset MCEEVENT0_FIFO_ERR_UNDERFLOW, MCEEVENTCLR0 ; clear any previous
        outbset MCEEVENT0_FIFO_ERR_UNDERFLOW, MCEEVENTMSK0 ; allow FIFO UNDERFLOW errors
        
	;; The preamble and sync word is sent. It's now time to send the payload
	outbset MODCTRL_SOFTTXENABLE, MODCTRL			; Go into SOFT Mode, due to manchester coding

	lli     0x03, r0					; MDMFIFORDCTRL, Nibble reads, back to MDMFIFORD
        output  r0, MDMFIFORDCTRL

        outbset MCEEVENT1_CLKEN_BAUD, MCEEVENTCLR1	; Clear any previous events
        outbset MCEEVENT1_CLKEN_BAUD, MCEEVENTMSK1	; Enable Baud Event	

        input   MDMSPAR2, r5    ; read if we should invert the payload data
        sr0     8, r5           ;
        sr0     5, r5           ;        
        and     1, r5           ;
	beq	MTX_SOFT_TMODE_PAYLOAD_LOOP
	mov	15,r5 		; 4 bit MASK
        
MTX_SOFT_TMODE_PAYLOAD_LOOP:
	input	MCEEVENT0, r0
	btst	MCEEVENT0_FIFO_ERR_UNDERFLOW, r0		; check for fifo underflow
	bne	MTX_SOFT_TMODE_PAYLOAD_FINISHED			; if undeflow go to postamble insert mode
	input   MDMFIFORD, r0					; contains bit to transmit
        xor     r5, r0                                          ; optional invertion of data                
        lli     TMODE_3OUTOF6_ENCODING_LUT, r1			; Points R1 to the Encoding LUT
	add	r0,r1
	lmd	(r1),r2

	lli	6,r3						; N-1  
MTX_WMBUS_TMODE_PAYLOAD_LOOP_INNER:
	btst	5,r2						; Bit 5 / MSB is transmitted first
	;; NRZ Soft TX 
	beq	MTX_WMBUS_TMODE_ZERO
	mov	1, r1
	jmp	MTX_WMBUS_TMODE_SOFTTX
MTX_WMBUS_TMODE_ZERO:
	mov	0x1F, r1
MTX_WMBUS_TMODE_SOFTTX:	
        output  r1, MODSOFTTX					; Bug fix: Need to output soft symbol before baud strobe, otherwise a zero is inserted after preamble
        outbset MCEEVENT1_CLKEN_BAUD, MCEEVENTCLR1	; Clear any previous events
        wait
	sl0	1,r2						; Enable Next bit in position 5 / MSB
	;;mov	r0, r9
	;;mov	r1,r10						; Load r10 for debug trace
	;;DBG_PRINT2 "WMBUS, 3 out of 6 encoding, Input Nibble = %d, MODSOFTTX Symbol = %d", r11, r10
	;;DBG_PRINT1 "TMODE, MODSOFTTX Symbol = %d", r10
	;;mov	r9, r0
	;;loop	MTX_WMBUS_TMODE_PAYLOAD_LOOP_INNER
	sub	1,r3
	bne	MTX_WMBUS_TMODE_PAYLOAD_LOOP_INNER
	jmp	MTX_SOFT_TMODE_PAYLOAD_LOOP	
	
	
MTX_SOFT_TMODE_PAYLOAD_FINISHED:		
        outbclr MCEEVENT1_CLKEN_BAUD, MCEEVENTMSK1	; Enable Baud Event	
        outbset MCEEVENT1_CLKEN_BAUD, MCEEVENTCLR1	; Clear any previous events
        outbclr MCEEVENT0_FIFO_ERR_UNDERFLOW, MCEEVENTMSK0 ; Remove FIFO UNDERFLOW events
        jsr     MTX_Termination
        jmp     CMD_OK_END

	


;;; ============================================================================================
;;; ============================================================================================
;;; ============================================================================================
;;; ============================================================================================
;;; MRX
;;; ============================================================================================
;;; ============================================================================================
;;; ============================================================================================
;;; ============================================================================================

	
	
	
		

        

        
MRX_COMMONEND:   
        outbset RDCAPT0_DEMLQIE0, RDCAPT0                  ; Capture LQI into MDMSTATUS[15:8]
        input	MDMSTATUS, r0
        input   DEMLQIE0, r1
        sr0     2, r1                                      ; Divide by 4 first
        sl0     8, r1 
	or	r0,r1
        output  r1, MDMSTATUS
        DBG_PRINT0 "All bits received, MCE Ending"
        ;; Hard init of all modules except FIFO
        outset  TIMCTRL          ;
        outclr  TIMCTRL          ;
        outclr  MODCTRL          ; 
        outclr  MDMENABLE_ADCDIG, MDMENABLE      ; 
        outclr  MDMENABLE_TIMEBASE, MDMENABLE    ;
        outclr  MDMENABLE_DEMODULATOR, MDMENABLE ;
        outset  DEMINIT0         ;
        outset  DEMINIT1         ;
        jmp     CMD_OK_END

        
;;; --------------------------------------------------------------------------
;;; No FEC, directly use MLSE output
MRX_WMBUS_TMODE:
        mov     0x03, r0            ; From Modem, 4 bits at a time (output from MDMFIFOWR)
        output  r0, MDMFIFOWRCTRL   ; FIFO write from register, 1 bit words write

        outbset DEMMISC2_MLSERUN, DEMMISC2                 ; Enable decisions
        DBG_PRINT0 "WMBUS T-MODE, RX Started, Symbol rate is hardcoded to 100kbps....."
	;; No initialization, directly into Sync Search
        outbset MCEEVENT0_CPEFWEVENT0, MCEEVENTCLR0        ; Clear any pending CPEFWEVENT0
	
	;; Enable Demodulator, ADCDIG, Timebase and submodules
        ;;; Enable ADCDIG, and start sync search
        outbset MDMENABLE_ADCDIG, MDMENABLE
        outbset MDMINIT_ADCDIG, MDMINIT
        ;;; Enable RX, and start sync search
        outbset MDMENABLE_DEMODULATOR, MDMENABLE
        outbset MDMINIT_DEMODULATOR, MDMINIT
        ;; Enable timebase now.
        outbset MDMENABLE_TIMEBASE, MDMENABLE
        outbset MDMINIT_TIMEBASE, MDMINIT

       ;; Initial Module Enables  
        lmd     DEMENABLE0_RX_GENFSK, r0
        output  r0, DEMENABLE0  ; cmix, ctil, bdec, chfi, fidc, frac ,iqmc, enable
        output  r0, DEMINIT0
        lmd     DEMENABLE1_RX_GENFSK, r0
        output  r0, DEMENABLE1  ;        
        output  r0, DEMINIT1
        outbset 5, RFESEND                              ; send message to rfe that reception is starting

	;; Set Correlator in early-late mode
	outbset	MODCTRL_EARLYLATE, MODCTRL ; start early-late mode


	;; Main Payload loop
	outbset MCEEVENT0_CLKEN_4BAUD, MCEEVENTCLR0	; Clear any previous events
        outbset MCEEVENT0_CLKEN_4BAUD, MCEEVENTMSK0	; Check now CLKEN_4BAUD events

	;; ***********************************************************
	;;
	;; Initialisation at CMD_RX
	;; 
	;; ***********************************************************

	

MRX_WMBUS_RESTART:

	;; Turn TIMER off
	lli	0, r0
	output	r0, TIMPERIOD
	output	r0, TIMCTRL

	input	MDMSPAR3, r0
	output	r0, DEMMISC0				; Set Default Rx IF (1 MHz) 
	lli	0, r0
	output	r0, DEMMISC1				; Set 260kHz RX BW bandwidth during CS/Preamble search
	
	
	lli	0x3, r0					; 
	output	r0, DEMFIFE0				; Set Fast Frequence Estimation Looop Gain (OK if only used in preamble)
	outbset	DEMINIT1_FIFE, DEMINIT1			; Clear DEMFIFE0 if Restart occur 
	
	outbclr MCEEVENT0_TIMER_IRQ, MCEEVENTMSK0	; Disable the mask!
 	outbset MCEEVENT0_TIMER_IRQ, MCEEVENTCLR0	; Clear IRQ
        outbset MCEEVENT0_CLKEN_4BAUD, MCEEVENTCLR0   


	;; Swap and invert sync word for MIPS optimisation of Sync Detect Search
	input	MDMSYNC1,r1				; 16 bit sync detect
	mov	0,r0					; Shift register with decoded bits
	sub	1,r0					; Mask for bit invert
	xor	r0, r1					; 
	lli	15,r0
SYNCWORD_SWAP_LOOP:
	mov	r1,r2
	and	1, r2
	sl0	1, r14
	add	r2,r14					; r14 contain 16 bit sync word 
	sr0	1, r1	
	loop	SYNCWORD_SWAP_LOOP
	lmd	PREAMBLE_PATTERN,r12			; Preamble Pattern

	DBG_PRINT0 "Start search for packet header.... (WARNING: THIS PATCH SUPPORT ONLY cc13xx PG2.0 Prop API, (NOT cc26xx or cc13xx pg1.0)"
	
	;; Set Baudrate to 100kbps (override Prop API setting as these may be changed by MCE patch)
	mov	0, r13
	;; Subroutine:	Input R13 (MODIFY R0,R1,R13) -> MODIFY HARDWARE REGISTERS (RATEWORD AND FRACTIONAL DECIMATOR)
	jsr	WRITE2MDMBAUD
		
	mov	0,r10					; r10 contain 4 phase counter 
	mov	0,r15					; Clear data bit counter.  This is algorithm write 4 data bits to FIFO and CM0 write N-1 to MDMCMDPAR1.
	sub	1,r15					; Set data bit counter to -1 for MIPS optimisation.
	mov	0,r11					; R11 contain shift register with decoded bits
	sub	1,r11					; Set shift register initial value to ones.
	
	
	lli	0xF, r0					; Set CS timer to 18 periods
	add	6, r0
	output	r0, TIMPERIOD
	
		
MRX_WMBUS_PREAMBLE_SEARCH:		
	
	;; ***********************************************************
	;;
	;; Step1: TopSM start searching for 16 bit Preamble and CS (High gain in frequency compensation loop) 
	;; 
	;; ***********************************************************
	;; This loop implement a Zero Crossing receiver, hard decision and Preamble correlation.
	;; 
	;; Register Overview during loop session:
	;; R10 = 4x Sample counter
	;; R11 = Shift register with decoded bits
	;; R12 = Preamble pattern (0xAAAA)
	;; R0-R1 = Volatile

	
	;; note that MCEEVENT0_CLKEN_4BAUD is running at 400kHz, max 30 instructions in the loop
MRX_WMBUS_PREAMBLE_SEARCH_LOOP:
        outbset MCEEVENT0_CLKEN_4BAUD, MCEEVENTCLR0   
	wait    	
	outbset RDCAPT0_DEMSOFD0, RDCAPT0		; Capture  DEMSOFDx
	input   MCEEVENT0, r0				; Check if CS timeout present 
        btst    MCEEVENT0_TIMER_IRQ,r0			; 
	bne	MRX_WMBUS_PREAMBLE_FOUND
	cmp	2, r10					; Capture Mid Sample on Phase 2 
	bne	MRX_WMBUS_CS_TIMER_TASK			; Check Carrier sense 
	input	DEMSOFD1, r0				; Capture Mid Sample only
	sr0	7, r0					; Do hard decision
	sl0	1, r11
	add	r0, r11					; Add sign bit to Shift register with decoded bits 
	cmp	r12, r11
	beq	MRX_WMBUS_PREAMBLE_FOUND		; Break loop if preamble is found
	jmp	MRX_WMBUS_PREAMBLE_ZEROCROSS_DETECTION ; End FIFO

MRX_WMBUS_CS_TIMER_TASK:	
        input   RFERCEV, r0                             ; check CS status from RFE
        btst    2, r0                                   ;
        beq     MRX_WMBUS_CS_TIMER_CLR
	lli	3, r0					; enable CS timer, tick on baud rate
	output	r0, TIMCTRL
	jmp	MRX_WMBUS_PREAMBLE_ZEROCROSS_DETECTION
MRX_WMBUS_CS_TIMER_CLR:	
	lli	0, r0					; clr CS timer
	output	r0, TIMCTRL

	
	
MRX_WMBUS_PREAMBLE_ZEROCROSS_DETECTION:
	input   DEMSOFD3, r0				;  Check if zero crossing is present, then reset sample counter
	input   DEMSOFD4, r1				; 
	xor	r0,r1
	btst	7, r1
	beq	MRX_WMBUS_PREAMBLE_NO_CROSSING_CHECK
MRX_WMBUS_PREAMBLE_CLEAR_SAMPLE_COUNTER:
	mov	0, r10
	jmp	MRX_WMBUS_PREAMBLE_ZEROCROSS_DETECTION_FINISHED
MRX_WMBUS_PREAMBLE_NO_CROSSING_CHECK:
	cmp	3, r10
	beq	MRX_WMBUS_PREAMBLE_CLEAR_SAMPLE_COUNTER
	add	1, r10	
MRX_WMBUS_PREAMBLE_ZEROCROSS_DETECTION_FINISHED:		

	
	jmp	MRX_WMBUS_PREAMBLE_SEARCH_LOOP

	
 
	
	;; ***********************************************************
	;;
	;; Step2: TopSM start estimating symbol rate (Low gain in frequency compensation loop and should freeze front end gain, TBD) 
	;; 
	;; ***********************************************************
	;; 
	;; The preamble is n*(01) with n>=15. Hence we know that we have a preamble of at least 30 bits before the sync word is sent
	;; We will use the preamble to try to get an estimation of the data rate error, change the baudrate accordingly and then wait for sync
	;;
	;; When the correlator is set in early-late mode, the reference vector of correlator A and B are replaced with EARLY_REFERENCE and LATE_REFERENCE respectively.
	;; EARLY_REFERENCE represents the preamble sent with 12% data rate error and LATE_REFERENCE represents it with a -12% data rate error.
	;; Hence the more positive data rate error, the higher correlation value of correlator A be and vice versa for correlator B
	;; Adding up their values for a certain amount of baud periods will give us an estimate of the data rate error.
	;; To add up the correlation values, we use a filter that filters out the high frequency componants to give us a more reproducible value.
	;; 
	;; The EARLY_REFERENCE correlator will have high accuracy (meaning there is a high probability its estimate is close to the actual error) for data rate errors from 12% down to 3% and the LATE_REFERENCE correlator will have high accuracy for data rate error from -12% up to -3%
	;; Outside their respective intervals, none of the correlators have a high probability of determining the right data rate error.
	;; 
	;; These observations suggest the following algorithm:
		;; First use the correlator in early-late mode for M baud periods
		;; Based on the values of EARLY_REFERENCE and LATE_REFERENCE correlator decide if the data rate error is
			;; a) between -3% and -12%
			;; b) between -3% and 3%
			;; c) between 3% and 12%
		;; for b) Set data rate estimate to 0% (no error).

	
MRX_WMBUS_PREAMBLE_FOUND:

        btst    MCEEVENT0_TIMER_IRQ,r0			; 
	bne	MRX_WMBUS_PREAMBLE_CS_TIMEOUT_PRINT
	DBG_PRINT0 "Preamble Found...."
	jmp	MRX_WMBUS_PREAMBLE_SR_ESTIMATION_START
MRX_WMBUS_PREAMBLE_CS_TIMEOUT_PRINT:	
	DBG_PRINT0 "CS Timeout present...."

MRX_WMBUS_PREAMBLE_SR_ESTIMATION_START:			
	;; Turn timer off
	lli	0, r0
	output	r0, TIMPERIOD
	output	r0, TIMCTRL
	outbclr MCEEVENT0_TIMER_IRQ, MCEEVENTMSK0	; Disable the mask!
 	outbset MCEEVENT0_TIMER_IRQ, MCEEVENTCLR0	; Clear IRQ

	lli	0x7, r0					; Set Medium DEMFIFE0 gain during symbol rate estimation
	output	r0, DEMFIFE0				; 

	;; Symbol rate estimator use the following registers (only R14 as input)
	;; R9  = BLOCK AVERAGE EARLY correlator estimate
	;; R10 = BLOCK AVERAGE LATE correlator estimate
	;; R11 = MAX PEAK EARLY correlator estimate
	;; R12 = MAX PEAK LATE correlator estimate
	;; R13 = Data Rate Error in %
	;; R14 = IN/OUT Sync Word (kept for later use)
	;; R0-R5 = Volatile
	DBG_PRINT0 "Packet Header Found, either CS or Preamble. Start Symbol Rate Error Estimation...."

		
	;; TBD, use both max peak and average, then use avg for small values (use the same threshold)!
	lmd	SYMBOLRATE_ESTIMATION_TIME, r0		; Max Peak algorithm for 2 symbol window (i.e. 8 peaks)
	lli	0, r9					; Block average Early value
	lli	0, r10					; Block average Late value
	lli	0, r11					; Max peak Early value
	lli	0, r12					; Max peak Early value
	;; Find MAX PEAK: Input R0, R11, R12 (MODIFY R0,R1,R2,R9, R10, R11,R12) -> OUTPUT R9, R10, R11, R12 		
	jsr	MAXPEAK_CORR_VALUES
	srx	5, r9					; Divide by block length 
	srx	5, r10					; Divide by block length 
	
	;; Update RX IF with DEMFIFE0 estimate to meet frequency offset tolerance requirement (+.50kHz).
	outbset RDCAPT0_DEMFIFE2, RDCAPT0		; Capture  DEMFIFE2	
	input	DEMFIFE2, r0
	sl0	8, r0					; Sign extend 
	srx	8, r0
	lmd	FB2PLL_ROUNDING, r1
	add	r1, r0					; Rounding add 0.5 lsb + magic bias
	srx	3, r0					; Simple calculating from DEMFIFE0 to CMIXN tick 
	input	MDMSPAR3, r8
	sub	r0, r8					; Add default RX-IF with DEMFIFE0 estimate 
	output	r8, DEMMISC0
	outbset	DEMINIT1_FIFE, DEMINIT1			; Clear DEMFIFE0 after updating CMIXN
	lmd	DEMFIFE0_BYPASS, r0			; Enabled, but OFF + MANUAL compensation = 0.
	output	r0, DEMFIFE0				; 
	
	lli	2, r0
	output	r0, DEMMISC1				; Set 220kHz RX bandwidth
	
	;;;mov	r11, r4					; Debugging EARLY/LATE estimate
	;;;mov	r11, r2
	;;;sub	r12, r2
	;;;mov	r9, r4
	;;;mov	r9, r1
	;;;sub	r10, r1
	;;;DBG_PRINT1 "(BLOCK) EARLY-LATE %3d", r1
	;;;DBG_PRINT1 "(PEAK) EARLY-LATE %3d", r2

	;;; Use MAXPEAK comparison for finding max value
	cmp	r11, r12
	bpl	CHECK_DATARATEERROR_LESS_THAN_NEG3
		
	;; First determine if the data rate error is a) between -3% and -12%, b) between -3% and 3% or c) between 3% and 12%
CHECK_DATARATEERROR_MORE_THAN_NEG3:	
	lmd	EARLY_COARSE_THRESHOLD, r0
	sub	r11, r0				; Threshold - max peak value 
	bmi	DATARATEERROR_MORE_THAN_3	; b) between -3% and 12%
	sub	10, r0				; Second limit THR2 = THR-10.. -> THR2 - CORRVAL 
	sub	r11, r0
	bpl	DATARATEERROR_LESS_THAN_ABS3	; a) less than -3%	
	mov	r11, r13			; Otherwise CORR_THR-10 < LATE_CORR < CORR_THR, in the range of 3%....
	mov	r12, r3
	jsr	FIND_DISTANCE_R13_R3
	lmd	EARLY_DISTANCE_THRESHOLD, r0
	cmp	r0, r5				; 
	bpl	DATARATEERROR_MORE_THAN_3
	jmp	DATARATEERROR_LESS_THAN_ABS3	; b) between -3% and 3% (EARLY/LATE algorithm does not work for minor symbol rate offset)

CHECK_DATARATEERROR_LESS_THAN_NEG3:		
	lmd	LATE_COARSE_THRESHOLD, r0
	sub	r12, r0				; LATE_COARSE_THRESHOLD - LATE_CORR > ?
	bmi	DATARATEERROR_LESS_THAN_NEG3	; a) between -3% and 12%
	sub	10, r0				; LATE_COARSE_THRESHOLD-10 < LATE_CORR < LATE_COARSE_THRESHOLD
	sub	r12, r0
	bpl	DATARATEERROR_LESS_THAN_ABS3	; a) less than -3%	
	mov	r11, r13			; Otherwise LATE_COARSE_THRESHOLD-10 < LATE_CORR < LATE_COARSE_THRESHOLD, in the range of -3%.... 
	mov	r12, r3
	jsr	FIND_DISTANCE_R13_R3
	lmd	LATE_DISTANCE_THRESHOLD, r0
	cmp	r0, r5				; ABS(EARLY_CORR-LATE_CORR) - LATE_DISTANCE_THRESHOLD > ?
	bpl	DATARATEERROR_LESS_THAN_NEG3
	jmp	DATARATEERROR_LESS_THAN_ABS3	; b) between -3% and 3% (EARLY/LATE algorithm does not work for minor symbol rate offset)
	
	
	
DATARATEERROR_MORE_THAN_3:
	;; R11 already contains the value we're interested in
	lli	EARLY_CORR_COMP_LUT, r12	; pointer to reference table
	lli	13, r0				; # of values in the table
	;; Subroutine:	Input R0, R11, R12 (MODIFY R0-R5,R13) -> OUTPUT R13
	jsr	DETERMINE_DATARATEERROR_OUTSIDE_NEG3_AND_3
	;; modify R13 a little bit so that it contains the actual data rate error
	mov	r13, r5
	sub	12, r5
	mov	0, r13
	sub	r5, r13				; r13 now contains the most probable data rate error	
	jmp	MRX_WMBUS_MODIFY_DATARATE
	
DATARATEERROR_LESS_THAN_NEG3:
	mov	r12, r11			; cumulated correlator value
	lli	LATE_CORR_COMP_LUT, r12		; pointer to reference table
	lli	13, r0				; # of values in the table
	;; Subroutine:	Input R0, R11, R12 (MODIFY R0-R5,R13) -> OUTPUT R13
	jsr	DETERMINE_DATARATEERROR_OUTSIDE_NEG3_AND_3
	mov	r13, r5
	mov	0, r13
	sub	r5, r13				; r13 now contains the most probable data rate error
	jmp	MRX_WMBUS_MODIFY_DATARATE
	
;;; latency delay must align with data rate calculation delay
DATARATEERROR_LESS_THAN_ABS3:
	lli	0, r13				; Force estimate to zero 	
	jmp	MRX_WMBUS_MODIFY_DATARATE
	
	
	;; ***********************************************************
	;;
	;; Step3: TopSM SyncWord Search
	;; 
	;; ***********************************************************
	;; This loop implement a Zero Crossing receiver, hard decision and SyncWord Correlation.
	;; 
	;; Register Overview during loop session:	
	;; R6  = Flag for task interleaving (MIPS optimisation)
	;; R7  = Flag for task interleaving (MIPS optimisation)
	;; R8  = MASK for setting 12 bit sync detect
	;; R9  = MASK for sign bit invert
	;; R10 = Sample counter
	;; R11 = Shift register with decoded bits
	;; R14 = SyncWord
	;; R0-R1 = Volatile
	
MRX_WMBUS_MODIFY_DATARATE:
	DBG_PRINT1 "Start Searching for SyncWord, Symbol Rate Error Estimate %2d %%", r13
	;; Subroutine:	Input R13 (MODIFY R0,R1,R13) -> MODIFY HARDWARE REGISTERS (RATEWORD AND FRACTIONAL DECIMATOR)
	jsr	WRITE2MDMBAUD
	mov	0, r11					; Shift register Hard decision 
	sub	1, r11					; Set defult value All ones 
	lli	0, r10

	lli	0, r6					; Clear flag for task interleaving during packet reception (after sync detect)
	lli	0, r7					; Clear flag for task interleaving during packet reception (after sync detect)

	lli	0, r9
	input   MDMSPAR2, r0				; read if we should invert the data before manchester encoding
        sr0     8, r0					;
        sr0     5, r0					;        
        and     1, r0					;
	bne	MRX_WMBUS_MASK_DONE
	lmd	CHIP_SIGNBIT_INVERT_MASK, r9		; Load chip sign bit invert mask
MRX_WMBUS_MASK_DONE:	

	lmd	SYNCWORD_DETECT_MASK,r8			; SyncWord MASK
	and	r8,r14					; mask preamble reference 
	lmd	DEMFIFE0_SETTLE_TIME, r2		; DEMFIFE0 settle time (critical)


	;;; Wait for Digital filter delay has settled! (4 symbols)
	lli	0x0F, r0
MRX_WMBUS_WAIT_LOOP:
	outbset MCEEVENT0_CLKEN_4BAUD, MCEEVENTCLR0          ; Clear timer event
	wait
	loop	MRX_WMBUS_WAIT_LOOP	
	
	lli	0x3, r0					; Medium DEMFIFE0 gain. Must be settled before sync word is present.
	output	r0, DEMFIFE0				; 
	outbset	DEMINIT1_FIFE, DEMINIT1			; Clear DEMFIFE0 after updating CMIXN

	;; TIMEOUT set to 32 symbols
	lmd	SYNCSEARCH_TIMEOUT_PERIOD, r0			; 32 sync search timeout periods
	output	r0, TIMPERIOD
	lli	3, r0					; enable timer, tick on baud rate
	output	r0, TIMCTRL

	;; note that MCEEVENT0_CLKEN_4BAUD is running at 400kHz, max 30 instructions in the loop
MRX_WMBUS_SYNCSEARCH_LOOP:	
        outbset MCEEVENT0_CLKEN_4BAUD, MCEEVENTCLR0   
	
	wait    
	input	MCEEVENT0,r0				; Check Timeout at 4baud rate
	btst	MCEEVENT0_TIMER_IRQ,r0
	bne	MRX_WMBUS_RESTART_PRINT			; Jump to printing restart for easy trace debugging 

	outbset RDCAPT0_DEMSOFD0, RDCAPT0		; Capture  DEMSOFDx	
	cmp	2, r10					; Capture Mid Sample on Phase 2 
	bne	MRX_WMBUS_SAMPLE_COUNTER_NOT_2
	input	DEMSOFD1, r0				; Capture Mid Sample only
	sr0	7, r0
	sl0	1, r11
	add	r0, r11					; Add sign bit to Shift register with decoded bits
	and	r8, r11					; MASK SyncWord
	cmp	r14, r11				; SyncWord Correlation 
	beq	MRX_WMBUS_SYNC_FOUND			; Break loop if SyncWord is found
	jmp	MRX_WMBUS_ZEROCROSS_DETECTION

MRX_WMBUS_SAMPLE_COUNTER_NOT_2:
	cmp	0, r2					; Check if timeout on DEMFIFE0 settle period 
	beq	MRX_WMBUS_SET_DEMFIFE0_TRACKING
	sub	1, r2
	jmp	MRX_WMBUS_ZEROCROSS_DETECTION
MRX_WMBUS_SET_DEMFIFE0_TRACKING:	
	lli	0x7, r0					; Set Medium DEMFIFE0 gain after fast settle time period is finished
	output	r0, DEMFIFE0				; 
	
MRX_WMBUS_ZEROCROSS_DETECTION:
	input   DEMSOFD3, r0				;  Check if zero crossing is present, then reset sample counter
	input   DEMSOFD4, r1				; 
	xor	r0,r1
	btst	7, r1
	beq	MRX_WMBUS_NO_CROSSING_CHECK
MRX_WMBUS_CLEAR_SAMPLE_COUNTER:
	mov	0, r10
	jmp	MRX_WMBUS_ZEROCROSS_DETECTION_FINISHED
MRX_WMBUS_NO_CROSSING_CHECK:
	cmp	3, r10
	beq	MRX_WMBUS_CLEAR_SAMPLE_COUNTER
	add	1, r10	
MRX_WMBUS_ZEROCROSS_DETECTION_FINISHED:		
	jmp	MRX_WMBUS_SYNCSEARCH_LOOP


MRX_WMBUS_RESTART_PRINT:
	input	DEMMISC0, r8
	DBG_PRINT1 "Restart Preamble Search, DEMMISC0 = %d", r8
	jmp	MRX_WMBUS_RESTART
	

	
	;; ***********************************************************
	;;
	;; Step4: TopSM Receiving packet
	;; 
	;; ***********************************************************
	;; This loop implement a Zero Crossing receiver, hard decision and Packet Length comparison
	;; 
	;; Register Overview during loop session:	
	;; R6  = Flag for task interleaving (MIPS optimisation)
	;; R7  = Flag for task interleaving (MIPS optimisation)
	;; R9  = Decoded nibble
	;; R10 = Sample counter
	;; R11 = Shift register with decoded bits
	;; R14 = chip counter (0-6)
	;; R15 = bit counter
	;; R0-R1 = Volatile
		
MRX_WMBUS_SYNC_FOUND:
	add	1, r10					; Increment sample counter 
	lli	0, r11					; clear shift register 
	lli	0, r14					; clear chip counter 
	outbset 1, RFESEND                              ; Notify by bit 1 in RFESEND (sync found)
 
MRX_WMBUS_SYNC_FOUND_PRINT:	
        outbset MCESTROBES0_EVENT0, MCESTROBES0         ; signal sync found
        ;;DBG_PRINT0 "Sync found."			; Print removed due to time critical design

	;; note that MCEEVENT0_CLKEN_4BAUD is running at 400kHz, max 30 instructions in between each 4baud sample
MRX_WMBUS_PAYLOAD_LOOP:	        
	outbset MCEEVENT0_CLKEN_4BAUD, MCEEVENTCLR0	; 2 instructions 
	wait    
	
	outbset RDCAPT0_DEMSOFD0, RDCAPT0		; Capture  DEMSOFDx
	cmp	2, r10
	bne	MRX_WMBUS_3OUTOF6_DECODING_TASK_CHECK
	input	DEMSOFD1, r0				; Capture Sofd0
	;mov	r0, r8					; Debug GUI 
	sr0	7, r0					; Place Sign bit in bit position 0 
	sl0	1, r11
	add	r0, r11					; Add Sign bit to Shift register with decoded bits
	add	1, r14					; Increment chip counter 
	cmp	6, r14					; Check if 6 bit available before doing 3 out of 6 decoding 
	bne	MRX_WMBUS_ZEROCROSS_DETECTION2
	mov	1, r6					; Set flag so are able to do task interleaving (writing to fifo and not check packet length)
	mov	0, r14					; Clear chip counter 
	jmp	MRX_WMBUS_ZEROCROSS_DETECTION2

;;; Check Packet length if not writing to FIFO (This part of code is running in phase 3, then skip 
MRX_WMBUS_3OUTOF6_DECODING_TASK_CHECK:			; 10 instructions if not PHASE2 
	mov	r6, r6					; Check flag so are able to do task interleaving 
	beq	MRX_WMBUS_BYPASS_3OUTOF6_DECODING_TASK
        lli     TMODE_3OUTOF6_DECODING_LUT, r0		; Points R0 to the Decoding Table
	xor	r9, r11					; Invert Sign bit
	add	r11,r0
	lmd	(r0),r1
        mov	r1, r1					; TBD if this command is needed (safety between LMD/OUTPUT) 
	output  r1, MDMFIFOWR				; 
	lli	0, r6
	lli	1, r7
	jmp	MRX_WMBUS_ZEROCROSS_DETECTION2
MRX_WMBUS_BYPASS_3OUTOF6_DECODING_TASK:			; (This part of code is running in phase 3 or 0, or 1)
	mov	r7,r7					; Check flag for task interleaving 
	beq	MRX_WMBUS_ZEROCROSS_DETECTION2		; Check if FIFO is updated! (if not bypass bit counter increment and packet length check)
	add     4, r15					; Increment bit counter
	lli	0, r11					; Clear shift register with decided bits 
	mov	0, r7					; Clear flag so are able to do task interleaving
MRX_WMBUS_CHECK_PACKET_LENGTH:		
	;; Check now if the Packet Length is there (CPE updates it)
        input   MDMCMDPAR1, r0				; Packet size in bits, N-1
	cmp	0, r0
	beq	MRX_WMBUS_ZEROCROSS_DETECTION2		; Packet size not ready, infinite rx 
	cmp     r15, r0					; Check againt fifo bits (possible with this check as writing 4 bits to the FIFO)
        bpl     MRX_WMBUS_ZEROCROSS_DETECTION2		; Keep on receiving if not receive complete packet
	jmp	MR_WMBUS_FINISHED	

MRX_WMBUS_ZEROCROSS_DETECTION2:				; 
	input   DEMSOFD3, r0				; Zero Crossing Detection
	input   DEMSOFD4, r1				; 
	xor	r0,r1
	btst	7, r1
	beq	MRX_WMBUS_NO_CROSSING_CHECK2
MRX_WMBUS_CLEAR_SAMPLE_COUNTER2:	
	mov	0, r10
	jmp	MRX_WMBUS_ZEROCROSS_DETECTION_FINISHED2
MRX_WMBUS_NO_CROSSING_CHECK2:
	cmp	3, r10
	beq	MRX_WMBUS_CLEAR_SAMPLE_COUNTER2
	add	1, r10
MRX_WMBUS_ZEROCROSS_DETECTION_FINISHED2:	
	jmp	MRX_WMBUS_PAYLOAD_LOOP
	

MR_WMBUS_FINISHED:	

        outbclr 5, RFESEND				; tell RFE packet is ending
	outbclr MCEEVENT0_CLKEN_4BAUD, MCEEVENTMSK0	; fne: here or before terminate_pkg?
	outbset MCEEVENT0_CLKEN_4BAUD, MCEEVENTCLR0     ; Clear the event again
	outbclr MCEEVENT0_TIMER_IRQ, MCEEVENTMSK0	; Disable the mask!
 	outbset MCEEVENT0_TIMER_IRQ, MCEEVENTCLR0	; Clear IRQ
	
        jmp     MRX_COMMONEND

	
	

DETERMINE_DATARATEERROR_OUTSIDE_NEG3_AND_3:
	;; Determine the data rate error based the cumulated correlation values for error outside -3% and 3%
	;; R0  = IN number of values in the table
	;; R13 = OUT index of most probable data rate error
	;; R11 = IN cumulated correlator value
	;; R12 = IN pointer to reference (either EARLY or LATE)
	;; R1-R5 = Volatile

	sub	1, r0		; number of iterations = number of values in table -1
	lli	0, r1		; contains smallest value
	lli	12, r2		; contains index of most probable data rate error
	lmd	(r12), r13
	add	1, r12		; increment pointer
	mov	r11, r3
	jsr	FIND_DISTANCE_R13_R3
	mov	r5, r1
	sub	1, r0		; number of iterations remaining
	;; Compare now with the remaing value in the table
OUTSIDE_NEG3_AND_3_LOOP:
	lmd	(r12), r13
	add	1, r12		; increment pointer
	;; R3 already contains the cumulated correlator value
	jsr	FIND_DISTANCE_R13_R3
	;; see if the difference is the smallest so far
	cmp	r1, r5
	bmi	OUTSIDE_NEG3_AND_3_NEW_ESTIMATE
	loop	OUTSIDE_NEG3_AND_3_LOOP	
	jmp	OUTSIDE_NEG3_AND_3_END
OUTSIDE_NEG3_AND_3_NEW_ESTIMATE:
	mov	r5, r1
	mov	r0, r2		; memorize the index
	loop	OUTSIDE_NEG3_AND_3_LOOP
OUTSIDE_NEG3_AND_3_END:	
	mov	r2, r13		; write most probable data rate error to output
	rts			; end of subroutine
	

	
FIND_DISTANCE_R13_R3:
	;; Finds the difference in absolute value of two points
	;; R13 = IN point1
	;; R3  = IN point2
	;; R5  = OUT abs(point1 - point2)
	cmp	r13, r3
	bmi	POINT1_GREATER
	mov	r3, r5
	sub	r13, r5
	rts			; end of subroutine
POINT1_GREATER:
	mov	r13, r5
	sub	r3, r5
	rts			; end of subroutine

	;;; For Tmode there is maximum 30 instruction per input bit (12e6/0.4e6). Cumulate_Corr_Values use 26 instructions per loop
MAXPEAK_CORR_VALUES:
	outbset MCEEVENT0_CLKEN_4BAUD, MCEEVENTCLR0          ; Clear timer event
	wait
	outbset	RDCAPT0_DEMC1BEX, RDCAPT0

	;; Start calculating the output of the filter for correlator A, x[n] = 16*abs(corrA)
	input	DEMC1BE3, r1	; read correlator A
	;; Take absolute value
	mov	r1, r2
	bpl	MAXPEAK_POS_CORRA_VALUE
	mov	0, r1
	sub	r2, r1
MAXPEAK_POS_CORRA_VALUE:	
	;; R1 = abs(corrA)
	sl0	3, r1		; Fixed data Gain
	add	r1, r9		; Integrate value (block average)
	cmp	r1, r11		; max peak
	bpl	MAXPEAK_R11_GREATER
	mov	r1, r11
MAXPEAK_R11_GREATER:

	;; Now do the same for correlator B
	input	DEMC1BE4, r1	; read correlator B
	;; Take absolute value
	mov	r1, r2
	bpl	MAXPEAK_POS_CORRB_VALUE
	mov	0, r1
	sub	r2, r1
MAXPEAK_POS_CORRB_VALUE:	
	;; R1 = abs(corrB)	
	sl0	3, r1		; MagicFixed data gain
	add	r1, r10		; Integrate value (block average)
	cmp	r1, r12		; max peak
	bpl	MAXPEAK_R12_GREATER
	mov	r1, r12
MAXPEAK_R12_GREATER:	
	loop	MAXPEAK_CORR_VALUES
	rts			; end of subroutine



	

WRITE2MDMBAUD:	
	;; Determines the value of MDMBAUD for a given date rate error and writes it to the register
	;; R13 = IN point1
	;; R0-R1 = Volatile
	;;   pdif_factor = 1024 >> pdifdecim
	;;   dec_factor  = 1 << bdec;
	;;   p = 27*rateword*dec_factor;
	;;   q = pdif_factor*pre*channel;

	;; R13 = data rate error in %
	lli	MDMBAUD_VALUES_LUT, r1	; pointer to the MDMBAUD values
	add	12, r1		; R1 points to the 0% data rate error value
	add	r1, r13		; R13 will now point to the corresponding MDMBAUD value
	lmd	(r13), r1	; R1 will now contain the corresponding MDMBAUD value
	mov	r1, r13
	;; Calculate new P value
	output  r1, LOCMULTA      ; Send rate word to A
	lli     27, r0            ; B = 27
	sl0	5, r0
	input   DEMMISC3, r2      ; Find BDEC in DEMMISC3
	mov	r2, r1
	and     7, r1             ; 3 bits mask (bdec1)
	sr0	5, r2
	and     3, r2             ; 2 bits mask (bdec2)
	add	r2, r1
	add	1, r1		  ; Needed for cc13xx pg2.0 (Due to additional Qshift)
	
	beq     ZEROBDEC            ; see if /= 0
	sl0     r1, r0            ; if yes, multiply by 2,4,8 or 16
ZEROBDEC: 
	output  r0, LOCMULTB      ; Now send to B
	mov     7, r0             ; Wait at least 8 instructions....
WRITE2MDMBAUD_WAIT: 
	loop    WRITE2MDMBAUD_WAIT
	output	r13, MDMBAUD	  ; send it to the register (close to p-req write)	
	input   LOCMULTC0, r0     ; Read the LSBs
	output  r0, DEMFRAC0      ; Program P value [15:0]
	input   LOCMULTC1, r0     ; Read the MSBs
	output  r0, DEMFRAC1      ; Program P value [27:16]
	rts	
			
;;; --------------------------------------------------------------------------------
;;; Function for handling debug print
;;; 
        DBG_FUNC

*/

#endif
