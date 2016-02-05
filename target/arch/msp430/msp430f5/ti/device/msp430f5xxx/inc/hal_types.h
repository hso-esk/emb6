/*
 * hal_types.h
 *
 *  Created on: Dec 16, 2013
 *      Author: Fesseha
 */
/***********************************************************************************
 @file     hal_types.h

 @brief    HAL type definitions

 ***********************************************************************************/
#ifndef HAL_TYPES_H
#define HAL_TYPES_H

#ifdef __cplusplus
extern "C"
{
#endif


/*============================================================================*/
/*                                   DEFINES                                  */
/*============================================================================*/


/*! Definition of FALSE value */
#ifndef FALSE
#define FALSE                         0U
#endif /* FALSE */

/*! Definition of TRUE value */
#ifndef TRUE
#define TRUE                          1U
#endif /* TRUE */

/*============================================================================*/
/*                              MACROS                                        */
/*============================================================================*/

#ifdef CCS_COMPILER
#define NOP()  _nop()
#endif
#ifdef IAR_COMPILER
#define NOP() asm("NOP")
#define REG8B                   unsigned char volatile*
#define REG16B                  unsigned short volatile*
#define REG8B_CONST             unsigned char const volatile*
#define REG16B_CONST            unsigned short const volatile*
#endif
#ifdef GCC_COMPILER
#define NOP() _NOP()
#define REG8B                   unsigned char volatile*
#define REG16B                  unsigned int volatile*
#define REG8B_CONST             unsigned char const volatile*
#define REG16B_CONST            unsigned int const volatile*
#endif

#define st(x)      do { x } while (__LINE__ == -1)

#ifdef __cplusplus
}
#endif

#endif
