/*============================================================================*/
/**
 * \file    mcu.c
 *
 * \author  Tobias Neff
 *
 * \brief   .
 *
 *          .
 */
/*============================================================================*/

/*============================================================================*/
/*                                INCLUDES                                    */
/*============================================================================*/
#include <stdint.h>
#include <stdlib.h>
#include <msp430.h>
#include "targetconfig.h"
#include "mcu.h"
#include "int.h"
#include "io.h"
#include "led.h"

/*============================================================================*/
/*                                DEFINES                                     */
/*============================================================================*/

#define DCO_MULT_1MHZ           30
#define DCO_MULT_4MHZ           122
#define DCO_MULT_8MHZ           244
#define DCO_MULT_12MHZ          366
#define DCO_MULT_16MHZ          488
#define DCO_MULT_20MHZ          610
#define DCO_MULT_25MHZ          763

#define DCORSEL_1MHZ            DCORSEL_2
#define DCORSEL_4MHZ            DCORSEL_4
#define DCORSEL_8MHZ            DCORSEL_4
#define DCORSEL_12MHZ           DCORSEL_5
#define DCORSEL_16MHZ           DCORSEL_5
#define DCORSEL_20MHZ           DCORSEL_6
#define DCORSEL_25MHZ           DCORSEL_7

#define VCORE_1MHZ              PMMCOREV_0
#define VCORE_4MHZ              PMMCOREV_0
#define VCORE_8MHZ              PMMCOREV_0
#define VCORE_12MHZ             PMMCOREV_1
#define VCORE_16MHZ             PMMCOREV_1
#define VCORE_20MHZ             PMMCOREV_2
#define VCORE_25MHZ             PMMCOREV_3



/*============================================================================*/
/*                            LOCAL VARIABLES                                 */
/*============================================================================*/

/** current speed of the master clock */
static uint32_t gul_mcu_MclkSpeed = 0;

/*============================================================================*/
/*                          LOCAL FUNCTION PROTOTYPES                         */
/*============================================================================*/

/** initialization routine for XT1 */
static void _mcu_startXT1 (void);

/** sets the MCLK frequency */
void _mcu_sysClockSpeedSet( uint32_t ul_systemClockSpeed );

/** assert function */
void _mcu_assert( void );

/** Get function for the DCORSEL, VCORE, and DCO multiplier */
static void _mcu_getSystemClockSettings( uint32_t ul_systemClockSpeed,
    uint8_t *puc_setDcoRange, uint8_t *puc_setVCore,
    uint32_t *pul_setMultiplier );

/** sets the PMM core voltage */
static void _mcu_setVCore( uint8_t uc_level );

/** increments the VCore setting */
static void _mcu_setVCoreUp( uint8_t uc_level );

/** decrements the VCore setting */
static void _mcu_setVCoreDown( uint8_t uc_level );

/*============================================================================*/
/*                              LOCAL FUNCTIONS                               */
/*============================================================================*/

/*============================================================================*/
/**
 * @brief    This function runs the initialization routine for XT1. It sets the
 *           necessary internal capacitor values, and loops until all oscillator
 *           fault flags remain cleared. XT1 is in Low Power Mode.
 *
 */
/*============================================================================*/
static void _mcu_startXT1 (void)
{
  /* Set internal cap values */
  UCSCTL6 |= XCAP_3;

  /* Check OFIFG fault flag */
  while (SFRIFG1 & OFIFG)
  {
    /* Check OFIFG fault flag */
    while (SFRIFG1 & OFIFG)
    {
      /* Clear OSC fault flags and OFIFG fault flag */
      UCSCTL7 = 0;
      SFRIFG1 &= ~OFIFG;
    }

    /* Reduce the drive strength */
    UCSCTL6 &= ~(XT1DRIVE1_L + XT1DRIVE0);
  }
}

/*============================================================================*/
/**
 * @brief    This function sets the MCLK frequency. It also selects XT1 as ACLK
 *           source with no division in Low Power mode.
 *
 * @param    ul_systemClockSpeed    is the intended frequency of operation.
 *
 * @return   None
 */
/*============================================================================*/
void _mcu_sysClockSpeedSet (uint32_t ul_systemClockSpeed)
{
  uint8_t uc_setDcoRange, uc_setVCore;
  uint32_t ul_setMultiplier;

  /*
   * Set clocks (doing sanity check)
   * MCLK     = ui32SysClockSpeed;
   * SMCLK    = ui32SysClockSpeed;
   * ACLK     = 32 768 Hz
   */
  if ((ul_systemClockSpeed > MCU_SYSCLK_25MHZ ) ||
      (ul_systemClockSpeed < MCU_SYSCLK_1MHZ))
  {
    _mcu_assert();
  }

  /* set internal clock speed */
  gul_mcu_MclkSpeed = ul_systemClockSpeed;

  /* Get DCO, VCore and multiplier settings for the given clock speed */
  _mcu_getSystemClockSettings( ul_systemClockSpeed, &uc_setDcoRange,
      &uc_setVCore, &ul_setMultiplier );

  /* Set VCore setting */
  _mcu_setVCore( uc_setVCore );

  /*
   * Disable FLL control loop, set lowest possible DCOx, MODx and select
   * a suitable range
   */
  __bis_SR_register( SCG0);
  UCSCTL0 = 0x00;
  UCSCTL1 = uc_setDcoRange;

  /* Set DCO multiplier and reenable the FLL control loop */
  UCSCTL2 = ul_setMultiplier + FLLD_1;
  UCSCTL4 = SELA__XT1CLK | SELS__DCOCLKDIV | SELM__DCOCLKDIV;
  __bic_SR_register( SCG0);

  /*
   * Loop until osciallator fault falgs (XT1, XT2 & DCO fault flags)
   * are cleared
   */
  do
  {
    /* Clear XT2, XT1, DCO fault flags */
    UCSCTL7 = 0;
    SFRIFG1 &= ~OFIFG;
  } while (SFRIFG1 & OFIFG);

  /*
   * Worst-case settling time for the DCO when the DCO range bits have been
   * changed is n x 32 x 32 x f_FLL_reference. See UCS chapter in 5xx UG
   * for optimization.
   * 32 x 32 x / f_FLL_reference (32,768 Hz) = .03125 = t_DCO_settle
   * t_DCO_settle / (1 / 25 MHz) = 781250 = counts_DCO_settle
   */
  __delay_cycles(781250);
}

/*============================================================================*/
/**
 * @brief    This is an assert function. It runs an infinite loop that blinks
 *           all LEDs quickly. The function assumes LEDs to be initialized by,
 *           for example,  bspInit().
 *
 * @return   None
 */
/*============================================================================*/
void _mcu_assert (void)
{
  int i = 0;
  /* Set all LEDs to the same state before the infinite loop */
  for( i = 0; i < E_LED_MAX; i++ )
    led_clear( (e_led_t)i );

  while (1)
  {
    /* Toggle LEDs */
    for( i = 0; i < E_LED_MAX; i++ )
      led_toggle( (e_led_t)i );

    /* Simple wait */
    __delay_cycles(900000);
  }
}

/*============================================================================*/
/**
 * @brief    Get function for the DCORSEL, VCORE, and DCO multiplier
 *           settings that map to a given clock speed.
 *
 * @param    ul_systemClockSpeed    is the target DCO frequency; can be one of
 *           the following values:
 *           \li \b BSP_SYS_CLK_1MHZ
 *           \li \b BSP_SYS_CLK_4MHZ
 *           \li \b BSP_SYS_CLK_8MHZ
 *           \li \b BSP_SYS_CLK_12MHZ
 *           \li \b BSP_SYS_CLK_16MHZ
 *           \li \b BSP_SYS_CLK_20MHZ
 *           \li \b BSP_SYS_CLK_25MHZ
 * @param    puc_setDcoRange         is a pointer to the DCO range select bits.
 * @param    puc_setVCore            is a pointer to the VCore level bits.
 * @param    pul_setMultiplier       is a pointer to the DCO multiplier bits.
 *
 * @return      None
 */
/*============================================================================*/
static void _mcu_getSystemClockSettings (uint32_t ul_systemClockSpeed,
    uint8_t *puc_setDcoRange, uint8_t *puc_setVCore,
    uint32_t *pul_setMultiplier)
{
  switch (ul_systemClockSpeed) {
    case MCU_SYSCLK_1MHZ:
      *puc_setDcoRange = DCORSEL_1MHZ;
      *puc_setVCore = VCORE_1MHZ;
      *pul_setMultiplier = DCO_MULT_1MHZ;
      break;
    case MCU_SYSCLK_4MHZ:
      *puc_setDcoRange = DCORSEL_4MHZ;
      *puc_setVCore = VCORE_4MHZ;
      *pul_setMultiplier = DCO_MULT_4MHZ;
      break;
    case MCU_SYSCLK_8MHZ:
      *puc_setDcoRange = DCORSEL_8MHZ;
      *puc_setVCore = VCORE_8MHZ;
      *pul_setMultiplier = DCO_MULT_8MHZ;
      break;
    case MCU_SYSCLK_12MHZ:
      *puc_setDcoRange = DCORSEL_12MHZ;
      *puc_setVCore = VCORE_12MHZ;
      *pul_setMultiplier = DCO_MULT_12MHZ;
      break;
    case MCU_SYSCLK_16MHZ:
      *puc_setDcoRange = DCORSEL_16MHZ;
      *puc_setVCore = VCORE_16MHZ;
      *pul_setMultiplier = DCO_MULT_16MHZ;
      break;
    case MCU_SYSCLK_20MHZ:
      *puc_setDcoRange = DCORSEL_20MHZ;
      *puc_setVCore = VCORE_20MHZ;
      *pul_setMultiplier = DCO_MULT_20MHZ;
      break;
    case MCU_SYSCLK_25MHZ:
      *puc_setDcoRange = DCORSEL_25MHZ;
      *puc_setVCore = VCORE_25MHZ;
      *pul_setMultiplier = DCO_MULT_25MHZ;
      break;
  }
}

/*============================================================================*/
/**
 * @brief    This function sets the PMM core voltage (PMMCOREV) setting.
 *
 * @param    uc_level        is the target VCore setting.
 *
 * @return   None
 */
/*============================================================================*/
static void _mcu_setVCore (uint8_t uc_level)
{
  uint8_t uc_actLevel;
  do
  {
    uc_actLevel = PMMCTL0_L & PMMCOREV_3;
    if (uc_actLevel < uc_level)
    {
      /* Set Vcore (step by step) */
      _mcu_setVCoreUp(++uc_actLevel);
    }
    if (uc_actLevel > uc_level)
    {
      /* Set VCore step by step */
      _mcu_setVCoreDown(--uc_actLevel);
    }
  } while( uc_actLevel != uc_level );
}

/*============================================================================*/
/**
 * @brief    Increments the VCore setting.
 *
 * @param    uc_level        is the target VCore setting.
 *
 * @return   None
 */
/*============================================================================*/
static void _mcu_setVCoreUp (uint8_t uc_level)
{
  /* Open PMM module registers for write access */
  PMMCTL0_H = 0xA5;

  /* Set SVS/M high side to new uc_level */
  SVSMHCTL = (SVSMHCTL & ~(SVSHRVL0 * 3 + SVSMHRRL0))
      | (SVSHE + SVSHRVL0 * uc_level + SVMHE + SVSMHRRL0 * uc_level);

  /* Set SVM new Level */
  SVSMLCTL = SVSLE + SVMLE + SVSMLRRL0 * uc_level;

  /* Set SVS/M low side to new level */
  SVSMLCTL = (SVSMLCTL & ~(SVSMLRRL_3)) | (SVMLE + SVSMLRRL0 * uc_level);

  /* Wait until SVM is settled */
  while ((PMMIFG & SVSMLDLYIFG) == 0)
  {
  }

  /* Set VCore to 'level' and clear flags */
  PMMCTL0_L = PMMCOREV0 * uc_level;
  PMMIFG &= ~(SVMLVLRIFG + SVMLIFG);

  if ((PMMIFG & SVMLIFG))
  {
    /* Wait until level is reached */
    while ((PMMIFG & SVMLVLRIFG) == 0)
      ;
  }

  /* Set SVS/M Low side to new level */
  SVSMLCTL = (SVSMLCTL & ~(SVSLRVL0 * 3 + SVSMLRRL_3))
      | (SVSLE + SVSLRVL0 * uc_level + SVMLE + SVSMLRRL0 * uc_level);

  /* Lock PMM module registers from write access */
  PMMCTL0_H = 0x00;
}

/*============================================================================*/
/**
 * @brief    This function decrements the VCore setting.
 *
 * @param    uc_level    is the target VCore setting.
 *
 * @return   None
 */
/*============================================================================*/
static void _mcu_setVCoreDown (uint8_t uc_level)
{
  /* Open PMM module registers for write access */
  PMMCTL0_H = 0xA5;

  /* Set SVS/M low side to new level */
  SVSMLCTL = (SVSMLCTL & ~(SVSLRVL0 * 3 + SVSMLRRL_3))
      | (SVSLRVL0 * uc_level + SVMLE + SVSMLRRL0 * uc_level);

  /* Wait until SVM is settled */
  while ((PMMIFG & SVSMLDLYIFG) == 0)
  {
  }

  /* Set VCore to new level */
  PMMCTL0_L = (uc_level * PMMCOREV0);

  /* Lock PMM module registers for write access */
  PMMCTL0_H = 0x00;
}

/*============================================================================*/
/*                              FUNCTIONS                                     */
/*============================================================================*/

/*============================================================================*/
/**
 * @brief    Function initializes the MSP430F5438a clocks and I/O for use on
 *           SmartRF06EB.
 *
 *           The function assumes an external crystal oscillator to be available
 *           to the MSP430F5438a. The MSP430F5438a main system clock (MCLK) and
 *           Sub Main System Clock (SMCLK) are set to the frequency given by
 *           input argument \e ui32SysClockSpeed. ACLK is set to 32768 Hz.
 *
 * @param    ul_sysClockSpeed   is the system clock speed in MHz; it must be
 *                               one of the following:
 *           \li \b BSP_SYS_CLK_1MHZ
 *           \li \b BSP_SYS_CLK_4MHZ
 *           \li \b BSP_SYS_CLK_8MHZ
 *           \li \b BSP_SYS_CLK_12MHZ
 *           \li \b BSP_SYS_CLK_16MHZ
 *           \li \b BSP_SYS_CLK_20MHZ
 *           \li \b BSP_SYS_CLK_25MHZ
 *
 * @return   None
 */
/*============================================================================*/
void mcu_sysClockInit (uint32_t ul_sysClockSpeed)
{
  uint16_t ul_intState;

  /* Stop watchdog timer (prevent timeout reset) */
  WDTCTL = WDTPW + WDTHOLD;

  /* Disable global interrupts */
  ul_intState = __get_interrupt_state();
  __disable_interrupt();

  /* Set capacitor values for XT1, 32768 Hz */
  _mcu_startXT1();

  /* set the clock speed */
  _mcu_sysClockSpeedSet( ul_sysClockSpeed );

  /* Return to previous interrupt state */
  __set_interrupt_state(ul_intState);
}

/*============================================================================*/
/**
 * @brief    Function returns the system clock speed.
 *
 * @return   Returns the system clock speed.
 */
/*============================================================================*/
uint32_t mcu_sysClockSpeedGet(void)
{
  /* return the current master clock speed */
  return gul_mcu_MclkSpeed;
}


/*=============================================================================
 *  int_irqRTC()
 *============================================================================*/
void mcu_enterPm( e_mcu_pm_t e_pm )
{
  /* diable interrupts (will be enabled when entering power mode) */
  __disable_interrupt();

  /* set the current power mode in interrupt module */
  int_setPm( e_pm );

  /* enter the power mode */
  switch( e_pm )
  {
    case E_MCU_PM_0:
      __bis_SR_register(LPM0_bits + GIE); /* Enter LPM0 with interrupts enabled */
      break;
    case E_MCU_PM_1:
      __bis_SR_register(LPM1_bits + GIE); /* Enter LPM1 with interrupts enabled */
      break;
    case E_MCU_PM_2:
      __bis_SR_register(LPM2_bits + GIE); /* Enter LPM2 with interrupts enabled */
      break;
    case E_MCU_PM_3:
      __bis_SR_register(LPM3_bits + GIE); /* Enter LPM3 with interrupts enabled */
      break;
    case E_MCU_PM_4:
      __bis_SR_register(LPM4_bits + GIE); /* Enter LPM4 with interrupts enabled */
      break;
    default:
      break;
  }
}/* mcu_enterPm() */


/*=============================================================================
 *  mcu_enterCS()
 *============================================================================*/
void mcu_enterCS( void )
{
  /* Disable global interrupts */
  __disable_interrupt();

} /* mcu_enterCS() */


/*=============================================================================
 *  mcu_exitCS()
 *============================================================================*/
void mcu_exitCS( void )
{
  /* enable global interrupts */
  __enable_interrupt();
} /* mcu_exitCS() */
