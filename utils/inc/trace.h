/**
 * @file	  trace.h
 * @date	  Apr 7, 2016
 * @author 	Phuong Nguyen
 */

#ifndef TRACE_INCLUDED
#define TRACE_INCLUDED

/* enable/disable trace module */
#ifndef TRACE_CFG_EN
#define TRACE_CFG_EN      0u
#endif

#ifndef ASSERT_CFG_EN
#define ASSERT_CFG_EN           0u
#endif

/* enable/disable function tracing */
#ifndef TRACE_CFG_FNCT_EN
#define TRACE_CFG_FNCT_EN       0U
#endif

/* overwrite implementation of C function printf */
#ifndef PRINTF
#define PRINTF            trace_printf
#endif

/**
 * @note  level0: disable
 *        level1: show main results i.e., statistic on unicast transmission
 *        level2: interrupts
 *        level3: detailed information
 */
enum {
  TRACE_LEVEL_NONE,
  TRACE_LEVEL_ERR,
  TRACE_LEVEL_MAIN,
  TRACE_LEVEL_INT,
  TRACE_LEVEL_FULL
};

#ifndef TRACE_CFG_LEVEL
#define TRACE_CFG_LEVEL             TRACE_LEVEL_ERR
#endif


#if (TRACE_CFG_FNCT_EN == 1U)
  #define TRACE_ENTER_FNCT(pname)     trace_printf(pname ": enter")
  #define TRACE_EXIT_FNCT(pname)      trace_printf(pname ": exit")
#else
  #define TRACE_ENTER_FNCT(pname)
  #define TRACE_EXIT_FNCT(pname)
#endif


#if (TRACE_CFG_EN == 1U)
  #define TRACE_LOG_ERR( pmsg, ...)   trace_log(TRACE_LEVEL_ERR,  "e " pmsg, ##__VA_ARGS__)
  #define TRACE_LOG_MAIN(pmsg, ...)   trace_log(TRACE_LEVEL_MAIN,      pmsg, ##__VA_ARGS__)
  #define TRACE_LOG_INT( pmsg, ...)   trace_log(TRACE_LEVEL_INT,       pmsg, ##__VA_ARGS__)
  #define TRACE_LOG_FULL(pmsg, ...)   trace_log(TRACE_LEVEL_FULL,      pmsg, ##__VA_ARGS__)

  #define TRACE_LOG_ENTER_ISR() \
    do {                                          \
      if (TRACE_CFG_LEVEL >= TRACE_LEVEL_INT) {   \
        trace_enter_isr();                        \
      }                                           \
    } while(0)

  #define TRACE_LOG_EXIT_ISR()  \
    do {                                          \
      if (TRACE_CFG_LEVEL >= TRACE_LEVEL_INT) {   \
        trace_exit_isr();                         \
      }                                           \
    } while(0)

  #define trace_log(level, pmsg, ...)             \
    do {                                          \
      if (TRACE_CFG_LEVEL >= level) {             \
        trace_printf((pmsg), ##__VA_ARGS__);      \
      }                                           \
    } while(0)
#else
  #define TRACE_LOG_ERR( pmsg, ...)
  #define TRACE_LOG_MAIN(pmsg, ...)
  #define TRACE_LOG_INT( pmsg, ...)
  #define TRACE_LOG_FULL(pmsg, ...)
  #define TRACE_LOG_ENTER_ISR()
  #define TRACE_LOG_EXIT_ISR()
  #define trace_log(level, pmsg, ...)
#endif

#define assert_equ(msg_, exp_, act_)    ((exp_) == (act_)) ? \
    ((void)0) : trace_printf(msg_ "failed exp=%02x, act=%02x", (exp_), (act_))


#if (TRACE_CFG_EN == 1U)
  void trace_init(void);
  void trace_start(void);
  void trace_enter_isr(void);
  void trace_exit_isr(void);
  void trace_printf(const char *s, ...);
  void trace_printHex(const char *p_msg, uint8_t *p_data, uint16_t len);
#else
  static inline void trace_init(void) {
  }

  static inline void trace_start(void) {
  }

  static inline void trace_enter_isr(void) {
  }

  static inline void trace_exit_isr(void) {
  }

  static inline void trace_printf(const char *s, ...) {
  }

  static inline void trace_printHex(const char *p_msg, uint8_t *p_data, uint16_t len) {
  }
#endif

#endif /* TRACE_INCLUDED */
