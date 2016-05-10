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
#define ASSERT_CFG_EN     0u
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


#define TRACE_LOG_ERR( pmsg, ...)   trace_log(TRACE_LEVEL_ERR,  "! " pmsg, ##__VA_ARGS__)
#define TRACE_LOG_MAIN(pmsg, ...)   trace_log(TRACE_LEVEL_MAIN,      pmsg, ##__VA_ARGS__)
#define TRACE_LOG_INT( pmsg, ...)   trace_log(TRACE_LEVEL_INT,       pmsg, ##__VA_ARGS__)
#define TRACE_LOG_FULL(pmsg, ...)   trace_log(TRACE_LEVEL_FULL,      pmsg, ##__VA_ARGS__)


#define TRACE_LOG_ENTER_ISR() \
    do {                                        \
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

#define assert_equ(msg_, exp_, act_)    ((exp_) == (act_)) ? \
    ((void)0) : trace_printf(msg_ "failed exp=%02x, act=%02x", (exp_), (act_))


#if (TRACE_CFG_EN == FALSE)
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


#else
void trace_init(void);
void trace_start(void);
void trace_enter_isr(void);
void trace_exit_isr(void);
void trace_printf(const char *s, ...);
void trace_printHex(const char *p_msg, uint8_t *p_data, uint16_t len);
#endif

#endif /* TRACE_INCLUDED */
