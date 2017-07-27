#ifndef TIMER_H_
#define TIMER_H_

#include "stm8l15x.h"
#include "stm8l15x_tim4.h"

// Time Parameters
// Could be CLK_SYSCLKDiv_1, CLK_SYSCLKDiv_2, CLK_SYSCLKDiv_4
// while bigger value is supposed to save power
#define SYS_CLOCK_DIVIDER       CLK_SYSCLKDiv_2

// We will make it to 2ms, (1/16MHz)*128*250 = 2mS or (1/8MHz)*128*125 = 2mS
#define TIM4_PERIOD             (0xFA >> SYS_CLOCK_DIVIDER)

// We will check the time at every 2mS *5 = 10mS
#define TIM4_CHECK_TICKS        5

// Idle duration before enter low power mode
#define TIMEOUT_IDLE            1500            // The unit is 10 ms, so the duration is 15 s.

typedef void (*TM4_CallBack_t)();

extern TM4_CallBack_t TIM4_10ms_handler;

extern u16 tmrIdleDuration;

// Timer function pointers
typedef void (*app_timer_timeout_handler_t)(uint8_t);

void timer_init(void);

void timer_create(u8* timer_index, u8 timer_tag, app_timer_timeout_handler_t timerout_hander);

void timer_start(u8 timer_index, u32 duration);

void timer_stop(u8 timer_index);

void tick_timeout_handler(void);

#endif // TIMER_H_
