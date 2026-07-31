#ifndef INTERRUPT_H
#define INTERRUPT_H

#include "ti_msp_dl_config.h"

extern volatile uint32_t counter_1_A;
extern volatile uint32_t counter_2_A;

extern volatile uint8_t key_start_flag;
extern volatile uint8_t key_mode_trace;
extern volatile uint8_t key_mode_angle;
extern volatile uint8_t key_stop_flag;

#endif
