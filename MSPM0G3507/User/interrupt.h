#ifndef INTERRUPT_H
#define INTERRUPT_H

#include "ti_msp_dl_config.h"

extern volatile uint32_t counter_1_A;
extern volatile uint32_t counter_2_A;
extern volatile uint32_t encoder_total;
extern volatile uint8_t  key_start_flag;
extern volatile uint8_t  key_task3_flag;
extern volatile uint8_t  key_task456_flag;

#endif
