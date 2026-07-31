#ifndef TRACE_H
#define TRACE_H

#include "ti_msp_dl_config.h"
#include <stdint.h>
#include <stdbool.h>

extern uint8_t trace_data[6];

void    trace_read(void);
int8_t  trace_get_error(void);
uint8_t trace_get_active(void);

#define trace_is_cross()  (trace_get_active() >= 3)
#define trace_is_lost()   (trace_get_active() == 0)

#endif
