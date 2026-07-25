#ifndef TRACE_H
#define TRACE_H

#include "ti_msp_dl_config.h"
#include <stdint.h>
#include <stdbool.h>

/* ── Raw sensor data (public for debug / OLED display) ── */
extern uint8_t trace_data[4];

/* ── Sensor API ── */
void    trace_read(void);              /* read 4 IR sensors into trace_data[]   */
int8_t  trace_get_error(void);         /* weighted line‑position error -10..+10 */
uint8_t trace_get_active(void);        /* how many sensors see white (1..4)     */
bool    trace_inner_both_line(void);   /* true when inner pair (X1,X3) on line   */

/* Convenience macros — call trace_read() first */
#define trace_is_cross()  (trace_get_active() == 0)   /* all on black line     */
#define trace_is_lost()   (trace_get_active() == 4)   /* all on white bg       */

#endif
