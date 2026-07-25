#ifndef CONTROL_CONFIG_H
#define CONTROL_CONFIG_H

/* ═══════════════════════════════════════════════════════════════════════════
 *  Control mode selection — enable ONE
 *
 *     #define CONTROL_OPEN_LOOP     → open‑loop  (duty = STRAIGHT ± error × GAIN)
 *     #define CONTROL_CLOSED_LOOP   → closed‑loop (PID speed control)
 * ═══════════════════════════════════════════════════════════════════════════ */

#define CONTROL_OPEN_LOOP
// #define CONTROL_CLOSED_LOOP

#endif
