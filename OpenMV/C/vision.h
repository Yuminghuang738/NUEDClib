#ifndef VISION_H
#define VISION_H

#include <stdint.h>

void process_deviation(void);
void process_deviation_task3(void);

/* 任务三调试变量 */
extern volatile int16_t  dbg_t3_desired;
extern volatile int16_t  dbg_t3_current;
extern volatile int8_t   dbg_t3_dir;
extern volatile uint8_t  dbg_t3_state;

#endif