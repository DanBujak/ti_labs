#ifndef __HEARBEAT_H
#define __HEARBEAT_H

#ifdef __cplusplus
extern "C" {
#endif

/* POSIX Header files */
#include <pthread.h>

#define HB_STACKSIZE       768


void gpio_initialize(void);
void heartbeat_initialize(void);
void *hb_task_fn (void *arg0);
void gpioButtonFxn0(uint_least8_t index);
void gpioButtonFxn1(uint_least8_t index);

#endif /* __HEARBEAT_H */
