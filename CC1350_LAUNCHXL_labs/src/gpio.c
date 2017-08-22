
#include <ti/sysbios/knl/Clock.h>
#include <ti/drivers/GPIO.h>

#include "gpio.h"
#include "Board.h"

/* GPIO Initialization */
void gpio_initialize(void)
{
  GPIO_init();

  /* Turn on user LED */
  GPIO_write(Board_GPIO_RLED, Board_GPIO_LED_ON);
  GPIO_write(Board_GPIO_GLED, Board_GPIO_LED_OFF);

  /* Install button callback */
  GPIO_setCallback(Board_GPIO_BUTTON0, gpioButtonFxn0);
  GPIO_setCallback(Board_GPIO_BUTTON1, gpioButtonFxn1);

  /* Enable interrupts */
  GPIO_enableInt(Board_GPIO_BUTTON0);
  GPIO_enableInt(Board_GPIO_BUTTON1);

  /* Heartbeat initialize */
  heartbeat_initialize();
}


void heartbeat_initialize(void)
{
  pthread_t           hbThread;
  pthread_attr_t      attrs;
  struct sched_param  priParam;
  int                 retc;

  /* Create hb_task_fn thread */
  pthread_attr_init(&attrs);

  /* Set priority and stack size attributes */
  retc = pthread_attr_setdetachstate(&attrs, PTHREAD_CREATE_DETACHED);
  if (retc != 0) {
    /* pthread_attr_setdetachstate() failed */
    while (1);
  }

  retc |= pthread_attr_setstacksize(&attrs, HB_STACKSIZE);
  if (retc != 0) {
    /* pthread_attr_setstacksize() failed */
    while (1);
  }

  /* Create hb_task_fn thread */
  priParam.sched_priority = sched_get_priority_min(0);
  pthread_attr_setschedparam(&attrs, &priParam);

  retc = pthread_create(&hbThread, &attrs, hb_task_fn, NULL);
  if (retc != 0) {
    /* pthread_create() failed */
    while (1);
  }
}


void *hb_task_fn (void *arg0)
{
  while (1) {
    GPIO_toggle(Board_GPIO_GLED);
    Task_sleep(1000 * (1000 / Clock_tickPeriod));
  }

  //return (NULL);
}


/*
 *  ======== gpioButtonFxn0 ========
 *  Callback function for the GPIO interrupt on Board_GPIO_BUTTON0.
 */
void gpioButtonFxn0(uint_least8_t index)
{
    /* Toggle an LED and increment count */
    GPIO_toggle(Board_GPIO_LED0);
}

/*
 *  ======== gpioButtonFxn1 ========
 *  Callback function for the GPIO interrupt on Board_GPIO_BUTTON1.
 *  This may not be used for all boards.
 */
void gpioButtonFxn1(uint_least8_t index)
{
    /* Toggle an LED and decrement count */
    GPIO_toggle(Board_GPIO_LED1);
}
