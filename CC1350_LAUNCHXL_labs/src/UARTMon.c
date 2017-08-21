/*
 * Copyright (c) 2017, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 *  ======== UARTMon.c ========
 */

#include <string.h>
#include <stdint.h>
#include <stdio.h>

/* POSIX Header files */
#include <pthread.h>

/* Driver Header files */
#include <ti/drivers/UART.h>
#include "UARTMon.h"
#include "globals.h"

char buffer_tx[10] = "tencharbf";

/*
 *  ======== UARTMon_init ========
 */
void UARTMon_init()
{
    pthread_t           uartmonThread;
    pthread_attr_t      attrs;
    struct sched_param  priParam;
    int                 retc;

    /* Call driver init functions */
    UART_init();

    /* Create UARTMon_taskFxn thread */
    pthread_attr_init(&attrs);

    /* Set priority and stack size attributes */
    retc = pthread_attr_setdetachstate(&attrs, PTHREAD_CREATE_DETACHED);
    if (retc != 0) {
        /* pthread_attr_setdetachstate() failed */
        while (1);
    }

    retc |= pthread_attr_setstacksize(&attrs, UARTMon_STACKSIZE);
    if (retc != 0) {
        /* pthread_attr_setstacksize() failed */
        while (1);
    }

    /* Create UARTMon_taskFxn thread */
    priParam.sched_priority = sched_get_priority_min(0);
    pthread_attr_setschedparam(&attrs, &priParam);

    retc = pthread_create(&uartmonThread, &attrs, UARTMon_taskFxn, NULL);
    if (retc != 0) {
        /* pthread_create() failed */
        while (1);
    }
}

/*
 *  ======== UARTMon_taskFxn ========
 *  Thread that serves as the UART Monitor. This thread will open an instance
 *  of the UART and perform UART read & write operations.
 */
void *UARTMon_taskFxn(void *arg0)
{
    UART_Handle uart;
    UART_Params uartParams;
    uint8_t input[UARTMon_CMDSIZE];
    uint8_t cmd;

    /* Initialize and open UART */
    UART_Params_init(&uartParams);

    uartParams.writeDataMode = UART_DATA_BINARY;
    uartParams.readDataMode = UART_DATA_BINARY;
    uartParams.readReturnMode = UART_RETURN_FULL;

    uart = UART_open(UARTMon_INDEX, (UART_Params *) &uartParams);

    if (uart == NULL) {
        /* UART_open() failed */
        while (1);
    }

    while (UART_read(uart, input, sizeof(input)) > 0) {

        /* First byte is the command */
        cmd = input[0];

        switch (cmd) {
            case 'a':
                input[0] = 'A';
                UART_write(uart, input, 1);

                break;

            case 'b':
                input[0] = 'B';
                UART_write(uart, input, 1);

                break;

            case 'c':
                /* write the data from host to the memory directly */
                //UART_read(uart, (void *)address, length);

                /* send the cmd + length byte back as status */
                //UART_write(uart, input, 1);

                //UART_write(uart, (void *)address, length);
                UART_write(uart, buffer_tx, 4);

                break;

            default:
                input[0] = 'X';
                UART_write(uart, input, 1);

                break;
        }
    }

    return (NULL);
}
