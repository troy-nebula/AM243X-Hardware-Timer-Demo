/*
 *  Copyright (C) 2021 Texas Instruments Incorporated
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <stdio.h>
#include <kernel/dpl/DebugP.h>
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"
#include <kernel/dpl/DebugP.h>
#include <FreeRTOS.h>
#include <task.h>
#include <kernel/dpl/TimerP.h>
#include "ti_dpl_config.h"
#include "ti_drivers_config.h"

/*
 * This is an empty project provided for all cores present in the device.
 * User can use this project to start their application by adding more SysConfig modules.
 *
 * This application does driver and board init and just prints the pass string on the console.
 * In case of the main core, the print is redirected to the UART console.
 * For all other cores, CCS prints are used.
 */

uint32_t gCount = 0;
uint32_t gOtherCount = 0;


void fastTimerISR() {

    gCount++;
    gOtherCount++;

}

void hwTimerDemo(void *args)
{
    /* Open drivers to open the UART driver for console */
    Drivers_open();
    Board_driversOpen();

    DebugP_log("Starting!!\r\n");

    TimerP_start(gTimerBaseAddr[CONFIG_TIMER0]);

    uint32_t timer = 1, tTarget = 0;
    uint32_t countArray[1000];
    uint32_t prevCount = gCount;
    uint32_t microseconds = 1000;
    uint32_t iterations = 1000;
    float expectedUsecPerTick = 2.48;
    float actualUsecPerTick;
    float expectedTicksPerIteration = microseconds / expectedUsecPerTick;

    for (int i = 0; i < iterations; i++) {
        tTarget = (xTaskGetTickCount() * portTICK_PERIOD_MS) + timer;
        while (xTaskGetTickCount() * portTICK_PERIOD_MS < tTarget) {}
        countArray[i] = gCount;
    }

    uint32_t sum = 0;
    for (int i = 0; i < iterations-1; i++) {
        sum += countArray[i + 1] - countArray[i];
    }
    uint32_t averageTicksPerIteration = sum / (iterations-1);
    actualUsecPerTick = (float)microseconds / (float)averageTicksPerIteration;
    DebugP_log("Actual usec per tick is %.4f.\r\n", actualUsecPerTick);

    float marginOfErrorRatio = (float)(expectedTicksPerIteration - averageTicksPerIteration) / microseconds * 100;
    DebugP_log("Margin of error is %.2f%%.\r\n", marginOfErrorRatio);
    DebugP_log("Done!!\r\n");

    Board_driversClose();
    Drivers_close();
}
