/* The Clear BSD License
 *
 * Copyright (c) 2025 EdgeImpulse Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted (subject to the limitations in the disclaimer
 * below) provided that the following conditions are met:
 *
 *   * Redistributions of source code must retain the above copyright notice,
 *   this list of conditions and the following disclaimer.
 *
 *   * Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the distribution.
 *
 *   * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY
 * THIS LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 * CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
 * IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "FreeRTOS.h"
#include "task.h"
#include "display_thread_interface.h"
#include "graphic/graphic.h"
#include "hal_data.h"

#include "peripheral/led.h"

#define DISPLAY_TASK_STACK_SIZE_BYTE        (1024u)
#define DISPLAY_TASK_PRIORITY               (3u)

/* FreeRTOS module */
static TaskHandle_t display_thread_handle;

static void display_thread_entry(void *pvParameters);

/**
 *
 * @return
 */
bool start_display_thread(void)
{
    BaseType_t retval;

    graphic_init();

    /* create a task to send data via usb */
    retval = xTaskCreate(display_thread_entry,
        (const char*) "Display Thread",
        DISPLAY_TASK_STACK_SIZE_BYTE / 4, // in words
        NULL, //pvParameters
        DISPLAY_TASK_PRIORITY, //uxPriority
        &display_thread_handle);

    return (pdPASS == retval);
}

/* Display Thread entry function */
/* pvParameters contains TaskHandle_t */
static void display_thread_entry(void *pvParameters)
{
    FSP_PARAMETER_NOT_USED (pvParameters);

    while (1) {
        // wait vsync
        xSemaphoreTake(g_vsync_display, portMAX_DELAY);

        if (xSemaphoreTake(g_new_fb_semaphore, 0) == pdTRUE) {  // sync on new image to be displayed
            /* prepare frame buffer */
            graphic_start_buffer();

            // update
            graphic_display_draw();

            /* end of graphic operation */
            graphic_end_frame();


            /* Swap the active framebuffer */
            display_swap_buffer();

            /* Now that the framebuffer is ready, update the GLCDC buffer pointer on the next Vsync */
            display_change_buffer();
        }
    }

    graphic_deinit();
}
