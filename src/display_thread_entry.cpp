/*
 * Copyright (c) 2023 EdgeImpulse Inc.+
 *
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing,
 * software distributed under the License is distributed on an "AS
 * IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either
 * express or implied. See the License for the specific language
 * governing permissions and limitations under the License.
 *
 * SPDX-License-Identifier: Apache-2.0
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
