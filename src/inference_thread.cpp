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
#include "event_groups.h"
#include "inference_thread_interface.h"
#include "inference/ei_run_impulse.h"
#include "ingestion-sdk-platform/platform/ei_device_renesas_ek_ra8d1.h"
#include "edge-impulse-sdk/porting/ei_classifier_porting.h"
#include "common_events.h"

#define INFERENCE_TASK_STACK_SIZE_BYTE        (4096u)
#define INFERENCE_TASK_PRIORITY               (4u)

/* FreeRTOS module */
static TaskHandle_t inference_thread_handle = NULL;
static void inference_thread_entry(void *pvParameters);
static bool lcd_stream_was_active;

/**
 * @brief start inference thread
 * @return
 */
bool start_inference_thread(void)
{
    BaseType_t retval;
    EiCameraArduCam* cam = static_cast<EiCameraArduCam*>(EiCameraArduCam::get_camera());

    lcd_stream_was_active = cam->is_lcd_stream_active();

    if (lcd_stream_was_active == true) {
        xEventGroupSetBits(g_camera_event_group, CAMERA_STOP_LCD_INFERENCE);    // signal stop stream
        while(cam->is_lcd_stream_active() == true) {
            ei_sleep(10);
        }
    }

    if (inference_thread_handle != NULL) {
        ei_printf("Inference Thread already running!\r\n");
        return false;
    }

    /* create a task to send data via usb */
    retval = xTaskCreate(inference_thread_entry,
        (const char*) "Inference Thread",
        INFERENCE_TASK_STACK_SIZE_BYTE / 4, // in words
        NULL, //pvParameters
        INFERENCE_TASK_PRIORITY, //uxPriority
        &inference_thread_handle);

    return (pdPASS == retval);
}

/**
 *
 * @param pvParameters
 */
static void inference_thread_entry(void *pvParameters)
{
    (void)pvParameters;

    do {
        ei_run_impulse();
    }while(is_inference_running() == true);

    if (lcd_stream_was_active == true) {
        xEventGroupSetBits(g_camera_event_group, CAMERA_START_LCD_INFERENCE);    // signal start lcd stream
    }

    if (inference_thread_handle != NULL) {
        inference_thread_handle = NULL;
        vTaskDelete(NULL);
    }

    while (1) {
        vTaskDelay(portMAX_DELAY);
    }
}

