/*
 * Copyright (c) 2023 EdgeImpulse Inc.
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
#include "event_groups.h"
#include "camera_thread_interface.h"
#include "peripheral/ceu.h"
#include "common_events.h"
#include "ingestion-sdk-platform/platform/ei_device_renesas_ek_ra8d1.h"
#include "edge-impulse-sdk/porting/ei_classifier_porting.h"
#include "edge-impulse-sdk/dsp/image/image.hpp"
#include "inference/ei_run_impulse.h"
#include "graphic/graphic.h"
#include "peripheral/led.h"

#define CAMERA_STACK_SIZE_BYTE              (8192)
#define CAMERA_TASK_PRIORITY                (2u)
#define HEAP_STATS                          (0)

/* FreeRTOS module */
static TaskHandle_t camera_thread_handle;

static void camera_thread_entry(void *pvParameters);
static void set_lcd_centroids(ei_impulse_result_bounding_box_t* pbox, uint16_t index);
static void prepare_fb(uint8_t* pbuff, int& cropWidth, int& cropHeight);

/**
 *
 */
bool start_camera_thread(void)
{
    BaseType_t retval;
    /* create a task to send data via usb */
    retval = xTaskCreate(camera_thread_entry,
        (const char*) "Camera Thread",
        CAMERA_STACK_SIZE_BYTE / 4, // in words
        NULL, //pvParameters
        CAMERA_TASK_PRIORITY, //uxPriority
        &camera_thread_handle);

    return (pdPASS == retval);
}

/**
 * @brief Camera Thread entry function
 * @param pvParameters
 */
static void camera_thread_entry(void *pvParameters)
{
    FSP_PARAMETER_NOT_USED (pvParameters);
    EventBits_t   camera_event;
    ei_impulse_result_t result;

    uint32_t old_time = 0;
    uint32_t new_time = 0;
    uint32_t fsp = 0.0;

    EiCameraArduCam* cam = static_cast<EiCameraArduCam*>(EiCameraArduCam::get_camera());
    if (cam->is_camera_present() == false) {
        ei_printf("Err: camera not found!");
        while(1) {
            ei_led_toggle(e_user_led_red);
            vTaskDelay(500);
        }
    }

    ceu_init();
    // let's start with the inference settings
    cam->start_lcd_stream(CAMERA_STREAM_WIDTH, CAMERA_STREAM_HEIGHT);
    int finalW;
    int finalH;

    while (1) {

        if (cam->is_lcd_stream_active() == true) {

#if HEAP_STATS == 1
            HeapStats_t heap_stats;

            vPortGetHeapStats(&heap_stats);

            ei_printf("xAvailableHeapSpaceInBytes %d\r\n", heap_stats.xAvailableHeapSpaceInBytes);
            ei_printf("xSizeOfLargestFreeBlockInBytes %d\r\n", heap_stats.xSizeOfLargestFreeBlockInBytes);
            ei_printf("xSizeOfSmallestFreeBlockInBytes %d\r\n", heap_stats.xSizeOfSmallestFreeBlockInBytes);
            ei_printf("xNumberOfFreeBlocks %d\r\n", heap_stats.xNumberOfFreeBlocks);
            //ei_printf("xMinimumEverFreeBytesRemaining %d\r\n", heap_stats.xMinimumEverFreeBytesRemaining);
            ei_printf("xNumberOfSuccessfulAllocations %d\r\n", heap_stats.xNumberOfSuccessfulAllocations);
            ei_printf("xNumberOfSuccessfulFrees %d\r\n", heap_stats.xNumberOfSuccessfulFrees);

#endif

            ei_led_turn_on(e_user_led_green);
            cam->lcd_stream();

            // prepare buffer
            prepare_fb(cam->get_lcd_stream(), finalW, finalH);

            ei_run_stream_impulse(cam->get_lcd_stream(), finalW, finalH, &result);
            ei_led_turn_off(e_user_led_green);

            graphic_set_framebuffer(cam->get_lcd_stream(), finalW, finalH);

            new_time = (uint32_t)ei_read_timer_ms();
            fsp = 1000/(new_time - old_time);
            old_time = new_time;
            graphic_set_timing(fsp, (int32_t)result.timing.dsp_us, (int32_t)result.timing.classification_us);

            if (result.bounding_boxes[0].value > 0) {
                graphic_start_detection();
                for (size_t ix = 0; ix < result.bounding_boxes_count; ix++) {
                    auto bb = result.bounding_boxes[ix];
                    if (bb.value == 0) {
                        continue;
                    }
                    // set box
                    set_lcd_centroids((ei_impulse_result_bounding_box_t*)&bb, (uint16_t)ix);
                }
            }
            else {
                graphic_no_detection();
            }

            xSemaphoreGive(g_new_fb_semaphore); // counting semaphore
            camera_event = xEventGroupWaitBits(
                                    g_camera_event_group,       /* The event group being tested. */
                                    CAMERA_STOP_LCD_INFERENCE,  /* The bits within the event group to wait for. */
                                    pdTRUE,         /* should be cleared before returning. */
                                    pdFALSE,        /* Don't wait for both bits, either bit will do. */
                                    0 );            /* Returns immediatly */

            if (camera_event & CAMERA_STOP_LCD_INFERENCE) {
                cam->stop_lcd_stream();
            }
        }
        else if (cam->is_stream_active()) {
            cam->run_stream();

            camera_event = xEventGroupWaitBits(
                                    g_camera_event_group,       /* The event group being tested. */
                                    CAMERA_STOP_STREAM,  /* The bits within the event group to wait for. */
                                    pdTRUE,         /* should be cleared before returning. */
                                    pdFALSE,        /* Don't wait for both bits, either bit will do. */
                                    0);            /* Returns immediatly */

            if (camera_event & CAMERA_STOP_STREAM) {
                cam->stop_stream();
            }
        }
        else {
            camera_event = xEventGroupWaitBits(
                        g_camera_event_group,   /* The event group being tested. */
                        CAMERA_EVENT_REQUEST | CAMERA_START_LCD_INFERENCE | CAMERA_START_STREAM, /* The bits within the event group to wait for. */
                        pdTRUE,        /* should be cleared before returning. */
                        pdFALSE,       /* Don't wait for both bits, either bit will do. */
                        portMAX_DELAY );/* Wait forever */

            if (camera_event & CAMERA_START_LCD_INFERENCE) {    // we prioritize start of lcd stream
                cam->start_lcd_stream(CAMERA_STREAM_WIDTH, CAMERA_STREAM_HEIGHT);
            }
        }

        //
        vTaskDelay(5);
    }
}

/**
 * @brief crop camera buffer
 *
 * @param pbuff
 * @param cropWidth
 * @param cropHeight
 */
static void prepare_fb(uint8_t* pbuff, int& cropWidth, int& cropHeight)
{
    int pixel_size_B = 2;

    ei::image::processing::calculate_crop_dims(CAMERA_STREAM_WIDTH, CAMERA_STREAM_HEIGHT,
            EI_CLASSIFIER_INPUT_WIDTH, EI_CLASSIFIER_INPUT_HEIGHT,
            cropWidth, cropHeight);

    ei::image::processing::cropImage(pbuff,
            CAMERA_STREAM_WIDTH * pixel_size_B, CAMERA_STREAM_HEIGHT,
            ((CAMERA_STREAM_WIDTH - cropWidth) / 2) * pixel_size_B,
            (CAMERA_STREAM_HEIGHT - cropHeight) / 2,
            pbuff,
            cropWidth * pixel_size_B,
            cropHeight,
            8);
}

/**
 *
 * @param pbox
 * @param index
 */
static void set_lcd_centroids(ei_impulse_result_bounding_box_t* pbox, uint16_t index)
{
    float ratio = (float)CAM_LAYER_SIZE_Y / (float)EI_CLASSIFIER_INPUT_HEIGHT;
    //
    graphic_set_box(pbox->label, pbox->x, pbox->y, pbox->width, pbox->height, (uint16_t)index, ratio);
}
