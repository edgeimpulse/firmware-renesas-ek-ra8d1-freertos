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
#include "camera_thread_interface.h"
#include "peripheral/ceu.h"
#include "common_events.h"
#include "ingestion-sdk-platform/platform/ei_device_renesas_ek_ra8d1.h"
#include "edge-impulse-sdk/porting/ei_classifier_porting.h"
#include "edge-impulse-sdk/dsp/image/image.hpp"
#include "inference/ei_run_impulse.h"
#include "graphic/graphic.h"
#include "peripheral/led.h"
#include "model-parameters/model_metadata.h"
#include "edge-impulse-sdk/classifier/ei_model_types.h"

#define CAMERA_STACK_SIZE_BYTE              (8192)
#define CAMERA_TASK_PRIORITY                (2u)
#define HEAP_STATS                          (0)

/* FreeRTOS module */
static TaskHandle_t camera_thread_handle;

static void camera_thread_entry(void *pvParameters);
static void prepare_fb(uint8_t* pbuff, int& cropWidth, int& cropHeight);

#if (EI_CLASSIFIER_OBJECT_DETECTION == 1)
static void set_lcd_detection(ei_impulse_result_bounding_box_t* pbox, uint16_t index);
#endif

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

#if EI_CLASSIFIER_OBJECT_DETECTION == 1
            if (result.bounding_boxes){
                if (result.bounding_boxes[0].value > 0) {
                    graphic_start_detection();
                    for (size_t ix = 0; ix < result.bounding_boxes_count; ix++) {
                        auto bb = result.bounding_boxes[ix];
                        if (bb.value == 0) {
                            continue;
                        }
                        // set detection
                        set_lcd_detection((ei_impulse_result_bounding_box_t*)&bb, (uint16_t)ix);
                    }
                }
                else {
                    graphic_no_detection();
                }
            }
#else
            for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++) {
                graphic_classification(result.classification[ix].label, result.classification[ix].value, (uint16_t)ix);
            }
#endif

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

#if EI_CLASSIFIER_OBJECT_DETECTION == 1
/**
 *
 * @param pbox
 * @param index
 */
static void set_lcd_detection(ei_impulse_result_bounding_box_t* pbox, uint16_t index)
{
    float ratio = (float)CAM_LAYER_SIZE_Y / (float)EI_CLASSIFIER_INPUT_HEIGHT;
#if (EI_CLASSIFIER_OBJECT_DETECTION_LAST_LAYER == EI_CLASSIFIER_LAST_LAYER_FOMO)
    graphic_set_centroid(pbox->label, pbox->x, pbox->y, pbox->width, pbox->height, (uint16_t)index, ratio);
#else
    graphic_set_box(pbox->label, pbox->x, pbox->y, pbox->width, pbox->height, (uint16_t)index, ratio);
#endif
}

#endif
