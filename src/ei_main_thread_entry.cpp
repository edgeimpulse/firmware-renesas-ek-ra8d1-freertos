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
#include "event_groups.h"

#include "ei_main_thread.h"
#include "bsp_api.h"
#include "common_events.h"
#include "camera_thread_interface.h"
#include "display_thread_interface.h"
#include "peripheral/led.h"
#include "peripheral/timer_handler.h"
#include "peripheral/flash_handler.h"
#include "peripheral/i2c.h"
#include "inference/ei_run_impulse.h"
#include "edge-impulse-sdk/porting/ei_classifier_porting.h"

#include "ingestion-sdk-platform/platform/ei_at_handlers.h"
#include "ingestion-sdk-platform/platform/ei_device_renesas_ek_ra8d1.h"
#if (USE_UART == 1)
#include <peripheral/uart.h>
#else
#include "usb_thread_interface.h"
#endif

/* Private variables -------------------------------------------------------------------- */
static ATServer *at;
static EiDeviceRenesasEKRA8D1* p_dev;


/* EI Main Thread entry function */
/* pvParameters contains TaskHandle_t */
void ei_main_thread_entry(void *pvParameters)
{
    FSP_PARAMETER_NOT_USED (pvParameters);

    bool in_rx_loop = false;
    EventBits_t   event_bit;
    EiCameraArduCam* cam = static_cast<EiCameraArduCam*>(EiCameraArduCam::get_camera());

    ei_timer_init();
    ei_timer0_start();
    ei_i2c_init();

    flash_handler_init();

    p_dev =  static_cast<EiDeviceRenesasEKRA8D1*>(EiDeviceInfo::get_device());

#if (USE_UART == 1)
    if (uart_console_init() == FSP_SUCCESS) {
        at = ei_at_init(p_dev);
        ei_led_turn_on(e_user_led_green);
    }
    else {
        ei_led_turn_on(e_user_led_red);
    }
#else
    // usb has his own task
    at = ei_at_init(p_dev);
    ei_led_turn_on(e_user_led_green);
#endif

    if (start_display_thread() == false) {
        while (1) {
            __NOP();
        };
    }

    if (start_camera_thread() == false) {
        while (1) {
            __NOP();
        };
    }

    ei_printf("Type AT+HELP to see a list of commands.\r\n");
    ei_printf("Starting main loop\r\n");

    at->print_prompt();

    while (1)
    {
        event_bit = xEventGroupWaitBits(g_common_event, EVENT_RX_READY | CAMERA_READY, pdTRUE, pdFALSE , portMAX_DELAY);

        // rx console event
        if (event_bit & EVENT_RX_READY) {
            /* handle command comming from uart */
#if (USE_UART == 1)
            char data = uart_get_rx_data(is_inference_running());
#else
            char data = ei_get_serial_byte(is_inference_running());
#endif
            in_rx_loop = false;

            while ((uint8_t)data != 0xFF) {
                if ((is_inference_running() == true) && (data == 'b') && (in_rx_loop == false)) {
                    ei_stop_impulse();
                    at->print_prompt();
                    continue;
                }

                if ((cam->is_stream_active()) && (data =='b') && (in_rx_loop == false)) {
                  xEventGroupSetBits(g_camera_event_group, CAMERA_STOP_STREAM);    // signal stop stream
                  while(cam->is_stream_active() == true) {
                      ei_sleep(1);
                  }
                  at->print_prompt();
                  xEventGroupSetBits(g_camera_event_group, CAMERA_START_LCD_INFERENCE);    // signal start lcd stream
                  continue;
                }

                in_rx_loop = true;
                at->handle(data);
#if (USE_UART == 1)
                data = uart_get_rx_data(is_inference_running());
#else
                data = ei_get_serial_byte(is_inference_running());
#endif
            }
        }

    }
}
