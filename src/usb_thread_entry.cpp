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
#include "usb_thread.h"
#include "peripheral/usb/usb.h"
#include <string.h>
#include "common_events.h"

#define UART_RX_BUFFER_SIZE         2048
static uint8_t g_temp_buffer[UART_RX_BUFFER_SIZE] = {0};

/* Counter to update g_temp_buffer index */
static uint32_t g_rx_index = 0;
/* Index of data sent to at hanlder */
static uint32_t g_uart_rx_read_index = 0;

static void usb_read(void);

void usb_thread_entry(void *pvParameters)
{
    FSP_PARAMETER_NOT_USED (pvParameters);
    fsp_err_t err = FSP_SUCCESS;

    /* Open the comms driver */
    err = comms_open((uint8_t)true);
    if (FSP_SUCCESS != err)
    {
        /* Stop as comms close failure */
        __BKPT(1);
    }

    while (1)
    {
        usb_read();
        vTaskDelay (1);
    }
}


/**
 * @brief Returns char from uart rx buffer
 *
 * @param is_inference_running If inference is running, we need to check for a single 'b'
 * @return
 */
char ei_get_serial_byte(uint8_t is_inference_running)
{
    FSP_PARAMETER_NOT_USED(is_inference_running);
    char to_send = -1;

    if (g_rx_index != 0) {
        to_send = g_temp_buffer[g_uart_rx_read_index++];    // get one
    }

    if ((g_uart_rx_read_index == g_rx_index) && (g_uart_rx_read_index != 0)) {  // when equal and different from zero
        g_uart_rx_read_index = 0;   // reset
        g_rx_index = 0;
        memset(g_temp_buffer, 0, sizeof(g_temp_buffer));
    }

    return to_send;
}

/**
 *
 * @return
 */
uint32_t ei_get_serial_buffer(uint8_t* pbuffer, uint32_t max_len)
{
    uint32_t sent = 0;

    if (g_rx_index > 0) {
        if (max_len < g_rx_index) {
            sent = max_len;
        }
        else {
            sent = g_rx_index;
        }
        memcpy(pbuffer, g_temp_buffer, sent);
        g_uart_rx_read_index = 0;   // reset
        g_rx_index = 0;
        memset(g_temp_buffer, 0, sizeof(g_temp_buffer));
    }

    return sent;
}

/**
 *
 */
void usb_clean_buffer(void)
{
    g_uart_rx_read_index = 0;   // reset
    g_rx_index = 0;
    memset(g_temp_buffer, 0, sizeof(g_temp_buffer));
}

/**
 * @brief Handles blockin read
 */
static void usb_read(void)
{
    uint32_t read = 0;
    if (comms_read(&g_temp_buffer[0], &read, portMAX_DELAY) == FSP_SUCCESS) {
        g_rx_index += read;
        xEventGroupSetBits(g_common_event, EVENT_RX_READY);
    }
}
