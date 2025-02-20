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

#include <peripheral/uart.h>
#include "FreeRTOS.h"
#include "event_groups.h"
#include "common_events.h"

#include "hal_data.h"

/* Macro definition */
#define CARRIAGE_ASCII            (13u)     /* Carriage return */

#define TRANSFER_LENGTH           (1024)

/*
 * Private function declarations
 */
const sci_b_baud_setting_t g_uart9_baud_max_setting =
        {
        /* Baud rate calculated with 1.725% error. */.baudrate_bits_b.abcse = 0,
          .baudrate_bits_b.abcs = 0, .baudrate_bits_b.bgdm = 1, .baudrate_bits_b.cks = 0, .baudrate_bits_b.brr = 1, .baudrate_bits_b.mddr =
                  (uint8_t) 256,
          .baudrate_bits_b.brme = false };

const sci_b_baud_setting_t g_uart9_baud_default_setting =
        {
        /* Baud rate calculated with 1.725% error. */.baudrate_bits_b.abcse = 0,
          .baudrate_bits_b.abcs = 0, .baudrate_bits_b.bgdm = 1, .baudrate_bits_b.cks = 0, .baudrate_bits_b.brr = 15, .baudrate_bits_b.mddr =
                  (uint8_t) 256,
          .baudrate_bits_b.brme = false };

static uint8_t  g_out_of_band_received[TRANSFER_LENGTH];

static SemaphoreHandle_t sem_uart_operation;

/* Flag RX completed */
static volatile uint8_t g_uart_rx_completed = false;
static volatile bool g_transfer_complete = false;
static volatile bool g_receive_complete  = false;

/* Counter to update g_temp_buffer index */
static volatile uint16_t g_rx_index = 0;

/* Index of data sent to at hanlder */
static volatile uint16_t g_uart_rx_read_index = 0;

/**
 *
 * @return
 */
fsp_err_t uart_console_init(void)
{
    fsp_err_t fsp_err = FSP_SUCCESS;

    fsp_err = R_SCI_B_UART_Open(&g_uart9_ctrl, &g_uart9_cfg);

    /* Certain SCI9 pins cannot be configured correctly through the pin configurator at the moment. */
    R_IOPORT_PinCfg(&g_ioport_ctrl, BSP_IO_PORT_10_PIN_14, ((uint32_t) IOPORT_CFG_PERIPHERAL_PIN | (uint32_t) IOPORT_PERIPHERAL_SCI1_3_5_7_9));
    R_IOPORT_PinCfg(&g_ioport_ctrl, BSP_IO_PORT_10_PIN_15, ((uint32_t) IOPORT_CFG_PERIPHERAL_PIN | (uint32_t) IOPORT_PERIPHERAL_SCI1_3_5_7_9));

    sem_uart_operation = xSemaphoreCreateBinary();
    xSemaphoreGive(sem_uart_operation); // counting semaphore

    return fsp_err;
}

/**
 *
 * @param p_data
 */
void uart_print_to_console(uint8_t * p_data, uint16_t len)
{
    fsp_err_t err = FSP_SUCCESS;
    EventBits_t   event_bit;

    //SCB_CleanDCache();
    //SCB_EnableDCache();

    xSemaphoreTake(sem_uart_operation, portMAX_DELAY); // counting semaphore

    g_transfer_complete = false;

    err = R_SCI_B_UART_Write(&g_uart9_ctrl, (uint8_t *)p_data, len);

    //assert(FSP_SUCCESS == err);

    if (err == FSP_SUCCESS) {
        event_bit = xEventGroupWaitBits(g_common_event, EVENT_TX_DONE, pdTRUE, pdFALSE , 1000);
        //while (g_transfer_complete == false) {
            //vTaskDelay(1);
        //}
    }

    xSemaphoreGive(sem_uart_operation); // counting semaphore

    //SCB_DisableDCache();
    //SCB_CleanDCache();

    //return (err);
}

/**
 *
 * @param c
 */
void uart_putc(uint8_t c)
{
    EventBits_t   event_bit;

    xSemaphoreTake(sem_uart_operation, portMAX_DELAY); // counting semaphore

    g_transfer_complete = false;

    R_SCI_B_UART_Write(&g_uart9_ctrl, &c, 1);

    event_bit = xEventGroupWaitBits(g_common_event, EVENT_TX_DONE, pdTRUE, pdFALSE , 1000);
    while(g_transfer_complete == false) {
        //vTaskDelay(1);
    };

    xSemaphoreGive(sem_uart_operation); // counting semaphore
}

/**
 *
 * @param is_max_baud
 * @return
 */
fsp_err_t uart_set_baud(bool is_max_baud)
{
    fsp_err_t err = FSP_SUCCESS;

    xSemaphoreTake(sem_uart_operation, portMAX_DELAY); // counting semaphore

    if (is_max_baud == true) {
        err = R_SCI_B_UART_BaudSet(&g_uart9_ctrl, &g_uart9_baud_max_setting);
    }
    else{
        err = R_SCI_B_UART_BaudSet(&g_uart9_ctrl, &g_uart9_baud_default_setting);
    }

    xSemaphoreGive(sem_uart_operation); // counting semaphore

    return err;
}

/**
 * @brief Returns the local rx uart buffer only if reception is complete (ie received a '\r\n') or if inference is running (just care of 'b')
 *
 * @param is_inference_running
 * @return
 */
char uart_get_rx_data(uint8_t is_inference_running)
{
    char ret_val = -1;

    if (g_uart_rx_completed == true)
    {
        if (g_uart_rx_read_index < g_rx_index)
        {
            ret_val = (char)g_out_of_band_received[g_uart_rx_read_index++];

            if (g_uart_rx_read_index == g_rx_index)
            {
                g_rx_index = 0;
                g_uart_rx_read_index = 0;
                g_uart_rx_completed = false;
            }
        }
        else
        {
            g_rx_index = 0;
            g_uart_rx_read_index = 0;
        }
    }
    else if (is_inference_running)
    {
        /* need to check if any 'b' */
        uint16_t i;

        for (i = g_uart_rx_read_index; i< g_rx_index; i ++)
        {
            if (g_out_of_band_received[i] == 'b')
            {
                ret_val = 'b';
                break;
            }
        }
    }

    return ret_val;
}

/* callback from driver */
void jlink_console_callback(uart_callback_args_t *p_args)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    /* Handle the UART event */
    switch (p_args->event)
    {
        /* Received a character */
        case UART_EVENT_RX_CHAR:
        {
            /* Only put the next character in the receive buffer if there is space for it */
            if (sizeof(g_out_of_band_received) > g_rx_index) {
                g_out_of_band_received[g_rx_index++] = (uint8_t) p_args->data;

                if (p_args->data == CARRIAGE_ASCII) {
                    g_uart_rx_completed = true;
                    xEventGroupSetBitsFromISR(g_common_event, EVENT_RX_READY, &xHigherPriorityTaskWoken);
                }
                else if (p_args->data == 'b') {
                    g_uart_rx_completed = true;
                    xEventGroupSetBitsFromISR(g_common_event, EVENT_RX_READY, &xHigherPriorityTaskWoken);
                }
            }
        }
        break;
        /* Receive complete */
        case UART_EVENT_RX_COMPLETE:
        {
            g_receive_complete = true;
        }
        break;
        /* Transmit complete */
        case UART_EVENT_TX_COMPLETE:
        {
            g_transfer_complete = true;
            xEventGroupSetBitsFromISR(g_common_event, EVENT_TX_DONE, &xHigherPriorityTaskWoken);
        }
        break;
        case UART_EVENT_TX_DATA_EMPTY:
        {

        }
        break;
        default:
        {
        }
        break;
    }

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}
