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

#include "usb.h"
#include "r_usb_pcdc_api.h"
#include "r_usb_basic.h"
#include "task.h"
#include <stdbool.h>

#define USB_DATA_LEN              30000 // 1000 // 50000 // 10000 //  // 1024 //
#define MAX_PACKET_SIZE           64
#define LINE_CODING_LENGTH        7
#define CONTROL_LINE_STATE_LENGTH 1

typedef enum data_bits
{
    DATA_BITS_5 = 5,
    DATA_BITS_6,
    DATA_BITS_7,
    DATA_BITS_8,
    DATA_BITS_16 = 16,
}data_bits_t;

typedef enum parity_type
{
    PARITY_NONE = 0,
    PARITY_ODD,
    PARITY_EVEN,
    PARITY_MARK,
    PARITY_SPACE,
}parity_type_t;

typedef enum stop_bits
{
    STOP_BITS_1 = 0,
    STOP_BITS_1_5,
    STOP_BITS_2,
}stop_bits_t;

volatile usb_pcdc_linecoding_t g_line_coding_in_use = {0};
/* 115200 8n1 by default */
usb_pcdc_linecoding_t g_line_coding = {0};

volatile usb_pcdc_ctrllinestate_t g_control_line_state = {
    .bdtr = 0,
    .brts = 0,
};

usb_pcdc_linecoding_t g_line_coding_fs = {
    .dw_dte_rate    = (1843200), // 115200, //
    .b_char_format = STOP_BITS_1,
    .b_parity_type = PARITY_NONE,
    .b_data_bits   = DATA_BITS_8,
};

uint8_t g_comms_opened_flag = 0;

extern usb_instance_ctrl_t g_basic0_ctrl;
extern const usb_cfg_t g_basic0_cfg;

fsp_err_t comms_send_low(uint8_t * p_src, uint32_t len, uint32_t period);

/**
 *
 * @param event
 * @param handle
 * @param onoff
 */
void usb_cdc_rtos_callback(usb_event_info_t * event, usb_hdl_t handle, usb_onoff_t onoff)
{
    //FSP_PARAMETER_NOT_USED(event);
    FSP_PARAMETER_NOT_USED(handle);
    FSP_PARAMETER_NOT_USED(onoff);

    BaseType_t xHigherPriorityTaskWoken;

    usb_setup_t             setup;

    /* We have not woken a task at the start of the ISR. */
    xHigherPriorityTaskWoken = pdFALSE;

    switch (event->event)
    {
        case USB_STATUS_CONFIGURED :
        break;
        case USB_STATUS_WRITE_COMPLETE :
            if (pdTRUE == xSemaphoreGiveFromISR(g_usb_write_complete_binary_semaphore, &xHigherPriorityTaskWoken)) {
                __NOP();
            }
        break;
        case USB_STATUS_READ_COMPLETE :
            if (pdTRUE == xQueueSendFromISR(g_usb_read_queue, &event->data_size, &xHigherPriorityTaskWoken ) ) {
                __NOP();
            }

        break;
        case USB_STATUS_REQUEST : /* Receive Class Request */
            g_usb_on_usb.setupGet(event, &setup);
            if (USB_PCDC_GET_LINE_CODING == (setup.request_type & USB_BREQUEST)) {
                g_usb_on_usb.periControlDataSet(event, (uint8_t *) &g_line_coding, LINE_CODING_LENGTH);
            }
            else if (USB_PCDC_SET_LINE_CODING == (setup.request_type & USB_BREQUEST)) {
                /* Configure virtual UART settings */
                g_usb_on_usb.periControlDataGet(&g_basic0_ctrl, (uint8_t *) &g_line_coding, LINE_CODING_LENGTH);
                if (g_line_coding.dw_dte_rate != g_line_coding_in_use.dw_dte_rate) {
                    g_line_coding_in_use.dw_dte_rate = g_line_coding.dw_dte_rate ;
                }
            }
            else if (USB_PCDC_SET_CONTROL_LINE_STATE == (event->setup.request_type & USB_BREQUEST))
            {
                //g_usb_on_usb.periControlStatusSet(&g_basic0_ctrl, USB_SETUP_STATUS_ACK);
                fsp_err_t err = g_usb_on_usb.periControlDataGet(event, (uint8_t *) &g_control_line_state, sizeof(g_control_line_state));
                if (FSP_SUCCESS == err) {
                    g_control_line_state.bdtr = (unsigned char)((event->setup.request_value >> 0) & 0x01);
                    g_control_line_state.brts = (unsigned char)((event->setup.request_value >> 1) & 0x01);
                    g_comms_opened_flag = 1;
                    g_usb_on_usb.periControlStatusSet(&g_basic0_ctrl, USB_SETUP_STATUS_ACK);
                }

            }
            else
            {
                /* none */
            }
        break;
        case USB_STATUS_REQUEST_COMPLETE :
            __NOP();
        break;
        case USB_STATUS_SUSPEND :
        case USB_STATUS_DETACH :
        case USB_STATUS_DEFAULT :
            __NOP();
        break;
        default :
            __NOP();
        break;
    }

    /* We can switch context if necessary. */
    /* Actual macro used here is port specific. */
    portYIELD_FROM_ISR (xHigherPriorityTaskWoken);
}

/**
 *
 * @param wait
 * @return
 */
fsp_err_t comms_open(uint8_t wait)
{
    fsp_err_t err;

    err = g_usb_on_usb.open(&g_basic0_ctrl, &g_basic0_cfg);
    if (FSP_SUCCESS != err) {
        return (err);
    }

    if (wait)
    {
        /* Wait for the application to open the COM port */
        while (0 == g_control_line_state.bdtr) {
            vTaskDelay(1);
        }

        if (pdTRUE == xSemaphoreGive(g_usb_ready)) {
            __NOP();
        }

        g_comms_opened_flag = 1;
        vTaskDelay (500);
    }
    return FSP_SUCCESS;
}

/**
 *
 * @param p_src
 * @param len
 * @param period
 * @return
 */
fsp_err_t comms_send(uint8_t * p_src, uint32_t len, uint32_t period)
{
    fsp_err_t       err = FSP_SUCCESS;
    uint32_t i = 0;
    uint32_t offset = 0;
    uint32_t len1 = 0;

    if (0 == g_comms_opened_flag) {
        return FSP_ERR_USB_NOT_OPEN;
    }

    while(1) {
        i++;
        if (len - offset >= USB_DATA_LEN) {
            len1 = USB_DATA_LEN;
        }
        else {
            len1 = len - offset;
        }


        err = comms_send_low(&(p_src[offset]), len1, period);
        if (err == FSP_SUCCESS) {
            offset += len1;
        }


        if (offset >= len)
            break;
    }

    return err;
}

/**
 *
 * @param p_src
 * @param len
 * @param period
 * @return
 */
fsp_err_t comms_send_low(uint8_t * p_src, uint32_t len, uint32_t period)
{
    fsp_err_t       err;
    uint8_t timeout = 0;

    timeout = 2;
    timeout += len/1024;

    err = g_usb_on_usb.write(&g_basic0_ctrl, p_src, len,  USB_CLASS_PCDC);
    if (FSP_SUCCESS != err)
    {
        return  err;
    }

    if (period != 0) {
        if (period == 0xFFFFFFFF) {
            if( xSemaphoreTake( g_usb_write_complete_binary_semaphore, portMAX_DELAY ) == pdTRUE ) {
                __NOP();
            }
            else {
                err = FSP_ERR_ABORTED;
            }
        }
        else
        {
            //vTaskDelay (period);
            if( xSemaphoreTake( g_usb_write_complete_binary_semaphore, period ) == pdTRUE ) {
                __NOP();
            }
            else {
                err = FSP_ERR_ABORTED;
            }
        }
    }
    else {
        vTaskDelay (1);
    }

    return  err;
}

fsp_err_t comms_read(uint8_t * p_dest, uint32_t * len, uint32_t timeout_milliseconds)
{
    fsp_err_t       err             = FSP_SUCCESS;
    BaseType_t      fr_err;
    TickType_t      timeout;

    if (0 == g_comms_opened_flag) {
        return FSP_ERR_USB_NOT_OPEN;
    }

    err = g_usb_on_usb.read(&g_basic0_ctrl, p_dest, 512,  USB_CLASS_PCDC);
    if (FSP_SUCCESS != err) {
        return FSP_ERR_USB_FAILED;
    }

    if(timeout_milliseconds == portMAX_DELAY) {
        timeout = portMAX_DELAY;
    }
    else {
        timeout = timeout_milliseconds / portTICK_PERIOD_MS;
    }

    /* Wait for the USB Read to complete */
    *len = 0;
    fr_err = xQueueReceive(g_usb_read_queue, len, timeout);
    if (pdTRUE == fr_err)
    {
        return FSP_SUCCESS;
    }
    else
    {
        /* If there was a timeout, we need to terminate the USB transfer */
        //trans.type            = USB_CLASS_PCDC;
        //trans.module_number   = USB_CFG_USE_USBIP;
        //err = g_usb_on_usb.stop(&g_ctrl, USB_TRANSFER_READ, &trans);
        err = g_usb_on_usb.stop(&g_basic0_ctrl, USB_TRANSFER_READ, USB_CLASS_PCDC);

        *len = 0;

        return FSP_ERR_TIMEOUT;
    }
}

/**
 *
 * @return
 */
bool comms_get_is_open(void)
{
    return (g_comms_opened_flag != 0);
}

/**
 *
 * @return
 */
fsp_err_t comms_close(void)
{
    fsp_err_t err;

    err = g_usb_on_usb.close(&g_basic0_ctrl);
    if (FSP_SUCCESS != err) {
        return (err);
    }

    return FSP_SUCCESS;
}

/**
 *
 * @return
 */
uint32_t usb_get_speed(void)
{
    return g_line_coding.dw_dte_rate;
}
