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
#include "semphr.h"
#include "ceu.h"
#include "hal_data.h"
#include "stdbool.h"
#include "r_ceu.h"

ceu_instance_ctrl_t* pceu[] = {
   &g_ceu_96_64_ctrl,
   &g_ceu_96_96_ctrl,
   &g_ceu_160_120_ctrl,
   &g_ceu_176_144_ctrl,
   &g_ceu_320_240_ctrl,
};

capture_cfg_t const* pceu_config[] = {
    &g_ceu_96_64_cfg,
    &g_ceu_96_96_cfg,
    &g_ceu_160_120_cfg,
    &g_ceu_176_144_cfg,
    &g_ceu_320_240_cfg,
};

// Local variables
volatile capture_event_t ceu_event;
static t_ceu_supported_resolution actual_res;
static SemaphoreHandle_t sem_ceu_operation;

static volatile bool end_cycle = false;
static volatile bool error = false;
static volatile uint32_t err_code;

static uint32_t ceu_wait_end_operation(void);

/**
 *
 * @param p_buffer
 * @return
 */
uint32_t ceu_start_capture(t_ceu_supported_resolution res)
{
    uint32_t err = 0;
    capture_status_t _ceu_state;

    if (res >= e_ceu_res_max) {
        return 0;
    }

   err = R_CEU_Open(pceu[res], pceu_config[(int)res]);
   R_CEU_StatusGet(pceu[res], &_ceu_state);

    if (_ceu_state.state != CAPTURE_STATE_IDLE) {
        // peripheral is busy
        return FSP_ERR_IN_USE;
    }

    actual_res = res;

    return err;
}

/**
 *
 * @param p_buffer
 * @return
 */
uint32_t ceu_capture(uint8_t * const p_buffer)
{
    uint32_t err = 0;
    capture_status_t _ceu_state;

    R_CEU_StatusGet(pceu[actual_res], &_ceu_state);
    if (_ceu_state.state != CAPTURE_STATE_IDLE) {
        // peripheral is busy
        return FSP_ERR_IN_USE;
    }

    if (p_buffer != NULL) {
        err = R_CEU_CaptureStart(pceu[actual_res], p_buffer);
    }

    if (err == FSP_SUCCESS) {
        ceu_event = CEU_EVENT_NONE;
        err = ceu_wait_end_operation();
    }
    ceu_event = CEU_EVENT_NONE;

    return err;
}

/**
 *
 */
void ceu_stop_capture(void)
{
    if (actual_res < e_ceu_res_max) {
        R_CEU_Close(pceu[actual_res]);
    }
}

/**
 *
 * @return
 */
int ceu_init(void)
{
    fsp_err_t err = FSP_SUCCESS;

    ceu_event = CEU_EVENT_NONE;
    actual_res = e_ceu_res_max;

    sem_ceu_operation = xSemaphoreCreateBinary();

    if (sem_ceu_operation == NULL) {
        err = FSP_ERR_OUT_OF_MEMORY;
    }

    return (int)err;
}

/**
 *
 * @return
 */
int ceu_deinit(void)
{
    fsp_err_t err = FSP_SUCCESS;
    vSemaphoreDelete(sem_ceu_operation);

    return (int)err;
}

/**
 * @brief Wait till CEU completed or any error
 * @return
 */
static uint32_t ceu_wait_end_operation(void)
{
    end_cycle = false;
    error = false;
    err_code = 0;

    if (xSemaphoreTake(sem_ceu_operation, 2000) == pdFALSE) {
        err_code = CEU_EVENT_VD_MISSING;
    }

    if ((end_cycle == false) && (err_code == 0)){
        // do something ?
        err_code = CEU_EVENT_VD_MISSING;
    }

    return (err_code);
}

/**
 *
 * @param p_args
 */
void g_ceu_user_callback(capture_callback_args_t *p_args)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    bool unlock = false;

    switch(p_args->event) {
        case CEU_EVENT_FRAME_END:
        {
            end_cycle = true;
            unlock = true;
        }
        break;
        case CEU_EVENT_HD:
        {

        }
        break;
        case CEU_EVENT_VD:
        {

        }
        break;
        case CEU_EVENT_CRAM_OVERFLOW:
        {
            error = true;
            err_code = CEU_EVENT_CRAM_OVERFLOW;
            unlock = true;
        }
        break;
        case CEU_EVENT_HD_MISMATCH:
        {
            error = true;
            err_code = CEU_EVENT_HD_MISMATCH;
            unlock = true;
        }
        break;
        case CEU_EVENT_VD_MISMATCH:
        {
            error = true;
            err_code = CEU_EVENT_VD_MISMATCH;
            unlock = true;
        }
        break;
        case CEU_EVENT_VD_ERROR:
        {
            error = true;
            err_code = CEU_EVENT_VD_ERROR;
            unlock = true;
        }
        break;
        case CEU_EVENT_FIREWALL:
        {
            error = true;
            err_code = CEU_EVENT_FIREWALL;
            unlock = true;
        }
        break;
        case CEU_EVENT_HD_MISSING:  // problematic handling...
        {
            //error = true;
            //err_code = CEU_EVENT_HD_MISSING;
        }
        break;
        case CEU_EVENT_VD_MISSING:
        {
            error = true;
            err_code = CEU_EVENT_VD_MISSING;
            unlock = true;
        }
        break;
        default:
        {

        }
        break;
    }

    if (unlock) {
        xSemaphoreGiveFromISR(sem_ceu_operation, &xHigherPriorityTaskWoken);
    }

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}
