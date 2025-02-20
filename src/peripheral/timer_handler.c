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

/* Includes */
#include "timer_handler.h"
#include "hal_data.h"
#include <stdint.h>

#define MICROSECONDS_TO_SECONDS 1000000

static uint64_t timer_overflow_times;
static uint64_t div_ratio = 0;

static inline void set_timer_overflow_times(uint64_t value);
static inline uint64_t get_timer_overflow_times(void);
static inline uint64_t get_timer_count(void);

/* public functions */
/**
 *
 */
void ei_timer_init(void)
{
    fsp_err_t err = FSP_SUCCESS;
    timer_info_t info;

    err = R_GPT_Open (&g_timer_us_ctrl, &g_timer_us_cfg);
    err = R_GPT_Open (&g_timer_CCLK_ctrl, &g_timer_CCLK_cfg);

    if (err != FSP_SUCCESS)
    {
        while(1) {
            __NOP();
        }
    }

    (void) R_GPT_InfoGet(&g_timer_us_ctrl, &info);

    div_ratio = (info.clock_frequency / MICROSECONDS_TO_SECONDS);
}

/**
 *
 */
void ei_timer0_start(void)
{
    fsp_err_t err = FSP_SUCCESS;

    /* Start GPT module - no error control for now */
    err = R_GPT_Start (&g_timer_us_ctrl);
    set_timer_overflow_times(0);
}

/**
 *
 */
void ei_timer3_start(void)
{
    fsp_err_t err = FSP_SUCCESS;

    /* Stop GPT module - no error control for now */
    err =  R_GPT_Start(&g_timer_CCLK_ctrl);

}

/**
 *
 */
void ei_timer0_stop(void)
{
    fsp_err_t err = FSP_SUCCESS;

    /* Stop GPT module - no error control for now */
    err =  R_GPT_Stop(&g_timer_us_ctrl);
}

/**
 *
 */
void ei_timer3_stop(void)
{
    fsp_err_t err = FSP_SUCCESS;

    /* Stop GPT module - no error control for now */
    err =  R_GPT_Stop(&g_timer_CCLK_ctrl);
}

/**
 * @brief callback function for interrupt
 *
 * @param p_args
 */
void periodic_timer_msgq_cb(timer_callback_args_t *p_args)
{
    if (TIMER_EVENT_CYCLE_END == p_args->event) {
        set_timer_overflow_times(get_timer_overflow_times() + 1);
    }
}

/**
 *
 * @return
 */
uint32_t timer_get_ms(void)
{
    return (timer_get_us()/1000u);
}

/**
 *
 * @return
 */
uint32_t timer_get_us(void)
{
    uint64_t overflow_time = ((uint64_t)1 << 32) / div_ratio;
    return (uint32_t)((get_timer_overflow_times() * overflow_time)
         + (get_timer_count()/div_ratio));
}


/**
 *
 * @param value
 */
static inline void set_timer_overflow_times(uint64_t value)
{
    timer_overflow_times = value;
}

/**
 *
 * @return
 */
static inline uint64_t get_timer_overflow_times(void)
{
    return timer_overflow_times;
}

/**
 *
 * @return
 */
static inline uint64_t get_timer_count(void)
{
    timer_status_t status;
    R_GPT_StatusGet(&g_timer_us_ctrl, &status);
    return status.counter;
}
