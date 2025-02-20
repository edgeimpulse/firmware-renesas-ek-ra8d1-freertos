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
#include "led.h"
#include "bsp_pin_cfg.h"

static const bsp_io_port_pin_t _led_vector[e_user_led_max] =
{
 USER_LED3_RED,
 USER_LED2_GREEN,
 USER_LED1_BLUE
};


/**
 * @brief turn led off
 *
 * @param led
 */
void ei_led_turn_off(t_user_led led)
{
    if (led < e_user_led_max){
        R_BSP_PinAccessEnable();
        R_BSP_PinWrite((bsp_io_port_pin_t)_led_vector[led], BSP_IO_LEVEL_LOW);
        R_BSP_PinAccessDisable();
    }
}

/**
 * @brief turn led on
 *
 * @param led
 */
void ei_led_turn_on(t_user_led led)
{
    if (led < e_user_led_max){
        R_BSP_PinAccessEnable();
        R_BSP_PinWrite((bsp_io_port_pin_t)_led_vector[led], BSP_IO_LEVEL_HIGH);
        R_BSP_PinAccessDisable();
    }
}

void ei_led_toggle(t_user_led led)
{
    if (led < e_user_led_max){
        R_BSP_PinAccessEnable();

        if (R_BSP_PinRead((bsp_io_port_pin_t)_led_vector[led]) == BSP_IO_LEVEL_HIGH) {
            R_BSP_PinWrite((bsp_io_port_pin_t)_led_vector[led], BSP_IO_LEVEL_LOW);
        }
        else {
            R_BSP_PinWrite((bsp_io_port_pin_t)_led_vector[led], BSP_IO_LEVEL_HIGH);
        }
        R_BSP_PinAccessDisable();
    }
}
