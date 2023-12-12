/*
 * Copyright (c) 2022 EdgeImpulse Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
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
