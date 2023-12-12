/*
 * Copyright (c) 2023 EdgeImpulse Inc.+
 *
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

#include "peripheral/led.h"
#include "bsp_api.h"

extern void vApplicationMallocFailedHook( void );

/**
 *
 */
void vApplicationMallocFailedHook( void )
{
    ei_led_turn_off(e_user_led_red);
    ei_led_turn_off(e_user_led_green);
    ei_led_turn_off(e_user_led_blue);

    while(1) {
        ei_led_toggle(e_user_led_red);
        ei_led_toggle(e_user_led_green);
        ei_led_toggle(e_user_led_blue);
        R_BSP_SoftwareDelay(500, BSP_DELAY_UNITS_MILLISECONDS);
    }
}
