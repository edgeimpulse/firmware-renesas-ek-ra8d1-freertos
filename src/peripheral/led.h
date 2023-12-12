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

#ifndef PERIPHERAL_LED_H_
#define PERIPHERAL_LED_H_

#include "bsp_api.h"

typedef enum
{
    e_user_led_red,
    e_user_led_green,
    e_user_led_blue,
    e_user_led_max,
}t_user_led;

FSP_CPP_HEADER

extern void ei_led_turn_off(t_user_led led);
extern void ei_led_turn_on(t_user_led led);
extern void ei_led_toggle(t_user_led led);

FSP_CPP_FOOTER

#endif /* PERIPHERAL_LED_H_ */
