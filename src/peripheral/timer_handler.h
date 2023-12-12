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
#ifndef PERIPHERAL_TIMER_HANDLER_H_
#define PERIPHERAL_TIMER_HANDLER_H_

#include "bsp_api.h"

FSP_CPP_HEADER

extern void ei_timer_init(void);
extern void ei_timer0_start(void);
extern void ei_timer0_stop(void);
extern void ei_timer3_start(void);
extern void ei_timer3_stop(void);
extern uint32_t timer_get_ms(void);
extern uint32_t timer_get_us(void);

FSP_CPP_FOOTER

#endif /* PERIPHERAL_TIMER_HANDLER_H_ */
