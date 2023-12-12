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

#ifndef USB_THREAD_INTERFACE_H_
#define USB_THREAD_INTERFACE_H_

#include "stdint.h"

extern char ei_get_serial_byte(uint8_t is_inference_running);
extern uint32_t ei_get_serial_buffer(uint8_t* pbuffer, uint32_t max_len);
extern void usb_clean_buffer(void);


#endif /* USB_THREAD_INTERFACE_H_ */
