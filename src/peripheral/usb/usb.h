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

#ifndef COMMS_H_
#define COMMS_H_

#include "hal_data.h"

#if defined(__cplusplus)
extern "C" {
#endif

#define RAW_COUNT_MS    120000
#define WAIT_FOREVER    0xFFFFFFFF
#define MAX_PACKET_SIZE 64


/* Function prototypes */
fsp_err_t comms_open(uint8_t wait);
fsp_err_t comms_send(uint8_t * p_src, uint32_t len, uint32_t period);
fsp_err_t comms_read(uint8_t * p_dest, uint32_t * len, uint32_t timeout_milliseconds);
fsp_err_t comms_close(void);
bool comms_get_is_open(void);

void usb_set_high_speed(void);
uint32_t usb_get_speed(void);

#if defined(__cplusplus)
}
#endif

#endif /* COMMS_H_ */
