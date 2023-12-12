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

#ifndef PERIPHERAL_CEU_H_
#define PERIPHERAL_CEU_H_

#include "bsp_api.h"

FSP_CPP_HEADER

typedef enum {
    e_ceu_96_64,
    e_ceu_96_96,
    e_ceu_160_120,
    e_ceu_176_144,
    e_ceu_320_240,
    e_ceu_res_max
}t_ceu_supported_resolution;

extern uint32_t ceu_start_capture(t_ceu_supported_resolution res);
extern uint32_t ceu_capture(uint8_t * const p_buffer);
extern void ceu_stop_capture(void);
extern int ceu_init(void);
extern int ceu_deinit(void);

FSP_CPP_FOOTER

#endif /* PERIPHERAL_CEU_H_ */
