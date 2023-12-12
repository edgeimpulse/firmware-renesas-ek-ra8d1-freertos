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

#ifndef PERIPHERAL_UART_H_
#define PERIPHERAL_UART_H_

#include "bsp_api.h"

FSP_HEADER

extern fsp_err_t uart_console_init(void);
extern void uart_print_to_console(uint8_t * p_data, uint16_t len);
extern void uart_putc(uint8_t c);
extern fsp_err_t uart_set_baud(bool is_max_baud);
extern char uart_get_rx_data(uint8_t is_inference_running);

FSP_FOOTER

#endif /* PERIPHERAL_UART_H_ */
