/*
 * Copyright (c) 2022 EdgeImpulse Inc.
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

#ifndef PERIPHERAL_I2C_HANDLER_H_
#define PERIPHERAL_I2C_HANDLER_H_

#include "bsp_api.h"
#include "hal_data.h"

FSP_CPP_HEADER

#define I2C_NO_WAIT_BETWEEN_W_R     (0)

extern int ei_i2c_init(void);
extern int ei_i2c_deinit(void);
extern int ei_i2c_write(rm_comms_ctrl_t * const p_api_ctrl, uint8_t*data, uint16_t bytes);
extern int ei_i2c_read_byte_command(rm_comms_ctrl_t * const p_api_ctrl, uint8_t read_cmd, uint8_t* read_data, uint16_t bytes);
extern int ei_i2c_read_word_command(rm_comms_ctrl_t * const p_api_ctrl, uint16_t read_cmd, uint8_t* read_data, uint16_t bytes);

FSP_CPP_FOOTER

#endif /* PERIPHERAL_I2C_HANDLER_H_ */
