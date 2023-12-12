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

#ifndef PERIPHERAL_FLASH_HANDLER_H_
#define PERIPHERAL_FLASH_HANDLER_H_

#include "bsp_api.h"
#include <stdint.h>

typedef enum e_flash_memory
{
    e_flash_code = 0,
    e_flash_data,
    e_flash_max
}t_flash_memory;

/*
 * programming data flash   : 4/8/16 bytes
 * erase data flash         : 64/182/256 bytes
 */
FSP_CPP_HEADER

#define FLASH_HANDLER_ERASE_TIME                        (250u)      /* in ms */

#define FLASH_HANDLER_AFTER_CLEAR                       (0xFF)

/* Code Flash */
#define FLASH_HP_CODE_FLASH_START_ADDRESS_FIRST_HALF    (0x02000000U)
#define FLASH_HP_CODE_FLASH_END_ADDRESS_FIRST_HALF      (0x02100000U)

#define FLASH_HP_CODE_FLASH_START_ADDRESS_SECOND_HALF   (0x02100000U)
#define FLASH_HP_CODE_FLASH_END_ADDRESS_SECOND_HALF     (0x02200000U)

#define FLASH_HP_CODE_FLASH_BLOCK_SIZE_32KB             (32*1024)   /* Block Size 32 KB */
#define FLASH_HP_CODE_FLASH_BLOCK_SIZE_8KB              (8*1024)    /* Block Size 8KB */

#define FLASH_HP_CODE_FLASH_BLOCK_WR_SIZE               (128u)

#define FLASH_HP_CODE_8KB_BLOCK_NUM                     (8u)
#define FLASH_HP_CODE_32KB_BLOCK_NUM                    (69 - 9) /* 96 total blocks - 8k blocks */

/* Data Flash */
#define FLASH_HP_DATA_FLASH_BLOCK_SIZE              (64)
#define FLASH_HP_DATA_FLASH_BLOCK_NUM               (192)

#define FLASH_HP_DATA_FLASH_BLOCK_WR_SIZE           (4u)

#define FLASH_HP_DATA_FLASH_SIZE                    (FLASH_HP_DATA_FLASH_BLOCK_SIZE * FLASH_HP_DATA_FLASH_BLOCK_NUM)

#define FLASH_HP_DATA_FLASH_START_ADDRESS           (0x27000000)
#define FLASH_HP_DATA_FLASH_END_ADDRESS             (FLASH_HP_DATA_FLASH_START_ADDRESS + FLASH_HP_DATA_FLASH_SIZE)

extern int flash_handler_init(void);
extern uint32_t flash_handler_erase(t_flash_memory flash_mem, uint32_t address, uint32_t bytes_to_clear);
extern uint32_t flash_handler_write(t_flash_memory flash_mem, uint32_t address, const uint8_t *data, uint32_t size);
extern uint32_t flash_handler_get_write_size(uint32_t desired_size, uint32_t write_size);
extern uint32_t flash_handler_get_blocks_number(uint32_t desired_size, uint32_t block_size);

FSP_CPP_FOOTER

#endif /* PERIPHERAL_FLASH_HANDLER_H_ */
