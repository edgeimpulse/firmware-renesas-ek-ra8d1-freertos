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

#ifndef EI_FLASH_MEMORY_H
#define EI_FLASH_MEMORY_H

#include "firmware-sdk/ei_device_memory.h"
#include "peripheral/flash_handler.h"

class EiFlashMemory : public EiDeviceMemory {
private:
    const uint16_t flash_type;
    const uint32_t base_address;
    const uint16_t write_size_multiple;

    uint8_t residual_to_write;
    uint8_t residual_array[128];
    uint32_t last_offset;

    uint32_t bytes_written;

protected:

public:
    void write_residual(void);
    uint32_t read_sample_data(uint8_t *sample_data, uint32_t address, uint32_t sample_data_size);
    uint32_t write_sample_data(uint8_t *sample_data, uint32_t address, uint32_t sample_data_size);
    uint32_t erase_sample_data(uint32_t address, uint32_t num_bytes);

    uint32_t read_data(uint8_t *data, uint32_t address, uint32_t num_bytes);
    uint32_t write_data(const uint8_t *data, uint32_t address, uint32_t num_bytes);
    uint32_t erase_data(uint32_t address, uint32_t num_bytes);

public:
    EiFlashMemory(uint16_t to_set_flash_type, uint32_t config_struct_size);
};

#endif /* EI_FLASH_MEMORY_H */
