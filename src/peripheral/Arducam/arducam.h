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
/*
    Based on ArduCAM.cpp by Lee
    https://github.com/ArduCAM/Arduino/tree/3577fbd1b5a9f4afbe9923718fa0f265d671dc3a
    http://www.ArduCAM.com
*/

#ifndef ARDUCAM_H_
#define ARDUCAM_H_

#include <cstdint>
#include "hal_data.h"
#include "peripheral/ceu.h"

struct sensor_reg {
    uint8_t reg;
    uint8_t val;
};

struct sensor_reg_wA {
    uint16_t reg;
    uint8_t val;
};

typedef enum {
    e_arducam_sleep,
    e_arducam_stream,
    e_arducam_capture,
    e_arducam_off
}t_arducam_state;

typedef enum {
    e_format_rgb565,
    e_format_rgb555,
    e_format_jpg,
    e_format_raw,
    e_format_yuv422,
    e_format_max
}t_image_format;

typedef enum {
    e_camera_ov2640,
    e_camera_ov3640,
    e_no_camera
}t_camera_type;

class ArduCAM {
    public:
        ArduCAM(void);
        ~ArduCAM(void) {};

        void hw_init(void);
        void DeinitCAM(void);
        uint32_t capture_frame(uint8_t *buf);

        bool detect_camera(void);
        void init_config(void);
        void set_format(t_image_format format);
        void set_size(uint8_t size);
        void flip(bool flip_image);
    
    protected:
        t_arducam_state state;
        t_ceu_supported_resolution ceu_resolution;
        t_image_format format;

        // Write 8 bit values to 16 bit register address
        int wrSensorRegs8_8(const struct sensor_reg *, uint16_t size);

        // Read/write 8 bit value to/from 8 bit register address
        int wrSensorReg8_8(uint8_t regID, uint8_t regDat);

        // Read/write 8 bit value to/from 16 bit register address
        int rdSensorReg8_8(uint8_t regID, uint8_t *regDat);

        // Write 8 bit values to 16 bit register address
        int wrSensorRegs16_8(const struct sensor_reg_wA *);

        // Read/write 8 bit value to/from 16 bit register address
        int wrSensorReg16_8(uint16_t regID, uint8_t regDat);
        int rdSensorReg16_8(uint16_t regID, uint8_t *regDat);


    private:
        t_camera_type           my_type;
        rm_comms_instance_t const*   p_i2c_instance;

        bool ov2640_detect_camera(void);
        void ov2640_init_config(void);
        void ov2640_set_format(t_image_format format);
        void ov2640_set_size(uint8_t size);
        bool ov2640_set_manual_size(uint16_t width, uint16_t height, uint8_t index);
        bool ov2640_set_manual_size(uint16_t width, uint16_t height);
        void ov2640_flip(bool flip_image);

        void ov2640_wakeup(void);
        void ov2640_sleep(void);

        bool ov3640_detect_camera(void);
        void ov3640_init_config(void);
        void ov3640_set_format(t_image_format format);
        void ov3640_set_size(uint8_t size);

        void ov3640_wakeup(void);
        void ov3640_sleep(void);
        void ov3640_center_image(uint16_t width, uint16_t height);
        void ov3640_flip(bool flip_image);
};

#endif
