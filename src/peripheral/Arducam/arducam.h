/* The Clear BSD License
 *
 * Copyright (c) 2025 EdgeImpulse Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted (subject to the limitations in the disclaimer
 * below) provided that the following conditions are met:
 *
 *   * Redistributions of source code must retain the above copyright notice,
 *   this list of conditions and the following disclaimer.
 *
 *   * Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the distribution.
 *
 *   * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY
 * THIS LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 * CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
 * IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
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
