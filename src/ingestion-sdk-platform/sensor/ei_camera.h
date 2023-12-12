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

#ifndef EI_CAMERA_ARDUCAM_H
#define EI_CAMERA_ARDUCAM_H

#include <peripheral/Arducam/arducam.h>
#include "firmware-sdk/ei_camera_interface.h"
#include "firmware-sdk/ei_device_info_lib.h"

typedef enum {
    eicamera_resolution_96x64 = e_ceu_96_64,
    eicamera_resolution_96x96 = e_ceu_96_96,
    eicamera_resolution_160x120 = e_ceu_160_120,
    eicamera_resolution_176x144 = e_ceu_176_144,    // QCIF
    eicamera_resolution_320x240 = e_ceu_320_240,    // QVGA
    eicamera_resolution_max
}t_eicamera_resolution;

class EiCameraArduCam : public EiCamera {
private:
    ArduCAM *cam;
    static ei_device_snapshot_resolutions_t resolutions[];
    bool stream_active;
    bool lcd_stream_active;
    uint32_t width;
    uint32_t height;
    uint32_t output_width;
    uint32_t output_height;
    bool stream_do_resize;
    uint8_t *stream_buffer;
    uint32_t stream_buffer_size;
    bool camera_present;
    t_eicamera_resolution actual_resolution;

public:
    EiCameraArduCam();
    bool initialize_camera(void);

    bool ei_camera_capture_jpeg(uint8_t *image, uint32_t *image_size, uint16_t width, uint16_t height);
    bool ei_camera_capture_rgb888_packed_big_endian(uint8_t *image, uint32_t image_size);
    bool is_camera_present(void);

    bool init(uint16_t width, uint16_t height) override;
    bool deinit(void) override;
    void get_resolutions(ei_device_snapshot_resolutions_t **res, uint8_t *res_num) override;
    bool set_resolution(const ei_device_snapshot_resolutions_t res) override;
    ei_device_snapshot_resolutions_t get_min_resolution(void) override;

    bool start_stream(uint32_t width_to_set, uint32_t height_to_set);
    bool run_stream(void);
    bool is_stream_active(void);
    bool stop_stream(void);

    bool is_lcd_stream_active(void);
    bool start_lcd_stream(uint32_t width_to_set, uint32_t height_to_set);
    bool lcd_stream(void);
    void stop_lcd_stream(void);
    uint8_t* get_lcd_stream(void) {return stream_buffer;};

    bool get_snapshot_and_send(uint16_t width, uint16_t height);
};

extern bool RBG565ToRGB888(uint8_t *src_buf, uint8_t *dst_buf, uint32_t src_len);
extern bool ei_camera_read_encode_send_sample_buffer(const char *input, size_t length);

#endif
