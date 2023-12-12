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


#include "ei_camera.h"
#include "ingestion-sdk-platform/platform/ei_device_renesas_ek_ra8d1.h"
#include "edge-impulse-sdk/porting/ei_classifier_porting.h"
#include "edge-impulse-sdk/dsp/image/image.hpp"
#include "firmware-sdk/at_base64_lib.h"
#include <cmath>
#include "common_events.h"

#if (USE_UART == 1)
#include <peripheral/uart.h>
#else
#include <peripheral/usb/usb.h>
#endif

#define ALIGN_PTR(p,a)   ((p & (a-1)) ?(((uintptr_t)p + a) & ~(uintptr_t)(a-1)) : p)
#define MEASURE_TIME        (0)
#define USE_SDRAM           (1)

#if (USE_SDRAM == 1)
uint8_t           ei_camera_frame_buffer     [ 320 * 240 * 2 ] BSP_PLACE_IN_SECTION(".sdram") BSP_ALIGN_VARIABLE(8);
#else
static uint8_t *ei_camera_frame_mem = NULL;
static uint8_t *ei_camera_frame_buffer = NULL; // 32-byte aligned
#endif

ei_device_snapshot_resolutions_t EiCameraArduCam::resolutions[] = {
       { .width = 96, .height = 64 },
       { .width = 96, .height = 96 },
       { .width = 160, .height = 120 },
       { .width = 176, .height = 144 },
       { .width = 320, .height = 240 },
    };

EiCameraArduCam::EiCameraArduCam()
{
    static ArduCAM camera_object;

    this->cam = &camera_object;

    stream_active = false;
    this->camera_present = false;

    actual_resolution = eicamera_resolution_320x240;

    width = EiCameraArduCam::resolutions[actual_resolution].width;
    height = EiCameraArduCam::resolutions[actual_resolution].height;

    stream_do_resize = false;
    stream_buffer = nullptr;
    stream_buffer_size = 0;

    lcd_stream_active = false;

    output_height = 0;
    output_width = 0;
}

/**
 *
 * @return
 */
bool EiCameraArduCam::initialize_camera(void)
{
    this->cam->hw_init();

    camera_present = this->cam->detect_camera();

    if (camera_present == true) {

        this->cam->init_config();

        this->actual_resolution = eicamera_resolution_320x240;
        this->cam->set_format(e_format_rgb565);        
    }
    else {
        // deinit timer ?
        this->cam->DeinitCAM();
    }

    return camera_present;
}

/**
 *
 * @param width
 * @param height
 * @return
 */
bool EiCameraArduCam::init(uint16_t width, uint16_t height)
{
    // try to set required resolution, returned is possible
    ei_device_snapshot_resolutions_t sensor_res = this->search_resolution(width, height);

    if(set_resolution(sensor_res) == 0) {
        ei_printf("ERR: Failed to set camera resolution!\n");
        return false;
    }

    return true;
}

/**
 *
 * @return
 */
bool EiCameraArduCam::deinit(void)
{

    return true;
}

/**
 *
 * @param res
 * @return
 */
bool EiCameraArduCam::set_resolution(const ei_device_snapshot_resolutions_t res)
{
    bool ret = false;

    if(res.width == 96 && res.height == 64) {
        cam->set_size(eicamera_resolution_96x64);
        actual_resolution = (t_eicamera_resolution)eicamera_resolution_96x64;
        ret = true;
    }
    else if(res.width == 96 && res.height == 96) {
        cam->set_size(eicamera_resolution_96x96);
        actual_resolution = (t_eicamera_resolution)eicamera_resolution_96x96;
        ret = true;
    }
    else if(res.width == 160 && res.height == 120) {
        cam->set_size(eicamera_resolution_160x120);
        actual_resolution = (t_eicamera_resolution)eicamera_resolution_160x120;
        ret = true;
    }
    else if(res.width == 176 && res.height == 144) {
        cam->set_size(eicamera_resolution_176x144);
        actual_resolution = (t_eicamera_resolution)eicamera_resolution_176x144;
        ret = true;
    }
    else if(res.width == 320 && res.height == 240) {
        cam->set_size(eicamera_resolution_320x240);
        actual_resolution = (t_eicamera_resolution)eicamera_resolution_320x240;
        ret = true;
    }
    else {
        //
    }        

    if(ret == true) {
        width = res.width;
        height = res.height;
    }
    else {
        width = 0;
        height = 0;
    }

    return ret;
}

/**
 *
 * @return
 */
ei_device_snapshot_resolutions_t EiCameraArduCam::get_min_resolution(void)
{
    return resolutions[0];
}

/**
 *
 * @return
 */
EiCamera* EiCamera::get_camera(void)
{
    static EiCameraArduCam cam;

    return &cam;
}

/**
 *
 * @return
 */
bool EiCameraArduCam::is_camera_present(void)
{
    return camera_present;
}

/**
 *
 * @param image
 * @param image_size
 * @param width_to_set
 * @param height_to_set
 * @return
 */
bool EiCameraArduCam::ei_camera_capture_jpeg(uint8_t *image, uint32_t *image_size, uint16_t width_to_set, uint16_t height_to_set)
{
    ei_printf("Error ei_camera_capture_jpeg not implemented\n");
    return false;
}

/**
 *
 * @param image
 * @param image_size
 * @return
 */
bool EiCameraArduCam::ei_camera_capture_rgb888_packed_big_endian(uint8_t *image, uint32_t image_size)
{
    uint32_t captured_image_size;
#if MEASURE_TIME == 1
    uint32_t t_start = 0;
    uint32_t t_end = 0;

    t_start = ei_read_timer_ms();
#endif

    if (image_size < (this->height * this->width * 3)) {
        ei_printf("Buffer too small\n");
        return false;
    }

#if USE_SDRAM == 0
    ei_camera_frame_mem = (uint8_t *)ei_malloc(this->height * this->width * 2 + 32 /*alignment*/);

    if (ei_camera_frame_mem == nullptr) {
        printf("Error can't allocate image buffer");
        return false;
    }

    ei_camera_frame_buffer = (uint8_t *)ALIGN_PTR((uintptr_t)ei_camera_frame_mem, 32);
#endif
    memset(ei_camera_frame_buffer, 0, (this->height * this->width * 2));
    
    captured_image_size = this->cam->capture_frame(&ei_camera_frame_buffer[0]); // returns an error code (if any) 0 is OK. is not the size!

    if (captured_image_size != 0) {
        ei_printf("Error getting snapshot, error code 0x%x\n", captured_image_size);
#if USE_SDRAM == 0
        ei_free(ei_camera_frame_mem);
#endif
        return false;
    }
    captured_image_size = (this->height * this->width * 2); // everything is ok...


#if MEASURE_TIME == 1
    t_end = ei_read_timer_ms();
    ei_printf("capture took: %ld\n", (t_end-t_start));

    t_start = ei_read_timer_ms();
#endif

#if 1
    bool converted = RBG565ToRGB888(&ei_camera_frame_buffer[0], image, captured_image_size);
#else
    using namespace ei::image::processing;

    bool converted = false;

    if (yuv422_to_rgb888(&ei_camera_frame_buffer[0], (unsigned char*)image, captured_image_size, BIG_ENDIAN_ORDER) == ei::EIDSP_OK) {        
        converted = true;
    }
    else {

    }
#endif

    if (!converted) {
        ei_printf("ERR: Conversion failed\n");
#if USE_SDRAM == 0
        ei_free(ei_camera_frame_mem);
#endif
        return false;
    }

#if MEASURE_TIME == 1
    t_end = ei_read_timer_ms();
    ei_printf("conversion took: %ld\n", (t_end-t_start));
#endif
#if USE_SDRAM == 0
    ei_free(ei_camera_frame_mem);
#endif

    return true;
}

/**
 *
 * @param res
 * @param res_num
 */
void EiCameraArduCam::get_resolutions(ei_device_snapshot_resolutions_t **res, uint8_t *res_num)
{
    *res = &EiCameraArduCam::resolutions[0];
    *res_num = sizeof(EiCameraArduCam::resolutions) / sizeof(ei_device_snapshot_resolutions_t);
}

/**
 *
 * @param width_to_set
 * @param height_to_set
 * @return
 */
bool EiCameraArduCam::start_stream(uint32_t width_to_set, uint32_t height_to_set)
{
    // try to set required resolution, returned is what has been set
    ei_device_snapshot_resolutions_t sensor_res = this->search_resolution(width_to_set, height_to_set);

    if (is_lcd_stream_active()) { // if lcd stream is running need to stop it!
        xEventGroupSetBits(g_camera_event_group, CAMERA_STOP_LCD_INFERENCE);    // signal stop

        while(is_lcd_stream_active() == true) {
            ei_sleep(10);   // wait till finish
        }
    }

    if (set_resolution(sensor_res) == 0) {
        ei_printf("ERR: Failed to set camera resolution!\n");
        return false;
    }

    // store required output res
    this->output_width = width_to_set;
    this->output_height = height_to_set;

    // check if we have to do resize/crop
    this->stream_do_resize = this->width != width_to_set || this->height != height_to_set;

    // get bigger image resolution (snapshot or output) to allocate big enough buffer
    this->stream_buffer_size = std::max(this->width * this->height, this->output_width * this->output_height) * 3;
    this->stream_buffer = (uint8_t*)ei_malloc(stream_buffer_size);

    if (this->stream_buffer == nullptr) {
        ei_printf("ERR: Failed to allocate stream buffer!\n");
        return false;
    }

    this->stream_active = true;
    xEventGroupSetBits(g_camera_event_group, CAMERA_START_STREAM);    // signal start stream

    return true;
}

/**
 *
 * @return
 */
bool EiCameraArduCam::run_stream(void)
{
    if(stream_active == false) {
        return false;
    }

    if (ei_camera_capture_rgb888_packed_big_endian(this->stream_buffer, this->stream_buffer_size) == false) {
    	ei_printf("Err: ei_camera_capture_rgb888_packed_big_endian\r\n");
    	return false;
    }

    if (this->stream_do_resize) {
        // interpolate in place
        ei::image::processing::crop_and_interpolate_rgb888(
            this->stream_buffer,
            this->width,
            this->height,
            this->stream_buffer,
            this->output_width,
            this->output_height);
    }

    ei_camera_read_encode_send_sample_buffer((char*)this->stream_buffer, (this->output_height * this->output_width * 3));
    ei_printf("\r\n");

    return true;
}

/**
 * @brief 
 * 
 * @param width 
 * @param height 
 * @return true 
 * @return false 
 */
bool EiCameraArduCam::get_snapshot_and_send(uint16_t width_to_set, uint16_t height_to_set)
{
    uint8_t* image;
    uint32_t size = (width_to_set * height_to_set * 3);

    if (!init(width_to_set, height_to_set)) {
        ei_printf("Failed to init camera\n");
        return false;
    }

    image = (uint8_t*)ei_malloc(size);

    if (!image) {
        ei_printf("Take snapshot: Out of memory\n");
        return false;
    }

    bool isOK = this->ei_camera_capture_rgb888_packed_big_endian(image, size);
    if (!isOK) {
        ei_free(image);
        image = nullptr;
        return false;
    }

    ei_camera_read_encode_send_sample_buffer((char*)image, (size));
    ei_free(image);
    image = nullptr;

    return true;
}

/**
 *
 * @return
 */
bool EiCameraArduCam::is_stream_active(void)
{
    return stream_active;
}

/**
 *
 * @return
 */
bool EiCameraArduCam::stop_stream(void)
{
    auto dev = EiDeviceRenesasEKRA8D1::get_device();

    ei_free(this->stream_buffer);
    stream_buffer = nullptr;

    ei_printf("OK\r\n");
    // // we can call it even if the baudrate wasn't changed
    dev->set_default_data_output_baudrate();
    ei_sleep(100);
    ei_printf("Snapshot streaming stopped by user\n");
    ei_printf("OK\n");

    //ei_printf("> ");
    stream_active = false;

    return true;
}

/**
 *
 * @return
 */
bool EiCameraArduCam::is_lcd_stream_active(void)
{
    return lcd_stream_active;
}

/**
 *
 * @return
 */
bool EiCameraArduCam::start_lcd_stream(uint32_t width_to_set, uint32_t height_to_set)
{
    ei_device_snapshot_resolutions_t snapshot_resolution = this->search_resolution(width_to_set, height_to_set);

    if (set_resolution(snapshot_resolution) == 0) {
        ei_printf("ERR: Failed to set camera resolution!\n");
        return false;
    }

    this->output_width = width_to_set;
    this->output_height = height_to_set;

    // check if we have to do resize/crop
    this->stream_do_resize = ((this->width != width_to_set) || (this->height != height_to_set));

    this->stream_buffer_size = std::max(this->width * this->height, this->output_width * this->output_height) * 2;
    this->stream_buffer = (uint8_t*)ei_malloc(stream_buffer_size);

    if (this->stream_buffer == nullptr) {
        ei_printf("ERR: Failed to allocate stream buffer!\n");
        return false;
    }

    if (!init(width, height)) {
        ei_printf("Failed to init camera\n");
        return false;
    }

    lcd_stream_active = true;

    return true;
}

/*
 *
 */
void EiCameraArduCam::stop_lcd_stream(void)
{
    ei_free(stream_buffer);
    stream_buffer = nullptr;

    lcd_stream_active = false;
}

/**
 *
 */
bool EiCameraArduCam::lcd_stream(void)
{
    bool retval = true;
    uint32_t captured_image_size = 0;
    //
    if (stream_buffer != nullptr) {
        memset(stream_buffer, 0, (stream_buffer_size));
        captured_image_size = this->cam->capture_frame(&stream_buffer[0]); // returns an error code (if any) 0 is OK. is not the size!

        if (captured_image_size != 0) {
            retval = false;
            //ei_printf("Error getting lcd stream, error code 0x%x\n", captured_image_size);
        }
    }
    else {
        lcd_stream_active = false;
        ei_printf("ERR: stram_buffer has not been allocated\r\n");
        retval = false;
    }

    return retval;
}

/**
 *
 * @param src_buf
 * @param dst_buf
 * @param src_len
 * @return
 */
bool RBG565ToRGB888(uint8_t *src_buf, uint8_t *dst_buf, uint32_t src_len)
{
    uint8_t hb, lb;
    uint32_t pix_count = src_len / 2;

    for(uint32_t i = 0; i < pix_count; i ++) {
        hb = *src_buf++;
        lb = *src_buf++;

        *dst_buf++ = hb & 0xF8;
        *dst_buf++ = (hb & 0x07) << 5 | (lb & 0xE0) >> 3;
        *dst_buf++ = (lb & 0x1F) << 3;
    }

    return true;
}

/**
 *
 * @param input
 * @param length
 * @return
 */
bool ei_camera_read_encode_send_sample_buffer(const char *input, size_t length)
{
    uint32_t address = 0;
    // we are encoiding data into base64, so it needs to be divisible by 3
    //const int buffer_size = 16416;
    const int buffer_size = 513;
    uint8_t* buffer = (uint8_t*)input;

    size_t output_size_check = floor(buffer_size / 3 * 4);
    size_t mod = buffer_size % 3;
    output_size_check += mod;

    uint8_t* buffer_out = (uint8_t*)ei_malloc(output_size_check);

    while (1) {
        size_t bytes_to_read = buffer_size;

        if (bytes_to_read > length) {
            bytes_to_read = length;
        }

        if (bytes_to_read == 0) {
            ei_free(buffer_out);
            return true;
        }

        int to_send = base64_encode_buffer((char *)&buffer[address], bytes_to_read, (char *)buffer_out, output_size_check);
#if (USE_UART == 1)
        uart_print_to_console(buffer_out, to_send);
#else
        comms_send(buffer_out, to_send, 1000);
#endif

        address += bytes_to_read;
        length -= bytes_to_read;
    }

    return true;
}
