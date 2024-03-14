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

#include "arducam.h"
#include <cstdio>
#include "edge-impulse-sdk/porting/ei_classifier_porting.h"
#include "peripheral/i2c.h"
#include "peripheral/timer_handler.h"
#include "ov2640_regs.h"
#include "ov3640_regs.h"

ArduCAM::ArduCAM()
{
    format = e_format_rgb565;
    state = e_arducam_off;
    ceu_resolution = e_ceu_320_240;
    p_i2c_instance = nullptr;
    my_type = e_no_camera;
}

/**
 * @brief Peripheral initialization for the Camera
 */
void ArduCAM::hw_init(void)
{
    ei_timer3_start();  // XCLK for the camera

    R_BSP_PinAccessEnable();
    R_BSP_PinWrite((bsp_io_port_pin_t)CAM_PWDN, BSP_IO_LEVEL_LOW);  // PWDRN low
    R_BSP_PinAccessDisable();
}

/**
 * @brief Turn off camera
 */
void ArduCAM::DeinitCAM(void)
{
    ei_timer3_stop();  // XCLK for the camera

    R_BSP_PinAccessEnable();
    R_BSP_PinWrite((bsp_io_port_pin_t)CAM_PWDN, BSP_IO_LEVEL_HIGH);  // PWDRN low
    R_BSP_PinAccessDisable();
}

/**
 *
 * @return
 */
bool ArduCAM::detect_camera(void)
{
    bool found = false;
    my_type = e_no_camera;
    // check if ov2640
    p_i2c_instance = &g_comms_i2c_ov2640;
    p_i2c_instance->p_api->open(p_i2c_instance->p_ctrl, p_i2c_instance->p_cfg);
    if (ov2640_detect_camera() == true) {
        my_type = e_camera_ov2640;
        found = true;
    }
    else {
        p_i2c_instance->p_api->close(p_i2c_instance->p_ctrl);

        // check if ov3640
        p_i2c_instance = &g_comms_i2c_ov3640;
        p_i2c_instance->p_api->open(p_i2c_instance->p_ctrl, p_i2c_instance->p_cfg);
        if (ov3640_detect_camera() == true) {
            my_type = e_camera_ov3640;
            found = true;
        }
        else {
            p_i2c_instance->p_api->close(p_i2c_instance->p_ctrl);
            p_i2c_instance = nullptr;
        }
    }


    return found;
}

/**
 *
 */
void ArduCAM::init_config(void)
{
    if (e_camera_ov2640 == my_type) {
        this->ov2640_init_config();
    }
    else if (e_camera_ov3640 == my_type){
        this->ov3640_init_config();
    }
    else {
        // ...
    }
}

/**
 *
 * @param format
 */
void ArduCAM::set_format(t_image_format format)
{
    if (e_camera_ov2640 == my_type) {
        this->ov2640_set_format(format);
    }
    else if (e_camera_ov3640 == my_type){
        this->ov3640_set_format(format);
    }
    else {
        // ...
    }
}

/**
 *
 * @param size
 */
void ArduCAM::set_size(uint8_t size)
{
    if (e_camera_ov2640 == my_type) {
        this->ov2640_set_size(size);
    }
    else if (e_camera_ov3640 == my_type){
        this->ov3640_set_size(size);
    }
    else {
        // ...
    }
}

/**
 *
 * @param flip_image
 */
void ArduCAM::flip(bool flip_image)
{
    if (e_camera_ov2640 == my_type) {
        this->ov2640_flip(flip_image);
    }
    else if (e_camera_ov3640 == my_type){
        this->ov3640_flip(flip_image);
    }
    else {
        // ...
    }
}

/**
 *
 * @param buf
 * @return
 */
uint32_t ArduCAM::capture_frame(uint8_t *buf)
{
    int retval = 0;

    if ((buf != nullptr) || (ceu_resolution < e_ceu_res_max)) {
        //retval = ceu_start_capture(ceu_resolution);
        retval = ceu_capture(buf);
    }
    
    return retval;
}

/**
 * @brief Write 8 bit values to 16 bit register address
 * @param reglist
 * @return
 */
int ArduCAM::wrSensorRegs8_8(const struct sensor_reg reglist[], uint16_t size)
{
    int err = 0;
    //int written = 0;

    for (uint16_t i = 0; i < size; i++) {
        err += wrSensorReg8_8(reglist[i].reg, reglist[i].val);
    }

    return err;
}

/**
 * @brief Read/write 8 bit value to/from 16 bit register address
 * @param regID
 * @param regDat
 * @return
 */
int ArduCAM::wrSensorReg8_8(uint8_t regID, uint8_t regDat)
{
    uint8_t tx_buffer[2];
    tx_buffer[0] = (regID);
    tx_buffer[1] = regDat;

    return ei_i2c_write(p_i2c_instance->p_ctrl, tx_buffer, 2u);
}

/**
 * @brief Read/write 8 bit value to/from 16 bit register address
 * @param regID
 * @param regDat
 * @return
 */
int ArduCAM::rdSensorReg8_8(uint8_t regID, uint8_t *regDat)
{
    return ei_i2c_read_byte_command(p_i2c_instance->p_ctrl, regID, regDat, 1u);
}

/**
 * @brief Write 8 bit values to 16 bit register address
 * @param reglist
 * @return
 */
int ArduCAM::wrSensorRegs16_8(const struct sensor_reg_wA reglist[])
{
    int err = 0;
    uint16_t reg_addr = 0;
    uint8_t reg_val = 0;
    const struct sensor_reg_wA *next = reglist;

    while ((reg_addr != 0xffff) || (reg_val != 0xff)) {
        reg_addr = next->reg;
        reg_val = next->val;
        err += wrSensorReg16_8(reg_addr, reg_val);
        next++;
    }

    return err;
}

/**
 * @brief Read/write 8 bit value to/from 16 bit register address
 * @param regID
 * @param regDat
 * @return
 */
int ArduCAM::wrSensorReg16_8(uint16_t regID, uint8_t regDat)
{
    uint8_t tx_buffer[3];
    tx_buffer[0] = (uint8_t)(regID >> 8);
    tx_buffer[1] = (uint8_t)(regID & 0x00ff);
    tx_buffer[2] = regDat;

    ei_i2c_write(p_i2c_instance->p_ctrl, tx_buffer, 3u);

    return 0;
}

/**
 * @brief I2C Read 16bit address, 16bit data
 * @param regID
 * @param regDat
 * @return
 */
int ArduCAM::rdSensorReg16_8(uint16_t regID, uint8_t *regDat)
{
    return ei_i2c_read_word_command(p_i2c_instance->p_ctrl, regID, regDat, 1u);
}

/**
 *
 * @return
 */
bool ArduCAM::ov2640_detect_camera(void)
{
    uint8_t chip_id_h = 0xFF;
    uint8_t chip_id_l = 0xFF;

    wrSensorReg8_8(0xFF, 0x01);
    wrSensorReg8_8(0x12, 0x80); // reset
    ei_sleep(50);

    wrSensorReg8_8(0xFF, 1);
    rdSensorReg8_8(OV2640_CHIPID_HIGH, &chip_id_h);
    rdSensorReg8_8(OV2640_CHIPID_LOW, &chip_id_l);

    /* check if chip id is correct */
    if ((chip_id_h != 0x26) || (chip_id_l != 0x42)) {

        return false;
    }
    else {

    }

    return true;
}

/**
 * @brief Init camera with standard config
 */
void ArduCAM::ov2640_init_config(void)
{
    uint16_t len = sizeof(OV2640_INIT)/sizeof(sensor_reg);
    //uint16_t len = sizeof(OV2640_QVGA)/sizeof(sensor_reg);

    wrSensorReg8_8(0xFF, 0x01);
    wrSensorReg8_8(0x12, 0x80);
    ei_sleep(100);

    wrSensorRegs8_8(OV2640_INIT, len);
    //wrSensorRegs8_8(OV2640_QVGA, len);

    // LIGHT
    wrSensorReg8_8(0xff, 0x00);
    wrSensorReg8_8(0xc7, 0x10); // simple AWB ON
    // wrSensorReg8_8(0xc7, 0x00); // advanced AWB ON - quite bad with low resolution

    // SATURATION
#if 0
    wrSensorReg8_8(0xff, 0x00);
    wrSensorReg8_8(0x7c, 0x00);
    wrSensorReg8_8(0x7d, 0x02);
    wrSensorReg8_8(0x7c, 0x03);
    wrSensorReg8_8(0x7d, 0x48);
    wrSensorReg8_8(0x7d, 0x48);
#endif

    // BIGHTNESS
#if 1
    wrSensorReg8_8(0xff, 0x00);
    wrSensorReg8_8(0x7c, 0x00);
    wrSensorReg8_8(0x7d, 0x04);
    wrSensorReg8_8(0x7c, 0x09);
    wrSensorReg8_8(0x7d, 0x20);
    wrSensorReg8_8(0x7d, 0x00);
#endif

    // Contrast
#if 0
    wrSensorReg8_8(0xff, 0x00);
    wrSensorReg8_8(0x7c, 0x00);
    wrSensorReg8_8(0x7d, 0x04);
    wrSensorReg8_8(0x7c, 0x07);
    wrSensorReg8_8(0x7d, 0x20);
    wrSensorReg8_8(0x7d, 0x20);
    wrSensorReg8_8(0x7d, 0x20);
    wrSensorReg8_8(0x7d, 0x06);
#endif

    // Special Effects
#if 1
    wrSensorReg8_8(0xff, 0x00);
    wrSensorReg8_8(0x7c, 0x00);
    wrSensorReg8_8(0x7d, 0x00);
    wrSensorReg8_8(0x7c, 0x05);
    wrSensorReg8_8(0x7d, 0x80);
    wrSensorReg8_8(0x7d, 0x80);
#endif

    // preview (?)
#if 0
    wrSensorReg8_8(0xff, 0x01);
    wrSensorReg8_8(OV2640_SENSOR_CLKRC, 0x00);
    wrSensorReg8_8(OV2640_SENSOR_COM7, 0x40);
    wrSensorReg8_8(0x2A, 0x00);
    wrSensorReg8_8(0x2B, 0x00);
    wrSensorReg8_8(OV2640_SENSOR_FLL, 0x00);
    wrSensorReg8_8(OV2640_SENSOR_FLH, 0x00);
    wrSensorReg8_8(0x3D, 0x38);
#endif
}

/**
 *
 * @param format_to_set
 */
void ArduCAM::ov2640_set_format(t_image_format format_to_set)
{
    switch(format_to_set)
    {
        case e_format_rgb565:
        {
            wrSensorRegs8_8(OV2640_FORMAT_RGB565, sizeof(OV2640_FORMAT_RGB565)/sizeof(sensor_reg));
        }
        break;
        case e_format_rgb555:
        {

        }
        break;
        case e_format_jpg:
        {
            wrSensorRegs8_8(OV2640_FORMAT_JPG, sizeof(OV2640_FORMAT_JPG)/sizeof(sensor_reg));
        }
        break;
        case e_format_raw:  // processed raw
        {
            wrSensorRegs8_8(OV2640_FORMAT_RAW, sizeof(OV2640_FORMAT_RAW)/sizeof(sensor_reg));
        }
        break;
        case e_format_yuv422:
        {
            wrSensorRegs8_8(OV2640_FORMAT_YUV422, sizeof(OV2640_FORMAT_YUV422)/sizeof(sensor_reg));
        }
        break;
        case e_format_max:  // force to jpg
        default:
        {

            format_to_set = e_format_jpg;
        }
        break;
    }

    format = format_to_set;
}

/**
 *
 * @param size
 */
void ArduCAM::ov2640_set_size(uint8_t size)
{
    ceu_stop_capture();
    switch(size)
    {
        case e_ceu_96_64:
        {
            ov2640_set_manual_size(96, 64);
            //wrSensorRegs8_8(OV2640_160x120_, sizeof(OV2640_160x120_)/sizeof(sensor_reg));
        }
        break;
        case e_ceu_96_96:
        {
            ov2640_set_manual_size(96, 96);
            //wrSensorRegs8_8(OV2640_160x120_, sizeof(OV2640_160x120_)/sizeof(sensor_reg));
        }
        break;
        case e_ceu_160_120:
        {
            ov2640_set_manual_size(160, 120);
            //wrSensorRegs8_8(OV2640_160x120_, sizeof(OV2640_160x120_)/sizeof(sensor_reg));        
        }
        break;
        case e_ceu_176_144:
        {
            ov2640_set_manual_size(176, 144);
            //wrSensorRegs8_8(OV2640_176x144_, sizeof(OV2640_176x144_)/sizeof(sensor_reg));
        }
        break;
        case e_ceu_320_240:
        {
            ov2640_set_manual_size(320, 240);
            //wrSensorRegs8_8(OV2640_320x240_, sizeof(OV2640_320x240_)/sizeof(sensor_reg));
        }
        break;
        default:
        {
            ov2640_set_manual_size(320, 240);
            //wrSensorRegs8_8(OV2640_640x480_);
            size = e_ceu_320_240;
            break;
        }

    }
    ceu_resolution = (t_ceu_supported_resolution)size;
    ceu_start_capture(ceu_resolution);
}

typedef struct {
        uint16_t offset_x;
        uint16_t offset_y;
        uint16_t max_x;
        uint16_t max_y;
} ov2640_ratio_settings_t;

static const ov2640_ratio_settings_t ratio_table[] = {
    // ox,  oy,   mx,   my
    {   0,   0, 1600, 1200 }, //4x3
    {   8,  72, 1584, 1056 }, //3x2
    {   0, 100, 1600, 1000 }, //16x10
    {   0, 120, 1600,  960 }, //5x3
    {   0, 150, 1600,  900 }, //16x9
    {   2, 258, 1596,  684 }, //21x9
    {  50,   0, 1500, 1200 }, //5x4
    { 200,   0, 1200, 1200 }, //1x1
    { 462,   0,  676, 1200 }  //9x16
};

/**
 *
 * @param width
 * @param height
 * @param index
 * @return
 */
bool ArduCAM::ov2640_set_manual_size(uint16_t width, uint16_t height, uint8_t index)
{
    uint16_t max_x = ratio_table[index].max_x/4;
    uint16_t max_y = ratio_table[index].max_y/4;
    uint16_t offset_x = ratio_table[index].offset_x/4;
    uint16_t offset_y = ratio_table[index].offset_y/4;
    
    wrSensorReg8_8(OV2640_BANK_SEL, OV2640_BANK_SEL_DSP);
    wrSensorReg8_8(OV2640_DSP_R_BYPASS, 0x01);
    wrSensorRegs8_8(OV2640_FORMAT_CIF, sizeof(OV2640_FORMAT_CIF)/sizeof(sensor_reg));

    // 
    wrSensorReg8_8(OV2640_BANK_SEL, OV2640_BANK_SEL_DSP);
    wrSensorReg8_8(OV2640_DSP_HSIZE, max_x & 0xFF);
    wrSensorReg8_8(OV2640_DSP_VSIZE, max_y & 0xFF);
    wrSensorReg8_8(OV2640_DSP_XOFFL, offset_x & 0xFF);
    wrSensorReg8_8(OV2640_DSP_YOFFL, offset_y & 0xFF);
    wrSensorReg8_8(OV2640_DSP_VHYX, ((max_y >> 1) & 0X80) | ((offset_y >> 4) & 0X70) | ((max_x >> 5) & 0X08) | ((offset_x >> 8) & 0X07));
    wrSensorReg8_8(OV2640_DSP_TEST, (max_x >> 2) & 0X80);
    wrSensorReg8_8(OV2640_DSP_ZMOW, width & 0xFF);
    wrSensorReg8_8(OV2640_DSP_ZMOH, height & 0xFF);
    wrSensorReg8_8(OV2640_DSP_ZMHH, ((height>>6)&0x04)|((width>>8)&0x03));

    // clock settings
    //wrSensorReg8_8(OV2640_BANK_SEL, OV2640_BANK_SEL_SENSOR);
    //wrSensorReg8_8(OV2640_SENSOR_CLKRC, 0x00);
    //wrSensorReg8_8(OV2640_BANK_SEL, OV2640_BANK_SEL_DSP);
    //wrSensorReg8_8(OV2640_DSP_R_DVP_SP, 0x01);

    wrSensorReg8_8(OV2640_DSP_R_BYPASS, 0x00);

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
bool ArduCAM::ov2640_set_manual_size(uint16_t width, uint16_t height)
{
    uint16_t outh = 0;
    uint16_t outw = 0;
    uint8_t temp = 0;

    if (width % 4) {
        return false;
    }

    if (height % 4) { 
        return false;
    }

    outw = (width >> 2);
    outh = (height >> 2);
    wrSensorReg8_8(0xFF, 0x00); // DSP
    wrSensorReg8_8(OV2640_DSP_RESET, 0x04);
    wrSensorReg8_8 (OV2640_DSP_ZMOW, (uint8_t)(outw & 0xFF));
    wrSensorReg8_8 (OV2640_DSP_ZMOH, (uint8_t)(outh & 0xFF));
    temp = ((outw>>8) & 0x03);
    temp |= ((outh>>6) & 0x04);
    wrSensorReg8_8(OV2640_DSP_ZMHH, temp);
    wrSensorReg8_8(OV2640_DSP_RESET, 0x00);

    return true;
}

/**
 * @brief Put camera to sleep
 */
void ArduCAM::ov2640_sleep(void)
{
    uint8_t actual_val = 0xFF;
    /*
    TODO
    */
}

/**
 * @brief wake up camera!
 */
void ArduCAM::ov2640_wakeup(void)
{
    uint8_t actual_val = 0xFF;
    /*
    TODO
    */
}

/**
 *
 * @param flip_image
 */
void ArduCAM::ov2640_flip(bool flip_image)
{
    if (flip_image == true) {
        wrSensorReg8_8(0xFF, 0x01);
        //wrSensorReg8_8(OV2640_SENSOR_REG04, 0x68);
    }
    else {
        wrSensorReg8_8(0xFF, 0x01);
        //wrSensorReg8_8(OV2640_SENSOR_REG04, 0x28);
    }
}

/**
 *
 * @return
 */
bool ArduCAM::ov3640_detect_camera(void)
{
    uint8_t vid = 0xFF;
    uint8_t pid = 0xFF;
    uint8_t sscb_id = 0xFF;
    int retval = 0xFF;

    /* TODO check retval */
    retval = rdSensorReg16_8(PIDH, &vid);
    retval = rdSensorReg16_8(PIDL, &pid);
    retval = rdSensorReg16_8(SCCB_ID, &sscb_id);

    if ((vid != EXPECTED_PIDH) || (pid != EXPECTED_PIDL) || (sscb_id != EXPECTED_SSCB_ID)) {

        return false;
    }
    else {

    }

    return true;
}

/**
 * @brief Init camera with standard config
 */
void ArduCAM::ov3640_init_config(void)
{
    wrSensorReg16_8(0x3012, 0x80);
    ei_sleep(50);
    //wrSensorRegs16_8(OV3640_VGA);
    //wrSensorRegs16_8(rowboat_init_xga); // common init

    //wrSensorRegs16_8(xga_565);  // format

    wrSensorReg16_8(0x3012, 0x80);  // another reset ...
    ei_sleep(50);    
    //wrSensorRegs16_8(ov3640_fmt_yuv422_vga);
    wrSensorRegs16_8(ov3640_fmt_yuv422_qvga);

    wrSensorRegs16_8(ov3640_vga);
        
    // light mode
    //wrSensorReg16_8(0x332b, 0x00);//AWB auto, bit[3]:0,auto

    // Saturation0
    wrSensorReg16_8(0x3302, 0xef);
    wrSensorReg16_8(0x3355, 0x02);
    wrSensorReg16_8(0x3358, 0x40);
    wrSensorReg16_8(0x3359, 0x40);

    // brightness0
    wrSensorReg16_8(0x3302, 0xef);
    wrSensorReg16_8(0x3355, 0x04);
    wrSensorReg16_8(0x3354, 0x01);
    wrSensorReg16_8(0x335e, 0x00);

    // Contrast
    wrSensorReg16_8(0x3302, 0xef);
    wrSensorReg16_8(0x3355, 0x04);
    wrSensorReg16_8(0x3354, 0x01);
    wrSensorReg16_8(0x335c, 0x20);
    wrSensorReg16_8(0x335d, 0x20);

    // Special effect: normal
    //wrSensorReg16_8(0x3302, 0xef);
    //wrSensorReg16_8(0x3355, 0x00);

    // exposure: default
    wrSensorReg16_8(0x3018, 0x38);
    wrSensorReg16_8(0x3019, 0x30);
    wrSensorReg16_8(0x301a, 0x61);

    // sharpness: auto
    wrSensorReg16_8(0x332d, 0x60);
    wrSensorReg16_8(0x332f, 0x03);
}

/**
 *
 * @param format_to_set
 */
void ArduCAM::ov3640_set_format(t_image_format format_to_set)
{
    switch(format_to_set)
    {
        case e_format_rgb565:
        {
            wrSensorRegs16_8(arducam_rgb565);            
        }
        break;
        case e_format_rgb555:
        {
            wrSensorReg16_8(0x3100, 0x02);
		    wrSensorReg16_8(0x3301, 0xDE);
            wrSensorReg16_8(0x3304, 0x00);
            wrSensorReg16_8(0x3400, 0x01);
		    wrSensorReg16_8(0x3404, 0x13);
            wrSensorReg16_8(0x3600, 0xC4);
        }
        break;

        case e_format_jpg:
        {
            wrSensorReg16_8(0x3100, 0x32); 
		    wrSensorReg16_8(0x3304, 0x00);
            wrSensorReg16_8(0x3400, 0x02);      // YUV 422
		    wrSensorReg16_8(0x3404, 0x22);      // sure ?
            wrSensorReg16_8(0x3500, 0x00); 
		    wrSensorReg16_8(0x3600, 0x42);
            wrSensorReg16_8(0x3610, 0x00);      // WIDTH_CTRL
		    wrSensorReg16_8(0x3611, 0x20);      // OUT_CTRL
            wrSensorReg16_8(0x3507, 0x04); 
		    wrSensorReg16_8(0x350A, 0x4f);
            wrSensorReg16_8(0x304C, 0x82); 
        }
        break;
        case e_format_raw:  // processed raw
        {
            wrSensorReg16_8(0x3100, 0x02);
		    wrSensorReg16_8(0x3301, 0xDE);
            wrSensorReg16_8(0x3304, 0x00);
            wrSensorReg16_8(0x3400, 0x01); 
		    wrSensorReg16_8(0x3404, 0x08);  // not sure, shopuld be 0x18 ?
            wrSensorReg16_8(0x3600, 0xC4);
        }
        break;
        case e_format_yuv422:
        {
            wrSensorReg16_8(0x3100, 0x02);
		    wrSensorReg16_8(0x3301, 0xDE);
            wrSensorReg16_8(0x3304, 0x00);            
            wrSensorReg16_8(0x3400, 0x00);
		    wrSensorReg16_8(0x3404, 0x00);  // not sure, should be 0x01 ? ->no, see AN pag 23, 0x00==> Y U Y V
            wrSensorReg16_8(0x3600, 0xC4);            
        }
        break;
        case e_format_max:  // force to jpg
        default:
        {
            wrSensorReg16_8(0x3100, 0x32); 
		    wrSensorReg16_8(0x3304, 0x00);
            wrSensorReg16_8(0x3400, 0x02);
		    wrSensorReg16_8(0x3404, 0x22);
            wrSensorReg16_8(0x3500, 0x00); 
		    wrSensorReg16_8(0x3600, 0x42);
            wrSensorReg16_8(0x3610, 0x00); 
		    wrSensorReg16_8(0x3611, 0x20);
            wrSensorReg16_8(0x3507, 0x04); 
		    wrSensorReg16_8(0x350A, 0x4f);
            wrSensorReg16_8(0x304C, 0x82); 
            format_to_set = e_format_jpg;
        }
        break;
    }

    format = format_to_set;
}

/**
 *
 * @param size
 */
void ArduCAM::ov3640_set_size(uint8_t size)
{
    ceu_stop_capture();
    switch(size)
    {
        case e_ceu_96_64:
        {
            wrSensorRegs16_8(resize_ov3640_96_64);
        }
        break;        
        case e_ceu_96_96:
        {
            wrSensorRegs16_8(resize_ov3640_96_96);
        }
        break;
        case e_ceu_160_120:
        {
            wrSensorRegs16_8(resize_ov3640_160_120);
        }
        break;
        case e_ceu_176_144:
        {
            wrSensorRegs16_8(resize_ov3640_176_144);
        }
        break;
        case e_ceu_320_240:
        {
            wrSensorRegs16_8(resize_ov3640_320_240);
        }
        break;
        default:
            wrSensorRegs16_8(resize_ov3640_320_240);
            size = e_ceu_320_240;
            break;
    }
    ceu_resolution = (t_ceu_supported_resolution)size;
    ceu_start_capture(ceu_resolution);
}

/**
 * @brief Put camera to sleep
 */
void ArduCAM::ov3640_sleep(void)
{
    uint8_t actual_val = 0xFF;

    rdSensorReg16_8(0x3086, &actual_val);
    actual_val |= 0x03;
    wrSensorReg16_8(0x3086, actual_val);
}

/**
 * @brief wake up camera!
 */
void ArduCAM::ov3640_wakeup(void)
{
    uint8_t actual_val = 0xFF;

    rdSensorReg16_8(0x3086, &actual_val);
    actual_val &= ~0x03;
    wrSensorReg16_8(0x3086, actual_val);
}

/**
 *
 * @param flip_image
 */
void ArduCAM::ov3640_flip(bool flip_image)
{
    if (flip_image == true) {
        wrSensorReg16_8(OV3640_TMC6, 0x11); // flip
        //wrSensorReg16_8(OV3640_VS_L, 0x09); // ??
        wrSensorReg16_8(0x3090, 0xC0);
    }
    else {
        wrSensorReg16_8(OV3640_TMC6, 0x10); // no mirror/flip
        wrSensorReg16_8(0x3090, 0xC0);
        //wrSensorReg16_8(OV3640_VS_L, 0x0A);
    }
}

#define TOT_SIZE_H
#define TOT_SIZE_V

void ArduCAM::ov3640_center_image(uint16_t width, uint16_t height)
{
    uint16_t new_start_h;
    uint16_t new_start_v;

    new_start_h = (TOT_SIZE_H - 1024) >> 2;
    new_start_v = (TOT_SIZE_V - 768) >> 2;

    wrSensorReg16_8(OV3640_HS_H, (uint8_t)(new_start_h & 0xFF00) >> 8);
    wrSensorReg16_8(OV3640_HS_L, (uint8_t)(new_start_h & 0x00FF));

    wrSensorReg16_8(OV3640_VS_H, (uint8_t)(new_start_v & 0xFF00) >> 8);
    wrSensorReg16_8(OV3640_VS_L, (uint8_t)(new_start_v & 0x00FF));

}
