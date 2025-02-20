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
#ifndef OV3640_REGS_H
#define OV3640_REGS_H

#include "arducam.h"
#include "ov3640_reg_addr.h"

/* Register address */
const uint16_t PIDH = 0x300a;
const uint16_t PIDL = 0x300b;
const uint16_t SCCB_ID= 0x300c;

/* Constant expected values */
const uint8_t EXPECTED_PIDH     = 0x36;
const uint8_t EXPECTED_PIDL     = 0x4c;
const uint8_t EXPECTED_SSCB_ID  = 0x78;

// init from differente sources
const  sensor_reg_wA AN_OV3640_VGA[] =
{
	{0x304d, 0x45},
	{0x30a7, 0x5e},
	{0x3087, 0x16},
	{0x309c, 0x1a},
	{0x30a2, 0xe4},
	{0x30aa, 0x42},
	{0x30b0, 0xff},
	{0x30b1, 0xff},
	{0x30b2, 0x10},
	{0x300e, 0x32},
	{0x300f, 0x21},
	{0x3010, 0x20},
	{0x3011, 0x04},
	{0x304c, 0x81},
	{0x30d7, 0x10},
	{0x30d9, 0x0d},
	{0x30db, 0x08},
	{0x3016, 0x82},
	{0x3018, 0x38},
	{0x3019, 0x30},
	{0x301a, 0x61},
	{0x307d, 0x00},
	{0x3087, 0x02},
	{0x3082, 0x20},
	{0x3015, 0x12}, // 8x ??
	{0x3014, 0x04},
	{0x3013, 0xf7},
	{0x303c, 0x08},
	{0x303d, 0x18},
	{0x303e, 0x06},
	{0x303f, 0x0c},
	{0x3030, 0x62},
	{0x3031, 0x26},
	{0x3032, 0xe6},
	{0x3033, 0x6e},
	{0x3034, 0xea},
	{0x3035, 0xae},
	{0x3036, 0xa6},
	{0x3037, 0x6a},
	{0x3104, 0x02},
	{0x3105, 0xfd},
	{0x3106, 0x00},
	{0x3107, 0xff},
	{0x3300, 0x12},
	{0x3301, 0xde},
	{OV3640_DSP_CTRL_2, 0xef},
	{0x3316, 0xff},
	{0x3317, 0x00},
	{0x3312, 0x26},
	{0x3314, 0x42},
	{0x3313, 0x2b},
	{0x3315, 0x42},
	{0x3310, 0xd0},
	{0x3311, 0xbd},
	{0x330c, 0x18},
	{0x330d, 0x18},
	{0x330e, 0x56},
	{0x330f, 0x5c},
	{0x330b, 0x1c},
	{0x3306, 0x5c},
	{0x3307, 0x11},
	{0x336a, 0x52},
	{0x3370, 0x46},
	{0x3376, 0x38},
	{0x3300, 0x13},
	{0x30b8, 0x20},
	{0x30b9, 0x17},
	{0x30ba, 0x04},
	{0x30bb, 0x08},
	{0x3507, 0x06},
	{0x350a, 0x4f},
	{0x3100, 0x02},
	{0x3301, 0xde},
	{0x3304, 0x00},
	{0x3400, 0x00},
	{0x3404, 0x00},
	{OV3640_SIZE_IN_MISC, 0x68},
	{OV3640_HSIZE_IN_L, 0x18},
	{OV3640_VSIZE_IN_L, 0x0c},
	{OV3640_SIZE_OUT_MISC, 0x12},
	{OV3640_HSIZE_OUT_L, 0x88},
	{OV3640_VSIZE_OUT_L, 0xe4},
	{OV3640_ISP_PAD_CTR2, 0x42},
	{OV3640_ISP_XOUT_H, 0x02},
	{OV3640_ISP_XOUT_L, 0x80},
	{OV3640_ISP_YOUT_H, 0x01},
	{OV3640_ISP_YOUT_L, 0xe0},
	{0x308d, 0x04},
	{0x3086, 0x03},
	{0x3086, 0x00},
	{0x3011, 0x00},
	{0x304c, 0x85},
	{0xFFFF, 0xFF}
};

//
const  sensor_reg_wA AN_OV3640_QVGA[] =
{
	{0x304d, 0x45}, 
	{0x30a7, 0x5e}, 
	{0x3087, 0x16}, 
	{0x309c, 0x1a}, 
	{0x30a2, 0xe4}, 
	{0x30aa, 0x42}, 
	{0x30b0, 0xff}, 
	{0x30b1, 0xff}, 
	{0x30b2, 0x10}, 
	{0x300e, 0x32}, 
	{0x300f, 0x21}, 
	{0x3010, 0x20}, 
	{0x3011, 0x04}, 
	{0x304c, 0x81}, 
	{0x30d7, 0x10}, 
	{0x30d9, 0x0d}, 
	{0x30db, 0x08}, 
	{0x3016, 0x82}, 
	{0x3018, 0x58}, 
	{0x3019, 0x59}, 
	{0x301a, 0x61}, 
	{0x307d, 0x00}, 
	{0x3087, 0x02}, 
	{0x3082, 0x20}, 
	{0x3015, 0x12}, 
	{0x3014, 0x84}, 
	{0x3013, 0xf7}, 
	{0x303c, 0x08}, 
	{0x303d, 0x18}, 

	{0x303e, 0x06}, 
	{0x303f, 0x0c}, 
	{0x3030, 0x62}, 
	{0x3031, 0x26}, 
	{0x3032, 0xe6}, 
	{0x3033, 0x6e}, 
	{0x3034, 0xea}, 
	{0x3035, 0xae}, 
	{0x3036, 0xa6}, 
	{0x3037, 0x6a}, 
	{0x3104, 0x02}, 
	{0x3105, 0xfd}, 
	{0x3106, 0x00}, 
	{0x3107, 0xff}, 
	{0x3300, 0x12}, 
	{0x3301, 0xde}, 
	{OV3640_DSP_CTRL_2, 0xef}, 
	{0x3316, 0xff}, 
	{0x3317, 0x00}, 
	{0x3312, 0x26}, 
	{0x3314, 0x42}, 
	{0x3313, 0x2b}, 
	{0x3315, 0x42}, 
	{0x3310, 0xd0}, 
	{0x3311, 0xbd}, 
	{0x330c, 0x18}, 
	{0x330d, 0x18}, 
	{0x330e, 0x56}, 
	{0x330f, 0x5c}, 
	{0x330b, 0x1c}, 
	{0x3306, 0x5c}, 
	{0x3307, 0x11}, 
	{0x336a, 0x52}, 
	{0x3370, 0x46}, 
	{0x3376, 0x38}, 
	{0x3300, 0x13}, 
	{0x30b8, 0x20}, 
	{0x30b9, 0x17}, 
	{0x30ba, 0x4}, 
	{0x30bb, 0x8}, 

	{0x3507, 0x06}, 
	{0x350a, 0x4f}, 
	{0x3100, 0x02}, 
	{0x3301, 0xde}, 
	{0x3304, 0x00}, 
	{0x3400, 0x01}, 
	{0x3404, 0x11}, 
	{OV3640_SIZE_IN_MISC, 0x68}, 
	{OV3640_HSIZE_IN_L, 0x18}, 
	{OV3640_VSIZE_IN_L, 0xc}, 
	{OV3640_SIZE_OUT_MISC, 0x12}, 
	{OV3640_HSIZE_OUT_L, 0x88}, 
	{OV3640_VSIZE_OUT_L, 0xe4}, 
	{OV3640_ISP_PAD_CTR2, 0x42}, 

	{OV3640_ISP_XOUT_H, 0x2}, 
	{OV3640_ISP_XOUT_L, 0x80}, 
	{OV3640_ISP_YOUT_H, 0x1}, 
	{OV3640_ISP_YOUT_L, 0xe0},

	{0x308d, 0x4}, 
	{0x3086, 0x3}, 
	{0x3086, 0x0}, 
	{0x3011, 0x0}, 
	{0x304c, 0x85}, 
	{0x3600, 0xd0}, 
	{OV3640_SIZE_IN_MISC, 0x68}, 
	{OV3640_HSIZE_IN_L, 0x18}, 
	{OV3640_VSIZE_IN_L, 0xc}, 
	{OV3640_SIZE_OUT_MISC, 0x1}, 
	{OV3640_HSIZE_OUT_L, 0x48}, 
	{OV3640_VSIZE_OUT_L, 0xf4}, 
	{OV3640_ISP_PAD_CTR2, 0x42}, 
	{OV3640_ISP_XOUT_H, 0x1}, 
	{OV3640_ISP_XOUT_L, 0x40}, 
	{OV3640_ISP_YOUT_H, 0x0}, 
	{OV3640_ISP_YOUT_L, 0xf0}, 
	
	{0x307c, 0x12},
	{0x3090, 0xc8},
	
	//{0x3080, 0x02},
	//{0x307D, 0x80},
	//{0x306C, 0x10},
	
	{0x3600, 0xc4},
	{0xffff,0xff},
};

// Application note
// pag 27
// QXGA -> QVGA. but it scales from QXGA ?
const  sensor_reg_wA from_AN_QVGA_preview[] =
{
    {OV3640_SIZE_IN_MISC, 0x68},
    {OV3640_HSIZE_IN_L,		0x18},
    {OV3640_VSIZE_IN_L,	0x0c},
    {OV3640_SIZE_OUT_MISC, 0x01},
    {OV3640_HSIZE_OUT_L, 0x48},
    {OV3640_VSIZE_OUT_L, 0xf4},
    {OV3640_ISP_PAD_CTR2, 0x42},
    {OV3640_ISP_XOUT_H, 0x01},
    {OV3640_ISP_XOUT_L, 0x40},
    {OV3640_ISP_YOUT_H, 0x00},
    {OV3640_ISP_YOUT_L, 0xf0},
};

// Application note
// pag 30
// QXGA-> QCIF 174x144
const  sensor_reg_wA from_AN_QVGA_capture[] =
{
    {OV3640_DSP_CTRL_2, 0xef},
    {OV3640_SIZE_IN_MISC, 0x68},
    {OV3640_HSIZE_IN_L, 0x18},
    {OV3640_VSIZE_IN_L, 0x0c},
    {OV3640_SIZE_OUT_MISC, 0x00},
    {OV3640_HSIZE_OUT_L, 0xb8},
    {OV3640_VSIZE_OUT_L, 0x94},
    {OV3640_ISP_PAD_CTR2, 0x42},
    {OV3640_ISP_XOUT_H, 0x00},
    {OV3640_ISP_XOUT_L, 0xb0},
    {OV3640_ISP_YOUT_H, 0x00},
    {OV3640_ISP_YOUT_L, 0x90},
    {0x304c, 0x84},   //14Mhz PCLK output
};

// from arducam code
const static sensor_reg_wA arducam_rgb565[] = {
    {0x3100, 0x02},
    {0x3301, 0xDE},
    {0x3304, 0x00}, // lens correction, needed ?
    {0x3400, 0x01},  // 0x01 ok
    {0x3404, 0x11},  // 0x30 vs 0x11, but 0x11 is RGB. 0x30 is GBR
    {0x3600, 0xC4},
    {0xFFFF, 0xff}
};

const static  sensor_reg_wA resize_ov3640_96_64[] = {
	    // ISP scaling input control
	    {OV3640_SIZE_IN_MISC,		0x34},  //  52 or 104 ?
	    {OV3640_HSIZE_IN_L,		0x4},  //  174 -> 12
	    {OV3640_VSIZE_IN_L,		0x4},  //  4 (?)

	    // ISP output
	    {OV3640_SIZE_OUT_MISC,		0x00},  //  0
	    {OV3640_HSIZE_OUT_L,		0x68},  //  104
	    {OV3640_VSIZE_OUT_L,		0x44},  //  68

	    {OV3640_ISP_PAD_CTR2,   0x42},  //  66

	    // ISP out
	    {OV3640_ISP_XOUT_H,     0x00},  //  0
	    {OV3640_ISP_XOUT_L,     0x60},  //  96
	    {OV3640_ISP_YOUT_H,     0x00},  //  0
	    {OV3640_ISP_YOUT_L,     0x40},  //  64
	#if 0
	    // Horizontal start
	    {OV3640_HS_H, 0x00 }, {OV3640_HS_L, 0x00 },
	    // Vertical start
	    {OV3640_VS_H, 0x00 }, {OV3640_VS_L, 0x00 }, /* crop window setting*/
	    // Horizontal width
	    {OV3640_HW_H, 0x00 }, {OV3640_HW_L, 0xb0 },   // 2075 ??
	    // Vertical height
	    {OV3640_VH_H, 0x00 }, {OV3640_VH_L, 0x90 },   // 772
	#endif
	    {0xFFFF, 0xff}
};

const static  sensor_reg_wA resize_ov3640_96_96[] = {
	    // ISP scaling input control
	    {OV3640_SIZE_IN_MISC,        0x34},  //  52 or 104 ?
	    {OV3640_HSIZE_IN_L,        0x4},  //  174 -> 12
	    {OV3640_VSIZE_IN_L,        0x4},  //  4 (?)

	    // ISP output
	    {OV3640_SIZE_OUT_MISC,    0x00},  //  0
	    {OV3640_HSIZE_OUT_L,    0x68},  //  104
	    {OV3640_VSIZE_OUT_L,    0x64},  //  100

	    {OV3640_ISP_PAD_CTR2,   0x42},  //  66

	    // ISP out
	    {OV3640_ISP_XOUT_H,     0x00},  //  0
	    {OV3640_ISP_XOUT_L,     0x60},  //  96
	    {OV3640_ISP_YOUT_H,     0x00},  //  0
	    {OV3640_ISP_YOUT_L,     0x60},  //  96
	#if 0
	    // Horizontal start
	    {OV3640_HS_H, 0x00 }, {OV3640_HS_L, 0x00 },
	    // Vertical start
	    {OV3640_VS_H, 0x00 }, {OV3640_VS_L, 0x00 }, /* crop window setting*/
	    // Horizontal width
	    {OV3640_HW_H, 0x00 }, {OV3640_HW_L, 0xb0 },   // 2075 ??
	    // Vertical height
	    {OV3640_VH_H, 0x00 }, {OV3640_VH_L, 0x90 },   // 772
	#endif
	    {0xFFFF, 0xff}
};

const static  sensor_reg_wA resize_ov3640_160_120[] = {
    // ISP scaling input control
    {OV3640_SIZE_IN_MISC,        0x34},  //  52 or 104 ?
    {OV3640_HSIZE_IN_L,        0x4},  //  174 -> 12
    {OV3640_VSIZE_IN_L,        0x4},  //  4 (?)

    // ISP output
    {OV3640_SIZE_OUT_MISC,    0x00},  //  0
    {OV3640_HSIZE_OUT_L,    0xa8},  //  168
    {OV3640_VSIZE_OUT_L,    0x7c},  //  124

    {OV3640_ISP_PAD_CTR2,   0x42},  //  66

    // ISP out
    {OV3640_ISP_XOUT_H,     0x00},  //  0
    {OV3640_ISP_XOUT_L,     0xa0},  //  160
    {OV3640_ISP_YOUT_H,     0x00},  //  0
    {OV3640_ISP_YOUT_L,     0x78},  //  120
#if 0
    // Horizontal start
    {OV3640_HS_H, 0x00 }, {OV3640_HS_L, 0x00 },
    // Vertical start
    {OV3640_VS_H, 0x00 }, {OV3640_VS_L, 0x00 }, /* crop window setting*/
    // Horizontal width
    {OV3640_HW_H, 0x00 }, {OV3640_HW_L, 0xb0 },   // 2075 ??
    // Vertical height
    {OV3640_VH_H, 0x00 }, {OV3640_VH_L, 0x90 },   // 772
#endif
    {0xFFFF, 0xff}
};

// manually calculated !!!! from 1036*772 (??)
const static  sensor_reg_wA resize_ov3640_176_144[] = {
    // ISP scaling input control
    {OV3640_SIZE_IN_MISC,		0x33},  //  51 or 104 ? - vs 0x68
    {OV3640_HSIZE_IN_L,		0xAE},  //  174 -> 12		- vs 0x18
    {OV3640_VSIZE_IN_L,		0x04},  //  4 (?)			- vs 0x0c

    // ISP output
    {OV3640_SIZE_OUT_MISC,	0x00},  //  0
    {OV3640_HSIZE_OUT_L,	0xb8},  //  184
    {OV3640_VSIZE_OUT_L,    0x94},  //  148

    {OV3640_ISP_PAD_CTR2,   0x42},  //  66

    // ISP out
    {OV3640_ISP_XOUT_H,     0x00},  //  0
    {OV3640_ISP_XOUT_L,     0xb0},  //  176
    {OV3640_ISP_YOUT_H,     0x00},  //  0
    {OV3640_ISP_YOUT_L,     0x90},  //  144
#if 0
    // Horizontal start
    {OV3640_HS_H, 0x00 }, {OV3640_HS_L, 0x00 },
    // Vertical start
    {OV3640_VS_H, 0x00 }, {OV3640_VS_L, 0x00 }, /* crop window setting*/
    // Horizontal width
    {OV3640_HW_H, 0x00 }, {OV3640_HW_L, 0xb0 },   // 2075 ??
    // Vertical height
    {OV3640_VH_H, 0x00 }, {OV3640_VH_L, 0x90 },   // 772
#endif
    {0xFFFF, 0xff}
};

const static  sensor_reg_wA resize_ov3640_320_240[] = {
	// ISP scaling input control
    {OV3640_SIZE_IN_MISC,	0x34},	// vs 0x68
    {OV3640_HSIZE_IN_L,		0x0c},	// vs 0x18
    {OV3640_VSIZE_IN_L,		0x04},	// vs 0x0C

	// ISP output
    {OV3640_SIZE_OUT_MISC,	0x01},
    {OV3640_HSIZE_OUT_L,    0x48}, // 328
    {OV3640_VSIZE_OUT_L,    0xf4}, // 244

    {OV3640_ISP_PAD_CTR2,	0x42},

	// ISP out
    {OV3640_ISP_XOUT_H,		0x01},
    {OV3640_ISP_XOUT_L,		0x40},
    {OV3640_ISP_YOUT_H,		0x00},
    {OV3640_ISP_YOUT_L,		0xf0},
#if 0
    // Horizontal start
    {OV3640_HS_H, 0x01 }, {OV3640_HS_L, 0x1d },
    // Vertical start
    {OV3640_VS_H, 0x00 }, {OV3640_VS_L, 0x06 }, /* crop window setting*/
    // Horizontal width
    {OV3640_HW_H, 0x08 }, {OV3640_HW_L, 0x18 },   // 2075 ??
    // Vertical height
    {OV3640_VH_H, 0x03 }, {OV3640_VH_L, 0x04 },   // 772
    #endif
    {0xFFFF, 0xff}
};

const static  sensor_reg_wA resize_ov3640_352_288[] = {
    {OV3640_SIZE_IN_MISC,   0x33},
    {OV3640_HSIZE_IN_L,     0xAE},
    {OV3640_VSIZE_IN_L,     0x04},
    {OV3640_SIZE_OUT_MISC,  0x11},
    {OV3640_HSIZE_OUT_L,    0x68},
    {OV3640_VSIZE_OUT_L,    0x24},  // 36
    {OV3640_ISP_PAD_CTR2,   0x42},
    {OV3640_ISP_XOUT_H,     0x01},
    {OV3640_ISP_XOUT_L,     0x60},
    {OV3640_ISP_YOUT_H,     0x01},
    {OV3640_ISP_YOUT_L,     0x20},
    {0xFFFF, 0xff}
};

const static  sensor_reg_wA resize_ov3640_640_480[] = {
    {OV3640_SIZE_IN_MISC,   0x34},
    {OV3640_HSIZE_IN_L,     0x04},
    {OV3640_VSIZE_IN_L,     0x04},
    {OV3640_SIZE_OUT_MISC,  0x12},
    {OV3640_HSIZE_OUT_L,    0x88},
    {OV3640_VSIZE_OUT_L,    0xE4},
    {OV3640_ISP_PAD_CTR2,   0x42},
    {OV3640_ISP_XOUT_H,     0x02},
    {OV3640_ISP_XOUT_L,     0x80},
    {OV3640_ISP_YOUT_H,     0x01},
    {OV3640_ISP_YOUT_L,     0xe0},
    {0xFFFF, 0xff}
};

const static sensor_reg_wA ov3640_vga[] = {
	{0x3012, 0x10},
	{0x3023, 0x06},
	{0x3026, 0x03},
	{0x3027, 0x04},
	{0x302a, 0x03},
	{0x302b, 0x10},
	{0x3075, 0x24},
	{0x300d, 0x01},
	{0x30d7, 0x90},
	{0x3069, 0x04},
	{0x303e, 0x00},
	{0x303f, 0xc0},
	{0x3302, 0xef},
	{0x335f, 0x34},
	{0x3360, 0x0c},
	{0x3361, 0x04},
	{0x3362, 0x12},
	{0x3363, 0x88},
	{0x3364, 0xe4},
	{0x3403, 0x42},
	{0x3088, 0x02},//12->02
	{0x3089, 0x80},
	{0x308a, 0x01},
	{0x308b, 0xe0},
	{0x308b, 0xe0},
	{0xFFFF, 0xff}
};

const static sensor_reg_wA ov3640_fmt_yuv422_qvga[] = {
	{0x3002, 0x06 }, {0x3003, 0x1F }, {0x3001, 0x12 }, {0x304d, 0x45 },
	{0x30aa, 0x45 }, {0x30B0, 0xff }, {0x30B1, 0xff }, {0x30B2, 0x10 },
	{0x30d7, 0x10 }, {0x3047, 0x00 }, {0x3018, 0x60 }, {0x3019, 0x58 },
	{0x301A, 0xa1 }, {0x3087, 0x02 }, {0x3082, 0x20 }, {0x303C, 0x08 },
	{0x303d, 0x18 }, {0x303e, 0x06 },
	{0x303f, 0x0c }, {0x3030, 0x62 }, {0x3031, 0x26 }, {0x3032, 0xe6 },
	{0x3033, 0x6e }, {0x3034, 0xea }, {0x3035, 0xae }, {0x3036, 0xa6 },
	{0x3037, 0x6a }, {0x3015, 0x12 }, {0x3013, 0xfd }, {0x3104, 0x02 },
	{0x3105, 0xfd }, {0x3106, 0x00 }, {0x3107, 0xff }, {0x3308, 0xa5 },
	{0x3316, 0xff }, {0x3317, 0x00 }, {0x3087, 0x02 }, {0x3082, 0x20 },
	{0x3300, 0x13 }, {0x3301, 0xd6 }, {OV3640_DSP_CTRL_2, 0xef }, {0x30B8, 0x20 },
	{0x30B9, 0x17 }, {0x30BA, 0x04 }, {0x30BB, 0x08 }, {0x3507, 0x06 },
	{0x350a, 0x4f }, {0x3600, 0xc4 }, {0x332B, 0x00 }, {0x332D, 0x45 },
	{0x332D, 0x60 }, {0x332F, 0x03 },
	{0x3100, 0x02 }, {0x3304, 0xfc }, {0x3400, 0x00 }, {0x3404, 0x02 }, /* YUV422 */
	{0x3601, 0x01 }, {0x302a, 0x06 }, {0x302b, 0x20 },
	{0x300E, 0x32 }, {0x300F, 0x21 }, {0x3010, 0x21 }, {0x3011, 0x01 }, /* QXGA PLL setting*/
	{0x304c, 0x81 },
	{0x3602, 0x22 }, {0x361E, 0x00 }, {0x3622, 0x18 }, {0x3623, 0x69 }, /* CSI setting */
	{0x3626, 0x00 }, {0x3627, 0xf0 }, {0x3628, 0x00 }, {0x3629, 0x26 },
	{0x362A, 0x00 }, {0x362B, 0x5f }, {0x362C, 0xd0 }, {0x362D, 0x3c },
	{0x3632, 0x10 }, {0x3633, 0x28 }, {0x3603, 0x4d }, {0x364C, 0x04 },
	{0x309e, 0x00 },

	/* crop window setting*/
	// Horizontal start
	#if 0
	{OV3640_HS_H, 0x01 }, {OV3640_HS_L, 0x1d }, // 285
	// Vertical start
	{OV3640_VS_H, 0x00 }, {OV3640_VS_L, 0x0a }, // 10
	// Horizontal width
	{OV3640_HW_H, 0x04 }, {OV3640_HW_L, 0x00 },	// 1024
	// Vertical height
	{OV3640_VH_H, 0x03 }, {OV3640_VH_L, 0x00 },	// 768
	#else
	{OV3640_HS_H, 0x01 }, {OV3640_HS_L, 0x00 }, 
	// Vertical start
	{OV3640_VS_H, 0x00 }, {OV3640_VS_L, 0x00 }, 
	// Horizontal width
	{OV3640_HW_H, 0x08 }, {OV3640_HW_L, 0x18 },
	// Vertical height
	{OV3640_VH_H, 0x06 }, {OV3640_VH_L, 0x0C },
	#endif

	// SIZE
	// ISP scaling in (zoom out ? )
	{OV3640_SIZE_IN_MISC, 0x68 }, // high part V, low part H
	{OV3640_HSIZE_IN_L, 0x18 }, //
	{OV3640_VSIZE_IN_L, 0x0C }, //

	// ISP_OUT
	{OV3640_SIZE_OUT_MISC, 0x01 }, // high part
	{OV3640_HSIZE_OUT_L, 0x40 }, // 328
	{OV3640_VSIZE_OUT_L, 0xf0 }, // 240

	{OV3640_ISP_PAD_CTR2, 0x42 },  /* QVGA */

	// SIZE OUT
	{OV3640_ISP_XOUT_H, 0x01 }, {OV3640_ISP_XOUT_L, 0x40 }, 	// X 320
	{OV3640_ISP_YOUT_H, 0x00 }, {OV3640_ISP_YOUT_L, 0xf0 },		// Y 240

	{0x3355, 0x04 }, {0x3354, 0x01 }, {0x335e, 0x05 },      /* brightness */
	{0x3355, 0x04 }, {0x335c, 0x20 }, {0x335d, 0x20 },      /* contrast */
	{0xffff, 0xff}
};

const static sensor_reg_wA test_bar_enable[] {
   {0x306c, 0x00}, //Enable color bar
   {0x307b, 0x42}, //Select color bar
   {0x307d, 0x80},  //Enable color bar
   {0xFFFF, 0xff}
};

const static sensor_reg_wA test_bar_disable[] {
   {0x306c, 0x00}, //Enable color bar
   {0xFFFF, 0xff}
};

#endif

