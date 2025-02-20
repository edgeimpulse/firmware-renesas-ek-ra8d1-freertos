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

#ifndef GRAPHIC_GRAPHIC_H_
#define GRAPHIC_GRAPHIC_H_

#include "bsp_api.h"

#ifdef FULL_SCREEN
// full screen
#define CAM_LAYER_SIZE_X 480 // 000 --> 480
#define CAM_LAYER_SIZE_Y 854 // 000 --> 854


#else
// normal screen
//#define CAM_LAYER_SIZE_X 476 // 000 --> LCD_VPIX
#define CAM_LAYER_SIZE_X 480
#define CAM_LAYER_SIZE_Y CAM_LAYER_SIZE_X // 000 --> 480
//#define CAM_LAYER_SIZE_X 320 // 000 --> LCD_VPIX
//#define CAM_LAYER_SIZE_Y 240 // 000 --> 480

#endif

FSP_CPP_HEADER

extern void display_swap_buffer(void);
extern void display_change_buffer(void);
extern int graphic_init(void);
extern void graphic_deinit(void);
extern void graphic_display_draw(void);
extern void graphic_set_framebuffer(uint8_t* new_fb, uint16_t width, uint16_t height);
extern void graphic_set_timing(int32_t fsp, int32_t dsp_us, int32_t classification_us);
extern void graphic_set_box(const char* label, uint16_t x0, uint16_t y0, uint16_t width, uint16_t height, uint16_t idx, float ratio);
extern void graphic_set_centroid(const char* label, uint16_t x0, uint16_t y0, uint16_t width, uint16_t height, uint16_t idx, float ratio);
extern void graphic_no_detection(void);
extern void graphic_start_detection(void);
extern void graphic_classification(const char* label, float value, uint16_t idx);

extern uint8_t* graphic_get_draw_buffer(void);
extern void graphic_start_buffer(void);
extern void graphic_end_frame(void);

FSP_CPP_FOOTER

#endif /* GRAPHIC_GRAPHIC_H_ */
