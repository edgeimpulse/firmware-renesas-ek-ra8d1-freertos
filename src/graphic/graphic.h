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
extern void graphic_no_detection(void);
extern void graphic_start_detection(void);

extern uint8_t* graphic_get_draw_buffer(void);
extern void graphic_start_buffer(void);
extern void graphic_end_frame(void);

FSP_CPP_FOOTER

#endif /* GRAPHIC_GRAPHIC_H_ */
