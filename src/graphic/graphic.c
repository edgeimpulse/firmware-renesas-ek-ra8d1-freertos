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

#include "FreeRTOS.h"
#include "semphr.h"

#include "graphic.h"
#include "hal_data.h"
#include "peripheral/mipi_display/display_mipi_ili9806e.h"
#include "peripheral/mipi_display/dsi_layer.h"
#include "text/text.h"
#include <stdio.h>

#ifdef NORMAL_SCREEN
#define LCD_HPIX       (800)
#define LCD_STRIDE     (800)
#define LCD_VPIX       (480)
#else
#define LCD_HPIX       DISPLAY_HSIZE_INPUT0 // 480
#define LCD_STRIDE     DISPLAY_HSIZE_INPUT0
#define LCD_VPIX       DISPLAY_VSIZE_INPUT0 // 854
//#define LCD_VPIX       (858)
#endif

/* RESOLUTION FROM CAMERA */
//#define CAM_IMG_SIZE_X   320
//#define CAM_IMG_SIZE_Y   240  // Trim the Right Hand Edge hiding corruption

//#define CAM_IMAGE_CENTER_X  ((DISPLAY_HSIZE_INPUT0 - CAM_IMG_SIZE_X) / 2)
//#define CAM_IMAGE_CENTER_X  ((DISPLAY_HSIZE_INPUT0 - CAM_LAYER_SIZE_Y) / 2)

// spacing between strings
#define STRING_SPACING          (20u)

// predictions strings coordinate
#define PREDICTIONS_Y_OFFSET    (10u)
#define DSP_Y_OFFSET            (PREDICTIONS_Y_OFFSET + STRING_SPACING)
#define CLASSIFICATION_Y_OFFSET (DSP_Y_OFFSET + STRING_SPACING)

// for detection
#define DETECTION_Y_OFFSET              (CLASSIFICATION_Y_OFFSET + STRING_SPACING)
#define DETECTION_LABEL_Y_OFFSET        (DETECTION_Y_OFFSET + STRING_SPACING)
#define DETECTION_COORD_Y_OFFSET        (DETECTION_LABEL_Y_OFFSET + STRING_SPACING)

#define LCD_COLOR_SIZE (2)
#define LCD_BUF_NUM    (2)

#define BYTES_PER_PIXEL         (2U)
#define ARRAY_INDEX             (0U)
#define BITFIELD_VALUE          (0U)
#define RENDER_DELAY            (25U)
#define SW_DELAY                (15U)
#define RESET_FLAG              (0U)
#define SET_FLAG                (1U)

#define ALPHA_VALUE             (102U)
#define ANTI_ALIASING_VAL       (1U)
#define BUFFER_CLEAR_VAL        (0x000000)
#define RED_COLOR_VAL           (0xFF0000)
#define GREEN_COLOR_VAL         (0x00FF00)
#define BLUE_COLOR_VAL          (0x0000FF)
#define YELLOW_COLOR_VAL        (0xFFFF00)
#define BLACK_COLO_VAL          (0xFFFFFF)
#define SHIFT_VALUE             (4U)

#define ROTATE_SNAPSHOT         (0)
#define DRAW_BOX                (0)

extern d2_device * d2_handle;
static uint8_t * p_framebuffer = NULL;

static uint8_t * g_p_single_buffer;
static uint8_t * g_p_double_buffer;

/* Variables to store resolution information */
static uint16_t g_hz_size = 0;
static uint16_t g_vr_size = 0;

display_runtime_cfg_t glcd_layer_change;

// used when copying local frame buffer to frame buffer
static int32_t cam_witdh = 0;
static int32_t cam_heigth = 0;

static uint8_t index_local_buffer;
static uint8_t local_camera_buff[2][320 * 240 * 2] BSP_PLACE_IN_SECTION(".sdram") BSP_ALIGN_VARIABLE(64);

static int32_t gfsp;
static int32_t gdsp_us;
static int32_t gclassification_us;

static void init_dave(void);
static void graphic_error_handler(d2_s32 DRW_err);
static void graphic_drw_set(void);
static void graphic_print_strings(void);
static void graphic_draw_detection(void);
static void graphic_copy_to_fb(void);
static void rot90_clock(uint8_t* input_image, uint8_t* output_image, int n_ch, int ip_w, int ip_h);
static void reverse_u32pixel(uint32_t* addr, uint32_t length);

static SemaphoreHandle_t sem_local_buffer;

/**
 * @brief init d2 driver
 * @return
 */
int graphic_init(void)
{
    fsp_err_t err;

    display_input_cfg_t const *p_input   = &g_display0.p_cfg->input[0];
    display_output_cfg_t const *p_output = &g_display0.p_cfg->output;

    /* init d/ave driver */
    init_dave();

    /* Reset display */
    dsi_layer_hw_reset();

    memset(&fb_background[0][0], 0x00, DISPLAY_BUFFER_STRIDE_BYTES_INPUT0 * DISPLAY_VSIZE_INPUT0);
    memset(&fb_background[1][0], 0x00, DISPLAY_BUFFER_STRIDE_BYTES_INPUT0 * DISPLAY_VSIZE_INPUT0);
    memset(local_camera_buff, 0x00, sizeof(local_camera_buff));

    index_local_buffer = 0;

    /* copy the data to runtime - for GLCDC layer change */
    glcd_layer_change.input = g_display0.p_cfg->input[0];
    glcd_layer_change.layer = g_display0.p_cfg->layer[0];

    /* Center image */
   glcd_layer_change.layer.coordinate.x = (int16_t)(int16_t)(p_output->htiming.display_cyc - p_input->hsize) / 2;
   glcd_layer_change.layer.coordinate.y = (int16_t)(int16_t)(p_output->vtiming.display_cyc - p_input->vsize) / 2;

   (void)R_GLCDC_LayerChange(&g_display0.p_ctrl, &glcd_layer_change, DISPLAY_FRAME_LAYER_1);

    /* Get LCDC configuration */
    g_hz_size = (g_display0_cfg.input[ARRAY_INDEX].hsize);
    g_vr_size = (g_display0_cfg.input[ARRAY_INDEX].vsize);

    //g_p_single_buffer = (uint8_t *) g_display0_cfg.input[ARRAY_INDEX].p_base;
    g_p_single_buffer = (uint8_t *) fb_background[0];

    /* second buffer */
    g_p_double_buffer = (uint8_t *) fb_background[1];

    p_framebuffer = g_p_single_buffer;

    err = display_mipi_ili9806e_init(&g_display0);
    sem_local_buffer = xSemaphoreCreateBinary();

    if ((FSP_SUCCESS != err) || (sem_local_buffer == NULL)) {
        while(1) {
            // error
        };
    }

    xSemaphoreGive(sem_local_buffer); // it begins empty !

    return (int)err;
}

/**
 *
 */
void graphic_deinit(void)
{
    d2_s32      DRW_err;

    /* Wait for rendering operations to finish */
    DRW_err = d2_flushframe(d2_handle);
    graphic_error_handler(DRW_err);

    /* De-initialize hardware and close the handle */
    DRW_err = d2_deinithw(d2_handle);
    graphic_error_handler(DRW_err);

    DRW_err = d2_closedevice(d2_handle);
    graphic_error_handler(DRW_err);
}

/*******************************************************************************************************************//**
 * User-defined function to draw the current display to a framebuffer
 *
 * @param[in]  framebuffer    Pointer to frame buffer
 * @retval     none
 **********************************************************************************************************************/
void graphic_display_draw(void)
{
    if (p_framebuffer == NULL) {
        return;
    }

    /* set frame buffer properties */
    graphic_drw_set();

    /* DRW operations happens here */

    /* get the latest camera */
    graphic_copy_to_fb();

    /* draw bounding boxes */
    graphic_draw_detection();

    /* print strings */
    graphic_print_strings();
}

/**
 *
 */
void display_swap_buffer(void)
{
    /* Swap the active framebuffer */
    p_framebuffer = (p_framebuffer == g_p_single_buffer) ? g_p_double_buffer : g_p_single_buffer;
}

/**
 *
 */
void display_change_buffer(void)
{
    R_GLCDC_BufferChange(&g_display0_ctrl, p_framebuffer, DISPLAY_FRAME_LAYER_1);
}

/*******************************************************************************************************************//**
 * Error handler for DRW.
 *
 * @param[in]  err       Error returned from DRW APIs
 * @param[in]  gp_dave   Pointer to device structure
 * @retval     none
 **********************************************************************************************************************/
static void graphic_error_handler(d2_s32 err)
{
    if (D2_OK != err) {
        /* error */
        while(1) {
            __NOP();
            R_BSP_PinWrite(BSP_IO_PORT_01_PIN_07, BSP_IO_LEVEL_HIGH);
            R_BSP_SoftwareDelay(500, BSP_DELAY_UNITS_MILLISECONDS);
            R_BSP_PinWrite(BSP_IO_PORT_01_PIN_07, BSP_IO_LEVEL_LOW);
            R_BSP_SoftwareDelay(500, BSP_DELAY_UNITS_MILLISECONDS);
        }
    }

}

/**
 * @brief Start a new display list, set the framebuffer and add a clear operation
 *
 * @note This function will automatically prepare an empty framebuffer.
 */
void graphic_start_buffer(void)
{
    /* Start a new display list */
    d2_startframe(d2_handle);

    /* Set the new buffer to the current draw buffer */
    d2_framebuffer(d2_handle, (void *)p_framebuffer, LCD_HPIX, LCD_STRIDE, LCD_VPIX, d2_mode_rgb565);

    /* Clear the frame buffer */
    d2_clear(d2_handle, BUFFER_CLEAR_VAL);
}

/**
 * @brief End the current display list and flip the active framebuffer
 *
 * @note WARNING: As part of d2_endframe the D2 driver will wait for the current frame to finish displaying.
 */
void graphic_end_frame(void)
{
    /* Wait for previous frame rendering to finish, then finalize this frame and flip the buffers */
    d2_flushframe(d2_handle);

    /* End the current display list */
    d2_endframe(d2_handle);

    /* Flip the framebuffer */
    //display_swap_buffer();

    /* Clean data cache */
    SCB_CleanDCache();
}

/**
 *
 * @return
 */
uint8_t* graphic_get_draw_buffer(void)
{
    // can be removed ?
    return p_framebuffer;
}

/**
 *
 * @param new_fb
 * @param x0
 * @param y0
 * @param width
 * @param height
 */
void graphic_set_framebuffer(uint8_t* new_fb, uint16_t width, uint16_t height)
{
    uint8_t copy_local_buffer = (index_local_buffer) ? 0 : 1;

    SCB_CleanDCache();
    SCB_EnableDCache();

#if (ROTATE_SNAPSHOT == 1)
    rot90_clock(new_fb, local_camera_buff[copy_local_buffer], 2, width, height);
#else
    // basically copy the new_fb into a local fb that later on will be used (on next draw call)
    memcpy(local_camera_buff[copy_local_buffer], new_fb, (width * height * BYTES_PER_PIXEL));
#endif
    reverse_u32pixel((uint32_t*)local_camera_buff[copy_local_buffer], ((width * height * BYTES_PER_PIXEL) /4));

    SCB_DisableDCache();
    SCB_CleanDCache();

    // critical section
    xSemaphoreTake(sem_local_buffer, portMAX_DELAY);
    cam_witdh = width;
    cam_heigth = height;
    index_local_buffer = copy_local_buffer;
    xSemaphoreGive(sem_local_buffer);
}

/**
 *
 * @param dsp_us
 * @param classification_us
 */
void graphic_set_timing(int32_t fsp, int32_t dsp_us, int32_t classification_us)
{
    gfsp = fsp;
    gdsp_us = dsp_us;
    gclassification_us = classification_us;
}

/**
 * @brief 
 * 
 * @param label 
 * @param x0 
 * @param y0 
 * @param width 
 * @param height 
 * @param idx 
 * @param ratio 
 */
void graphic_set_centroid(const char* label, uint16_t x0, uint16_t y0, uint16_t width, uint16_t height, uint16_t idx, float ratio)
{
    char temp_str[256] = {0};
    uint16_t new_x0 = (x0 * ratio);
    uint16_t new_y0 = (y0 * ratio);
    uint16_t new_width = (width * ratio);
    uint16_t new_height = (height * ratio);

    d2_setcolor(d2_handle, 0, GREEN_COLOR_VAL);

    d2_point x_center = (((new_x0 << 1) + new_width) >> 1);
    d2_point y_center = (((new_y0 << 1)+ new_height) >> 1);

    d2_rendercircle(d2_handle, (x_center << 4), (y_center << 4), (d2_width)(10 << 4), (d2_width)(2 << 4));
    d2_setcolor(d2_handle, 0, YELLOW_COLOR_VAL);
    snprintf(temp_str, 256, "    %s", label);
    text_print(d2_handle, temp_str, (uint8_t)strlen(temp_str), 10, CAM_LAYER_SIZE_Y + DETECTION_LABEL_Y_OFFSET + (idx * STRING_SPACING), YELLOW_COLOR_VAL);

    snprintf(temp_str, 256, "    [ x: %d, y: %d, width: %d, height: %d ]", x0, y0, width, height);
    text_print(d2_handle, temp_str, (uint8_t)strlen(temp_str), 10, CAM_LAYER_SIZE_Y + DETECTION_COORD_Y_OFFSET + (idx * STRING_SPACING), YELLOW_COLOR_VAL);
}

/**
 * @brief 
 * 
 * @param label 
 * @param x0 
 * @param y0 
 * @param width 
 * @param height 
 * @param idx 
 * @param ratio 
 */
void graphic_set_box(const char* label, uint16_t x0, uint16_t y0, uint16_t width, uint16_t height, uint16_t idx, float ratio)
{
    char temp_str[256] = {0};
    uint16_t new_x0 = (x0 * ratio);
    uint16_t new_y0 = (y0 * ratio);
    uint16_t new_width = (width * ratio);
    uint16_t new_height = (height * ratio);

    d2_setcolor(d2_handle, 0, GREEN_COLOR_VAL);

    d2_renderline(d2_handle, (d2_point) ((new_x0) << 4), (d2_point) (new_y0 << 4), (d2_point) ((new_x0 + new_width) << 4), (d2_point) ((new_y0 ) << 4), (d2_point) (2 << 4), 0);
    d2_renderline(d2_handle, (d2_point) ((new_x0 + new_width) << 4), (d2_point) (new_y0 << 4), (d2_point) ((new_x0 + new_width) << 4), (d2_point) ((new_y0 + new_height) << 4), (d2_point) (2 << 4), 0);
    d2_renderline(d2_handle, (d2_point) ((new_x0 + new_width) << 4), (d2_point) ((new_y0 + new_height) << 4), (d2_point) (new_x0 << 4), (d2_point) ((new_y0 + new_height) << 4), (d2_point) (2 << 4), 0);
    d2_renderline(d2_handle, (d2_point) ((new_x0) << 4), (d2_point) ((new_y0 + new_height) << 4), (d2_point) (new_x0 << 4), (d2_point) (new_y0 << 4), (d2_point) (2 << 4), 0);

    d2_setcolor(d2_handle, 0, YELLOW_COLOR_VAL);
    snprintf(temp_str, 256, "    %s", label);
    text_print(d2_handle, temp_str, (uint8_t)strlen(temp_str), 10, CAM_LAYER_SIZE_Y + DETECTION_LABEL_Y_OFFSET + (idx * STRING_SPACING), YELLOW_COLOR_VAL);

    snprintf(temp_str, 256, "    [ x: %d, y: %d, width: %d, height: %d ]", x0, y0, width, height);
    text_print(d2_handle, temp_str, (uint8_t)strlen(temp_str), 10, CAM_LAYER_SIZE_Y + DETECTION_COORD_Y_OFFSET + (idx * STRING_SPACING), YELLOW_COLOR_VAL);
}

/**
 *
 */
void graphic_no_detection(void)
{
    const char temp_str[] = "No object detected";

    text_print(d2_handle, temp_str, (uint8_t)strlen(temp_str), 10, CAM_LAYER_SIZE_Y + DETECTION_Y_OFFSET, YELLOW_COLOR_VAL);
}

/**
 *
 */
void graphic_classification(const char* label, float value, uint16_t idx)
{
    char temp_str[256] = {0};
    char float_string[5] = {0};

    float n = value;

    static double PRECISION = 0.00001;
    static int MAX_NUMBER_STRING_SIZE = 32;

    char s[MAX_NUMBER_STRING_SIZE];

    if (n == 0.0) {
        strcpy(s, "0");
    }
    else {
        int digit, m;
        char *c = s;
        int neg = (n < 0);
        if (neg) {
            n = -n;
        }
        // calculate magnitude
        m = log10(n);
        if (neg) {
            *(c++) = '-';
        }
        if (m < 1.0) {
            m = 0;
        }
        // convert the number
        while (n > PRECISION || m >= 0) {
            double weight = pow(10.0, m);
            if (weight > 0 && !isinf(weight)) {
                digit = floor(n / weight);
                n -= (digit * weight);
                *(c++) = '0' + digit;
            }
            if (m == 0 && n > 0) {
                *(c++) = '.';
            }
            m--;
        }
        *(c) = '\0';
    }

    d2_setcolor(d2_handle, 0, YELLOW_COLOR_VAL);

    snprintf(temp_str, 256, "    %s: %s", label, s);
    text_print(d2_handle, temp_str, (uint8_t)strlen(temp_str), 10, CAM_LAYER_SIZE_Y + DETECTION_Y_OFFSET + (idx * STRING_SPACING), YELLOW_COLOR_VAL);
}

/**
 *
 */
void graphic_start_detection(void)
{
    const char temp_str[] = "Detected:";

    text_print(d2_handle, temp_str, (uint8_t)strlen(temp_str), 10, CAM_LAYER_SIZE_Y + DETECTION_Y_OFFSET, YELLOW_COLOR_VAL);
}

/**
 * @brief Copy local fb to the display fb
 */
static void graphic_copy_to_fb(void)
{
    if ((cam_witdh == 0) || (cam_heigth == 0)) {
        return;
    }

    // critical section
    xSemaphoreTake(sem_local_buffer, portMAX_DELAY);
    d2_setblitsrc(d2_handle, local_camera_buff[index_local_buffer], (d2_s32)cam_witdh, (d2_s32)cam_witdh, (d2_s32)cam_heigth, (d2_u32)d2_mode_rgb565);
    d2_blitcopy(d2_handle,
            (d2_s32)cam_witdh, (d2_s32)cam_heigth,  // Source width/height
                (d2_blitpos) 0, (d2_blitpos) 0, // Source position
                (d2_width) ((CAM_LAYER_SIZE_X) << 4), (d2_width) ((CAM_LAYER_SIZE_Y) << 4), // Destination size width/height
                0, 0,  // Destination offset position
                //(d2_width) (CAM_IMAGE_CENTER_X << 4), (d2_width) (10 << 4), // Destination offset position
                d2_bf_filter);
    xSemaphoreGive(sem_local_buffer);
}

/**
 * @brief print timing strings on fb
 */
static void graphic_print_strings(void)
{
    char timing_str[256] = {0};

    snprintf(timing_str, 256, "Edge Impulse live inference - FSP: %ld", gfsp);
    text_print(d2_handle, timing_str, (uint8_t)strlen(timing_str), 10, 10, YELLOW_COLOR_VAL);

    snprintf(timing_str, 256, "Predictions time:");
    text_print(d2_handle, timing_str, (uint8_t)strlen(timing_str), 10, CAM_LAYER_SIZE_Y + PREDICTIONS_Y_OFFSET, YELLOW_COLOR_VAL);

    snprintf(timing_str, 256, "            DSP: %ld us.", gdsp_us);
    text_print(d2_handle, timing_str, (uint8_t)strlen(timing_str), 10, CAM_LAYER_SIZE_Y + DSP_Y_OFFSET, YELLOW_COLOR_VAL);

    snprintf(timing_str, 256, "            Classification: %ld us.", gclassification_us);
    text_print(d2_handle, timing_str, (uint8_t)strlen(timing_str), 10, CAM_LAYER_SIZE_Y + CLASSIFICATION_Y_OFFSET, YELLOW_COLOR_VAL);
}

/**
 * @brief Draw detection box
 */
static void graphic_draw_detection()
{
    // todo...
}

/*******************************************************************************************************************//**
 * User-defined function to set framebuffer properties
 *
 * @param[in]  framebuffer    Pointer to frame buffer
 * @retval     none
 **********************************************************************************************************************/
static void graphic_drw_set(void)
{
    /* Set anti-aliasing and line cap settings */
    d2_setantialiasing(d2_handle, ANTI_ALIASING_VAL);

    d2_setlinecap(d2_handle, d2_lc_round);

    d2_setlinejoin(d2_handle, d2_lj_none);

    /* Set render mode */
    d2_selectrendermode(d2_handle , d2_rm_solid);
}

/**
 * @brief Init D/ave driver
 */
static void init_dave(void)
{
    /* Initialize D/AVE 2D driver */
    d2_handle = d2_opendevice(0);
    d2_inithw(d2_handle, 0);

    /* Clear both buffers */
    d2_framebuffer(d2_handle, fb_background, LCD_HPIX, LCD_STRIDE, LCD_VPIX * LCD_BUF_NUM, d2_mode_rgb565);
    d2_clear(d2_handle, BLUE_COLOR_VAL);

    /* Process active displaylist to clear framebuffers */
    d2_startframe(d2_handle);
    //d2_flushframe(d2_handle);
    d2_endframe(d2_handle);

    /* Set various D2 parameters */
    d2_setcolor(d2_handle, 0, 0xffffff );   // set white
    d2_setblendmode(d2_handle, d2_bm_alpha, d2_bm_one_minus_alpha);
    d2_setalphamode(d2_handle, d2_am_constant);
    d2_setalpha(d2_handle, 0xff);
    d2_setantialiasing(d2_handle, 1);
    d2_setlinecap(d2_handle, d2_lc_butt);
    d2_setlinejoin(d2_handle, d2_lj_none);
}

/**
 *
 * @param p_args
 */
void glcd_user_callback(display_callback_args_t *p_args)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if (p_args->event == DISPLAY_EVENT_LINE_DETECTION) {
        xSemaphoreGiveFromISR(g_vsync_display, &xHigherPriorityTaskWoken);
    }

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/**
 *
 * @param input_image
 * @param output_image
 * @param n_ch
 * @param ip_w
 * @param ip_h
 */
static void rot90_clock(uint8_t* input_image, uint8_t* output_image, int n_ch, int ip_w, int ip_h)
{
    // Make sure n_ch is 2 for RGB565
    if (n_ch != 2) {
        return;
    }

    for (int y = 0; y < ip_h; y++) {
        for (int x = 0; x < ip_w; x++) {
            // For each pixel, find its new position after a 90 degree rotation
            int newX = ip_h - 1 - y;
            int newY = x;

            // Calculate source and destination offsets
            int srcOffset = (y * ip_w + x) * n_ch;
            int dstOffset = (newY * ip_h + newX) * n_ch;

            // Copy the RGB565 value from input to output image
            output_image[dstOffset] = input_image[srcOffset];
            output_image[dstOffset + 1] = input_image[srcOffset + 1];
        }
    }
}

/**
 *
 * @param addr
 * @param length
 */
static void reverse_u32pixel(uint32_t* addr, uint32_t length)
{
    if(NULL == addr) {
        return;
    }

    uint32_t data;
    uint32_t* pend = (addr + length);

    for(; addr<pend; addr++)
    {
        data = *(addr);
        *(addr) = ((data & 0x000000FF) << 24) | ((data & 0x0000FF00) << 8) |
                ((data & 0x00FF0000) >> 8) | ((data & 0xFF000000) >> 24) ;
    }  //1.7ms
}
