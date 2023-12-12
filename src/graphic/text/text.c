#include "text.h"
#include "hal_data.h"
#include "fonts/04b_03_FNT.h"
#include "fonts/04b_03_TGA.h"
#include "fonts/DOS_8x16_FNT.h"
#include "fonts/DOS_8x16_TGA.h"

/******************************************************************************
 * Macro definitions
 *****************************************************************************/
/* Macro to access FNT char data */
#define CHAR_PTR(C)			(uint16_t *)(fnt_base + 20*(C) + 4)

#define FONT_TGA    DOS_8x16_TGA
#define FONT_FNT    DOS_8x16_FNT
#define TGA_STRIDE_PX 128
#define TGA_SIZE_X    128
#define TGA_SIZE_Y    128
#define CHAR_MAX_SIZE_X 8
#define CHAR_MAX_SIZE_Y 16

/******************************************************************************
 * Function definitions
 *****************************************************************************/

/*******************************************************************************************************************//**
 * Print text onto the current framebuffer using D2 texture mapping
 * 
 * @param      d2_handle       D2 driver handle
 * @param      string          String to print
 * @param[in]  length          Length of string
 * @param[in]  x               X-coordinate
 * @param[in]  y               Y-coordinate
 * @param[in]  color           Text color
 * 
 **********************************************************************************************************************/
void text_print(d2_device *d2_handle, char *string, uint8_t length, uint16_t x, uint16_t y, uint32_t color) {
    uint8_t ch;
    uint16_t *ch_ptr;
    uint16_t i = 0;
    uint16_t tga_x0, tga_y0;
    uint16_t lcd_x0, lcd_y0;
    uint16_t size_x, size_y;
    uint16_t offset_x, offset_y;

    /* Get the base address of the font texture and offset data */
    const uint8_t * tga_base = FONT_TGA + 18;
    const uint8_t * fnt_base = FONT_FNT;

    /* Set the texture operation to chroma-key replace the background and to adjust foreground color */
    d2_settextureoperation(d2_handle, d2_to_copy, d2_to_replace, d2_to_replace, d2_to_replace);
    d2_settexopparam(d2_handle, d2_cc_red, color>>16, 0);
    d2_settexopparam(d2_handle, d2_cc_green, (color&0x0000FF00)>>8, 0);
    d2_settexopparam(d2_handle, d2_cc_blue, color&0x000000FF, 0);
    d2_setcolorkey(d2_handle, 1, 0x000000);

    /* Set the graphics fill mode to use texture mapped data */
    d2_u8 prevfillmode = d2_getfillmode(d2_handle);
    d2_setfillmode(d2_handle, d2_fm_texture);

    ch = (uint8_t) string[0];

    /* Traverse the string */
    while(ch != '\0' && length)
    {
        /* Jump to the FNT file block where the information for this char is located */
        ch_ptr = CHAR_PTR(ch);

        /* Retrieve targa size and offset information for this char */
        tga_x0 = *ch_ptr>>8;
        tga_y0 = *(++ch_ptr)>>8;
        size_x = *(++ch_ptr)>>8;
        size_y = *(++ch_ptr)>>8;
        offset_x = *(++ch_ptr)>>8;
        offset_y = *(++ch_ptr)>>8;

        /* Limit character size in the case of special or invalid characters */
        if(offset_x > CHAR_MAX_SIZE_X) offset_x = CHAR_MAX_SIZE_X;
        if(offset_y > CHAR_MAX_SIZE_Y) offset_y = CHAR_MAX_SIZE_Y;

        /* Get start LCD coordinates based on character offsets */
        lcd_x0 = (uint16_t)(x + offset_x);
        lcd_y0 = (uint16_t)(y + offset_y);

        /* Set texture buffer in D2 driver */
        d2_settexture(d2_handle, (void *) tga_base, TGA_STRIDE_PX, TGA_SIZE_X, TGA_SIZE_Y, d2_mode_alpha8);

        /* Set texture mapping for the following:
         *   - Map on the display starting at lcd_x0, lcd_y0
         *   - Start in the texture at tga_x0, tga_y0
         *   - For each pixel in the X and Y direction on the display move exactly one pixel in the respective
         *     direction in the texture
         */
        d2_settexturemapping(d2_handle,
                             (d2_point)(lcd_x0 << 4), (d2_point)(lcd_y0 << 4),
                             (tga_x0 << 16), (tga_y0 << 16),
                             (1 << 16), 0,
                             0, (1 << 16));

        /* Render a box the size of the character (it will be filled with the texture based on the configured
         *   parameters)
         */
        d2_renderbox(d2_handle,
                     (d2_point)(lcd_x0 << 4), (d2_point)(lcd_y0 << 4),
                     (d2_width)(size_x << 4), (d2_width)(size_y << 4));

        /* Advance x */
        x = (uint16_t) (x + (*(++ch_ptr)>>8));
        ch = (uint8_t) string[++i];
        length--;
    }

    /* Reset settings */
    d2_setfillmode(d2_handle, prevfillmode);
    d2_setcolorkey(d2_handle, 0, 0x000000);
}
