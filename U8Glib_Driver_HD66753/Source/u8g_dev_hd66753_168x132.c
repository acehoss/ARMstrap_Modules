/*

  u8g_dev_hd66753_168x132.c

  * Portions from U8Glib
  * Portions from TI MSP430 Eval HAL
  * Portions from Charles Armstrap
  * Portions from Aaron Heise
  *
  ****LICENSE DISCLOSURES****

  Universal 8bit Graphics Library
  
  Copyright (c) 2011, olikraus@gmail.com
  All rights reserved.

  Redistribution and use in source and binary forms, with or without modification, 
  are permitted provided that the following conditions are met:

  * Redistributions of source code must retain the above copyright notice, this list 
    of conditions and the following disclaimer.
    
  * Redistributions in binary form must reproduce the above copyright notice, this 
    list of conditions and the following disclaimer in the documentation and/or other 
    materials provided with the distribution.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND 
  CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, 
  INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF 
  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE 
  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR 
  CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT 
  NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; 
  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER 
  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, 
  STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF 
  ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.  
  
 *
 *  Copyright (C) 2010 Texas Instruments Incorporated - http://www.ti.com/
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  
*/

#include "u8g.h"

/* Aaron Heise 18 SEP 2014
 * This controller can accommodate screens up to 168x132. However, it often is used with screens
 * with smaller dimensions. In the case of my test screen, the size was 138x110. The following few
 * lines set how the driver will accommodate those varying sizes.
 *
 * The controller doesn't care if it is attached to a smaller screen, it still has the same amount of
 * on-board RAM and the address counter is incremented as if it were attached to a full screen.
 * By default, DRAW_FULL_CGRAM_SIZE is defined and U8G will render the full 168x132 to the screen.
 *
 * In many cases, as was the case with my 138x110 screen, this actually results in the same or better
 * performance (frame rate) as sending only the frames for my particular screen.
 *
 * This is because to reset the device's address counter to the beginning of the next row, an overhead
 * of 10 bytes per row is incurred to write out the address and restart the pixel transfer. At 2 bits
 * per pixel, 10 bytes per row takes the same amount of space as 40 additional pixels. Depending on
 * the dimensions of the screen, the difference in rows may make up for this entirely.
 *
 * CASE: 138x110
 * (138 pixels / row) * (2 bits / 1 pixel) * (1 byte / 8 bits) = 34.5 bytes/row
 * Then, because bytes are written to CGRAM in pairs:
 *    34.5 bytes/row => 36 bytes/row
 * Then, because each row incurs a 10 byte overhead:
 *    36 bytes/row + 10 bytes/row = 46 bytes/row
 * For 110 rows:
 * (46 bytes / row) * (110 rows / 1 frame) = 5060 bytes/frame
 *
 * - Compared to -
 *
 * CASE: 168x132
 * (168 pixels / row) * (2 bits / 1 pixel) * (1 byte / 8 bits) = 42 bytes/row
 * No overhead for writing in pairs:
 *    42 bytes/row
 * No per row overhead (there *is* a 10 byte overhead at the beginning of the write):
 *    42 bytes/row
 * For 132 rows:
 * (42 bytes / row) * (132 rows / 1 screen)  = 5544 bytes/frame
 * Plus 10 byte overhead:
 *    5544 bytes/frame + 10 bytes/screen = 5554 bytes/frame
 *
 *
 * SO...I didn't demonstrate that writing out the whole screen is necessarily faster. On the bench, using
 * a very basic frame rate test program, the performance of each was surprisingly similar (both setups
 * ranged between 68 and 74 fps across a 1 second average), and the math demonstrates how similar they
 * are. Where it would seem by the amount of pixel data to be transferred that matching the dimensions of
 * the screen would result in a frame transfer size that was nearly 30% less than sending the full CGRAM
 * size, it actually results in a reduction of less than 10%.
 *
 * That said, there are other reasons. Rendering a smaller area means less memory usage. If memory is at
 * a premium, constraining the rendering window will reduce the memory usage.
 */


//Uncomment the following line to disable resetting address at the beginning of each line
//Per above description, saves 10 bytes per row.
//#define DRAW_FULL_CGRAM_SIZE

//this is essentially a divisor on the amount RAM needed for a video buffer
//Define DRAW_FULL_CGRAM_SIZE and NUM_PAGES = 1 will result in 5544 bytes of RAM used.
//A smaller display area or additional pages will reduce memory consumption
//make this a number that divides HEIGHT
#define NUM_PAGES 1

#ifdef DRAW_FULL_CGRAM_SIZE
#define WIDTH 168
#define HEIGHT 132
#else
//make width the next largest integer divisible by 8
#define WIDTH 144
#define HEIGHT 110
#endif

#define PAGE_HEIGHT (HEIGHT / NUM_PAGES)
#define BPP 2
#define PIXELS_PER_BLOCK 8
#define MEM_SIZE (WIDTH * PAGE_HEIGHT * BPP /8)
#define BLOCK_MEM_SIZE (PIXELS_PER_BLOCK * BPP / 8)
#define WIDTH_BLOCKS (WIDTH / PIXELS_PER_BLOCK)

#define BIT0 0x01
#define BIT1 0x02
#define BIT2 0x04
#define BIT3 0x08


//static unsigned char Read_Block_Address_Macro[] = {0x74, 0x00, 0x12, 0x77, 0x00, 0x00};
static unsigned char Draw_Block_Value_Macro[] = {0x74, 0x00, 0x12, 0x76, 0xFF, 0xFF};
static unsigned char Draw_Block_Address_Macro[] = {0x74, 0x00, 0x11, 0x76, 0x00, 0x00};


static uint8_t LcdInitMacro[] = {

  0x74, 0x00, 0x00, 0x76, 0x00, 0x01,            // R00 start oscillation
  0x74, 0x00, 0x01, 0x76, 0x00, 0x0D,            // R01 driver output control
  0x74, 0x00, 0x02, 0x76, 0x00, 0x4C,            // R02 LCD - driving waveform control
  0x74, 0x00, 0x03, 0x76, 0x12, 0x14,            // R03 Power control
  0x74, 0x00, 0x04, 0x76, 0x04, 0x66,            // R04 Contrast control
  0x74, 0x00, 0x05, 0x76, 0x00, 0x10,            // R05 Entry mode
  0x74, 0x00, 0x06, 0x76, 0x00, 0x00,            // R06 Rotation
  0x74, 0x00, 0x07, 0x76, 0x00, 0x15,            // R07 Display control
  0x74, 0x00, 0x08, 0x76, 0x00, 0x03,            // R08 Cursor Control
  0x74, 0x00, 0x09, 0x76, 0x00, 0x00,            // R09 NOOP
  0x74, 0x00, 0x0A, 0x76, 0x00, 0x00,            // R0A NOOP
  0x74, 0x00, 0x0B, 0x76, 0x00, 0x03,            // R0B Horizontal Cursor Position
  0x74, 0x00, 0x0C, 0x76, 0x00, 0x03,            // R0C Vertical Cursor Position
  0x74, 0x00, 0x0D, 0x76, 0x00, 0x00,            // R0D 1st Driving Position
  0x74, 0x00, 0x0E, 0x76, 0x00, 0x15,            // R0E 2nd Driving Position
  0x74, 0x00, 0x0F, 0x76, 0x00, 0x03,            // R0F
  0x74, 0x00, 0x10, 0x76, 0x00, 0x00,            // R10 RAM write mask
  0x74, 0x00, 0x11, 0x76, 0x00, 0x00,            // R11 RAM Address
};


static void LcdWriteCommand(u8g_t* u8g, u8g_dev_t* dev, uint8_t* cmd)
{
    u8g_SetChipSelect(u8g, dev, 1);
 //there seems to be a bug with write sequence in my device driver.
    u8g_WriteByte(u8g,dev, cmd[0]);
    u8g_WriteByte(u8g,dev, cmd[1]);
    u8g_WriteByte(u8g,dev, cmd[2]);
//    u8g_WriteSequence(u8g, dev, 3, cmd);
    u8g_SetChipSelect(u8g, dev, 0);
    //u8g_Delay(10);
    u8g_SetChipSelect(u8g,dev,1);
    u8g_WriteByte(u8g,dev, cmd[3]);
    u8g_WriteByte(u8g,dev, cmd[4]);
    u8g_WriteByte(u8g,dev, cmd[5]);
//    u8g_WriteSequence(u8g,dev,3, &cmd[3]);
    u8g_SetChipSelect(u8g,dev,0);
    //u8g_Delay(10);
}

static void LcdSetAddress(u8g_t* u8g, u8g_dev_t* dev, uint16_t Address)
{
    Draw_Block_Address_Macro[4] = Address >> 8;
    Draw_Block_Address_Macro[5] = Address & 0xFF;
    LcdWriteCommand(u8g, dev, Draw_Block_Address_Macro);
}

static void LcdDrawBlock(u8g_t* u8g, u8g_dev_t* dev, uint8_t row, uint8_t block, uint8_t* buf)
{
	if(block == 0)
	{
		uint16_t addr = (uint16_t)row * (uint16_t)0x0020 + (uint16_t)block;
		LcdSetAddress(u8g, dev, addr);
	}

	uint16_t block_index = (row*WIDTH_BLOCKS*BLOCK_MEM_SIZE) + block*BLOCK_MEM_SIZE;
    Draw_Block_Value_Macro[4] = buf[block_index+1];
    Draw_Block_Value_Macro[5] = buf[block_index];

    LcdWriteCommand(u8g, dev, Draw_Block_Value_Macro);
}

void LcdSetContrast(u8g_t* u8g, u8g_dev_t* dev, uint8_t ContrastLevel)
{
    if (ContrastLevel > 127) ContrastLevel = 127;
    if (ContrastLevel < 70) ContrastLevel = 70;
    LcdInitMacro[0x04 * 6 + 5] = ContrastLevel;
    LcdWriteCommand(u8g, dev, &LcdInitMacro[0x04 * 6]);
}

uint8_t u8g_dev_hd66753_168x132_fn(u8g_t *u8g, u8g_dev_t *dev, uint8_t msg, void *arg)
{
  switch(msg)
  {
    case U8G_DEV_MSG_INIT:
      u8g_InitCom(u8g, dev, U8G_SPI_CLK_CYCLE_NONE);

      u8g_SetResetHigh(u8g,dev);
      u8g_Delay(20);
      u8g_SetResetLow(u8g, dev);
      u8g_Delay(20);
      u8g_SetResetHigh(u8g,dev);
      u8g_Delay(100);

      LcdWriteCommand(u8g, dev, LcdInitMacro);         // R00 start oscillation

      // Wait a minimum of 25ms after issuing "start oscillation"
      // command (to accomodate for MCLK up to 25MHz)
      u8g_Delay(100);

      LcdInitMacro[3 * 6 + 5] |= BIT3;
      LcdInitMacro[3 * 6 + 5] &= ~BIT0;
      LcdWriteCommand(u8g, dev, &LcdInitMacro[3 * 6]);

      // LCD Initialization Routine Using Predefined Macros
      LcdWriteCommand(u8g, dev, &LcdInitMacro[1 * 6]);
      LcdWriteCommand(u8g, dev, &LcdInitMacro[2 * 6]);
      LcdWriteCommand(u8g, dev, &LcdInitMacro[4 * 6]);
      LcdWriteCommand(u8g, dev, &LcdInitMacro[5 * 6]);
      LcdWriteCommand(u8g, dev, &LcdInitMacro[6 * 6]);
      LcdWriteCommand(u8g, dev, &LcdInitMacro[7 * 6]);
      LcdWriteCommand(u8g, dev, &LcdInitMacro[16 * 6]);
      LcdWriteCommand(u8g, dev, &LcdInitMacro[17 * 6]);

      LcdSetContrast(u8g, dev, 110);

      LcdSetAddress(u8g, dev, 0);

      // Clear screen
      u8g_SetChipSelect(u8g, dev, 1);
      u8g_WriteByte(u8g, dev, Draw_Block_Value_Macro[0]);
      u8g_WriteByte(u8g, dev, Draw_Block_Value_Macro[1]);
      u8g_WriteByte(u8g, dev, Draw_Block_Value_Macro[2]);
      u8g_SetChipSelect(u8g, dev, 0);

      u8g_SetChipSelect(u8g,dev,1);
      u8g_WriteByte(u8g, dev, Draw_Block_Value_Macro[3]);
      for(uint16_t i = 0; i < 5544; i++)
      {
    	  u8g_WriteByte(u8g, dev, 0);
      }
      u8g_SetChipSelect(u8g, dev, 0);

      break;
    case U8G_DEV_MSG_STOP:
      break;
    case U8G_DEV_MSG_PAGE_NEXT:
      {
        uint8_t row_after_last_row, row, block;
        uint8_t *ptr;
        u8g_pb_t *pb = (u8g_pb_t *)(dev->dev_mem);

        row_after_last_row = pb->p.page_y1;
        ptr = pb->buf;

#ifdef DRAW_FULL_CGRAM_SIZE
        LcdSetAddress(u8g, dev, pb->p.page_y0 * 0x0020);

        u8g_SetChipSelect(u8g, dev, 1);
        u8g_WriteByte(u8g, dev, Draw_Block_Value_Macro[0]);
        u8g_WriteByte(u8g, dev, Draw_Block_Value_Macro[1]);
        u8g_WriteByte(u8g, dev, Draw_Block_Value_Macro[2]);
        u8g_SetChipSelect(u8g, dev, 0);

        u8g_SetChipSelect(u8g,dev,1);
        u8g_WriteByte(u8g, dev, Draw_Block_Value_Macro[3]);
#endif

        for( row = 0; row < pb->p.page_height; row++)
        {
        	uint16_t row_offset = row*WIDTH_BLOCKS*BLOCK_MEM_SIZE;

#ifndef DRAW_FULL_CGRAM_SIZE
        	uint16_t row_addr = (pb->p.page_y0 + row)*0x0020;
            LcdSetAddress(u8g, dev, row_addr);

            u8g_SetChipSelect(u8g, dev, 1);
            u8g_WriteByte(u8g, dev, Draw_Block_Value_Macro[0]);
            u8g_WriteByte(u8g, dev, Draw_Block_Value_Macro[1]);
            u8g_WriteByte(u8g, dev, Draw_Block_Value_Macro[2]);
            u8g_SetChipSelect(u8g, dev, 0);

            u8g_SetChipSelect(u8g,dev,1);
            u8g_WriteByte(u8g, dev, Draw_Block_Value_Macro[3]);
#endif

            //I didn't see this in the data sheet, but evidently you can keep writing blocks to the display (docs appear to say only one at a time)
        	//as a result, the set up to write happens before the row loop, and the entire screen is sent down the wire.
            for( block = 0; block < WIDTH_BLOCKS; block++ )
            {
            	uint16_t block_offset = row_offset+block*BLOCK_MEM_SIZE;
            	//block bytes must be reversed before sent out.
            	u8g_WriteByte(u8g, dev, ptr[block_offset+1]);
            	u8g_WriteByte(u8g, dev, ptr[block_offset]);
            }
#ifndef DRAW_FULL_CGRAM_SIZE
            u8g_SetChipSelect(u8g ,dev , 0);
#endif
        }

#ifdef DRAW_FULL_CGRAM_SIZE
            u8g_SetChipSelect(u8g , dev , 0);
#endif

      }
      break;
  }
  return u8g_dev_pbxh2_base_fn(u8g, dev, msg, arg);
}


//U8G_PB_DEV(u8g_dev_hd66753_168x132_sw_spi, WIDTH, HEIGHT, HEIGHT, u8g_dev_hd66753_168x132_fn, U8G_COM_SW_SPI);
uint8_t u8g_dev_hd66753_168x132_sw_spi_buf[MEM_SIZE] U8G_NOCOMMON;
u8g_pb_t u8g_dev_hd66753_168x132_sw_spi_pb = { {PAGE_HEIGHT, HEIGHT, 0, 0, 0},  WIDTH, u8g_dev_hd66753_168x132_sw_spi_buf};
u8g_dev_t u8g_dev_hd66753_168x132_sw_spi = { u8g_dev_hd66753_168x132_fn, &u8g_dev_hd66753_168x132_sw_spi_pb, U8G_COM_SW_SPI };

//U8G_PB_DEV(u8g_dev_hd66753_168x132_hw_spi, WIDTH, HEIGHT, HEIGHT, u8g_dev_hd66753_168x132_fn, U8G_COM_HW_SPI);
uint8_t u8g_dev_hd66753_168x132_hw_spi_buf[MEM_SIZE] U8G_NOCOMMON ;
u8g_pb_t u8g_dev_hd66753_168x132_hw_spi_pb = { {PAGE_HEIGHT, HEIGHT, 0, 0, 0},  WIDTH, u8g_dev_hd66753_168x132_hw_spi_buf};
u8g_dev_t u8g_dev_hd66753_168x132_hw_spi = { u8g_dev_hd66753_168x132_fn, &u8g_dev_hd66753_168x132_hw_spi_pb, U8G_COM_HW_SPI };
