/*
 * U8G_Display_Test.c
 *
 *  Created on: Jan 2, 2015
 *      Author: aaron
 */

#include <U8G_Display_Test/U8G_Display_Test.h>
#include <Utils/itoa.h>
#include <stm32f4xx_hal_rcc.h>

static volatile uint32_t gSecondCounter = 0;
static volatile uint32_t gFrameCounter = 0;
static volatile uint32_t gLastFrameCount = 0;

static char gFrameCountStr[20];
static char gPageCountStr[30];

void draw(u8g_t* u8g, uint8_t pos, uint8_t first_page);

void U8G_Display_Test_Draw(u8g_t* u8g)
{
	/* counter for display animation */
	static uint8_t pos = 0;
	/* holds the number of frames between loops */
	static uint8_t pageCount = 0;
	/* counter for pages */
	uint8_t pageCounter = 0;
	/* temp variable for building a display string */
	char pageCountTempStr[10];

	/* build string for pages per frame and MPU speed */
	itoa(pageCount, pageCountTempStr, 10);
	strcat(pageCountTempStr, " ppf @ ");
	strcpy(gPageCountStr, pageCountTempStr);
	itoa(SystemCoreClock/1000000, pageCountTempStr, 10);
	strcat(pageCountTempStr, " MHz");
	strcat(gPageCountStr, pageCountTempStr);

	/* build string for frames per second */
	itoa(gLastFrameCount, gFrameCountStr, 10);
	strcat(gFrameCountStr, " fps");

	uint16_t fc_scale = gLastFrameCount / 8;
	uint16_t pos_scale = fc_scale < 1 ? 1 : fc_scale;

	/* Picture Loop */
	/* To conserve memory U8G renders the screen in pages.
	 * For each page, the entire screen is rendered but clipped to the bounds of
	 * the page. In the case of the NHD-2.7-12864 display, there are either 8 or
	 * 16 pages depending on the U8G device constructor used. U8G does not
	 * support single-page rendering out of the box, but it can with a few
	 * relatively simple tweaks. This yields a noticeable performance gain. */
	u8g_FirstPage(u8g);
	do
	{
		//the first parameter of draw is used to cycle the display through various positions.
		//it is primarily used to cycle the spinner through four positions. The clock freq index is
		//used as a simple prescaler to keep the spinner going similar speeds across the various clock
		//speeds. There is an additional /2 prescaler used here to slow it down a bit more.
		//The second parameter is a first page indicator, to allow counters and such inside the draw
		//function to incremented only on the first page.
		draw(u8g, pos / pos_scale, pageCounter == 0);
		pageCounter++;
	} while ( u8g_NextPage(u8g) );

	/* manage counters */
	pageCount = pageCounter;
	gFrameCounter++;
	pos = (pos+1) % (4*pos_scale);
}

void draw(u8g_t* u8g, uint8_t pos, uint8_t first_page)
{
	/* Draw the pages per frame string */
	u8g_SetColorIndex(u8g, 2);
	u8g_SetFont(u8g, u8g_font_6x10);
	u8g_DrawStr(u8g,  0, 10, gPageCountStr);

	/* Draw the frames per second string */
	u8g_SetColorIndex(u8g, 3);
	u8g_SetFont(u8g, u8g_font_timR12);

	if(gLastFrameCount == 0)
	{
		u8g_DrawStr(u8g,  0, 30, "Reading fps...");
	}
	else
	{
		u8g_DrawStr(u8g,  0, 30, gFrameCountStr);

		/* Draw the spinner */
		u8g_SetColorIndex(u8g, 3);
		u8g_SetFont(u8g, u8g_font_10x20);
		u8g_DrawStr(u8g, 60, 60, pos == 0 ? "|" : pos == 1 ? "/" : pos == 2 ? "-" : "\\");
	}

	/* Draw the hello world message */
	static int16_t hello_pos = 168;
	if(first_page)
	{
		hello_pos--;
		if(hello_pos <= -168)
			hello_pos = 168;
	}
	u8g_SetColorIndex(u8g, 3);
	u8g_SetFont(u8g, u8g_font_ncenR14);
	u8g_DrawStr(u8g, hello_pos, 100, "Hello World!");
}

void U8G_Display_Test_Systick(const uint32_t ticks_per_second)
{
	if(++gSecondCounter >= ticks_per_second)
	{
		gSecondCounter = 0;
		gLastFrameCount = gFrameCounter;
		gFrameCounter = 0;
	}
}
