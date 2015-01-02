/*
 * U8G_Display_Test.h
 *
 *  Created on: Jan 2, 2015
 *      Author: aaron
 */

#ifndef U8G_DISPLAY_TEST_H_
#define U8G_DISPLAY_TEST_H_

#include <u8g.h>
#include <stm32f4xx_hal_rcc.h>
#include <string.h>

#ifdef _cplusplus
extern "C" {
#endif

void U8G_Display_Test_Draw(u8g_t* u8g);
void U8G_Display_Test_Systick(const uint32_t ticks_per_second);

#ifdef _cplusplus
}
#endif

#endif /* U8G_DISPLAY_TEST_H_ */
