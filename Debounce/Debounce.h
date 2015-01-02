/*
 * Debounce.h
 *
 *  Created on: Jan 2, 2015
 *      Author: aaron
 */

#ifndef DEBOUNCE_H_
#define DEBOUNCE_H_

#include <stm32f4xx_hal_gpio.h>

#define DEBOUNCE_DEFAULT_STABLE_TICKS 15

#ifdef _cplusplus
extern "C" {
#endif

#ifndef GPIO_PIN_T
#define GPIO_PIN_T
typedef uint16_t GPIO_Pin;
#endif

typedef struct
{
	GPIO_TypeDef *Port;
	GPIO_Pin Pin;
	GPIO_PinState State;
	uint16_t Counts;
	uint16_t StableTicks;
} DebouncedPin;

void Debounce_Init(DebouncedPin *debounced_pin, GPIO_TypeDef *port, GPIO_Pin pin);
void Debounce_Init_Ticks(DebouncedPin *debounced_pin, GPIO_TypeDef *port, GPIO_Pin pin, uint16_t stable_ticks);
void Debounce_Update(DebouncedPin *debounced_pin);

#ifdef _cplusplus
}
#endif

#endif /* DEBOUNCE_H_ */
