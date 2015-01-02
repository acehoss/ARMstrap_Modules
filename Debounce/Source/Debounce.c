/*
 * Debounce.c
 *
 *  Created on: Jan 2, 2015
 *      Author: aaron
 */

#include "../Debounce.h"


void Debounce_Init(DebouncedPin *debounced_pin, GPIO_TypeDef *port, GPIO_Pin pin)
{
	Debounce_Init_Ticks(debounced_pin, port, pin, DEBOUNCE_DEFAULT_STABLE_TICKS);
}

void Debounce_Init_Ticks(DebouncedPin *debounced_pin, GPIO_TypeDef *port, GPIO_Pin pin, uint16_t stable_ticks)
{
	debounced_pin->Port = port;
	debounced_pin->Pin = pin;
	debounced_pin->State = GPIO_PIN_RESET;
	debounced_pin->Counts = 0;
	debounced_pin->StableTicks = stable_ticks;
}

void Debounce_Update(DebouncedPin *debounced_pin)
{
	GPIO_PinState state = HAL_GPIO_ReadPin(debounced_pin->Port, debounced_pin->Pin);

	uint16_t stable_ticks = debounced_pin->StableTicks;
	int32_t counts = debounced_pin->Counts;

	if(state == GPIO_PIN_SET)
	{
		if(++counts > stable_ticks)
		{
			counts = stable_ticks;
			debounced_pin->State = GPIO_PIN_SET;
		}
	}
	else
	{
		if(--counts < 0)
		{
			counts = 0;
			debounced_pin->State = GPIO_PIN_RESET;
		}
	}

	debounced_pin->Counts = counts;
}
