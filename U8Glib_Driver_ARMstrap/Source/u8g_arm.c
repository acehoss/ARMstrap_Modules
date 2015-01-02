/*

  u8g_arm.c


  u8g utility procedures for LPC122x

  Universal 8bit Graphics Library

  Copyright (c) 2013, olikraus@gmail.com
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


  The following delay procedures must be implemented for u8glib. This is done in this file:

  void u8g_Delay(uint16_t val)		Delay by "val" milliseconds
  void u8g_MicroDelay(void)		Delay be one microsecond
  void u8g_10MicroDelay(void)	Delay by 10 microseconds

  Additional requirements:

      SysTick must be enabled, but SysTick IRQ is not required. Any LOAD values are fine,
      it is prefered to have at least 1ms
      Example:
        SysTick->LOAD = (SystemCoreClock/1000UL*(unsigned long)SYS_TICK_PERIOD_IN_MS) - 1;
	SysTick->VAL = 0;
	SysTick->CTRL = 7;   // enable, generate interrupt (SysTick_Handler), do not divide by 2
 */

#include "../u8g_arm.h"

static SPI_HandleTypeDef* spi_device = 0;
static GPIO_TypeDef *cs_port = 0;
static uint16_t cs_pin = 0;
static GPIO_TypeDef *dc_port = 0;
static uint16_t dc_pin = 0;
static GPIO_TypeDef *rs_port = 0;
static uint16_t rs_pin = 0;

void u8g_com_hw_spi_set_parameters(SPI_HandleTypeDef *_spi_device, GPIO_TypeDef *_cs_port, uint16_t _cs_pin, GPIO_TypeDef *_dc_port, uint16_t _dc_pin, GPIO_TypeDef *_rs_port, uint16_t _rs_pin)
{
	spi_device = _spi_device;
	cs_port = _cs_port;
	cs_pin = _cs_pin;
	dc_port = _dc_port;
	dc_pin = _dc_pin;
	rs_port = _rs_port;
	rs_pin = _rs_pin;
}

/*========================================================================*/
/* Generic ARM delay procedure, based on the system timer (SysTick) */

/*
  Delay by the provided number of system ticks.
  The delay must be smaller than the RELOAD value.
  This delay has an imprecision of about +/- 20 system ticks.   
 */
static void _delay_system_ticks_sub(uint32_t sys_ticks)
{
	uint32_t start_val, end_val, curr_val;
	uint32_t load;

	start_val = SysTick->VAL;
	start_val &= 0x0ffffffUL;
	end_val = start_val;

	if ( end_val < sys_ticks )
	{
		/* check, if the operation after this if clause would lead to a negative result */
		/* if this would be the case, then add the reload value first */
		load = SysTick->LOAD;
		load &= 0x0ffffffUL;
		end_val += load;
	}
	/* counter goes towards zero, so end_val is below start value */
	end_val -= sys_ticks;


	/* wait until interval is left */
	if ( start_val >= end_val )
	{
		for(;;)
		{
			curr_val = SysTick->VAL;
			curr_val &= 0x0ffffffUL;
			if ( curr_val <= end_val )
				break;
			if ( curr_val > start_val )
				break;
		}
	}
	else
	{
		for(;;)
		{
			curr_val = SysTick->VAL;
			curr_val &= 0x0ffffffUL;
			if ( curr_val <= end_val && curr_val > start_val )
				break;
		}
	}
}

/*
  Delay by the provided number of system ticks.
  Any values between 0 and 0x0ffffffff are allowed.
 */
void delay_system_ticks(uint32_t sys_ticks)
{
	uint32_t load4;
	load4 = SysTick->LOAD;
	load4 &= 0x0ffffffUL;
	load4 >>= 2;

	while ( sys_ticks > load4 )
	{
		sys_ticks -= load4;
		_delay_system_ticks_sub(load4);
	}
	_delay_system_ticks_sub(sys_ticks);
}

/*
  Delay by the provided number of micro seconds.
  Limitation: "us" * System-Freq in MHz must now overflow in 32 bit.
  Values between 0 and 1.000.000 (1 second) are ok.
 */
void delay_micro_seconds(uint32_t us)
{
	uint32_t sys_ticks;

	sys_ticks = SystemCoreClock;
	sys_ticks /=1000000UL;
	sys_ticks *= us;
	delay_system_ticks(sys_ticks);
}



/*========================================================================*/
/*
  The following delay procedures must be implemented for u8glib

  void u8g_Delay(uint16_t val)		Delay by "val" milliseconds
  void u8g_MicroDelay(void)		Delay be one microsecond
  void u8g_10MicroDelay(void)	Delay by 10 microseconds

 */

void u8g_Delay(uint16_t val)
{

	delay_micro_seconds(1000UL*(uint32_t)val);
}

void u8g_MicroDelay(void)
{
	delay_micro_seconds(1);
}

void u8g_10MicroDelay(void)
{
	delay_micro_seconds(10);
}

/* Sends a sequence of bytes from the specified SPI interface */
void screen_tx_seq(SPI_HandleTypeDef* spi, uint8_t* buf, const uint16_t len, const uint8_t command)
{
//	for(uint16_t i = 0; i < len; i++)
//	{
		while(HAL_SPI_GetState(spi) == HAL_SPI_STATE_BUSY);
		HAL_SPI_Transmit(spi, buf, len,1000);
//	}
}

/* Sends a byte from the specified SPI interface */
void screen_tx(SPI_HandleTypeDef* spi, uint8_t byte, const uint8_t command)
{
	screen_tx_seq(spi, &byte, 1, command);
}


/*========================================================================*/
/* u8glib com procedure */

uint8_t u8g_com_hw_spi_fn(u8g_t *u8g, uint8_t msg, uint8_t arg_val, void *arg_ptr)
{
	/* the last set state of the data/command pin */
	static uint8_t command = 0;

	/* make sure device is initialized */
	if(spi_device == 0)
		return 0;

	switch(msg)
	{
	case U8G_COM_MSG_STOP:
		break;

	case U8G_COM_MSG_INIT:
		/* SPI initialization should be done before U8G is initialized */
		break;

	case U8G_COM_MSG_ADDRESS:                     /* define cmd (arg_val = 0) or data mode (arg_val = 1) */
		/* if the requested state is different from the current state */
		if(arg_val != command)
		{
			/* Wait until the transmission is complete before changing state */
			while(HAL_SPI_GetState(spi_device) == HAL_SPI_STATE_BUSY);
			command = arg_val;
		}
		HAL_GPIO_WritePin(dc_port, dc_pin, command);
		break;

	case U8G_COM_MSG_CHIP_SELECT:
		/* Wait until the transmission is complete before changing the chip select state */
		while(HAL_SPI_GetState(spi_device) == HAL_SPI_STATE_BUSY);
		HAL_GPIO_WritePin(cs_port, cs_pin, !(arg_val > 0));
		break;

	case U8G_COM_MSG_RESET:
		HAL_GPIO_WritePin(rs_port, rs_pin, arg_val);
		u8g_10MicroDelay();
		break;

	case U8G_COM_MSG_WRITE_BYTE:
		screen_tx(spi_device, arg_val, command);
		break;

	case U8G_COM_MSG_WRITE_SEQ:
	case U8G_COM_MSG_WRITE_SEQ_P:
	{
		//uint8_t *ptr = arg_ptr;
		screen_tx_seq(spi_device, arg_ptr, arg_val, command);
	}
	break;
	}
	return 1;
}

