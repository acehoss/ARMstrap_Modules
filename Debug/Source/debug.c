
#include "../debug.h"
#include <stm32f4xx_HAL_UART.h>
#include <sys/types.h>
#include <string.h>
#include "platform_config.h"

#define TO_HEX(i) ( (((i) & 0xf) <= 9) ? ('0' + ((i) & 0xf)) : ('A' - 10 + ((i) & 0xf)) )

char* itoa(int32_t value, char* result, int base);
char* uitoa(uint32_t value, char* result, int base);

void debug_led_set(int v) {
  if (v) {
    HAL_GPIO_WritePin(DEBUG_LED_PORT, DEBUG_LED_PIN, GPIO_PIN_SET);
  } else {
	HAL_GPIO_WritePin(DEBUG_LED_PORT, DEBUG_LED_PIN, GPIO_PIN_RESET);
  }
}

void debug_write_line(const char* str) {
  debug_write(str);
  debug_write_ch('\n');
}

void debug_write_bytes(uint8_t *data, uint16_t len) {
	HAL_UART_Transmit(DEBUG_UART_DEVICE, data, len, 1000);
	while(HAL_UART_GetState(DEBUG_UART_DEVICE) == HAL_UART_STATE_BUSY);
}

void debug_write(const char* str) {
  size_t len = strlen(str);
  debug_write_bytes((uint8_t*)str, len);
}

void debug_write_ch(char ch) {
	uint8_t* buf = (uint8_t*)&ch;
	HAL_UART_Transmit(DEBUG_UART_DEVICE, buf, 1, 1000);
  while (HAL_UART_GetState(DEBUG_UART_DEVICE) == HAL_UART_STATE_BUSY);
}

void debug_write_u8(uint8_t val, uint8_t base) {
  if (base == 16) {
    debug_write_ch(TO_HEX(val >> 4));
    debug_write_ch(TO_HEX(val >> 0));
  } else {
    char buffer[20];
    uitoa(val, buffer, base);
    debug_write(buffer);
  }
}

void debug_write_u16(uint16_t val, uint8_t base) {
  if (base == 16) {
    debug_write_ch(TO_HEX(val >> 12));
    debug_write_ch(TO_HEX(val >> 8));
    debug_write_ch(TO_HEX(val >> 4));
    debug_write_ch(TO_HEX(val >> 0));
  } else {
    char buffer[20];
    uitoa(val, buffer, base);
    debug_write(buffer);
  }
}

void debug_write_u32(uint32_t val, uint8_t base) {
  if (base == 16) {
    debug_write_ch(TO_HEX(val >> 28));
    debug_write_ch(TO_HEX(val >> 24));
    debug_write_ch(TO_HEX(val >> 20));
    debug_write_ch(TO_HEX(val >> 16));
    debug_write_ch(TO_HEX(val >> 12));
    debug_write_ch(TO_HEX(val >> 8));
    debug_write_ch(TO_HEX(val >> 4));
    debug_write_ch(TO_HEX(val >> 0));
  } else {
    char buffer[20];
    uitoa(val, buffer, base);
    debug_write(buffer);
  }
}

void debug_write_i32(int32_t val, uint8_t base) {
  char buffer[20];
  itoa(val, buffer, base);
  debug_write(buffer);
}

void debug_write_u8_array(uint8_t *p, int len) {
  for (int i = 0; i < len; i++) {
    debug_write_u8(p[i], 16);
    debug_write_ch(' ');
  }
}

//char* itoa(int32_t value, char* result, int base) {
//  // check that the base if valid
//  if (base < 2 || base > 36) {
//    *result = '\0';
//    return result;
//  }
//
//  char* ptr = result, *ptr1 = result, tmp_char;
//  int tmp_value;
//
//  do {
//    tmp_value = value;
//    value /= base;
//    *ptr++ = "zyxwvutsrqponmlkjihgfedcba9876543210123456789abcdefghijklmnopqrstuvwxyz" [35 + (tmp_value - value * base)];
//  } while (value);
//
//  // Apply negative sign
//  if (tmp_value < 0) {
//    *ptr++ = '-';
//  }
//  *ptr-- = '\0';
//  while (ptr1 < ptr) {
//    tmp_char = *ptr;
//    *ptr-- = *ptr1;
//    *ptr1++ = tmp_char;
//  }
//  return result;
//}

char* uitoa(uint32_t value, char* result, int base) {
  // check that the base if valid
  if (base < 2 || base > 36) {
    *result = '\0';
    return result;
  }

  char* ptr = result, *ptr1 = result, tmp_char;
  int tmp_value;

  do {
    tmp_value = value;
    value /= base;
    *ptr++ = "zyxwvutsrqponmlkjihgfedcba9876543210123456789abcdefghijklmnopqrstuvwxyz" [35 + (tmp_value - value * base)];
  } while (value);

  *ptr-- = '\0';
  while (ptr1 < ptr) {
    tmp_char = *ptr;
    *ptr-- = *ptr1;
    *ptr1++ = tmp_char;
  }
  return result;
}
