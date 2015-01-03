# ARMstrap_Modules
These are some reusable modules for the ARMstrap CubeMX Template project structure. They are generally
compatible with STM32F4xx Cube-based (HAL) projects.
(See [https://github.com/hossboss/ARMstrap_CubeMX_Template](https://github.com/hossboss/ARMstrap_CubeMX_Template)).

## Modules
* **Debounce** - Integrator debouncer
* **Debug** - Functions for printing debug information to a UART. Works out of the box with the ARMstrap
  FTDI port if used with the default platform\_config.h 
* **U8Glib** - Version 1.15 (ARM) of the [U8G graphics library](https://code.google.com/p/u8glib/).
* **U8Glib_Driver_ARMstrap** - U8G comm driver for the ARMstrap board
* **U8Glib_Driver_HD66753** - U8G device driver for the HD66753 LCD controller
* **Utils** - Various utility functions

## Using Modules
To use the modules, copy the directory containing the module you want to use into the *Modules* directory
in the project. Then `#include <module_name/module_header.h>` wherever you want to use the functionality.
