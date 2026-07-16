#pragma once

// Central place to pick the right vendor LL GPIO header for the STM32 family
// this library is being built for. Consumers define the family macro
// (STM32G4, STM32H5, ...) via their own CMake/CubeMX project settings;
// every file in common_stm32/tmc2209/usb_pd includes this header instead
// of a family-specific stm32xxxx_ll_gpio.h directly.
#if defined(STM32G4)
#include "stm32g4xx_ll_gpio.h"
#elif defined(STM32H5)
#include "stm32h5xx_ll_gpio.h"
#else
#error "ll_header_selector.h: unknown/unsupported STM32 family - add a branch here"
#endif
