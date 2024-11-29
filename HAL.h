#ifndef HAL_H
#define HAL_H


#if defined(__AVR__)

#include "src/Boards/AVR/HAL.h"

#elif defined(__STM32F0__) || defined(STM32F0xx) || \
		defined(__STM32F1__) || defined(STM32F1xx) || \
		defined(__STM32F2__) || defined(STM32F2xx) || \
		defined(__STM32F3__) || defined(STM32F3xx) || \
		defined(__STM32F4__) || defined(STM32F4xx)

#include "src/Boards/STM32/HAL.h"

#else
#error "no match MCU defined"
#endif

#endif // HAL_H
