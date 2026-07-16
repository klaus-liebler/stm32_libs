#pragma once

#include "hal_header_selector.h"
#include "ll_header_selector.h"
#include "gpio/pins.hh"


static constexpr uint32_t GPIOx_MODER = 0x00;
static constexpr uint32_t GPIOx_OTYPER = 0x04;
static constexpr uint32_t GPIOx_OSPEEDR = 0x08;
static constexpr uint32_t GPIOx_PUPDR = 0x0C;
static constexpr uint32_t GPIOx_IDR = 0x10;
static constexpr uint32_t GPIOx_ODR = 0x14;
static constexpr uint32_t GPIOx_BSRR = 0x18;
static constexpr uint32_t GPIOx_LCKR = 0x1C;
static constexpr uint32_t GPIOx_AFRL = 0x20;
static constexpr uint32_t GPIOx_AFRH = 0x24;
static constexpr uint32_t GPIOx_BRR = 0x28;
namespace gpio {


enum class Mode : uint8_t {
  Input = 0b00,
  Output = 0b01,
  AlternateFunction = 0b10,
  Analog = 0b11,
};

enum class PullDirection : uint8_t {
  NONE = 0b00,
  UP = 0b01,
  DOWN = 0b10,
};

enum class OutputType : uint8_t {
  PUSH_PULL = 0b00,
  OPEN_DRAIN = 0b01,
};

enum class OutputSpeed : uint8_t {
  LOW = 0b00,
  MEDIUM = 0b01,
  HIGH = 0b10,
  VERY_HIGH = 0b11,
};

class Gpio {
private:
  inline static uint8_t pin2port(Pin pin) { return ((uint8_t)pin) >> 4; }

  inline static uint8_t pin2localPin(Pin pin) { return ((uint8_t)pin) & 0x0F; }

  inline static uint32_t pin2GPIOx_BASE(Pin pin) {
    return (GPIOA_BASE + (GPIOB_BASE - GPIOA_BASE) * (pin2port(pin)));
  }

  static void Modify2Bits(uint32_t *reg, uint8_t indexInRegister,
                          uint32_t val) {
    indexInRegister &= 0xF;
    indexInRegister *= 2;
    uint32_t clearer = ~(0x3 << indexInRegister);
    __IO uint32_t regVal = *reg;
    regVal &= clearer;
    uint32_t setter = (val & 0x3) << (indexInRegister);
    regVal |= setter;
    *reg = regVal;
  }

  static void Modify4Bits(uint32_t *reg, uint8_t indexInRegister,
                          uint32_t val) {
    indexInRegister &= 0x7;
    indexInRegister *= 4;
    uint32_t clearer = ~(0xF << indexInRegister);
    __IO uint32_t regVal = *reg;
    regVal &= clearer;
    uint32_t setter = (val & 0xF) << (indexInRegister);
    regVal |= setter;
    *reg = regVal;
  }

  inline static void EnableGPIOClock(Pin pin) {
    uint8_t portIndex = pin2port(pin);
    uint32_t clockEnableBit = RCC_AHB2ENR_GPIOAEN << portIndex;
    SET_BIT(RCC->AHB2ENR, clockEnableBit);
  }

  inline static uint32_t pin2AFRxOffset(Pin pin) {
    return GPIOx_AFRL + 4 * ((((uint32_t)pin) & 0x08) >> 3);
  }

public:
  static void ConfigureGPIOInput(Pin pin,
                                 PullDirection pulldir = PullDirection::NONE) {
    if (pin == Pin::NO_PIN)
      return;
    EnableGPIOClock(pin);
    uint8_t pinIndex = pin2localPin(pin);
    uint32_t GPIOx_BASE = pin2GPIOx_BASE(pin);
    Modify2Bits((uint32_t *)(GPIOx_BASE + GPIOx_MODER), pinIndex,
                (uint32_t)Mode::Input);
    Modify2Bits((uint32_t *)(GPIOx_BASE + GPIOx_PUPDR), pinIndex,
                (uint32_t)pulldir);
  }

  static void ConfigureGPIOOutput(Pin pin, bool initialValue,
                                  OutputType ot = OutputType::PUSH_PULL,
                                  OutputSpeed speed = OutputSpeed::LOW,
                                  PullDirection pulldir = PullDirection::NONE) {
    if (pin == Pin::NO_PIN)
      return;
    EnableGPIOClock(pin);
    uint8_t pinIndex = pin2localPin(pin);
    uint32_t GPIOx_BASE = pin2GPIOx_BASE(pin);
    Modify2Bits((uint32_t *)(GPIOx_BASE + GPIOx_OTYPER), pinIndex,
                (uint32_t)ot);
    Modify2Bits((uint32_t *)(GPIOx_BASE + GPIOx_OSPEEDR), pinIndex,
                (uint32_t)speed);
    Modify2Bits((uint32_t *)(GPIOx_BASE + GPIOx_PUPDR), pinIndex,
                (uint32_t)pulldir);
    Modify2Bits((uint32_t *)(GPIOx_BASE + GPIOx_MODER), pinIndex,
                (uint32_t)Mode::Output);
    Set(pin, initialValue);
  }

  static void
  ConfigureAlternateFunction(Pin pin, uint32_t GPIO_AFx_yyyy,
                             OutputType ot = OutputType::PUSH_PULL,
                             OutputSpeed speed = OutputSpeed::LOW,
                             PullDirection pulldir = PullDirection::NONE) {
    if (pin == Pin::NO_PIN)
      return;
    EnableGPIOClock(pin);
    uint8_t pinIndex = pin2localPin(pin);
    uint32_t GPIOx_BASE = pin2GPIOx_BASE(pin);
    Modify2Bits((uint32_t *)(GPIOx_BASE + GPIOx_MODER), pinIndex,
                (uint32_t)Mode::AlternateFunction);
    Modify2Bits((uint32_t *)(GPIOx_BASE + GPIOx_OTYPER), pinIndex,
                (uint32_t)ot);
    Modify2Bits((uint32_t *)(GPIOx_BASE + GPIOx_OSPEEDR), pinIndex,
                (uint32_t)speed);
    Modify2Bits((uint32_t *)(GPIOx_BASE + GPIOx_PUPDR), pinIndex,
                (uint32_t)pulldir);
    volatile uint32_t offset = pin2AFRxOffset(pin);
    Modify4Bits((uint32_t *)(GPIOx_BASE + offset), pinIndex, GPIO_AFx_yyyy);
  }

  static void ConfigureAnalog(Pin pin) {
    if (pin == Pin::NO_PIN)
      return;
    EnableGPIOClock(pin);
    uint8_t pinIndex = pin2localPin(pin);
    uint32_t GPIOx_BASE = pin2GPIOx_BASE(pin);
    Modify2Bits((uint32_t *)(GPIOx_BASE + GPIOx_MODER), pinIndex,
                (uint32_t)Mode::Analog);
  }

  static void Set(Pin pin, bool value) {
    if (pin == Pin::NO_PIN)
      return;
    uint8_t pinIndex = pin2localPin(pin);
    uint32_t GPIOx_BASE = pin2GPIOx_BASE(pin);
    uint32_t val = 1 << (pinIndex + 16 * !value);
    *((uint32_t *)(GPIOx_BASE + GPIOx_BSRR)) = val;
  }

  static bool Get(Pin pin) {
    if (pin == Pin::NO_PIN)
      return false;
    uint8_t pinIndex = pin2localPin(pin);
    uint32_t GPIOx_BASE = pin2GPIOx_BASE(pin);
    return *((uint32_t *)(GPIOx_BASE + GPIOx_IDR)) & (1 << pinIndex);
  }

  static void Toggle(Pin pin) {
    if (pin == Pin::NO_PIN)
      return;
    uint8_t pinIndex = pin2localPin(pin);
    uint32_t GPIOx_BASE = pin2GPIOx_BASE(pin);
    uint32_t regVal = *((uint32_t *)(GPIOx_BASE + GPIOx_ODR));
    *((uint32_t *)(GPIOx_BASE + GPIOx_BSRR)) =
        (1 << (pinIndex + (regVal & (1 << pinIndex) ? 16 : 0)));
  }
};
}