#pragma once
#include <cstdint>

namespace gpio {

enum class Pin : uint8_t {
  #include "pins_gen.inc"
  NO_PIN = UINT8_MAX
};
}