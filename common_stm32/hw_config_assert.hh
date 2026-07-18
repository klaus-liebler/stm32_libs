#pragma once
// HW_CONFIG_ASSERT prueft Hardware-/Peripherie-Konfigurationsvoraussetzungen (verlinktes DMA,
// aktivierter NVIC-Interrupt etc.), die CubeMX bei einer falschen .ioc-Einstellung oder beim
// naechsten "Generate Code" leise vergisst -- und die sich sonst erst beim naechsten Aufruf
// zeigen wuerden, oft nur als kryptischer TX/RX-Timeout (TMC2209-UART) oder sogar als
// HardFault statt eines erkennbaren HAL_ERROR (siehe ws2812.hpp, ShowEach()). Anders als
// HAL_ERROR_CHECK (errorcheck.hh, prueft Rueckgabewerte einzelner HAL-Aufrufe) geht es hier um
// Konfigurationsvoraussetzungen, die VOR dem eigentlichen Betrieb einmalig gelten muessen.
//
// Nur in Debug-Builds aktiv (DEBUG-Makro, von CubeMX/CMake nur fuer die Debug-Konfiguration
// gesetzt, s. cmake/stm32cubemx/CMakeLists.txt: "$<$<CONFIG:Debug>:DEBUG>") -- im
// Produktions-/Release-Build entfaellt die Pruefung vollstaendig (kein Codegroessen-/
// Laufzeit-Overhead). Das ist bewusst so: Konfigurationsfehler dieser Art sollen beim Bringup
// auffallen (lautes Anhalten + Log), nicht die Firmware im Feld verlangsamen/aufblaehen -- im
// Feld greift stattdessen die jeweilige graceful-degradation-Behandlung im aufrufenden Code
// (z.B. ws2812.hpp ShowEach(): log_warn() + LED-Ausgabe uebersprungen, kein Halt).
#include "log.h"

#ifdef DEBUG
#define HW_CONFIG_ASSERT(condition, message)                                                \
    do {                                                                                    \
        if (!(condition)) {                                                                 \
            log_error("HW_CONFIG_ASSERT failed at %s:%d: (%s) -- %s", __FILE__, __LINE__,   \
                      #condition, message);                                                 \
            while (1) { }                                                                   \
        }                                                                                    \
    } while (0)
#else
#define HW_CONFIG_ASSERT(condition, message) do { } while (0)
#endif
