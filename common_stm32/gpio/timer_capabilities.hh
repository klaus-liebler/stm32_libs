#pragma once
#include <array>
#include "pin_mapping.hh"
#include "../hal_header_selector.h"

namespace gpio {

    // Register-Fakten je Timer-Instanz, aus der STM32CubeMX-MCU-Datenbank generiert:
    // - maxPrescaler/maxAutoReload/maxRepetitionCounter: Registerbreite (z.B. 32-Bit-ARR bei TIM2/TIM5
    //   auf vielen Familien - chip-/familienabhängig, siehe TIMx_8<family>-gptimer2_..._Modes.xml).
    // - channelCount/hasComplementaryChannels: aus den tatsächlich auf Pins gemultiplexten AF-Signalen
    //   dieses exakten Chips abgeleitet (0 Kanäle für Basic-Timer wie TIM6/TIM7 ohne GPIO-Anbindung).
    // - instance/enableClock: bewusst NICHT als rohe Adresse/Register-Offset generiert. Auf
    //   TrustZone-Chips (STM32H5/L5/U5) hat z.B. TIM1_BASE zwei Werte (TIM1_BASE_S/_NS je nachdem, ob
    //   secure oder non-secure gebaut wird) - ein einzelner fest eincodierter Zahlenwert wäre für eine
    //   der beiden Welten schlicht falsch. Stattdessen wird hier der literale CMSIS-/HAL-Name erzeugt
    //   (TIM2, __HAL_RCC_TIM2_CLK_ENABLE) - der bereits eingebundene HAL-Header hat die
    //   secure/non-secure-Auflösung zu diesem Zeitpunkt schon korrekt vorgenommen, unabhängig davon,
    //   für welche Familie/Welt gerade übersetzt wird.
    struct TimerCapabilities {
        Peripheral peripheral;
        uint32_t maxPrescaler;
        uint32_t maxAutoReload;
        uint32_t maxRepetitionCounter;
        uint8_t channelCount;
        bool hasComplementaryChannels;
        TIM_TypeDef* instance;
        void (*enableClock)();
    };

    inline constexpr std::array timer_capabilities{
        #include "timer_capabilities_gen.inc"
    };

    consteval const TimerCapabilities& get_timer_capabilities(Peripheral searchPeri) {
        for (const auto& tc : gpio::timer_capabilities) {
            if (tc.peripheral == searchPeri) {
                return tc;
            }
        }
        // Keine Timer-Capabilities bekannt - existiert diese Peripherie als Timer auf diesem Chip?
        return detail::consteval_fail<const TimerCapabilities&>();
    }
}
