#pragma once
#include <array>
#include "pins.hh"


namespace gpio {

    // Peripherie-Typen, generiert aus allen tatsächlich vorkommenden Alternate-Function-Signalen
enum class Peripheral {
    None,
    #include "generated/peripherals_gen.inc"
};

// Signal-Typen, generiert aus allen tatsächlich vorkommenden Alternate-Function-Signalen
enum class Signal {
    None,
    #include "generated/signals_gen.inc"
};

// Ein AF-Slot an einem Pin. Bewusst EIN Eintrag pro (Pin, AF, Peripherie, Signal) statt eines
// pro-Pin-Arrays mit genau einem Slot je AF-Nummer: manche Chips multiplexen mehr als ein Signal
// auf dieselbe AF-Nummer (z.B. teilen sich USART2_CTS und USART2_NSS oft eine AF, weil CTS nur im
// asynchronen und NSS nur im synchronen Modus benutzt wird) - ein Array mit einem Slot pro Nummer
// würde eines der beiden Signale stillschweigend verlieren.
struct PinAlternateFunction {
    gpio::Pin pin;
    uint8_t af;
    Peripheral peripheral;
    Signal signal;
};


    using P = Peripheral;
    using S = Signal;

    namespace detail {
        // Absichtlich nur deklariert, nie definiert: ein Aufruf ist ausschliesslich als
        // Compile-Zeit-Fehler in consteval-Funktionen gedacht (siehe find_alternate_function/
        // get_timer_capabilities/ChannelIndex) - dort schlaegt "Aufruf einer nicht-constexpr-
        // Funktion" die Auswertung fehl, ganz ohne von `throw` (und damit -fexceptions)
        // abzuhaengen, das in vielen eingebetteten Builds per -fno-exceptions aus ist.
        template <typename T> T consteval_fail();
    }

    inline constexpr std::array pin_matrix{
        #include "pin_mapping_gen.inc"
    };


    consteval uint8_t find_alternate_function(gpio::Pin searchPin, Peripheral searchPeri, Signal searchSig) {
        for (const auto& entry : gpio::pin_matrix) {
            if (entry.pin == searchPin && entry.peripheral == searchPeri && entry.signal == searchSig) {
                return entry.af;
            }
        }
        // Fehlererzeugung für den Compiler: Kombination nicht erlaubt, dieses Signal existiert
        // an diesem Pin nicht.
        return detail::consteval_fail<uint8_t>();
    }

    // Fest verdrahtete (nicht AF-gemultiplexte) Signale je Pin: ADC-/COMP-/OPAMP-/DAC-Kanäle,
    // RTC_TAMP, SYS_WKUP, RCC_LSCO, UCPD_FRSTX, ... - kommen direkt aus der Pin-Liste der MCU-XML,
    // nicht aus der GPIO-AF-Tabelle.
    struct PinFixedSignal {
        gpio::Pin pin;
        Peripheral peripheral;
        Signal signal;
    };

    inline constexpr std::array fixed_signal_matrix{
        #include "pin_fixed_signals_gen.inc"
    };

    consteval bool has_fixed_signal(gpio::Pin searchPin, Peripheral searchPeri, Signal searchSig) {
        for (const auto& entry : gpio::fixed_signal_matrix) {
            if (entry.pin == searchPin && entry.peripheral == searchPeri && entry.signal == searchSig) {
                return true;
            }
        }
        return false;
    }

    // Welche Peripherie-Instanzen dieser exakte Chip laut MCU-Datenbank tatsächlich besitzt
    // (z.B. TIM8, I2C3) - nutzbar für static_assert-Guards gegen nicht existierende Peripherie.
    inline constexpr std::array available_peripherals{
        #include "available_peripherals_gen.inc"
    };

    consteval bool has_peripheral(Peripheral searchPeri) {
        for (const auto& peri : gpio::available_peripherals) {
            if (peri == searchPeri) {
                return true;
            }
        }
        return false;
    }

    // EXTI-Line entspricht bei allen STM32-Familien immer der Pin-Nummer (0-15) innerhalb des
    // Ports, unabhängig vom Port selbst - reine Arithmetik, keine Chip-spezifischen Daten nötig.
    consteval uint8_t pin_to_exti_line(gpio::Pin pin) {
        return static_cast<uint8_t>(pin) & 0x0F;
    }
}