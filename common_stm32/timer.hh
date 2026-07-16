#pragma once
#include "gpio.hh"
#include "gpio/pin_mapping.hh"
#include "gpio/timer_capabilities.hh"

namespace timer {

// Welcher Output-Compare-Kanal (1-4) ein Signal anspricht, und ob es der komplementäre Ausgang ist.
// CH1N/CH2N/CH3N teilen sich Kompare-Register und CCMR-Konfiguration mit ihrem Hauptkanal - nur
// das CCER-Freigabebit unterscheidet sich (CCxE vs. CCxNE).
consteval uint8_t ChannelIndex(gpio::Signal signal) {
  if (signal == gpio::Signal::CH1_ || signal == gpio::Signal::CH1N_) return 1;
  if (signal == gpio::Signal::CH2_ || signal == gpio::Signal::CH2N_) return 2;
  if (signal == gpio::Signal::CH3_ || signal == gpio::Signal::CH3N_) return 3;
  if (signal == gpio::Signal::CH4_) return 4;
  // ChannelIndex: kein PWM-Output-Kanal (erwartet CH1..CH4 oder CH1N..CH3N)
  return gpio::detail::consteval_fail<uint8_t>();
}

consteval bool IsComplementary(gpio::Signal signal) {
  return signal == gpio::Signal::CH1N_ || signal == gpio::Signal::CH2N_ || signal == gpio::Signal::CH3N_;
}

// PWM-Ausgabe auf einem Timer-Kanal.
//
// TimerClockSourceHz wird bewusst NICHT selbst berechnet: der tatsächliche Timer-Kerntakt hängt von
// der Clock-Tree-Konfiguration des jeweiligen Projekts ab (u.a. vom bekannten STM32-Fallstrick, dass
// der Timer-Takt das Doppelte des APB-Takts ist, sobald der zugehörige APB-Prescaler != 1 ist) - das
// kann diese Bibliothek nicht aus der Chip-Datenbank ableiten, das muss der Aufrufer wissen.
template <gpio::Pin Pin, gpio::Peripheral Timer, gpio::Signal Channel, uint32_t TimerClockSourceHz,
          uint32_t TargetFrequencyHz = 1000>
class PwmOutput {
  static_assert(gpio::has_peripheral(Timer),
                "Dieser Timer existiert auf dem gewählten Chip nicht (siehe available_peripherals)");

  static constexpr gpio::TimerCapabilities Capabilities = gpio::get_timer_capabilities(Timer);
  static constexpr uint8_t Ch = ChannelIndex(Channel);
  static_assert(Ch <= Capabilities.channelCount,
                "Dieser Timer hat auf diesem Chip nicht so viele Kanäle");
  static_assert(!IsComplementary(Channel) || Capabilities.hasComplementaryChannels,
                "Dieser Timer hat keine komplementären Ausgänge");

  // Löst gleichzeitig gpio::find_alternate_function auf und prüft zur Compilezeit, dass Pin, Timer
  // und Channel tatsächlich zusammenpassen (kompiliert sonst gar nicht erst).
  static constexpr uint8_t AfNumber = gpio::find_alternate_function(Pin, Timer, Channel);

  // Prescaler/Autoreload so wählen, dass die Zielfrequenz möglichst genau getroffen wird, ohne die
  // (chip-/timer-spezifische!) ARR-Breite zu überschreiten - 16 Bit bei den meisten Timern, 32 Bit
  // z.B. bei TIM2/TIM5 auf vielen Familien (siehe gpio::get_timer_capabilities).
  static constexpr uint32_t CalculatedTicks = TimerClockSourceHz / TargetFrequencyHz;
  static constexpr uint32_t Prescaler = (CalculatedTicks - 1) / (uint64_t{Capabilities.maxAutoReload} + 1);
  static constexpr uint32_t Period = CalculatedTicks / (Prescaler + 1) - 1;

  static_assert(Prescaler <= Capabilities.maxPrescaler,
                "Zielfrequenz zu niedrig - selbst mit maximalem Prescaler nicht erreichbar");
  static_assert(Period <= Capabilities.maxAutoReload,
                "Zielfrequenz zu niedrig für die Autoreload-Registerbreite dieses Timers");

  static TIM_TypeDef *Tim() { return Capabilities.instance; }

  // OCxM (PWM-Modus 1, TIM_OCxM = 0b110) + OCxPE (Preload) im passenden CCMR-Register setzen.
  // CH1/CH2 liegen in CCMR1, CH3/CH4 in CCMR2 - beide Register sind byteweise identisch aufgebaut.
  static void ConfigureOutputCompareMode() {
    constexpr uint32_t ocMode = 0b110 << ((Ch == 1 || Ch == 3) ? 4 : 12);
    constexpr uint32_t ocPreload = 0b1 << ((Ch == 1 || Ch == 3) ? 3 : 11);
    if constexpr (Ch == 1 || Ch == 2) {
      Tim()->CCMR1 = (Tim()->CCMR1 & ~(0xFFu << ((Ch - 1) * 8))) | ocMode | ocPreload;
    } else {
      Tim()->CCMR2 = (Tim()->CCMR2 & ~(0xFFu << ((Ch - 3) * 8))) | ocMode | ocPreload;
    }
  }

  static void EnableOutput() {
    constexpr uint32_t enableBit = 0b1u << ((Ch - 1) * 4 + (IsComplementary(Channel) ? 2 : 0));
    Tim()->CCER |= enableBit;
  }

  static void WriteCompareRegister(uint32_t value) {
    if constexpr (Ch == 1) Tim()->CCR1 = value;
    else if constexpr (Ch == 2) Tim()->CCR2 = value;
    else if constexpr (Ch == 3) Tim()->CCR3 = value;
    else Tim()->CCR4 = value;
  }

public:
  PwmOutput() = default;

  static void Init(uint16_t initialDutyCycleBasisPoints = 0,
                    gpio::OutputSpeed speed = gpio::OutputSpeed::HIGH) {
    Capabilities.enableClock();
    gpio::Gpio::ConfigureAlternateFunction(Pin, AfNumber, gpio::OutputType::PUSH_PULL, speed);

    Tim()->PSC = Prescaler;
    Tim()->ARR = Period;
    Tim()->CR1 |= TIM_CR1_ARPE; // Autoreload gepuffert, sonst Glitches bei Frequenzänderung

    ConfigureOutputCompareMode();
    WriteCompareRegister(0);
    EnableOutput();

    // Advanced-Control-Timer (TIM1/TIM8/TIM15/TIM16/TIM17) geben ohne Main-Output-Enable in BDTR
    // gar nichts auf den Pin aus - erkennbar zuverlässig daran, dass sie (als einzige) einen
    // Repetition-Counter besitzen, statt den Timernamen hier hart zu verdrahten.
    if constexpr (Capabilities.maxRepetitionCounter > 0) {
      Tim()->BDTR |= TIM_BDTR_MOE;
    }

    Tim()->EGR = TIM_EGR_UG;   // Prescaler/ARR/CCMR sofort ins Schattenregister übernehmen
    Tim()->CR1 |= TIM_CR1_CEN; // Zähler starten

    SetDutyCycle_0_10000(initialDutyCycleBasisPoints);
  }

  // 0 (0.00%) bis 10000 (100.00%)
  static void SetDutyCycle_0_10000(uint16_t basisPoints) {
    uint32_t ccrValue = (uint64_t{Period + 1} * basisPoints) / 10000;
    WriteCompareRegister(ccrValue);
  }
};

} // namespace timer
