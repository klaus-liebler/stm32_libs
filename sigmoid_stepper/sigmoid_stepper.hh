#pragma once

#include "gpio.hh"
#include "gpio/timer_capabilities.hh"
#include "sigmoid_acceleration_profile.hh"
#include "hal_header_selector.h"
#include "log.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>

enum class MotionPhase { Idle, Accel, Cruise, Decel, NON_SIGMOID };

// TimerPeripheral: welcher Timer die Step-Pulse per Update-Interrupt erzeugt (Default TIM17,
// wie urspruenglich fest per #define verdrahtet). Als Template-Parameter statt #define erlaubt
// das jetzt mehrere SigmoidStepper-Instanzen gleichzeitig, je auf einem eigenen Timer (z.B. zwei
// unabhaengige Stepper-Achsen auf TIM16 + TIM17).
//
// NVIC-Konfiguration (SetPriority/EnableIRQ) sowie das Weiterleiten des jeweiligen
// TIMx_IRQHandler an Handle_update_interrupt_() bleiben bewusst Aufgabe des Aufrufers: die
// passende IRQn und der passende Handler-Funktionsname sind pro Timer verschieden und lassen
// sich nicht generisch templaten (globale ISR-Symbole muessen konkrete Namen haben).
template <gpio::Peripheral TimerPeripheral = gpio::Peripheral::TIM17_>
class SigmoidStepper {
  static_assert(gpio::has_peripheral(TimerPeripheral),
                "Dieser Timer existiert auf dem gewaehlten Chip nicht (siehe available_peripherals)");

  static TIM_TypeDef *Tim() { return gpio::get_timer_capabilities(TimerPeripheral).instance; }

private:
  gpio::Pin step_pin;
  gpio::Pin dir_pin;
  const SigmoidAccelerationProfile *profile;
  // Kurzer Bezeichner fuer Log-Ausgaben (z.B. "S1 - Sorter"), um bei mehreren
  // SigmoidStepper-Instanzen unterscheiden zu koennen, welche Achse eine Meldung ausgegeben
  // hat. Nur gehalten, nicht kopiert -- ein String-Literal an der Instanziierungsstelle reicht.
  const char *name_;
  bool motion_direction{true};
  MotionPhase motion_phase{MotionPhase::Idle};
  int32_t current_position_{0};
  int32_t target_position_{0};
  int32_t current_arr32{0};
  size_t current_segment_idx{0};
  size_t current_step_in_segment{0};

  void start_motion() {
    reset_lookup();
    Tim()->PSC = profile->getPrescaler();
    Tim()->ARR = current_arr32 >> profile->getShiftBits();
    Tim()->CNT = 0;
    Tim()->DIER |= TIM_DIER_UIE;
    Tim()->CR1 |= TIM_CR1_CEN;
  }


      /// Reset des Beschleunigungsprofils
    void reset_lookup() {
        current_segment_idx = 0;
        current_step_in_segment = 0;
    }

    /// Liefert den nächsten ARR-Wert für einen Schritt
    /// @return ARR-Wert für den Timer (0-65535), oder 65535 wenn fertig
    bool accelerate() {
        if (current_segment_idx >= profile->getNumSegments()) {
            return false;
        }
        const AccelerationSegment& seg = profile->getSegment(current_segment_idx);
        current_arr32 += seg.arr_slope;

        current_step_in_segment++;
        // Wenn wir das Segment abgeschlossen haben, zum nächsten gehen
        if (current_step_in_segment >= seg.steps) {
            current_segment_idx++;
            current_step_in_segment = 0;
        }
        return true;
    }

    /// Deceleration uses the same LUT but in reverse
    bool deceleration() {
        if (current_segment_idx >= profile->getNumSegments()) {
            return false;
        }
      const uint8_t last_idx = static_cast<uint8_t>(profile->getNumSegments() - 1U);
      const uint8_t seg_idx = static_cast<uint8_t>(last_idx - current_segment_idx);
      const AccelerationSegment& seg = profile->getSegment(seg_idx);
        current_arr32 -= seg.arr_slope;

        current_step_in_segment++;
        // Wenn wir das Segment abgeschlossen haben, zum nächsten gehen
        if (current_step_in_segment >= seg.steps) {
            current_segment_idx++;
            current_step_in_segment = 0;
        }
        return true;
    }

    void setARRand50PercentDutyCycle(uint32_t arr32) {
        uint16_t arr16 = static_cast<uint16_t>(arr32 >> profile->getShiftBits());
        Tim()->ARR = arr16;
        Tim()->CCR1 = arr16 / 2; // 50% duty cycle
    }

public:
  /**
   * Constructor: initializes with step and direction pins.
   * @param step_pin GPIO pin for step signal
   * @param dir_pin GPIO pin for direction signal
   */
  SigmoidStepper(gpio::Pin step_pin, gpio::Pin dir_pin,
                 const SigmoidAccelerationProfile *profile,
                 const char *name = "SigmoidStepper")
      : step_pin(step_pin), dir_pin(dir_pin), profile(profile), name_(name),
        motion_direction(true), motion_phase(MotionPhase::Idle),
        current_position_(0), target_position_(0) {}

  /**
   * Initialize the RampStepper subsystem.
   * Must be called once during setup. NVIC_SetPriority()/NVIC_EnableIRQ() fuer die zum
   * gewaehlten TimerPeripheral passende IRQn sowie das Weiterleiten von deren
   * TIMx_IRQHandler() an Handle_update_interrupt_() sind Aufgabe des Aufrufers (s.
   * Klassenkommentar oben).
   */
  void Init() {
    gpio::Gpio::ConfigureGPIOOutput(step_pin, false); // Initialize step pin low
    gpio::Gpio::ConfigureGPIOOutput(dir_pin, false);  // Initialize dir pin low
    gpio::get_timer_capabilities(TimerPeripheral).enableClock();
    Tim()->PSC = profile->getPrescaler();
    Tim()->ARR = current_arr32 >> profile->getShiftBits();
    Tim()->CCR1 = current_arr32 >> (profile->getShiftBits() + 1); // 50% duty cycle
    Tim()->DIER |= TIM_DIER_UIE;
  }

  /**
   * Plan motion to an absolute position.
   * @param targetPosition target position in steps (can be positive or
   * negative)
   */
  void GotoPosition(int targetPosition) {
    const int32_t delta =
        static_cast<int32_t>(targetPosition) - current_position_;
    if (delta == 0) {
      return;
    }
    reset_lookup();
    target_position_ = targetPosition;
    motion_direction = delta > 0;

    // Set direction pin accordingly
    gpio::Gpio::Set(dir_pin, motion_direction);

    const uint32_t steps = static_cast<uint32_t>(std::abs(delta));

    if (steps + 2 < (profile->getTotalSteps() / 2)) {//die +2 ist ein kleiner Puffer, damit wir nicht zu früh in den Decel-Teil wechseln
      motion_phase = MotionPhase::NON_SIGMOID;
    } else {
      motion_phase = MotionPhase::Accel;
    }
    current_arr32 = profile->getLowSpeedARR16() << profile->getShiftBits();
    log_debug("%s: gotoPosition: delta=%ld steps=%lu phase=%d", name_, delta, static_cast<unsigned long>(steps), static_cast<int>(motion_phase));
    start_motion();
  }

  uint16_t GetCurrentARR() const { return static_cast<uint16_t>(current_arr32 >> profile->getShiftBits()); }

  int32_t GetCurrentPosition() const { return current_position_; }

  // Setzt current_position_ direkt, ohne eine Fahrt auszuloesen -- fuer sensorless Homing (s.
  // TMC2209::HomeSensorless()): nach einer per StallGuard erkannten Referenzfahrt steht der
  // Motor an einer bekannten Position, aber current_position_ kennt diese Fahrt nicht (sie lief
  // ueber den TMC2209-internen VACTUAL-Generator, nicht ueber Handle_update_interrupt_()). Der
  // Aufrufer ruft dies direkt danach auf, um die Zaehlung neu zu referenzieren. Setzt auch
  // target_position_ mit, damit compute_status() (s. stepper_control.cpp) nicht faelschlich
  // "in Bewegung, Ziel abweichend" meldet.
  void SetCurrentPosition(int32_t position) {
    current_position_ = position;
    target_position_ = position;
  }

  bool IsMotionActive() const { return TIM_CR1_CEN & Tim()->CR1; }

  bool GetLastDirection() const { return motion_direction; }

  int32_t GetTargetPosition() const { return target_position_; }

  void Handle_update_interrupt_() {
    // Toggle step pin
    gpio::Gpio::Toggle(step_pin);

    // Count every toggle as one pulse
    current_position_ += motion_direction ? 1 : -1;


    const int32_t steps_to_go = std::abs(target_position_ - current_position_);

    switch (motion_phase) {
    case MotionPhase::Accel: {
      const bool is_accelerating = accelerate();
      setARRand50PercentDutyCycle(this->current_arr32);
      if (!is_accelerating) {
        motion_phase = MotionPhase::Cruise;
      }
      break;
    }
    case MotionPhase::Cruise: {
      if (steps_to_go <= static_cast<int32_t>(profile->getTotalSteps() / 2)) {
        motion_phase = MotionPhase::Decel;
        current_segment_idx = 0; // start decel at first reverse index (last LUT segment)
        current_step_in_segment = 0;
      }
      break;
    }
    case MotionPhase::NON_SIGMOID: {
      // Keep constant speed for very short moves
      setARRand50PercentDutyCycle(this->current_arr32);
      break;
    }
    case MotionPhase::Decel: {
      deceleration(); // deceleration uses the same LUT but in reverse
      setARRand50PercentDutyCycle(this->current_arr32);
      break;
    }
    default:
      break;
    }

    // Stop when target reached
    if (current_position_ == target_position_) {
      Tim()->CR1 &= ~TIM_CR1_CEN;
      motion_phase = MotionPhase::Idle;
    }
  }
};
