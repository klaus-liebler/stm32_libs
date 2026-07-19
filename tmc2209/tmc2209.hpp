#pragma once

#include <cstdint>
#include "hal_header_selector.h"
#include "gpio.hh"
#include "tmc2209_reg.hpp"

namespace tmc2209 {
/*
17HS4401S:
    - Step angle: 1.8 degrees (200 steps/rev)
    - max current: 1.7A
    - PhaseVoltage: 2.6V
    - Resistance: 1.5 Ohm
    - Inductance: 2.8 mH
    - Holding torque: 43 Ncm

*/

enum class MicroStepResolution { RES256=0b0000, RES128=0b0001, RES64=0b0010, RES32=0b0011, RES16=0b0100, RES8=0b0101, RES4=0b0110, RES2=0b0111, RES_FULL_STEP=0b1000 };


struct TuningResults{
    uint32_t pwm_grad;
    uint32_t pwm_ofs;
    uint32_t pwm_scale;
    uint32_t pwm_auto;
};

class TMC2209 {
public:
    // address: 0..15 (MSB used for read flag internally)
    // Optional EN pin: pass Pin::NO_PIN to disable GPIO control
    // name: kurzer Bezeichner fuer Log-Ausgaben (z.B. "S1 - Sorter"), um bei mehreren
    // TMC2209-Instanzen am selben UART-Bus in den Logs unterscheiden zu koennen, welche
    // Instanz eine Meldung ausgegeben hat. Der Zeiger wird nur gehalten, nicht kopiert -- muss
    // also mindestens so lange gueltig bleiben wie das TMC2209-Objekt (ein String-Literal an
    // der Aufrufstelle reicht dafuer aus).
    TMC2209(UART_HandleTypeDef* huart, uint8_t address, gpio::Pin enPin = gpio::Pin::NO_PIN, const char* name = "TMC2209");

    bool InitForNormalSpeedAndUartBasedOperation(bool usePotentiometerForCurrentScaling = false, bool disable_read = false,  MicroStepResolution resolution = MicroStepResolution::RES256);
    bool ShaftTurnReversed(bool reversed);
    void PrintPrettyFullSystemState();
    bool PerformStealthChopAutoTuningForQuietOperation();
    bool EnableCoolStep(uint32_t tcoolthrs = 450, uint8_t sgthrs = 10,
                        uint8_t semin = 5, uint8_t seup = 1,
                        uint8_t semax = 2, uint8_t sedn = 1,
                        bool seimin = false);
    bool LogCoolStepRuntimeStatus();

   

    // Generate steps at specified speed in full steps per second. A common stepper motor with 200 steps/rev
    // will rotate at with 2 Rotations per second when set to value "400"
    bool GenerateSteps(int fullStepsPerSecond);

    // Convenience APIs
    void enable();
    void disable();

    // IHOLD_IRUN fields: ihold/run in 0..31, iholddelay 0..15
    bool setIHoldIRun(uint8_t ihold, uint8_t irun, uint8_t iholddelay);

    // Sensorless Homing per StallGuard: treibt den Motor ueber den internen
    // Geschwindigkeitsgenerator (VACTUAL, s. GenerateSteps()) mit konstanter Geschwindigkeit,
    // bis ein mechanischer Anschlag per StallGuard erkannt wird (SG_RESULT faellt unter
    // sgthrs*2) -- kein Endschalter noetig. Aktiviert waehrend der Fahrt CoolStep (lastabhaengige
    // automatische Stromabsenkung, s. EnableCoolStep()), damit der Motor nicht unnoetig heiss
    // laeuft, waehrend er ggf. laenger gegen den Anschlag drueckt; schaltet danach wieder auf
    // StealthChop zurueck (CoolStep braucht SpreadCycle, das ist also bewusst nur waehrend des
    // Homings aktiv, nicht im Normalbetrieb).
    // Setzt current_position_ einer begleitenden SigmoidStepper-Instanz NICHT selbst zurueck
    // (kennt sie nicht) -- das macht der Aufrufer per SigmoidStepper::SetCurrentPosition() nach
    // einer true-Rueckgabe.
    // @param homingSpeedFullStepsPerSecond Geschwindigkeit waehrend der Fahrt (Vorzeichen = Richtung)
    // @param sgthrs StallGuard-Schwelle (0..255): hoehere Werte = empfindlicher, erkennt einen Stall frueher/leichter
    // @param timeoutMs Sicherheits-Timeout, falls kein Stall erkannt wird
    // @return true wenn ein Stall erkannt wurde, false bei Timeout oder Kommunikationsfehler
    bool HomeSensorless(int homingSpeedFullStepsPerSecond, uint8_t sgthrs = 100, uint32_t timeoutMs = 10000, uint32_t travelOppositeMs = 800, uint32_t homingDeadTimeMs = 300);

    // Nicht blockierende Variante von HomeSensorless() fuer Aufrufer, die aus einem gemeinsamen
    // Loop()/Tick-Kontext heraus arbeiten und nicht Sekunden lang blockieren duerfen (z.B. ein
    // IO-Thread, der auch andere Sensoren/Aktoren bedient). StartHomingNonBlocking() stoesst nur
    // die erste Phase an (nicht blockierend) und liefert sofort zurueck; TickHoming(now_ms) muss
    // danach bei jedem Zyklus (now_ms = HAL_GetTick()) erneut aufgerufen werden, bis etwas
    // anderes als InProgress zurueckkommt. Dieselben Phasen wie HomeSensorless() (Gegenfahrt,
    // Einschwingzeit, Totzeit, StallGuard-Polling), nur zeitgesteuert statt per HAL_Delay().
    // Setzt current_position_ einer begleitenden SigmoidStepper-Instanz NICHT selbst zurueck
    // (kennt sie nicht) -- das macht der Aufrufer per SigmoidStepper::SetCurrentPosition() nach
    // einem Success-Ergebnis.
    enum class HomingResult { InProgress, Success, Timeout, Error };
    bool StartHomingNonBlocking(int homingSpeedFullStepsPerSecond, uint8_t sgthrs = 100, uint32_t timeoutMs = 10000, uint32_t travelOppositeMs = 800, uint32_t homingDeadTimeMs = 300);
    HomingResult TickHoming(uint32_t now_ms);

private:
    UART_HandleTypeDef* huart_;
    uint8_t addr_; // 0..15
    gpio::Pin en_;
    const char* name_;
    REG_FIELD::GCONF gconf;
    REG_FIELD::CHOPCONF chopconf;
    uint32_t last_tcoolthrs_ = 0;
    uint8_t last_sgthrs_ = 0;
    uint32_t last_coolconf_ = 0;
    bool coolstep_configured_ = false;

    enum class HomingPhase { Idle, OppositeTravel, Settle, DeadTime, StallDetect };
    HomingPhase homing_phase_ = HomingPhase::Idle;
    uint32_t homing_phase_start_ms_ = 0;
    int homing_speed_ = 0;
    uint8_t homing_sgthrs_ = 0;
    uint32_t homing_timeout_ms_ = 0;
    uint32_t homing_travel_opposite_ms_ = 0;
    uint32_t homing_dead_time_ms_ = 0;

    bool fetchImportantRegistersForLocalMirroring();
    bool clearGStat(const char* error_msg);
    bool writeRegister(RegIdx reg, uint32_t value, uint32_t timeoutMs = 200);
    bool readRegister(RegIdx reg, uint32_t& value, uint32_t timeoutMs = 200, bool checkStartByte = true);

    bool checkedWriteRegister(RegIdx reg, uint32_t value, const char* error_msg);                                   
    bool checkedReadRegister(RegIdx reg, uint32_t& value, const char* error_msg);

    static uint8_t crc8(uint8_t* datagram, size_t datagram_size_with_crc);
};

} // namespace tmc2209
