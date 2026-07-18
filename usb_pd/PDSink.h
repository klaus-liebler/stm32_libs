//
// USB Power Delivery for Arduino
// Copyright (c) 2023 Manuel Bleichenbacher
//
// Licensed under MIT License
// https://opensource.org/licenses/MIT
//
// USB PD sink
//

#pragma once

#include "IUsbPdEventHandler.h"
#include "PDController.h"
#include "PDSourceCapability.h"
#include <cstddef>
#include <memory>
#include <vector>

/**
 * Power Delivery Sink
 *
 * Communicates with a power source (power supply) to control the voltage
 * and current being supplied.
 *
 * A single global instance of this class exists. It is called 'PowerSink'.
 */
class PDSink {
public:

    /**
    * Checks if the power supply can provide the specified voltage.
    */    
    bool canProvideVoltage(int voltage);
    /**
     * @brief Tries to request the first available voltage from a priority-sorted list.
     *
     * Voltages must be ordered from highest to lowest priority.
     *
     * @param desiredVoltagesDescendingPriority_mV desired voltages in mV, sorted by priority
        * @return true if one of the desired voltages was successfully requested, otherwise false
     */
        bool TryRequestPreferredVoltages(std::vector<int> desiredVoltagesDescendingPriority_mV);

    /**
     * @brief Construct a new instance
     * 
     * For correct voltage reporting, the power sink needs to know if it is
     * bus powered (MCU is powered from the same USB connection that it
     * interacts with for the USB-PD communication), or self-powered (MCU has
     * power supply separate from the one it interacts with).
     * 
     * @param busPowered 'true' if power sink is bus powered, 'false' otherwise
     */
    PDSink(bool busPowered = false);

    /**
     * Starts USB Power Delivery as a power sink.
     *
     * The handler's HandleUsbPDEvent() will be called when 'Loop()' is called and
     * an event has occurred since the last call of 'Loop()'.
     *
     * @param handler event handler (implements IUsbPdEventHandler), or nullptr for none
     */
    void start(IUsbPdEventHandler* handler = nullptr);

    /**
     * Polls for new events.
     * 
     * Call this function frequently from 'loop()'.
     */
    void Loop();

    /// Indicates if the sink is connected to a USB PD power supply
    bool isConnected();

    /**
     * Requests a new voltage and optionally a maximum current.
     * 
     * If the method is successful, itwill also set 'requestedVoltage' and
     * 'requestedCurrent' to reflect what has actually been requested from
     * the power supply. These values might differ because of rounding and
     * unspecified current.
     * 
     * If the method is not successful, it will still remember the voltage
     * and current values. If the power supply becomes available or changes,
     * the sink will try to request these values.
     * 
     * @param voltage voltage, in mV
     * @param maxCurrent maximum current, in mA (or 0 to selected maximum current)
     * @return 'true' if the desired voltage and current are available
     */
    bool requestPower(int voltage, int maxCurrent = 0);

    /**
     * @brief Formats the current source capabilities as a human-readable,
     * tabular multi-line string, e.g. for a single log statement.
     *
     * The first line summarizes the capability count; every following line
     * describes one capability and is indented by `indentFromLine2` spaces,
     * so the table lines up under a caller-side log line prefix of that width
     * (e.g. timestamp/level/file:line).
     *
     * The returned buffer is allocated to the exact size needed (no
     * truncation) and owned by the caller via `unique_ptr` -- no manual
     * `delete[]` required.
     *
     * @param indentFromLine2 number of spaces to indent every line after the first
     * @return the formatted, null-terminated string
     */
    std::unique_ptr<char[]> printCapabilitiesToBuf(int indentFromLine2);

    /// Number of valid elements in `sourceCapabilities` array
    int numSourceCapabilities = 0;

    /// Array of supply capabilities
    PDSourceCapability sourceCapabilities[7];

    /// Active voltage in mV
    int activeVoltage;

    /// Active maximum current in mA
    int activeCurrent;

    /// Requested voltage in mV
    int requestedVoltage;

    /// Requested current in mA
    int requestedCurrent;

private:
    enum ErrorCode {
        ok,
        notConnected,
        noMatchingCapability,
        controllerBusy
    };

    bool isBusPowered;

    IUsbPdEventHandler* eventHandler;
    int ppsIndex;
    int desiredVoltage;
    int desiredCurrent;

    bool capabilitiesChanged;

    // event flags
    bool flagSourceCapChanged;
    bool flagVoltageChanged;
    bool flagPowerRejected;

    static constexpr uint32_t SourceCapsRequestDelayUs = 100000;
    static constexpr uint32_t SourceCapsRetryDelayUs = 200000;
    // after this many unanswered Get_Source_Cap requests, escalate to a Hard Reset
    // (some supplies only (re-)broadcast Source_Capabilities around a Hard Reset,
    // e.g. if they were already attached before this device started communicating
    // and their post-attach broadcast window was missed)
    static constexpr int MaxSourceCapsRetriesBeforeHardReset = 3;

    int sourceCapsRetryCount;

    ErrorCode requestPowerCore(int voltage, int maxCurrent);
    void handleEvent(const PDControllerEvent& event);

    void onMessageReceived(const PDMessage* message);
    void onSourceCapabilities(const PDMessage* message);
    void onPsReady();

    void reset(bool connected);

    void onRerequestPPS();
    static void rerequestPPSCallback();

    void onRequestSourceCaps();
    static void requestSourceCapsCallback();
};

/// Global object for controlling USB power sink
extern class PDSink PowerSink;
