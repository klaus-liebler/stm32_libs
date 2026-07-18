#pragma once

// Events, die PDSink waehrend PDSink::Loop() an den registrierten IUsbPdEventHandler ausliefert
// (Capabilities-Aenderung, Spannungswechsel, abgelehnte Leistungsanforderung). Frueher direkt in
// PDSink.h definiert -- hierher verschoben, da sowohl PDSink.h (Parametertyp von start()) als
// auch dieses Interface den Typ brauchen.
enum class PDSinkEventType {
    sourceCapabilitiesChanged,
    voltageChanged,
    powerRejected
};

// Empfaengerschnittstelle fuer PDSink-Events (s. PDSink::start()) -- ersetzt den vorherigen
// std::function-basierten Callback: Implementierungen (z.B. USBPDControl) behandeln Events als
// gewoehnliche ueberschriebene Memberfunktion, ohne eine freie Funktion/Lambda als Vermittler
// zwischen PDSink und der eigentlichen Objektmethode zu brauchen.
class IUsbPdEventHandler {
public:
    virtual ~IUsbPdEventHandler() = default;
    virtual void HandleUsbPDEvent(PDSinkEventType eventType) = 0;
};
