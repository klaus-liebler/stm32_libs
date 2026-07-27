#pragma once
#include <cstdint>
#include <cstddef>
#include "modbus_commons.hh"
#include "modbus_register_model.hh"
#include "common.hh"
#include "tx_api.h"

// Modbus-RTU-Slave ueber eine USB-CDC-Schnittstelle (Adress-Byte + CRC16-Framing statt des
// MBAP-Headers von ModbusTcpServer). FC03/04/06/16 sind bewusst identisch zu ModbusTcpServer, da
// beide dasselbe Modbus::IModbusRegisterModel binden -- FC01/02/05/15 (Coils) sind NICHT
// portiert, weil IModbusRegisterModel gar keine Coils kennt (nur Input-/Holding-Register, s.
// register-map.json) -- exakt dieselbe Einschraenkung wie schon bei ModbusTcpServer.
//
// CRC16/Framing-Logik portiert aus der ESP32-Referenz (modbusslave.hh/modbuscommons.hh,
// C:\repos\espidf-components\modbus_rtu), an dieses Projekt angepasst: Registerzugriff laeuft
// ueber IModbusRegisterModel statt roher std::vector-Zeiger.
//
// Diese Klasse enthaelt bewusst NUR die Protokoll-Logik (Framing/CRC/Register-Zugriff), keine
// eigene Thread-/USB-IO-Schleife mehr -- die lebt komplett in App::ModbusRtuThread() (app.cc),
// die auch den Puffer (Buffer()/BufferSize()) und die eigentlichen blockierenden
// ux_device_class_cdc_acm_read()/_write()-Aufrufe besitzt. ProcessChunk() unten ist die
// "Einsprungfunktion", die App::ModbusRtuThread() nach jedem neu empfangenen Chunk aufruft --
// message_length (Bytes einer noch unvollstaendigen Anfrage) und response_length (Bytes einer
// fertigen Antwort, 0 = keine) sind dafuer der Vertrag. Empfangen und Senden schliessen sich in
// App::ModbusRtuThread() gegenseitig aus, was fuer Modbus-RTU (strikt Anfrage/Antwort, nie
// beides gleichzeitig unterwegs) ohnehin passt.
//
// Vertrag von ProcessChunk(size_t* message_length, size_t* response_length):
//  1) Frame (noch) unvollstaendig (z.B. nur der erste USB-Chunk einer Anfrage angekommen):
//     *message_length/*response_length bleiben unveraendert, der Aufrufer haengt weitere Bytes
//     ab Buffer()[*message_length] an.
//  2) Frame weiterhin unvollstaendig UND seit InterFrameTimeoutTicks() keine neuen Bytes (z.B.
//     beim Hochfahren, wenn nur das Ende einer Anfrage ankam): *message_length=0,
//     *response_length=0 -- wird NICHT von ProcessChunk() erkannt (die kennt keine Uhrzeit),
//     sondern vom Aufrufer selbst, der dafuer ProcessChunk() gar nicht erst aufruft.
//  3) Frame vollstaendig: Antwort wird direkt in Buffer() geschrieben, *message_length=0,
//     *response_length=Antwortlaenge (>0).
class ModbusRtuServer {
public:
    // inter_frame_timeout_ticks: wie lange (ThreadX-Ticks) ohne neue Bytes gewartet wird, bevor
    // ein angefangener, unvollstaendiger Frame verworfen wird. Modbus RTU kennt dafuer eigentlich
    // "3.5 Zeichenzeiten Stille" auf einem echten UART mit fester Baudrate -- ueber USB-CDC-Bulk-
    // Transfers (paketweise, keine feste Baudrate) gibt es das nicht 1:1, ein grosszuegiger
    // fester Wert genuegt hier.
    ModbusRtuServer(uint8_t slave_id, Modbus::IModbusRegisterModel& registers,
                    ULONG inter_frame_timeout_ticks = 10)
        : m_slave_id(slave_id), m_registers(registers),
          m_inter_frame_timeout_ticks(inter_frame_timeout_ticks) {
    }

    // Empfangs- UND Sendepuffer zugleich (s. Klassenkommentar) -- Eigentum dieser Klasse, aber
    // von App::ModbusRtuThread() direkt bei ux_device_class_cdc_acm_read()/_write() als Ziel-
    // /Quellpuffer benutzt.
    uint8_t* Buffer() { return m_buf; }
    uint8_t SlaveId() const { return m_slave_id; }
    static constexpr size_t BufferSize() { return MAX_ADU_SIZE; }
    ULONG InterFrameTimeoutTicks() const { return m_inter_frame_timeout_ticks; }

    // "Einsprungfunktion" des Slaves (s. Klassenkommentar oben fuer den vollstaendigen
    // Vertrag): arbeitet auf Buffer() (Empfangs- UND Sendepuffer zugleich), die Antwort wird bei
    // vollstaendigem Frame direkt ueber die bereits verarbeiteten Anfrage-Bytes geschrieben (jede
    // Process*()-Methode liest alle noetigen Eingabe-Bytes, bevor sie an dieselbe Position etwas
    // zurueckschreibt -- In-Place ist damit sicher, ohne einen zweiten Puffer zu brauchen).
    void ProcessChunk(size_t* message_length, size_t* response_length) {
        size_t tx_size = 0;
        int result = Parse(*message_length, tx_size);
        if (result == (int)ParseResult::NOT_YET_COMPLETE) {
            return;  // Fall 1: weitere Bytes abwarten, *message_length bleibt stehen
        }
        // Fall 3 (OK) oder Frame verworfen (NOT_FOR_ME/CRC-/Laengenfehler): so oder so ist der
        // Frame in Buffer() fertig verarbeitet, *message_length faengt wieder bei 0 an.
        *response_length = (result == (int)ParseResult::OK) ? tx_size : 0;
        *message_length = 0;
    }

private:
    enum class ParseResult : int {
        OK = 0,
        NOT_FOR_ME = 1,
        NOT_YET_COMPLETE = 2,
        CRC_ERROR = -1,
        LENGTH_ERROR = -2,
    };

    static constexpr size_t MAX_ADU_SIZE = 256;

    // --- CRC16 (Modbus), portiert aus der ESP32-Referenz modbuscommons.hh ---
    static uint16_t CalcCRC(const uint8_t* data, size_t len) {
        static const uint8_t crcHiTable[] = {
            0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
            0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
            0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01,
            0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
            0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81,
            0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
            0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01,
            0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
            0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
            0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
            0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01,
            0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
            0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
            0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
            0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01,
            0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
            0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
            0x40};

        static const uint8_t crcLoTable[] = {
            0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2, 0xC6, 0x06, 0x07, 0xC7, 0x05, 0xC5, 0xC4,
            0x04, 0xCC, 0x0C, 0x0D, 0xCD, 0x0F, 0xCF, 0xCE, 0x0E, 0x0A, 0xCA, 0xCB, 0x0B, 0xC9, 0x09,
            0x08, 0xC8, 0xD8, 0x18, 0x19, 0xD9, 0x1B, 0xDB, 0xDA, 0x1A, 0x1E, 0xDE, 0xDF, 0x1F, 0xDD,
            0x1D, 0x1C, 0xDC, 0x14, 0xD4, 0xD5, 0x15, 0xD7, 0x17, 0x16, 0xD6, 0xD2, 0x12, 0x13, 0xD3,
            0x11, 0xD1, 0xD0, 0x10, 0xF0, 0x30, 0x31, 0xF1, 0x33, 0xF3, 0xF2, 0x32, 0x36, 0xF6, 0xF7,
            0x37, 0xF5, 0x35, 0x34, 0xF4, 0x3C, 0xFC, 0xFD, 0x3D, 0xFF, 0x3F, 0x3E, 0xFE, 0xFA, 0x3A,
            0x3B, 0xFB, 0x39, 0xF9, 0xF8, 0x38, 0x28, 0xE8, 0xE9, 0x29, 0xEB, 0x2B, 0x2A, 0xEA, 0xEE,
            0x2E, 0x2F, 0xEF, 0x2D, 0xED, 0xEC, 0x2C, 0xE4, 0x24, 0x25, 0xE5, 0x27, 0xE7, 0xE6, 0x26,
            0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21, 0x20, 0xE0, 0xA0, 0x60, 0x61, 0xA1, 0x63, 0xA3, 0xA2,
            0x62, 0x66, 0xA6, 0xA7, 0x67, 0xA5, 0x65, 0x64, 0xA4, 0x6C, 0xAC, 0xAD, 0x6D, 0xAF, 0x6F,
            0x6E, 0xAE, 0xAA, 0x6A, 0x6B, 0xAB, 0x69, 0xA9, 0xA8, 0x68, 0x78, 0xB8, 0xB9, 0x79, 0xBB,
            0x7B, 0x7A, 0xBA, 0xBE, 0x7E, 0x7F, 0xBF, 0x7D, 0xBD, 0xBC, 0x7C, 0xB4, 0x74, 0x75, 0xB5,
            0x77, 0xB7, 0xB6, 0x76, 0x72, 0xB2, 0xB3, 0x73, 0xB1, 0x71, 0x70, 0xB0, 0x50, 0x90, 0x91,
            0x51, 0x93, 0x53, 0x52, 0x92, 0x96, 0x56, 0x57, 0x97, 0x55, 0x95, 0x94, 0x54, 0x9C, 0x5C,
            0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E, 0x5A, 0x9A, 0x9B, 0x5B, 0x99, 0x59, 0x58, 0x98, 0x88,
            0x48, 0x49, 0x89, 0x4B, 0x8B, 0x8A, 0x4A, 0x4E, 0x8E, 0x8F, 0x4F, 0x8D, 0x4D, 0x4C, 0x8C,
            0x44, 0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86, 0x82, 0x42, 0x43, 0x83, 0x41, 0x81, 0x80,
            0x40};

        uint8_t crcHi = 0xFF;
        uint8_t crcLo = 0xFF;
        while (len--) {
            uint8_t index = crcLo ^ *data++;
            crcLo = crcHi ^ crcHiTable[index];
            crcHi = crcLoTable[index];
        }
        return (uint16_t)((crcHi << 8) | crcLo);
    }

    // CRC wird Low-Byte-zuerst geschrieben (Modbus-RTU-Konvention) -- WriteU16() (nicht die
    // _BigEndian-Variante) aus common_stm32/common.hh macht genau das.
    static void WriteCRC(uint8_t* data, size_t position) {
        WriteU16(CalcCRC(data, position), data, position);
    }

    static bool ValidCRCInLastTwoBytes(const uint8_t* data, size_t len) {
        uint16_t received = data[len - 2] | ((uint16_t)data[len - 1] << 8);
        return CalcCRC(data, len - 2) == received;
    }

    // Arbeitet auf m_buf (Empfangs- UND Sendepuffer, s. ProcessChunk()); rx_len ist die Anzahl
    // gueltiger, bereits empfangener Bytes darin.
    int Parse(size_t rx_len, size_t& tx_size) {
        if (rx_len < 8) {
            return (int)ParseResult::NOT_YET_COMPLETE;  // kuerzeste gueltige Anfrage ist 8 Byte
        }
        if (m_buf[0] != m_slave_id) {
            return (int)ParseResult::NOT_FOR_ME;
        }

        uint8_t function_code = m_buf[1];
        switch (function_code) {
            case 0x03: return ProcessReadRegisters(rx_len, tx_size, function_code, /*holding=*/true);
            case 0x04: return ProcessReadRegisters(rx_len, tx_size, function_code, /*holding=*/false);
            case 0x06: return ProcessWriteSingleRegister(rx_len, tx_size);
            case 0x10: return ProcessWriteMultipleRegisters(rx_len, tx_size);
            default:   return BuildExceptionResponse(tx_size, function_code, 0x01);  // Illegal Function
        }
    }

    int ProcessReadRegisters(size_t rx_len, size_t& tx_size, uint8_t function_code, bool holding) {
        if (rx_len != 8) return (int)ParseResult::LENGTH_ERROR;
        if (!ValidCRCInLastTwoBytes(m_buf, rx_len)) return (int)ParseResult::CRC_ERROR;

        // Adresse/Anzahl zuerst vollstaendig auslesen -- die Antwort ueberschreibt m_buf ab Byte
        // 0 (In-Place, s. Klassenkommentar), die hier gelesenen Bytes 2..5 duerfen also erst
        // DANACH angefasst werden.
        uint16_t start_addr = ParseU16_BigEndian(m_buf, 2);
        uint16_t quantity = ParseU16_BigEndian(m_buf, 4);
        uint16_t max_index = holding ? ModbusRegisters::HOLDING_REGISTER_MAX_INDEX
                                     : ModbusRegisters::INPUT_REGISTER_MAX_INDEX;
        if (quantity == 0 || quantity > 125 || start_addr + quantity > (uint32_t)max_index + 1) {
            return BuildExceptionResponse(tx_size, function_code, 0x02);  // Illegal Data Address
        }

        m_buf[0] = m_slave_id;
        m_buf[1] = function_code;
        m_buf[2] = (uint8_t)(quantity * 2);
        size_t offset = 3;
        for (uint16_t i = 0; i < quantity; i++) {
            uint16_t value = holding ? m_registers.GetHoldingRegister(start_addr + i)
                                     : m_registers.GetInputRegister(start_addr + i);
            m_buf[offset++] = (uint8_t)(value >> 8);
            m_buf[offset++] = (uint8_t)(value & 0xFF);
        }
        WriteCRC(m_buf, offset);
        tx_size = offset + 2;
        return (int)ParseResult::OK;
    }

    int ProcessWriteSingleRegister(size_t rx_len, size_t& tx_size) {
        if (rx_len != 8) return (int)ParseResult::LENGTH_ERROR;
        if (!ValidCRCInLastTwoBytes(m_buf, rx_len)) return (int)ParseResult::CRC_ERROR;

        uint16_t addr = ParseU16_BigEndian(m_buf, 2);
        uint16_t value = ParseU16_BigEndian(m_buf, 4);
        if (addr > ModbusRegisters::HOLDING_REGISTER_MAX_INDEX) {
            return BuildExceptionResponse(tx_size, 0x06, 0x02);
        }
        m_registers.SetHoldingRegister(addr, value);

        // Antwort = Echo der Anfrage (FC06-Konvention) -- bereits an Ort und Stelle in m_buf,
        // nur die CRC muss (mit denselben 6 Bytes) neu geschrieben werden.
        WriteCRC(m_buf, 6);
        tx_size = 8;
        return (int)ParseResult::OK;
    }

    int ProcessWriteMultipleRegisters(size_t rx_len, size_t& tx_size) {
        if (rx_len < 9) return (int)ParseResult::NOT_YET_COMPLETE;

        uint16_t start_addr = ParseU16_BigEndian(m_buf, 2);
        uint16_t quantity = ParseU16_BigEndian(m_buf, 4);
        uint8_t byte_count = m_buf[6];

        if (quantity == 0 || quantity > 123 || byte_count != quantity * 2) {
            return BuildExceptionResponse(tx_size, 0x10, 0x03);  // Illegal Data Value
        }

        size_t necessary_length = 7 + byte_count + 2;  // Header(6)+ByteCount(1) + Daten + CRC(2)
        if (rx_len < necessary_length) {
            return (int)ParseResult::NOT_YET_COMPLETE;
        }
        if (rx_len > necessary_length) {
            return (int)ParseResult::LENGTH_ERROR;
        }
        if (!ValidCRCInLastTwoBytes(m_buf, rx_len)) {
            return (int)ParseResult::CRC_ERROR;
        }
        if (start_addr + quantity > (uint32_t)ModbusRegisters::HOLDING_REGISTER_MAX_INDEX + 1) {
            return BuildExceptionResponse(tx_size, 0x10, 0x02);  // Illegal Data Address
        }

        // Alle Register-Werte vollstaendig auslesen, BEVOR die Antwort ab Byte 0 in m_buf
        // hineingeschrieben wird (In-Place, s. Klassenkommentar) -- die Werte liegen ab Byte 7,
        // die Antwort ist nur 8 Byte lang, ueberschneidet sich also ohnehin nicht, aber die
        // Reihenfolge (erst lesen, dann schreiben) bleibt auch hier Voraussetzung.
        for (uint16_t i = 0; i < quantity; i++) {
            uint16_t value = ParseU16_BigEndian(m_buf, 7 + i * 2);
            m_registers.SetHoldingRegister(start_addr + i, value);
        }

        m_buf[0] = m_slave_id;
        m_buf[1] = 0x10;
        WriteU16_BigEndian(start_addr, m_buf, 2);
        WriteU16_BigEndian(quantity, m_buf, 4);
        WriteCRC(m_buf, 6);
        tx_size = 8;
        return (int)ParseResult::OK;
    }

    int BuildExceptionResponse(size_t& tx_size, uint8_t function_code, uint8_t exception_code) {
        m_buf[0] = m_slave_id;
        m_buf[1] = function_code | 0x80;
        m_buf[2] = exception_code;
        WriteCRC(m_buf, 3);
        tx_size = 5;
        return (int)ParseResult::OK;  // Exception-Antworten WERDEN gesendet (anders als CRC-Fehler)
    }

    uint8_t m_slave_id;
    Modbus::IModbusRegisterModel& m_registers;
    ULONG m_inter_frame_timeout_ticks;
    // Empfangs- UND Sendepuffer zugleich (s. Klassenkommentar) -- so gross wie die maximale
    // Modbus-ADU (256 Byte, s. MAX_ADU_SIZE).
    uint8_t m_buf[MAX_ADU_SIZE];
};
