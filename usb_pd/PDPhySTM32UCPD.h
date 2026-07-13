//
// USB Power Delivery for Arduino
// Copyright (c) 2023 Manuel Bleichenbacher
//
// Licensed under MIT License
// https://opensource.org/licenses/MIT
//

#pragma once

// STM32H5 support: the UCPD1 LL API (LL_UCPD_*) is essentially identical to STM32G4's,
// but the DMA controller behind it is not - STM32H5 uses GPDMA (different
// LL_DMA_InitTypeDef layout, GPDMA1/2 instances, LL_GPDMA1_REQUEST_UCPD1_RX/TX request
// IDs) instead of STM32G4's classic DMA1+DMAMUX. CC1/CC2 pin mapping for STM32H5
// (PB13/PB14) verified against firmware_control_unit_ethercat's CubeMX-generated
// MX_UCPD1_Init(). Not yet verified on real H5 hardware - please test before relying on it.
#if defined(STM32G0xx) || defined(STM32G4xx) || defined(STM32H5xx)

#include "PDPhy.h"

extern "C" void UCPD1_IRQHandler();
extern "C" void UCPD1_2_IRQHandler();

/**
 * @brief Physical layer for USB PD communication.
 * 
 */
struct PDPhySTM32UCPD : PDPhy {
private:
    static void init(bool isMonitor);
    static void enableCommunication(int cc);
    static void disableCommunication();
    static void enableRead();
    static PDSOPSequence mapSOPSequence(uint32_t orderedSet);

    static void handleInterrupt();

    friend class PDPhy;
    friend void UCPD1_IRQHandler();
    friend void UCPD1_2_IRQHandler();
};

#endif
