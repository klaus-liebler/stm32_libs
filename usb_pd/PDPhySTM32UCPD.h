//
// USB Power Delivery for Arduino
// Copyright (c) 2023 Manuel Bleichenbacher
//
// Licensed under MIT License
// https://opensource.org/licenses/MIT
//

#pragma once

// TODO(stm32_libs H5 port): this PHY talks to the UCPD peripheral via the STM32G4 LL API
// (LL_UCPD_*, DMA channel/request names, CC1/CC2 pin mapping). STM32H5 also has a UCPD
// peripheral, but the LL API details and pin mapping need to be re-verified against H5
// hardware before enabling this for STM32H5xx - not done as part of the stm32_libs
// consolidation/monorepo migration.
#if defined(STM32G0xx) || defined(STM32G4xx)

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
