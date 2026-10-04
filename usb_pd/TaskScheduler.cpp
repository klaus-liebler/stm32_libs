
#include "TaskScheduler.h"
#include <cstring>
#include "log.h"
#include "hal_header_selector.h"
#include "common.hh"
#include "hw_config_assert.hh"
#include "tx_api.h"
//#define USE_TIMER6_FOR_SCHEDULER
#define USE_TIMER7_FOR_SCHEDULER

#ifdef USE_TIMER6_FOR_SCHEDULER
    // NOTE: TIM6_DAC_IRQn/_IRQHandler is STM32G4 naming (TIM6 shares its vector with DAC there).
    // On STM32H5 the vector is plain TIM6_IRQn/TIM6_IRQHandler - adjust if this branch is enabled there.
    #define SCHEDULER_TIMER_NAME "TIM6"
    #define SCHEDULER_TIMER TIM6
    #define SCHEDULER_TIMER_IRQn TIM6_DAC_IRQn
    #define SCHEDULER_TIMER_IRQ_HANDLER TIM6_DAC_IRQHandler
    #define ENABLE_TIMER_CLOCK() __HAL_RCC_TIM6_CLK_ENABLE()
#elif defined(USE_TIMER7_FOR_SCHEDULER)
    #define SCHEDULER_TIMER_NAME "TIM7"
    #define SCHEDULER_TIMER TIM7
    #define SCHEDULER_TIMER_IRQn TIM7_IRQn
    #define SCHEDULER_TIMER_IRQ_HANDLER TIM7_IRQHandler
    #define ENABLE_TIMER_CLOCK() __HAL_RCC_TIM7_CLK_ENABLE()
#else
    #error "No timer selected for Task Scheduler"
#endif
#define _countof(a) (sizeof(a) / sizeof(*(a)))

static TIM_HandleTypeDef schedulerTimerHandle{};

static uint32_t getTimerClockHz() {
    uint32_t pclk1 = HAL_RCC_GetPCLK1Freq();
#ifdef RCC_CFGR_PPRE1
    if ((RCC->CFGR & RCC_CFGR_PPRE1) != RCC_CFGR_PPRE1_DIV1)
        pclk1 *= 2;
#endif
    return pclk1;
}

// Zeitbasis in us aus dem ThreadX-Tick (SysTick, TX_TIMER_TICKS_PER_SECOND) plus der Position
// innerhalb des laufenden Ticks (SysTick->VAL zaehlt von LOAD auf 0 herunter). Ersetzt das
// fruehere DWT-basierte micros(): dieses lief schon nach 2^32 Takten (~27s bei 160MHz) ueber,
// nicht nach 2^32 us -- die Wrap-Arithmetik unten (timeDifference/hasExpired) setzt aber einen
// vollen 32-Bit-us-Zaehler voraus. Hier wird ticks * us_per_tick in uint32_t gerechnet und
// laeuft damit exakt modulo 2^32 us (~71 min) ueber, lueckenlos auch ueber den Ueberlauf von
// tx_time_get() hinweg. Setzt einen laufenden ThreadX-Kernel voraus (vor tx_kernel_enter() steht
// tx_time_get() auf 0) -- der Scheduler wird erst danach gestartet (s. USBPDControl::Start()).
uint32_t TaskScheduler::now() {
    constexpr uint32_t us_per_tick = 1000000UL / TX_TIMER_TICKS_PER_SECOND;
    const uint32_t cycles_per_us = SystemCoreClock / 1000000UL;

    // Unter gesperrten Interrupts, damit Tick-Zaehler und SysTick->VAL zusammenpassen. Ist der
    // SysTick bereits uebergelaufen, sein Interrupt aber noch nicht abgearbeitet (PENDSTSET --
    // z.B. weil dieser Code selbst im hoeher priorisierten UCPD-/TIM7-Interrupt laeuft), hinkt
    // tx_time_get() um einen Tick hinterher: dann den Tick selbst mitzaehlen und VAL neu lesen
    // (ein erstes VAL kann noch von vor dem Ueberlauf stammen).
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    uint32_t ticks = (uint32_t)tx_time_get();
    uint32_t val = SysTick->VAL;
    if ((SCB->ICSR & SCB_ICSR_PENDSTSET_Msk) != 0U) {
        ticks += 1U;
        val = SysTick->VAL;
    }
    const uint32_t load = SysTick->LOAD;
    __set_PRIMASK(primask);

    return ticks * us_per_tick + (load - val) / cycles_per_us;
}

inline static uint32_t timeDifference(uint32_t time, uint32_t now) {
    return time - now;
}

inline static bool hasExpired(uint32_t time, uint32_t now) {
    return (time - now) > 0xfff0000;
}

TaskScheduler Scheduler{};

TaskScheduler::TaskScheduler() : numScheduledTasks(-1) { }

void TaskScheduler::start() {
    numScheduledTasks = 0;
    ENABLE_TIMER_CLOCK();
    uint32_t prescaler = ((getTimerClockHz() + 500000) / 1000000)-1;
    log_info("Starting Scheduler using Timer %s with timer frequency %d and a prescaler %d", SCHEDULER_TIMER_NAME, getTimerClockHz(), prescaler);
    schedulerTimerHandle.Instance = SCHEDULER_TIMER;
    schedulerTimerHandle.Init.Prescaler = prescaler;
    schedulerTimerHandle.Init.CounterMode = TIM_COUNTERMODE_UP;
    schedulerTimerHandle.Init.Period = 0xffff;
    schedulerTimerHandle.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    schedulerTimerHandle.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    HAL_TIM_Base_Init(&schedulerTimerHandle);

    // one-pulse mode
    SCHEDULER_TIMER->CR1 |= TIM_CR1_OPM;
    // disable ARR buffering
    SCHEDULER_TIMER->CR1 &= ~TIM_CR1_ARPE_Msk;

    __HAL_TIM_CLEAR_IT(&schedulerTimerHandle, TIM_IT_UPDATE);
    __HAL_TIM_ENABLE_IT(&schedulerTimerHandle, TIM_IT_UPDATE);

    HAL_NVIC_SetPriority(SCHEDULER_TIMER_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(SCHEDULER_TIMER_IRQn);

    // Selbstkonsistenz-Check: faengt z.B. eine falsche Priority-Grouping-Konfiguration ab, bei
    // der NVIC_EnableIRQ() die Freigabe still nicht wirksam werden laesst.
    HW_CONFIG_ASSERT(NVIC_GetEnableIRQ(SCHEDULER_TIMER_IRQn) != 0,
                      SCHEDULER_TIMER_NAME " NVIC-Interrupt nach NVIC_EnableIRQ() nicht aktiv");
}

void TaskScheduler::scheduleTaskAfter(TaskFunction task, uint32_t delay) {
    scheduleTaskAt(task, now() + delay);
}

void TaskScheduler::scheduleTaskAt(TaskFunction task, uint32_t time) {
    if (numScheduledTasks == -1) {
        log_fatal("Task Scheduler has not been initialized");
        __builtin_trap();
    }

    if (numScheduledTasks >= static_cast<int>(_countof(scheduledItems))) {
        __builtin_trap();
    }

    // pause timer
    SCHEDULER_TIMER->CR1 &= ~TIM_CR1_CEN_Msk;
    uint32_t now = TaskScheduler::now();

    // find insertion index
    // (tasks are sorted by time but time wraps around)
    int index = 0;
    uint32_t delay = timeDifference(time, now);
    while (index < numScheduledTasks) {
        if (delay < timeDifference(scheduledItems[index].time, now))
            break;
        index += 1;
    }

    // move elements after insertion point if needed
    if (index < numScheduledTasks) {
        memmove(&scheduledItems[index + 1], &scheduledItems[index],
            sizeof(scheduledItems[0]) * (numScheduledTasks - index));
    }

    numScheduledTasks += 1;

    // set values
    scheduledItems[index].time = time;
    scheduledItems[index].task = task;

    checkPendingTasks();
}

void TaskScheduler::cancelTask(TaskFunction task) {
    if (numScheduledTasks == -1)
        return;

    // pause timer
    SCHEDULER_TIMER->CR1 &= ~TIM_CR1_CEN_Msk;

    // find task to remove
    int index;
    for (index = 0; index < numScheduledTasks; index += 1) {
        if (scheduledItems[index].task == task)
            break;
    }

    if (index < numScheduledTasks) {
        numScheduledTasks -= 1;

        // move remaining elements if needed
        if (index < numScheduledTasks) {
            memmove(&scheduledItems[index], &scheduledItems[index + 1],
                sizeof(scheduledItems[0]) * (numScheduledTasks - index));
        }
    }

    checkPendingTasks();
}

void TaskScheduler::cancelAllTasks() {
    if (numScheduledTasks == -1)
        return;

    // pause timer
    SCHEDULER_TIMER->CR1 &= ~TIM_CR1_CEN_Msk;
    numScheduledTasks = 0;
}

void TaskScheduler::checkPendingTasks() {

    uint32_t now = 0;

    while (true) {
        if (numScheduledTasks == 0)
            return; // no pending tasks

        now = TaskScheduler::now();
        if (!hasExpired(scheduledItems[0].time, now))
            break; // next task has not yet expired

        TaskFunction task = scheduledItems[0].task;
        numScheduledTasks -= 1;

        // move remaining elements if needed
        if (numScheduledTasks > 0) {
            memmove(&scheduledItems[0], &scheduledItems[1],
                sizeof(scheduledItems[0]) * numScheduledTasks);
        }

        // execute task
        task();
    }

    uint32_t delayToFirstTask = timeDifference(scheduledItems[0].time, TaskScheduler::now());
    //log_info("Delay to next first task %d", delayToFirstTask);
    if (delayToFirstTask > 0xffff)
        delayToFirstTask = 0xffff;

    // restart timer
    SCHEDULER_TIMER->CNT = 0;
    SCHEDULER_TIMER->ARR = delayToFirstTask;
    SCHEDULER_TIMER->CR1 |= TIM_CR1_CEN;
}

void TaskScheduler::onInterrupt() {
    Scheduler.checkPendingTasks();
}

extern "C" void SCHEDULER_TIMER_IRQ_HANDLER(void) {
    if (__HAL_TIM_GET_FLAG(&schedulerTimerHandle, TIM_FLAG_UPDATE) != RESET) {
        __HAL_TIM_CLEAR_IT(&schedulerTimerHandle, TIM_IT_UPDATE);
        Scheduler.onInterrupt();
    } else {
        __HAL_TIM_CLEAR_IT(&schedulerTimerHandle, TIM_IT_UPDATE);
    }
}