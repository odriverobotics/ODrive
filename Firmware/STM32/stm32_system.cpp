
#include "stm32_system.h"
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"

#include "freertos_vars.h"

#include "stm32_system.h"


uint32_t _reboot_cookie __attribute__ ((section (".noinit")));
extern char _estack; // provided by the linker script

// Gets called from the startup assembly code
extern "C" void early_start_checks(void) {
  if(_reboot_cookie == 0xDEADFE75) {
    /* The STM DFU bootloader enables internal pull-up resistors on PB10 (AUX_H)
    * and PB11 (AUX_L), thereby causing shoot-through on the brake resistor
    * FETs and obliterating them unless external 3.3k pull-down resistors are
    * present. Pull-downs are only present on ODrive 3.5 or newer.
    * On older boards we disable DFU by default but if the user insists
    * there's only one thing left that might save it: time.
    * The brake resistor gate driver needs a certain 10V supply (GVDD) to
    * make it work. This voltage is supplied by the motor gate drivers which get
    * disabled at system reset. So over time GVDD voltage _should_ below
    * dangerous levels. This is completely handwavy and should not be relied on
    * so you are on your own on if you ignore this warning.
    *
    * This loop takes 5 cycles per iteration and at this point the system runs
    * on the internal 16MHz RC oscillator so the delay is about 2 seconds.
    */
    for (size_t i = 0; i < (16000000UL / 5UL * 2UL); ++i) {
      __NOP();
    }
    _reboot_cookie = 0xDEADBEEF;
  }

  /* We could jump to the bootloader directly on demand without rebooting
  but that requires us to reset several peripherals and interrupts for it
  to function correctly. Therefore it's easier to just reset the entire chip. */
  if(_reboot_cookie == 0xDEADBEEF) {
    _reboot_cookie = 0xCAFEFEED;  //Reset bootloader trigger
    __set_MSP((uintptr_t)&_estack);
    // http://www.st.com/content/ccc/resource/technical/document/application_note/6a/17/92/02/58/98/45/0c/CD00264379.pdf/files/CD00264379.pdf
    void (*builtin_bootloader)(void) = (void (*)(void))(*((uint32_t *)0x1FFF0004));
    builtin_bootloader();
  }

  /* The bootloader might fail to properly clean up after itself,
  so if we're not sure that the system is in a clean state we
  just reset it again */
  if(_reboot_cookie != 42) {
    _reboot_cookie = 42;
    NVIC_SystemReset();
  }
}

#define TICK_TIMER_ARR  50000
STM32_Timer_t* tick_timer_ = nullptr;

uint64_t get_ticks_us() {
    static volatile uint64_t tick_us = 0; // rolls over after ~600'000 years
    static volatile uint16_t last_cnt = 0;

    if (tick_timer_) {
        uint32_t mask = cpu_enter_critical();
        uint16_t new_cnt = tick_timer_->htim.Instance->CNT;
        uint16_t delta = new_cnt - last_cnt;
        tick_us += delta;
        last_cnt = new_cnt;
        cpu_exit_critical(mask);
    }

    return tick_us;
}

uint32_t get_ticks_ms() {
    return  get_ticks_us() / 1000ULL;
}

extern "C" uint32_t HAL_GetTick(void) {
    return get_ticks_ms();
}

// the interval between two calls to this function must be less than the
// overflow of the tick timer
void tick_callback(void* ctx) {
    (void)ctx;
    (void)get_ticks_us();
}

bool init_monotonic_clock(STM32_Timer_t* tick_timer, uint8_t tick_timer_priority) {
    if (!tick_timer) {
        return false;
    }

    tick_timer_ = tick_timer;

    uint32_t freq;
    if (!tick_timer->get_source_freq(&freq))
        return false;

    // Compute the prescaler value to have counter clock equal to 1MHz
    uint32_t prescaler = (uint32_t) ((freq / 1000000) - 1);

    /* Initialize TIMx peripheral as follow:
    + Period = [(TIMxCLK/1000) - 1]. to have a (1/1000) s time base.
    + Prescaler = (freq/1000000 - 1) to have a 1MHz counter clock.
    + ClockDivision = 0
    + Counter direction = Up
    */
    if (!tick_timer->init(TICK_TIMER_ARR, STM32_Timer_t::UP, prescaler))
        return false;
    if (!tick_timer->setup_pwm(1, nullptr, nullptr, true, true, TICK_TIMER_ARR >> 1)
        || !tick_timer->on_cc_.set<void>([](void*, uint32_t, uint32_t) { tick_callback(nullptr); }, nullptr)
        || !tick_timer->on_update_.set<void>(tick_callback, nullptr)
        || !tick_timer->enable_cc_interrupt(tick_timer_priority)
        || !tick_timer->enable_update_interrupt(tick_timer_priority)
        || !tick_timer->start()) {

        return false;
    }

    return true;
}


bool system_clock_config(void) {
    RCC_OscInitTypeDef RCC_OscInitStruct;
    RCC_ClkInitTypeDef RCC_ClkInitStruct;

    // Configure the main internal regulator output voltage
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    // Initialize the CPU, AHB and APB busses clocks 
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.LSIState = RCC_LSI_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM = 4;
    RCC_OscInitStruct.PLL.PLLN = 168;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = 7;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
        return false;

    // Initialize the CPU, AHB and APB busses clocks 
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                                |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
        return false;

    // Configure the Systick interrupt time 
    if (HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000) != 0)
        return false;

    // Configure the Systick 
    HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

    // SysTick_IRQn interrupt configuration
    HAL_NVIC_SetPriority(SysTick_IRQn, 15, 0);

    return true;
}

bool system_init(void) {
    // Reset of all peripherals, Initializes the Flash interface and the Systick.
    HAL_Init();

    __HAL_RCC_SYSCFG_CLK_ENABLE();
    __HAL_RCC_PWR_CLK_ENABLE();

    HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

    /* System interrupt init*/
    /* MemoryManagement_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(MemoryManagement_IRQn, 0, 0);
    /* BusFault_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(BusFault_IRQn, 0, 0);
    /* UsageFault_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(UsageFault_IRQn, 0, 0);
    /* SVCall_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(SVCall_IRQn, 0, 0);
    /* DebugMonitor_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(DebugMonitor_IRQn, 0, 0);
    /* PendSV_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(PendSV_IRQn, TICK_INT_PRIORITY, 0);
    /* SysTick_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(SysTick_IRQn, TICK_INT_PRIORITY, 0);

    return system_clock_config();
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
    /* User can add his own implementation to report the HAL error return state */
    while(1) 
    {
    }
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line) {
    /* User can add his own implementation to report the file name and line number,
      ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
}
#endif /* USE_FULL_ASSERT */


/* Cortex-M4 Processor Exception Handlers ------------------------------------*/

extern "C" {

void get_regs(void** stack_ptr) {
    void* volatile r0 __attribute__((unused)) = stack_ptr[0];
    void* volatile r1 __attribute__((unused)) = stack_ptr[1];
    void* volatile r2 __attribute__((unused)) = stack_ptr[2];
    void* volatile r3 __attribute__((unused)) = stack_ptr[3];

    void* volatile r12 __attribute__((unused)) = stack_ptr[4];
    void* volatile lr __attribute__((unused)) = stack_ptr[5];  // Link register
    void* volatile pc __attribute__((unused)) = stack_ptr[6];  // Program counter
    void* volatile psr __attribute__((unused)) = stack_ptr[7];  // Program status register

    volatile bool stay_looping = true;
    while(stay_looping);
}

/** @brief Entrypoint for Non maskable interrupt. */
void NMI_Handler(void) {
    // TODO: add handler
}

/** @brief Entrypoint for Hard fault interrupt. */
__attribute__((naked))
void HardFault_Handler(void) {
    __asm(
        " tst lr, #4     \n\t"
        " ite eq         \n\t"
        " mrseq r0, msp  \n\t"
        " mrsne r0, psp  \n\t"
        " b get_regs     \n\t"
    );
}

/** @brief Entrypoint for Memory management fault. */
void MemManage_Handler(void) {
    __asm( // TODO: not sure if this is a valid action in this situation
        " tst lr, #4     \n\t"
        " ite eq         \n\t"
        " mrseq r0, msp  \n\t"
        " mrsne r0, psp  \n\t"
        " b get_regs     \n\t"
    );
}

/** @brief Entrypoint for Pre-fetch fault, memory access fault. */
void BusFault_Handler(void) {
    __asm( // TODO: not sure if this is a valid action in this situation
        " tst lr, #4     \n\t"
        " ite eq         \n\t"
        " mrseq r0, msp  \n\t"
        " mrsne r0, psp  \n\t"
        " b get_regs     \n\t"
    );
}

/** @brief Entrypoint for Undefined instruction or illegal state. */
__attribute__((naked))
void UsageFault_Handler(void) {
    __asm( // TODO: not sure if this is a valid action in this situation
        " tst lr, #4     \n\t"
        " ite eq         \n\t"
        " mrseq r0, msp  \n\t"
        " mrsne r0, psp  \n\t"
        " b get_regs     \n\t"
    );
}

/** @brief Entrypoint for Debug monitor. */
void DebugMon_Handler(void) {
    // TODO: add debug support code (allows adding breakpoints while the CPU is running)
}

/** @brief Entrypoint for System tick timer. */
void SysTick_Handler(void) {
    osSystickHandler();
}

}
