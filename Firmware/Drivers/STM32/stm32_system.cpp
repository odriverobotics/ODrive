
#include "stm32_system.h"

uint32_t irq_counters[254]; // 14 core interrupts, 240 NVIC interrupts


// TODO: this is ODrive Motor Controller specific and should technically not live in this file
static inline void panic_pwm_off() {
    // disable motor/brake PWM
    TIM1->BDTR &= ~(TIM_BDTR_AOE_Msk | TIM_BDTR_MOE_Msk);
    TIM8->BDTR &= ~(TIM_BDTR_AOE_Msk | TIM_BDTR_MOE_Msk);
}

extern "C" {

/**
 * @brief This function handles Non maskable interrupt.
 */
void NMI_Handler(void) {
    COUNT_IRQ(NonMaskableInt_IRQn);
}

__attribute__((used))
void get_regs(void** stack_ptr) {
    panic_pwm_off();

    void* volatile r0 __attribute__((unused)) = stack_ptr[0];
    void* volatile r1 __attribute__((unused)) = stack_ptr[1];
    void* volatile r2 __attribute__((unused)) = stack_ptr[2];
    void* volatile r3 __attribute__((unused)) = stack_ptr[3];

    void* volatile r12 __attribute__((unused)) = stack_ptr[4];
    void* volatile lr __attribute__((unused)) = stack_ptr[5];  // Link register
    void* volatile pc __attribute__((unused)) = stack_ptr[6];  // Program counter
    void* volatile psr __attribute__((unused)) = stack_ptr[7];  // Program status register

    void* volatile cfsr __attribute__((unused)) = (void*)SCB->CFSR; // Configurable fault status register
    void* volatile cpacr __attribute__((unused)) = (void*)SCB->CPACR;
    void* volatile fpccr __attribute__((unused)) = (void*)FPU->FPCCR;

    volatile bool preciserr __attribute__((unused)) = (uint32_t)cfsr & 0x200;
    volatile bool ibuserr __attribute__((unused)) = (uint32_t)cfsr & 0x100;

#if defined(FLASH_SR_DBECCERR)
    volatile bool flash_dbeccerr1 __attribute__((unused)) = FLASH->SR1 & FLASH_SR_DBECCERR; // True if a double (uncorrectable) error occurred on bank 1 read access
    volatile uint32_t flash_ecc_fa1 __attribute__((unused)) = FLASH->ECC_FA1; // Word number that caused the bank 1 bus error (the address is 16*word_number + BANK0_START)
    volatile bool flash_dbeccerr2 __attribute__((unused)) = FLASH->SR2 & FLASH_SR_DBECCERR; // True if a double (uncorrectable) error occurred on bank 2 read access
    volatile uint32_t flash_ecc_fa2 __attribute__((unused)) = FLASH->ECC_FA2; // Word number that caused the bank 2 bus error (the address is 16*word_number + BANK1_START)
#endif

    volatile int stay_looping = 1;
    while(stay_looping);
}

/**
  * @brief This function handles Hard fault interrupt.
  */
__attribute__((naked))
void HardFault_Handler(void)
{
  __asm(
    " tst lr, #4     \n\t"
    " ite eq         \n\t"
    " mrseq r0, msp  \n\t"
    " mrsne r0, psp  \n\t"
    " b get_regs     \n\t"
  );
}

/**
 * @brief This function handles Memory management fault.
 */
void MemManage_Handler(void) {
    COUNT_IRQ(MemoryManagement_IRQn);
    for (;;) {
        panic_pwm_off();
    }
}

/**
 * @brief This function handles Pre-fetch fault, memory access fault.
 */
void BusFault_Handler(void) {
    COUNT_IRQ(BusFault_IRQn);
    for (;;) {
        panic_pwm_off();
    }
}

/**
 * @brief This function handles Undefined instruction or illegal state.
 */
void UsageFault_Handler(void) {
    COUNT_IRQ(UsageFault_IRQn);
    for (;;) {
        panic_pwm_off();
    }
}

/**
 * @brief This function handles Debug monitor.
 */
void DebugMon_Handler(void) {
    COUNT_IRQ(DebugMonitor_IRQn);
}

}
