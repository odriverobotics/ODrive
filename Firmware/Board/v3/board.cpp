/*
* @brief Contains board specific variables and initialization functions
*/

#include <board.h>

Stm32SpiArbiter spi3_arbiter{&hspi3};
Stm32SpiArbiter& ext_spi_arbiter = spi3_arbiter;

Drv8301 m0_gate_driver{
    &spi3_arbiter,
    {M0_nCS_GPIO_Port, M0_nCS_Pin}, // nCS
    {EN_GATE_GPIO_Port, EN_GATE_Pin}, // EN pin (shared between both motors)
    {nFAULT_GPIO_Port, nFAULT_Pin} // nFAULT pin (shared between both motors)
};

Drv8301 m1_gate_driver{
    &spi3_arbiter,
    {M1_nCS_GPIO_Port, M1_nCS_Pin}, // nCS
    {EN_GATE_GPIO_Port, EN_GATE_Pin}, // EN pin (shared between both motors)
    {nFAULT_GPIO_Port, nFAULT_Pin} // nFAULT pin (shared between both motors)
};

const float fet_thermistor_poly_coeffs[] =
    {363.93910201f, -462.15369634f, 307.55129571f, -27.72569531f};
const size_t fet_thermistor_num_coeffs = sizeof(fet_thermistor_poly_coeffs)/sizeof(fet_thermistor_poly_coeffs[1]);

OnboardThermistorCurrentLimiter fet_thermistors[AXIS_COUNT] = {
    {
        15, // adc_channel
        &fet_thermistor_poly_coeffs[0], // coefficients
        fet_thermistor_num_coeffs // num_coeffs
    }, {
#if HW_VERSION_MAJOR == 3 && HW_VERSION_MINOR >= 3
        4, // adc_channel
#else
        1, // adc_channel
#endif
        &fet_thermistor_poly_coeffs[0], // coefficients
        fet_thermistor_num_coeffs // num_coeffs
    }
};

Motor motors[AXIS_COUNT] = {
    {
        &htim1, // timer
        TIM_1_8_PERIOD_CLOCKS, // control_deadline
        1.0f / SHUNT_RESISTANCE, // shunt_conductance [S]
        m0_gate_driver, // gate_driver
        m0_gate_driver // opamp
    },
    {
        &htim8, // timer
        (3 * TIM_1_8_PERIOD_CLOCKS) / 2, // control_deadline
        1.0f / SHUNT_RESISTANCE, // shunt_conductance [S]
        m1_gate_driver, // gate_driver
        m1_gate_driver // opamp
    }
};

Encoder encoders[AXIS_COUNT] = {
    {
        &htim3, // timer
        {M0_ENC_Z_GPIO_Port, M0_ENC_Z_Pin}, // index_gpio
        {M0_ENC_A_GPIO_Port, M0_ENC_A_Pin}, // hallA_gpio
        {M0_ENC_B_GPIO_Port, M0_ENC_B_Pin}, // hallB_gpio
        {M0_ENC_Z_GPIO_Port, M0_ENC_Z_Pin}, // hallC_gpio
        &spi3_arbiter // spi_arbiter
    },
    {
        &htim4, // timer
        {M1_ENC_Z_GPIO_Port, M1_ENC_Z_Pin}, // index_gpio
        {M1_ENC_A_GPIO_Port, M1_ENC_A_Pin}, // hallA_gpio
        {M1_ENC_B_GPIO_Port, M1_ENC_B_Pin}, // hallB_gpio
        {M1_ENC_Z_GPIO_Port, M1_ENC_Z_Pin}, // hallC_gpio
        &spi3_arbiter // spi_arbiter
    }
};


void board_init() {
    // Ensure that debug halting of the core doesn't leave the motor PWM running
    __HAL_DBGMCU_FREEZE_TIM1();
    __HAL_DBGMCU_FREEZE_TIM8();
    __HAL_DBGMCU_FREEZE_TIM13();

    /*
    * Initial intention of the synchronization:
    * Synchronize TIM1, TIM8 and TIM13 such that:
    *  1. The triangle waveform of TIM1 leads the triangle waveform of TIM8 by a
    *     90° phase shift.
    *  2. The timer update events of TIM1 and TIM8 are symmetrically interleaved.
    *  3. Each TIM13 reload coincides with a TIM1 lower update event.
    * 
    * However right now this synchronization only ensures point (1) and (3) but because
    * TIM1 and TIM3 only trigger an update on every third reload, this does not
    * allow for (2).
    * 
    * TODO: revisit the timing topic in general.
    * 
    */
    Stm32Timer::start_synchronously<3>(
        {&htim1, &htim8, &htim13},
        {TIM_1_8_PERIOD_CLOCKS / 2 - 1 * 128 /* TODO: explain why this offset */, 0, TIM_1_8_PERIOD_CLOCKS / 2 - 1 * 128}
    );
}


extern "C" {

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi) {
    HAL_SPI_TxRxCpltCallback(hspi);
}

void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi) {
    HAL_SPI_TxRxCpltCallback(hspi);
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi) {
    if (hspi == &hspi3) {
        spi3_arbiter.on_complete();
    }
}



void TIM1_UP_TIM10_IRQHandler(void) {
    __HAL_TIM_CLEAR_IT(&htim1, TIM_IT_UPDATE);
    motors[0].tim_update_cb();
}

void TIM8_UP_TIM13_IRQHandler(void) {
    __HAL_TIM_CLEAR_IT(&htim8, TIM_IT_UPDATE);
    motors[1].tim_update_cb();
}

}