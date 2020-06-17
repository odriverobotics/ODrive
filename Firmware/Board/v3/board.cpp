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

OnboardThermistorCurrentLimiter m0_fet_thermistor{
    15, // adc_channel
    &fet_thermistor_poly_coeffs[0], // coefficients
    fet_thermistor_num_coeffs // num_coeffs
};

OnboardThermistorCurrentLimiter m1_fet_thermistor{
#if HW_VERSION_MAJOR == 3 && HW_VERSION_MINOR >= 3
    4, // adc_channel
#else
    1, // adc_channel
#endif
    &fet_thermistor_poly_coeffs[0], // coefficients
    fet_thermistor_num_coeffs // num_coeffs
};

Motor m0{
    &htim1, // timer
    TIM_1_8_PERIOD_CLOCKS, // control_deadline
    1.0f / SHUNT_RESISTANCE, // shunt_conductance [S]
    m0_gate_driver, // gate_driver
    m0_gate_driver // opamp
};

Motor m1{
    &htim8, // timer
    (3 * TIM_1_8_PERIOD_CLOCKS) / 2, // control_deadline
    1.0f / SHUNT_RESISTANCE, // shunt_conductance [S]
    m1_gate_driver, // gate_driver
    m1_gate_driver // opamp
};

Motor* motors[AXIS_COUNT] = {&m0, &m1};
OnboardThermistorCurrentLimiter* fet_thermistors[AXIS_COUNT] = {&m0_fet_thermistor, &m1_fet_thermistor};


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
