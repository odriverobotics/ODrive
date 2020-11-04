
#include "drv8353.hpp"
#include "utils.hpp"
#include "cmsis_os.h"
#include "board.h"

const SPI_InitTypeDef Drv8353::spi_config_ = {
    .Mode = SPI_MODE_MASTER,
    .Direction = SPI_DIRECTION_2LINES,
    .DataSize = SPI_DATASIZE_16BIT,
    .CLKPolarity = SPI_POLARITY_LOW,
    .CLKPhase = SPI_PHASE_2EDGE,
    .NSS = SPI_NSS_SOFT,
    .BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16,
    .FirstBit = SPI_FIRSTBIT_MSB,
    .TIMode = SPI_TIMODE_DISABLE,
    .CRCCalculation = SPI_CRCCALCULATION_DISABLE,
    .CRCPolynomial = 10,
};

bool Drv8353::config(float requested_gain, float* actual_gain) {
    // Calculate gain setting: Snap down to have equal or larger range as
    // requested or largest possible range otherwise

    uint16_t gain_setting = 3;
    float gain_choices[] = {5.0f, 10.0f, 20.0f, 40.0f};
    while (gain_setting && (gain_choices[gain_setting] > requested_gain)) {
        gain_setting--;
    }

    if (actual_gain) {
        *actual_gain = gain_choices[gain_setting];
    }

    // For reference:
    // Rds(on) of NTMFS5C628NL is ~3mOhm at 160A, 100°C and we have two in parallel
    // Rshunt of ODrive v4 is 1mOhm

    RegisterFile new_config;

    new_config.driver_control =
          (0b1 << 10) // overcurrent protection of any half bridge shuts down all half bridges
        | (0b0 << 9) // enable Vcp and Vgls undervoltage lockout fault
        | (0b0 << 8) // enable gate drive fault
        | (0b1 << 7) // report overtemperature warning on nFAULT
        | (0b00 << 5) // 6x PWM mode
        | (0b0 << 4) // [applies to 1x PWM mode only]
        | (0b0 << 3) // [applies to 1x PWM mode only]
        | (0b0 << 2) // don't coast
        | (0b0 << 1) // don't brake
        | (0b0 << 0); // don't clear faults

    new_config.gate_drive_hs =
          (0b011 << 8) // don't lock registers
        | (0b1111 << 4) // 1A source current on high side FET drivers
        | (0b1111 << 0); // 2A sink current on high side FET drivers

    new_config.gate_drive_ls =
          (0b1 << 10) // clear overcurrent faults at next PWM input or t_retry (whichever comes first) - this has no effect since we use latched overcurrent fault mode
        | (0b01 << 8) // 1000 ns peak gate current drive time
        | (0b1111 << 4) // 1A source current on low side FET drivers
        | (0b1111 << 0); // 2A sink current on low side FET drivers

    new_config.ocp_control =
          (0b0 << 10) // retry time for Vds and shunt overcurrent protection: 8ms
        | (0b01 << 8) // 100ns deadtime (we configure the STM timer to do 120ns deadtime as well)
        | (0b00 << 6) // overcurrent causes a latching fault (no retry)
        | (0b10 << 4) // overcurrent deglitch of 4us
        | (0b0101 << 0); // Vds trip level 0.25 V (approx. ~133A per MOSFET at 100°C)

    new_config.csa_control =
          (0b0 << 10) // measure current across SPx to SNx
        | (0b1 << 9) // use Vref/2 as sense amplifier reference voltage
        | (0b0 << 8) // measure Vds across SHx to SPx
        | (gain_setting << 6) // select gain
        | (0b0 << 5) // sense overcurrent fault enabled
        | (0b000 << 2) // normal current sense operation on all three phases
        | (0b00 << 0); // sense overcurrent protection at 0.25V sense input (corresponds to ~250A)

    bool regs_equal = (regs_.driver_control == new_config.driver_control)
                   && (regs_.gate_drive_hs == new_config.gate_drive_hs)
                   && (regs_.gate_drive_ls == new_config.gate_drive_ls)
                   && (regs_.ocp_control == new_config.ocp_control)
                   && (regs_.csa_control == new_config.csa_control);

    if (!regs_equal) {
        regs_ = new_config;
        state_ = kStateUninitialized;
        enable_gpio_.write(false);
    }

    return true;
}

bool Drv8353::init() {
    uint16_t val;

    if (state_ == kStateReady) {
        return true;
    }

    // Reset DRV chip. The enable pin also controls the SPI interface, not only
    // the driver stages.
    enable_gpio_.write(false);
    delay_us(100); // t_rst, max = 40us
    state_ = kStateUninitialized; // make is_ready() ignore transient errors before registers are set up
    enable_gpio_.write(true);
    osDelay(2); // t_wake, max = 1ms

    // Write current configuration
    bool did_write_regs = write_reg(kRegNameDriverControl, regs_.driver_control)
                       && write_reg(kRegNameGateDriveHs, regs_.gate_drive_hs)
                       && write_reg(kRegNameGateDriveLs, regs_.gate_drive_ls)
                       && write_reg(kRegNameOcpControl, regs_.ocp_control)
                       && write_reg(kRegNameCsaControl, regs_.csa_control);
    if (!did_write_regs) {
        return false;
    }

    // Wait for configuration to be applied
    delay_us(100);
    state_ = kStateStartupChecks;

    bool did_read_regs = read_reg(kRegNameDriverControl, &val) && (val == regs_.driver_control)
                      && read_reg(kRegNameGateDriveHs, &val) && (val == regs_.gate_drive_hs)
                      && read_reg(kRegNameGateDriveLs, &val) && (val == regs_.gate_drive_ls)
                      && read_reg(kRegNameOcpControl, &val) && (val == regs_.ocp_control)
                      && read_reg(kRegNameCsaControl, &val) && (val == regs_.csa_control);
    if (!did_read_regs) {
        return false;
    }
    

    if (get_error() != FaultType_NoFault) {
        return false;
    }

    // There could have been an nFAULT edge meanwhile. In this case we shouldn't
    // consider the driver ready.
    CRITICAL_SECTION() {
        if (state_ == kStateStartupChecks) {
            state_ = kStateReady;
        }
    }

    return state_ == kStateReady;
}

void Drv8353::do_checks() {
    if (state_ != kStateUninitialized && !nfault_gpio_.read()) {
        state_ = kStateUninitialized;
    }
}

bool Drv8353::is_ready() {
    return state_ == kStateReady;
}

Drv8353::FaultType_e Drv8353::get_error() {
    uint16_t fault1, fault2;

    if (!read_reg(kRegNameFaultStatus1, &fault1) ||
        !read_reg(kRegNameFaultStatus2, &fault2)) {
        return (FaultType_e)0xffffffff;
    }

    return (FaultType_e)((uint32_t)fault1 | ((uint32_t)fault2 << 16));
}

bool Drv8353::read_reg(const RegName_e regName, uint16_t* data) {
    tx_buf_ = build_ctrl_word(DRV8353_CtrlMode_Read, regName, 0);
    rx_buf_ = 0xffff;
    if (!spi_arbiter_->transfer(spi_config_, ncs_gpio_, (uint8_t *)(&tx_buf_), (uint8_t *)(&rx_buf_), 1, 1000)) {
        return false;
    }

    delay_us(1);

    if (data) {
        *data = rx_buf_ & 0x07FF;
    }

    return true;
}

bool Drv8353::write_reg(const RegName_e regName, const uint16_t data) {
    // Do blocking write
    tx_buf_ = build_ctrl_word(DRV8353_CtrlMode_Write, regName, data);
    if (!spi_arbiter_->transfer(spi_config_, ncs_gpio_, (uint8_t *)(&tx_buf_), nullptr, 1, 1000)) {
        return false;
    }
    delay_us(1);

    return true;
}
