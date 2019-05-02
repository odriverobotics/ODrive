#ifndef __DRV8301_HPP
#define __DRV8301_HPP

#include "drv8301.h"

#include <stm32_gpio.hpp>
#include <stm32_spi.hpp>
#include <devices.hpp>
#include <fibre/protocol.hpp>

class DRV8301_t : public GateDriver_t, public OpAmp_t {
public:
    STM32_SPI_t* spi_;
    STM32_GPIO_t* chip_select_gpio;
    STM32_GPIO_t* enable_gpio;
    GPIO_t* nfault_gpio;
    DRV8301_Obj gate_driver_;
    float gain_ = 1.0f;
    bool is_setup_ = false;

    DRV8301_t(STM32_SPI_t* spi, STM32_GPIO_t* chip_select_gpio, STM32_GPIO_t* enable_gpio, GPIO_t* nfault_gpio) :
        spi_(spi), chip_select_gpio(chip_select_gpio), enable_gpio(enable_gpio), nfault_gpio(nfault_gpio),
        gate_driver_{
            .spiHandle = &spi_->hspi,
            .EngpioHandle = enable_gpio->port,
            .EngpioNumber = (uint16_t)(1U << enable_gpio->pin_number),
            .nCSgpioHandle = chip_select_gpio->port,
            .nCSgpioNumber = (uint16_t)(1U << chip_select_gpio->pin_number)
        } {
    }
    
    bool init() final;
    bool set_enabled(bool enabled) final { return true; }
    bool check_fault();
    uint32_t get_error() final;
    float set_gain(float requested_gain) final;
    float get_gain() final;

    float get_midpoint() final {
        return 0.5f; // [V]
    }

    float get_max_output_swing() final {
        return 1.35f / 1.65f; // +-1.35V, normalized from a scale of +-1.65V to +-0.5
    }

private:
    DRV_SPI_8301_Vars_t local_regs_; // Local view of DRV registers (initialized by init())
    DRV8301_FaultType_e drv_fault_ = DRV8301_FaultType_NoFault;

    // Communication protocol definitions
    auto make_protocol_definitions() {
        return make_protocol_member_list(
            make_protocol_ro_property("drv_fault", &drv_fault_)
            // make_protocol_ro_property("status_reg_1", &local_regs_.Stat_Reg_1_Value),
            // make_protocol_ro_property("status_reg_2", &local_regs_.Stat_Reg_2_Value),
            // make_protocol_ro_property("ctrl_reg_1", &local_regs_.Ctrl_Reg_1_Value),
            // make_protocol_ro_property("ctrl_reg_2", &local_regs_.Ctrl_Reg_2_Value)
        );
    }
};

#endif // __DRV8301_HPP