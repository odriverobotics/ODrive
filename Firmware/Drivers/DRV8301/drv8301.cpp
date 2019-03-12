
#include "drv8301.hpp"

#include <gpio.hpp>
#include <devices.hpp>

#include <algorithm>

bool DRV8301_t::init() {
    if (!is_setup_) {
        //if (!spi.init()) // handled in main init function
        //    return false;
        if (chip_select_gpio)
            if (!chip_select_gpio->init(GPIO_t::OUTPUT, GPIO_t::NO_PULL, true))
                return false;
        if (enable_gpio)
            if (!enable_gpio->init(GPIO_t::OUTPUT, GPIO_t::NO_PULL, false))
                return false;
        if (nfault_gpio)
            if (!nfault_gpio->init(GPIO_t::INPUT, GPIO_t::PULL_DOWN, false))
                return false;

        DRV8301_enable(&gate_driver_);
        DRV8301_setupSpi(&gate_driver_, &local_regs_);
        is_setup_ = true;
    }

    return true;
}

bool DRV8301_t::check_fault() {
    if (nfault_gpio) {
        if (!nfault_gpio->read()) {
            // Update DRV Fault Code
            drv_fault_ = DRV8301_getFaultType(&gate_driver_);
            // Update/Cache all SPI device registers
            // DRV_SPI_8301_Vars_t* local_regs = &gate_driver_regs_;
            // local_regs->RcvCmd = true;
            // DRV8301_readData(&gate_driver_, local_regs);
            return false;
        }
    }
    return true;
}

float DRV8301_t::set_gain(float requested_gain) {
    // for reference:
    // 20V/V on 500uOhm gives a range of +/- 150A
    // 40V/V on 500uOhm gives a range of +/- 75A
    // 20V/V on 666uOhm gives a range of +/- 110A
    // 40V/V on 666uOhm gives a range of +/- 55A

    // Snap down to have equal or larger range as requested or largest possible range otherwise

    // Decoding array for snapping gain
    std::array<std::pair<float, DRV8301_ShuntAmpGain_e>, 4> gain_choices = { 
        std::make_pair(10.0f, DRV8301_ShuntAmpGain_10VpV),
        std::make_pair(20.0f, DRV8301_ShuntAmpGain_20VpV),
        std::make_pair(40.0f, DRV8301_ShuntAmpGain_40VpV),
        std::make_pair(80.0f, DRV8301_ShuntAmpGain_80VpV)
    };

    // We use lower_bound in reverse because it snaps up by default, we want to snap down.
    auto gain_snap_down = std::lower_bound(gain_choices.crbegin(), gain_choices.crend(), requested_gain, 
    [](std::pair<float, DRV8301_ShuntAmpGain_e> pair, float val){
        return (bool)(pair.first > val);
    });

    // If we snap to outside the array, clip to smallest val
    if (gain_snap_down == gain_choices.crend())
        --gain_snap_down;
    
    local_regs_.Ctrl_Reg_1.OC_MODE = DRV8301_OcMode_LatchShutDown;
    // Overcurrent set to approximately 150A at 100degC. This may need tweaking.
    local_regs_.Ctrl_Reg_1.OC_ADJ_SET = DRV8301_VdsLevel_0p730_V;
    local_regs_.Ctrl_Reg_2.GAIN = gain_snap_down->second;

    local_regs_.SndCmd = true;
    DRV8301_writeData(&gate_driver_, &local_regs_);
    local_regs_.RcvCmd = true;
    DRV8301_readData(&gate_driver_, &local_regs_);

    return (gain_ = gain_snap_down->first);
}

float DRV8301_t::get_gain() {
    return gain_;
}
