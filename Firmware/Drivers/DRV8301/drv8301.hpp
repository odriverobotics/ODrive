#ifndef __DRV8301_HPP
#define __DRV8301_HPP

#include "stdbool.h"
#include "stdint.h"

#include <Drivers/gate_driver.hpp>
#include <Drivers/STM32/stm32_spi_arbiter.hpp>
#include <Drivers/STM32/stm32_gpio.hpp>


class Drv8301 : public GateDriverBase, public OpAmpBase {
public:
    typedef enum {
        FaultType_NoFault  = (0 << 0),  //!< No fault

        // Status Register 1
        FaultType_FETLC_OC = (1 << 0),  //!< FET Low side, Phase C Over Current fault
        FaultType_FETHC_OC = (1 << 1),  //!< FET High side, Phase C Over Current fault
        FaultType_FETLB_OC = (1 << 2),  //!< FET Low side, Phase B Over Current fault
        FaultType_FETHB_OC = (1 << 3),  //!< FET High side, Phase B Over Current fault
        FaultType_FETLA_OC = (1 << 4),  //!< FET Low side, Phase A Over Current fault
        FaultType_FETHA_OC = (1 << 5),  //!< FET High side, Phase A Over Current fault
        FaultType_OTW      = (1 << 6),  //!< Over Temperature Warning fault
        FaultType_OTSD     = (1 << 7),  //!< Over Temperature Shut Down fault
        FaultType_PVDD_UV  = (1 << 8),  //!< Power supply Vdd Under Voltage fault
        FaultType_GVDD_UV  = (1 << 9),  //!< DRV8301 Vdd Under Voltage fault
        FaultType_FAULT    = (1 << 10),

        // Status Register 2
        FaultType_GVDD_OV  = (1 << 23)  //!< DRV8301 Vdd Over Voltage fault
    } FaultType_e;

    Drv8301(Stm32SpiArbiter* spi_arbiter, Stm32Gpio ncs_gpio,
            Stm32Gpio enable_gpio, Stm32Gpio nfault_gpio)
            : spi_arbiter_(spi_arbiter), ncs_gpio_(ncs_gpio),
              enable_gpio_(enable_gpio), nfault_gpio_(nfault_gpio) {}

    /**
     * @brief Prepares the gate driver's configuration.
     *
     * If the gate driver was in ready state and the new configuration is
     * different from the old one then the gate driver will exit ready state.
     *
     * In any case changes to the configuration only take effect with a call to
     * init().
     */
    bool config(float requested_gain, float* actual_gain);
    
    /**
     * @brief Initializes the gate driver to the configuration prepared with
     * config().
     *
     * Returns true on success or false otherwise (e.g. if the gate driver is
     * not connected or not powered or if config() was not yet called).
     */
    bool init();

    /**
     * @brief Monitors the nFAULT pin.
     *
     * This must be run at an interval of <8ms from the moment the init()
     * functions starts to run, otherwise it's possible that a temporary power
     * loss is missed, leading to unwanted register values.
     * In case of power loss the nFAULT pin can be low for as little as 8ms.
     */
    void do_checks();

    /**
     * @brief Returns true if and only if the DRV8301 chip is in an initialized
     * state and ready to do switching and current sensor opamp operation.
     */
    bool is_ready() final;

    /**
     * @brief This has no effect on this driver chip because the drive stages are
     * always enabled while the chip is initialized
     */
    bool set_enabled(bool enabled) final { return true; }

    FaultType_e get_error();

    float get_midpoint() final {
        return 0.5f; // [V]
    }

    float get_max_output_swing() final {
        return 1.35f / 1.65f; // +-1.35V, normalized from a scale of +-1.65V to +-0.5
    }

private:
    enum CtrlMode_e {
        DRV8301_CtrlMode_Read = 1 << 15,   //!< Read Mode
        DRV8301_CtrlMode_Write = 0 << 15   //!< Write Mode
    };

    enum RegName_e {
        kRegNameStatus1  = 0 << 11,  //!< Status Register 1
        kRegNameStatus2  = 1 << 11,  //!< Status Register 2
        kRegNameControl1 = 2 << 11,  //!< Control Register 1
        kRegNameControl2 = 3 << 11   //!< Control Register 2
    };

    struct RegisterFile {
        uint16_t control_register_1;
        uint16_t control_register_2;
    };

    static inline uint16_t build_ctrl_word(const CtrlMode_e ctrlMode,
                                           const RegName_e regName,
                                           const uint16_t data) {
        return ctrlMode | regName | (data & 0x07FF);
    }

    /** @brief Reads data from a DRV8301 register */
    bool read_reg(const RegName_e regName, uint16_t* data);

    /** @brief Writes data to a DRV8301 register. There is no check if the write succeeded. */
    bool write_reg(const RegName_e regName, const uint16_t data);

    static const SPI_InitTypeDef spi_config_;

    // Configuration
    Stm32SpiArbiter* spi_arbiter_;
    Stm32Gpio ncs_gpio_;
    Stm32Gpio enable_gpio_;
    Stm32Gpio nfault_gpio_;

    RegisterFile regs_; //!< Current configuration. If is_ready_ is
                        //!< true then this can be considered consistent
                        //!< with the actual file on the DRV8301 chip.

    // We don't put these buffers on the stack because we place the stack in
    // a RAM section which cannot be used by DMA.
    uint16_t tx_buf_, rx_buf_;

    enum {
        kStateUninitialized,
        kStateStartupChecks,
        kStateReady,
    } state_ = kStateUninitialized;
};


#endif // __DRV8301_HPP
