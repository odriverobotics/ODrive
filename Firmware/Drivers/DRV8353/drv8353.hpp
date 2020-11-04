#ifndef __DRV8353_HPP
#define __DRV8353_HPP

#include "stdbool.h"
#include "stdint.h"

#include <Drivers/gate_driver.hpp>
#include <Drivers/STM32/stm32_spi_arbiter.hpp>
#include <Drivers/STM32/stm32_gpio.hpp>


class Drv8353 : public GateDriverBase, public OpAmpBase {
public:
    typedef enum {
        FaultType_NoFault = (0 << 0),

        // Fault Status Register 1
        FaultType_FAULT = (1 << 10),
        FaultType_VDS_OCP = (1 << 9),
        FaultType_GDF = (1 << 8),
        FaultType_UVLO = (1 << 7),
        FaultType_OTSD = (1 << 6),
        FaultType_VDS_HA = (1 << 5),
        FaultType_VDS_LA = (1 << 4),
        FaultType_VDS_HB = (1 << 3),
        FaultType_VDS_LB = (1 << 2),
        FaultType_VDS_HC = (1 << 1),
        FaultType_VDS_LC = (1 << 0),

        // Fault Status Register 2
        FaultType_SA_OC = (1 << 26),
        FaultType_SB_OC = (1 << 25),
        FaultType_SC_OC = (1 << 24),
        FaultType_OTW = (1 << 23),
        FaultType_GDUV = (1 << 22),
        FaultType_VGS_HA = (1 << 21),
        FaultType_VGS_LA = (1 << 20),
        FaultType_VGS_HB = (1 << 19),
        FaultType_VGS_LB = (1 << 18),
        FaultType_VGS_HC = (1 << 17),
        FaultType_VGS_LC = (1 << 16),
    } FaultType_e;

    Drv8353(Stm32SpiArbiter* spi_arbiter, Stm32Gpio ncs_gpio,
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
     * @brief Returns true if and only if the DRV8353 chip is in an initialized
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
        DRV8353_CtrlMode_Read = 1 << 15,   //!< Read Mode
        DRV8353_CtrlMode_Write = 0 << 15   //!< Write Mode
    };

    enum RegName_e {
        kRegNameFaultStatus1 = (0 << 11),
        kRegNameFaultStatus2 = (1 << 11),
        kRegNameDriverControl = (2 << 11),
        kRegNameGateDriveHs = (3 << 11),
        kRegNameGateDriveLs = (4 << 11),
        kRegNameOcpControl = (5 << 11),
        kRegNameCsaControl = (6 << 11)
    };

    struct RegisterFile {
        uint16_t driver_control;
        uint16_t gate_drive_hs;
        uint16_t gate_drive_ls;
        uint16_t ocp_control;
        uint16_t csa_control;
    };

    static inline uint16_t build_ctrl_word(const CtrlMode_e ctrlMode,
                                           const RegName_e regName,
                                           const uint16_t data) {
        return ctrlMode | regName | (data & 0x07FF);
    }

    /** @brief Reads data from a DRV8353 register */
    bool read_reg(const RegName_e regName, uint16_t* data);

    /** @brief Writes data to a DRV8353 register. There is no check if the write succeeded. */
    bool write_reg(const RegName_e regName, const uint16_t data);

    static const SPI_InitTypeDef spi_config_;

    // Configuration
    Stm32SpiArbiter* spi_arbiter_;
    Stm32Gpio ncs_gpio_;
    Stm32Gpio enable_gpio_;
    Stm32Gpio nfault_gpio_;

    RegisterFile regs_; //!< Current configuration. If is_ready_ is
                        //!< true then this can be considered consistent
                        //!< with the actual file on the DRV8353 chip.

    // We don't put these buffers on the stack because we place the stack in
    // a RAM section which cannot be used by DMA.
    uint16_t tx_buf_, rx_buf_;

    enum {
        kStateUninitialized,
        kStateStartupChecks,
        kStateReady,
    } state_ = kStateUninitialized;
};


#endif // __DRV8353_HPP
