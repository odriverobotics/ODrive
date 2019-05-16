#ifndef __STM32_WWDG_HPP
#define __STM32_WWDG_HPP

#include "stm32_system.h"
#include <math.h>

// todo: move to platform-independent file
class Watchdog {
public:
    /**
     * @brief Starts the watchdog.
     */
    virtual bool start() = 0;

    /**
     * @brief Resets the watchdog.
     */
    virtual bool reset() = 0;
};

// todo: move to platform-independent file
template<typename TBaseWatchdog, TBaseWatchdog& watchdog, size_t N>
class MultiWatchdog {
public:
    bool start() { return watchdog.start(); }
    
    bool reset(size_t slot) {
        cookies |= (1 << slot); // feed it cookies
        if (cookies == (1 << N) - 1) {
            cookies = 0;
            return watchdog.reset();
        } else {
            return true;
        }
    }

private:
    uint32_t cookies = 0;
};

template<uint32_t INSTANCE>
class STM32_WWDG_t : public Watchdog {
public:
    WWDG_HandleTypeDef hwwdg = { .Instance = (WWDG_TypeDef*) INSTANCE };
    constexpr static const float prescaler = 1; // TODO: make adjustable

    /**
     * @brief Configures the minimum and maximum refresh intervals.
     * 
     * The new settings will come into effect at the next refresh.
     * The actual window will be slightly larger because of finite watchdog
     * resolution.
     * 
     * @param min_interval: The minimum interval after which the watchdog may be
     *        reset the next time.
     * @param max_interval: The maximum interval before which the watchdog must
     *        be reset the next time. If the next reset does not occur, an
     *        interrupt is fired ("last call") and shortly after the chip is reset.
     */
    bool config(float min_interval, float max_interval) {
        float tick_interval = 1.0f / ((float)HAL_RCC_GetPCLK1Freq() / 4096.0f / prescaler);
        uint32_t max_ticks = static_cast<uint32_t>(ceilf(max_interval / tick_interval));
        uint32_t min_ticks = static_cast<uint32_t>(floorf(min_interval / tick_interval));
        if (max_ticks > 0x3f)
            return false;
        hwwdg.Init.Counter = 0x40 + max_ticks;
        hwwdg.Init.Window = hwwdg.Init.Counter - min_ticks;
        return true;
    }

    /**
     * @brief Starts the watchdog. config() should be called before.
     */
    bool start() final {
        __HAL_RCC_WWDG_CLK_ENABLE();
        hwwdg.Init.Prescaler = WWDG_PRESCALER_1;
        hwwdg.Init.EWIMode = WWDG_EWI_ENABLE;
        __HAL_DBGMCU_FREEZE_WWDG();
        return HAL_WWDG_Init(&hwwdg) == HAL_OK;
    }

    bool reset() final {
        return HAL_WWDG_Refresh(&hwwdg) == HAL_OK;
    }
};

extern STM32_WWDG_t<WWDG_BASE> wwdg;

#endif // __STM32_WWDG_HPP