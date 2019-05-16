#ifndef __STM32_ADC_HPP
#define __STM32_ADC_HPP

#include <stm32_gpio.hpp>
#include <stm32_tim.hpp>
#include <stm32_dma.hpp>
#include <adc.hpp>

#include <array>

const float stm32_adc_full_scale = (float)(1 << 12);

class STM32_ADCSequence_t;

class STM32_ADC_t {
public:
    ADC_HandleTypeDef hadc;
    DMA_HandleTypeDef dma;

    std::array<STM32_GPIO_t*, 16> gpios;
    const STM32_DMAChannel_t* dmas;

    STM32_ADC_t(ADC_TypeDef* instance, std::array<STM32_GPIO_t*, 16> gpios,
                const STM32_DMAChannel_t* dmas) :
            hadc{ .Instance = instance },
            gpios(gpios),
            dmas(dmas) { }

    bool init();

    bool is_setup_ = false;
};

template<typename TSequence>
class STM32_ADCChannel_t : public ADCChannel_t {
public:
    TSequence& adc_;
    STM32_GPIO_t* gpio_; // may be NULL (e.g. for internal temp sensor)
    uint32_t channel_num_;
    uint32_t seq_pos_ = 0xffffffff; // position in the parent sequence (set in link())
    uint32_t sampling_time = 3; // TODO: expose in constructor

    STM32_ADCChannel_t(TSequence& adc, STM32_GPIO_t* gpio, uint32_t channel_num) :
        adc_(adc),
        gpio_(gpio),
        channel_num_(channel_num) {}

    bool enqueue() {
        return adc_.append(this, &seq_pos_);
    }

    bool init() final {
        return !gpio_ || gpio_->setup_analog();
    }

    bool get_normalized(float* value) final {
        uint16_t raw_value;

        if (!adc_.get_raw_value(seq_pos_, &raw_value))
            return false;

        if (value)
            *value = (float)raw_value / stm32_adc_full_scale;

        return true;
    }

    bool has_value() final {
        return adc_.has_completed();
    }

    bool reset_value() final {
        return adc_.reset_values();
    }

    bool enable_updates() final {
        return adc_.enable_updates();
    }

    /** @brief See STM32_ADCSequence_t::get_timing() for details */
    bool get_timing(uint32_t* sample_start_timestamp, uint32_t* sample_end_timestamp) {
        return adc_.get_timing(seq_pos_, sample_start_timestamp, sample_end_timestamp);
    }

    bool is_valid() {
        return channel_num_ < 19;
    }

    void handle_update() {
        on_update_.invoke();
    }
};

/**
 * @brief ADC Channel Sequence base class
 * 
 * Currently the assumption is that you init this object only once at startup
 * in the following order:
 * 
 *  1. init()
 *  2. set_trigger() (unless software trigger is used)
 *  3. append() (multiple times)
 *  4. apply()
 *  5. enable()
 * 
 */
class STM32_ADCSequence_t {
public:
    STM32_ADCSequence_t(STM32_ADC_t& adc) : adc_(adc) {}

    /**
     * @brief Sets up the underlying ADC and associates the given DMA stream
     * with it.
     * 
     * If a DMA is provided (non-NULL), the DMA is used to read in the sequence
     * after every ADC trigger. This is only supported for regular sequences.
     * If no DMA is provided (NULL), interrupts are used to read in the data.
     * This is only supported for injected sequences (though possible to enable
     * for regular sequences).
     * In both cases, the on_update_ event of every channel is invoked in order
     * every time the complete sequence was read in.
     */
    virtual bool init(STM32_DMAStream_t* dma) = 0;

    /**
     * @brief Configures the trigger output of the specified timer as trigger
     * for starting this conversion sequence.
     * 
     * Take care of the init order described on the STM32_ADCSequence_t class.
     * Note that not all ADC Sequence + Timer combinations are allowed.
     * 
     * Returns true on success or false otherwise (e.g. if the Timer is invalid
     * for this sequence).
     */
    virtual bool set_trigger(STM32_Timer_t* timer) = 0;

    /**
     * @brief Appends the given channel to this sequence.
     * Do not use this function directly, use STM32_ADCChannel_t::enqueue()
     * instead.
     * Returns true on success or false otherwise (e.g. if the sequence is full).
     */
    virtual bool append(ADCChannel_t* channel, uint32_t *seq_pos) = 0;

    /**
     * @brief Applies the settings that were configured using set_trigger() and
     * append().
     * 
     * Returns true on success or false otherwise.
     */
    virtual bool apply() = 0;

    virtual bool enable_updates() = 0;

    virtual bool enable_interrupts(uint8_t priority) = 0;

    virtual bool disable() = 0;

    /**
     * @brief Returns true if the complete sequence was refreshed since the last
     * time reset_values() was called.
     */
    virtual bool has_completed() = 0;

    /**
     * @brief Resets the state of has_values(). Should not be called while a
     * the sequence is running, otherwise the result may be undefined.
     */
    virtual bool reset_values() = 0;

    virtual bool get_raw_value(size_t channel_num, uint16_t *raw_value) = 0;

    /**
     * @brief Reports the timing of the sampling window of a particular item in the
     * sequence.
     * 
     * The sequence must be initialized before this can be used.
     * The timing is reported as number of HCLK ticks relative to the trigger event.
     * 
     * @param sample_start_timestamp: Minimum number of HCLK ticks between the
     *        trigger event and the opening of the sampling window of the specified
     *        channel. For the first channel in the sequence this is 0.
     * @param sample_end_timestamp: Maximum number of HCLK ticks between the trigger
     *        event and the closing of the sampling window. For the first
     *        channel in the sequennce this is the sampling time + 1 ADC clock
     *        cycle to account for the fact that the trigger may fire right
     *        after an ADC clock tick.
     */
    bool get_timing(size_t seq_pos, uint32_t* sample_start_timestamp, uint32_t* sample_end_timestamp);

    STM32_ADC_t& adc_;
    uint32_t channel_sequence_length = 0;
};

template<typename TSelf, unsigned int MAX_SEQ_LENGTH>
class STM32_ADCSequence_N_t : public STM32_ADCSequence_t {
public:
    using Channel_t = STM32_ADCChannel_t<TSelf>;

    Channel_t* channel_sequence[MAX_SEQ_LENGTH] = { nullptr };

    STM32_ADCSequence_N_t(STM32_ADC_t& adc) : STM32_ADCSequence_t(adc) {}

    bool append(Channel_t* channel, uint32_t* seq_pos) {
        if (!channel || !channel->is_valid())
            return false;
        if (channel_sequence_length >= MAX_SEQ_LENGTH)
            return false;
        if (seq_pos)
            *seq_pos = channel_sequence_length;
        channel_sequence[channel_sequence_length++] = channel;
        return true;
    }

    bool append(ADCChannel_t* channel, uint32_t *seq_pos) final {
        return append(static_cast<Channel_t*>(channel), seq_pos);
    }
    
    Channel_t get_channel(STM32_GPIO_t* gpio) {
        for (size_t i = 0; i < adc_.gpios.size(); ++i) {
            if (adc_.gpios[i] == gpio) {
                return Channel_t(dynamic_cast<TSelf&>(*this), gpio, i);
            }
        }

        return Channel_t(dynamic_cast<TSelf&>(*this), nullptr, UINT32_MAX);
    }

    Channel_t get_internal_temp_channel() {
        if (adc_.hadc.Instance == ADC1) {
#if defined(STM32F405xx) || defined(stm32f415xx) || defined(STM32F407xx) || defined(stm32f417xx)
            return Channel_t(this, nullptr, 16); // actually applies to all STM32F40x and STM32F41x but there's no macro for that
#elif defined(STM32F425xx) || defined(STM32F435xx)
            return Channel_t(this, nullptr, 18); // actually applies to all STM32F42x and STM32F43x but there's no macro for that
#else
#  error "unknown temp channel"
#endif
        }
        return Channel_t::invalid_channel();
    }

    Channel_t get_vrefint_channel() {
        if (adc_.hadc.Instance == ADC1) {
            return Channel_t(this, nullptr, 17);
        }
        return Channel_t::invalid_channel();
    }

    Channel_t get_vbat_channel() {
        if (adc_.hadc.Instance == ADC1) {
            return Channel_t(this, nullptr, 18);
        }
        return Channel_t::invalid_channel();
    }

    bool get_timing(size_t seq_pos, uint32_t* sample_start_timestamp, uint32_t* sample_end_timestamp) {
        uint32_t conversion_time;
        switch (ADC_GET_RESOLUTION(&adc_.hadc)) {
            case ADC_RESOLUTION_6B: conversion_time = 6; break;
            case ADC_RESOLUTION_8B: conversion_time = 8; break;
            case ADC_RESOLUTION_10B: conversion_time = 10; break;
            case ADC_RESOLUTION_12B: conversion_time = 12; break;
            default: return false;
        }

        uint32_t clk_mult = HAL_RCC_GetHCLKFreq() / HAL_RCC_GetPCLK2Freq();
        switch (ADC_COMMON_REGISTER(adc->hadc.Instance)->CCR & ADC_CCR_ADCPRE) {
    #if defined(ADC_CLOCK_SYNC_PCLK_DIV1)
            case ADC_CLOCK_SYNC_PCLK_DIV1: clk_mult *= 1; break;
    #endif
            case ADC_CLOCK_SYNC_PCLK_DIV2: clk_mult *= 2; break;
            case ADC_CLOCK_SYNC_PCLK_DIV4: clk_mult *= 4; break;
            case ADC_CLOCK_SYNC_PCLK_DIV6: clk_mult *= 6; break;
            case ADC_CLOCK_SYNC_PCLK_DIV8: clk_mult *= 8; break;
            default: return false;
        }

        uint32_t timestamp = 0;
        for (uint32_t i = 0; i < channel_sequence_length; ++i) {
            if (sample_start_timestamp) {
                *sample_start_timestamp = timestamp * clk_mult;
            }
            timestamp += channel_sequence[i].sampling_time;
            if (sample_end_timestamp) {
                *sample_end_timestamp = (timestamp + 1) * clk_mult;
            }
            timestamp += conversion_time * clk_mult;
            if (i == seq_pos) {
                return true;
            }
        }

        return false;
    }
};

class STM32_ADCRegular_t : public STM32_ADCSequence_N_t<STM32_ADCRegular_t, 16> {
public:
    using Channel_t = STM32_ADCChannel_t<STM32_ADCRegular_t>;

    STM32_ADCRegular_t(STM32_ADC_t& adc) : STM32_ADCSequence_N_t(adc) {}

    bool init(STM32_DMAStream_t* dma) final;
    bool set_trigger(STM32_Timer_t* timer) final;
    bool apply() final;
    bool enable_updates() final;
    bool enable_interrupts(uint8_t priority) final;
    bool disable() final;
    bool has_completed() final;
    bool reset_values() final;
    bool get_raw_value(size_t seq_pos, uint16_t *raw_value) final;
    void handle_irq();
    void handle_dma_complete();

    STM32_DMAStream_t* dma_ = nullptr;
    uint32_t trigger_source = ADC_SOFTWARE_START;
    uint32_t next_pos = 0; // TODO: ensure that this is properly synced to the ADC
    uint16_t raw_values[16];
    bool error_ = false;
};

class STM32_ADCInjected_t : public STM32_ADCSequence_N_t<STM32_ADCInjected_t, 4> {
public:
    using Channel_t = STM32_ADCChannel_t<STM32_ADCInjected_t>;

    STM32_ADCInjected_t(STM32_ADC_t& adc) : STM32_ADCSequence_N_t(adc) {}

    bool init(STM32_DMAStream_t* dma) final;
    bool set_trigger(STM32_Timer_t* timer) final;
    bool apply() final;
    bool enable_updates() final;
    bool enable_interrupts(uint8_t priority) final;
    bool disable() final;
    bool has_completed() final;
    bool reset_values() final;
    bool get_raw_value(size_t seq_pos, uint16_t *raw_value) final;
    void handle_irq();

    uint32_t trigger_source = ADC_INJECTED_SOFTWARE_START;
};


extern volatile uint32_t adc_irq_ticks;

extern STM32_ADC_t adc1, adc2, adc3;

extern STM32_ADCRegular_t adc1_regular, adc2_regular, adc3_regular;
extern STM32_ADCInjected_t adc1_injected, adc2_injected, adc3_injected;

/*constexpr int get_channel(STM32_ADC_t* adc, STM32_GPIO_t* gpio) {
    return STM32_ADCSequence_t::Channel_t();
}

int adc_ch1 = get_channel(&adc1, &pa0);*/

#endif // __STM32_ADC_HPP