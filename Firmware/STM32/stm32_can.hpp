#ifndef __STM32_CAN_HPP
#define __STM32_CAN_HPP

#include <subscriber.hpp>

#include <stm32_gpio.hpp>

typedef struct {
    uint32_t id = 0x000;  // 11-bit max is 0x7ff, 29-bit max is 0x1FFFFFFF
    bool isExt = false;
    bool rtr = false;
    uint8_t len = 8;
    uint8_t buf[8] = {0, 0, 0, 0, 0, 0, 0, 0};
} CAN_message_t;

// Most common CAN baud rates
enum CAN_baudrate_t {
    CAN_BAUD_125K   = 125000,
    CAN_BAUD_250K   = 250000,
    CAN_BAUD_500K   = 500000,
    CAN_BAUD_1000K  = 1000000,
    CAN_BAUD_1M     = 1000000
};

class STM32_CAN_t {
public:
    CAN_HandleTypeDef hcan;

    const STM32_GPIO_t** rx_gpios;
    const STM32_GPIO_t** tx_gpios;
    uint8_t gpio_af;

    static constexpr const size_t N_RX_FIFOS = 2;
    static constexpr const size_t N_RX_FILTER_BANKS = 13;

    STM32_CAN_t(CAN_TypeDef* instance, const STM32_GPIO_t** rx_gpios, const STM32_GPIO_t** tx_gpios, uint8_t gpio_af) :
        hcan{ .Instance = instance },
        rx_gpios(rx_gpios),
        tx_gpios(tx_gpios),
        gpio_af(gpio_af) {}

    /**
     * @brief Initializes the CAN interface.
     */
    bool init(STM32_GPIO_t* rx_gpio, STM32_GPIO_t* tx_gpio);

    /**
     * @brief Configures the CAN interface.
     * 
     * This function must not be used while the interface is running (i.e.
     * started with start()).
     */
    bool config(CAN_baudrate_t baudrate);

    bool enable_interrupts(uint8_t priority);

    /**
     * @brief Starts the CAN interface.
     */
    bool start();

    /**
     * @brief Stops the CAN interface.
     */
    bool stop() { return HAL_CAN_Stop(&hcan) == HAL_OK; }

    /**
     * @brief Adds a filter for incoming messages.
     * Any incoming message that matches this filter is put into the specified
     * FIFO.
     * This function should only be used once config() has been called.
     * TODO: the callback is invoked in an interrupt context, may want to support deferred processing
     * 
     * @param id The expected message ID
     * @param mask A mask for the ID. A bit value of 0 means "don't care", 1 means "must match".
     * @param fifo The FIFO to use for accepted messages
     */
    bool subscribe(uint32_t id, uint32_t mask, uint32_t fifo, void(*callback)(void* ctx, CAN_message_t& msg), void* ctx);

    /**
     * @brief Sends a CAN message on the bus.
     * This function is non-blocking.
     * 
     * @param msg The message to dispatch
     * @param mailbox Outputs the mailbox ID that was used for sending. Can be NULL.
     * @returns True on success or false otherwise
     */
    bool send(CAN_message_t& msg, uint32_t* mailbox = nullptr);

    bool handle_rx_message(uint32_t fifo);

private:
    size_t filter_banks_in_use_ = 0;
    size_t n_fifo0_subscribers = 0;
    size_t n_fifo1_subscribers = 0;

    // TODO: theoretically the maximum number of filters for a single FIFO is 4 * N_RX_FILTER_BANKS
    // however we currently don't implement any filters other than 32-bit masks.
    Subscriber<CAN_message_t&> fifo0_subscribers[N_RX_FILTER_BANKS];
    Subscriber<CAN_message_t&> fifo1_subscribers[N_RX_FILTER_BANKS];
};

extern STM32_CAN_t can1, can2;

#endif // __STM32_CAN_HPP