#ifndef __STM32_SPI_ARBITER_HPP
#define __STM32_SPI_ARBITER_HPP

#include "stm32_gpio.hpp"

#include <spi.h>

class Stm32SpiArbiter {
public:
    struct SpiTask {
        SPI_InitTypeDef config;
        Stm32Gpio ncs_gpio;
        const uint8_t* tx_buf;
        uint8_t* rx_buf;
        size_t length;
        void (*on_complete)(void*, bool);
        void* on_complete_ctx;
        bool is_in_use = false;
        struct SpiTask* next;
    };

    Stm32SpiArbiter(SPI_HandleTypeDef* hspi): hspi_(hspi) {}

    /**
     * Reserves the task for the caller if it's not in use currently.
     *
     * This can be used by the caller to ensure that the task structure is not
     * overwritten while it's in use in a preceding transfer.
     *
     * Example:
     *
     *     if (acquire_task(&task)) {
     *         transfer_async(&task)
     *     }
     *
     * A call to release_task() makes the task available for use again.
     */
    static bool acquire_task(SpiTask* task);

    /**
     * Releases the task so that the next call to `acquire_task()` returns true.
     * This should usually be called inside the on_complete() callback after
     * the rx buffer has been processed.
     */
    static void release_task(SpiTask* task);

    /**
     * @brief Enqueues a non-blocking transfer.
     * 
     * Once the transfer completes, fails or is aborted, the callback is invoked.
     * 
     * This function is thread-safe with respect to all other public functions
     * of this class.
     * 
     * @param task: Contains all configuration data for this transfer.
     *        The struct pointed to by this argument must remain valid and
     *        unmodified until the completion callback is invoked.
     */
    void transfer_async(SpiTask* task);

    /**
     * @brief Executes a blocking transfer.
     * 
     * If the SPI is busy this function waits until it becomes available or
     * the specified timeout passes, whichever comes first.
     * 
     * Returns true on successful transfer or false otherwise.
     * 
     * This function is thread-safe with respect to all other public functions
     * of this class.
     * 
     * @param config: The SPI configuration to apply for this transfer.
     * @param ncs_gpio: The active low GPIO to actuate during this transfer.
     * @param tx_buf: Buffer for the outgoing data to be sent. Can be null unless
     *        rx_buf is null too. 
     * @param rx_buf: Buffer for the incoming data to be sent. Can be null unless
     *        tx_buf is null too.
     */
    bool transfer(SPI_InitTypeDef config, Stm32Gpio ncs_gpio, const uint8_t* tx_buf, uint8_t* rx_buf, size_t length, uint32_t timeout_ms);

    /**
     * @brief Completion method to be called from HAL_SPI_TxCpltCallback,
     * HAL_SPI_RxCpltCallback and HAL_SPI_TxRxCpltCallback.
     */
    void on_complete();

private:
    bool start();
    
    SPI_HandleTypeDef* hspi_;
    SpiTask* task_list_ = nullptr;
};

#endif // __STM32_SPI_ARBITER_HPP