
#include "stm32_spi_arbiter.hpp"
#include "stm32_system.h"
#include "utils.hpp"
#include <cmsis_os.h>

bool equals(const SPI_InitTypeDef& lhs, const SPI_InitTypeDef& rhs) {
  return (lhs.Mode == rhs.Mode)
      && (lhs.Direction == rhs.Direction)
      && (lhs.DataSize == rhs.DataSize)
      && (lhs.CLKPolarity == rhs.CLKPolarity)
      && (lhs.CLKPhase == rhs.CLKPhase)
      && (lhs.NSS == rhs.NSS)
      && (lhs.BaudRatePrescaler == rhs.BaudRatePrescaler)
      && (lhs.FirstBit == rhs.FirstBit)
      && (lhs.TIMode == rhs.TIMode)
      && (lhs.CRCCalculation == rhs.CRCCalculation)
      && (lhs.CRCPolynomial == rhs.CRCPolynomial);
}

bool Stm32SpiArbiter::acquire_task(SpiTask* task) {
    return !__atomic_exchange_n(&task->is_in_use, true, __ATOMIC_SEQ_CST);
}

void Stm32SpiArbiter::release_task(SpiTask* task) {
    task->is_in_use = false;
}

bool Stm32SpiArbiter::start() {
    if (!task_list_) {
        return false;
    }

    SpiTask& task = *task_list_;
    if (!equals(task.config, hspi_->Init)) {
        HAL_SPI_DeInit(hspi_);
        hspi_->Init = task.config;
        HAL_SPI_Init(hspi_);
        __HAL_SPI_ENABLE(hspi_);
    }
    task.ncs_gpio.write(false);
    
    HAL_StatusTypeDef status = HAL_ERROR;

    if (hspi_->hdmatx->State != HAL_DMA_STATE_READY || hspi_->hdmarx->State != HAL_DMA_STATE_READY) {
        // This can happen if the DMA or interrupt priorities are not configured properly.
        status = HAL_BUSY;
    } else if (task.tx_buf && task.rx_buf) {
        status = HAL_SPI_TransmitReceive_DMA(hspi_, (uint8_t*)task.tx_buf, task.rx_buf, task.length);
    } else if (task.tx_buf) {
        status = HAL_SPI_Transmit_DMA(hspi_, (uint8_t*)task.tx_buf, task.length);
    } else if (task.rx_buf) {
        status = HAL_SPI_Receive_DMA(hspi_, task.rx_buf, task.length);
    }

    if (status != HAL_OK) {
        task.ncs_gpio.write(true);
    }

    return status == HAL_OK;
}

void Stm32SpiArbiter::transfer_async(SpiTask* task) {
    task->next = nullptr;
    
    // Append new task to task list.
    // We could try to do this lock free but we could also use our time for useful things.
    SpiTask** ptr = &task_list_;
    CRITICAL_SECTION() {
        while (*ptr)
            ptr = &(*ptr)->next;
        *ptr = task;
    }

    // If the list was empty before, kick off the SPI arbiter now
    if (ptr == &task_list_) {
        if (!start()) {
            if (task->on_complete) {
                (*task->on_complete)(task->on_complete_ctx, false);
            }
        }
    }
}

// TODO: this currently only works when called in a CMSIS thread.
bool Stm32SpiArbiter::transfer(SPI_InitTypeDef config, Stm32Gpio ncs_gpio, const uint8_t* tx_buf, uint8_t* rx_buf, size_t length, uint32_t timeout_ms) {
    volatile uint8_t result = 0xff;

    SpiTask task = {
        .config = config,
        .ncs_gpio = ncs_gpio,
        .tx_buf = tx_buf,
        .rx_buf = rx_buf,
        .length = length,
        .on_complete = [](void* ctx, bool success) { *(volatile uint8_t*)ctx = success ? 1 : 0; },
        .on_complete_ctx = (void*)&result,
        .is_in_use = false,
        .next = nullptr
    };

    transfer_async(&task);

    while (result == 0xff) {
        osDelay(1); // TODO: honor timeout
    }

    return result;
}

void Stm32SpiArbiter::on_complete() {
    if (!task_list_) {
        return; // this should not happen
    }

    // Wrap up transfer
    task_list_->ncs_gpio.write(true);
    if (task_list_->on_complete) {
        (*task_list_->on_complete)(task_list_->on_complete_ctx, true);
    }

    // Start next task if any
    SpiTask* next = nullptr;
    CRITICAL_SECTION() {
        next = task_list_ = task_list_->next;
    }
    if (next) {
        start();
    }
}
