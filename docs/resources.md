
Most information in this file can be reproduced by running `dump_interrupts(odrv0)`, `dump_dma(odrv0)` and `dump_threads(odrv0)` in `odrivetool` (minor manual postprocessing was applied to the output of those functions).

Take this info with a grain of salt as we might forget to update it from time to time. When in doubt check the file history.

# ODrive v3.6

## Interrupt Vectors

 - lowest priority: 15
 - highest priority: 0

|   # | Name                    | Prio |
|-----|-------------------------|------|
| -12 | MemoryManagement_IRQn   |    0 |
| -11 | BusFault_IRQn           |    0 |
| -10 | UsageFault_IRQn         |    0 |
|  -5 | SVCall_IRQn             |    3 |
|  -4 | DebugMonitor_IRQn       |    0 |
|  -2 | PendSV_IRQn             |   15 |
|  -1 | SysTick_IRQn            |   15 |
|   6 | EXTI0_IRQn              |    1 |
|   7 | EXTI1_IRQn              |    1 |
|   8 | EXTI2_IRQn              |    1 |
|   9 | EXTI3_IRQn              |    1 |
|  10 | EXTI4_IRQn              |    1 |
|  11 | DMA1_Stream0_IRQn       |    4 |
|  13 | DMA1_Stream2_IRQn       |   10 |
|  15 | DMA1_Stream4_IRQn       |   10 |
|  16 | DMA1_Stream5_IRQn       |   10 |
|  17 | DMA1_Stream6_IRQn       |   10 |
|  19 | CAN1_TX_IRQn            |    9 |
|  20 | CAN1_RX0_IRQn           |    9 |
|  21 | CAN1_RX1_IRQn           |    9 |
|  22 | CAN1_SCE_IRQn           |    9 |
|  23 | EXTI9_5_IRQn            |    1 |
|  40 | EXTI15_10_IRQn          |    1 |
|  44 | TIM8_UP_TIM13_IRQn      |    0 |
|  45 | TIM8_TRG_COM_TIM14_IRQn |    6 |
|  47 | DMA1_Stream7_IRQn       |    3 |
|  50 | TIM5_IRQn               |    1 |
|  52 | UART4_IRQn              |   10 |
|  67 | OTG_FS_IRQn             |    6 |
|  77 | OTG_HS_IRQn (aka ControlLoop_IRQn) |    5 |


## DMA Streams

 - lowest priority: 0
 - highest priority: 3

| Name         | Prio | Channel                          | High Level Func |
|--------------|------|----------------------------------|-----------------|
| DMA1_Stream0 |    1 | 0 (SPI3_RX)                      | SPI_A           |
| DMA1_Stream2 |    0 | 4 (UART4_RX)                     | UART_A          |
| DMA1_Stream4 |    0 | 4 (UART4_TX)                     | UART_A          |
| DMA1_Stream5 |    0 | 4 (USART2_RX)                    | UART_B          |
| DMA1_Stream6 |    0 | 4 (USART2_TX)                    | UART_B          |
| DMA1_Stream7 |    2 | 0 (SPI3_TX)                      | SPI_A           |
| DMA2_Stream0 |    0 | 0 (ADC1)                         | freerunning ADC |


## Threads

 - lowest priority: -3
 - highest priority: 3

| Name    | Stack Size [B] | Prio |
|---------|----------------|------|
| axis0   |           2048 |    3 |
| axis1   |           2048 |    2 |
| can     |           1024 |    0 |
| startup |           2048 |    0 |
| uart    |           4096 |    0 |
| usb     |           4096 |    0 |


# ODrive v4.0

## Interrupt Vectors

 - lowest priority: 15
 - highest priority: 0

|   # | Name                    | Prio |
|-----|-------------------------|------|
| -12 | MemoryManagement_IRQn   |    0 |
| -11 | BusFault_IRQn           |    0 |
| -10 | UsageFault_IRQn         |    0 |
|  -5 | SVCall_IRQn             |    0 |
|  -4 | DebugMonitor_IRQn       |    0 |
|  -2 | PendSV_IRQn             |   15 |
|  -1 | SysTick_IRQn            |   15 |
|  11 | DMA1_Stream0_IRQn       |    5 |
|  14 | DMA1_Stream3_IRQn       |    5 |
|  15 | DMA1_Stream4_IRQn       |    5 |
|  16 | DMA1_Stream5_IRQn       |    5 |
|  17 | DMA1_Stream6_IRQn       |    5 |
|  18 | ADC_IRQn                |    1 |
|  19 | CAN1_TX_IRQn            |    6 |
|  20 | CAN1_RX0_IRQn           |    6 |
|  21 | CAN1_RX1_IRQn           |    6 |
|  22 | CAN1_SCE_IRQn           |    6 |
|  26 | TIM1_TRG_COM_TIM11_IRQn |    2 |
|  35 | SPI1_IRQn               |    5 |
|  36 | SPI2_IRQn               |    5 |
|  38 | USART2_IRQn             |    5 |
|  45 | TIM8_TRG_COM_TIM14_IRQn |    0 |
|  47 | DMA1_Stream7_IRQn       |    0 |
|  51 | SPI3_IRQn               |    5 |
|  59 | DMA2_Stream3_IRQn       |    5 |
|  77 | OTG_HS_IRQn             |    5 |

## DMA Streams

 - lowest priority: 0
 - highest priority: 3

| Name         | Prio | Channel                          | High Level Func |
|--------------|------|----------------------------------|-----------------|
| DMA1_Stream0 |    1 | 0 (SPI3_RX)                      | Onboard SPI     |
| DMA1_Stream3 |    0 | 0 (SPI2_RX)                      | Offboard SPI    |
| DMA1_Stream4 |    0 | 0 (SPI2_TX)                      | Offboard SPI    |
| DMA1_Stream5 |    0 | 4 (USART2_RX)                    | UART1           |
| DMA1_Stream6 |    0 | 4 (USART2_TX)                    | UART1           |
| DMA1_Stream7 |    1 | 0 (SPI3_TX)                      | Onboard SPI     |
| DMA2_Stream0 |    0 | 0 (ADC1)                         | freerunning ADC |
| DMA2_Stream3 |    0 | 3 (SPI1_TX)                      | Status LED      |

## Threads

**TODO**
