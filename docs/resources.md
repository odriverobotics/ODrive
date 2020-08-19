
Most information in this file can be reproduced by running `dump_interrupts(odrv0)` and `dump_dma(odrv0)` in `odrivetool`.

# ODrive v3.6

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
|  13 | DMA1_Stream2_IRQn       |    5 |
|  15 | DMA1_Stream4_IRQn       |    5 |
|  16 | DMA1_Stream5_IRQn       |    5 |
|  18 | ADC_IRQn                |    5 |
|  19 | CAN1_TX_IRQn            |    6 |
|  20 | CAN1_RX0_IRQn           |    6 |
|  21 | CAN1_RX1_IRQn           |    6 |
|  22 | CAN1_SCE_IRQn           |    6 |
|  25 | TIM1_UP_TIM10_IRQn      |    0 |
|  44 | TIM8_UP_TIM13_IRQn      |    0 |
|  45 | TIM8_TRG_COM_TIM14_IRQn |    0 |
|  50 | TIM5_IRQn               |    5 |
|  51 | SPI3_IRQn               |    5 |
|  52 | UART4_IRQn              |    5 |
|  67 | OTG_FS_IRQn             |    5 |

## DMA Streams

 - lowest priority: 0
 - highest priority: 3

| Name         | Prio | Channel                          | High Level Func |
|--------------|------|----------------------------------|-----------------|
| DMA1_Stream0 |    1 | 0 (SPI3_RX)                      | SPI             |
| DMA1_Stream2 |    0 | 4 (UART4_RX)                     | UART0           |
| DMA1_Stream4 |    0 | 4 (UART4_TX)                     | UART0           |
| DMA1_Stream5 |    1 | 0 (SPI3_TX)                      | SPI             |
| DMA2_Stream0 |    0 | 0 (ADC1)                         | freerunning ADC |

