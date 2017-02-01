break Src/main.c:89

target remote localhost:3333
monitor reset halt
load
# There is a breakpoint at Reset_Handler () at
# Drivers/CMSIS/Device/ST/STM32F4xx/Source/Templates/gcc/startup_stm32f405xx.s:80
# by default, continue will make sure the first breakpoint that we encounter will be HAL_Init
# in Src/main.c
continue
