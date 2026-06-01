# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

STM32CubeMX-based BLDC sensorless motor control (ESC) project targeting **STM32F051K8Ux** (Cortex-M0, 64KB flash, 8KB SRAM). Built with **Keil MDK-ARM v5** (uVision) using **ARMCLANG v6.9**.

The project implements a sensorless BLDC ESC (electronic speed controller) with:
- Six-step commutation with back-EMF zero-crossing detection
- MOS-FET self-test on startup
- WS2812B RGB status LED
- Battery voltage monitoring (3S-6S LiPo auto-detect)
- UART debug console with custom printf implementation
- Configurable open-loop start with retry logic

## Core Development Rules

### 1. Pure Register-Level Development
- **Strictly forbidden**: HAL library, standard peripheral library, or any high-level abstraction libraries for peripheral access.
- All peripheral operations must directly access hardware registers.
- **Register bit manipulation** must use CMSIS-defined `_Pos` macros for readability — never use raw shift counts:
  ```c
  // Correct
  ADC1->CFGR1 |= (1u << ADC_CFGR1_OVRMOD_Pos);

  // Wrong — raw magic numbers are forbidden
  ADC1->CFGR1 |= (1u << 12);
  ```
- Add concise comments on non-obvious register operations.

### 2. High Portability Architecture
- **Algorithm layer** (motor control math, FOC, observer/ESC logic) and **hardware driver layer** (PWM, ADC, GPIO) must be strictly decoupled.
- Hardware accesses must be wrapped through macro definitions or interface functions with `BSP_` prefix.
- Example pattern:
  ```c
  // In hardware driver header (MCU-specific)
  #define BLDCDRV_PWM_SET_DUTY(ch, val)  (TIM1->CCR##ch = (val))

  // In algorithm code (MCU-agnostic)
  BLDCDRV_PWM_SET_DUTY(1, duty_a);
  ```

### 3. Naming Conventions
- **Hardware Driver Layer**: `BSP_` prefix for all public functions (e.g., `BSP_PWM_Init`, `BSP_ADC_GetMetrics`)
- **ESC Control Layer**: `ESHL_` prefix for all ESC logic functions, types, and defines (e.g., `ESHL_ESC_Init`, `ESHL_STATE_READY`)
- **Register-level enums**: `REG_` prefix to avoid HAL namespace conflicts (e.g., `REG_MODE_OUT`, `REG_PIN_0`)

## Build System

- **IDE**: Keil uVision 5 (`.uvprojx` project file at `MDK-ARM/BLCD.uvprojx`)
- **Toolchain**: ARMCLANG v6.9 (AC6)
- **No CLI build** — compilation and flashing happen through the Keil IDE
- Output artifacts: `BLCD.axf` (ELF), `BLCD.hex` (Intel Hex) in `MDK-ARM/BLCD/`
- Debug probe: ST-Link (via STLink SWO DLL)
- Flash driver: STM32F0xx 64KB Flash

## Memory Layout

| Region | Start       | Size      | Usage      |
| ------ | ----------- | --------- | ---------- |
| Flash  | `0x08000000` | `0x10000` | Code + RO  |
| SRAM   | `0x20000000` | `0x02000` | RW + ZI data |

Stack: `0x400` bytes, Heap: `0x200` bytes (defined in `startup_stm32f051x8.s`). 8KB SRAM is tight — be mindful of stack depth and large global buffers (the DMA buffer for WS2812B, at ~280 bytes, is a significant allocation).

## Boot Sequence

```
Reset_Handler (startup_stm32f051x8.s)
  -> SystemInit() — CubeMX HAL init, configures PLL for 48MHz from 8MHz HSI
  -> __main — ARM C runtime init (data copy, BSS zero)
  -> main()
      -> SystemClock_Config() — register-level re-init of PLL
      -> BSP_DEBUG_Init(115200) — UART1 on PA9/PA10
      -> BSP_SysTick_Init_1ms()
      -> BSP_LED_Init() — WS2812B on PB8, TIM16_CH1 + DMA
      -> BSP_GPIO_Init() — PB5/PB6/PB7 as low-side FET outputs
      -> BSP_PWM_Init() — TIM2/3 on PA15/PB3/PB4 for high-side FET PWM
      -> BSP_Time_Init() — TIM6 for microsecond delays
      -> BSP_ADC_Init() — ADC1 DMA on PA0/PA1
      -> BSP_ADC_Start()
      -> ESHL_ESC_Init() — ESC state init, battery type detection
      -> MOS_SelfTest() — checks each FET for shorts
      -> main loop: state machine dispatches ESC states
```

## Architecture

### Hardware Driver Layer (`Inc/bldc_*.h` + `Src/bldc_*.c`)

All `BSP_` prefixed, all register-level, no HAL dependencies:

| Module | Files | Peripherals | Function |
| ------ | ----- | ----------- | -------- |
| `bldc_adc` | `bldc_adc.h/.c` | ADC1 + DMA1_CH1 | PA0 (VBUS), PA1 (phase current). Continuous scan, DMA circular mode, calibration |
| `bldc_pwm` | `bldc_pwm.h/.c` | TIM2 (CH1/CH2), TIM3 (CH1) | PA15=U, PB3=V, PB4=W. 16kHz PWM (PSC=2, ARR=991) |
| `bldc_gpio` | `bldc_gpio.h/.c` | GPIOB | PB7=AD, PB6=BD, PB5=CD low-side FET gates. Config-table driven init |
| `bldc_time` | `bldc_time.h/.c` | SysTick + TIM6 | `delay_ms()`, `delay_us()`, `get_tick()` |
| `bldc_clock` | `bldc_clock.h/.c` | RCC, SysTick | PLL 48MHz config, 1ms tick init |
| `bldc_debug` | `bldc_debug.h/.c` | USART1 | PA9(TX), PA10(RX). Interrupt-driven RX, custom `printf` via `logxxx()` |
| `bldc_led` | `bldc_led.h/.c` | TIM16_CH1 + DMA1_CH3 | WS2812B on PB8. Color/pattern state machine. No bit-banging |

### ESC Control Layer (`bldc_driver.h/.c`)

- Contains the ESC state machine (`ESHL_STATE_ENUM_T`), six-step commutation logic (`ESHL_U_D_Ctrl`), MOS self-test, startup beep sequences, battery detection, and current monitoring.
- Accesses hardware through `BSP_` interfaces and driver macros (e.g., `ESHL_AU_ENABLE(pwm)`)
- **Known issues**: still has some legacy HAL calls (`HAL_GetTick`, `HAL_COMP_Stop_IT`) and references to outdated variable names (`adc_val_buff`) — these need register-level replacements.

### Main Application (`main.c`)

- Contains the initialization sequence and main-loop state machine.
- `SysTick_Handler` is overridden here to call `BSP_LED_Tick_1ms()` and increment `g_system_ticks`.

## Project Structure

```
BLCD.ioc                    # CubeMX project file (skeleton generation only)
Inc/
  main.h                    # Includes stm32f0xx_hal.h (CubeMX requirement)
  gpio.h                    # CubeMX-generated (unused — bldc_gpio replaces it)
  stm32f0xx_hal_conf.h      # HAL config
  stm32f0xx_it.h            # IRQ header
  bldc_adc.h                # ADC driver
  bldc_clock.h              # Clock/SysTick driver
  bldc_debug.h              # UART debug/logging driver
  bldc_driver.h             # ESC control state machine + macros
  bldc_gpio.h               # GPIO driver
  bldc_led.h                # WS2812B LED driver
  bldc_pwm.h                # PWM timer driver
  bldc_time.h               # Time/delay driver
Src/
  main.c                    # Application entry + main loop
  bldc_adc.c, bldc_clock.c, bldc_debug.c, bldc_driver.c,
  bldc_gpio.c, bldc_led.c, bldc_pwm.c, bldc_time.c
  stm32f0xx_it.c            # CubeMX ISRs (NMI, HardFault, SVC, PendSV)
  stm32f0xx_hal_msp.c       # CubeMX HAL MSP — unused by driver code
  system_stm32f0xx.c        # CubeMX SystemInit (initial 48MHz via HAL)
Drivers/
  STM32F0xx_HAL_Driver/     # STM32F0 HAL — only CMSIS headers used
  CMSIS/                    # CMSIS-CORE + device register defs
MDK-ARM/
  BLCD.uvprojx              # Project file — add new .c/.h files here
  startup_stm32f051x8.s     # Vector table and reset handler
```

## Pin Mapping

| Pin  | Function       | Peripheral       | Notes                     |
|------|----------------|------------------|---------------------------|
| PA0  | VBUS sense     | ADC1_IN0         | Battery voltage divider   |
| PA1  | Phase current  | ADC1_IN1         | Shunt amp output          |
| PA9  | UART TX        | USART1_TX        | Debug console             |
| PA10 | UART RX        | USART1_RX        |                           |
| PA13 | SWDIO          | SWD              | Debug                     |
| PA14 | SWCLK          | SWD              | Debug                     |
| PA15 | Phase U high   | TIM2_CH1 (AF2)   | High-side FET U           |
| PB3  | Phase V high   | TIM2_CH2 (AF2)   | High-side FET V           |
| PB4  | Phase W high   | TIM3_CH1 (AF1)   | High-side FET W           |
| PB5  | Phase W low    | GPIO output      | Low-side FET W            |
| PB6  | Phase V low    | GPIO output      | Low-side FET V            |
| PB7  | Phase U low    | GPIO output      | Low-side FET U            |
| PB8  | Status LED     | TIM16_CH1 (AF2)  | WS2812B data line         |

## Common Tasks

### Adding a new .c/.h file
1. Create the file in `Inc/` or `Src/`
2. Add it to the Keil project in `MDK-ARM/BLCD.uvprojx` (open in uVision → Manage Project Items)
3. Follow `BSP_` prefix conventions for driver-layer functions

### Re-generating CubeMX skeleton
- Open `BLCD.ioc` in STM32CubeMX v6.12.0
- CubeMX respects `USER CODE BEGIN` / `USER CODE END` markers — user logic outside these markers will be overwritten
- After re-generation, verify no CubeMX HAL calls leaked into driver files

### Debugging
- UART console at 115200 baud on PA9/PA10
- Log levels: `LOG_ERROR`, `LOG_WARN`, `LOG_INFO`, `LOG_DEBUG` (configured via `LOG_LEVEL` in `bldc_debug.h`)
- WS2812B LED provides visual status feedback via `BSP_LED_SetStatus()`

## Known Issues & Gotchas

1. **HAL vestiges in `bldc_driver.c`**: `ESHL_RuningCurrentVBATChack()` still calls `HAL_GetTick()`, `HAL_COMP_Stop_IT()`, and `HAL_ADCEx_Calibration_Start()`. These need register-level replacements. The function also references `adc_val_buff[0]` / `adc_val_buff[1]` which don't exist in the current ADC interface — should use `g_adc_metrics` via `BSP_ADC_GetMetrics()`.

2. **TIM compilation**: The TIM HAL source files are compiled into the project, but `HAL_TIM_MODULE_ENABLED` is commented out in `stm32f0xx_hal_conf.h`. This is harmless for register-level development but relevant if CubeMX is re-generated with TIM peripheral enabled.

3. **`USE_HAL_DRIVER` define**: Retained because CubeMX-generated startup/system code depends on it. Application code must not use HAL APIs.

4. **Compiler defines**: `USE_HAL_DRIVER` `STM32F051x8`

5. **Flash endurance**: `ESHL_ADDR_FLASH_ADD` (`0x0800F800`) is used for storing the ESC address near the end of flash. Be careful not to overlap with code.

## Include Paths

```
../Inc
../Drivers/STM32F0xx_HAL_Driver/Inc
../Drivers/STM32F0xx_HAL_Driver/Inc/Legacy
../Drivers/CMSIS/Device/ST/STM32F0xx/Include
../Drivers/CMSIS/Include
```
