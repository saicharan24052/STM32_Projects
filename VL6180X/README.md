# VL6180X ToF Sensor — STM32H743ZI2

Embedded C firmware for interfacing the **VL6180X Time-of-Flight (ToF) sensor** with the **STM32H743ZI2** microcontroller using the **STM32 HAL (Hardware Abstraction Layer)**.

The project is configured specifically for the **STM32H743ZI2** and reads distance measurements from the VL6180X through **I²C**. The measured data is transmitted through **UART** and displayed on a Windows PC using **PuTTY**.

---

## Overview

This repository contains the Embedded C code developed to interface the VL6180X ToF sensor with an STM32 microcontroller.

The main application logic is available in the `Core/Src` and `Core/Inc` directories. The complete STM32CubeIDE project is also provided as a `.rar` archive.

### Data Flow

```text
        ┌──────────────────┐
        │     VL6180X      │
        │   ToF Sensor     │
        │                  │
        │ Distance Data    │
        └────────┬─────────┘
                 │
                 │ I²C
                 ▼
        ┌──────────────────┐
        │   STM32H743ZI2   │
        │  Microcontroller │
        │                  │
        │ VL6180X Driver   │
        │ Data Processing  │
        └────────┬─────────┘
                 │
                 │ UART
                 ▼
        ┌──────────────────┐
        │    Windows PC    │
        │                  │
        │      PuTTY       │
        │                  │
        │ Sensor Output    │
        └──────────────────┘
```

---

## Hardware

### Microcontroller

**STM32H743ZI2**

The firmware is configured for the STM32H743ZI2 and uses STM32 HAL drivers for peripheral initialization and communication.

### Sensor

**VL6180X**

The VL6180X is a Time-of-Flight sensor used for distance measurement.

### PC Interface

**UART → PC → PuTTY**

The measured distance data is transmitted from the STM32 through UART and monitored on a Windows PC using PuTTY.

---

## Communication Interfaces

| Device | Function | Interface |
|---|---|---|
| **VL6180X** | Distance / ToF measurement | I²C |
| **STM32H743ZI2** | Sensor control and processing | — |
| **Windows PC / PuTTY** | Sensor-data monitoring | UART |

---

## STM32 HAL

The project uses the **STM32 Hardware Abstraction Layer (HAL)** for accessing MCU peripherals.

The application uses HAL APIs for operations such as:

```c
HAL_I2C_Init();
HAL_I2C_Master_Transmit();
HAL_I2C_Master_Receive();

HAL_UART_Init();
HAL_UART_Transmit();
```

The actual HAL functions used are determined by the implementation in the project source files.

---

## Repository Contents

This repository is organized into two main parts:

### `Core/`

The `Core` directory contains the **main application source code** used for the VL6180X project.

```text
Core/
├── Inc/
│   ├── main.h
│   └── ...
│
└── Src/
    ├── main.c
    └── ...
```

The `Core/Src` and `Core/Inc` files contain the main firmware/application logic and can be studied or reused independently of the complete project archive.

### Full Project `.rar`

The repository also contains a **`.rar` archive containing the complete STM32CubeIDE project**.

The archive includes the complete project configuration and supporting files required to open, build, and program the project in STM32CubeIDE.

> **Use the `.rar` archive when you want the complete STM32CubeIDE project. Use the `Core/` directory when you only need to inspect or reuse the main application code.**

---

## Portability Across STM32 MCUs

This project is currently configured for:

```text
STM32H743ZI2
```

The main application logic in `Core/Src` and `Core/Inc` can generally be reused with other STM32 microcontrollers, but **MCU-specific configuration and initialization may require modification**.

When migrating to another STM32 MCU, the following may need to be updated:

- MCU/device configuration
- GPIO configuration
- I²C peripheral and pins
- UART/USART peripheral and pins
- Clock configuration
- Interrupt configuration
- STM32CubeIDE project configuration

The VL6180X application logic can remain largely the same while the MCU-specific peripheral configuration is adapted to the target STM32 device.

---

## Firmware Flow

```text
STM32 Initialization
        │
        ▼
Initialize I²C
        │
        ▼
Initialize UART
        │
        ▼
Initialize VL6180X
        │
        ▼
Read Distance Measurement
        │
        ▼
Process / Format Data
        │
        ▼
Transmit Data through UART
        │
        ▼
Display Output on PuTTY
```

---

## UART Output

The VL6180X measurement data is transmitted through the STM32 UART interface and displayed on the PC using PuTTY.

Example:

```text
VL6180X Initialized
Distance: 120 mm
Distance: 118 mm
Distance: 121 mm
Distance: 119 mm
```

The exact output format depends on the implementation in the source code.

---

## Software and Tools

- **Embedded C**
- **STM32CubeIDE**
- **STM32 HAL Drivers**
- **I²C**
- **UART / USART**
- **PuTTY**

---

## How to Build and Run

### 1. Clone the repository

```bash
git clone <your-repository-url>
cd <repository-name>
```

### 2. Open the complete project

Extract the `.rar` archive and open the project using **STM32CubeIDE**.

### 3. Verify the target MCU

The project is configured for:

```text
STM32H743ZI2
```

### 4. Connect the VL6180X

Connect the VL6180X to the configured STM32 I²C interface.

### 5. Build and flash

Build the project in STM32CubeIDE and program the STM32H743ZI2.

### 6. Monitor the output

Connect the STM32 UART output to the PC and open **PuTTY** using the configured serial settings.

The VL6180X distance measurements will be displayed in the terminal.

---

## Project Purpose

This project demonstrates:

- STM32 Embedded C development
- STM32 HAL-based peripheral programming
- VL6180X sensor interfacing
- I²C communication
- UART communication
- Sensor-driver integration
- Serial debugging and monitoring
- Reusable application logic across STM32 devices

---

## Project Status

**Implemented**

The firmware provides VL6180X interfacing on the STM32H743ZI2 and transmits the measured sensor data through UART for monitoring on a PC.

---

## Author

**CH. Sai Charan**
