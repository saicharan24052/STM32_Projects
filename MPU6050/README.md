# MPU6050 6-Axis IMU — STM32H743ZI2

Embedded C firmware for interfacing the **MPU6050 6-axis IMU** with the **STM32H743ZI2** microcontroller using **STM32 HAL drivers**.

The MPU6050 provides **3-axis accelerometer** and **3-axis gyroscope** measurements. The sensor communicates with the STM32 through **I²C**, and the acquired data is transmitted through **USART3** to a Windows PC for monitoring using **PuTTY**.

---

## System Architecture

```text
                 ┌──────────────────────┐
                 │       MPU6050        │
                 │      6-Axis IMU      │
                 │                      │
                 │ Accelerometer X/Y/Z  │
                 │   Gyroscope X/Y/Z    │
                 └──────────┬───────────┘
                            │
                            │ I²C1
                            ▼
                 ┌──────────────────────┐
                 │     STM32H743ZI2     │
                 │    Microcontroller   │
                 │                      │
                 │  Sensor Acquisition  │
                 │   Data Processing    │
                 └──────────┬───────────┘
                            │
                            │ USART3
                            ▼
                 ┌──────────────────────┐
                 │      Windows PC      │
                 │        PuTTY         │
                 │                      │
                 │ Accelerometer Data   │
                 │   Gyroscope Data     │
                 └──────────────────────┘
```

---

## Hardware

| Device | Function | Interface |
|---|---|---|
| **MPU6050** | 3-axis Accelerometer + 3-axis Gyroscope | I²C |
| **STM32H743ZI2** | Sensor interface and processing | I²C1 / USART3 |
| **Windows PC** | Serial data monitoring | UART |

---

## Firmware

The project uses **Embedded C** with STM32 HAL APIs for peripheral communication.

The firmware includes:

- MPU6050 initialization and device-ID check
- Accelerometer X/Y/Z data acquisition
- Gyroscope X/Y/Z data acquisition
- I²C register read/write operations
- UART data transmission
- `printf()` output through USART3

Example HAL APIs used:

```c
HAL_I2C_Mem_Read();
HAL_I2C_Mem_Write();
HAL_UART_Transmit();
```

---

## Data Flow

```text
MPU6050
   │
   │ I²C1
   ▼
STM32H743ZI2
   │
   │ USART3
   ▼
PuTTY
   │
   ├── Accelerometer X/Y/Z
   └── Gyroscope X/Y/Z
```

---

## UART Output

The STM32 sends the sensor measurements through **USART3**, allowing the values to be viewed on a PC using PuTTY.

Example:

```text
MPU6050 Initialized

Accelerometer:
X : ...
Y : ...
Z : ...

Gyroscope:
X : ...
Y : ...
Z : ...
```

The exact output format depends on the firmware implementation.

---

## Repository Contents

### `Core/`

Contains the main application and firmware source code:

```text
Core/
├── Inc/
└── Src/
```

The `Core/Src` and `Core/Inc` files contain the main MPU6050 interfacing, sensor acquisition, processing, and UART code.

### Complete Project `.rar`

The repository also includes a **`.rar` archive containing the complete STM32CubeIDE project**, including the project configuration and supporting files.

> Use the `.rar` archive for the complete STM32CubeIDE project. Use `Core/` when you only need the main source code.

---

## Portability

The current project is configured for:

```text
STM32H743ZI2
```

The application logic in `Core/Src` and `Core/Inc` can be adapted to other STM32 microcontrollers.

When migrating to another STM32 device, MCU-specific settings such as the following may need to be changed:

- I²C peripheral and pins
- UART/USART peripheral and pins
- GPIO configuration
- Clock configuration
- Interrupt configuration
- STM32CubeIDE device configuration

---

## How to Run

1. Extract the `.rar` archive.
2. Open the project in **STM32CubeIDE**.
3. Verify the target MCU is **STM32H743ZI2**.
4. Connect the MPU6050 to the configured **I²C1** interface.
5. Build and flash the firmware.
6. Connect the STM32 USART output to the PC.
7. Open **PuTTY** using the configured serial settings.
8. Observe the accelerometer and gyroscope data.

---

## Software & Tools

- Embedded C
- STM32CubeIDE
- STM32 HAL
- I²C
- UART / USART
- PuTTY

---

## Project Status

**Implemented**

MPU6050 accelerometer and gyroscope data acquisition and UART-based serial monitoring are implemented on the STM32H743ZI2.

---

## Author

**CH. Sai Charan**
