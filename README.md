# 6TRON BLE node

This Mbed OS application sends environmental and inertial data via BLE to a smartphone.

It runs on the completely integrated
[Z_Motion](https://gitlab.com/catie_6tron/z-motion-hardware/blob/master/doc/index.md) or
on a stack of 6TRON Zest boards (see [Hardware requirements](#hardware-requirements).

Dedicated smartphone applications are available on major platforms:

- [Android app](https://play.google.com/store/apps/details?id=com.minkagency.a6tron)
- [iOS app](https://itunes.apple.com/us/app/6tron/id1363884392?mt=8)

As BLE communications use the standard GATT protocol, you can also get data from generic
scanning and exploration tools like:

- [Nordic Semiconductor nRF Connect for Desktop](https://www.nordicsemi.com/Software-and-Tools/Development-Tools/nRF-Connect-for-desktop)
- [Nordic Semiconductor nRF Connect for Mobile](https://www.nordicsemi.com/Software-and-Tools/Development-Tools/nRF-Connect-for-mobile)

The application publishes data to the following BLE GATT services:

- the Environmental Service (standard GATT service: `0x181A`), with the following
  characteristics:
    - Temperature
    - Humidity
    - Pressure
- the Nordic Semiconductor UART service (custom service:
  `0x6E400000B5A3F393E0A9E50E24DCCA9E`), with the following characteristics:
    - RX
    - TX

## Requirements

### Hardware requirements

The following hardware is required:

[Z_Motion](https://gitlab.com/catie_6tron/z-motion-hardware/blob/master/doc/index.md)

*OR*

- [Zest_Core_STM32L496RG](https://gitlab.com/catie_6tron/zest-core-stm32l496rg-hardware/blob/master/doc/index.md)
  or [Zest_Core_STM32L4A6RG](https://gitlab.com/catie_6tron/zest-core-stm32l4a6rg-hardware/blob/master/doc/index.md)
- [Zest_Batttery_LiPo](https://gitlab.com/catie_6tron/zest-battery-lipo-hardware/blob/master/doc/index.md)
- [Zest_Sensor_IMU](https://gitlab.com/catie_6tron/zest-sensor-imu-hardware/blob/master/doc/index.md)
- [Zest_Radio_SPBTLE_RF0](https://gitlab.com/catie_6tron/zest-radio-spbtle-rf0-hardware/blob/master/doc/index.md)

### Software requirements

This application depends on the following libraries:

- [Mbed OS](https://gitlab.com/catie_6tron/mbed-os.git)
- [Bosch Sensortec BME280](https://gitlab.com/catie_6tron/bosch-sensortec-bme280)
- [Bosch Sensortec BNO055](https://gitlab.com/catie_6tron/bosch-sensortec-bno055/)
- [Maxime Integraded MAX17201](https://gitlab.com/catie_6tron/maxim-integrated-max17201.git)
- [STMicroelectronics SPBTLE-RF](https://github.com/ARMmbed/ble-x-nucleo-idb0xa1.git)

## Usage

To clone **and** deploy the project in one command, use `mbed import` and skip to
the target and toolchain definition:

  ```shell
  mbed import https://gitlab.com/catie_6tron/6tron-ble-node.git 6tron-ble-node
  ```

Alternatively:

- Clone to `6tron-ble-node` and enter it:

  ```shell
  git clone https://gitlab.com/catie_6tron/6tron-ble-node.git 6tron-ble-node
  cd 6tron-ble-node
  ```

- Create an empty Mbed CLI configuration file:

  - On Linux/macOS:
    ```shell
    touch .mbed on Linux/macOS
    ```

  - Or on Windows:
    ```shell
    echo.> .mbed
    ```

- Deploy software requirements with:

  ```shell
  mbed deploy
  ```

- Define your target (eg. `Z_MOTION`) and toolchain:

  ```shell
  mbed target Z_MOTION
  mbed toolchain GCC_ARM
  ```

- Export to Eclipse IDE with:

  ```shell
  mbed export -i eclipse_6tron
  ```
    
## Compiling and programming without IDE

- Compile the project:

  ```shell
  mbed compile
  ```

- Program the target device (eg. `STM32L496RG` for the Z_MOTION) with a J-Link debug
  probe:

  ```shell
  python dist/program.py STM32L496RG BUILD/Z_MOTION/GCC_ARM/6tron-ble-node.elf
  ```
