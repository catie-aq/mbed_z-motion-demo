# 6TRON BLE node

This project is an Mbed OS project using the Mbed Bluetooth API, to create a BLE device that can send environmental end inertial data, using 6TRON boards.

You can deploy this application using 6TRON prototyping boards or into the integrated object [Z_Motion](https://gitlab.com/catie_6tron/z-motion-hardware/blob/master/doc/index.md).

To interact with the object, you can use a basic application used to debug BLE that can communicate using the standard GATT protocol.
Or you can download the 6TRON application designed for this device:

* Android: https://play.google.com/store/apps/details?id=com.minkagency.a6tron
* iOS: https://itunes.apple.com/us/app/6tron/id1363884392?mt=8

To share its sensor data, the device use 2 BLE GATT services:

* The Environmental Service which is a standard GATT service (`0x181A`), with 3 characteristics:
    * Temperature
    * Humidity
    * Pressure
* The Nordic UART service with is a custom service (`0x6E400000B5A3F393E0A9E50E24DCCA9E`), with 2 characteristics
    * RX
    * TX

## Requirements

### Hardware requirements

The following boards are required :

* Zest_Core_STM32L496RG or Zest_Core_STM32L4A6RG
* Zest_Batttery_LiPo
* Zest_IMU
* Zest_Radio_SPBTLE_RF0

**or**

The Z_Motion

### Software requirements

This demo makes use of the following libraries (automatically imported by `mbed deploy` or `mbed import`):

- [Mbed OS](https://gitlab.com/catie_6tron/mbed-os.git)
- [Bosch Sensortec BME280](https://gitlab.com/catie_6tron/bosch-sensortec-bme280)
- [Bosch Sensortec BNO055](https://gitlab.com/catie_6tron/bosch-sensortec-bno055/)
- [Maxime Integraded Gauge MAX17201](https://gitlab.com/catie_6tron/maxim-integrated-max17201.git)
- [SPBTLE ST Bluetooth module](https://github.com/ARMmbed/ble-x-nucleo-idb0xa1.git)

## Usage

To clone **and** deploy the project in one command, use `mbed import` and skip to
the target and toolchain definition:

    mbed import https://gitlab.com/catie_6tron/6tron-ble-node.git YOUR_PROJECT_NAME

Alternatively:

* Clone to `YOUR_PROJECT_NAME` and enter it:

    ```sh
    git clone https://gitlab.com/catie_6tron/6tron-ble-node.git YOUR_PROJECT_NAME
    cd YOUR_PROJECT_NAME
    ```

* Create an empty Mbed configuration file, otherwise Mbed CLI commands won't work:

    On Linux/macOS:

    ```sh
    touch .mbed on Linux/macOS
    ```

    Or on Windows:

    ```sh
    echo.> .mbed
    ```

* Deploy Mbed OS with:

    ```sh
    mbed deploy
    ```

* Define your target (eg. `ZEST_CORE_STM32L496RG`) and toolchain:

    ```sh
    mbed target ZEST_CORE_STM32L496RG
    mbed toolchain GCC_ARM
    ```

* Export to Eclipse IDE with:

    ```sh
    mbed export -i eclipse_6tron
    ```
    
## Configure the application to publish to your Adafruit IO account

* You can change the name of the node by editting the variable `DEVICE_NAME`
* you can modify the conection parameters by editting the variable `CONNECTION_INTERVAL_MIN` and `CONNECTION_INTERVAL_MAX`
* Set the `NEED_LOG` define to 1 if you want more info on the serial link

## Compiling and programming without IDE

* Compile the project:

    ```sh
    mbed compile
    ```

* Program the target device (eg. `STM32L496RG` for the Zest_Core_STM32L496RG) with a
  J-Link debug probe:

    ```sh
    python dist/program.py STM32L496RG BUILD/ZEST_CORE_STM32L496RG/GCC_ARM/YOUR_PROJECT_NAME.elf
    ```

## Manage and save your project with Git

* Edit `README.md` file

* Modify remote URL to point to your repository and push the application template:

    ```sh
    git remote set-url origin YOUR_REPOSITORY_URL
    git push -u origin master
    ```
