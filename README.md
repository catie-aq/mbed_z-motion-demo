# BLE Gatt Service

This is an mbed-os project to design a BLE node that can stream data through a custom GATT service called UART service.
The main file is in the `source` folder.

## Compile the project

* Install the compiler `arm-none-eabi-gcc`

* Compile using `mbed-cli`. To install mbed-cli follow the instruction given here https://docs.mbed.com/docs/mbed-os-handbook/en/latest/dev_tools/cli/#installing-mbed-cli

## Modify the node settings

* You can change the name of the node by editting the variable `DEVICE_NAME`
* you can modify the conection parameters by editting the variable `CONNECTION_INTERVAL_MIN` and `CONNECTION_INTERVAL_MAX`
* Set the `NEED_LOG` define to 1 if you want more info on the serial link
