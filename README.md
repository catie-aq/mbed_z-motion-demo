# KOMBOS intertial node

This is the mbed-os project for the BLE intertial nodes, designed for the KOMBOS project.
The main file is in the `source` folder.

## Compile the project

* Install the compiler `arm-none-eabi-gcc`

* Compile using `mbed-cli`. To install mbed-cli follow the instruction given here https://docs.mbed.com/docs/mbed-os-handbook/en/5.1/dev_tools/cli/#installing-mbed-cli

## Modify the node settings

* You can change the name of the node by editting the variable `DEVICE_NAME`
* You can change the node ID, which is including into the BLE frames, by editting the variable `NODE_ID`
* you can modify the frequency of the node by editting the variable `KOMBOS_FREQUENCY`
* Set the `NEED_LOG` define to 1 if you want more info on the serial link
