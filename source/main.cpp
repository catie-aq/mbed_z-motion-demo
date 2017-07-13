/* mbed Microcontroller Library
 * Copyright (c) 2006-2015 ARM Limited
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <events/mbed_events.h>
#include <mbed.h>
#include "ble/BLE.h"
#include "ble/Gap.h"
#include "ble/services/UARTService.h"
#include "ble/services/BatteryService.h"
#include "MAX17201.hpp"
#include "bno055.hpp"

#define NEED_CONSOLE_OUTPUT 1 /* Set this if you need debug messages on the console;
                               * it will have an impact on code-size and power consumption. */

#if NEED_CONSOLE_OUTPUT
#define DEBUG1(STR) { if (uartServicePtr) uartServicePtr->write(STR, strlen(STR));/* printf("%d\n", strlen(STR));*/ }
#define DEBUG2(STR, SIZE) { if (uartServicePtr) uartServicePtr->write(STR, SIZE); /*printf("%d\n", SIZE);*/ }
#else
#define DEBUG(...) /* nothing */
#endif /* #if NEED_CONSOLE_OUTPUT */

DigitalOut led1(LED1, 1);
I2C i2c(I2C_SDA, I2C_SCL);
Serial pc(SERIAL_TX, SERIAL_RX);

BNO055 bno(&i2c);
MAX17201 gauge(&i2c);

Gap::ConnectionParams_t gap_params;

const static char     DEVICE_NAME[] = "HAND NODE";
static const uint16_t uuid16_list[] = {GattService::UUID_BATTERY_SERVICE};

static bno055_raw_quaternion_t quat;
static UARTService *uartServicePtr;
static BatteryService * battery_service;
static uint8_t battery_level = 50;
static uint16_t msg_cnt = 0;

static EventQueue eventQueue(
    /* event count */ 16 * /* event size */ 32
);

void disconnectionCallback(const Gap::DisconnectionCallbackParams_t *params)
{
    BLE::Instance().gap().startAdvertising(); // restart advertising
}

void connectionCallback (const Gap::ConnectionCallbackParams_t *params)
{
	led1 = 1;
}

void updateSensorValue()
{
    // Do blocking calls or whatever is necessary for sensor polling.
    // In our case, we simply update the quaternion measurement.
	uint8_t buffer[11];
	bno.read_quaternion(&quat);
	msg_cnt++;

    //snprintf(buffer, 21, "%.3f,%.3f,%.3f,%.3f\n", quat.w, quat.x, quat.y, quat.z);
	buffer[0] = (msg_cnt >> 8); buffer[1] = (msg_cnt & 0xFF);
	buffer[2] = (quat.w >> 8); buffer[3] = (quat.w & 0xFF);
	buffer[4] = (quat.x >> 8); buffer[5] = (quat.x & 0XFF);
	buffer[6] = (quat.y >> 8); buffer[7] = (quat.y & 0XFF);
	buffer[8] = (quat.z >> 8); buffer[9] = (quat.z & 0XFF);
	buffer[10] = 0x0A; // "\n"

    DEBUG2(buffer, sizeof(buffer));
    //DEBUG1("\n");
    //printf("Quaternion: %d, %d, %d, %d\n\r", quat.w, quat.x, quat.y, quat.z);
}

void periodicCallback(void)
{
    if (BLE::Instance().getGapState().connected) {
        eventQueue.call(updateSensorValue);
    }
    else {
	    led1 = !led1; /* Do blinky on LED1 while we're waiting for BLE events */
	    Thread::wait(200);
	}
}

void onBleInitError(BLE &ble, ble_error_t error)
{
    (void)ble;
    (void)error;
   /* Initialization error handling should go here */
}

void bleInitComplete(BLE::InitializationCompleteCallbackContext *params)
{
    BLE&        ble   = params->ble;
    ble_error_t error = params->error;

    if (error != BLE_ERROR_NONE) {
        onBleInitError(ble, error);
        return;
    }

    if (ble.getInstanceID() != BLE::DEFAULT_INSTANCE) {
        return;
    }

    ble.gap().onDisconnection(disconnectionCallback);

    /* Setup primary service. */
    uartServicePtr = new UARTService(ble);
    battery_service = new BatteryService(ble, battery_level);

    /* Setup advertising. */
    ble.gap().accumulateAdvertisingPayload(GapAdvertisingData::BREDR_NOT_SUPPORTED | GapAdvertisingData::LE_GENERAL_DISCOVERABLE);
    ble.gap().setAdvertisingType(GapAdvertisingParams::ADV_CONNECTABLE_UNDIRECTED);
    ble.gap().accumulateAdvertisingPayload(GapAdvertisingData::COMPLETE_LOCAL_NAME, (uint8_t *)DEVICE_NAME, sizeof(DEVICE_NAME));
    ble.gap().accumulateAdvertisingPayload(GapAdvertisingData::COMPLETE_LIST_16BIT_SERVICE_IDS, (uint8_t *) uuid16_list, sizeof(uuid16_list));
    ble.gap().accumulateAdvertisingPayload(GapAdvertisingData::COMPLETE_LIST_128BIT_SERVICE_IDS, (uint8_t *)UARTServiceUUID_reversed, sizeof(UARTServiceUUID_reversed));

    ble.gap().setAdvertisingInterval(100); /* 500ms */
    ble.gap().startAdvertising();
}

void scheduleBleEventsProcessing(BLE::OnEventsToProcessCallbackContext* context) {
    BLE &ble = BLE::Instance();
    eventQueue.call(Callback<void()>(&ble, &BLE::processEvents));
}

void print_gauge_info()
{
    printf("Capacity : %.3f mAh\n", gauge.reported_capacity());
    printf("SOC: %.3f percent\n", gauge.state_of_charge());
    printf("Voltage : %.3f Volts\n", gauge.cell_voltage()/1000);
    printf("Current : %.3f mA\n", gauge.average_current());
    printf("Time To Empty: %.2f hours\n", gauge.time_to_empty()/3600);
    printf("Temperature: %f °C\n\n", gauge.temperature());
}

void update_battery_info()
{
    if (BLE::Instance().getGapState().connected) {
    	battery_level = static_cast<uint8_t>(gauge.state_of_charge());
        battery_service->updateBatteryLevel(battery_level);
    }
}

int main()
{
	pc.baud(115200);
    printf("Start up...\n\r");
    printf("SystemCoreClock : %d\n", SystemCoreClock);

	if (bno.initialize()) {
		printf("BNO initialized !\n");
	}
	else {
		printf("Fail to initialize BNO055 ! \n");
	}

	/*if (gauge.configure(1, 800, 3.3, false, false)) {
		printf("Gauge initialized ! \n");
	}
	else {
		printf("Fail to initialized MAX17201 gauge ! \n");
	}*/

	battery_level = static_cast<uint8_t>(gauge.state_of_charge());

	gauge.set_current_alerts(80, -40);
	gauge.enable_alerts();

    BLE &ble = BLE::Instance();
    ble.onEventsToProcess(scheduleBleEventsProcessing);
    ble.init(bleInitComplete);
    ble.gap().onConnection(connectionCallback);

    eventQueue.call_every(20, periodicCallback); // 50 Hz
    //eventQueue.call_every(2000, print_gauge_info);
    eventQueue.call_every(5000, update_battery_info);
    eventQueue.dispatch_forever();

    return 0;
}
