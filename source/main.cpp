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

InterruptIn user_button(USER_BUTTON);

DigitalOut led1(LED1, 1);
I2C i2c(I2C_SDA, I2C_SCL);
Serial pc(SERIAL_TX, SERIAL_RX);

BNO055 bno(&i2c);
MAX17201 gauge(&i2c);

static Gap::Handle_t gap_h;
static Gap::ConnectionParams_t gap_params;

const static char     DEVICE_NAME[] = "FOREARM NODE";

static const uint16_t uuid16_list[] = {GattService::UUID_BATTERY_SERVICE};

static bno055_raw_quaternion_t quat;
static UARTService *uartServicePtr;
static BatteryService * battery_service;
static uint8_t battery_level = 50;
static uint16_t msg_cnt = 0;

static EventQueue eventQueue(
    /* event count */ 16 * /* event size */ 32
);
static int loop_id;

void updateSensorValue()
{
    // Do blocking calls or whatever is necessary for sensor polling.
    // In our case, we simply update the quaternion measurement.
	static uint8_t buffer[11];
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

void advertise()
{
	BLE::Instance().gap().startAdvertising();
}

void stopScan()
{
	BLE::Instance().gap().stopScan();
	wait_ms(200);
    eventQueue.call(advertise);
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

void scanCallback(const Gap::AdvertisementCallbackParams_t * params)
{
	if (params->peerAddr[0] == 0xAB && params->peerAddr[1] == 0xC9) {
		if (params->advertisingDataLen == 4) {
			uint32_t *dataptr = (uint32_t *) params->advertisingData;
			if (*dataptr == 0x04030201) {
				printf("Sync packet received !\n");
				led1 = 1;
				eventQueue.call(stopScan);
			    loop_id = eventQueue.call_every(20, periodicCallback); // 50 Hz
			}
		}
		//printf("addr received: [%02x %02x %02x %02x %02x %02x] RSSI : %d dB Length: %d\n", params->peerAddr[5], params->peerAddr[4], params->peerAddr[3],
		//		params->peerAddr[2], params->peerAddr[1], params->peerAddr[0], params->rssi, params->advertisingDataLen);
	}
}

void waitSyncPacket()
{
	if (BLE::Instance().getGapState().connected) {
		BLE::Instance().gap().disconnect(Gap::DisconnectionReason_t::LOCAL_HOST_TERMINATED_CONNECTION);
			wait_ms(500);
	}

	BLE::Instance().gap().stopAdvertising();

	if (BLE::Instance().gap().startScan(scanCallback) == BLE_ERROR_NONE) {
		eventQueue.cancel(loop_id);
		led1 = 0;
		printf("Scan Started\n");
	}
	else {
		printf("Error while trying to start scan\n");
	}
}

void uartDataWrittenCallback(const GattWriteCallbackParams * params)
{
	if (params->handle == uartServicePtr->getTXCharacteristicHandle()) {
		//char string[params->len];

		printf("TX message: %.*s\n", params->len, params->data);
		//printf("The string is: %.*s\n", params->len, string);
		if (strcmp((const char*) params->data, "SCAN") == 0) {
			printf("START scan !\n");
			waitSyncPacket();
		}
	}
}

void disconnectionCallback(const Gap::DisconnectionCallbackParams_t *params)
{
    BLE::Instance().gap().startAdvertising(); // restart advertising
}

void updateConnectionParams()
{
	BLE &ble = BLE::Instance();
	if (ble.getGapState().connected) {
	    gap_params.connectionSupervisionTimeout = 3200;
	    gap_params.minConnectionInterval = 6;
	    gap_params.maxConnectionInterval = 10;
	    gap_params.slaveLatency = 0;

	    if (ble.gap().updateConnectionParams(gap_h, &gap_params) == BLE_ERROR_NONE) {
	    	printf("Request a connection params update!\n");
	    }
	}
}

void connectionCallback (const Gap::ConnectionCallbackParams_t *params)
{
	gap_h = params->handle;
	led1 = 1;
	printf("Connected !\n");
	printf("min interval: %d\n", params->connectionParams->minConnectionInterval);
	printf("max interval: %d\n", params->connectionParams->maxConnectionInterval);

	updateConnectionParams();
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
    ble.gap().setDeviceName((uint8_t*) DEVICE_NAME);

    gap_params.connectionSupervisionTimeout = 3200;
    gap_params.minConnectionInterval = 6;
    gap_params.maxConnectionInterval = 16;
    gap_params.slaveLatency = 0;

    int8_t ret = ble.gap().setPreferredConnectionParams(&gap_params);

    ble.gap().onDisconnection(disconnectionCallback);

    /* Setup primary service. */
    uartServicePtr = new UARTService(ble);
	battery_service = new BatteryService(ble, battery_level);

    ble.gattServer().onDataWritten(uartDataWrittenCallback);

    /* Setup advertising. */
    ble.gap().accumulateAdvertisingPayload(GapAdvertisingData::BREDR_NOT_SUPPORTED | GapAdvertisingData::LE_GENERAL_DISCOVERABLE);
    ble.gap().setAdvertisingType(GapAdvertisingParams::ADV_CONNECTABLE_UNDIRECTED);
    ble.gap().accumulateAdvertisingPayload(GapAdvertisingData::Appearance_t::GENERIC_RUNNING_WALKING_SENSOR);
    ble.gap().accumulateAdvertisingPayload(GapAdvertisingData::COMPLETE_LOCAL_NAME, (uint8_t *)DEVICE_NAME, sizeof(DEVICE_NAME));
    ble.gap().accumulateAdvertisingPayload(GapAdvertisingData::COMPLETE_LIST_16BIT_SERVICE_IDS, (uint8_t *) uuid16_list, sizeof(uuid16_list));
    ble.gap().accumulateAdvertisingPayload(GapAdvertisingData::COMPLETE_LIST_128BIT_SERVICE_IDS, (uint8_t *)UARTServiceUUID_reversed, sizeof(UARTServiceUUID_reversed));

    ble.gap().setAdvertisingInterval(500); /* 500ms */
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

void button_handler()
{
	eventQueue.call(waitSyncPacket);
}

int main()
{
	pc.baud(115200);
    printf("Start up...\n\r");
    printf("SystemCoreClock : %d\n", SystemCoreClock);

    user_button.rise(button_handler);

	if (bno.initialize(BNO055::OperationMode::OperationMode_NDOF, true)) {
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

    loop_id = eventQueue.call_every(20, periodicCallback); // 50 Hz
    //eventQueue.call_every(2000, print_gauge_info);
    eventQueue.call_every(5000, update_battery_info);
    eventQueue.dispatch_forever();

    return 0;
}
