/*
 * Copyright (c) 2017, CATIE, All Rights Reserved
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <events/mbed_events.h>
#include <mbed.h>
#include "ble/BLE.h"
#include "ble/Gap.h"
#include "ble/services/UARTService.h"
#include "ble/services/BatteryService.h"
#include "max17201.h"

using namespace sixtron;

#define NEED_LOG 						0 /* Set this if you need debug messages on the console; */
#define BLE_CONNECTION_INTERVAL_MIN		32 /* The BLE connection interval in unit of 1.25 ms */
#define BLE_CONNECTION_INTERVAL_MAX		32

#define BLE_PRINT(STR) { if (uartServicePtr) uartServicePtr->write(STR, strlen(STR)); wait_ms(BLE_CONNECTION_INTERVAL_MAX*1.25);}
#define BLE_PRINT2(STR, SIZE) { if (uartServicePtr) uartServicePtr->write(STR, SIZE); wait_ms(BLE_CONNECTION_INTERVAL_MAX*1.25);}

#if NEED_LOG
#define LOG(STR)     printf(STR)
#else
#define LOG(...) /* nothing */
#endif /* #if NEED_CONSOLE_OUTPUT */

/* Peripherals definitions */
InterruptIn user_button(USER_BUTTON);
DigitalOut led1(LED1, 1);
//DigitalIn gaugeAlert(DIO4);
I2C i2c(I2C_SDA, I2C_SCL);
Serial pc(SERIAL_TX, SERIAL_RX);

/* Time management objects */
static EventQueue bleQueue; //(/* event count */ 16 * /* event size */ 32);
static int blink_id;
static IWDG_HandleTypeDef   IwdgHandle; // Watchdog
static uint32_t uwLsiFreq = 0;

/* sensors */
static MAX17201 gauge(&i2c);

static Gap::Handle_t gap_h;
static Gap::ConnectionParams_t gap_params;

const static char     DEVICE_NAME[] = "NODE 1";
static const uint16_t uuid16_list[] = {GattService::UUID_BATTERY_SERVICE};

static UARTService *uartServicePtr;
static BatteryService * battery_service;
static uint8_t battery_level = 50;

void blink()
{
    led1 = !led1;
}

void uartDataWrittenCallback(const GattWriteCallbackParams * params)
{
    if (params->handle == uartServicePtr->getTXCharacteristicHandle()) {
        printf("TX message: %.*s\n", params->len, params->data);
        //printf("The string is: %.*s\n", params->len, string);
        if (strcmp((const char*) params->data, "SCAN") == 0) {
            LOG("START scan !\n");
        }
    }
}

void disconnectionCallback(const Gap::DisconnectionCallbackParams_t *params)
{
    LOG("Disconnected!\n");
    /* If local host terminated connection, we assume that it is to start a scan so we dont restart advertising  */
    if (params->reason != Gap::DisconnectionReason_t::LOCAL_HOST_TERMINATED_CONNECTION) {
        blink_id = bleQueue.call_every(200, blink);
        BLE::Instance().gap().startAdvertising(); // restart advertising
    }
}

void updateConnectionParams()
{
    BLE &ble = BLE::Instance();
    if (ble.getGapState().connected) {
        gap_params.connectionSupervisionTimeout = 500;
        gap_params.minConnectionInterval = BLE_CONNECTION_INTERVAL_MIN;
        gap_params.maxConnectionInterval = BLE_CONNECTION_INTERVAL_MAX;
        gap_params.slaveLatency = 0;

        LOG("Request a connection params update!\n");
        if (ble.gap().updateConnectionParams(gap_h, &gap_params) != BLE_ERROR_NONE) {
            LOG("Error sending the request!\n");
        }
    }
}

void connectionCallback (const Gap::ConnectionCallbackParams_t *params)
{
    gap_h = params->handle;
    bleQueue.cancel(blink_id);
    led1 = 1;
    LOG("Connected !\n");

    bleQueue.call(updateConnectionParams);
}

void onBleInitError(BLE &ble, ble_error_t error)
{
    (void)ble;
    (void)error;
   /* Initialization error handling should go here */
    printf("Error: %d\n", error);
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

    gap_params.connectionSupervisionTimeout = 500;
    gap_params.minConnectionInterval = BLE_CONNECTION_INTERVAL_MIN;
    gap_params.maxConnectionInterval = BLE_CONNECTION_INTERVAL_MAX;
    gap_params.slaveLatency = 0;

    ble.gap().setPreferredConnectionParams(&gap_params);

    ble.gap().onDisconnection(disconnectionCallback);

    /* Setup primary service. */
    uartServicePtr = new UARTService(ble);
    battery_service = new BatteryService(ble, battery_level);

    ble.gattServer().onDataWritten(uartDataWrittenCallback);

    /* Setup advertising. */
    ble.gap().accumulateAdvertisingPayload(GapAdvertisingData::BREDR_NOT_SUPPORTED | GapAdvertisingData::LE_GENERAL_DISCOVERABLE);
    ble.gap().setAdvertisingType(GapAdvertisingParams::ADV_CONNECTABLE_UNDIRECTED);
    ble.gap().accumulateAdvertisingPayload(GapAdvertisingData::COMPLETE_LOCAL_NAME, (uint8_t *)DEVICE_NAME, sizeof(DEVICE_NAME));
    ble.gap().accumulateAdvertisingPayload(GapAdvertisingData::COMPLETE_LIST_16BIT_SERVICE_IDS, (uint8_t *) uuid16_list, sizeof(uuid16_list));
    ble.gap().accumulateAdvertisingPayload(GapAdvertisingData::COMPLETE_LIST_128BIT_SERVICE_IDS, (uint8_t *)UARTServiceUUID_reversed, sizeof(UARTServiceUUID_reversed));

    ble.gap().setAdvertisingInterval(500); /* 500ms */
    ble.gap().startAdvertising();
    blink_id = bleQueue.call_every(200, blink);
}

void scheduleBleEventsProcessing(BLE::OnEventsToProcessCallbackContext* context) {
    BLE &ble = BLE::Instance();
    bleQueue.call(Callback<void()>(&ble, &BLE::processEvents));
}

void print_gauge_info()
{
    char buffer[128];

    sprintf(buffer, "Capacity : %.3f mAh\n", gauge.reported_capacity());
    BLE_PRINT2(buffer, strlen(buffer));

    sprintf(buffer, "SOC: %.3f percent\n", gauge.state_of_charge());
    BLE_PRINT2(buffer, strlen(buffer));

    sprintf(buffer,"Voltage : %.3f Volts\n", gauge.cell_voltage()/1000);
    BLE_PRINT2(buffer, strlen(buffer));

    sprintf(buffer, "Current : %.3f mA\n", gauge.average_current());
    BLE_PRINT2(buffer, strlen(buffer));

    sprintf(buffer, "Time To Empty: %.2f hours\n", gauge.time_to_empty()/3600);
    BLE_PRINT2(buffer, strlen(buffer));

    sprintf(buffer, "Temperature: %f °C\n\n", gauge.temperature());
    BLE_PRINT2(buffer, strlen(buffer));
}

void update_battery_info()
{
    battery_level = static_cast<uint8_t>(gauge.state_of_charge());
    if (BLE::Instance().getGapState().connected) {
        battery_service->updateBatteryLevel(battery_level);
    }
}

void button_handler()
{
    bleQueue.call(print_gauge_info);
}

void watchdog_refresh()
{
	HAL_IWDG_Refresh(&IwdgHandle);
}

int main()
{
    pc.baud(115200);
    i2c.frequency(400000);
    printf("Start up...\n\r");
    printf("SystemCoreClock : %d\n", SystemCoreClock);

    /* ========== Init WatchDog ========== */

    /* Clear reset flags in any cases */
    __HAL_RCC_CLEAR_RESET_FLAGS(); // clear reset flags that may be rised by watchdog

    /*# Get the LSI frequency */
    uwLsiFreq = LSI_VALUE;

    /*# Configure & Start the IWDG peripheral #########################################*/
    /* Set counter reload value to obtain 4 sec. IWDG TimeOut.
     IWDG counter clock Frequency = uwLsiFreq
     Set Prescaler to 128 (IWDG_PRESCALER_128)
     Timeout Period = (Reload Counter Value * 32) / uwLsiFreq
     So Set Reload Counter Value = uwLsiFreq / 32 */
    IwdgHandle.Instance = IWDG;
    IwdgHandle.Init.Prescaler = IWDG_PRESCALER_128;
    IwdgHandle.Init.Reload = (uwLsiFreq / 32 );
    IwdgHandle.Init.Window = IWDG_WINDOW_DISABLE;

    if(HAL_IWDG_Init(&IwdgHandle) != HAL_OK)
    {
    /* Initialization Error */
            printf("Can't init Watchdog !\n");
    }

    /* ========== Init Components ========== */
    /* If gauge design capacity is 750 mAh, it means that the gauge has lost its
     * configuration and learning so it needs to be configured again */
    if (gauge.design_capacity() == 750) {
    	if (gauge.configure(1, 800, 3.1, false, false)) {
    		battery_level = uint8_t(gauge.state_of_charge());
    		printf("Gauge initialized ! \n");
    	} else {
            printf("Fail to initialized MAX17201 gauge ! \n");
    	}
    } else {
    	printf("Gauge already configured\n");
    	battery_level = uint8_t(gauge.state_of_charge());
    }

    /* ========== Configure tasks ========== */

    BLE &ble = BLE::Instance();
    ble.onEventsToProcess(scheduleBleEventsProcessing);
    ble.init(bleInitComplete);
    ble.gap().onConnection(connectionCallback);

    user_button.rise(button_handler);
    bleQueue.call_every(30000, update_battery_info);
    bleQueue.call_every(2000, watchdog_refresh);
    bleQueue.dispatch_forever();
    return 0;
}
