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
#include "ble/services/EnvironmentalService.h"
#include "max17201.h"
#include "bno055.h"
#include "bme280.h"

using namespace sixtron;

#define NEED_LOG 						0 /* Set this if you need debug messages on the console; */
#define BLE_CONNECTION_INTERVAL_MIN		16 /* The BLE connection interval in unit of 1.25 ms */
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
I2C i2c(I2C_SDA, I2C_SCL);
Serial pc(SERIAL_TX, SERIAL_RX);

/* Time management objects */
static EventQueue bleQueue; //(/* event count */ 16 * /* event size */ 32);
static int blink_id;
static IWDG_HandleTypeDef   IwdgHandle; // Watchdog
static uint32_t uwLsiFreq = 0;

/* sensors */
static MAX17201 gauge(&i2c);
static BNO055 bno(&i2c);
static BME280 bme(&i2c);

/* Sensors data */
static float temperature;
static float pressure;
static float humidity;
static uint8_t battery_level = 50;
static bno055_raw_quaternion_t quat;
static bno055_accel_t  accel;
static bno055_gyro_t gyro;
static bno055_mag_t mag;
static bno055_euler_t euler;
static uint8_t inertial_data[20];

static Gap::Handle_t gap_h;
static Gap::ConnectionParams_t gap_params;

const static char     DEVICE_NAME[] = "PIERRE-MARIE";
static const uint16_t uuid16_list[] = {GattService::UUID_BATTERY_SERVICE, GattService::UUID_ENVIRONMENTAL_SERVICE};
static uint8_t stream_config = 0;

static UARTService *uartServicePtr;
static BatteryService * batteryServicePtr;
static EnvironmentalService * envServicePtr;

void blink()
{
    led1 = !led1;
}

void uartDataWrittenCallback(const GattWriteCallbackParams * params)
{
    if (params->handle == uartServicePtr->getTXCharacteristicHandle()) {
        printf("TX message: %.*s\n", params->len, params->data);
        if (params->len == 1) {
        	stream_config = *params->data;
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
    batteryServicePtr = new BatteryService(ble, battery_level);
    envServicePtr = new EnvironmentalService(ble);

    ble.gattServer().onDataWritten(uartDataWrittenCallback);

    uint8_t deviceID[5] = {0x36, 0x54, 0x52, 0x4F, 0x4E};
    /* Setup advertising. */
    ble.gap().accumulateAdvertisingPayload(GapAdvertisingData::BREDR_NOT_SUPPORTED | GapAdvertisingData::LE_GENERAL_DISCOVERABLE);
    ble.gap().setAdvertisingType(GapAdvertisingParams::ADV_CONNECTABLE_UNDIRECTED);
    ble.gap().accumulateAdvertisingPayload(GapAdvertisingData::COMPLETE_LOCAL_NAME, (uint8_t *)DEVICE_NAME, sizeof(DEVICE_NAME));
    ble.gap().accumulateAdvertisingPayload(GapAdvertisingData::MANUFACTURER_SPECIFIC_DATA, (uint8_t *) deviceID, 5);
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
        batteryServicePtr->updateBatteryLevel(battery_level);
    }
}

void update_environmental_data() {
    // get environment data from sensor
    temperature = bme.temperature();
    printf("temperature: %f\n", temperature);
    pressure = bme.pressure();
    humidity = bme.humidity();

    if (!isnan(temperature)) {
    	envServicePtr->updateTemperature(temperature);
    }
    if (!isnan(humidity)) {
    	envServicePtr->updateHumidity(humidity);
    }
    if (!isnan(pressure)) {
    	envServicePtr->updatePressure(pressure);
    }

}

void update_inertial_data() {

	switch(stream_config) {
	case 0x00: // Sensors raw data
		bno.read_accel(&accel);
		bno.read_gyro(&gyro);
		bno.read_mag(&mag);

		inertial_data[0] = stream_config; // Stream configuration
		inertial_data[1] = (int16_t(accel.x*100) & 0xFF); inertial_data[2] = (int16_t(accel.x*100) >> 8) & 0xFF; 	 // accel
		inertial_data[3] = (int16_t(accel.y*100) & 0xFF); inertial_data[4] = (int16_t(accel.y*100) >> 8) & 0xFF; 	 // accel
		inertial_data[5] = (int16_t(accel.z*100) & 0xFF); inertial_data[6] = (int16_t(accel.z*100) >> 8) & 0xFF; 	 // accel

		inertial_data[7] = (int16_t(gyro.x*100) & 0xFF); inertial_data[8] = (int16_t(gyro.x*100) >> 8) & 0xFF; 	 	 // gyro
		inertial_data[9] = (int16_t(gyro.y*100) & 0xFF); inertial_data[10] = (int16_t(gyro.y*100) >> 8) & 0xFF; 	 // gyro
		inertial_data[11] = (int16_t(gyro.z*100) & 0xFF); inertial_data[12] = (int16_t(gyro.z*100) >> 8) & 0xFF; 	 // gyro

		inertial_data[13] = (int16_t(mag.x*100) & 0xFF); inertial_data[14] = (int16_t(mag.x*100) >> 8) & 0xFF; 		 // mag
		inertial_data[15] = (int16_t(mag.y*100) & 0xFF); inertial_data[16] = (int16_t(mag.y*100) >> 8) & 0xFF; 		 // mag
		inertial_data[17] = (int16_t(mag.z*100) & 0xFF); inertial_data[18] = (int16_t(mag.z*100) >> 8) & 0xFF; 		 // mag

		inertial_data[19] = 0x0A; // "\n"
	    //BLE_PRINT2(inertial_data, 20);
		break;

	case 0x01: // Orientation (Euler) + Accelerometer
		bno.read_euler(&euler);
		bno.read_accel(&accel);

		inertial_data[0] = stream_config; // Stream configuration
		inertial_data[1] = (int16_t(accel.x*100) & 0xFF); inertial_data[2] = (int16_t(accel.x*100) >> 8) & 0xFF; 	 // accel
		inertial_data[3] = (int16_t(accel.y*100) & 0xFF); inertial_data[4] = (int16_t(accel.y*100) >> 8) & 0xFF; 	 // accel
		inertial_data[5] = (int16_t(accel.z*100) & 0xFF); inertial_data[6] = (int16_t(accel.z*100) >> 8) & 0xFF; 	 // accel

		inertial_data[7] = (int16_t(euler.x*180/3.14*100) & 0xFF); inertial_data[8] = (int16_t(euler.x*180/3.14*1000) >> 8) & 0xFF; 	 // euler
		inertial_data[9] = (int16_t(euler.y*180/3.14*100) & 0xFF); inertial_data[10] = (int16_t(euler.y*180/3.14*100) >> 8) & 0xFF; 	 // euler
		inertial_data[11] = (int16_t(euler.z*180/3.14*100) & 0xFF); inertial_data[12] = (int16_t(euler.z*180/3.14*100) >> 8) & 0xFF; 	 // euler

		inertial_data[13] = 0x0A; // "\n"
	    //BLE_PRINT2(inertial_data, 14);
		break;

	case 0x02:
		bno.read_quaternion(&quat);
		bno.read_accel(&accel);

		inertial_data[0] = stream_config; // Stream configuration
		inertial_data[1] = (int16_t(accel.x*100) & 0xFF); inertial_data[2] = (int16_t(accel.x*100) >> 8) & 0xFF; 	 // accel
		inertial_data[3] = (int16_t(accel.y*100) & 0xFF); inertial_data[4] = (int16_t(accel.y*100) >> 8) & 0xFF; 	 // accel
		inertial_data[5] = (int16_t(accel.z*100) & 0xFF); inertial_data[6] = (int16_t(accel.z*100) >> 8) & 0xFF; 	 // accel

		inertial_data[7] = (int16_t(quat.w) & 0xFF); inertial_data[8] = (int16_t(quat.w) >> 8) & 0xFF; 	 	 // quat
		inertial_data[9] = (int16_t(quat.x) & 0xFF); inertial_data[10] = (int16_t(quat.x) >> 8) & 0xFF; 	 // quat
		inertial_data[11] = (int16_t(quat.y) & 0xFF); inertial_data[12] = (int16_t(quat.y) >> 8) & 0xFF; 	 // quat
		inertial_data[13] = (int16_t(quat.z) & 0xFF); inertial_data[14] = (int16_t(quat.z) >> 8) & 0xFF; 	 // quat

		inertial_data[15] = 0x0A; // "\n"
		//printf("QUA:%d,%d,%d,%d\n", quat.w, quat.x, quat.y, quat.z);
	    BLE_PRINT2(inertial_data, 16);
	    //printf("%.3f %.3f %.3f\n", accel.x, accel.y, accel.z);
		break;

	default:
		bno.read_accel(&accel);
		bno.read_gyro(&gyro);
		bno.read_mag(&mag);

		inertial_data[0] = stream_config; // Stream configuration
		inertial_data[1] = (int16_t(accel.x*100) & 0xFF); inertial_data[2] = (int16_t(accel.x*100) >> 8) & 0xFF; 	 // accel
		inertial_data[3] = (int16_t(accel.y*100) & 0xFF); inertial_data[4] = (int16_t(accel.y*100) >> 8) & 0xFF; 	 // accel
		inertial_data[5] = (int16_t(accel.z*100) & 0xFF); inertial_data[6] = (int16_t(accel.z*100) >> 8) & 0xFF; 	 // accel

		inertial_data[7] = (int16_t(gyro.x*100) & 0xFF); inertial_data[8] = (int16_t(gyro.x*100) >> 8) & 0xFF; 	 	 // gyro
		inertial_data[9] = (int16_t(gyro.y*100) & 0xFF); inertial_data[10] = (int16_t(gyro.y*100) >> 8) & 0xFF; 	 // gyro
		inertial_data[11] = (int16_t(gyro.z*100) & 0xFF); inertial_data[12] = (int16_t(gyro.z*100) >> 8) & 0xFF; 	 // gyro

		inertial_data[13] = (int16_t(mag.x*100) & 0xFF); inertial_data[14] = (int16_t(mag.x*100) >> 8) & 0xFF; 		 // mag
		inertial_data[15] = (int16_t(mag.y*100) & 0xFF); inertial_data[16] = (int16_t(mag.y*100) >> 8) & 0xFF; 		 // mag
		inertial_data[17] = (int16_t(mag.z*100) & 0xFF); inertial_data[18] = (int16_t(mag.z*100) >> 8) & 0xFF; 		 // mag

		inertial_data[19] = 0x0A; // "\n"
	    //BLE_PRINT2(inertial_data, 20);
		break;
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

    /* GAUGE */
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

    /* BME280 */
    if (!bme.initialize()){
        printf("Couldn't initialize the BME280...\n");
        return -1;
    }

    bme.set_sampling(BME280::SensorMode::NORMAL,
           BME280::SensorSampling::OVERSAMPLING_X1,
           BME280::SensorSampling::OVERSAMPLING_X1,
           BME280::SensorSampling::OVERSAMPLING_X1,
           BME280::SensorFilter::OFF,
           BME280::StandbyDuration::MS_1000);

    /* BNO055 */
    if (bno.initialize(BNO055::OperationMode::NDOF, true) != true) {
    	pc.printf("ERROR BNO055 not detected. Check your wiring and BNO I2C address\n");
    	return 0;
        }


    /* ========== Configure tasks ========== */

    BLE &ble = BLE::Instance();
    ble.onEventsToProcess(scheduleBleEventsProcessing);
    ble.init(bleInitComplete);
    ble.gap().onConnection(connectionCallback);

    user_button.rise(button_handler);
    bleQueue.call_every(30000, update_battery_info);
    bleQueue.call_every(2000, watchdog_refresh);
    bleQueue.call_every(5000, update_environmental_data);
    bleQueue.call_every(20, update_inertial_data);
    bleQueue.dispatch_forever();
    return 0;
}
