/*
 * Zmotion demo
 * Copyright (c) 2018, CATIE
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/*
 * ZestMotion.cpp
 *
 *  Created on: 14 avr. 2019
 *      Author: sepro
 *      MBED 5.10.4
 */

#include "ZestMotion.h"


using namespace sixtron;

static SWO swo;

/*
 * Zest Motion Class constructor
 *
 */
ZestMotion::ZestMotion(BLE &ble, events::EventQueue &event_queue, MAX17201 &gauge, BME280 &bme, BNO055 &bno) :
		_ble(ble),
		_event_queue(event_queue),
		_led1(LED1),
		_gap_params(),

		//_i2c(I2C_SDA, I2C_SCL),
		_gauge(gauge),
		_battery_level(50),
		_battery_servicePtr(NULL),

		_bme(bme),
		_env_servicePtr(NULL),

		_bno(bno),
		_stream_config(0),
		_uart_servicePtr(NULL)
{
}

/*
 * Default destructor
 */
ZestMotion::~ZestMotion()
{
}


/*
 * Méthode publique start
 * Appelé pour initialiser l'objet ZestMotion
 *
 */
void ZestMotion::start() {
	ble_error_t error;
	swo.printf("Start\n");
	_ble.gap().setEventHandler(this);

	if (_ble.hasInitialized()) {
		swo.printf("Ble instance already initialised.\r\n");
		return;
	}
	error = _ble.init(this, &ZestMotion::on_init_complete);
	if (error) {
		swo.printf("Error returned by BLE::init.\r\n");
		return;
	}

	/* to show we're running we'll blink every 500ms */
	_blink_id = _event_queue.call_every(500, this, &ZestMotion::blink);
	_event_queue.call_every(1000, this, &ZestMotion::update_battery_value);
	_event_queue.call_every(5000, this, &ZestMotion::update_environmental_data);


	/* this will not return until shutdown */
	_event_queue.dispatch_forever();
}


/*
 * Callbak d'initialisation executé lors de l'appel ble.init()
 * Le callback est responsable du paramétrage du GAP
 * Il met en place les callback appelé lors de la connexion et la déconnexion de l'objet
 * Mise en place des GattService
 *
 */
void ZestMotion::on_init_complete(BLE::InitializationCompleteCallbackContext *event)
{
	swo.printf("On_init_complete\n");
	if (event->error) {
		swo.printf("Error during the initialisation\r\n");
		return;
	}
	event->ble.gap().setDeviceName((uint8_t*) DEVICE_NAME);

	_gap_params.connectionSupervisionTimeout = 500;
	_gap_params.minConnectionInterval = 16;
	_gap_params.maxConnectionInterval = 16;
	_gap_params.slaveLatency = 0;

	event->ble.gap().setPreferredConnectionParams(&_gap_params);

	event->ble.gap().onConnection(this, &ZestMotion::on_connect);
	event->ble.gap().onDisconnection(this, &ZestMotion::on_disconnect);


	_battery_servicePtr = new BatteryService(event->ble, _battery_level);
	_env_servicePtr = new EnvironmentalService(event->ble);
	_uart_servicePtr = new UARTService(event->ble);

	_ble.gattServer().onDataWritten(this, &ZestMotion::uartDataWrittenCallback);

	start_advertising();
}


/*
 * Méthode start_advertising()
 * reponsable de l'advertising de l'objet avant connexion
 *
 */
void ZestMotion::start_advertising(){
	uint8_t deviceID[5] = { 0x36, 0x54, 0x52, 0x4F, 0x4E };

	swo.printf("Start Advertising\n");
	_ble.gap().accumulateAdvertisingPayload(
			GapAdvertisingData::BREDR_NOT_SUPPORTED
			| GapAdvertisingData::LE_GENERAL_DISCOVERABLE);

	_ble.gap().setAdvertisingType(
			GapAdvertisingParams::ADV_CONNECTABLE_UNDIRECTED);

	_ble.gap().accumulateAdvertisingPayload(
			GapAdvertisingData::COMPLETE_LOCAL_NAME, (uint8_t *) DEVICE_NAME,
			sizeof(DEVICE_NAME));

	_ble.gap().accumulateAdvertisingPayload(
			GapAdvertisingData::MANUFACTURER_SPECIFIC_DATA,
			(uint8_t *) deviceID, 5);

	_ble.gap().accumulateAdvertisingPayload(
			GapAdvertisingData::COMPLETE_LIST_16BIT_SERVICE_IDS,
			(uint8_t *) uuid16_list, sizeof(uuid16_list));

	_ble.gap().setAdvertisingInterval(500); /* 500ms */
	_ble.gap().startAdvertising();

}
/*
 * Callback de connexion
 * Appelé lorsque l'event connection est détecté
 *
 */
void ZestMotion::on_connect(const Gap::ConnectionCallbackParams_t *connection_event)
{
	_gap_h = connection_event->handle;
	swo.printf("Connected in \r\n");
	_event_queue.cancel(_blink_id);
	_led1 = 1;

	_bno.set_power_mode(BNO055::PowerMode::NORMAL);
	wait_ms(800);
	_bno.set_operation_mode(BNO055::OperationMode::NDOF);
	_inertial_id = _event_queue.call_every(20, this, &ZestMotion::update_inertial_data);

	//updateConnectionParams();
};


/*
 * Useless
 */
void ZestMotion::updateConnectionParams()
{
	if (_ble.gap().getState().connected) {
		_gap_params.connectionSupervisionTimeout = 500;
		_gap_params.minConnectionInterval = 16;
		_gap_params.maxConnectionInterval = 16;
		_gap_params.slaveLatency = 0;

		swo.printf("Request a connection params update!\n");
		if (_ble.gap().updateConnectionParams(_gap_h, &_gap_params) != BLE_ERROR_NONE) {
			swo.printf("Error sending the request!\n");
		}
	}
}

/*
 * Callback de déconnexion.
 * Sert à reprendre l'advertising en attendant une nouvelle connexion.
 *
 */

void ZestMotion::on_disconnect(const Gap::DisconnectionCallbackParams_t *event)
{
	swo.printf("Disconnected in \r\n");

	_event_queue.cancel(_inertial_id);
	_bno.set_operation_mode(BNO055::OperationMode::CONFIG);
	_bno.set_power_mode(BNO055::PowerMode::SUSPEND);

	_blink_id = _event_queue.call_every(200, this, &ZestMotion::blink);
	_ble.gap().startAdvertising();
};


void ZestMotion::blink(void)
{
	_led1 = !_led1;
};


/*
 * Méthode update battery value
 * Appelé périodiquement par l'eventQueue, elle envoie les données de batterie lorsque l'objet est connecté
 *
 */
void ZestMotion::update_battery_value() {
	//swo.printf("Update sensor info CB !\n");
	_battery_level = static_cast<uint8_t>(_gauge.state_of_charge());
	if (_ble.gap().getState().connected) {
		_battery_servicePtr->updateBatteryLevel(_battery_level);
		//swo.printf("Update Battery info!\n");
	}
}

void ZestMotion::update_environmental_data()
{
	// get environment data from sensor
	//swo.printf("Update environmental info\n");
	_temperature = _bme.temperature();
	_pressure = _bme.pressure();
	_humidity = _bme.humidity();

	if (!isnan(_temperature)) {
		_env_servicePtr->updateTemperature(_temperature);
	}
	if (!isnan(_humidity)) {
		_env_servicePtr->updateHumidity(_humidity);
	}
	if (!isnan(_pressure)) {
		_env_servicePtr->updatePressure(_pressure);
	}

}

void ZestMotion::uartDataWrittenCallback(const GattWriteCallbackParams * params)
{
    if (params->handle == _uart_servicePtr->getTXCharacteristicHandle()) {
        swo.printf("TX message: %.*s\n", params->len, params->data);
        if (params->len == 1) {
            _stream_config = *params->data;
        }
        if (strcmp((const char*) params->data, "DFU") == 0) {
            swo.printf("Entering DFU mode !\n");
            mbed_start_application(STM32L496RG_BOOTLOADER_ADDRESS);
        }

    }
}

void ZestMotion::update_inertial_data()
{
    /* Inertial data are sent through the UART service
     * By writing on the RX characteristic, the master (Smartphone application for example)
     * can choose which data are sent through the BLE notifications (0x02 for the 6TRON application)
     */
	//swo.printf("update inertial data \n");
    switch (_stream_config) {
    case 0x00: // Sensors raw data
        _accel = _bno.accelerometer();
        _gyro = _bno.gyroscope();
        _mag = _bno.magnetometer();

        _inertial_data[0] = _stream_config; // Stream configuration
        _inertial_data[1] = (int16_t(_accel.x * 100) & 0xFF);
        _inertial_data[2] = (int16_t(_accel.x * 100) >> 8) & 0xFF; 	 // accel
        _inertial_data[3] = (int16_t(_accel.y * 100) & 0xFF);
        _inertial_data[4] = (int16_t(_accel.y * 100) >> 8) & 0xFF; 	 // accel
        _inertial_data[5] = (int16_t(_accel.z * 100) & 0xFF);
        _inertial_data[6] = (int16_t(_accel.z * 100) >> 8) & 0xFF; 	 // accel

        _inertial_data[7] = (int16_t(_gyro.x * 100) & 0xFF);
        _inertial_data[8] = (int16_t(_gyro.x * 100) >> 8) & 0xFF; 	 // _gyro
        _inertial_data[9] = (int16_t(_gyro.y * 100) & 0xFF);
        _inertial_data[10] = (int16_t(_gyro.y * 100) >> 8) & 0xFF; 	 // _gyro
        _inertial_data[11] = (int16_t(_gyro.z * 100) & 0xFF);
        _inertial_data[12] = (int16_t(_gyro.z * 100) >> 8) & 0xFF; 	 // _gyro

        _inertial_data[13] = (int16_t(_mag.x * 100) & 0xFF);
        _inertial_data[14] = (int16_t(_mag.x * 100) >> 8) & 0xFF; 		 // _mag
        _inertial_data[15] = (int16_t(_mag.y * 100) & 0xFF);
        _inertial_data[16] = (int16_t(_mag.y * 100) >> 8) & 0xFF; 		 // _mag
        _inertial_data[17] = (int16_t(_mag.z * 100) & 0xFF);
        _inertial_data[18] = (int16_t(_mag.z * 100) >> 8) & 0xFF; 		 // _mag

        _inertial_data[19] = 0x0A; // "\n"
        inertial_data_write(_inertial_data, 20);
        break;

    case 0x01: // Orientation (_euler) + Accelerometer
        _euler = _bno.euler();
        _accel = _bno.accelerometer();

        _inertial_data[0] = _stream_config; // Stream configuration
        _inertial_data[1] = (int16_t(_accel.x * 100) & 0xFF);
        _inertial_data[2] = (int16_t(_accel.x * 100) >> 8) & 0xFF; 	 // accel
        _inertial_data[3] = (int16_t(_accel.y * 100) & 0xFF);
        _inertial_data[4] = (int16_t(_accel.y * 100) >> 8) & 0xFF; 	 // accel
        _inertial_data[5] = (int16_t(_accel.z * 100) & 0xFF);
        _inertial_data[6] = (int16_t(_accel.z * 100) >> 8) & 0xFF; 	 // accel

        _inertial_data[7] = (int16_t(_euler.x * 180 / 3.14 * 100) & 0xFF);
        _inertial_data[8] = (int16_t(_euler.x * 180 / 3.14 * 1000) >> 8) & 0xFF; // _euler
        _inertial_data[9] = (int16_t(_euler.y * 180 / 3.14 * 100) & 0xFF);
        _inertial_data[10] = (int16_t(_euler.y * 180 / 3.14 * 100) >> 8) & 0xFF; // _euler
        _inertial_data[11] = (int16_t(_euler.z * 180 / 3.14 * 100) & 0xFF);
        _inertial_data[12] = (int16_t(_euler.z * 180 / 3.14 * 100) >> 8) & 0xFF; // _euler

        _inertial_data[13] = 0x0A; // "\n"
        inertial_data_write(_inertial_data, 14);
        break;

    case 0x02:
        _quat = _bno.raw_quaternion();
        _accel = _bno.accelerometer();

        _inertial_data[0] = _stream_config; // Stream configuration
        _inertial_data[1] = (int16_t(_accel.x * 100) & 0xFF);
        _inertial_data[2] = (int16_t(_accel.x * 100) >> 8) & 0xFF; 	 // _accel
        _inertial_data[3] = (int16_t(_accel.y * 100) & 0xFF);
        _inertial_data[4] = (int16_t(_accel.y * 100) >> 8) & 0xFF; 	 // _accel
        _inertial_data[5] = (int16_t(_accel.z * 100) & 0xFF);
        _inertial_data[6] = (int16_t(_accel.z * 100) >> 8) & 0xFF; 	 // accel

        _inertial_data[7] = (int16_t(_quat.w) & 0xFF);
        _inertial_data[8] = (int16_t(_quat.w) >> 8) & 0xFF; 	 // _quat
        _inertial_data[9] = (int16_t(_quat.x) & 0xFF);
        _inertial_data[10] = (int16_t(_quat.x) >> 8) & 0xFF; 	 // _quat
        _inertial_data[11] = (int16_t(_quat.y) & 0xFF);
        _inertial_data[12] = (int16_t(_quat.y) >> 8) & 0xFF; 	 // _quat
        _inertial_data[13] = (int16_t(_quat.z) & 0xFF);
        _inertial_data[14] = (int16_t(_quat.z) >> 8) & 0xFF; 	 // _quat

        _inertial_data[15] = 0x0A; // "\n"
        inertial_data_write(_inertial_data, 16);
        break;

    default:
        _accel = _bno.accelerometer();
        _gyro = _bno.gyroscope();
        _mag = _bno.magnetometer();

        _inertial_data[0] = _stream_config; // Stream configuration
        _inertial_data[1] = (int16_t(_accel.x * 100) & 0xFF);
        _inertial_data[2] = (int16_t(_accel.x * 100) >> 8) & 0xFF; 	 // accel
        _inertial_data[3] = (int16_t(_accel.y * 100) & 0xFF);
        _inertial_data[4] = (int16_t(_accel.y * 100) >> 8) & 0xFF; 	 // accel
        _inertial_data[5] = (int16_t(_accel.z * 100) & 0xFF);
        _inertial_data[6] = (int16_t(_accel.z * 100) >> 8) & 0xFF; 	 // accel

        _inertial_data[7] = (int16_t(_gyro.x * 100) & 0xFF);
        _inertial_data[8] = (int16_t(_gyro.x * 100) >> 8) & 0xFF; 	 // _gyro
        _inertial_data[9] = (int16_t(_gyro.y * 100) & 0xFF);
        _inertial_data[10] = (int16_t(_gyro.y * 100) >> 8) & 0xFF; 	 // _gyro
        _inertial_data[11] = (int16_t(_gyro.z * 100) & 0xFF);
        _inertial_data[12] = (int16_t(_gyro.z * 100) >> 8) & 0xFF; 	 // _gyro

        _inertial_data[13] = (int16_t(_mag.x * 100) & 0xFF);
        _inertial_data[14] = (int16_t(_mag.x * 100) >> 8) & 0xFF; 		 // _mag
        _inertial_data[15] = (int16_t(_mag.y * 100) & 0xFF);
        _inertial_data[16] = (int16_t(_mag.y * 100) >> 8) & 0xFF; 		 // _mag
        _inertial_data[17] = (int16_t(_mag.z * 100) & 0xFF);
        _inertial_data[18] = (int16_t(_mag.z * 100) >> 8) & 0xFF; 		 // _mag

        _inertial_data[19] = 0x0A; // "\n"
        inertial_data_write(_inertial_data, 20);
        break;
    }
}

void ZestMotion::inertial_data_write(uint8_t data[20], int size) {
	//swo.printf("inertial data write \n");
if (_uart_servicePtr)
	_uart_servicePtr->write(data, size);

}
