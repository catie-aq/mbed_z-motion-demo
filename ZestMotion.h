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
 * ZestMotion.h
 *
 *  Created on: 14 avr. 2019
 *      Author: sepro
 */

#ifndef ZESTMOTION_H_
#define ZESTMOTION_H_

#include <events/mbed_events.h>
#include <mbed.h>
#include "ble/BLE.h"
#include "ble/Gap.h"
#include "ble/services/BatteryService.h"
#include "ble/services/EnvironmentalService.h"
#include "ble/services/UARTService.h"

#include "swo.h"

#include "max17201.h"
#include "bme280.h"
#include "bno055.h"


namespace {
#define STM32L496RG_BOOTLOADER_ADDRESS 0x1FFF0000
}


using namespace sixtron;

const static char DEVICE_NAME[] = "6tronNode3";
static const uint16_t uuid16_list[] = { GattService::UUID_BATTERY_SERVICE,
        GattService::UUID_ENVIRONMENTAL_SERVICE };


class ZestMotion : public Gap::EventHandler {
public:

	ZestMotion(BLE &ble, events::EventQueue &event_queue, MAX17201 &gauge, BME280 &bme, BNO055 &bno);
	~ZestMotion();
	void start();

private:

	void on_init_complete(BLE::InitializationCompleteCallbackContext *event);
	void start_advertising();
	void on_connect(const Gap::ConnectionCallbackParams_t *connection_event);
	void updateConnectionParams();
	void on_disconnect(const Gap::DisconnectionCallbackParams_t *event);
	void blink(void);
	void update_battery_value();
	void update_environmental_data();

	void update_inertial_data();
	void uartDataWrittenCallback(const GattWriteCallbackParams * params);
	void inertial_data_write(uint8_t data[20], int size);

private:
	BLE 					&_ble;
	events::EventQueue 		&_event_queue;
	PwmOut 					_led1;
	int						_blink_id;

	Gap::ConnectionParams_t _gap_params;
	Gap::Handle_t 			_gap_h;

	MAX17201 				&_gauge;
	uint8_t 				_battery_level;
	BatteryService* 		_battery_servicePtr;

	//Environnement
	BME280 					&_bme;
	float 					_temperature;
	float 					_pressure;
	float 					_humidity;
	EnvironmentalService* 	_env_servicePtr;

	//Capteur inertiel
	BNO055 					_bno;
	bno055_raw_quaternion_t _quat;
	bno055_accelerometer_t 	_accel;
	bno055_gyroscope_t 		_gyro;
	bno055_magnetometer_t 	_mag;
	bno055_euler_t 			_euler;
	uint8_t 				_inertial_data[20];
	uint8_t 				_stream_config;
	UARTService*			_uart_servicePtr;
	int						_inertial_id;

};



#endif /* ZESTMOTION_H_ */
