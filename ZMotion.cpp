/*
 * Z_Motion demo
 * Copyright (c) 2019, Sebastien Prouff
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

#include "ZMotion.h"

using namespace sixtron;
static SWO swo;

/**\name list of Gattservice provided by the ZestMotion                      */
static const uint16_t uuid16_list[] = { GattService::UUID_BATTERY_SERVICE,
        GattService::UUID_ENVIRONMENTAL_SERVICE };

/*
 * Z_Motion Class constructor
 *
 */
ZMotion::ZMotion(BLE &ble, events::EventQueue &event_queue, MAX17201 *gauge, BME280 *imu_environnment, BNO055 *imu_inertial) :
                _ble(ble),
                _event_queue(event_queue),
                _led1(LED1),
                _gap_params(),
                //_i2c(I2C_SDA, I2C_SCL),
                _battery_service(nullptr),
                _env_service(nullptr),
                _uart_service(nullptr),
                _battery_level(50),
                _temperature(0),
                _pressure(0),
                _humidity(0),
                _stream_config(0),
                _inertial_id(0),
                _blink_id(0)
    {
    _gauge = gauge ;
    _imu_environnment = imu_environnment;
    _imu_inertial = imu_inertial;
}

/*
 * Default destructor
 */
ZMotion::~ZMotion() {
    delete _battery_service;
    delete _env_service;
    delete _uart_service;
}

void ZMotion::start() {
    ble_error_t error;
    swo.printf("Start\n");
    _ble.gap().setEventHandler(this);

    if (_ble.hasInitialized()) {
        swo.printf("Ble instance already initialised.\r\n");
        return;
    }
    error = _ble.init(this, &ZMotion::on_init_complete);
    if (error) {
        swo.printf("Error returned by BLE::init.\r\n");
        return;
    }
    /* to show we're running we'll blink every 500ms */
    _blink_id = _event_queue.call_every(500, this, &ZMotion::blink);
    _event_queue.call_every(1000, this, &ZMotion::update_battery_value);
    _event_queue.call_every(5000, this, &ZMotion::update_environmental_data);
    /* this will not return until shutdown */
    _event_queue.dispatch_forever();
}

void ZMotion::on_init_complete(BLE::InitializationCompleteCallbackContext *event) {
    swo.printf("On_init_complete\n");
    if (event->error) {
        swo.printf("Error during the initialization\r\n");
        return;
    }
    event->ble.gap().setDeviceName((uint8_t*) DEVICE_NAME);
    _gap_params.connectionSupervisionTimeout = 500;
    _gap_params.minConnectionInterval = 16;
    _gap_params.maxConnectionInterval = 16;
    _gap_params.slaveLatency = 0;

    event->ble.gap().setPreferredConnectionParams(&_gap_params);
    event->ble.gap().onConnection(this, &ZMotion::on_connect);
    event->ble.gap().onDisconnection(this, &ZMotion::on_disconnect);

    _battery_service = new BatteryService(event->ble, _battery_level);
    _env_service = new EnvironmentalService(event->ble);
    _uart_service = new UARTService(event->ble);

    _ble.gattServer().onDataWritten(this, &ZMotion::uartDataWrittenCallback);
    start_advertising();
}

void ZMotion::start_advertising() {
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

void ZMotion::on_connect(const Gap::ConnectionCallbackParams_t *connection_event) {
    _gap_h = connection_event->handle;
    swo.printf("Connected in \r\n");
    _event_queue.cancel(_blink_id);
    _led1 = 1;

    _imu_inertial->set_power_mode(BNO055::PowerMode::NORMAL);
    wait_ms(800);
    _imu_inertial->set_operation_mode(BNO055::OperationMode::NDOF);
    _inertial_id = _event_queue.call_every(20, this,&ZMotion::update_inertial_data);
    updateConnectionParams();
}

void ZMotion::updateConnectionParams() {
    if (_ble.gap().getState().connected) {
        _gap_params.connectionSupervisionTimeout = 500;
        _gap_params.minConnectionInterval = 16;
        _gap_params.maxConnectionInterval = 16;
        _gap_params.slaveLatency = 0;

        swo.printf("Request a connection params update!\n");
        if (_ble.gap().updateConnectionParams(_gap_h, &_gap_params)
                != BLE_ERROR_NONE) {
            swo.printf("Error sending the request!\n");
        }
    }
}

void ZMotion::on_disconnect(const Gap::DisconnectionCallbackParams_t *event) {
    swo.printf("Disconnected in \r\n");
    _event_queue.cancel(_inertial_id);
    _imu_inertial->set_operation_mode(BNO055::OperationMode::CONFIG);
    _imu_inertial->set_power_mode(BNO055::PowerMode::SUSPEND);
    _blink_id = _event_queue.call_every(200, this, &ZMotion::blink);
    _ble.gap().startAdvertising();
}
;

void ZMotion::blink(void) {
    _led1 = !_led1;
}

void ZMotion::update_battery_value() {
    _battery_level = static_cast<uint8_t>(_gauge->state_of_charge());
    if (_ble.gap().getState().connected) {
        _battery_service->updateBatteryLevel(_battery_level);
    }
}

void ZMotion::update_environmental_data() {
    _temperature = _imu_environnment->temperature();
    _pressure = _imu_environnment->pressure();
    _humidity = _imu_environnment->humidity();
    if (!isnan(_temperature)) {
        _env_service->updateTemperature(_temperature);
    }
    if (!isnan(_humidity)) {
        _env_service->updateHumidity(_humidity);
    }
    if (!isnan(_pressure)) {
        _env_service->updatePressure(_pressure);
    }

}

void ZMotion::uartDataWrittenCallback(const GattWriteCallbackParams * params) {
    if (params->handle == _uart_service->getTXCharacteristicHandle()) {
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

void ZMotion::update_inertial_data() {
    /* Inertial data are sent through the UART service
     * By writing on the RX characteristic, the master (Smartphone application for example)
     * can choose which data are sent through the BLE notifications (0x02 for the 6TRON application)
     */
    switch (_stream_config) {
    case 0x00: // Sensors raw data
        _acceleration = _imu_inertial->accelerometer();
        _gyroscope = _imu_inertial->gyroscope();
        _magnetometer = _imu_inertial->magnetometer();

        _inertial_data[0] = _stream_config; // Stream configuration
        _inertial_data[1] = (int16_t(_acceleration.x * 100) & 0xFF);
        _inertial_data[2] = (int16_t(_acceleration.x * 100) >> 8) & 0xFF; 	 // accel
        _inertial_data[3] = (int16_t(_acceleration.y * 100) & 0xFF);
        _inertial_data[4] = (int16_t(_acceleration.y * 100) >> 8) & 0xFF; 	 // accel
        _inertial_data[5] = (int16_t(_acceleration.z * 100) & 0xFF);
        _inertial_data[6] = (int16_t(_acceleration.z * 100) >> 8) & 0xFF; 	 // accel

        _inertial_data[7] = (int16_t(_gyroscope.x * 100) & 0xFF);
        _inertial_data[8] = (int16_t(_gyroscope.x * 100) >> 8) & 0xFF; 	 // _gyro
        _inertial_data[9] = (int16_t(_gyroscope.y * 100) & 0xFF);
        _inertial_data[10] = (int16_t(_gyroscope.y * 100) >> 8) & 0xFF; 	 // _gyro
        _inertial_data[11] = (int16_t(_gyroscope.z * 100) & 0xFF);
        _inertial_data[12] = (int16_t(_gyroscope.z * 100) >> 8) & 0xFF; 	 // _gyro

        _inertial_data[13] = (int16_t(_magnetometer.x * 100) & 0xFF);
        _inertial_data[14] = (int16_t(_magnetometer.x * 100) >> 8) & 0xFF; 		 // _mag
        _inertial_data[15] = (int16_t(_magnetometer.y * 100) & 0xFF);
        _inertial_data[16] = (int16_t(_magnetometer.y * 100) >> 8) & 0xFF; 		 // _mag
        _inertial_data[17] = (int16_t(_magnetometer.z * 100) & 0xFF);
        _inertial_data[18] = (int16_t(_magnetometer.z * 100) >> 8) & 0xFF; 		 // _mag

        _inertial_data[19] = 0x0A; // "\n"
        inertial_data_write(_inertial_data, 20);
        break;

    case 0x01: // Orientation (_euler) + Accelerometer
        _euler_angles = _imu_inertial->euler();
        _acceleration = _imu_inertial->accelerometer();

        _inertial_data[0] = _stream_config; // Stream configuration
        _inertial_data[1] = (int16_t(_acceleration.x * 100) & 0xFF);
        _inertial_data[2] = (int16_t(_acceleration.x * 100) >> 8) & 0xFF; 	 // accel
        _inertial_data[3] = (int16_t(_acceleration.y * 100) & 0xFF);
        _inertial_data[4] = (int16_t(_acceleration.y * 100) >> 8) & 0xFF; 	 // accel
        _inertial_data[5] = (int16_t(_acceleration.z * 100) & 0xFF);
        _inertial_data[6] = (int16_t(_acceleration.z * 100) >> 8) & 0xFF; 	 // accel

        _inertial_data[7] = (int16_t(_euler_angles.x * 180 / 3.14 * 100) & 0xFF);
        _inertial_data[8] = (int16_t(_euler_angles.x * 180 / 3.14 * 1000) >> 8) & 0xFF; // _euler
        _inertial_data[9] = (int16_t(_euler_angles.y * 180 / 3.14 * 100) & 0xFF);
        _inertial_data[10] = (int16_t(_euler_angles.y * 180 / 3.14 * 100) >> 8) & 0xFF; // _euler
        _inertial_data[11] = (int16_t(_euler_angles.z * 180 / 3.14 * 100) & 0xFF);
        _inertial_data[12] = (int16_t(_euler_angles.z * 180 / 3.14 * 100) >> 8) & 0xFF; // _euler

        _inertial_data[13] = 0x0A; // "\n"
        inertial_data_write(_inertial_data, 14);
        break;

    case 0x02:
        _quaternion = _imu_inertial->raw_quaternion();
        _acceleration = _imu_inertial->accelerometer();

        _inertial_data[0] = _stream_config; // Stream configuration
        _inertial_data[1] = (int16_t(_acceleration.x * 100) & 0xFF);
        _inertial_data[2] = (int16_t(_acceleration.x * 100) >> 8) & 0xFF; 	 // _accel
        _inertial_data[3] = (int16_t(_acceleration.y * 100) & 0xFF);
        _inertial_data[4] = (int16_t(_acceleration.y * 100) >> 8) & 0xFF; 	 // _accel
        _inertial_data[5] = (int16_t(_acceleration.z * 100) & 0xFF);
        _inertial_data[6] = (int16_t(_acceleration.z * 100) >> 8) & 0xFF; 	 // accel

        _inertial_data[7] = (int16_t(_quaternion.w) & 0xFF);
        _inertial_data[8] = (int16_t(_quaternion.w) >> 8) & 0xFF; 	 // _quat
        _inertial_data[9] = (int16_t(_quaternion.x) & 0xFF);
        _inertial_data[10] = (int16_t(_quaternion.x) >> 8) & 0xFF; 	 // _quat
        _inertial_data[11] = (int16_t(_quaternion.y) & 0xFF);
        _inertial_data[12] = (int16_t(_quaternion.y) >> 8) & 0xFF; 	 // _quat
        _inertial_data[13] = (int16_t(_quaternion.z) & 0xFF);
        _inertial_data[14] = (int16_t(_quaternion.z) >> 8) & 0xFF; 	 // _quat

        _inertial_data[15] = 0x0A; // "\n"
        inertial_data_write(_inertial_data, 16);
        break;

    default:
        _acceleration = _imu_inertial->accelerometer();
        _gyroscope = _imu_inertial->gyroscope();
        _magnetometer = _imu_inertial->magnetometer();

        _inertial_data[0] = _stream_config; // Stream configuration
        _inertial_data[1] = (int16_t(_acceleration.x * 100) & 0xFF);
        _inertial_data[2] = (int16_t(_acceleration.x * 100) >> 8) & 0xFF; 	 // accel
        _inertial_data[3] = (int16_t(_acceleration.y * 100) & 0xFF);
        _inertial_data[4] = (int16_t(_acceleration.y * 100) >> 8) & 0xFF; 	 // accel
        _inertial_data[5] = (int16_t(_acceleration.z * 100) & 0xFF);
        _inertial_data[6] = (int16_t(_acceleration.z * 100) >> 8) & 0xFF; 	 // accel

        _inertial_data[7] = (int16_t(_gyroscope.x * 100) & 0xFF);
        _inertial_data[8] = (int16_t(_gyroscope.x * 100) >> 8) & 0xFF; 	 // _gyro
        _inertial_data[9] = (int16_t(_gyroscope.y * 100) & 0xFF);
        _inertial_data[10] = (int16_t(_gyroscope.y * 100) >> 8) & 0xFF; 	 // _gyro
        _inertial_data[11] = (int16_t(_gyroscope.z * 100) & 0xFF);
        _inertial_data[12] = (int16_t(_gyroscope.z * 100) >> 8) & 0xFF; 	 // _gyro

        _inertial_data[13] = (int16_t(_magnetometer.x * 100) & 0xFF);
        _inertial_data[14] = (int16_t(_magnetometer.x * 100) >> 8) & 0xFF; 		 // _mag
        _inertial_data[15] = (int16_t(_magnetometer.y * 100) & 0xFF);
        _inertial_data[16] = (int16_t(_magnetometer.y * 100) >> 8) & 0xFF; 		 // _mag
        _inertial_data[17] = (int16_t(_magnetometer.z * 100) & 0xFF);
        _inertial_data[18] = (int16_t(_magnetometer.z * 100) >> 8) & 0xFF; 		 // _mag

        _inertial_data[19] = 0x0A; // "\n"
        inertial_data_write(_inertial_data, 20);
        break;
    }
}

void ZMotion::inertial_data_write(uint8_t data[20], int size) {
    if (_uart_service)
        _uart_service->write(data, size);

}
