/*
 * Copyright (c) 2019, Sebastien Prouff
 * Copyright (c) 2022, CATIE
 * SPDX-License-Identifier: Apache-2.0
 */

#include "z_motion_app.h"

using namespace sixtron;
static SWO swo;

uint8_t _adv_buffer[ble::LEGACY_ADVERTISING_MAX_SIZE];

const uint8_t SixtronServiceUUID[UUID::LENGTH_OF_LONG_UUID] = {
    0x6E,
    0x40,
    0x00,
    0x01,
    0xB5,
    0xA3,
    0xF3,
    0x93,
    0xE0,
    0xA9,
    0xE5,
    0x0E,
    0x24,
    0xDC,
    0xCA,
    0x9E,
};

const uint8_t SixtronRXCharacteristicUUID[UUID::LENGTH_OF_LONG_UUID] = {
    0x6E,
    0x40,
    0x00,
    0x02,
    0xB5,
    0xA3,
    0xF3,
    0x93,
    0xE0,
    0xA9,
    0xE5,
    0x0E,
    0x24,
    0xDC,
    0xCA,
    0x9E,
};

const uint8_t SixtronTXCharacteristicUUID[UUID::LENGTH_OF_LONG_UUID] = {
    0x6E,
    0x40,
    0x00,
    0x03,
    0xB5,
    0xA3,
    0xF3,
    0x93,
    0xE0,
    0xA9,
    0xE5,
    0x0E,
    0x24,
    0xDC,
    0xCA,
    0x9E,
};

/**\name list of Gattservice provided by the ZestMotion                      */
static const uint16_t uuid16_list[]
        = { GattService::UUID_BATTERY_SERVICE, GattService::UUID_ENVIRONMENTAL_SERVICE };

/*
 * Z_Motion Class constructor
 *
 */
ZMotion::ZMotion(BLE &ble,
        events::EventQueue &event_queue,
        MAX17201 *gauge,
        BME280 *imu_environnment,
        BNO055 *imu_inertial):
        _ble(ble),
        _event_queue(event_queue),
        _led1(LED1),
        //_i2c(I2C_SDA, I2C_SCL),
        _battery_service(nullptr),
        _TXCharacteristic(SixtronTXCharacteristicUUID,
                0,
                GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_NOTIFY),
        _RXCharacteristic(SixtronRXCharacteristicUUID, NULL),
        _sixtron_service(nullptr),
        _env_service(nullptr),
        _battery_level(50),
        _temperature(0),
        _pressure(0),
        _humidity(0),
        _stream_config(0),
        _inertial_id(0),
        _blink_id(0)
{
    _gauge = gauge;
    _environnmental_sensor = imu_environnment;
    _imu = imu_inertial;
}

/*
 * Default destructor
 */
ZMotion::~ZMotion()
{
    delete _battery_service;
    delete _env_service;
    delete _sixtron_service;
}

void ZMotion::start()
{
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
    _blink_id = _event_queue.call_every(500ms, this, &ZMotion::blink);
    _event_queue.call_every(20s, this, &ZMotion::update_battery_value);
    _event_queue.call_every(10s, this, &ZMotion::update_environmental_data);
    /* this will not return until shutdown */
    _event_queue.dispatch_forever();
}

void ZMotion::on_init_complete(BLE::InitializationCompleteCallbackContext *event)
{
    swo.printf("On_init_complete\n");

    if (event->error) {
        swo.printf("Error during the initialization\r\n");
        return;
    }

    /* this allows us to receive events like onConnectionComplete() */
    _ble.gap().setEventHandler(this);

    /* Setup GATT services */
    _battery_service = new BatteryService(event->ble, _battery_level);
    _env_service = new EnvironmentalService(event->ble);
    GattCharacteristic *charTable[] = {
        &_TXCharacteristic,
        &_RXCharacteristic,
    };
    _sixtron_service = new GattService(
            SixtronServiceUUID, charTable, sizeof(charTable) / sizeof(charTable[0]));
    _ble.gattServer().addService(*_sixtron_service);
    _ble.gattServer().setEventHandler(this);

    /* Create advertising parameters and payload */
    ble::AdvertisingParameters adv_parameters(ble::advertising_type_t::CONNECTABLE_UNDIRECTED,
            ble::adv_interval_t(ble::millisecond_t(100)));

    ble::AdvertisingDataBuilder _adv_data_builder(_adv_buffer);
    _adv_data_builder.setFlags();
    _adv_data_builder.setName(MBED_CONF_APP_BLE_DEVICE_NAME);
    _adv_data_builder.setConnectionIntervalPreference(
            (ble::conn_interval_t)16, (ble::conn_interval_t)16);
    uint8_t deviceID[5] = { 0x36, 0x54, 0x52, 0x4F, 0x4E }; // 6TRON specifific manufacturer data
    _adv_data_builder.setManufacturerSpecificData(deviceID);

    /* Setup advertising */
    ble_error_t error
            = _ble.gap().setAdvertisingParameters(ble::LEGACY_ADVERTISING_HANDLE, adv_parameters);

    if (error) {
        printf("ble.gap().setAdvertisingParameters() failed\r\n");
        return;
    }

    error = _ble.gap().setAdvertisingPayload(
            ble::LEGACY_ADVERTISING_HANDLE, _adv_data_builder.getAdvertisingData());

    if (error) {
        printf("ble.gap().setAdvertisingPayload() failed\r\n");
        return;
    }

    /* Start advertising */
    error = _ble.gap().startAdvertising(ble::LEGACY_ADVERTISING_HANDLE);

    if (error) {
        printf("ble.gap().startAdvertising() failed\r\n");
        return;
    }
}

void ZMotion::onConnectionComplete(const ble::ConnectionCompleteEvent &event)
{
    swo.printf("Connected in \r\n");
    swo.printf("Connection parameters:\n");
    swo.printf("\tConnection interval: %d ms\n", event.getConnectionInterval().valueInMs());
    swo.printf("\tSlave latency: %d \n", event.getConnectionLatency());
    swo.printf("\tSupervision timeout: %d ms\n", event.getSupervisionTimeout().valueInMs());

    _connection_handler = event.getConnectionHandle();

    _event_queue.cancel(_blink_id);
    _led1 = 1;

    _imu->set_power_mode(BNO055::PowerMode::NORMAL);
    ThisThread::sleep_for(800ms);
    _imu->set_operation_mode(BNO055::OperationMode::NDOF);
    _inertial_id = _event_queue.call_every(20ms, this, &ZMotion::update_inertial_data);

    updateConnectionParams();
}

void ZMotion::updateConnectionParams()
{
    swo.printf("Request a connection params update!\n");

    _ble.gap().updateConnectionParameters(_connection_handler,
            ble::conn_interval_t(6),
            ble::conn_interval_t(10),
            0,
            ble::supervision_timeout_t(500));
}

void ZMotion::onConnectionParametersUpdateComplete(
        const ble::ConnectionParametersUpdateCompleteEvent &event)
{
    if (event.getStatus() == ble_error_t::BLE_ERROR_NONE) {
        swo.printf("Connection parameters updated\n");
        swo.printf("NEW connection parameters:\n");
        swo.printf("\tConnection interval: %d ms\n", event.getConnectionInterval().valueInMs());
        swo.printf("\tSlave latency: %d \n", event.getSlaveLatency());
        swo.printf("\tSupervision timeout: %d ms\n", event.getSupervisionTimeout().valueInMs());
    } else {
        swo.printf("Connection parameters update failed\n");
    }
}

void ZMotion::onDisconnectionComplete(const ble::DisconnectionCompleteEvent &event)
{
    swo.printf("Disconnected in \r\n");
    _event_queue.cancel(_inertial_id);
    _imu->set_operation_mode(BNO055::OperationMode::CONFIG);
    _imu->set_power_mode(BNO055::PowerMode::SUSPEND);
    _blink_id = _event_queue.call_every(200, this, &ZMotion::blink);
    _ble.gap().startAdvertising(ble::LEGACY_ADVERTISING_HANDLE);
};

void ZMotion::blink(void)
{
    _led1 = !_led1;
}

void ZMotion::update_battery_value()
{
    _battery_level = static_cast<uint8_t>(_gauge->state_of_charge());
    _battery_service->updateBatteryLevel(_battery_level);
}

void ZMotion::update_environmental_data()
{
    _temperature = _environnmental_sensor->temperature();
    _pressure = _environnmental_sensor->pressure();
    _humidity = _environnmental_sensor->humidity();
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

void ZMotion::onDataWritten(const GattWriteCallbackParams &params)
{
    if (params.handle == _RXCharacteristic.getValueHandle()) {
        swo.printf("TX message: %.*s\n", params.len, params.data);
        if (params.len == 1) {
            _stream_config = *params.data;
        }
        if (strcmp((const char *)params.data, "DFU") == 0) {
            swo.printf("Entering DFU mode !\n");
            mbed_start_application(STM32L496RG_BOOTLOADER_ADDRESS);
        }
    }
}

void ZMotion::update_inertial_data()
{
    /* Inertial data are sent through the UART service
     * By writing on the RX characteristic, the master (Smartphone application for example)
     * can choose which data are sent through the BLE notifications (0x02 for the 6TRON application)
     */
    switch (_stream_config) {
        case 0x00: // Sensors raw data
            _acceleration = _imu->acceleration();
            _gyroscope = _imu->angular_velocity();
            _magnetometer = _imu->magnetic_field();

            _inertial_data[0] = _stream_config; // Stream configuration
            _inertial_data[1] = (int16_t(_acceleration.x * 100) & 0xFF);
            _inertial_data[2] = (int16_t(_acceleration.x * 100) >> 8) & 0xFF; // accel
            _inertial_data[3] = (int16_t(_acceleration.y * 100) & 0xFF);
            _inertial_data[4] = (int16_t(_acceleration.y * 100) >> 8) & 0xFF; // accel
            _inertial_data[5] = (int16_t(_acceleration.z * 100) & 0xFF);
            _inertial_data[6] = (int16_t(_acceleration.z * 100) >> 8) & 0xFF; // accel

            _inertial_data[7] = (int16_t(_gyroscope.x * 100) & 0xFF);
            _inertial_data[8] = (int16_t(_gyroscope.x * 100) >> 8) & 0xFF; // _gyro
            _inertial_data[9] = (int16_t(_gyroscope.y * 100) & 0xFF);
            _inertial_data[10] = (int16_t(_gyroscope.y * 100) >> 8) & 0xFF; // _gyro
            _inertial_data[11] = (int16_t(_gyroscope.z * 100) & 0xFF);
            _inertial_data[12] = (int16_t(_gyroscope.z * 100) >> 8) & 0xFF; // _gyro

            _inertial_data[13] = (int16_t(_magnetometer.x * 100) & 0xFF);
            _inertial_data[14] = (int16_t(_magnetometer.x * 100) >> 8) & 0xFF; // _mag
            _inertial_data[15] = (int16_t(_magnetometer.y * 100) & 0xFF);
            _inertial_data[16] = (int16_t(_magnetometer.y * 100) >> 8) & 0xFF; // _mag
            _inertial_data[17] = (int16_t(_magnetometer.z * 100) & 0xFF);
            _inertial_data[18] = (int16_t(_magnetometer.z * 100) >> 8) & 0xFF; // _mag

            _inertial_data[19] = 0x0A; // "\n"
            inertial_data_write(_inertial_data, 20);
            break;

        case 0x01: // Orientation (_euler) + Accelerometer
            _euler_angles = _imu->euler();
            _acceleration = _imu->acceleration();

            _inertial_data[0] = _stream_config; // Stream configuration
            _inertial_data[1] = (int16_t(_acceleration.x * 100) & 0xFF);
            _inertial_data[2] = (int16_t(_acceleration.x * 100) >> 8) & 0xFF; // accel
            _inertial_data[3] = (int16_t(_acceleration.y * 100) & 0xFF);
            _inertial_data[4] = (int16_t(_acceleration.y * 100) >> 8) & 0xFF; // accel
            _inertial_data[5] = (int16_t(_acceleration.z * 100) & 0xFF);
            _inertial_data[6] = (int16_t(_acceleration.z * 100) >> 8) & 0xFF; // accel

            _inertial_data[7] = (int16_t(_euler_angles.x * 180 / 3.14 * 100) & 0xFF);
            _inertial_data[8] = (int16_t(_euler_angles.x * 180 / 3.14 * 100) >> 8) & 0xFF; // _euler
            _inertial_data[9] = (int16_t(_euler_angles.y * 180 / 3.14 * 100) & 0xFF);
            _inertial_data[10]
                    = (int16_t(_euler_angles.y * 180 / 3.14 * 100) >> 8) & 0xFF; // _euler
            _inertial_data[11] = (int16_t(_euler_angles.z * 180 / 3.14 * 100) & 0xFF);
            _inertial_data[12]
                    = (int16_t(_euler_angles.z * 180 / 3.14 * 100) >> 8) & 0xFF; // _euler

            _inertial_data[13] = 0x0A; // "\n"
            inertial_data_write(_inertial_data, 14);
            break;

        case 0x02:
            _quaternion = _imu->raw_quaternion();
            _acceleration = _imu->acceleration();

            _inertial_data[0] = _stream_config; // Stream configuration
            _inertial_data[1] = (int16_t(_acceleration.x * 100) & 0xFF);
            _inertial_data[2] = (int16_t(_acceleration.x * 100) >> 8) & 0xFF; // _accel
            _inertial_data[3] = (int16_t(_acceleration.y * 100) & 0xFF);
            _inertial_data[4] = (int16_t(_acceleration.y * 100) >> 8) & 0xFF; // _accel
            _inertial_data[5] = (int16_t(_acceleration.z * 100) & 0xFF);
            _inertial_data[6] = (int16_t(_acceleration.z * 100) >> 8) & 0xFF; // accel

            _inertial_data[7] = (int16_t(_quaternion.w) & 0xFF);
            _inertial_data[8] = (int16_t(_quaternion.w) >> 8) & 0xFF; // _quat
            _inertial_data[9] = (int16_t(_quaternion.x) & 0xFF);
            _inertial_data[10] = (int16_t(_quaternion.x) >> 8) & 0xFF; // _quat
            _inertial_data[11] = (int16_t(_quaternion.y) & 0xFF);
            _inertial_data[12] = (int16_t(_quaternion.y) >> 8) & 0xFF; // _quat
            _inertial_data[13] = (int16_t(_quaternion.z) & 0xFF);
            _inertial_data[14] = (int16_t(_quaternion.z) >> 8) & 0xFF; // _quat

            _inertial_data[15] = 0x0A; // "\n"
            inertial_data_write(_inertial_data, 16);
            break;

        default:
            _acceleration = _imu->acceleration();
            _gyroscope = _imu->angular_velocity();
            _magnetometer = _imu->magnetic_field();

            _inertial_data[0] = _stream_config; // Stream configuration
            _inertial_data[1] = (int16_t(_acceleration.x * 100) & 0xFF);
            _inertial_data[2] = (int16_t(_acceleration.x * 100) >> 8) & 0xFF; // accel
            _inertial_data[3] = (int16_t(_acceleration.y * 100) & 0xFF);
            _inertial_data[4] = (int16_t(_acceleration.y * 100) >> 8) & 0xFF; // accel
            _inertial_data[5] = (int16_t(_acceleration.z * 100) & 0xFF);
            _inertial_data[6] = (int16_t(_acceleration.z * 100) >> 8) & 0xFF; // accel

            _inertial_data[7] = (int16_t(_gyroscope.x * 100) & 0xFF);
            _inertial_data[8] = (int16_t(_gyroscope.x * 100) >> 8) & 0xFF; // _gyro
            _inertial_data[9] = (int16_t(_gyroscope.y * 100) & 0xFF);
            _inertial_data[10] = (int16_t(_gyroscope.y * 100) >> 8) & 0xFF; // _gyro
            _inertial_data[11] = (int16_t(_gyroscope.z * 100) & 0xFF);
            _inertial_data[12] = (int16_t(_gyroscope.z * 100) >> 8) & 0xFF; // _gyro

            _inertial_data[13] = (int16_t(_magnetometer.x * 100) & 0xFF);
            _inertial_data[14] = (int16_t(_magnetometer.x * 100) >> 8) & 0xFF; // _mag
            _inertial_data[15] = (int16_t(_magnetometer.y * 100) & 0xFF);
            _inertial_data[16] = (int16_t(_magnetometer.y * 100) >> 8) & 0xFF; // _mag
            _inertial_data[17] = (int16_t(_magnetometer.z * 100) & 0xFF);
            _inertial_data[18] = (int16_t(_magnetometer.z * 100) >> 8) & 0xFF; // _mag

            _inertial_data[19] = 0x0A; // "\n"
            inertial_data_write(_inertial_data, 20);
            break;
    }
}

void ZMotion::inertial_data_write(uint8_t data[20], int size)
{
    ble_error_t error = _ble.gattServer().write(_TXCharacteristic.getValueHandle(), data, size);

    if (error) {
        swo.printf("Update error !\n");
    }
}
