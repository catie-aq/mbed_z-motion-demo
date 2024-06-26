/*
 * Copyright (c) 2019, Sebastien Prouff
 * Copyright (c) 2022, CATIE
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZMOTION_H_
#define ZMOTION_H_

#include "ble/BLE.h"
#include "ble/Gap.h"
#include "ble/services/BatteryService.h"
#include "ble/services/EnvironmentalService.h"
#include "events/mbed_events.h"
#include "mbed.h"

#include "bme280.h"
#include "bno055.h"
#include "max17201.h"
#include "swo.h"

namespace {
#define STM32L496RG_BOOTLOADER_ADDRESS 0x1FFF0000
}

using namespace sixtron;

/*!
 *  \class ZMotion
 *  Handled BLE for ZEstMotion
 *
 *  This version works with MBED 5.10.4, the legacy API and bluetooth shield driver
 */

class ZMotion: public ble::Gap::EventHandler, public ble::GattServer::EventHandler {
public:

    /**
     * @brief Constructor for the ZMotion class.
     * 
     * @param ble BLE object used for BLE communication.
     * @param event_queue EventQueue object used for scheduling events.
     * @param gauge Pointer to the MAX17201 object used for battery monitoring.
     * @param imu_environnment Pointer to the BME280 object used for environmental sensing.
     * @param imu_inertial Pointer to the BNO055 object used for inertial sensing.
     */
    ZMotion(BLE &ble,
            events::EventQueue &event_queue,
            MAX17201 *gauge,
            BME280 *imu_environnment,
            BNO055 *imu_inertial);

    /*!
     * Zest Motion destructor
     */
    ~ZMotion();

    /*!
     * Public start_method
     * Start the device. This method complete the device initialization
     */
    void start();

private:
    /*!
     * Initialization Callback executed with ble.init()
     * This callback params the GAP and the callback executed on connection and deconnection events
     * and also start advertising
     */
    void on_init_complete(BLE::InitializationCompleteCallbackContext *event);

    /*!
     * To blink the led. This callback give information on the connexion
     * i.e. stop blinking while connected
     */
    void blink(void);

    /*!
     * Method to update connectionParam while connected
     *
     */
    void updateConnectionParams();

    /*!
     * Update_battery value and send it to the application if connected
     */
    void update_battery_value();

    /*!
     * Update environmental data and send them to the application if connected
     */
    void update_environmental_data();

    /*!
     * Update inertial data
     */
    void update_inertial_data();

    /*!
     * BLE characteristic data written CB
     */
    virtual void onDataWritten(const GattWriteCallbackParams &params);

    /*!
     * send the inertial data to the application if connected
     */
    void inertial_data_write(uint8_t data[20], int size);

    /*!
     * Event callback executed on the Event connection detected
     */
    virtual void onConnectionComplete(const ble::ConnectionCompleteEvent &event);

    /*!
     * Event callback executed on the Event Disconnection detected
     */
    virtual void onDisconnectionComplete(const ble::DisconnectionCompleteEvent &event);

    /*!
     * Event callback executed on the connection parameters has been updated
     */
    virtual void onConnectionParametersUpdateComplete(
            const ble::ConnectionParametersUpdateCompleteEvent &event);

private:
    BLE &_ble;
    events::EventQueue &_event_queue;
    PwmOut _led1;
    /*! Handle on the Gap connection */
    ble::connection_handle_t _connection_handler;
    /*! Sensors reference*/
    MAX17201 *_gauge;
    BME280 *_environnmental_sensor;
    BNO055 *_imu;
    /*! BLE Gatt services */
    BatteryService *_battery_service;
    ReadOnlyGattCharacteristic<uint8_t[20]> _TXCharacteristic;
    WriteOnlyGattCharacteristic<uint8_t> _RXCharacteristic;
    GattService *_sixtron_service;
    EnvironmentalService *_env_service;
    /*! Battery gauge sensor */
    uint8_t _battery_level;
    /*! Environmental sensor */
    float _temperature;
    float _pressure;
    float _humidity;
    /*! Inertial sensor*/
    bno055_raw_quaternion_t _quaternion;
    bno055_acceleration_t _acceleration;
    bno055_angular_velocity_t _gyroscope;
    bno055_magnetic_field_t _magnetometer;
    bno055_euler_t _euler_angles;
    uint8_t _inertial_data[20];
    uint8_t _stream_config;
    /*! Event_queue id  */
    int _inertial_id;
    int _blink_id;
};

#endif /* ZMOTION_H_ */
