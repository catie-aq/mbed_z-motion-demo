/*
 * Z_motion demo
 * Copyright (c) 2019, Sebastien Prouff
 * Copyright (c) 2022, CATIE
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

#include "z_motion_app.h"

using namespace sixtron;
static SWO swo;

static events::EventQueue event_queue(16 * EVENTS_EVENT_SIZE);
Thread z_motion_thread(osPriorityNormal, OS_STACK_SIZE, NULL, "z_motion_thread");

/*! Schedule processing of events from the BLE middleware in the event queue. */
void schedule_ble_events(BLE::OnEventsToProcessCallbackContext *context)
{
    event_queue.call(Callback<void()>(&context->ble, &BLE::processEvents));
}

I2C i2c(I2C_SDA, I2C_SCL);

/*!Battery gauge */
MAX17201 gauge(&i2c);

/*! Environmental sensor */
BME280 environnemental_sensor(&i2c);

/*! Inertial Sensor */
BNO055 imu(&i2c);

/*!
 * Battery initialization
 */
int battery_init()
{
    if (gauge.design_capacity() == 750) {
        if (gauge.configure(1, 160, 3.0, false, false)) {
            swo.printf("Gauge initialized ! \n");
            return 0;
        } else {
            swo.printf("Fail to initialized MAX17201 gauge ! \n");
            return -1;
        }
    } else {
        swo.printf("Gauge already configured\n");
        return 0;
    }
}

/*
 * Environment sensor initialization
 *
 */
int environnement_init()
{
    if (!environnemental_sensor.initialize()) {
        swo.printf("Couldn't initialize the BME280...\n");
        return -1;
    } else {
        environnemental_sensor.set_sampling(BME280::SensorMode::NORMAL,
                BME280::SensorSampling::OVERSAMPLING_X1,
                BME280::SensorSampling::OVERSAMPLING_X1,
                BME280::SensorSampling::OVERSAMPLING_X1,
                BME280::SensorFilter::OFF,
                BME280::StandbyDuration::MS_1000);
        swo.printf("BME280 initialized ...\n");
        return 0;
    }
}

/*
 * Inertial sensor initialization
 */
int inertial_init()
{
    if (imu.initialize(BNO055::OperationMode::CONFIG, true) != true) {
        swo.printf("ERROR BNO055 not detected. Check your wiring and BNO I2C address\n");
        return -1;
    } else {
        imu.set_power_mode(BNO055::PowerMode::SUSPEND);
        swo.printf("BNO055 initialized ...\n");
        return 0;
    }
}

int main()
{
    swo.printf("Welcome in the Z_Motion demo !\n");

    /*! Get of instance of the BLE */
    BLE &ble = BLE::Instance();
    ZMotion sixTronMotion(ble, event_queue, &gauge, &environnemental_sensor, &imu);

    ble.onEventsToProcess(schedule_ble_events);

    // Sensors initialization
    if (battery_init() != 0) {
        return -1;
    }
    if (environnement_init() != 0) {
        return -1;
    }
    if (inertial_init() != 0) {
        return -1;
    }

    z_motion_thread.start(callback(&sixTronMotion, &ZMotion::start));

    while (true) {
        ThisThread::sleep_for(1s);
    }

    return 0;
}
