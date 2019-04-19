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
 * main2.cpp
 *
 *  Created on: 14 avr. 2019
 *      Author: sepro
 *      MBED 5.10.4
 */

#include "ZestMotion.h"



using namespace sixtron;
static SWO swo;

static events::EventQueue event_queue(/* event count */ 16 * EVENTS_EVENT_SIZE);


/** Schedule processing of events from the BLE middleware in the event queue. */
void schedule_ble_events(BLE::OnEventsToProcessCallbackContext *context) {
	event_queue.call(Callback<void()>(&context->ble, &BLE::processEvents));
}

I2C i2c(I2C_SDA, I2C_SCL);

//Gauge de batterie
MAX17201 	gauge(&i2c);
static uint8_t battery_level = 50;

//Capteur environnemental
BME280 bme(&i2c);

//Acceléromètre
BNO055 bno(&i2c);



/*
 * Initialisation de la batterie
 */
void battery_init() {
	if (gauge.design_capacity() == 750) {
		if (gauge.configure(1, 160, 3.0, false, false)) {
			battery_level = uint8_t(gauge.state_of_charge());
			swo.printf("Gauge initialized ! \n");
		} else {
			swo.printf("Fail to initialized MAX17201 gauge ! \n");
		}
	} else {
		swo.printf("Gauge already configured\n");
		battery_level = uint8_t(gauge.state_of_charge());
	}
}

/*
 * Initialisation du capteur d'environnement
 *
 */
void environnement_init()	{

	if (!bme.initialize()) {
		swo.printf("Couldn't initialize the BME280...\n");
		return ;
	}

	bme.set_sampling(BME280::SensorMode::NORMAL,
			BME280::SensorSampling::OVERSAMPLING_X1,
			BME280::SensorSampling::OVERSAMPLING_X1,
			BME280::SensorSampling::OVERSAMPLING_X1, BME280::SensorFilter::OFF,
			BME280::StandbyDuration::MS_1000);
}

/*
 * Initialisation du capteur interniel
 */
void inertial_init()	{
	if (bno.initialize(BNO055::OperationMode::CONFIG, true) != true) {
		swo.printf("ERROR BNO055 not detected. Check your wiring and BNO I2C address\n");
		return;
	}
	bno.set_power_mode(BNO055::PowerMode::SUSPEND);
}


int main()
{
	swo.printf("LET's DO IT !\n");
	BLE &ble = BLE::Instance();
	ble.onEventsToProcess(schedule_ble_events);

	//initialisation des capteurs
	battery_init();
	environnement_init();
	inertial_init();

	ZestMotion myMotion(ble, event_queue, gauge, bme, bno);
	myMotion.start();

	return 0;
}
