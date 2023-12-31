/*
 * sensors.h
 *
 *  Created on: Dec 2, 2023
 *      Author: sruti
 */

#ifndef INC_SENSORS_H_
#define INC_SENSORS_H_

#ifndef _XTAL_FREQ
#define _XTAL_FREQ  8000000UL
#endif

#ifndef FCY
#define FCY 8000000UL


#endif /* INC_SENSORS_H_ */


// This is a guard condition so that contents of this file are not included
// more than once.


#include "stdint.h"
#include "stdbool.h"

#include "config.h"
#include "fsm.h"

// On the breadboard, the range of values for the potentiometer is 0 to 4095
#define PEDAL_MAX 4095

// There is some noise when reading from the brake pedal
// So give some room for error when driver presses on brake
#define BRAKE_ERROR_TOLERANCE 50

typedef struct{
    uint16_t raw;
    uint16_t min;
    uint16_t max;
    uint16_t range;
    uint16_t percent;
} CALIBRATED_SENSOR_t;

//function prototypes
uint16_t getConversion(ADC1_CHANNEL channel);
void run_calibration();
void update_sensor_vals();

bool sensors_calibrated();
bool has_discrepancy();
bool brake_implausible();
bool braking();
bool brake_mashed();

void temp_attenuate();
uint16_t requested_throttle();

uint16_t clamp(uint16_t in, uint16_t min, uint16_t max);
void update_percent(CALIBRATED_SENSOR_t* sensor);
void update_minmax(CALIBRATED_SENSOR_t* sensor);
void init_sensors();




#endif	/* XC_HEADER_TEMPLATE_H */
