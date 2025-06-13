/*
 * sensors.c
 *
 *  Created on: Feb 20, 2024
 *      Author: cogus
 */
#include "sensors.h"
#include "can_manager.h"
#include "frucd_display.h"
#include "driver_input.h"
#include <stdlib.h>
#include "traction_control.h"

CALIBRATED_SENSOR_t throttle1;
CALIBRATED_SENSOR_t throttle2;
CALIBRATED_SENSOR_t brake;
uint32_t torque_percentage = 0;
uint32_t launch_control_param = 0;
uint32_t torque_req = 0;

#define RADS_PER_RPM 0.10472
#define MAX_TORQUE_OVERTAKE (uint16_t)(MAX_TORQUE_NM * 0.8)

extern volatile uint8_t traction_control_enabled;
extern volatile int16_t motor_speed;
extern volatile uint16_t acc_current_adc;
extern volatile uint16_t acc_current_ref_adc;
extern volatile uint16_t pack_voltage;

uint16_t get_max_torque(uint32_t max_power);
uint32_t get_max_power();
//extern void Error_Handler();


void init_sensors(){
    throttle1.min = 0x7FFF;
    throttle1.max = 0;
    throttle1.range = 1;
    throttle2.min = 0x7FFF;
    throttle2.max = 0;
    throttle2.range = 1;
    brake.min = 0x7FFF;
    brake.max = 0;
    brake.range = 1;
}

void select_adc_channel(ADC_HandleTypeDef *hadc, ADC_CHAN channel)
{
    ADC_ChannelConfTypeDef sConfig = {0};
    sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
    switch (channel)
    {
        case APPS1:
            sConfig.Channel = ADC_CHANNEL_10;
			sConfig.Rank = 1;

			if (HAL_ADC_ConfigChannel(hadc, &sConfig) != HAL_OK)
			{
//				Error_Handler();
			}
			break;

        case APPS2:
			sConfig.Channel = ADC_CHANNEL_8;
			sConfig.Rank = 1;
			if (HAL_ADC_ConfigChannel(hadc, &sConfig) != HAL_OK)
			{
//				Error_Handler();
			}
			break;
        case BSE:
			sConfig.Channel = ADC_CHANNEL_15;
			sConfig.Rank = 1;
			if (HAL_ADC_ConfigChannel(hadc, &sConfig) != HAL_OK)
			{
//				Error_Handler();
			}
			break;
        case KNOB1:
			sConfig.Channel = ADC_CHANNEL_13;
			sConfig.Rank = 1;
			if (HAL_ADC_ConfigChannel(hadc, &sConfig) != HAL_OK)
			{
//				Error_Handler();
			}
			break;
        case KNOB2:
			sConfig.Channel = ADC_CHANNEL_12;
			sConfig.Rank = 1;
			if (HAL_ADC_ConfigChannel(hadc, &sConfig) != HAL_OK)
			{
//				Error_Handler();
			}
			break;
        case STRAIN_GAUGE:
			sConfig.Channel = ADC_CHANNEL_11;
			sConfig.Rank = 1;
			if (HAL_ADC_ConfigChannel(hadc, &sConfig) != HAL_OK)
			{
//				Error_Handler();
			}
			break;
        default:
            break;
    }
}

uint32_t get_adc_conversion(ADC_HandleTypeDef *hadc, ADC_CHAN channel) {

	select_adc_channel(hadc, channel);

	uint32_t conversion;

	HAL_ADC_Start(hadc);

	// Wait for the conversion to complete
	HAL_ADC_PollForConversion(hadc, HAL_MAX_DELAY);

	// Get the ADC value
	conversion = HAL_ADC_GetValue(hadc);

	return conversion;
}


// Update sensors

void run_calibration() {
    update_minmax(&throttle1);
    update_minmax(&throttle2);
    update_minmax(&brake);
}

void update_sensor_vals(ADC_HandleTypeDef *hadc1, ADC_HandleTypeDef *hadc3) {
	// pedals
    throttle1.raw = get_adc_conversion(hadc1, APPS1);
    update_percent(&throttle1);
    throttle2.raw = get_adc_conversion(hadc3, APPS2);
    update_percent(&throttle2);
    brake.raw = get_adc_conversion(hadc3, BSE);
    update_percent(&brake);

    // knobs
    torque_percentage = get_adc_conversion(hadc1, KNOB2) * 100 / 4095;
    launch_control_param = get_adc_conversion(hadc1, KNOB1) * 100 / 4095;
}

static float raw_to_mvolts(uint16_t adc_raw) {
    return ((float)adc_raw / 4095) * 3.3 * 1000;
}

static float mvolts_to_amps(float mVolts, float mVolt_ref) {
    return ((mVolts - mVolt_ref) * 7.4 / 4.7) / 6.667;
}

static bool accumulator_power_draw_exceeded() {
	float acc_current_amps = mvolts_to_amps(raw_to_mvolts(acc_current_adc), raw_to_mvolts(acc_current_ref_adc));
	float acc_voltage_volt = pack_voltage * 0.018 + 180;

	return acc_current_amps*acc_voltage_volt >= MAX_POWER_ACCUMULATOR_W;
}

uint16_t requested_throttle(){
    uint32_t max_power = get_max_power();
    uint16_t max_torque = get_max_torque(max_power);

    // zero throttle if brake is pressed at all, prevents hardware bspd
//	if (brake.percent >= BRAKE_BSPD_THRESHOLD) return 0;


    // MAKE EXTRA SURE 80kW accumulator power draw is not exceeded or FUSE WILL BLOW
	if (accumulator_power_draw_exceeded()) {// if exceed power limit of 80kW, cut torque
	   return 0;
	}

    torque_req = (throttle1.percent * max_torque * 10) / 100;  //upscale for MC code, Nm times 10

    // use reduced values from TC if TC torque request is lower
    if(is_button_enabled(TC_BUTTON) && (torque_req > TC_torque_req)){
		torque_req = TC_torque_req;
	}

    return (uint16_t)torque_req;
}

// get maximum power based on power limit
// attenuate for BMS temps between 50 and 60
uint32_t get_max_power(){
	if(PACK_TEMP < 50) {
		return MAX_POWER_MOTOR_W;
	} else if(PACK_TEMP < 58) {
		return (58 - PACK_TEMP)*(MAX_POWER_MOTOR_W / 8);
	} else {
		return 0;
	}
}

uint16_t get_max_torque(uint32_t max_power){
	float motor_speed_rads = (float)motor_speed * RADS_PER_RPM;
	float max_torque_power = max_power / motor_speed_rads; // max torque calculated from max power
	float max_torque_knob = MAX_TORQUE_NM * (float)torque_percentage / 100;

	// if overtake is enabled, return the lower of torque limit set by overtake value and power limit
	if (is_button_enabled(OVERTAKE_BUTTON) && MAX_TORQUE_OVERTAKE < max_torque_power) {
		return MAX_TORQUE_OVERTAKE;
	}
	// else return the lower of the torque limit set by the knob and by the power limit
	else {
		if (max_torque_knob < max_torque_power) {
			return (uint16_t)max_torque_knob;
		}
		else {
			return (uint16_t)max_torque_power;
		}
	}
}

bool sensors_calibrated(){
	return throttle1.range > APPS1_MIN_RANGE &&
		   throttle2.range > APPS2_MIN_RANGE &&
		   brake.range > BRAKE_MIN_RANGE;
}

bool braking(){
    return brake.raw > BRAKE_LIGHT_THRESHOLD;
}

bool brake_mashed(){
    return brake.percent > RTD_BRAKE_THRESHOLD;
}

// check differential between the throttle sensors
// returns true only if the sensor discrepancy is > 10%
// Note: after verifying there's no discrepancy, can use either sensor(1 or 2) for remaining checks
bool has_discrepancy() {
    if(abs((int)throttle1.percent - (int)throttle2.percent) > 10) return 1;  //percentage discrepancy

    return (throttle1.raw < APPS_OPEN_THRESH)
        || (throttle1.raw > APPS_SHORT_THRESH)
        || (throttle2.raw < APPS_OPEN_THRESH)
        || (throttle2.raw > APPS_SHORT_THRESH);   // checks for wiring fault (open or short circuit).
    											  // not sure why this is in the discrepancy check function but it's needed
	return false;

}

// check for soft BSPD (in rules, called "APPS / Brake Pedal Plausibility Check")
// see EV.4.7 of FSAE 2025 rulebook
bool is_brake_implausible() {
    if (error == BRAKE_IMPLAUSIBLE) {
        // once brake implausibility detected,
        // can only revert to normal if throttle "unapplied"
        return !(throttle1.percent <= APPS1_BSPD_RESET_THRESHOLD);
    }

    // if brake applied and throttle > 25%, brake implausible
    return (brake.percent >= BRAKE_BSPD_THRESHOLD && throttle1.percent > APPS1_BSPD_THRESHOLD);
}

void update_percent(CALIBRATED_SENSOR_t* sensor){
    uint32_t raw = (uint32_t)clamp(sensor->raw, sensor->min, sensor->max);
    sensor->percent = (uint16_t)((100*(raw-sensor->min))/((sensor->range)));
}

void update_minmax(CALIBRATED_SENSOR_t* sensor){
    if (sensor->raw > sensor->max) sensor->max = sensor->raw;
    else if (sensor->raw < sensor->min) sensor->min = sensor->raw;
    if(sensor->max > sensor->min) sensor->range = sensor->max - sensor->min;
}

void add_apps_deadzone(){
	add_deadzone(&throttle1, 5);
	add_deadzone(&throttle2, 5);
	add_deadzone(&brake, 10);
}

void add_deadzone(CALIBRATED_SENSOR_t* sensor, uint16_t deadzone_percentage){
	uint16_t deadzone = sensor->range * deadzone_percentage / 100;

	// catch funky cases that would end up with a negative or 0 range
	if(deadzone >= sensor->range) return;

	sensor->min += deadzone;
	sensor->range -= deadzone;
}

uint16_t clamp(uint16_t in, uint16_t min, uint16_t max){
    if(in > max) return max;
    if(in < min) return min;
    return in;
}


