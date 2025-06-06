#ifndef FSM_H
#define	FSM_H

#include <stdint.h>

/************ States ************/

typedef enum {
    LV,
    PRECHARGING,
    HV_ENABLED,
    DRIVE,
    FAULT
} state_t;

typedef enum {
    NONE,
    DRIVE_REQUEST_FROM_LV,
    PRECHARGE_TIMEOUT,
    BRAKE_NOT_PRESSED,
    HV_DISABLED_WHILE_DRIVING,
    SENSOR_DISCREPANCY,
    BRAKE_IMPLAUSIBLE,
    SHUTDOWN_CIRCUIT_OPEN,
    UNCALIBRATED,
    HARD_BSPD,
    MC_FAULT
} error_t;

// Initial FSM state
extern volatile state_t state;
extern volatile error_t error;

void change_state(const state_t new_state);
void report_fault(error_t _error);
uint8_t is_hv_requested();
uint8_t one_byte_state();

#endif	/* FSM_H */
