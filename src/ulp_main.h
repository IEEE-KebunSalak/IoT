/*
    Put your ULP globals here you want visibility
    for your sketch. Add "ulp_" to the beginning
    of the variable name and must be size 'uint32_t'
*/
#include "Arduino.h"

#pragma once

extern uint32_t ulp_changed;
extern uint32_t ulp_debounce_counter;
extern uint32_t ulp_debounce_max_count;
extern uint32_t ulp_edge_count;
extern uint32_t ulp_edge_detected;
extern uint32_t ulp_entry;
extern uint32_t ulp_io_number;
extern uint32_t ulp_next_edge;
extern uint32_t ulp_pulse_cur;
extern uint32_t ulp_pulse_detected;
extern uint32_t ulp_pulse_edge;
extern uint32_t ulp_pulse_lower;
extern uint32_t ulp_pulse_min;
extern uint32_t ulp_pulse_res;
extern uint32_t ulp_pulse_reset;
extern uint32_t ulp_pulse_tick;
extern uint32_t ulp_read_now;
