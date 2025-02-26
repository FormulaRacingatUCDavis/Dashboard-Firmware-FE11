/*
 * profiler.h
 *
 *  Created on: Jan 14, 2025
 *      Author: IanKM
 */

#ifndef INC_PROFILER_H_
#define INC_PROFILER_H_

#include <stdint.h>
#include "cmsis_os.h"

// Switch this to 1 to enable the profiler
#define FRUCD_PROFILER_ENABLED 1

typedef struct {
	// id: u32
	uint32_t magic;
	uint32_t thread_id;
	uint64_t start;
	uint64_t end;
	char scope_name[32];
} profiler_data_t;

extern volatile uint64_t profiler_perf_timer;

void profiler_record_marker(profiler_data_t* marker);

#if FRUCD_PROFILER_ENABLED
	#define PROFILER_SCOPE_AUTO(x) \
		profiler_data_t marker##__LINE__ __attribute__ ((__cleanup__(profiler_record_marker))); \
		marker##__LINE__.magic = 0xFA57FA57; \
		strncpy(marker##__LINE__.scope_name, x, 32); \
		marker##__LINE__.start = profiler_perf_timer; \
		marker##__LINE__.thread_id = (uint32_t)osThreadGetId();



	#define PROFILER_FUNC_AUTO() PROFILER_SCOPE_AUTO(__func__)

#else
	#define PROFILER_SCOPE_AUTO(x)
	#define PROFILER_FUNC_AUTO()
#endif

#endif /* INC_PROFILER_H_ */
