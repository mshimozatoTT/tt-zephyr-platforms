/*
 * Copyright (c) 2026 Tenstorrent AI ULC
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef CAPTURE_BUFFER_H
#define CAPTURE_BUFFER_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/** One stored AICLK transition: firmware sequence tick + packed (mhz, arbiter).
 *
 *  The 16-bit @c mhz field is packed:
 *    bits  0..11 : applied AICLK in MHz   (0..4095)
 *    bits 12..15 : dominant @c aiclk_arb_max throttler ID (0..15) that was
 *                  setting the AICLK ceiling at the moment of the transition
 *
 *  Versioned by @c CLOCK_PATTERN_INFO_MAGIC in the @c GET_CLOCK_PATTERN_INFO
 *  response (v2 = 0x02636c70). v1 (0x01636c70) stored only raw mhz with the
 *  upper bits unused.
 */
struct clock_pattern_event {
	uint32_t seq;
	uint16_t mhz;
} __packed __aligned(2);

/**
 * Leftover CSM between _end and sram_end (csm_app). Not a BSS array: size is
 * whatever the live firmware did not consume. Host must use GET_*_INFO.
 */
bool capture_buffer_ready(void);
size_t capture_power_bytes(void);
size_t capture_clock_bytes(void);
uint32_t capture_power_samples(void);
uint32_t capture_clock_rows(void);

uint16_t *power_pattern_data(void);
struct clock_pattern_event *clock_pattern_data(void);

#endif
