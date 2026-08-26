/*
 * Copyright (c) 2026 Tenstorrent AI ULC
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef CAPTURE_BUFFER_H
#define CAPTURE_BUFFER_H

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

/** Host-visible capture RAM (default 144 KiB). Power and clock_pattern share this region. */
#define CAPTURE_BUFFER_BYTES CONFIG_TT_BH_ARC_CAPTURE_BUFFER_BYTES
/** Byte size of dense power samples at the start of @ref capture_buffer (must be even). */
#define CAPTURE_POWER_BYTES  CONFIG_TT_BH_ARC_POWER_CAPTURE_BYTES
/** Remaining bytes for sparse clock events (6 bytes each). */
#define CAPTURE_CLOCK_BYTES  (CAPTURE_BUFFER_BYTES - CAPTURE_POWER_BYTES)

#define POWER_PATTERN_SAMPLES (CAPTURE_POWER_BYTES / (int)sizeof(uint16_t))
#define CLOCK_PATTERN_ROWS    (CAPTURE_CLOCK_BYTES / (int)sizeof(struct clock_pattern_event))

extern uint8_t capture_buffer[CAPTURE_BUFFER_BYTES];

uint16_t *power_pattern_data(void);
struct clock_pattern_event *clock_pattern_data(void);

#endif
