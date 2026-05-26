/*
 * Copyright (c) 2026 Tenstorrent AI ULC
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "capture_buffer.h"

#include <zephyr/sys/util.h>

BUILD_ASSERT(CAPTURE_POWER_BYTES >= (int)sizeof(uint16_t));
BUILD_ASSERT((CAPTURE_POWER_BYTES % 2) == 0);
BUILD_ASSERT(CAPTURE_CLOCK_BYTES >= (int)sizeof(struct clock_pattern_event));
BUILD_ASSERT((CAPTURE_CLOCK_BYTES % (int)sizeof(struct clock_pattern_event)) == 0);
BUILD_ASSERT(CAPTURE_POWER_BYTES + CAPTURE_CLOCK_BYTES == CAPTURE_BUFFER_BYTES);
BUILD_ASSERT(sizeof(struct clock_pattern_event) == 6);

uint8_t capture_buffer[CAPTURE_BUFFER_BYTES];

uint16_t *power_pattern_data(void)
{
	return (uint16_t *)(void *)capture_buffer;
}

struct clock_pattern_event *clock_pattern_data(void)
{
	return (struct clock_pattern_event *)(void *)(capture_buffer + CAPTURE_POWER_BYTES);
}
