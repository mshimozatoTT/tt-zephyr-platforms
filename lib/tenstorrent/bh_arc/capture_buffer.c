/*
 * Copyright (c) 2026 Tenstorrent AI ULC
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "capture_buffer.h"

#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/linker/linker-defs.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

LOG_MODULE_REGISTER(capture_buffer, CONFIG_TT_APP_LOG_LEVEL);

BUILD_ASSERT(sizeof(struct clock_pattern_event) == 6);

static uint8_t *capture_base;
static size_t power_bytes;
static size_t clock_bytes;

static void capture_buffer_layout(void)
{
	const uintptr_t sram_end =
		(uintptr_t)CONFIG_SRAM_BASE_ADDRESS + (uintptr_t)CONFIG_SRAM_SIZE * 1024U;
	/* 4-byte align so clock_pattern.seq is naturally aligned. */
	const uintptr_t start = ROUND_UP((uintptr_t)_end, 4);

	capture_base = NULL;
	power_bytes = 0;
	clock_bytes = 0;

	if (start >= sram_end) {
		LOG_ERR("capture: _end %#x at/past sram_end %#x", (uint32_t)start,
			(uint32_t)sram_end);
		return;
	}

	const uint32_t total = (uint32_t)(sram_end - start);
	uint32_t power = (total * CONFIG_TT_BH_ARC_POWER_CAPTURE_PERCENT / 100U) & ~3U;
	uint32_t clock = ((total - power) / 6U) * 6U;

	if (power < sizeof(uint16_t) || clock < sizeof(struct clock_pattern_event)) {
		LOG_ERR("capture: leftover %u B too small (power %u clock %u)", total, power,
			clock);
		return;
	}

	capture_base = (uint8_t *)start;
	power_bytes = power;
	clock_bytes = clock;

	LOG_INF("capture: %#x..%#x leftover %u B; power %u samples, clock %u events",
		(uint32_t)start, (uint32_t)sram_end, total, (uint32_t)(power / sizeof(uint16_t)),
		(uint32_t)(clock / sizeof(struct clock_pattern_event)));
}

static int capture_buffer_sys_init(void)
{
	capture_buffer_layout();
	return 0;
}

SYS_INIT(capture_buffer_sys_init, POST_KERNEL, 40);

bool capture_buffer_ready(void)
{
	return capture_base != NULL && power_bytes >= sizeof(uint16_t) &&
	       clock_bytes >= sizeof(struct clock_pattern_event);
}

size_t capture_power_bytes(void)
{
	return power_bytes;
}

size_t capture_clock_bytes(void)
{
	return clock_bytes;
}

uint32_t capture_power_samples(void)
{
	return (uint32_t)(power_bytes / sizeof(uint16_t));
}

uint32_t capture_clock_rows(void)
{
	return (uint32_t)(clock_bytes / sizeof(struct clock_pattern_event));
}

uint16_t *power_pattern_data(void)
{
	return (uint16_t *)(void *)capture_base;
}

struct clock_pattern_event *clock_pattern_data(void)
{
	if (capture_base == NULL) {
		return NULL;
	}

	return (struct clock_pattern_event *)(void *)(capture_base + power_bytes);
}
