/*
 * Copyright (c) 2026 Tenstorrent AI ULC
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "power_pattern.h"
#include "aiclk_ppm.h"
#include "capture_buffer.h"
#include "telemetry_internal.h"

#include <string.h>

#include <tenstorrent/smc_msg.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(power_pattern, CONFIG_TT_APP_LOG_LEVEL);

uint32_t power_pattern_next;
uint8_t power_pattern_ring_wrapped;

static bool enable_power_counter;
static uint16_t power_sample_phase;
static bool power_pattern_overflow_logged;
static uint32_t power_capture_not_before_ms;
static bool power_start_samples_on_go_busy;
static bool power_go_busy_seen_since_start;
static bool power_reset_index_on_go_busy;
static uint32_t power_capture_duration_ms;
static uint32_t power_capture_deadline_ms;

/** Magic for GET_POWER_PATTERN_INFO @c data[5] (bytes @c 70 77 72 01 = @c pwr + ver 1). */
#define POWER_PATTERN_INFO_MAGIC 0x01727770U

static uint16_t power_to_centiwatts(float power_w)
{
	if (power_w <= 0.f) {
		return 0U;
	}

	float cw = power_w * 100.f;

	if (cw >= 65535.f) {
		return 65535U;
	}

	return (uint16_t)cw;
}

void power_counter(void)
{
	if (!enable_power_counter) {
		return;
	}

	if (k_uptime_get_32() < power_capture_not_before_ms) {
		return;
	}

	if (power_start_samples_on_go_busy && !power_go_busy_seen_since_start) {
		return;
	}

	if (CONFIG_TT_BH_ARC_POWER_SAMPLE_DIVISOR > 1) {
		power_sample_phase++;
		if (power_sample_phase < CONFIG_TT_BH_ARC_POWER_SAMPLE_DIVISOR) {
			return;
		}
		power_sample_phase = 0;
	}

	if (power_capture_deadline_ms != 0U && k_uptime_get_32() >= power_capture_deadline_ms) {
		enable_power_counter = false;
		LOG_INF("power_pattern: capture_duration_ms elapsed — capture stopped");
		return;
	}

	TelemetryInternalData telemetry;

	ReadTelemetryInternal(1, &telemetry);

	uint32_t wr = power_pattern_next;

	if (IS_ENABLED(CONFIG_TT_BH_ARC_POWER_PATTERN_RING_BUFFER)) {
		if (wr >= POWER_PATTERN_SAMPLES) {
			power_pattern_ring_wrapped = 1U;
			wr = 0U;
			if (!power_pattern_overflow_logged) {
				LOG_INF("power_pattern ring: overwriting oldest samples (newest kept)");
				power_pattern_overflow_logged = true;
			}
		}
	} else if (wr >= POWER_PATTERN_SAMPLES) {
		if (!power_pattern_overflow_logged) {
			LOG_WRN("power_pattern full (%u samples); stopping capture",
				POWER_PATTERN_SAMPLES);
			power_pattern_overflow_logged = true;
		}
		enable_power_counter = false;
		return;
	}

	power_pattern_data()[wr] = power_to_centiwatts(telemetry.vcore_power);
	power_pattern_next = wr + 1U;
}

uint8_t power_pattern_start(const struct characterisation_clock_counter_start_submsg *params)
{
	memset(power_pattern_data(), 0, CAPTURE_POWER_BYTES);
	power_pattern_next = 0U;
	power_pattern_ring_wrapped = 0U;
	power_sample_phase = 0U;
	power_pattern_overflow_logged = false;

	uint32_t delay_ms = params->delay_ms;

	if (delay_ms > 300000U) {
		delay_ms = 300000U;
	}
	power_capture_not_before_ms = k_uptime_get_32() + delay_ms;
	if (delay_ms > 0U) {
		LOG_INF("power_pattern: delay %u ms before sampling", delay_ms);
	}

	power_start_samples_on_go_busy = (params->start_samples_on_go_busy & 1U) != 0;
	power_reset_index_on_go_busy = power_start_samples_on_go_busy;
	power_capture_duration_ms = params->capture_duration_ms;
	power_capture_deadline_ms = 0U;
	if (power_capture_duration_ms > 300000U) {
		power_capture_duration_ms = 300000U;
	}
	if (power_capture_duration_ms > 0U && !power_start_samples_on_go_busy) {
		power_capture_deadline_ms = power_capture_not_before_ms + power_capture_duration_ms;
		LOG_INF("power_pattern: auto-stop after %u ms from delay expiry",
			power_capture_duration_ms);
	}

	if (power_start_samples_on_go_busy) {
		power_go_busy_seen_since_start = aiclk_last_msg_busy();
		if (power_go_busy_seen_since_start) {
			power_reset_index_on_go_busy = false;
			if (power_capture_duration_ms > 0U) {
				power_capture_deadline_ms =
					power_capture_not_before_ms + power_capture_duration_ms;
			}
		}
		LOG_INF("power_pattern: defer samples until GO_BUSY (or already busy)");
	} else {
		power_go_busy_seen_since_start = true;
	}

	enable_power_counter = true;
	return 0;
}

uint8_t power_pattern_stop(void)
{
	enable_power_counter = false;
	power_start_samples_on_go_busy = false;
	power_go_busy_seen_since_start = false;
	power_reset_index_on_go_busy = false;
	power_capture_duration_ms = 0U;
	power_capture_deadline_ms = 0U;
	return 0;
}

uint8_t power_pattern_get_info(struct response *response)
{
	response->data[1] = (uint32_t)(uintptr_t)power_pattern_data();
	response->data[2] = POWER_PATTERN_SAMPLES;
	response->data[3] = (uint32_t)sizeof(uint16_t);
	response->data[4] = CONFIG_TT_BH_ARC_POWER_SAMPLE_DIVISOR;
	response->data[5] = POWER_PATTERN_INFO_MAGIC;
	response->data[6] = power_pattern_next;
	response->data[7] = power_pattern_ring_wrapped;
	return 0;
}

void power_pattern_on_go_busy(void)
{
	if (!enable_power_counter || !power_start_samples_on_go_busy ||
	    power_go_busy_seen_since_start) {
		return;
	}

	power_go_busy_seen_since_start = true;
	if (power_reset_index_on_go_busy) {
		memset(power_pattern_data(), 0, CAPTURE_POWER_BYTES);
		power_pattern_next = 0U;
		power_pattern_ring_wrapped = 0U;
		power_sample_phase = 0U;
		power_reset_index_on_go_busy = false;
		if (power_capture_duration_ms > 0U) {
			power_capture_deadline_ms = k_uptime_get_32() + power_capture_duration_ms;
			LOG_INF("power_pattern: GO_BUSY — index reset; auto-stop in %u ms",
				power_capture_duration_ms);
		} else {
			LOG_INF("power_pattern: GO_BUSY — index reset for compute window");
		}
	}
}
