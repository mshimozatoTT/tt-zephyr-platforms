/*
 * Copyright (c) 2026 Tenstorrent AI ULC
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef POWER_PATTERN_H
#define POWER_PATTERN_H

#include <stdint.h>

#include <tenstorrent/msgqueue.h>

struct response;

/** Sample VCORE TDP once per DVFS tick while capture is armed (see characterization START). */
void power_counter(void);

/** Call from AICLK GO_BUSY handler when power capture defers on GO_BUSY. */
void power_pattern_on_go_busy(void);

uint8_t power_pattern_start(const struct characterisation_clock_counter_start_submsg *params);
uint8_t power_pattern_stop(void);
uint8_t power_pattern_get_info(struct response *response);

#endif
