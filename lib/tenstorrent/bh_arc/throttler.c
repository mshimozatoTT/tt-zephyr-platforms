/*
 * Copyright (c) 2024 Tenstorrent AI ULC
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <string.h>

#include <zephyr/sys/util.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/zbus/zbus.h>
#include "throttler.h"
#include "aiclk_ppm.h"
#include <tenstorrent/smc_msg.h>
#include <tenstorrent/msgqueue.h>
#include "cm2dm_msg.h"
#include <zephyr/drivers/misc/bh_fwtable.h>
#include <zephyr/tracing/tracing.h>
#include "telemetry_internal.h"
#include "telemetry.h"
#include "noc2axi.h"
#include "tensix_state_msg.h"

static uint32_t power_limit;

static bool doppler;
static bool doppler_slow;
static bool doppler_t2;
static bool doppler_t3;
static const bool thermal_throttling = true;

/*
 * Kernel-throttler-at-AICLK-floor configuration. The defaults are sourced from
 * the firmware table at init (feature_enable.kernel_throttler_at_floor_en and
 * chip_limits.kernel_throttler_stop_nops_freq), so they can be persisted in SPI
 * flash and overridden via bh-mod.
 */
static uint32_t kernel_throttler_stop_nops_freq;
static uint32_t kernel_throttler_stop_nops_freq_default;


#define kThrottlerAiclkScaleFactor 500.0F
#define DEFAULT_BOARD_POWER_LIMIT  150

/* Anti-windup clamp for the integrator state, expressed as a fraction of
 * the current throttler limit. The integrator accumulates the absolute
 * error (in source units, e.g. W or degC) only when the loop is active
 * (i.e. error outside the deadband). A clamp of 5 * limit means the
 * integrator saturates after roughly 5 ticks of full-limit error, which
 * keeps it responsive without long unwind times. Saturation against
 * @c du_max_up / @c du_max_down further tightens this via back-calculation.
 */
#define THROTTLER_INTEGRAL_CLAMP_LIMIT_FRAC 5.0F

/* Runtime-toggleable: when false, UpdateThrottler() runs the legacy linear
 * law (single p_gain/d_gain on normalised error, no integrator/deadband/
 * slew cap) instead of the asymmetric law. Flipped via the
 * TT_SMC_MSG_THROTTLER_ASYMMETRIC_EN host message. Defaults to true so
 * existing behaviour is preserved unless the host explicitly opts out.
 */
static bool asymmetric_pd_enabled = true;

LOG_MODULE_REGISTER(throttler);

static const struct device *const fwtable_dev = DEVICE_DT_GET(DT_NODELABEL(fwtable));

typedef struct {
	float min;
	float max;
} ThrottlerLimitRange;

/* This table is used to restrict the throttler limits to reasonable ranges. */
/* They are passed in from the FW table in SPI */
/* clang-format off */
static const ThrottlerLimitRange throttler_limit_ranges[kThrottlerCount] = {
	[kThrottlerTDP]		= { .min = 50, .max = 500, },
	[kThrottlerFastTDC]	= { .min = 50, .max = 500, },
	[kThrottlerTDC]		= { .min = 50, .max = 400, },
	[kThrottlerThm]		= { .min = 50, .max = 100, },
	[kThrottlerBoardPower]	= { .min = 50, .max = 600, },
	[kThrottlerGDDRThm]	= { .min = 50, .max = 100, },
	[kThrottlerDopplerSlow]	= { .min = 50, .max = 1200, },
};
/* clang-format on */

typedef struct {
	const enum aiclk_arb_max arb_max; /* The arbiter associated with this throttler */

	ThrottlerParams params;
	float limit;
	float value;
	float error;        /* Normalised error: (limit - value) / limit (telemetry) */
	float integral;     /* Anti-windup-clamped sum of err_for_loop */
	float prev_err_abs; /* prev absolute error used by the asymmetric law */
	float du;           /* Per-tick frequency delta in MHz applied to the arbiter */
} Throttler;

/* clang-format off */
/* Only kThrottlerBoardPower ships with a fully populated asymmetric
 * configuration (under-limit gains, over-limit gains, deadbands and slew
 * caps). It operates on a power error in watts against the 300 W PSYS
 * ceiling from the fw_table. The asymmetric path in UpdateThrottler() is
 * gated to this throttler exclusively so the host toggle
 * (TT_SMC_MSG_THROTTLER_ASYMMETRIC_EN) cannot accidentally route any other
 * throttler through the asymmetric law with its asymmetric-extras left at
 * zero (which would produce du = 0 on over-limit and break those loops).
 *
 * All other throttlers (TDP / FastTDC / TDC / Thm / BoardGDDRThm /
 * DopplerSlow) carry the proportional / derivative values from the
 * pre-asymmetric linear-law implementation. They always run the legacy
 * linear law (operating on the dimensionless normalised error scaled by
 * kThrottlerAiclkScaleFactor), regardless of the asymmetric toggle. They
 * can be re-tuned at runtime via TT_SMC_MSG_THROTTLER_PD_PARAM (see
 * scripts/tune_throttler_pd.py).
 */
static Throttler throttler[kThrottlerCount] = {
	[kThrottlerTDP] = {
			.arb_max = aiclk_arb_max_tdp,
			.params = {
					.alpha_filter = 1.0,
					.p_gain = 0.015,
					.i_gain = 0.0,
					.d_gain = 0.1,
				},
		},
	[kThrottlerFastTDC] = {
			.arb_max = aiclk_arb_max_fast_tdc,
			.params = {
					.alpha_filter = 1.0,
					.p_gain = 0.5,
					.i_gain = 0.0,
					.d_gain = 0,
				},
		},
	[kThrottlerTDC] = {
			.arb_max = aiclk_arb_max_tdc,
			.params = {
					.alpha_filter = 0.1,
					.p_gain = 0.2,
					.i_gain = 0.0,
					.d_gain = 0,
				},
		},
	[kThrottlerThm] = {
			.arb_max = aiclk_arb_max_thm,
			.params = {
					.alpha_filter = 1.0,
					.p_gain = 0.2,
					.i_gain = 0.0,
					.d_gain = 0,
				},
		},
	[kThrottlerBoardPower] = {
			.arb_max = aiclk_arb_max_board_power,
			.params = {
					.alpha_filter = 1.0,
					.p_gain = 0.4f,
					.i_gain = 0.0f,
					.d_gain = 0.0f,
					.p_gain_over = 0.1f,
					.i_gain_over = 0.0f,
					.d_gain_over = 0.0f,
					.deadband_under = 0.01f,
					.deadband_over = 0.03f,
					.du_max_up = 50.0f,
					.du_max_down = -10.0f,
				},
		},
	[kThrottlerGDDRThm] = {
			.arb_max = aiclk_arb_max_gddr_thm,
			.params = {
					.alpha_filter = 1.0,
					.p_gain = 0.2,
					.i_gain = 0.0,
					.d_gain = 0,
				},
		},
	[kThrottlerDopplerSlow] = {
			.arb_max = aiclk_arb_max_doppler_slow,
			.params = {
					.alpha_filter = 1.0,
					.p_gain = 0.0025,
					.i_gain = 0.0,
					.d_gain = 0.3,
				},
		},
};
/* clang-format on */

static float get_throttler_clamped_limit(ThrottlerId id, float limit)
{
	return CLAMP(limit, throttler_limit_ranges[id].min, throttler_limit_ranges[id].max);
}

static void SetThrottlerLimit(ThrottlerId id, float limit)
{
	float clamped_limit = get_throttler_clamped_limit(id, limit);

	LOG_INF("Throttler %d limit set to %d", id, (uint32_t)clamped_limit);
	throttler[id].limit = clamped_limit;
}

static uint32_t throttle_counter;
static const uint32_t kKernelThrottleAddress = 0x10;
static bool tensixes_enabled = true;

static uint32_t nop_on_since_ms;      /* uptime when NOP last turned on */
static uint32_t nop_on_accum_ms;      /* total ms NOP's been on until now */
static uint32_t prev_nop_on_accum_ms; /* total ms NOP's been on until prev telemetry update */

static void BroadcastKernelThrottleState(void)
{
	const uint8_t kNocRing = 0;
	const uint8_t kNocTlb = 1;

	if (tensixes_enabled) {
		sys_trace_named_event("kernel_throttle", throttle_counter & 1, 0);
		NOC2AXITensixBroadcastTlbSetup(kNocRing, kNocTlb, kKernelThrottleAddress,
					       kNoc2AxiOrderingStrict);
		NOC2AXIWrite32(kNocRing, kNocTlb, kKernelThrottleAddress, throttle_counter);
	}
}

static void InitKernelThrottling(void)
{
	throttle_counter = 0;
	nop_on_since_ms = 0;
	nop_on_accum_ms = 0;
	prev_nop_on_accum_ms = 0;

	BroadcastKernelThrottleState();
}

/* must only be called when throttle state changes */
static void SendKernelThrottlingMessage(bool throttle)
{
	/* The LLK uses fast = even, slow = odd, but for debug purposes, they'd like to
	 * know how many times throttling has happened. Just in case CMFW somehow gets
	 * out of sync internally, double-check the parity.
	 */
	throttle_counter++;
	if ((throttle_counter & 1) != throttle) {
		throttle_counter++;
	}

	/* Accumulate NOP-on time: stamp the start on the rising edge, bank the
	 * elapsed interval on the falling edge. Centralised here so every edge
	 * (kernel throttler, Doppler, and feature-disable paths) is accounted for.
	 */
	if (throttle) {
		nop_on_since_ms = k_uptime_get_32();
	} else {
		nop_on_accum_ms += k_uptime_get_32() - nop_on_since_ms;
	}

	BroadcastKernelThrottleState();
}

static void doppler_tensix_state_callback(const struct zbus_channel *chan)
{
	const struct tensix_state_msg *msg = zbus_chan_const_msg(chan);

	tensixes_enabled = msg->enable;

	BroadcastKernelThrottleState();
}

ZBUS_LISTENER_DEFINE(doppler_tensix_state_listener, doppler_tensix_state_callback);
ZBUS_CHAN_ADD_OBS(tensix_state_chan, doppler_tensix_state_listener, 0);

void InitThrottlers(void)
{
	doppler = tt_bh_fwtable_get_fw_table(fwtable_dev)->feature_enable.doppler_en;
	doppler_slow = doppler;
	doppler_t2 = doppler;
	doppler_t3 = doppler;

	kernel_throttler_stop_nops_freq_default =
		tt_bh_fwtable_get_fw_table(fwtable_dev)
			->chip_limits.kernel_throttler_stop_nops_freq;
	/* A non-zero stop frequency must be within the valid AICLK floor range.
	 * An out-of-range value (e.g. from a board table or ccfgovr override)
	 * could otherwise leave kernel NOPs permanently engaged, so treat it as
	 * 0 (fall back to the effective minimum arbiter frequency at runtime).
	 */
	if (kernel_throttler_stop_nops_freq_default != 0U &&
	    (kernel_throttler_stop_nops_freq_default < (uint32_t)AICLK_FMIN_MIN ||
	     kernel_throttler_stop_nops_freq_default > (uint32_t)AICLK_FMIN_MAX)) {
		LOG_WRN("Invalid fwtable kernel_throttler_stop_nops_freq=%u MHz; using 0 (auto)",
			kernel_throttler_stop_nops_freq_default);
		kernel_throttler_stop_nops_freq_default = 0U;
	}
	kernel_throttler_stop_nops_freq = kernel_throttler_stop_nops_freq_default;
	UpdateTelemetryKernelThrottler(tt_bh_fwtable_get_fw_table(fwtable_dev)
					       ->feature_enable.kernel_throttler_at_floor_en,
				       kernel_throttler_stop_nops_freq);

	SetThrottlerLimit(kThrottlerTDP,
			  tt_bh_fwtable_get_fw_table(fwtable_dev)->chip_limits.tdp_limit);
	SetThrottlerLimit(kThrottlerFastTDC,
			  tt_bh_fwtable_get_fw_table(fwtable_dev)->chip_limits.tdc_fast_limit);
	SetThrottlerLimit(kThrottlerTDC,
			  tt_bh_fwtable_get_fw_table(fwtable_dev)->chip_limits.tdc_limit);
	SetThrottlerLimit(kThrottlerThm,
			  tt_bh_fwtable_get_fw_table(fwtable_dev)->chip_limits.thm_limit);
	/* Initialise kThrottlerBoardPower to the board input-power ceiling from
	 * the fw_table (300 W on P150A). This is the same ceiling
	 * Dm2CmSetBoardPowerLimit() clamps to when the cable / DMC negotiates a
	 * value, so starting from chip_limits gives us the full PSYS headroom
	 * out of the box and lets the asymmetric law on BoardPower actually
	 * exercise 300 W when no cable-side push happens (typical bring-up /
	 * characterisation setup).
	 *
	 * DEFAULT_BOARD_POWER_LIMIT is kept as a defensive fallback in case the
	 * fw_table reports a zero ceiling (which would otherwise wedge the
	 * throttler at 0 W).
	 */
	{
		uint32_t board_limit =
			tt_bh_fwtable_get_fw_table(fwtable_dev)->chip_limits.board_power_limit;
		if (board_limit == 0) {
			board_limit = DEFAULT_BOARD_POWER_LIMIT;
		}
		SetThrottlerLimit(kThrottlerBoardPower, board_limit);
	}
	/* kThrottlerDopplerSlow keeps the pre-asymmetric default: 150 W until
	 * Dm2CmSetBoardPowerLimit() pushes a cable-negotiated value (capped at
	 * chip_limits.board_power_limit). Matches the legacy behaviour exactly
	 * so the asymmetric A/B test cannot accidentally exercise DopplerSlow.
	 */
	SetThrottlerLimit(kThrottlerDopplerSlow, DEFAULT_BOARD_POWER_LIMIT);
	SetThrottlerLimit(kThrottlerGDDRThm,
			  tt_bh_fwtable_get_fw_table(fwtable_dev)->chip_limits.gddr_thm_limit);

	InitKernelThrottling();

	EnableArbMax(throttler[kThrottlerTDP].arb_max, !doppler);
	EnableArbMax(throttler[kThrottlerFastTDC].arb_max, !doppler);
	EnableArbMax(throttler[kThrottlerTDC].arb_max, !doppler);
	EnableArbMax(throttler[kThrottlerBoardPower].arb_max, !doppler);

	EnableArbMax(throttler[kThrottlerThm].arb_max, thermal_throttling);
	EnableArbMax(throttler[kThrottlerGDDRThm].arb_max, thermal_throttling);

	EnableArbMax(throttler[kThrottlerDopplerSlow].arb_max, doppler_slow);

	SetAiclkArbMax(aiclk_arb_max_doppler_critical, GetAiclkFmin());
	EnableArbMax(aiclk_arb_max_doppler_critical, false); /* enabled when limit triggered */
}

/* Sample period (s) used to scale the derivative term. DVFS runs every
 * DVFS_MSEC (= 1 ms) so Ts = 1e-3 s. Keeping this explicit makes the
 * user-facing Kd gain have units of MHz*s per source-unit.
 */
#define THROTTLER_TS_SECONDS 0.001f

static void UpdateThrottler(ThrottlerId id, float value)
{
	Throttler *t = &throttler[id];

	t->value = t->params.alpha_filter * value + (1 - t->params.alpha_filter) * t->value;

	/* Telemetry-facing normalised error. */
	t->error = (t->limit - t->value) / t->limit;

	/* The asymmetric law currently only ships with a fully populated config
	 * on kThrottlerBoardPower (under-limit/over-limit gains, deadbands and
	 * slew caps). All other throttlers carry legacy linear-law gains only
	 * and would behave incorrectly under the asymmetric path (du = 0 when
	 * over-limit, because p_gain_over/d_gain_over/du_max_down all default
	 * to 0). Gate the asymmetric path explicitly to BoardPower so the host
	 * toggle can't accidentally cripple the other throttlers.
	 */
	if (!asymmetric_pd_enabled || id != kThrottlerBoardPower) {
		/* Legacy linear law (pre-asymmetric-PD-loop behaviour): operate on
		 * the dimensionless normalised error, no integrator, no deadband,
		 * no slew cap. The @c prev_err_abs slot is reused to hold the
		 * previous normalised error in this mode; it is reset to 0 on
		 * every law transition by the host-facing toggle handler.
		 */
		float prev_norm = t->prev_err_abs;
		float output = t->params.p_gain * t->error +
			       t->params.d_gain * (t->error - prev_norm);

		t->prev_err_abs = t->error;
		t->du = output * kThrottlerAiclkScaleFactor;
		return;
	}

	float err_abs = t->limit - t->value;
	float deadband_under_thr = t->params.deadband_under * t->limit;
	float deadband_over_thr = t->params.deadband_over * t->limit;
	float kp;
	float kd;
	float ki;
	float err_for_loop;

	if (err_abs > deadband_under_thr) {
		kp = t->params.p_gain;
		kd = t->params.d_gain;
		ki = t->params.i_gain;
		err_for_loop = err_abs;
	} else if (err_abs < -deadband_over_thr) {
		kp = t->params.p_gain_over;
		kd = t->params.d_gain_over;
		ki = t->params.i_gain_over;
		err_for_loop = err_abs;
	} else {
		kp = 0.0f;
		kd = 0.0f;
		ki = 0.0f;
		err_for_loop = 0.0f;
	}

	/* Integrate only outside the deadband: inside, err_for_loop is 0 so
	 * the state is held; the I-term is also gated by ki=0 there so it
	 * doesn't push the loop while the deadband is intended to be quiet.
	 */
	t->integral += err_for_loop;

	float integral_max = t->limit * THROTTLER_INTEGRAL_CLAMP_LIMIT_FRAC;

	t->integral = CLAMP(t->integral, -integral_max, integral_max);

	float de = (err_for_loop - t->prev_err_abs) / THROTTLER_TS_SECONDS;
	float du = kp * err_for_loop + ki * t->integral + kd * de;

	/* Back-calculation anti-windup: when du saturates against a slew cap,
	 * remove the saturation excess from the integrator so it does not
	 * accumulate effort the actuator could not have delivered.
	 */
	if (t->params.du_max_up > 0.0f && du > t->params.du_max_up) {
		if (ki != 0.0f) {
			t->integral -= (du - t->params.du_max_up) / ki;
		}
		du = t->params.du_max_up;
	}
	if (t->params.du_max_down < 0.0f && du < t->params.du_max_down) {
		if (ki != 0.0f) {
			t->integral -= (du - t->params.du_max_down) / ki;
		}
		du = t->params.du_max_down;
	}

	t->prev_err_abs = err_for_loop;
	t->du = du;
}

static void UpdateThrottlerArb(ThrottlerId id)
{
	Throttler *t = &throttler[id];

	float arb_val = GetThrottlerArbMax(t->arb_max);

	arb_val += t->du;

	SetAiclkArbMax(t->arb_max, arb_val);
}

static uint16_t board_power_history[1000];
static uint16_t *board_power_history_cursor = board_power_history;
static uint32_t board_power_sum;
static bool kernel_nops_enabled;

static uint8_t t2_count;
static uint8_t t3_count;

#define ADVANCE_CIRCULAR_POINTER(pointer, array)                                                   \
	do {                                                                                       \
		if (++(pointer) == (array) + ARRAY_SIZE(array))                                    \
			(pointer) = (array);                                                       \
	} while (false)

static uint16_t UpdateMovingAveragePower(uint16_t current_power)
{
	board_power_sum += current_power - *board_power_history_cursor;
	*board_power_history_cursor = current_power;

	ADVANCE_CIRCULAR_POINTER(board_power_history_cursor, board_power_history);

	return board_power_sum / ARRAY_SIZE(board_power_history);
}

static bool DopplerActive(void)
{
	return doppler && power_limit > 0;
}

static void UpdateDoppler(const TelemetryInternalData *telemetry)
{
	uint16_t current_power = GetInputPower();
	uint16_t average_power = UpdateMovingAveragePower(current_power);

	UpdateThrottler(kThrottlerDopplerSlow, average_power);

	/* Doppler T2 throttler: 2x power limit for 10 consecutive samples */
	uint32_t t2_power_limit = power_limit * 2;

	if (current_power > t2_power_limit) {
		if (t2_count < UINT8_MAX) {
			t2_count++;
		}
	} else {
		t2_count = 0;
	}

	bool t2_triggered = t2_count >= 10 && doppler_t2;

	/* Doppler T3 throttler: 2.5x power limit for 2 consecutive samples */
	uint32_t t3_power_limit = power_limit * 5 / 2;

	if (current_power > t3_power_limit) {
		if (t3_count < UINT8_MAX) {
			t3_count++;
		}
	} else {
		t3_count = 0;
	}

	bool t3_triggered = t3_count >= 2 && doppler_t3;

	/* AICLK=Fmin isn't always enough to get below the board power limit. */
	bool start_nops = GetAiclkTarg() == GetAiclkFmin() && current_power > power_limit;
	bool stop_nops = GetAiclkTarg() == GetAiclkFmax() && current_power < power_limit;

	bool critical_throttling = t2_triggered || t3_triggered;

	bool new_kernel_nops_enabled =
		((kernel_nops_enabled || start_nops) && !stop_nops) || critical_throttling;

	if (new_kernel_nops_enabled != kernel_nops_enabled) {
		kernel_nops_enabled = new_kernel_nops_enabled;
		SendKernelThrottlingMessage(kernel_nops_enabled);
	}

	EnableArbMax(aiclk_arb_max_doppler_critical, critical_throttling);
}

/* Update kernel throttler NOPs state when running at the AICLK floor.
 *
 * This path is enabled when TAG_FW_ACTIVE_CONFIG_0 bit 0
 * (kernel_nops_at_aiclk_fmin) is set. The bit is seeded from the fwtable at
 * telemetry init and may later be changed at runtime via the characterization
 * message path.
 *
 * The stop frequency comes from kernel_throttler_stop_nops_freq. When that
 * value is 0, FW falls back to the effective minimum arbiter frequency.
 */
static void UpdateKernelThrottler(float current_power, float tdp_limit)
{
	telemetry_feature_flags_bits_0_t active_config = GetActiveFeatures();
	bool start_nops = false;
	bool stop_nops = false;
	enum aiclk_arb_min arb;

	if (active_config.kernel_nops_at_aiclk_fmin) {
		start_nops = GetAiclkTarg() == GetAiclkFmin() && current_power > tdp_limit;

		uint32_t stop_freq = kernel_throttler_stop_nops_freq;

		if (stop_freq == 0U) {
			stop_freq = get_aiclk_effective_arb_min(&arb);
		}

		stop_nops = GetAiclkTarg() >= stop_freq && current_power < tdp_limit;
	}

	bool new_kernel_nops_enabled = ((kernel_nops_enabled || start_nops) && !stop_nops);

	if (new_kernel_nops_enabled != kernel_nops_enabled) {
		kernel_nops_enabled = new_kernel_nops_enabled;
		SendKernelThrottlingMessage(kernel_nops_enabled);
	}
}

void CalculateThrottlers(void)
{
	TelemetryInternalData telemetry_internal_data;

	ReadTelemetryInternal(1, &telemetry_internal_data);

	if (DopplerActive()) {
		UpdateDoppler(&telemetry_internal_data);
	} else {
		UpdateThrottler(kThrottlerTDP, telemetry_internal_data.vcore_power);
		UpdateThrottler(kThrottlerFastTDC, telemetry_internal_data.vcore_current);
		UpdateThrottler(kThrottlerTDC, telemetry_internal_data.vcore_current);
		UpdateThrottler(kThrottlerBoardPower, GetInputPower());

		float current_power = telemetry_internal_data.vcore_power;
		float tdp_limit = throttler[kThrottlerTDP].limit;

		UpdateKernelThrottler(current_power, tdp_limit);
	}

	UpdateThrottler(kThrottlerThm, telemetry_internal_data.asic_temperature);
	UpdateThrottler(kThrottlerGDDRThm, telemetry_internal_data.gddr_temps.max_temp);

	for (ThrottlerId i = 0; i < kThrottlerCount; i++) {
		UpdateThrottlerArb(i);
	}
}

uint8_t ThrottlerSetKernelThrottlerEnabled(uint32_t enabled)
{
	if (enabled > 1) {
		return 1;
	}

	LOG_INF("kernel throttler at aiclk floor %s", enabled ? "enabled" : "disabled");

	/* Release NOPs immediately if the feature is being disabled while active. */
	if (!enabled && kernel_nops_enabled) {
		kernel_nops_enabled = false;
		SendKernelThrottlingMessage(false);
	}

	UpdateTelemetryKernelThrottler((bool)enabled, kernel_throttler_stop_nops_freq);
	return 0;
}

uint8_t ThrottlerSetKernelThrottlerStopFreq(uint32_t frequency)
{
	/* 0 restores the fwtable-provided default (which may itself be 0, meaning
	 * fall back to the effective minimum arbiter frequency at runtime).
	 */
	if (frequency == 0) {
		telemetry_feature_flags_bits_0_t active_config = GetActiveFeatures();

		kernel_throttler_stop_nops_freq = kernel_throttler_stop_nops_freq_default;
		LOG_INF("kernel throttler stop nops frequency restored to fwtable default %u MHz",
			kernel_throttler_stop_nops_freq);
		UpdateTelemetryKernelThrottler(active_config.kernel_nops_at_aiclk_fmin,
					       kernel_throttler_stop_nops_freq);
		return 0;
	}

	/* Reject if outside valid range [AICLK_FMIN_MIN, AICLK_FMIN_MAX] */
	if (frequency > (uint32_t)AICLK_FMIN_MAX || frequency < (uint32_t)AICLK_FMIN_MIN) {
		return 1;
	}

	telemetry_feature_flags_bits_0_t active_config = GetActiveFeatures();

	kernel_throttler_stop_nops_freq = frequency;
	LOG_INF("kernel throttler stop nops frequency set to %u MHz", frequency);
	UpdateTelemetryKernelThrottler(active_config.kernel_nops_at_aiclk_fmin,
				       kernel_throttler_stop_nops_freq);
	return 0;
}

int32_t Dm2CmSetBoardPowerLimit(const uint8_t *data, uint8_t size)
{
	if (size != 2) {
		return -1;
	}

	power_limit = sys_get_le16(data);

	LOG_INF("Cable Power Limit: %u", power_limit);
	power_limit = MIN(power_limit,
			  tt_bh_fwtable_get_fw_table(fwtable_dev)->chip_limits.board_power_limit);

	SetThrottlerLimit(kThrottlerBoardPower, power_limit);
	SetThrottlerLimit(kThrottlerDopplerSlow, power_limit);

	UpdateTelemetryBoardPowerLimit(power_limit);

	return 0;
}

static uint8_t set_tdp_limit_handler(const union request *request, struct response *response)
{
	float default_tdp_limit = get_throttler_clamped_limit(
		kThrottlerTDP, tt_bh_fwtable_get_fw_table(fwtable_dev)->chip_limits.tdp_limit);
	float max_tdp_limit =
		CLAMP(tt_bh_fwtable_get_fw_table(fwtable_dev)->chip_limits.max_tdp_limit,
		      default_tdp_limit, throttler_limit_ranges[kThrottlerTDP].max);
	float new_tdp_limit;

	if (request->set_tdp_limit.restore_default) {
		new_tdp_limit = default_tdp_limit;
	} else {
		new_tdp_limit = request->set_tdp_limit.tdp_limit;
	}

	/* Return an error if the new TDP limit is outside of the valid range */
	if (new_tdp_limit > max_tdp_limit) {
		return 1;
	} else if (get_throttler_clamped_limit(kThrottlerTDP, new_tdp_limit) != new_tdp_limit) {
		return 1;
	}

	SetThrottlerLimit(kThrottlerTDP, new_tdp_limit);
	UpdateTelemetryTdpLimit(throttler[kThrottlerTDP].limit);

	return 0;
}

uint32_t GetStartNOPCount(void)
{
	/* throttle_counter increments on every throttle-state change.
	 * Need to convert transition count to NOP start count
	 */
	return (throttle_counter + 1) >> 1;
}

uint32_t GetNOPOnAccumulatedTime(void)
{
	/* If NOPs are currently enabled, add time since they were last turned on to
	 * accumulated time. Wraps at ~49.7 days of cumulative NOP-on time; consumers
	 * must difference samples with unsigned (modular) arithmetic.
	 */
	if (kernel_nops_enabled) {
		return nop_on_accum_ms + (k_uptime_get_32() - nop_on_since_ms);
	} else {
		return nop_on_accum_ms;
	}
}

uint32_t GetNOPOnDuration(uint32_t window_ms)
{
	/* NOP-on time accrued since the previous call. Unsigned subtraction stays
	 * correct across accumulator wrap, since one window's delta is tiny relative
	 * to the 32-bit millisecond range.
	 */
	uint32_t accumulated_time = GetNOPOnAccumulatedTime();
	uint32_t duration = accumulated_time - prev_nop_on_accum_ms;

	prev_nop_on_accum_ms = accumulated_time;

	/* On the first call prev_nop_on_accum_ms is still 0 from init, so the delta is
	 * the entire NOP-on time banked since boot rather than a single window. Clamp
	 * that bootstrap sample to the window length. `seeded` makes this one-shot:
	 * later samples are returned unclamped so their running sum stays faithful to
	 * the true cumulative NOP-on time.
	 */
	static bool seeded;

	if (!seeded) {
		seeded = true;
		duration = MIN(duration, window_ms);
	}

	return duration;
}

REGISTER_MESSAGE(TT_SMC_MSG_SET_TDP_LIMIT, set_tdp_limit_handler);

static uint32_t pd_param_to_bits(float value)
{
	uint32_t bits;

	memcpy(&bits, &value, sizeof(bits));
	return bits;
}

static float pd_param_from_bits(uint32_t bits)
{
	float value;

	memcpy(&value, &bits, sizeof(value));
	return value;
}

int throttler_get_pd_param(ThrottlerId id, enum throttler_pd_param_id param, uint32_t *out_bits)
{
	if (id >= kThrottlerCount || out_bits == NULL) {
		return -1;
	}

	const ThrottlerParams *p = &throttler[id].params;

	switch (param) {
	case THROTTLER_PD_PARAM_ALPHA_FILTER:
		*out_bits = pd_param_to_bits(p->alpha_filter);
		return 0;
	case THROTTLER_PD_PARAM_P_GAIN:
		*out_bits = pd_param_to_bits(p->p_gain);
		return 0;
	case THROTTLER_PD_PARAM_D_GAIN:
		*out_bits = pd_param_to_bits(p->d_gain);
		return 0;
	case THROTTLER_PD_PARAM_I_GAIN:
		*out_bits = pd_param_to_bits(p->i_gain);
		return 0;
	case THROTTLER_PD_PARAM_P_GAIN_OVER:
		*out_bits = pd_param_to_bits(p->p_gain_over);
		return 0;
	case THROTTLER_PD_PARAM_I_GAIN_OVER:
		*out_bits = pd_param_to_bits(p->i_gain_over);
		return 0;
	case THROTTLER_PD_PARAM_D_GAIN_OVER:
		*out_bits = pd_param_to_bits(p->d_gain_over);
		return 0;
	case THROTTLER_PD_PARAM_DEADBAND_UNDER:
		*out_bits = pd_param_to_bits(p->deadband_under);
		return 0;
	case THROTTLER_PD_PARAM_DEADBAND_OVER:
		*out_bits = pd_param_to_bits(p->deadband_over);
		return 0;
	case THROTTLER_PD_PARAM_DU_MAX_UP:
		*out_bits = pd_param_to_bits(p->du_max_up);
		return 0;
	case THROTTLER_PD_PARAM_DU_MAX_DOWN:
		*out_bits = pd_param_to_bits(p->du_max_down);
		return 0;
	default:
		return -1;
	}
}

int throttler_set_pd_param(ThrottlerId id, enum throttler_pd_param_id param, uint32_t bits)
{
	if (id >= kThrottlerCount) {
		return -1;
	}

	ThrottlerParams *p = &throttler[id].params;
	float fvalue = pd_param_from_bits(bits);

	switch (param) {
	case THROTTLER_PD_PARAM_ALPHA_FILTER:
		if (!(fvalue >= 0.0f && fvalue <= 1.0f)) {
			return -1;
		}
		p->alpha_filter = fvalue;
		return 0;
	case THROTTLER_PD_PARAM_P_GAIN:
		p->p_gain = fvalue;
		return 0;
	case THROTTLER_PD_PARAM_D_GAIN:
		p->d_gain = fvalue;
		return 0;
	case THROTTLER_PD_PARAM_I_GAIN:
		p->i_gain = fvalue;
		/* Reset the integrator so a fresh gain doesn't multiply a state
		 * built up under the previous tuning.
		 */
		throttler[id].integral = 0.0f;
		return 0;
	case THROTTLER_PD_PARAM_P_GAIN_OVER:
		p->p_gain_over = fvalue;
		return 0;
	case THROTTLER_PD_PARAM_I_GAIN_OVER:
		p->i_gain_over = fvalue;
		/* Reset the (shared) integrator so a fresh over-limit gain
		 * doesn't multiply a state built up under the previous tuning.
		 */
		throttler[id].integral = 0.0f;
		return 0;
	case THROTTLER_PD_PARAM_D_GAIN_OVER:
		p->d_gain_over = fvalue;
		return 0;
	case THROTTLER_PD_PARAM_DEADBAND_UNDER:
		if (!(fvalue >= 0.0f && fvalue < 1.0f)) {
			return -1;
		}
		p->deadband_under = fvalue;
		return 0;
	case THROTTLER_PD_PARAM_DEADBAND_OVER:
		if (!(fvalue >= 0.0f && fvalue < 1.0f)) {
			return -1;
		}
		p->deadband_over = fvalue;
		return 0;
	case THROTTLER_PD_PARAM_DU_MAX_UP:
		if (fvalue < 0.0f) {
			return -1;
		}
		p->du_max_up = fvalue;
		return 0;
	case THROTTLER_PD_PARAM_DU_MAX_DOWN:
		if (fvalue > 0.0f) {
			return -1;
		}
		p->du_max_down = fvalue;
		return 0;
	default:
		return -1;
	}
}

static uint8_t throttler_pd_param_handler(const union request *request, struct response *response)
{
	uint8_t op = request->throttler_pd_param.op;
	uint8_t id = request->throttler_pd_param.throttler_id;
	uint8_t param = request->throttler_pd_param.param_id;

	switch (op) {
	case THROTTLER_PD_PARAM_OP_GET: {
		uint32_t bits = 0;
		int rc = throttler_get_pd_param((ThrottlerId)id,
						 (enum throttler_pd_param_id)param, &bits);
		if (rc != 0) {
			return 1;
		}
		response->data[1] = bits;
		return 0;
	}
	case THROTTLER_PD_PARAM_OP_SET: {
		int rc = throttler_set_pd_param((ThrottlerId)id,
						 (enum throttler_pd_param_id)param,
						 request->throttler_pd_param.value);
		if (rc != 0) {
			return 1;
		}
		LOG_INF("Throttler %u PD param %u updated (0x%08x)", id, param,
			request->throttler_pd_param.value);
		return 0;
	}
	default:
		return 1;
	}
}

REGISTER_MESSAGE(TT_SMC_MSG_THROTTLER_PD_PARAM, throttler_pd_param_handler);

static uint8_t throttler_asymmetric_en_handler(const union request *request,
					       struct response *response)
{
	bool en = (request->throttler_asymmetric_en.enable != 0);

	if (en != asymmetric_pd_enabled) {
		asymmetric_pd_enabled = en;
		/* Reset per-throttler state so the new law does not start
		 * from state accumulated under the previous law. (In particular
		 * the meaning of @c prev_err_abs differs between the two laws.)
		 */
		for (ThrottlerId i = 0; i < kThrottlerCount; i++) {
			throttler[i].integral = 0.0f;
			throttler[i].prev_err_abs = 0.0f;
			throttler[i].du = 0.0f;
		}
		LOG_INF("BoardPower asymmetric law %s",
			en ? "ENABLED" : "DISABLED (legacy linear)");
	}
	return 0;
}

REGISTER_MESSAGE(TT_SMC_MSG_THROTTLER_ASYMMETRIC_EN, throttler_asymmetric_en_handler);
