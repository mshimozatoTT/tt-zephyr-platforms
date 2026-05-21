/*
 * Copyright (c) 2024 Tenstorrent AI ULC
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef THROTTLER_H
#define THROTTLER_H

#include <stdint.h>

/**
 * @brief Identifies a throttler control loop.
 *
 * The numeric values are part of the host-facing interface used by the
 * @ref TT_SMC_MSG_THROTTLER_PD_PARAM message and must not be reordered.
 */
typedef enum {
	kThrottlerTDP = 0,
	kThrottlerFastTDC = 1,
	kThrottlerTDC = 2,
	kThrottlerThm = 3,
	kThrottlerBoardPower = 4,
	kThrottlerGDDRThm = 5,
	kThrottlerDopplerSlow = 6,
	kThrottlerCount,
} ThrottlerId;

/**
 * @brief Tunable parameters for an asymmetric throttler loop.
 *
 * The loop is always asymmetric: it operates on the absolute error
 * @c (limit - value) and produces a per-tick frequency delta in MHz that
 * is added to the arbiter. The "asymmetry" is in three independent
 * dimensions:
 *
 *   1. Under- vs. over-limit gains. @c p_gain / @c i_gain / @c d_gain apply
 *      when @c value is under the limit, @c p_gain_over / @c i_gain_over /
 *      @c d_gain_over apply when @c value is over the limit.
 *   2. Under- vs. over-limit deadbands (@c deadband_under,
 *      @c deadband_over) so the loop is quiet near the limit and the two
 *      sides can have different sensitivities.
 *   3. Up- vs. down-slew caps (@c du_max_up, @c du_max_down) so the loop
 *      can climb fast and back off slowly (or vice versa).
 *
 * Within that framework PD / PI / PID is selected purely by which gains
 * are non-zero:
 *
 *   - Asymmetric PD : @c i_gain == 0 and @c i_gain_over == 0,
 *                     @c d_gain != 0 (or @c d_gain_over).
 *   - Asymmetric PI : @c i_gain != 0 (or @c i_gain_over), @c d_gain == 0
 *                     and @c d_gain_over == 0.
 *   - Asymmetric PID: any non-zero @c i_gain* and any non-zero @c d_gain*.
 */
typedef struct {
	float alpha_filter;     /**< IIR coefficient applied to the measurement. */

	/**
	 * Proportional gain used when @c value is under the limit (positive
	 * absolute error, outside the under-limit deadband).
	 *
	 * Units: MHz per source unit (e.g. MHz/W for TDP, MHz/degC for thermals).
	 */
	float p_gain;

	/**
	 * Integral gain used when @c value is under the limit. The loop
	 * integrates the absolute error each tick, but only when the error
	 * is outside the active deadband (so the integrator does not wind
	 * up while the loop is intentionally idle). The integrator state is
	 * clamped to a fraction of the current limit and is further
	 * back-calculated whenever the per-tick @c du output saturates
	 * against @c du_max_up or @c du_max_down.
	 *
	 * Set to 0 (default) to disable the integral action and recover pure
	 * asymmetric-PD behaviour on the under-limit side.
	 *
	 * Units: MHz / (source-unit * tick), e.g. MHz/(W*tick) for TDP.
	 */
	float i_gain;

	/**
	 * Derivative gain used when @c value is under the limit (positive
	 * absolute error, outside the under-limit deadband).
	 *
	 * Units: MHz * s per source unit.
	 */
	float d_gain;

	/** Proportional gain used when @c value is over the limit. */
	float p_gain_over;
	/**
	 * Integral gain used when @c value is over the limit. Same
	 * integrator state as @c i_gain (a single integrator is shared
	 * across the under- and over-limit branches); only the multiplier
	 * applied to that state changes when the loop crosses the limit.
	 * Set to 0 to disable integral action on the over-limit side.
	 *
	 * Units: MHz / (source-unit * tick).
	 */
	float i_gain_over;
	/** Derivative gain used when @c value is over the limit. */
	float d_gain_over;

	/**
	 * Deadband as a fraction of @c limit on the under-limit side. The
	 * loop only acts when @c (limit - value) > deadband_under * limit.
	 * Set to 0 to disable.
	 */
	float deadband_under;

	/**
	 * Deadband as a fraction of @c limit on the over-limit side. The
	 * loop only acts when @c (value - limit) > deadband_over * limit.
	 * Set to 0 to disable.
	 */
	float deadband_over;

	/**
	 * Maximum positive frequency step per tick in MHz. Set to 0 to
	 * disable the up-slew cap.
	 */
	float du_max_up;

	/**
	 * Maximum (negative) frequency step per tick in MHz. Must be <= 0.
	 * Set to 0 to disable the down-slew cap.
	 */
	float du_max_down;
} ThrottlerParams;

/**
 * @brief Identifies which PD parameter is being read or written via the
 *        @ref TT_SMC_MSG_THROTTLER_PD_PARAM host message.
 *
 * The numeric values are part of the host-facing interface and must not be
 * reordered.
 */
enum throttler_pd_param_id {
	THROTTLER_PD_PARAM_ALPHA_FILTER = 0,     /**< float */
	THROTTLER_PD_PARAM_P_GAIN = 1,           /**< float (under-limit proportional gain) */
	THROTTLER_PD_PARAM_D_GAIN = 2,           /**< float (under-limit derivative gain) */
	THROTTLER_PD_PARAM_P_GAIN_OVER = 3,      /**< float (over-limit proportional gain) */
	THROTTLER_PD_PARAM_D_GAIN_OVER = 4,      /**< float (over-limit derivative gain) */
	THROTTLER_PD_PARAM_DEADBAND_UNDER = 5,   /**< float, fraction of limit */
	THROTTLER_PD_PARAM_DEADBAND_OVER = 6,    /**< float, fraction of limit */
	THROTTLER_PD_PARAM_DU_MAX_UP = 7,        /**< float, MHz per tick (>= 0) */
	THROTTLER_PD_PARAM_DU_MAX_DOWN = 8,      /**< float, MHz per tick (<= 0) */
	THROTTLER_PD_PARAM_I_GAIN = 9,           /**< float (under-limit integral gain) */
	THROTTLER_PD_PARAM_I_GAIN_OVER = 10,     /**< float (over-limit integral gain) */
	THROTTLER_PD_PARAM_COUNT,
};

/** @brief Operations supported by the throttler PD param message. */
enum throttler_pd_param_op {
	THROTTLER_PD_PARAM_OP_GET = 0,
	THROTTLER_PD_PARAM_OP_SET = 1,
};

void InitThrottlers(void);
void CalculateThrottlers(void);
int32_t Dm2CmSetBoardPowerLimit(const uint8_t *data, uint8_t size);

/**
 * @brief Read a throttler's loop parameter.
 *
 * @param id        Throttler identifier.
 * @param param     Parameter identifier.
 * @param out_bits  Output: parameter value as a float bit-pattern.
 *
 * @return 0 on success, non-zero on invalid arguments.
 */
int throttler_get_pd_param(ThrottlerId id, enum throttler_pd_param_id param, uint32_t *out_bits);

/**
 * @brief Write a throttler's loop parameter.
 *
 * @param id        Throttler identifier.
 * @param param     Parameter identifier.
 * @param bits      Parameter value as a float bit-pattern.
 *
 * @return 0 on success, non-zero on invalid arguments.
 */
int throttler_set_pd_param(ThrottlerId id, enum throttler_pd_param_id param, uint32_t bits);

#endif
