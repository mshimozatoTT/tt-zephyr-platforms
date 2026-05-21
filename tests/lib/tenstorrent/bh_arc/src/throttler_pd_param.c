/*
 * Copyright (c) 2026 Tenstorrent AI ULC
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <string.h>

#include <zephyr/ztest.h>

#include "throttler.h"
#include <tenstorrent/msgqueue.h>
#include <tenstorrent/smc_msg.h>

static uint32_t float_to_bits(float v)
{
	uint32_t bits;

	memcpy(&bits, &v, sizeof(bits));
	return bits;
}

static float bits_to_float(uint32_t bits)
{
	float v;

	memcpy(&v, &bits, sizeof(v));
	return v;
}

static uint8_t send_pd_param_msg(uint8_t op, uint8_t throttler_id, uint8_t param_id,
				 uint32_t value, uint32_t *response_value)
{
	union request req = {0};
	struct response rsp = {0};

	req.throttler_pd_param.command_code = TT_SMC_MSG_THROTTLER_PD_PARAM;
	req.throttler_pd_param.op = op;
	req.throttler_pd_param.throttler_id = throttler_id;
	req.throttler_pd_param.param_id = param_id;
	req.throttler_pd_param.value = value;

	msgqueue_request_push(0, &req);
	process_message_queues();
	msgqueue_response_pop(0, &rsp);

	if (response_value != NULL) {
		*response_value = rsp.data[1];
	}
	return rsp.data[0];
}

static void *throttler_pd_setup(void)
{
	return NULL;
}

ZTEST(throttler_pd, test_set_get_p_gain_roundtrip)
{
	uint8_t status;
	uint32_t out_bits;

	status = send_pd_param_msg(THROTTLER_PD_PARAM_OP_SET, kThrottlerTDP,
				   THROTTLER_PD_PARAM_P_GAIN, float_to_bits(0.42f), NULL);
	zassert_equal(status, 0, "SET P_GAIN should succeed");

	status = send_pd_param_msg(THROTTLER_PD_PARAM_OP_GET, kThrottlerTDP,
				   THROTTLER_PD_PARAM_P_GAIN, 0, &out_bits);
	zassert_equal(status, 0, "GET P_GAIN should succeed");
	zassert_within(bits_to_float(out_bits), 0.42f, 1e-6f,
		       "P_GAIN round-trip should match (got %f)",
		       (double)bits_to_float(out_bits));
}

ZTEST(throttler_pd, test_set_get_i_gain_roundtrip)
{
	uint8_t status;
	uint32_t out_bits;

	status = send_pd_param_msg(THROTTLER_PD_PARAM_OP_SET, kThrottlerTDP,
				   THROTTLER_PD_PARAM_I_GAIN, float_to_bits(0.005f), NULL);
	zassert_equal(status, 0, "SET I_GAIN should succeed");

	status = send_pd_param_msg(THROTTLER_PD_PARAM_OP_GET, kThrottlerTDP,
				   THROTTLER_PD_PARAM_I_GAIN, 0, &out_bits);
	zassert_equal(status, 0, "GET I_GAIN should succeed");
	zassert_within(bits_to_float(out_bits), 0.005f, 1e-6f,
		       "I_GAIN round-trip should match (got %f)",
		       (double)bits_to_float(out_bits));
}

ZTEST(throttler_pd, test_set_invalid_alpha_filter_rejected)
{
	uint8_t status;

	status = send_pd_param_msg(THROTTLER_PD_PARAM_OP_SET, kThrottlerTDP,
				   THROTTLER_PD_PARAM_ALPHA_FILTER, float_to_bits(1.5f), NULL);
	zassert_not_equal(status, 0, "SET alpha_filter > 1 should be rejected");

	status = send_pd_param_msg(THROTTLER_PD_PARAM_OP_SET, kThrottlerTDP,
				   THROTTLER_PD_PARAM_ALPHA_FILTER, float_to_bits(-0.1f), NULL);
	zassert_not_equal(status, 0, "SET alpha_filter < 0 should be rejected");

	status = send_pd_param_msg(THROTTLER_PD_PARAM_OP_SET, kThrottlerTDP,
				   THROTTLER_PD_PARAM_ALPHA_FILTER, float_to_bits(0.25f), NULL);
	zassert_equal(status, 0, "SET alpha_filter in [0,1] should succeed");
}

ZTEST(throttler_pd, test_set_invalid_du_caps_rejected)
{
	uint8_t status;

	status = send_pd_param_msg(THROTTLER_PD_PARAM_OP_SET, kThrottlerTDP,
				   THROTTLER_PD_PARAM_DU_MAX_UP, float_to_bits(-1.0f), NULL);
	zassert_not_equal(status, 0, "du_max_up must be >= 0");

	status = send_pd_param_msg(THROTTLER_PD_PARAM_OP_SET, kThrottlerTDP,
				   THROTTLER_PD_PARAM_DU_MAX_DOWN, float_to_bits(1.0f), NULL);
	zassert_not_equal(status, 0, "du_max_down must be <= 0");

	status = send_pd_param_msg(THROTTLER_PD_PARAM_OP_SET, kThrottlerTDP,
				   THROTTLER_PD_PARAM_DU_MAX_UP, float_to_bits(50.0f), NULL);
	zassert_equal(status, 0, "du_max_up = 50 should succeed");

	status = send_pd_param_msg(THROTTLER_PD_PARAM_OP_SET, kThrottlerTDP,
				   THROTTLER_PD_PARAM_DU_MAX_DOWN, float_to_bits(-10.0f), NULL);
	zassert_equal(status, 0, "du_max_down = -10 should succeed");
}

ZTEST(throttler_pd, test_set_invalid_deadband_rejected)
{
	uint8_t status;

	status = send_pd_param_msg(THROTTLER_PD_PARAM_OP_SET, kThrottlerTDP,
				   THROTTLER_PD_PARAM_DEADBAND_UNDER, float_to_bits(-0.01f), NULL);
	zassert_not_equal(status, 0, "deadband_under must be >= 0");

	status = send_pd_param_msg(THROTTLER_PD_PARAM_OP_SET, kThrottlerTDP,
				   THROTTLER_PD_PARAM_DEADBAND_OVER, float_to_bits(1.0f), NULL);
	zassert_not_equal(status, 0, "deadband_over must be < 1");
}

ZTEST(throttler_pd, test_invalid_throttler_id_rejected)
{
	uint8_t status;

	status = send_pd_param_msg(THROTTLER_PD_PARAM_OP_GET, kThrottlerCount,
				   THROTTLER_PD_PARAM_P_GAIN, 0, NULL);
	zassert_not_equal(status, 0, "GET with out-of-range throttler id should fail");
}

ZTEST(throttler_pd, test_invalid_param_id_rejected)
{
	uint8_t status;

	status = send_pd_param_msg(THROTTLER_PD_PARAM_OP_GET, kThrottlerTDP,
				   THROTTLER_PD_PARAM_COUNT, 0, NULL);
	zassert_not_equal(status, 0, "GET with out-of-range param id should fail");

	status = send_pd_param_msg(THROTTLER_PD_PARAM_OP_GET, kThrottlerTDP, 0xFF, 0, NULL);
	zassert_not_equal(status, 0, "GET with unknown param id should fail");
}

ZTEST(throttler_pd, test_invalid_op_rejected)
{
	uint8_t status;

	status = send_pd_param_msg(0xFF, kThrottlerTDP, THROTTLER_PD_PARAM_P_GAIN, 0, NULL);
	zassert_not_equal(status, 0, "Unknown op should be rejected");
}

ZTEST(throttler_pd, test_direct_api_roundtrip)
{
	uint32_t bits = 0;
	int rc;

	rc = throttler_set_pd_param(kThrottlerTDP, THROTTLER_PD_PARAM_P_GAIN_OVER,
				    float_to_bits(0.123f));
	zassert_equal(rc, 0);

	rc = throttler_get_pd_param(kThrottlerTDP, THROTTLER_PD_PARAM_P_GAIN_OVER, &bits);
	zassert_equal(rc, 0);
	zassert_within(bits_to_float(bits), 0.123f, 1e-6f);
}

ZTEST_SUITE(throttler_pd, NULL, throttler_pd_setup, NULL, NULL, NULL);
