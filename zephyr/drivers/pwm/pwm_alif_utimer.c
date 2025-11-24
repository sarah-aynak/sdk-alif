/*
 * Copyright (c) 2024 Alif Semiconductor
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT alif_pwm

#include <errno.h>
#include <zephyr/device.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/drivers/pinctrl.h>

#include <utimer.h>
#include <zephyr/dt-bindings/timer/alif_utimer.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(pwm_alif_utimer, CONFIG_PWM_LOG_LEVEL);

/** PWM configuration */
struct pwm_alif_utimer_config {
	uint32_t global_base;
	uint32_t timer_base;
	uint32_t frequency;
	uint32_t counterdirection;
	uint32_t driver_type;
	const uint8_t timer;
	const struct pinctrl_dev_config *pcfg;
};

/** PWM runtime data configuration */
struct pwm_alif_utimer_data {
	uint32_t prev_period;
	uint32_t prev_pulse;
	pwm_flags_t prev_flags;
};

static void utimer_set_direction(uint32_t reg_base, uint8_t direction)
{
	switch (direction) {
	case ALIF_UTIMER_COUNTER_DIRECTION_UP:
		sys_clear_bit(UTIMER_CNTR_CTRL(reg_base), CNTR_CTRL_DIR_DOWN_BIT);
		break;
	case ALIF_UTIMER_COUNTER_DIRECTION_DOWN:
		sys_set_bit(UTIMER_CNTR_CTRL(reg_base), CNTR_CTRL_DIR_DOWN_BIT);
		break;
	case ALIF_UTIMER_COUNTER_DIRECTION_TRIANGLE:
		sys_write32(CNTR_CTRL_TRIANGLE_BUF_TROUGH_CREST,
				UTIMER_CNTR_CTRL(reg_base));
		break;
	default:
		LOG_ERR("invalid counter-direction");
	}
}

static void utimer_config_driver(uint32_t reg_base, uint8_t driver, uint32_t value)
{
	uint32_t temp;
	uint32_t mask = (COMPARE_CTRL_DRV_COMP_MATCH_Msk | COMPARE_CTRL_DRV_CYCLE_END_Msk);

	if (driver & ALIF_UTIMER_DRIVER_A_OUTPUT_ENABLE) {
		temp = sys_read32(UTIMER_COMPARE_CTRL_A(reg_base));
		temp &= ~mask;
		temp |= value;
		sys_write32(temp, UTIMER_COMPARE_CTRL_A(reg_base));
	}
	if (driver & ALIF_UTIMER_DRIVER_B_OUTPUT_ENABLE) {
		temp = sys_read32(UTIMER_COMPARE_CTRL_B(reg_base));
		temp &= ~mask;
		temp |= value;
		sys_write32(temp, UTIMER_COMPARE_CTRL_B(reg_base));
	}
}

static void utimer_enable_output(uint32_t reg_base, uint8_t driver, uint8_t timer)
{
	if (driver & ALIF_UTIMER_DRIVER_A_OUTPUT_ENABLE) {
		sys_clear_bit(UTIMER_GLB_DRIVER_OEN(reg_base), (timer * 2));
	}
	if (driver & ALIF_UTIMER_DRIVER_B_OUTPUT_ENABLE) {
		sys_clear_bit(UTIMER_GLB_DRIVER_OEN(reg_base), ((timer * 2) + 1));
	}
}

static void utimer_compare_disable_driver(uint32_t reg_base, uint8_t driver, pwm_flags_t pwm_flag)
{
	if (driver & ALIF_UTIMER_DRIVER_A_OUTPUT_ENABLE) {
		sys_clear_bit(UTIMER_COMPARE_CTRL_A(reg_base),
				COMPARE_CTRL_DRV_DRIVER_EN_BIT);
		if (pwm_flag == PWM_POLARITY_INVERTED) {
			sys_set_bit(UTIMER_COMPARE_CTRL_A(reg_base),
					COMPARE_CTRL_DRV_DISABLE_VAL_HIGH_BIT);
		}
	}
	if (driver & ALIF_UTIMER_DRIVER_B_OUTPUT_ENABLE) {
		sys_clear_bit(UTIMER_COMPARE_CTRL_B(reg_base),
				COMPARE_CTRL_DRV_DRIVER_EN_BIT);
		if (pwm_flag == PWM_POLARITY_INVERTED) {
			sys_set_bit(UTIMER_COMPARE_CTRL_B(reg_base),
					COMPARE_CTRL_DRV_DISABLE_VAL_HIGH_BIT);
		}
	}
}

static void utimer_compare_enable_driver(uint32_t reg_base, uint8_t driver)
{
	if (driver & ALIF_UTIMER_DRIVER_A_OUTPUT_ENABLE) {
		sys_set_bit(UTIMER_COMPARE_CTRL_A(reg_base),
				COMPARE_CTRL_DRV_DRIVER_EN_BIT);
			sys_clear_bit(UTIMER_COMPARE_CTRL_A(reg_base),
					COMPARE_CTRL_DRV_DISABLE_VAL_HIGH_BIT);
	}
	if (driver & ALIF_UTIMER_DRIVER_B_OUTPUT_ENABLE) {
		sys_set_bit(UTIMER_COMPARE_CTRL_B(reg_base),
				COMPARE_CTRL_DRV_DRIVER_EN_BIT);
			sys_clear_bit(UTIMER_COMPARE_CTRL_B(reg_base),
					COMPARE_CTRL_DRV_DISABLE_VAL_HIGH_BIT);
	}
}

static void utimer_update_pulse(uint32_t reg_base, uint8_t driver, uint32_t pulse_value)
{
	if (driver & ALIF_UTIMER_DRIVER_A_OUTPUT_ENABLE) {
		sys_write32(pulse_value, UTIMER_COMPARE_A(reg_base));
	}
	if (driver & ALIF_UTIMER_DRIVER_B_OUTPUT_ENABLE) {
		sys_write32(pulse_value, UTIMER_COMPARE_B(reg_base));
	}
}

static int pwm_alif_utimer_set_cycles(const struct device *dev, uint32_t channel,
			 uint32_t period_cycles,
			 uint32_t pulse_cycles, pwm_flags_t flags)
{
	const struct pwm_alif_utimer_config *cfg = dev->config;
	struct pwm_alif_utimer_data *data = dev->data;
	uint32_t timer_base = cfg->timer_base;
	uint32_t global_base = cfg->global_base;
	uint32_t value;

	ARG_UNUSED(channel);

	if (flags != data->prev_flags) {
		value = (flags & PWM_POLARITY_INVERTED) ?
			(COMPARE_CTRL_DRV_HIGH_AT_COMP_MATCH | COMPARE_CTRL_DRV_LOW_AT_CYCLE_END) :
			(COMPARE_CTRL_DRV_LOW_AT_COMP_MATCH | COMPARE_CTRL_DRV_HIGH_AT_CYCLE_END);
		utimer_config_driver(timer_base, cfg->driver_type, value);

		data->prev_flags = flags;
	}

	/* update period value if it as been changed */
	if (period_cycles != data->prev_period) {
		/* config driver output as zero if period is zero */
		if (period_cycles == 0U) {
			utimer_compare_disable_driver(timer_base, cfg->driver_type,
					data->prev_flags);
		} else {
			/* update the period value if it has changed and is not zero. */
			if (data->prev_period == 0U) {
				/* enable driver as previously it is disabled */
				utimer_compare_enable_driver(timer_base, cfg->driver_type);
			}
			/* set period value */
			sys_write32(period_cycles, UTIMER_CNTR_PTR(timer_base));
		}
		data->prev_period = period_cycles;
	}

	/* update pulse value if it as been changed */
	if (pulse_cycles != data->prev_pulse) {
		/* config driver output as zero if pulse is zero */
		if (pulse_cycles == 0U) {
			utimer_compare_disable_driver(timer_base, cfg->driver_type,
					data->prev_flags);
		} else {
			if (data->prev_pulse == 0U) {
				/* enable driver as previously it is disabled */
				utimer_compare_enable_driver(timer_base, cfg->driver_type);
			}
			/* set pulse value */
			utimer_update_pulse(timer_base, cfg->driver_type, pulse_cycles);
		}
		data->prev_pulse = pulse_cycles;
	}

	/* start the counter it is not already running */
	if (!(sys_test_bit(UTIMER_GLB_CNTR_RUNNING(global_base), cfg->timer))) {
		sys_set_bit(UTIMER_GLB_CNTR_START(global_base), cfg->timer);
	}

	return 0;
}

static int pwm_alif_utimer_get_cycles_per_sec(const struct device *dev, uint32_t channel,
				 uint64_t *cycles)
{
	const struct pwm_alif_utimer_config *cfg = dev->config;

	ARG_UNUSED(channel);

	*cycles = cfg->frequency;
	return 0;
}

static const struct pwm_driver_api pwm_alif_utimer_driver_api = {
	.set_cycles = pwm_alif_utimer_set_cycles,
	.get_cycles_per_sec = pwm_alif_utimer_get_cycles_per_sec,
};

static int pwm_alif_utimer_init(const struct device *dev)
{
	const struct pwm_alif_utimer_config *cfg = dev->config;
	struct pwm_alif_utimer_data *data = dev->data;
	uint32_t timer_base = cfg->timer_base;
	uint32_t global_base = cfg->global_base;
	int32_t ret;
	uint32_t reg;

	/* apply pin configuration */
	ret = pinctrl_apply_state(cfg->pcfg, PINCTRL_STATE_DEFAULT);
	if (ret < 0) {
		return ret;
	}

	if ((cfg->driver_type < ALIF_UTIMER_DRIVER_A_OUTPUT_ENABLE) ||
			(cfg->driver_type > ALIF_UTIMER_DRIVER_B_OUTPUT_ENABLE)) {
		LOG_ERR("ERROR: Invalid output driver");
	}

	/* enable timer clock */
	sys_set_bit(UTIMER_GLB_CLOCK_ENABLE(global_base), cfg->timer);

	/* program enable for start, stop and clear counter */
	sys_set_bit(UTIMER_START_1_SRC(timer_base), CNTR_SRC1_PGM_EN_BIT);
	sys_set_bit(UTIMER_STOP_1_SRC(timer_base), CNTR_SRC1_PGM_EN_BIT);
	sys_set_bit(UTIMER_CLEAR_1_SRC(timer_base), CNTR_SRC1_PGM_EN_BIT);

	/* disable all buffer operations */
	sys_write32(0x0, UTIMER_BUF_OP_CTRL(timer_base));

	/* set counter direction */
	utimer_set_direction(timer_base, cfg->counterdirection);

	/* enable driver output and default driver settings */
	reg = (COMPARE_CTRL_DRV_COMPARE_EN | COMPARE_CTRL_DRV_DRIVER_EN |
		   (COMPARE_CTRL_DRV_LOW_AT_COMP_MATCH & COMPARE_CTRL_DRV_COMP_MATCH_Msk) |
		   (COMPARE_CTRL_DRV_HIGH_AT_CYCLE_END & COMPARE_CTRL_DRV_CYCLE_END_Msk));

	utimer_config_driver(timer_base, cfg->driver_type, reg);
	utimer_enable_output(global_base, cfg->driver_type, cfg->timer);
	data->prev_flags = PWM_POLARITY_NORMAL;

	/* enable timer counter */
	sys_set_bit(UTIMER_CNTR_CTRL(timer_base), CNTR_CTRL_EN_BIT);

	return 0;
}

/* Device Instantiation */
#define PWM_ALIF_UTIMER_INIT(n)									\
	PINCTRL_DT_INST_DEFINE(n);								\
	static struct pwm_alif_utimer_data pwm_alif_utimer_data_##n;				\
	static const struct pwm_alif_utimer_config pwm_alif_utimer_cfg_##n = {			\
		.global_base = (uint32_t) DT_REG_ADDR_BY_NAME(DT_INST_PARENT(n), global),	\
		.timer_base = (uint32_t) DT_REG_ADDR_BY_NAME(DT_INST_PARENT(n), timer),		\
		.frequency = DT_PROP(DT_INST_PARENT(n), clock_frequency),			\
		.counterdirection = DT_PROP(DT_INST_PARENT(n), counter_direction),		\
		.driver_type = DT_PROP(DT_INST_PARENT(n), driver_enable),			\
		.timer = DT_PROP(DT_INST_PARENT(n), timer_id),					\
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n),					\
	};											\
												\
	DEVICE_DT_INST_DEFINE(n,								\
				&pwm_alif_utimer_init,						\
				NULL,								\
				&pwm_alif_utimer_data_##n,					\
				&pwm_alif_utimer_cfg_##n,					\
				POST_KERNEL,							\
				CONFIG_PWM_INIT_PRIORITY,					\
				&pwm_alif_utimer_driver_api);

DT_INST_FOREACH_STATUS_OKAY(PWM_ALIF_UTIMER_INIT)

