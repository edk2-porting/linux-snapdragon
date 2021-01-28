// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2021, Caleb Connolly <caleb@connolly.tech>
 * 
 * Based on ./pm8xxx_vibrator.c
 * 
 * For implementation details and some rough hardware docs see: https://www.notion.so/calebccff/pmi8998-QPNP-Qcom-Plug-n-Play-haptics-9ccc9240c0f5498f8eb5feb2ff1059e5
 */

/*
 * TODO: Add suport for wave shape, play mode, brake?
 */

#include <dt-bindings/input/qcom,pmi8998-haptics.h>

#include <linux/atomic.h>
#include <linux/errno.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/slab.h>
#include <linux/log2.h>
#include <linux/bits.h>
#include <linux/time.h>

/*
 * Register and bit definitions
 */

#define HAP_STATUS_1_REG(chip)		(chip->base + 0x0A)
#define HAP_BUSY_BIT				BIT(1)
#define SC_FLAG_BIT					BIT(3)
#define AUTO_RES_ERROR_BIT			BIT(4)

#define HAP_LRA_AUTO_RES_LO_REG(chip)	(chip->base + 0x0B)
#define HAP_LRA_AUTO_RES_HI_REG(chip)	(chip->base + 0x0C)

#define HAP_EN_CTL_REG(chip)		(chip->base + 0x46)
#define HAP_EN_BIT			BIT(7)

#define HAP_EN_CTL2_REG(chip)		(chip->base + 0x48)
#define BRAKE_EN_BIT				BIT(0)

#define HAP_AUTO_RES_CTRL_REG(chip)	(chip->base + 0x4B)
#define AUTO_RES_EN_BIT			BIT(7)
#define AUTO_RES_ERR_RECOVERY_BIT	BIT(3)
#define AUTO_RES_EN_FLAG_BIT	BIT(0)

#define HAP_CFG1_REG(chip)			(chip->base + 0x4C)
#define HAP_ACT_TYPE_MASK			BIT(0)

#define HAP_CFG2_REG(chip)		(chip->base + 0x4D)
#define HAP_LRA_RES_TYPE_MASK		BIT(0)

#define HAP_SEL_REG(chip)			(chip->base + 0x4E)
#define HAP_WF_SOURCE_MASK			GENMASK(5, 4)
#define HAP_WF_SOURCE_SHIFT			4

#define HAP_LRA_AUTO_RES_REG(chip)	(chip->base + 0x4F)
#define LRA_AUTO_RES_MODE_MASK		GENMASK(6, 4)
#define LRA_AUTO_RES_MODE_SHIFT		4
#define LRA_HIGH_Z_MASK				GENMASK(3, 2)
#define LRA_HIGH_Z_SHIFT			2
#define LRA_RES_CAL_MASK			GENMASK(1, 0)
#define HAP_RES_CAL_PERIOD_MIN		4
#define HAP_RES_CAL_PERIOD_MAX		32

#define HAP_VMAX_CFG_REG(chip)		(chip->base + 0x51)
#define HAP_VMAX_OVD_BIT			BIT(6)
#define HAP_VMAX_MASK				GENMASK(5, 1)
#define HAP_VMAX_SHIFT				1

#define HAP_ILIM_CFG_REG(chip)		(chip->base + 0x52)
#define HAP_ILIM_SEL_MASK			BIT(0)
#define HAP_ILIM_400_MA				0
#define HAP_ILIM_800_MA				1


#define HAP_SC_DEB_REG(chip)		(chip->base + 0x53)
#define HAP_SC_DEB_MASK				GENMASK(2, 0)
#define HAP_SC_DEB_CYCLES_MIN		0
#define HAP_DEF_SC_DEB_CYCLES		8
#define HAP_SC_DEB_CYCLES_MAX		32

#define HAP_RATE_CFG1_REG(chip)		(chip->base + 0x54)
#define HAP_RATE_CFG1_MASK		GENMASK(7, 0)
#define HAP_RATE_CFG2_SHIFT		8 // As CFG2 is the most significant byte

#define HAP_RATE_CFG2_REG(chip)		(chip->base + 0x55)
#define HAP_RATE_CFG2_MASK		GENMASK(3, 0)

#define HAP_SC_CLR_REG(chip)		(chip->base + 0x59)
#define SC_CLR_BIT			BIT(0)

#define HAP_BRAKE_REG(chip)		(chip->base + 0x5C)
#define HAP_BRAKE_PAT_MASK		0x3

#define HAP_WF_REPEAT_REG(chip)		(chip->base + 0x5E)
#define WF_REPEAT_MASK			GENMASK(6, 4)
#define WF_REPEAT_SHIFT			4
#define WF_REPEAT_MIN			1
#define WF_REPEAT_MAX			128
#define WF_S_REPEAT_MASK		GENMASK(1, 0)
#define WF_S_REPEAT_MIN			1
#define WF_S_REPEAT_MAX			8

#define HAP_WF_S1_REG(chip)		(chip->base + 0x60)
#define HAP_WF_SIGN_BIT			BIT(7)
#define HAP_WF_OVD_BIT			BIT(6)
#define HAP_WF_SAMP_MAX			GENMASK(5, 1)
#define HAP_WF_SAMPLE_LEN		8

#define HAP_PLAY_REG(chip)		(chip->base + 0x70)
#define PLAY_BIT			BIT(7)
#define PAUSE_BIT			BIT(0)

#define HAP_SEC_ACCESS_REG(chip)	(chip->base + 0xD0)
#define HAP_SEC_ACCESS_UNLOCK		0xA5

// For pmi8998 AUTO_RES_EN_BIT is written here to enable auto resonance
// mode, what the h*ck
#define HAP_TEST2_REG(chip)		(chip->base + 0xE3)

// Values
#define HAP_VMAX_MIN_MV				116
#define HAP_VMAX_MAX_MV				3596
#define HAP_VMAX_MAX_MV_STRONG		3596

#define HAP_WAVE_PLAY_RATE_DEF_US	5715
#define HAP_WAVE_PLAY_RATE_MIN_US	0
#define HAP_WAVE_PLAY_RATE_MAX_US	20475
#define HAP_WAVE_PLAY_TIME_MAX_MS	15000

#define AUTO_RES_ERR_POLL_TIME_NS	(20 * NSEC_PER_MSEC)
#define HAPTICS_BACK_EMF_DELAY_US	20000

#define HAP_BRAKE_PAT_LEN			4
#define HAP_WAVE_SAMP_LEN			8
#define NUM_WF_SET					4
#define HAP_WAVE_SAMP_SET_LEN		(HAP_WAVE_SAMP_LEN * NUM_WF_SET)
#define HAP_RATE_CFG_STEP_US		5

#define SC_MAX_COUNT		5
#define SC_COUNT_RST_DELAY_US	1000000

enum hap_play_control {
	HAP_STOP,
	HAP_PAUSE,
	HAP_PLAY,
};

/**
 * struct pmi8998_haptics - struct for qpnp haptics data.
 */
struct pmi8998_haptics {
	struct platform_device *pdev;
	struct regmap *regmap;
	struct input_dev *haptics_input_dev;
	struct work_struct work;
	u16 base;

	atomic_t active;
	bool auto_res_enabled;
	u16 drive_period_code_max_limit;
	u16 drive_period_code_min_limit;

	int play_irq;
	int sc_irq;
	ktime_t last_sc_time;
	u8 sc_count;

	u8 actuator_type;
	u8 wave_shape;
	u8 play_mode;
	u32 last_play_rate;
	int magnitude;
	u32 vmax;
	u32 current_limit;
	u32 play_wave_rate;
	u8 wave_samp_idx;

	u32 wave_samp[HAP_WAVE_SAMP_SET_LEN];
	u32 brake_pat[HAP_BRAKE_PAT_LEN];

	struct mutex play_lock;
};

/**
 * constrain() - Constrain a value to a range
 * @val: The value to constrain
 * @min: The minimum allowed value
 * @max: The maximum allowed value
 */
static inline u32 constrain(u32 val, u32 min, u32 max) {
	return val < min ? min
		 : (val > max ? max
		 : val);
}

static inline bool is_secure_addr(u16 addr)
{
	return (addr & 0xFF) > 0xD0;
}

static int pmi8998_haptics_read(struct pmi8998_haptics *haptics, u16 addr, u8 *val, int len)
{
	int ret;

	ret = regmap_bulk_read(haptics->regmap, addr, val, len);
	if (ret < 0)
		pr_err("Error reading address: 0x%x, ret %d\n", addr, ret);
	
	pr_debug("%s: read 0x%x from 0x%x", __func__, *val, addr);

	return ret;
}

static int pmi8998_haptics_write(struct pmi8998_haptics *haptics, u16 addr, u8 *val, int len)
{
	int ret, i;

	if (is_secure_addr(addr)) {
		for (i = 0; i < len; i++) {
			pr_info("%s: unlocking for addr: 0x%x, val: 0x%x", __func__, addr, val[i]);
			ret = regmap_write(haptics->regmap,
				HAP_SEC_ACCESS_REG(haptics), HAP_SEC_ACCESS_UNLOCK);
			if (ret < 0) {
				pr_err("Error writing unlock code, ret %d\n",
					ret);
				return ret;
			}

			ret = regmap_write(haptics->regmap, addr + i, val[i]);
			if (ret < 0) {
				pr_err("Error writing address 0x%x, ret %d\n",
					addr + i, ret);
				return ret;
			}
		}
	} else {
		if (len > 1)
			ret = regmap_bulk_write(haptics->regmap, addr, val, len);
		else
			ret = regmap_write(haptics->regmap, addr, *val);
	}

	if (ret < 0)
		pr_err("%s: Error writing address: 0x%x, ret %d\n", __func__, addr, ret);

	return ret;
}

static int pmi8998_haptics_write_masked(struct pmi8998_haptics *haptics, u16 addr, u8 mask, u8 val)
{
	int ret;

	if (is_secure_addr(addr)) {
		ret = regmap_write(haptics->regmap,
			HAP_SEC_ACCESS_REG(haptics), HAP_SEC_ACCESS_UNLOCK);
		if (ret < 0) {
			pr_err("Error writing unlock code - ret %d\n", ret);
			return ret;
		}
	}

	ret = regmap_update_bits(haptics->regmap, addr, mask, val);
	if (ret < 0)
		pr_err("Error writing address: 0x%x - ret %d\n", addr, ret);
	
	return ret;
}

static bool is_haptics_idle(struct pmi8998_haptics *haptics)
{
	int ret;
	u8 val;

	if (haptics->play_mode == HAP_PLAY_DIRECT ||
			haptics->play_mode == HAP_PLAY_PWM)
		return true;

	ret = pmi8998_haptics_read(haptics, HAP_STATUS_1_REG(haptics), &val, 1);
	if (ret < 0 || (val & HAP_BUSY_BIT))
		return false;

	return true;
}

static int pmi8998_haptics_module_enable(struct pmi8998_haptics *haptics, bool enable)
{
	u8 val;
	int rc;

	pr_debug("pmi8998_haptics: setting module enable: %d", enable);

	if (!enable) {
		if (!is_haptics_idle(haptics))
			pr_debug("Disabling module forcibly\n");
	}

	val = enable ? HAP_EN_BIT : 0;
	rc = pmi8998_haptics_write(haptics, HAP_EN_CTL_REG(haptics), &val, 1);
	if (rc < 0)
		return rc;

	return 0;
}

/* configuration api for max voltage */
static int pmi8998_haptics_write_vmax(struct pmi8998_haptics *haptics)
{
	u8 val = 0;
	int ret;
	u32 vmax_mv = haptics->vmax;

	pr_debug("Setting vmax to: %d", haptics->vmax);

	vmax_mv = constrain(vmax_mv, HAP_VMAX_MIN_MV, HAP_VMAX_MAX_MV);

	val = DIV_ROUND_CLOSEST(vmax_mv, HAP_VMAX_MIN_MV);
	val <<= HAP_VMAX_SHIFT;
	// overdrive only supported on pm660 apparently
	val &= ~HAP_VMAX_OVD_BIT;

	ret = pmi8998_haptics_write_masked(haptics, HAP_VMAX_CFG_REG(haptics),
			HAP_VMAX_MASK | HAP_VMAX_OVD_BIT, val);
	return ret;
}

/* configuration api for ilim */
static int pmi8998_haptics_write_current_limit(struct pmi8998_haptics *haptics)
{
	int ret;

	haptics->current_limit = constrain(haptics->current_limit, HAP_ILIM_400_MA, HAP_ILIM_800_MA);

	pr_debug("Setting current_limit to: 0x%x", haptics->current_limit);

	ret = pmi8998_haptics_write_masked(haptics, HAP_ILIM_CFG_REG(haptics),
			HAP_ILIM_SEL_MASK, haptics->current_limit);
	return ret;
}

/* configuration api for play mode */
static int pmi8998_haptics_write_play_mode(struct pmi8998_haptics *haptics)
{
	u8 val = 0;
	int ret;

	if (!is_haptics_idle(haptics))
		return -EBUSY;

	pr_debug("Setting play_mode to: 0x%x", haptics->play_mode);

	val = haptics->play_mode << HAP_WF_SOURCE_SHIFT;
	ret = pmi8998_haptics_write_masked(haptics, HAP_SEL_REG(haptics),
			HAP_WF_SOURCE_MASK, val);

	return ret;
}

static int pmi8998_haptics_write_play_rate(struct pmi8998_haptics *haptics, u16 play_rate)
{
	int rc;
	u8 val[2];

	pr_debug("Setting play_rate to: %d", play_rate);

	if (haptics->last_play_rate == play_rate) {
		pr_debug("Same rate_cfg %x\n", play_rate);
		return 0;
	}

	val[0] = play_rate & HAP_RATE_CFG1_MASK;
	val[1] = (play_rate >> HAP_RATE_CFG2_SHIFT) & HAP_RATE_CFG2_MASK;
	rc = pmi8998_haptics_write(haptics, HAP_RATE_CFG1_REG(haptics), val, 2); // We can write both at once
	if (rc < 0)
		return rc;

	haptics->last_play_rate = play_rate;
	return 0;
}

/*
 * pmi8998_haptics_set_auto_res() - Auto resonance
 * allows the haptics to automatically adjust the
 * speed of the oscillation in order to maintain
 * the resonant frequency.
 */
static int pmi8998_haptics_set_auto_res(struct pmi8998_haptics *haptics, bool enable)
{
	int rc = 0;
	u8 val;

	// LRAs are the only type to support auto res
	if (haptics->actuator_type != HAP_TYPE_LRA)
		return 0;

	val = enable ? AUTO_RES_EN_BIT : 0;

	rc = pmi8998_haptics_write_masked(haptics, HAP_TEST2_REG(haptics),
			AUTO_RES_EN_BIT, val);
	if (rc < 0)
		return rc;

	haptics->auto_res_enabled = enable;

	pr_debug("auto_res enabled: %d", enable);
	return rc;
}

/*
 * Write the configured frequency to the device
 */
static void pmi8998_haptics_lra_freq_write(struct pmi8998_haptics *haptics)
{
	u8 lra_auto_res[2], val;
	u32 play_rate_code;
	u16 rate_cfg;
	int rc;

	// First we read the auto resonance value
	rc = pmi8998_haptics_read(haptics, HAP_LRA_AUTO_RES_LO_REG(haptics),
				lra_auto_res, 2);
	if (rc < 0) {
		pr_err("Error in reading LRA_AUTO_RES_LO/HI, rc=%d\n", rc);
		return;
	}

	play_rate_code =
		 (lra_auto_res[1] & 0xF0) << 4 | (lra_auto_res[0] & 0xFF);

	pr_debug("lra_auto_res_lo = 0x%x lra_auto_res_hi = 0x%x play_rate_code = 0x%x\n",
		lra_auto_res[0], lra_auto_res[1], play_rate_code);

	rc = pmi8998_haptics_read(haptics, HAP_STATUS_1_REG(haptics), &val, 1);
	if (rc < 0)
		return;

	/*
	 * If the drive period code read from AUTO_RES_LO and AUTO_RES_HI
	 * registers is more than the max limit percent variation or less
	 * than the min limit percent variation specified through DT, then
	 * auto-resonance is disabled.
	 */

	if ((val & AUTO_RES_ERROR_BIT) ||
		((play_rate_code <= haptics->drive_period_code_min_limit) ||
		(play_rate_code >= haptics->drive_period_code_max_limit))) {
		if (val & AUTO_RES_ERROR_BIT)
			pr_err("Auto-resonance error %x\n", val);
		else
			pr_debug("play rate %x out of bounds [min: 0x%x, max: 0x%x]\n",
				play_rate_code,
				haptics->drive_period_code_min_limit,
				haptics->drive_period_code_max_limit);
		rc = pmi8998_haptics_set_auto_res(haptics, false);
		if (rc < 0)
			pr_err("Auto-resonance disable failed\n");
		return;
	}

	/*
	 * bits[7:4] of AUTO_RES_HI should be written to bits[3:0] of RATE_CFG2
	 */
	lra_auto_res[1] >>= 4;
	rate_cfg = lra_auto_res[1] << 8 | lra_auto_res[0];
	rc = pmi8998_haptics_write_play_rate(haptics, rate_cfg);
	if (rc < 0)
		pr_err("Error in updating rate_cfg\n");
}

/*
 * Write the brake pattern.
 */
static int pmi8998_haptics_write_brake(struct pmi8998_haptics *haptics)
{
	int ret, i;
	u32 temp, *ptr;
	u8 val;

	pr_debug("configuring brake pattern");

	/* Configure BRAKE register */
	ret = pmi8998_haptics_write_masked(haptics, HAP_EN_CTL2_REG(haptics),
			BRAKE_EN_BIT, 1);
	if (ret < 0)
		return ret;

	ptr = haptics->brake_pat;

	for (i = HAP_BRAKE_PAT_LEN - 1, val = 0; i >= 0; i--) {
		ptr[i] &= HAP_BRAKE_PAT_MASK;
		temp = i << 1;
		val |= ptr[i] << temp;
	}

	ret = pmi8998_haptics_write(haptics, HAP_BRAKE_REG(haptics), &val, 1);
	if (ret < 0)
		return ret;

	return 0;
}

/* configuration api for buffer mode */
static int pmi8998_haptics_write_buffer_config(struct pmi8998_haptics *haptics)
{
	u8 buf[HAP_WAVE_SAMP_LEN];
	int rc, i;

	pr_debug("Writing buffer config");

	if (haptics->wave_samp_idx >= ARRAY_SIZE(haptics->wave_samp)) {
		pr_err("Incorrect wave_samp_idx %d\n",
			haptics->wave_samp_idx);
		return -EINVAL;
	}

	for (i = 0; i < HAP_WAVE_SAMP_LEN; i++) {
		buf[i] = haptics->wave_samp[i];
	}

	rc = pmi8998_haptics_write(haptics, HAP_WF_S1_REG(haptics), buf,
			HAP_WAVE_SAMP_LEN);

	return rc;
}

// configuration api for haptics wave repeat
/**
 * pmi8998_haptics_write_wave_repeat() - write wave repeat values. A false
 * 	value means that it WONT be written.
 * @wave_repeat: Write the number of times to repeat each wave (8 samples)
 * @wave_sample_repeat: Write the number of times to repeat each wave sample
 */
static int pmi8998_haptics_write_wave_repeat(struct pmi8998_haptics *haptics,
					bool wave_repeat, bool wave_sample_repeat)
{
	int ret;
	u8 val = 0, mask = 0;

	if (wave_repeat) {
		mask = WF_REPEAT_MASK;
		val = ilog2(1) << WF_REPEAT_SHIFT; // Currently hard coded to default of 1
	}

	if (wave_sample_repeat) {
		mask |= WF_S_REPEAT_MASK;
		val |= ilog2(1);
	}

	ret = pmi8998_haptics_write_masked(haptics, HAP_WF_REPEAT_REG(haptics),
			mask, val);
	return ret;
}

static int pmi8998_haptics_write_play_control(struct pmi8998_haptics *haptics,
					enum hap_play_control ctrl)
{
	u8 val;
	int rc;

	switch (ctrl) {
	case HAP_STOP:
		val = 0;
		break;
	case HAP_PAUSE:
		val = PAUSE_BIT;
		break;
	case HAP_PLAY:
		val = PLAY_BIT;
		break;
	default:
		return 0;
	}

	rc = pmi8998_haptics_write(haptics, HAP_PLAY_REG(haptics), &val, 1);
	if (rc < 0) {
		pr_err("Error in writing to PLAY_REG, rc=%d\n", rc);
		return rc;
	}

	pr_debug("haptics play ctrl: %d\n", ctrl);
	return rc;
}

/*
 * This IRQ is used to load the next wave sample set.
 * As we only currently support a single sample set, it's unused.
 */
static irqreturn_t pmi8998_haptics_play_irq_handler(int irq, void *data) {
	pr_debug("pmi8998_haptics: play_irq triggered");

	return IRQ_HANDLED;
}

/*
 * Fires every ~50ms whilst the haptics are active.
 * If the SC_FLAG_BIT is set then that means there isn't a short circuit
 * and we just need to clear the IRQ to indicate that the device should
 * keep vibrating.
 *
 * Otherwise, it means a short circuit situation has occured.
 * 
 * Vibration will stop if we don't clear the flag bit, we act as a watchdog
 * in that sense.
 */
static irqreturn_t pmi8998_haptics_sc_irq_handler(int irq, void *data) {
	struct pmi8998_haptics *haptics = data;
	int ret;
	u8 val;
	s64 sc_delta_time_us;
	ktime_t temp;

	ret = pmi8998_haptics_read(haptics, HAP_STATUS_1_REG(haptics), &val, 1);
	if (ret < 0)
		goto irq_handled;

	if (!(val & SC_FLAG_BIT)) {
		haptics->sc_count = 0;
		goto irq_handled;
	}

	temp = ktime_get();
	sc_delta_time_us = ktime_us_delta(temp, haptics->last_sc_time);
	haptics->last_sc_time = temp;

	if (sc_delta_time_us > SC_COUNT_RST_DELAY_US)
		haptics->sc_count = 0;
	else
		haptics->sc_count++;

	// Clear the interrupt flag
	val = SC_CLR_BIT;
	ret = pmi8998_haptics_write(haptics, HAP_SC_CLR_REG(haptics), &val, 1);
	if (ret < 0) {
		pr_err("Error in writing to SC_CLR_REG, rc=%d\n", ret);
		goto irq_handled;
	}

	/* Permanently disable module if SC condition persists */
	if (haptics->sc_count > SC_MAX_COUNT) {
		pr_crit("SC persists, permanently disabling haptics\n");
		ret = pmi8998_haptics_module_enable(haptics, false);
		if (ret < 0) {
			pr_err("Error in disabling module, rc=%d\n", ret);
			goto irq_handled;
		}
	}

irq_handled:
	return IRQ_HANDLED;
}


/************************************************************
 * pmi8998_haptics_init() - Initialise haptics hardware for use
 * @haptics: haptics device
 */
static int pmi8998_haptics_init(struct pmi8998_haptics *haptics) {
	int ret;
	u8 val, mask;
	u16 lra_res_cal_period, auto_res_mode;
	u16 play_rate = 0;

	ret = pmi8998_haptics_write_masked(haptics, HAP_CFG1_REG(haptics),
		HAP_ACT_TYPE_MASK, haptics->actuator_type);
	if (ret < 0)
		return ret;
	
	// Configure auto resonance
	// see qpnp_haptics_lra_auto_res_config downstream
	lra_res_cal_period = 32; // Hard coded default
	auto_res_mode = HAP_AUTO_RES_ZXD_EOP << LRA_AUTO_RES_MODE_SHIFT; // oneplus 6 default
	
	val = ilog2(lra_res_cal_period / HAP_RES_CAL_PERIOD_MIN); // For sdm660 add 1 here

	val |= (auto_res_mode << LRA_AUTO_RES_MODE_SHIFT);
	// The 1 here is for OPT1, there are 3 options and no documentation
	// indicating the difference
	val |= (1 << LRA_HIGH_Z_SHIFT);
	mask = LRA_AUTO_RES_MODE_MASK | LRA_HIGH_Z_MASK | LRA_RES_CAL_MASK;

	ret = pmi8998_haptics_write_masked(haptics, HAP_LRA_AUTO_RES_REG(haptics),
			mask, val);

	pr_debug("%s: mode: %d hi_z period: %d cal_period: %d\n", __func__,
		auto_res_mode, 1,
		lra_res_cal_period);
	
	/* Configure the PLAY MODE register */
	ret = pmi8998_haptics_write_play_mode(haptics);
	if (ret < 0)
		return ret;

	ret = pmi8998_haptics_write_vmax(haptics);
	if (ret < 0)
		return ret;

	/* Configure the ILIM register */
	ret = pmi8998_haptics_write_current_limit(haptics);
	if (ret < 0)
		return ret;

	// Configure the debounce for the short-circuit
	// detection
	val = HAP_SC_DEB_CYCLES_MAX;
	ret = pmi8998_haptics_write_masked(haptics, HAP_SC_DEB_REG(haptics),
			HAP_SC_DEB_MASK, HAP_SC_DEB_CYCLES_MAX);
	if (ret < 0)
		return ret;

	// write the wave shape
	ret = pmi8998_haptics_write_masked(haptics, HAP_CFG2_REG(haptics),
			HAP_LRA_RES_TYPE_MASK, haptics->wave_shape);
	if (ret < 0)
		return ret;

	// The play rate is the wave_rate / cycles per wave
	play_rate = haptics->play_wave_rate / HAP_RATE_CFG_STEP_US;

	/*
	 * Configure RATE_CFG1 and RATE_CFG2 registers.
	 * Note: For ERM these registers act as play rate and
	 * for LRA these represent resonance period
	 */
	ret = pmi8998_haptics_write_play_rate(haptics, play_rate);
	if (haptics->actuator_type == HAP_TYPE_LRA) {
		haptics->drive_period_code_max_limit = (play_rate * (100 + 5)) / 100;
		haptics->drive_period_code_min_limit = (play_rate * (100 - 5)) / 100;

		pr_debug("Drive period code max limit %x min limit %x\n",
			haptics->drive_period_code_max_limit,
			haptics->drive_period_code_min_limit);
	}

	ret = pmi8998_haptics_write_brake(haptics);
	if (ret < 0)
		return ret;

	if (haptics->play_mode == HAP_PLAY_BUFFER) {
		ret = pmi8998_haptics_write_wave_repeat(haptics, true, true);
		if (ret < 0)
			return ret;

		ret = pmi8998_haptics_write_buffer_config(haptics);
	}

	/* setup play irq */
	if (haptics->play_irq >= 0) {
		pr_debug("%s: Requesting play IRQ, dev pointer: %p, irq: %d", __func__,
			haptics->pdev->dev, haptics->play_irq);
		ret = devm_request_threaded_irq(&haptics->pdev->dev, haptics->play_irq,
			NULL, pmi8998_haptics_play_irq_handler, IRQF_ONESHOT,
			"haptics_play_irq", haptics);

		if (ret < 0) {
			pr_err("Unable to request play(%d) IRQ(err:%d)\n",
				haptics->play_irq, ret);
			return ret;
		}

		//haptics->play_irq_en = true;
		/* use play_irq only for buffer mode */
		if (true || haptics->play_mode != HAP_PLAY_BUFFER) {
			disable_irq(haptics->play_irq);
			//haptics->play_irq_en = false;
		}
	}

	/* setup short circuit1 irq */
	if (haptics->sc_irq >= 0) {
		pr_debug("%s: Requesting play IRQ, dev pointer: %p, irq: %d", __func__,
			haptics->pdev->dev, haptics->play_irq);
		ret = devm_request_threaded_irq(&haptics->pdev->dev, haptics->sc_irq,
			NULL, pmi8998_haptics_sc_irq_handler, IRQF_ONESHOT,
			"haptics_sc_irq", haptics);

		if (ret < 0) {
			pr_err("Unable to request sc(%d) IRQ(err:%d)\n",
				haptics->sc_irq, ret);
			return ret;
		}
	}

	return ret;
}

/**
 * pmi8998_haptics_set - handler to start/stop vibration
 * @haptics: pointer to vibrator structure
 * @on: state to set
 */
static int pmi8998_haptics_set(struct pmi8998_haptics *haptics, bool enable)
{
	int ret;

	mutex_lock(&haptics->play_lock);

	if (enable) {
		// Are we in a short circuit state
		if (haptics->sc_count > SC_MAX_COUNT) {
			pr_err("Can't play while in short circuit");
			ret = -1;
			goto out;
		}
		ret = pmi8998_haptics_set_auto_res(haptics, false);
		if (ret < 0) {
			pr_err("Error in disabling auto_res, ret=%d\n", ret);
			goto out;
		}

		ret = pmi8998_haptics_module_enable(haptics, true);
		if (ret < 0) {
			pr_err("Error in enabling module, ret=%d\n", ret);
			goto out;
		}

		ret = pmi8998_haptics_write_play_control(haptics, HAP_PLAY);
		if (ret < 0) {
			pr_err("Error in enabling play, ret=%d\n", ret);
			goto out;
		}
		
		ret = pmi8998_haptics_set_auto_res(haptics, true);
		if (ret < 0) {
			pr_err("Error in enabling auto_res, ret=%d\n", ret);
			goto out;
		}
	} else {
		ret = pmi8998_haptics_write_play_control(haptics, HAP_STOP);
		if (ret < 0) {
			pr_err("Error in disabling play, ret=%d\n", ret);
			goto out;
		}

		ret = pmi8998_haptics_module_enable(haptics, false);
		if (ret < 0) {
			pr_err("Error in disabling module, ret=%d\n", ret);
			goto out;
		}

		if (haptics->auto_res_enabled)
			pmi8998_haptics_lra_freq_write(haptics);
	}

out:
	mutex_unlock(&haptics->play_lock);
	return ret;
}

/*
 * Work function to update the haptics state.
 */
static void pmi8998_process_work(struct work_struct *work)
{
	struct pmi8998_haptics *haptics = container_of(work, struct pmi8998_haptics, work);

	int ret;
	bool enable;

	enable = atomic_read(&haptics->active);
	pr_debug("%s: state: %d\n", __func__, enable);

	ret = pmi8998_haptics_set(haptics, enable);
	if (ret < 0)
		pr_err("Error setting haptics, ret=%d", ret);
}

/**
 * pmi8998_haptics_close - callback for input device close
 * @dev: input device pointer
 *
 * Turns off the vibrator.
 */
static void pmi8998_haptics_close(struct input_dev *dev)
{
	struct pmi8998_haptics *haptics = input_get_drvdata(dev);

	cancel_work_sync(&haptics->work);
	if (atomic_read(&haptics->active)) {
		atomic_set(&haptics->active, 0);
		schedule_work(&haptics->work);
	}
}

/**
 * pmi8998_haptics_play_effect - play haptics effects
 * @dev: input device pointer
 * @data: data of effect
 * @effect: effect to play
 *
 * Currently this driver supports only rumble effects.
 */
static int pmi8998_haptics_play_effect(struct input_dev *dev, void *data,
				  struct ff_effect *effect)
{
	struct pmi8998_haptics *haptics = input_get_drvdata(dev);

	pr_debug("%s: Rumbling with strong: %d and weak: %d", __func__,
		effect->u.rumble.strong_magnitude, effect->u.rumble.weak_magnitude);

	haptics->magnitude = effect->u.rumble.strong_magnitude >> 8;
	if (!haptics->magnitude)
		haptics->magnitude = effect->u.rumble.weak_magnitude >> 9;

	if (haptics->magnitude) {
		atomic_set(&haptics->active, 1);
		haptics->vmax = ((HAP_VMAX_MAX_MV - HAP_VMAX_MIN_MV) * haptics->magnitude) / 0x0e +
						HAP_VMAX_MIN_MV;
	} else {
		atomic_set(&haptics->active, 0);
		haptics->vmax = HAP_VMAX_MIN_MV;
	}

	haptics->vmax = constrain(haptics->vmax, HAP_VMAX_MIN_MV, HAP_VMAX_MAX_MV);

	pr_debug("%s: magnitude: %d, vmax: %d", __func__, haptics->magnitude, haptics->vmax);

	pmi8998_haptics_write_vmax(haptics);

	schedule_work(&haptics->work);

	return 0;
}

static int pmi8998_haptics_probe(struct platform_device *pdev)
{
	struct pmi8998_haptics *haptics;
	struct device_node *node;
	struct input_dev *input_dev;
	int ret;
	unsigned int val;
	int temp, i;

	haptics = devm_kzalloc(&pdev->dev, sizeof(*haptics), GFP_KERNEL);
	if (!haptics)
		return -ENOMEM;

	haptics->regmap = dev_get_regmap(pdev->dev.parent, NULL);
	if (!haptics->regmap)
		return -ENODEV;

	node = pdev->dev.of_node;

	haptics->pdev = pdev;	

	ret = of_property_read_u32(node, "reg", &temp);
	if (ret < 0) {
		pr_err("Couldn't find reg in node = %s ret = %d\n",
			node->full_name, ret);
		return ret;
	}

	if (temp <= 0) {
		pr_err("Invalid base address: 0x%x\n", temp);
		return -EINVAL;
	}
	haptics->base = (u16)temp;

	haptics->play_irq = platform_get_irq_byname(pdev, "hap-play-irq");
	if (haptics->play_irq < 0) {
		dev_err(&pdev->dev, "Unable to get play irq\n");
		ret = haptics->play_irq;
		goto register_fail;
	}

	haptics->sc_irq = platform_get_irq_byname(pdev, "hap-sc-irq");
	if (haptics->sc_irq < 0) {
		dev_err(&pdev->dev, "Unable to get sc irq\n");
		ret = haptics->sc_irq;
		goto register_fail;
	}

	haptics->actuator_type = HAP_TYPE_LRA;

	haptics->play_mode = HAP_PLAY_BUFFER;

	haptics->play_wave_rate = HAP_WAVE_PLAY_RATE_DEF_US;
	ret = of_property_read_u32(node,
			"qcom,wave-play-rate-us", &val);
	if (!ret) {
		haptics->play_wave_rate = val;
	} else if (ret != -EINVAL) {
		pr_err("Unable to read play rate ret=%d\n", ret);
		goto register_fail;
	}

	haptics->wave_shape = HAP_WAVE_SINE;
	ret = of_property_read_u32(node, "qcom,wave-shape", &val);
	if (!ret) {
		if (val != HAP_WAVE_SINE && val != HAP_WAVE_SQUARE) {
			dev_err(&pdev->dev, "qcom,wave-shape is invalid: %d\n", val);
			ret = -EINVAL;
			goto register_fail;
		}
		haptics->wave_shape = val;
	}

	haptics->brake_pat[0] = 0x3;
	haptics->brake_pat[1] = 0x3;
	haptics->brake_pat[2] = 0x3;
	haptics->brake_pat[3] = 0x3;

	haptics->wave_samp_idx = 0;

	for (i = 0; i < HAP_WAVE_SAMP_LEN; i++)
			haptics->wave_samp[i] = HAP_WF_SAMP_MAX;

	haptics->play_wave_rate =
		constrain(haptics->play_wave_rate,
		HAP_WAVE_PLAY_RATE_MIN_US, HAP_WAVE_PLAY_RATE_MAX_US);

	ret = pmi8998_haptics_init(haptics);
	if (ret < 0) {
		dev_err(&pdev->dev, "Error in configuring haptics, ret=%d\n",
			ret);
		goto register_fail;
	}

	platform_set_drvdata(pdev, haptics);

	input_dev = devm_input_allocate_device(&pdev->dev);
	if (!input_dev)
		return -ENOMEM;

	INIT_WORK(&haptics->work, pmi8998_process_work);
	haptics->haptics_input_dev = input_dev;

	input_dev->name = "pmi8998_haptics";
	input_dev->id.version = 1;
	input_dev->close = pmi8998_haptics_close;
	input_set_drvdata(input_dev, haptics);
	// Figure out how to make this FF_PERIODIC
	input_set_capability(haptics->haptics_input_dev, EV_FF, FF_RUMBLE);

	ret = input_ff_create_memless(input_dev, NULL,
					pmi8998_haptics_play_effect);
	if (ret) {
		dev_err(&pdev->dev,
			"couldn't register vibrator as FF device\n");
		goto register_fail;
	}

	ret = input_register_device(input_dev);
	if (ret) {
		dev_err(&pdev->dev, "couldn't register input device\n");
		goto register_fail;
	}

	return 0;

register_fail:
	cancel_work_sync(&haptics->work);
	mutex_destroy(&haptics->play_lock);

	return ret;
}

static int __maybe_unused pmi8998_haptics_suspend(struct device *dev)
{
	struct pmi8998_haptics *haptics = dev_get_drvdata(dev);

	cancel_work_sync(&haptics->work);
	pmi8998_haptics_set(haptics, false);

	return 0;
}

static SIMPLE_DEV_PM_OPS(pmi8998_haptics_pm_ops, pmi8998_haptics_suspend, NULL);

static int qpnp_haptics_remove(struct platform_device *pdev)
{
	struct pmi8998_haptics *haptics = dev_get_drvdata(&pdev->dev);

	cancel_work_sync(&haptics->work);
	mutex_destroy(&haptics->play_lock);
	dev_set_drvdata(&pdev->dev, NULL);

	return 0;
}

static void qpnp_haptics_shutdown(struct platform_device *pdev)
{
	struct pmi8998_haptics *haptics = dev_get_drvdata(&pdev->dev);

	cancel_work_sync(&haptics->work);

	pmi8998_haptics_set(haptics, false);
}

static const struct of_device_id pmi8998_haptics_id_table[] = {
	{ .compatible = "qcom,qpnp-haptics-buffer" },
	{ }
};
MODULE_DEVICE_TABLE(of, pmi8998_haptics_id_table);

static struct platform_driver pmi8998_haptics_driver = {
	.probe		= pmi8998_haptics_probe,
	.remove		= qpnp_haptics_remove,
	.shutdown	= qpnp_haptics_shutdown,
	.driver		= {
		.name	= "qpnp-haptics",
		.pm	= &pmi8998_haptics_pm_ops,
		.of_match_table = pmi8998_haptics_id_table,
	},
};
module_platform_driver(pmi8998_haptics_driver);

MODULE_ALIAS("platform:pmi8998_haptics");
MODULE_DESCRIPTION("PMI8998 vibrator driver based on ff-memless framework");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Caleb Connolly <caleb@connolly.tech>");
