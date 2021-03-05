// SPDX-License-Identifier: GPL-2.0-only
/* Copyright (c) 2020, The Linux Foundation. All rights reserved. */

#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/of_device.h>
#include <linux/of_address.h>
#include <linux/regmap.h>
#include <linux/device.h>
#include <linux/mutex.h>
#include <linux/types.h>
#include <linux/power_supply.h>
#include <linux/module.h>
#include <linux/math64.h>
#include <linux/of_irq.h>
#include <linux/interrupt.h>

#include "qcom_fg.h"

/************************
 * IO FUNCTIONS
 * **********************/

/**
 * qcom_fg_read() - Read multiple registers with regmap_bulk_read
 * 
 * @param map The regmap to read
 * @param val Pointer to read values into
 * @param addr Address to read from
 * @param len Number of registers (bytes) to read
 * @return int 0 on success, negative errno on error
 */
static int qcom_fg_read(struct regmap *map, u8 *val, u16 addr, int len)
{
	if ((addr & 0xff00) == 0) {
		pr_err("base cannot be zero base=0x%02x\n", addr);
		return -EINVAL;
	}

	//pr_info("%s: reading 0x%x bytes from 0x%x", __func__, len, addr);

	return regmap_bulk_read(map, addr, val, len);
}

/**
 * @brief qcom_fg_write() - Write multiple registers with regmap_bulk_write
 * 
 * @param map The regmap to write
 * @param val Pointer to write values into
 * @param addr Address to write from
 * @param len Number of registers (bytes) to write
 * @return int 0 on success, negative errno on error
 */
static int qcom_fg_write(struct regmap *map, u8 *val, u16 addr, int len)
{
	int rc;
	bool sec_access = (addr & 0xff) > 0xd0;
	u8 sec_addr_val = 0xa5;

	if (sec_access) {
		rc = regmap_bulk_write(map,
				(addr & 0xff00) | 0xd0,
				&sec_addr_val, 1);
	}

	if ((addr & 0xff00) == 0) {
		pr_err("addr cannot be zero base=0x%02x\n", addr);
		return -EINVAL;
	}

	return regmap_bulk_write(map, addr, val, len);
}

/**
 * @brief qcom_fg_masked_write() - like qcom_fg_write but applies
 * a mask first.
 * 
 * @param map The regmap to write
 * @param val Pointer to write values into
 * @param addr Address to write from
 * @param len Number of registers (bytes) to write
 * @return int 0 on success, negative errno on error
 */
static int qcom_fg_masked_write(struct regmap *map, u16 addr,
		u8 mask, u8 val)
{
	int error;
	u8 reg;
	error = qcom_fg_read(map, &reg, addr, 1);
	if (error)
		return error;

	reg &= ~mask;
	reg |= val & mask;

	error = qcom_fg_write(map, &reg, addr, 1);
	return error;
}

static int64_t twos_compliment_extend(int64_t val, int nbytes)
{
	int i;
	int64_t mask;

	mask = 0x80LL << ((nbytes - 1) * 8);
	if (val & mask) {
		for (i = 8; i > nbytes; i--) {
			mask = 0xFFLL << ((i - 1) * 8);
			val |= mask;
		}
	}
	return val;
}

/************************
 * SRAM FUNCTIONS
 * **********************/

/**
 * qcom_fg_sram_check_access() - Check if SRAM is accessible
 *
 * @param chip Pointer to chip
 * @return bool true if accessible, false otherwise
 */
static bool qcom_fg_sram_check_access(struct qcom_fg_chip *chip)
{
	int rc;
	u8 mem_if_status;

	rc = qcom_fg_read(chip->regmap, &mem_if_status,
		REG_MEM + MEM_INTF_STS, 1);

	if(!(mem_if_status & MEM_INTF_AVAIL))
		return false;

	rc = qcom_fg_read(chip->regmap, &mem_if_status,
		REG_MEM + MEM_INTF_CFG, 1);

	if(rc)
		return false;

	return !!(mem_if_status & RIF_MEM_ACCESS_REQ);
}

/**
 * qcom_fg_sram_request_access() - Request access to SRAM and wait for it
 * 
 * @param chip Pointer to chip
 * @return int 0 on success, negative errno on error
 */
static int qcom_fg_sram_request_access(struct qcom_fg_chip *chip)
{
	int rc;

	if(!qcom_fg_sram_check_access(chip)) {
		rc = qcom_fg_masked_write(chip->regmap, REG_MEM + MEM_INTF_CFG,
				RIF_MEM_ACCESS_REQ, RIF_MEM_ACCESS_REQ);
		if (rc) {
			dev_err(chip->dev,
				"Failed to set SRAM access request bit: %d\n", rc);
			return rc;
		}
	}

	/* Wait to get access to SRAM, and try again if interrupted */
	do {
		rc = wait_for_completion_interruptible_timeout(
			&chip->sram_access_granted,
			msecs_to_jiffies(MEM_IF_TIMEOUT_MS));
	} while(rc == -ERESTARTSYS);

	if(rc <= 0) {
		rc = -ETIMEDOUT;
		return rc;
	}

	return 0;
}

/**
 * qcom_fg_sram_release_access() - Release access to SRAM
 *
 * @param chip Pointer to chip
 * @return int 0 on success, negative errno on error
 */
static int qcom_fg_sram_release_access(struct qcom_fg_chip *chip)
{
	int rc;

	rc = qcom_fg_masked_write(chip->regmap, REG_MEM + MEM_INTF_CFG,
			RIF_MEM_ACCESS_REQ, 0);
	if (rc) {
		dev_err(chip->dev,
			"Failed to set SRAM access request bit: %d\n", rc);
		return rc;
	}

	reinit_completion(&chip->sram_access_granted);

	return rc;
}

/**
 * qcom_fg_sram_config_access() - Configure access to SRAM
 *
 * @param chip Pointer to chip
 * @param write 0 for read access, 1 for write access
 * @param burst 1 to access mutliple addresses successively
 * @return int 0 on success, negative errno on error
 */
static int qcom_fg_sram_config_access(struct qcom_fg_chip *chip,
		bool write, bool burst)
{
	int rc;
	u8 intf_ctl = (write ? MEM_INTF_CTL_WR_EN : 0)
			| (burst ? MEM_INTF_CTL_BURST : 0);

	rc = qcom_fg_write(chip->regmap, &intf_ctl,
			REG_MEM + MEM_INTF_CTL, 1);

	if(rc)
		dev_err(chip->dev,
			"Failed to configure SRAM access: %d\n", rc);

	return rc;
}

/**
 * qcom_fg_sram_read() - Read data from SRAM
 *
 * @param chip Pointer to chip
 * @param val Pointer to read values into
 * @param addr Address to read from
 * @param len Number of bytes to read
 * @return int 0 on success, negative errno on error
 */
static int qcom_fg_sram_read(struct qcom_fg_chip *chip,
		u8 *val, u16 addr, int len, int offset)
{
	int rc;
	u8 *rd_data = val;

	rc = qcom_fg_sram_request_access(chip);
	if(rc) {
		dev_err(chip->dev, "Failed to request SRAM access: %d", rc);
		return rc;
	}

	dev_dbg(chip->dev,
		"Reading address 0x%x with offset %d of length %d from SRAM",
		addr, len, offset);

	rc = qcom_fg_sram_config_access(chip, 0, (len > 4));
	if(rc) {
		dev_err(chip->dev, "Failed to configure SRAM access: %d", rc);
		return rc;
	}

	while(len > 0) {
		/* Set SRAM address register */
		rc = qcom_fg_write(chip->regmap, (u8 *) &addr,
				REG_MEM + MEM_INTF_ADDR_LSB, 2);
		if(rc) {
			dev_err(chip->dev, "Failed to set SRAM address: %d", rc);
			return rc;
		}

		rc = qcom_fg_read(chip->regmap, rd_data,
				REG_MEM + MEM_INTF_RD_DATA0 + offset, len);

		addr += 4;

		if(rc)
			return rc;

		rd_data += 4 - offset;
		len -= 4 - offset;
		offset = 0;
	}

	rc = qcom_fg_sram_release_access(chip);

	return rc;
}

/**
 * qcom_fg_sram_write() - Write data to SRAM
 *
 * @param chip Pointer to chip
 * @param val Pointer to write values into
 * @param addr Address to write to
 * @param len Number of bytes to write
 * @return int 0 on success, negative errno on error
 */
static int qcom_fg_sram_write(struct qcom_fg_chip *chip,
		u8 *val, u16 addr, int len, int offset)
{
	int rc;
	u8 *wr_data = val;

	rc = qcom_fg_sram_request_access(chip);
	if(rc) {
		dev_err(chip->dev, "Failed to request SRAM access: %d", rc);
		return rc;
	}

	dev_dbg(chip->dev,
		"Reading address 0x%x with offset %d of length %d from SRAM",
		addr, len, offset);

	rc = qcom_fg_sram_config_access(chip, 1, (len > 4));
	if(rc) {
		dev_err(chip->dev, "Failed to configure SRAM access: %d", rc);
		return rc;
	}

	while(len > 0) {
		/* Set SRAM address register */
		rc = qcom_fg_write(chip->regmap, (u8 *) &addr,
				REG_MEM + MEM_INTF_ADDR_LSB, 2);
		if(rc) {
			dev_err(chip->dev, "Failed to set SRAM address: %d", rc);
			return rc;
		}

		rc = qcom_fg_write(chip->regmap, wr_data,
				REG_MEM + MEM_INTF_WR_DATA0 + offset, len);

		addr += 4;

		if(rc)
			return rc;

		wr_data += 4 - offset;
		len -= 4 - offset;
		offset = 0;
	}

	rc = qcom_fg_sram_release_access(chip);

	return rc;
}

/*************************
 * Battery Status RW
 * ***********************/

static int qcom_fg_get_capacity(struct qcom_fg_chip *chip, int *val)
{
	u8 cap[2];
	int error = qcom_fg_read(chip->regmap, cap, REG_BASE + BATT_MONOTONIC_SOC, 2);
	if (error)
		return error;
	if (cap[0] != cap[1]) {
		cap[0] = cap[0] < cap[1] ? cap[0] : cap[1];
	}
	*val = DIV_ROUND_CLOSEST((cap[0] - 1) * 98, 0xff - 2) + 1;
	return 0;
}

static int qcom_fg_get_temperature(struct qcom_fg_chip *chip, int *val)
{
	int rc, temp;
	u8 readval[2];

	rc = qcom_fg_sram_read(chip, readval, 0x550, 2, 2);
	if(rc) {
		dev_err(chip->dev, "Failed to read temperature: %d", rc);
		return rc;
	}

	temp = readval[1] << 8 | readval[0];
	*val = temp * 625 / 1000 - 2730;
	return 0;
}

static int qcom_fg_get_current(struct qcom_fg_chip *chip, int *val)
{
	int rc, temp;
	u8 readval[2];

	rc = qcom_fg_sram_read(chip, readval, 0x5CC, 2, 3);
	if(rc) {
		dev_err(chip->dev, "Failed to read current: %d", rc);
		return rc;
	}
	//handle rev 1 too
	temp = readval[1] << 8 | readval[0];
	temp = twos_compliment_extend(temp, 15);
	*val = div_s64((s64)temp * 152587, 1000);
	return 0;
}

static int qcom_fg_get_voltage(struct qcom_fg_chip *chip, int *val)
{
	int rc, temp;
	u8 readval[2];

	rc = qcom_fg_sram_read(chip, readval, 0x5CC, 2, 1);
	if(rc) {
		dev_err(chip->dev, "Failed to read voltage: %d", rc);
		return rc;
	}

	temp = readval[1] << 8 | readval[0];
	*val = div_u64((u64)temp * 152587, 1000);

	return 0;
}

/*************************
 * Battery Status RW, Gen3
 * ***********************/

static int qcom_fg_gen3_get_temperature(struct qcom_fg_chip *chip, int *val)
{
	int rc, temp;
	u8 readval[2];

	rc = qcom_fg_read(chip->regmap, readval, REG_BATT + PARAM_ADDR_BATT_TEMP, 2);
	if (rc) {
		pr_err("Failed to read temperature\n");
		return rc;
	}
	temp = ((readval[1] & BATT_TEMP_MSB_MASK) << 8) |
		(readval[0] & BATT_TEMP_LSB_MASK);
	temp = DIV_ROUND_CLOSEST(temp * 10, 4);

	*val = temp -2730;
	return 0;
}

static int qcom_fg_gen3_get_current(struct qcom_fg_chip *chip, int *val)
{
	int rc, temp;
	u8 readval[2];

	rc = qcom_fg_read(chip->regmap, readval, REG_BATT + PARAM_ADDR_BATT_CURRENT, 2);
	if (rc) {
		pr_err("Failed to read current\n");
		return rc;
	}
	//handle rev 1 too
	temp = readval[1] << 8 | readval[0];
	temp = twos_compliment_extend(temp, 15);
	*val = div_s64((s64)temp * 488281, 1000);
	return 0;
}

static int qcom_fg_gen3_get_voltage(struct qcom_fg_chip *chip, int *val)
{
	int rc, temp;
	u8 readval[2];

	rc = qcom_fg_read(chip->regmap, readval, REG_BATT + PARAM_ADDR_BATT_VOLTAGE, 2);
	if (rc) {
		pr_err("Failed to read voltage\n");
		return rc;
	}
	//handle rev 1 too
	temp = readval[1] << 8 | readval[0];
	*val = div_u64((u64)temp * 122070, 1000);
	return 0;
}

/********************
 * Init stuff
 * ******************/

static int qcom_fg_iacs_clear_sequence(struct qcom_fg_chip *chip)
{
	int rc = 0;
	u8 temp;

	/* clear the error */
	rc = qcom_fg_masked_write(chip->regmap, REG_MEM + MEM_INTF_IMA_CFG,
				BIT(2), BIT(2));
	if (rc) {
		pr_err("Error writing to IMA_CFG, rc=%d\n", rc);
		return rc;
	}

	temp = 0x4;
	rc = qcom_fg_write(chip->regmap, &temp, REG_MEM + MEM_INTF_ADDR_LSB + 1, 1);
	if (rc) {
		pr_err("Error writing to MEM_INTF_ADDR_MSB, rc=%d\n", rc);
		return rc;
	}

	temp = 0x0;
	rc = qcom_fg_write(chip->regmap, &temp, REG_MEM + MEM_INTF_WR_DATA0 + 3, 1);
	if (rc) {
		pr_err("Error writing to WR_DATA3, rc=%d\n", rc);
		return rc;
	}

	rc = qcom_fg_read(chip->regmap, &temp, REG_MEM + MEM_INTF_RD_DATA0 + 3, 1);
	if (rc) {
		pr_err("Error writing to RD_DATA3, rc=%d\n", rc);
		return rc;
	}

	rc = qcom_fg_masked_write(chip->regmap, REG_MEM + MEM_INTF_IMA_CFG,
				BIT(2), 0);
	if (rc) {
		pr_err("Error writing to IMA_CFG, rc=%d\n", rc);
		return rc;
	}
	return rc;
}

static int qcom_fg_clear_ima(struct qcom_fg_chip *chip,
		bool check_hw_sts)
{
	int rc = 0, ret = 0;
	u8 err_sts = 0, exp_sts = 0, hw_sts = 0;
	bool run_err_clr_seq = false;

	rc = qcom_fg_read(chip->regmap, &err_sts,
			REG_MEM + MEM_INTF_IMA_ERR_STS, 1);
	if (rc) {
		dev_err(chip->dev, "failed to read IMA_ERR_STS, rc=%d\n", rc);
		return rc;
	}

	rc = qcom_fg_read(chip->regmap, &exp_sts,
			REG_MEM + MEM_INTF_IMA_EXP_STS, 1);
	if (rc) {
		dev_err(chip->dev, "Error in reading IMA_EXP_STS, rc=%d\n", rc);
		return rc;
	}

	if (check_hw_sts) {
		rc = qcom_fg_read(chip->regmap, &hw_sts,
				REG_MEM + MEM_INTF_IMA_HW_STS, 1);
		if (rc) {
			dev_err(chip->dev, "Error in reading IMA_HW_STS, rc=%d\n", rc);
			return rc;
		}
		/*
		 * Lower nibble should be equal to upper nibble before SRAM
		 * transactions begins from SW side.
		 */
		if ((hw_sts & 0x0f) != hw_sts >> 4) {
			dev_err(chip->dev, "IMA HW not in correct state, hw_sts=%x\n",
					hw_sts);
			run_err_clr_seq = true;
		}
	}

	if (exp_sts & (BIT(0) | BIT(1) | BIT(3) |
		BIT(4) | BIT(5) | BIT(6) |
		BIT(7))) {
		dev_warn(chip->dev, "IMA exception bit set, exp_sts=%x\n", exp_sts);
		run_err_clr_seq = true;
	}

	if (run_err_clr_seq) {
		ret = qcom_fg_iacs_clear_sequence(chip);
		if (!ret)
			return -EAGAIN;
		else
			dev_err(chip->dev, "Error clearing IMA exception ret=%d\n", ret);
	}

	return rc;
}

int qcom_fg_get_prop_usb_online(struct qcom_fg_chip *chip, int *val){
	unsigned int stat;
	int rc;

	rc = regmap_read(chip->regmap, POWER_PATH_STATUS_REG, &stat);
	if (rc < 0){
		dev_err(chip->dev, "Couldn't read POWER_PATH_STATUS! ret=%d\n", rc);
		return rc;
	}

	dev_dbg(chip->dev, "USB POWER_PATH_STATUS : 0x%02x\n", stat);
	*val = (stat & BIT(4)) && (stat & BIT(0));
	return rc;
}

int qcom_fg_get_prop_batt_status(struct qcom_fg_chip *chip, int *val){
	int usb_online_val;
	unsigned int stat;
	int rc;
	bool usb_online;

	rc = qcom_fg_get_prop_usb_online(chip, &usb_online_val);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't get usb online property rc=%d\n", rc);
		return rc;
	}
	dev_dbg(chip->dev, "USB ONLINE val : %d\n", usb_online_val);
	usb_online = (bool)usb_online_val;

	if (!usb_online) {
		*val = POWER_SUPPLY_STATUS_DISCHARGING;
		return rc;
	}

	rc = regmap_read(chip->regmap, BATTERY_CHARGER_STATUS_REG(chip), &stat);
	if (rc < 0){
		dev_err(chip->dev, "Charging status REGMAP read failed! ret=%d\n", rc);
		return rc;
	}
		
	stat = stat & BATTERY_CHARGER_STATUS_MASK;
	dev_dbg(chip->dev, "Charging status : %d!\n", stat);

	switch (stat) {
		case TRICKLE_CHARGE:
		case PRE_CHARGE:
		case FAST_CHARGE:
		case FULLON_CHARGE:
		case TAPER_CHARGE:
		case TERMINATE_CHARGE:
		case INHIBIT_CHARGE:
			*val = POWER_SUPPLY_STATUS_CHARGING;
			break;
		case DISABLE_CHARGE:
			*val = POWER_SUPPLY_STATUS_NOT_CHARGING;
			break;
		default:  
			*val = POWER_SUPPLY_STATUS_UNKNOWN;
			break;
	}

	return rc;
}

int qcom_fg_get_prop_health_status(struct qcom_fg_chip *chip, int *val){
	unsigned int stat;
	int rc;

	rc = regmap_read(chip->regmap, BATTERY_HEALTH_STATUS_REG(chip), &stat);
	if (rc < 0){
		dev_err(chip->dev, "Health status REGMAP read failed! ret=%d\n", rc);
		return rc;
	}

	if (stat & BIT(0))
		*val = POWER_SUPPLY_HEALTH_COLD;
	else if (stat & BIT(1))
		*val = POWER_SUPPLY_HEALTH_OVERHEAT;
	else if (stat & BIT(2))
		*val = POWER_SUPPLY_HEALTH_COOL;
	else if (stat & BIT(3))
		*val = POWER_SUPPLY_HEALTH_WARM;
	else
		*val = POWER_SUPPLY_HEALTH_GOOD;
	
	return rc;
}

static int qcom_fg_get_temp_threshold(struct qcom_fg_chip *chip,
				enum power_supply_property psp, int *val)
{
	int rc;
	u8 temp;
	int offset;

	switch (psp) {
	case POWER_SUPPLY_PROP_TEMP_MIN:
		offset = 0;
		break;
	case POWER_SUPPLY_PROP_TEMP_MAX:
		offset = 1;
		break;
	case POWER_SUPPLY_PROP_TEMP_ALERT_MIN:
		offset = 2;
		break;
	case POWER_SUPPLY_PROP_TEMP_ALERT_MAX:
		offset = 3;
		break;
	default:
		return -EINVAL;
	}

	rc = qcom_fg_sram_read(chip, &temp, 0x454, 1, offset);
	if (rc < 0) {
		dev_err(chip->dev, "Error in reading jeita level for psp:%d, rc=%d\n", psp, rc);
		return rc;
	}

	*val = (temp - 30) * 10;

	return 0;
}

static int qcom_fg_gen3_get_temp_threshold(struct qcom_fg_chip *chip,
				enum power_supply_property psp, int *val)
{
	int rc;
	u8 temp;
	u16 reg;

	switch (psp) {
	case POWER_SUPPLY_PROP_TEMP_MIN:
		reg = BATT_INFO_JEITA_COLD(chip);
		break;
	case POWER_SUPPLY_PROP_TEMP_MAX:
		reg = BATT_INFO_JEITA_HOT(chip);
		break;
	case POWER_SUPPLY_PROP_TEMP_ALERT_MIN:
		reg = BATT_INFO_JEITA_COOL(chip);
		break;
	case POWER_SUPPLY_PROP_TEMP_ALERT_MAX:
		reg = BATT_INFO_JEITA_WARM(chip);
		break;
	default:
		return -EINVAL;
	}

	rc = qcom_fg_read(chip->regmap, &temp, reg, 1);
	if (rc < 0) {
		dev_err(chip->dev, "Error in reading jeita level for psp:%d, rc=%d\n", psp, rc);
		return rc;
	}

	/* Resolution is 0.5C. Base is -30C. */
	*val = (((5 * temp) / 10) - 30) * 10;
	return 0;
}

static int qcom_fg_set_temp_threshold(struct qcom_fg_chip *chip,
				enum power_supply_property psp, int val)
{
	int rc;
	u8 temp;
	int offset;

	switch (psp) {
	case POWER_SUPPLY_PROP_TEMP_MIN:
		offset = 0;
		break;
	case POWER_SUPPLY_PROP_TEMP_MAX:
		offset = 1;
		break;
	case POWER_SUPPLY_PROP_TEMP_ALERT_MIN:
		offset = 2;
		break;
	case POWER_SUPPLY_PROP_TEMP_ALERT_MAX:
		offset = 3;
		break;
	default:
		return -EINVAL;
	}

	temp = val / 10 + 30;

	rc = qcom_fg_sram_write(chip, &temp, 0x454, 1, offset);
	if (rc < 0) {
		dev_err(chip->dev, "Error in writing jeita level for psp:%d, rc=%d\n", psp, rc);
		return rc;
	}

	return 0;
}

static int qcom_fg_gen3_set_temp_threshold(struct qcom_fg_chip *chip,
				enum power_supply_property psp, int val)
{
	/* Not implemented yet */
	return -ENOTSUPP;
}


static void qcom_fg_get_model_name(struct qcom_fg_chip *chip, union power_supply_propval *val)
{
	switch (chip->subtype)
	{
	case PMI8994_SUBTYPE:
		val->strval = "PMI8994 Battery";
		break;
	case PMI8996_SUBTYPE:
		val->strval = "PMI8996 Battery";
		break;
	case PMI8998_SUBTYPE:
		val->strval = "PMI8998 Battery";
		break;
	case PM8998_SUBTYPE:
		val->strval = "PM8998 Battery";
		break;
	/* Handle other PMICs */
	default:
		val->strval = "Unknown PMIC Battery";
	}
}

static int fg_get_property(struct power_supply *psy,
		enum power_supply_property psp,
		union power_supply_propval *val)
{
	struct qcom_fg_chip *chip = power_supply_get_drvdata(psy);
	int error = 0;

	dev_dbg(chip->dev, "Getting property: %d", psp);

	switch (psp) {
	case POWER_SUPPLY_PROP_MANUFACTURER:
		val->strval = "Qualcomm";
		break;
	case POWER_SUPPLY_PROP_MODEL_NAME:
		qcom_fg_get_model_name(chip, val);
		break;
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = POWER_SUPPLY_TECHNOLOGY_LION;
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		error = chip->ops->get_capacity(chip, &val->intval);
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		error = chip->ops->get_current(chip, &val->intval);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		error = chip->ops->get_voltage(chip, &val->intval);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MIN:
	case POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN:
		val->intval = chip->batt_min_voltage_uv;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX:
	case POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN:
		val->intval = chip->batt_max_voltage_uv;
		break;
	case POWER_SUPPLY_PROP_STATUS:
		error = chip->ops->get_batt_status(chip, &val->intval);
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		error = chip->ops->get_health_status(chip, &val->intval);
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN:
	case POWER_SUPPLY_PROP_CHARGE_FULL: /* TODO: Implement capacity learning */
		val->intval = chip->batt_cap_uah;
		break;
	case POWER_SUPPLY_PROP_TEMP:
		error = chip->ops->get_temperature(chip, &val->intval);
		break;
	case POWER_SUPPLY_PROP_TEMP_MIN:
	case POWER_SUPPLY_PROP_TEMP_MAX:
	case POWER_SUPPLY_PROP_TEMP_ALERT_MIN:
	case POWER_SUPPLY_PROP_TEMP_ALERT_MAX:
		error = chip->ops->get_temp_threshold(chip, psp, &val->intval);
		break;
	//POWER_SUPPLY_PROP_TIME_TO_FULL_NOW,POWER_SUPPLY_PROP_TIME_TO_FULL_AVG - calculate time remaining for full charge - implementable
	//POWER_SUPPLY_PROP_TIME_TO_EMPTY_NOW,POWER_SUPPLY_PROP_TIME_TO_EMPTY_AVG - calculate time remaining when discharging - implementable
	//POWER_SUPPLY_PROP_CHARGE_NOW - requires capacity learning
	//POWER_SUPPLY_PROP_CHARGE_FULL - requires capacity learning
	//POWER_SUPPLY_PROP_CHARGE_COUNTER - requires capacity learning
	//POWER_SUPPLY_PROP_CYCLE_COUNT - needs votables - no idea how they work
	//POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT -  needs votables - no idea how they work
	//POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX -  needs votables - no idea how they work
	//POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT - needs votables - no idea how they work
	default:
		dev_err(chip->dev, "invalid property: %d\n", psp);
		return -EINVAL;
	}
	return error;
}

static const struct power_supply_desc bms_psy_desc = {
	.name = "bms",
	.type = POWER_SUPPLY_TYPE_BATTERY,
	.properties = fg_properties,
	.num_properties = ARRAY_SIZE(fg_properties),
	.get_property = fg_get_property,
};

irqreturn_t qcom_fg_handle_usb_plugin(int irq, void *data){
	struct qcom_fg_chip *chip = data;
	int rc;
	unsigned int stat;
	bool vbus_rising;
	union power_supply_propval val;

	rc = regmap_read(chip->regmap, USBIN_BASE + INT_RT_STS, &stat);
	if (rc < 0){
		dev_err(chip->dev, "Couldn't read USB status from reg! ret=%d\n", rc);
		return rc;
	}
	vbus_rising = (bool)(stat & BIT(4));

	if (vbus_rising) {
		val.intval = POWER_SUPPLY_STATUS_CHARGING;
		power_supply_set_property(chip->bms_psy, POWER_SUPPLY_PROP_STATUS, &val);
	} else {
		val.intval = POWER_SUPPLY_STATUS_DISCHARGING;
		power_supply_set_property(chip->bms_psy, POWER_SUPPLY_PROP_STATUS, &val);
	}	

	dev_dbg(chip->dev, "USB IRQ: %s\n", vbus_rising ? "attached" : "detached");
	power_supply_changed(chip->bms_psy);
	return IRQ_HANDLED;
}

irqreturn_t qcom_fg_handle_mem_avail(int irq, void *data){
	struct qcom_fg_chip *chip = data;

	if(qcom_fg_sram_check_access(chip)) {
		complete_all(&chip->sram_access_granted);
		dev_dbg(chip->dev, "SRAM access granted");
	} else {
		dev_dbg(chip->dev, "SRAM access revoked");
	}

	return IRQ_HANDLED;
}

/* Pre-Gen3 fuel gauge. PMI8996 and older */
static const struct qcom_fg_ops ops_fg = {
	.get_capacity = qcom_fg_get_capacity,
	.get_temperature = qcom_fg_get_temperature,
	.get_current = qcom_fg_get_current,
	.get_voltage = qcom_fg_get_voltage,
	.get_batt_status = qcom_fg_get_prop_batt_status,
	.get_health_status = qcom_fg_get_prop_health_status,
	.get_temp_threshold = qcom_fg_get_temp_threshold,
	.set_temp_threshold = qcom_fg_set_temp_threshold,
};

/* Gen3 fuel gauge. PMI8998 and newer */
static const struct qcom_fg_ops ops_fg_gen3 = {
	.get_capacity = qcom_fg_get_capacity,
	.get_temperature = qcom_fg_gen3_get_temperature,
	.get_current = qcom_fg_gen3_get_current,
	.get_voltage = qcom_fg_gen3_get_voltage,
	.get_batt_status = qcom_fg_get_prop_batt_status,
	.get_health_status = qcom_fg_get_prop_health_status,
	.get_temp_threshold = qcom_fg_gen3_get_temp_threshold,
	.set_temp_threshold = qcom_fg_gen3_set_temp_threshold,
};

static int qcom_fg_probe(struct platform_device *pdev)
{
	struct power_supply_config supply_config = {};
	struct qcom_fg_chip *chip;
	const __be32 *prop_addr;
	int rc = 0, irq;
	u8 dma_status;
	bool error_present;

	chip = devm_kzalloc(&pdev->dev, sizeof(*chip), GFP_KERNEL);
	if (!chip) {
		return -ENOMEM;
	}

	chip->dev = &pdev->dev;
	chip->ops = of_device_get_match_data(&pdev->dev);

	mutex_init(&chip->lock);

	chip->regmap = dev_get_regmap(pdev->dev.parent, NULL);
	if (!chip->regmap) {
		dev_err(chip->dev, "failed to locate the regmap\n");
		return -ENODEV;
	}

	// Get base address
	prop_addr = of_get_address(pdev->dev.of_node, 0, NULL, NULL);
	if (!prop_addr) {
		dev_err(chip->dev, "Couldn't read SOC base address from dt\n");
		return -EINVAL;
	}
	chip->base = be32_to_cpu(*prop_addr);

	prop_addr = of_get_address(pdev->dev.of_node, 1, NULL, NULL);
	if (!prop_addr) {
		dev_err(chip->dev, "Couldn't read CHG base address from dt\n");
		return -EINVAL;
	}
	chip->chg_base = be32_to_cpu(*prop_addr);

	rc = of_property_read_u32(pdev->dev.of_node, "qcom,min-voltage-uv",
					&chip->batt_min_voltage_uv);
	if (rc < 0) {
		dev_err(chip->dev, "Error in reading qcom,min-voltage-uv, rc=%d\n", rc);
		return rc;
	}

	rc = of_property_read_u32(pdev->dev.of_node, "qcom,max-voltage-uv",
					&chip->batt_max_voltage_uv);
	if (rc < 0) {
		dev_err(chip->dev, "Error in reading qcom,max-voltage-uv, rc=%d\n", rc);
		return rc;
	}
	
	rc = of_property_read_u32(pdev->dev.of_node, "qcom,battery-capacity-ua",
					&chip->batt_cap_uah);
	if (rc < 0) {
		dev_err(chip->dev, "Error in reading qcom,battery-capacity-ua, rc=%d\n", rc);
		return rc;
	}

	// Init memif fn inlined here (chip hardware info)
	rc = qcom_fg_read(chip->regmap, chip->revision, REG_MEM + DIG_MINOR, 4);
	if (rc) {
		dev_err(chip->dev, "Unable to read FG revision rc=%d\n", rc);
		return rc;
	}

	rc = regmap_read(chip->regmap, PMIC_SUBTYPE, &chip->subtype);
	if (rc < 0) {
		dev_err(chip->dev, "Unable to read FG subtype rc=%d\n", rc);
		return rc;
	}

	dev_dbg(chip->dev, "PMIC revision DIG:%d.%d ANA:%d.%d\n",
		chip->revision[DIG_MAJOR], chip->revision[DIG_MINOR],
		chip->revision[ANA_MAJOR], chip->revision[ANA_MINOR]);
	
	/*
	 * Change the FG_MEM_INT interrupt to track IACS_READY
	 * condition instead of end-of-transaction. This makes sure
	 * that the next transaction starts only after the hw is ready.
	 * IACS_INTR_SRC_SLCT is BIT(3)
	 */
	rc = qcom_fg_masked_write(chip->regmap,
		REG_MEM + MEM_INTF_IMA_CFG, BIT(3), BIT(3));
	if (rc) {
		dev_err(chip->dev,
			"failed to configure interrupt source %d\n",
			rc);
		return rc;
	}

	rc = qcom_fg_clear_ima(chip, true);
	if (rc && rc != -EAGAIN) {
		dev_err(chip->dev, "Error clearing IMA, exception rc=%d", rc);
		return rc;
	}

	// Check and clear DMA errors
	rc = qcom_fg_read(chip->regmap, &dma_status, REG_MEM + 0x70, 1);
	if (rc < 0) {
		pr_err("failed to read dma_status, rc=%d\n", rc);
		return rc;
	}

	error_present = dma_status & (BIT(1) | BIT(2));
	rc = qcom_fg_masked_write(chip->regmap, REG_MEM + 0x71, BIT(0),
			error_present ? BIT(0) : 0);
	if (rc < 0) {
		pr_err("failed to write dma_ctl, rc=%d\n", rc);
		return rc;
	}

	supply_config.drv_data = chip;
	supply_config.of_node = pdev->dev.of_node;

	chip->bms_psy = devm_power_supply_register(chip->dev,
			&bms_psy_desc, &supply_config);
	if (IS_ERR(chip->bms_psy)) {
		dev_err(&pdev->dev, "failed to register battery\n");
		return PTR_ERR(chip->bms_psy);
	}

	platform_set_drvdata(pdev, chip);

	/* Initialize IRQs */
	switch(chip->subtype)
	{
	case PMI8994_SUBTYPE:
	case PMI8996_SUBTYPE:
		irq = of_irq_get_byname(pdev->dev.of_node, "mem-avail");
		if (irq < 0) {
			dev_err(&pdev->dev, "Couldn't get irq mem-avail byname\n");
			return irq;
		}

		rc = devm_request_threaded_irq(chip->dev, irq, NULL,
						qcom_fg_handle_mem_avail,
						IRQF_ONESHOT, "mem-avail", chip);
		if (rc < 0) {
			pr_err("Couldn't request irq %d\n", irq);
			return rc;
		}

		init_completion(&chip->sram_access_granted);
		break;

	case PM8998_SUBTYPE:
	case PMI8998_SUBTYPE:
		irq = of_irq_get_byname(pdev->dev.of_node, "usb-plugin");
		if (irq < 0) {
			dev_err(&pdev->dev, "Couldn't get irq usb-plugin byname\n");
			return irq;
		}

		rc = devm_request_threaded_irq(chip->dev, irq, NULL,
						qcom_fg_handle_usb_plugin,
						IRQF_ONESHOT, "usb-plugin", chip);
		if (rc < 0) {
			pr_err("Couldn't request irq %d\n", irq);
			return rc;
		}

		break;
	}

	// Set default temperature thresholds
	rc = chip->ops->set_temp_threshold(chip,
					POWER_SUPPLY_PROP_TEMP_MIN,
					BATT_TEMP_JEITA_COLD);
	rc = chip->ops->set_temp_threshold(chip,
					POWER_SUPPLY_PROP_TEMP_MAX,
					BATT_TEMP_JEITA_WARM);
	rc = chip->ops->set_temp_threshold(chip,
					POWER_SUPPLY_PROP_TEMP_ALERT_MIN,
					BATT_TEMP_JEITA_COOL);
	rc = chip->ops->set_temp_threshold(chip,
					POWER_SUPPLY_PROP_TEMP_ALERT_MAX,
					BATT_TEMP_JEITA_HOT);
	if(rc == -ENOTSUPP) {
		dev_warn(chip->dev,
			"Setting temperature thresholds not supported");
	}
	else if(rc < 0) {
		dev_err(chip->dev,
			"Setting temperature thresholds failed: %d\n", rc);
		return rc;
	}

	return 0;
}

static int qcom_fg_remove(struct platform_device *pdev)
{
	return 0;
}

static const struct of_device_id fg_match_id_table[] = {
	{ .compatible = "qcom,pmi8994-fg", .data = &ops_fg },
	{ .compatible = "qcom,pmi8998-fg", .data = &ops_fg_gen3 },
	{ /* sentinal */ }
};
MODULE_DEVICE_TABLE(of, fg_match_id_table);

static struct platform_driver qcom_fg_driver = {
	.probe = qcom_fg_probe,
	.remove = qcom_fg_remove,
	.driver = {
		.name = "qcom-fg",
		.of_match_table = fg_match_id_table,
	},
};

module_platform_driver(qcom_fg_driver);

MODULE_AUTHOR("Caleb Connolly <caleb@connolly.tech>");
MODULE_AUTHOR("Joel Selvaraj <jo@jsfamily.in>");
MODULE_AUTHOR("Yassine Oudjana <y.oudjana@protonmail.com>");
MODULE_DESCRIPTION("Qualcomm PMIC Fuel Gauge Driver");
MODULE_LICENSE("GPL v2");
