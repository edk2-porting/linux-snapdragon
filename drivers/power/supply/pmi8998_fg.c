// SPDX-License-Identifier: GPL-2.0-only
/* Copyright (c) 2020, The Linux Foundation. All rights reserved. */

#include <linux/kernel.h>
#include <linux/platform_device.h>
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

#include "pmi8998_fg.h"

/************************
 * IO FUNCTIONS
 * **********************/

/**
 * pmi8998_read() - Read multiple registers with regmap_bulk_read
 * 
 * @param map The regmap to read
 * @param val Pointer to read values into
 * @param addr Address to read from
 * @param len Number of registers (bytes) to read
 * @return int 0 on success, negative errno on error
 */
static int pmi8998_read(struct regmap *map, u8 *val, u16 addr, int len)
{
	if ((addr & 0xff00) == 0) {
		pr_err("base cannot be zero base=0x%02x\n", addr);
		return -EINVAL;
	}

	//pr_info("%s: reading 0x%x bytes from 0x%x", __func__, len, addr);

	return regmap_bulk_read(map, addr, val, len);
}

/**
 * @brief pmi8998_write() - Write multiple registers with regmap_bulk_write
 * 
 * @param map The regmap to write
 * @param val Pointer to write values into
 * @param addr Address to write from
 * @param len Number of registers (bytes) to write
 * @return int 0 on success, negative errno on error
 */
static int pmi8998_write(struct regmap *map, u8 *val, u16 addr, int len)
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
 * @brief pmi8998_masked_write() - like pmi8998_write but applies
 * a mask first.
 * 
 * @param map The regmap to write
 * @param val Pointer to write values into
 * @param addr Address to write from
 * @param len Number of registers (bytes) to write
 * @return int 0 on success, negative errno on error
 */
static int pmi8998_masked_write(struct regmap *map, u16 addr,
		u8 mask, u8 val)
{
	int error;
	u8 reg;
	error = pmi8998_read(map, &reg, addr, 1);
	if (error)
		return error;

	reg &= ~mask;
	reg |= val & mask;

	error = pmi8998_write(map, &reg, addr, 1);
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

/*************************
 * Battery Status RW
 * ***********************/

static int pmi8998_fg_get_capacity(struct pmi8998_fg_chip *chip, int *val)
{
	u8 cap[2];
	int error = pmi8998_read(chip->regmap, cap, REG_BASE + BATT_MONOTONIC_SOC, 2);
	if (error)
		return error;
	if (cap[0] != cap[1]) {
		cap[0] = cap[0] < cap[1] ? cap[0] : cap[1];
	}
	*val = DIV_ROUND_CLOSEST((cap[0] - 1) * 98, 0xff - 2) + 1;
	return 0;
}

static int pmi8998_fg_get_temperature(struct pmi8998_fg_chip *chip, int *val)
{
	int rc, temp;
	u8 readval[2];

	rc = pmi8998_read(chip->regmap, readval, REG_BATT + PARAM_ADDR_BATT_TEMP, 2);
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

static int pmi8998_fg_get_current(struct pmi8998_fg_chip *chip, int *val)
{
	int rc, temp;
	u8 readval[2];

	rc = pmi8998_read(chip->regmap, readval, REG_BATT + PARAM_ADDR_BATT_CURRENT, 2);
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

static int pmi8998_fg_get_voltage(struct pmi8998_fg_chip *chip, int *val)
{
	int rc, temp;
	u8 readval[2];

	rc = pmi8998_read(chip->regmap, readval, REG_BATT + PARAM_ADDR_BATT_VOLTAGE, 2);
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

static int pmi8998_iacs_clear_sequence(struct pmi8998_fg_chip *chip)
{
	int rc = 0;
	u8 temp;

	/* clear the error */
	rc = pmi8998_masked_write(chip->regmap, REG_MEM + MEM_INTF_IMA_CFG,
				BIT(2), BIT(2));
	if (rc) {
		pr_err("Error writing to IMA_CFG, rc=%d\n", rc);
		return rc;
	}

	temp = 0x4;
	rc = pmi8998_write(chip->regmap, &temp, REG_MEM + MEM_INTF_ADDR_LSB + 1, 1);
	if (rc) {
		pr_err("Error writing to MEM_INTF_ADDR_MSB, rc=%d\n", rc);
		return rc;
	}

	temp = 0x0;
	rc = pmi8998_write(chip->regmap, &temp, REG_MEM + MEM_INTF_WR_DATA0 + 3, 1);
	if (rc) {
		pr_err("Error writing to WR_DATA3, rc=%d\n", rc);
		return rc;
	}

	rc = pmi8998_read(chip->regmap, &temp, REG_MEM + MEM_INTF_RD_DATA0 + 3, 1);
	if (rc) {
		pr_err("Error writing to RD_DATA3, rc=%d\n", rc);
		return rc;
	}

	rc = pmi8998_masked_write(chip->regmap, REG_MEM + MEM_INTF_IMA_CFG,
				BIT(2), 0);
	if (rc) {
		pr_err("Error writing to IMA_CFG, rc=%d\n", rc);
		return rc;
	}
	return rc;
}

static int pmi8998_clear_ima(struct pmi8998_fg_chip *chip,
		bool check_hw_sts)
{
	int rc = 0, ret = 0;
	u8 err_sts = 0, exp_sts = 0, hw_sts = 0;
	bool run_err_clr_seq = false;

	rc = pmi8998_read(chip->regmap, &err_sts,
			REG_MEM + MEM_INTF_IMA_ERR_STS, 1);
	if (rc) {
		dev_err(chip->dev, "failed to read IMA_ERR_STS, rc=%d\n", rc);
		return rc;
	}

	rc = pmi8998_read(chip->regmap, &exp_sts,
			REG_MEM + MEM_INTF_IMA_EXP_STS, 1);
	if (rc) {
		dev_err(chip->dev, "Error in reading IMA_EXP_STS, rc=%d\n", rc);
		return rc;
	}

	if (check_hw_sts) {
		rc = pmi8998_read(chip->regmap, &hw_sts,
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
		ret = pmi8998_iacs_clear_sequence(chip);
		if (!ret)
			return -EAGAIN;
		else
			dev_err(chip->dev, "Error clearing IMA exception ret=%d\n", ret);
	}

	return rc;
}

int pmi8998_get_prop_usb_online(struct pmi8998_fg_chip *chip, int *val){
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

int pmi8998_get_prop_batt_status(struct pmi8998_fg_chip *chip, int *val){
	int usb_online_val;
	unsigned int stat;
	int rc;
	bool usb_online;

	rc = pmi8998_get_prop_usb_online(chip, &usb_online_val);
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

int pmi8998_get_prop_health_status(struct pmi8998_fg_chip *chip, int *val){
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

static int pmi8998_get_temp_threshold(struct pmi8998_fg_chip *chip,
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

	rc = pmi8998_read(chip->regmap, &temp, reg, 1);
	if (rc < 0) {
		dev_err(chip->dev, "Error in reading jeita level for psp:%d, rc=%d\n", psp, rc);
		return rc;
	}

	/* Resolution is 0.5C. Base is -30C. */
	*val = (((5 * temp) / 10) - 30) * 10;
	return 0;
}

static void fg_get_model_name(struct pmi8998_fg_chip *chip, union power_supply_propval *val)
{
	switch (chip->subtype)
	{
	case PMI8998_SUBTYPE:
		val->strval = "PMI8998 Battery";
		break;
	case PM8998_SUBTYPE:
		val->strval = "PM8998 Battery";
		break;
	//handle pm660 and other socs that use fg3
	default:
		val->strval = "Unknown PMIC Battery";
	}
}

static int fg_get_property(struct power_supply *psy,
		enum power_supply_property psp,
		union power_supply_propval *val)
{
	struct pmi8998_fg_chip *chip = power_supply_get_drvdata(psy);
	int error = 0;

	dev_dbg(chip->dev, "Getting property: %d", psp);

	switch (psp) {
	case POWER_SUPPLY_PROP_MANUFACTURER:
		val->strval = "Qualcomm";
		break;
	case POWER_SUPPLY_PROP_MODEL_NAME:
		fg_get_model_name(chip, val);
		break;
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = POWER_SUPPLY_TECHNOLOGY_LION;
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		error = pmi8998_fg_get_capacity(chip, &val->intval);
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		error = pmi8998_fg_get_current(chip, &val->intval);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		error = pmi8998_fg_get_voltage(chip, &val->intval);
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
		error = pmi8998_get_prop_batt_status(chip, &val->intval);
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		error = pmi8998_get_prop_health_status(chip, &val->intval);
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN:
	case POWER_SUPPLY_PROP_CHARGE_FULL: /* TODO: Implement capacity learning */
		val->intval = chip->batt_cap_uah;
		break;
	case POWER_SUPPLY_PROP_TEMP:
		error = pmi8998_fg_get_temperature(chip, &val->intval);
		break;
	case POWER_SUPPLY_PROP_TEMP_MIN:
	case POWER_SUPPLY_PROP_TEMP_MAX:
	case POWER_SUPPLY_PROP_TEMP_ALERT_MIN:
	case POWER_SUPPLY_PROP_TEMP_ALERT_MAX:
		error = pmi8998_get_temp_threshold(chip, psp, &val->intval);
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

irqreturn_t pmi8998_handle_usb_plugin(int irq, void *data){
	struct pmi8998_fg_chip *chip = data;
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

static int pmi8998_fg_probe(struct platform_device *pdev)
{
	struct power_supply_config supply_config = {};
	struct pmi8998_fg_chip *chip;
	const __be32 *prop_addr;
	int rc = 0, irq;
	u8 dma_status;
	bool error_present;

	chip = devm_kzalloc(&pdev->dev, sizeof(*chip), GFP_KERNEL);
	if (!chip) {
		return -ENOMEM;
	}

	chip->dev = &pdev->dev;
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
	rc = pmi8998_read(chip->regmap, chip->revision, REG_MEM + DIG_MINOR, 4);
	if (rc) {
		dev_err(chip->dev, "Unable to read FG revision rc=%d\n", rc);
		return rc;
	}

	rc = regmap_read(chip->regmap, PMIC_SUBTYPE, &chip->subtype);
	if (rc < 0) {
		dev_err(chip->dev, "Unable to read FG subtype rc=%d\n", rc);
		return rc;
	}

	dev_dbg(chip->dev, "pmi8998 revision DIG:%d.%d ANA:%d.%d\n",
		chip->revision[DIG_MAJOR], chip->revision[DIG_MINOR],
		chip->revision[ANA_MAJOR], chip->revision[ANA_MINOR]);
	
	/*
	 * Change the FG_MEM_INT interrupt to track IACS_READY
	 * condition instead of end-of-transaction. This makes sure
	 * that the next transaction starts only after the hw is ready.
	 * IACS_INTR_SRC_SLCT is BIT(3)
	 */
	rc = pmi8998_masked_write(chip->regmap,
		REG_MEM + MEM_INTF_IMA_CFG, BIT(3), BIT(3));
	if (rc) {
		dev_err(chip->dev,
			"failed to configure interrupt source %d\n",
			rc);
		return rc;
	}

	rc = pmi8998_clear_ima(chip, true);
	if (rc && rc != -EAGAIN) {
		dev_err(chip->dev, "Error clearing IMA, exception rc=%d", rc);
		return rc;
	}

	// Check and clear DMA errors
	rc = pmi8998_read(chip->regmap, &dma_status, REG_MEM + 0x70, 1);
	if (rc < 0) {
		pr_err("failed to read dma_status, rc=%d\n", rc);
		return rc;
	}

	error_present = dma_status & (BIT(1) | BIT(2));
	rc = pmi8998_masked_write(chip->regmap, REG_MEM + 0x71, BIT(0),
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

	irq = of_irq_get_byname(pdev->dev.of_node, "usb-plugin");
	if (irq < 0) {
		dev_err(&pdev->dev, "Couldn't get irq usb-plugin byname\n");
		return irq;
	}

	rc = devm_request_threaded_irq(chip->dev, irq, NULL,
					pmi8998_handle_usb_plugin,
					IRQF_ONESHOT, "usb-plugin", chip);
	if (rc < 0) {
		pr_err("Couldn't request irq %d\n", irq);
		return rc;
	}

	return 0;
}

static int pmi8998_fg_remove(struct platform_device *pdev)
{
	return 0;
}

static const struct of_device_id fg_match_id_table[] = {
	{ .compatible = "qcom,pmi8998-fg" },
	{ /* sentinal */ }
};
MODULE_DEVICE_TABLE(of, fg_match_id_table);

static struct platform_driver qcom_fg_driver = {
	.probe = pmi8998_fg_probe,
	.remove = pmi8998_fg_remove,
	.driver = {
		.name = "pmi8998-fg",
		.of_match_table = fg_match_id_table,
	},
};

module_platform_driver(qcom_fg_driver);

MODULE_AUTHOR("Caleb Connolly <caleb@connolly.tech>");
MODULE_AUTHOR("Joel Selvaraj <jo@jsfamily.in>");
MODULE_DESCRIPTION("Qualcomm PMI8998 Fuel Guage Driver");
MODULE_LICENSE("GPL v2");
