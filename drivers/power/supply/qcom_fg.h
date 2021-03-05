
// #define FG_PARAM_MAX 49

/**** Registers *****/

#define PMIC_SUBTYPE		    0x105

#define	PMI8994_SUBTYPE             0x0a
#define	PMI8996_SUBTYPE             0x13
#define PM8998_SUBTYPE		    0x14
#define PMI8998_SUBTYPE		    0x15


/* SRAM */
#define MEM_INTF_STS                0x10
#define MEM_INTF_CFG                0x50
#define MEM_INTF_CTL                0x51
#define MEM_INTF_IMA_CFG            0x52
#define MEM_INTF_IMA_OPR_STS        0x54
#define MEM_INTF_IMA_EXP_STS        0x55
#define MEM_INTF_IMA_HW_STS         0x56
#define MEM_INTF_BEAT_COUNT         0x57
#define MEM_INTF_IMA_ERR_STS        0x5f
#define MEM_INTF_IMA_BYTE_EN        0x60
#define MEM_INTF_ADDR_LSB           0x61
#define MEM_INTF_RD_DATA0           0x67
#define MEM_INTF_WR_DATA0           0x63

#define MEM_INTF_AVAIL              BIT(0)
#define MEM_INTF_CTL_BURST          BIT(7)
#define MEM_INTF_CTL_WR_EN          BIT(6)
#define RIF_MEM_ACCESS_REQ          BIT(7)


/* Battery info */
#define BATT_INFO_CHARGE_MAX_DESIGN 0x4a
#define BATT_INFO_THERM_C1          0x5c
#define BATT_INFO_VBATT_LSB         0xa0
#define BATT_INFO_VBATT_MSB         0xa1
#define BATT_INFO_IBATT_LSB         0xa2
#define BATT_INFO_IBATT_MSB         0xa3
#define BATT_INFO_BATT_TEMP_LSB     0x50
#define BATT_INFO_BATT_TEMP_MSB     0x51
#define BATT_MONOTONIC_SOC          0x09

#define BATT_TEMP_LSB_MASK          GENMASK(7, 0)
#define BATT_TEMP_MSB_MASK          GENMASK(2, 0)


/* Base addresses */
#define REG_BASE                    0x4000
#define REG_BATT                    0x4100
#define REG_MEM                     0x4400


/* Interrupt offsets */
#define INT_RT_STS                  0x10
#define INT_EN_CLR                  0x16


/* Param addresses */
#define PARAM_ADDR_BATT_TEMP        0x50
#define PARAM_ADDR_BATT_VOLTAGE     0xa0
#define PARAM_ADDR_BATT_CURRENT     0xa2


#define MISC_BASE	            0x1600
#define USBIN_BASE                  0x1300


#define BATT_INFO_JEITA_COLD(chip)		(REG_BATT + 0x62)
#define BATT_INFO_JEITA_COOL(chip)		(REG_BATT + 0x63)
#define BATT_INFO_JEITA_WARM(chip)		(REG_BATT + 0x64)
#define BATT_INFO_JEITA_HOT(chip)		(REG_BATT + 0x65)

#define BATT_TEMP_JEITA_COLD        100
#define BATT_TEMP_JEITA_COOL        50
#define BATT_TEMP_JEITA_WARM        400
#define BATT_TEMP_JEITA_HOT         450

#define BATTERY_CHARGER_STATUS_REG(chip)	(chip->chg_base + 0x06)
#define BATTERY_HEALTH_STATUS_REG(chip)	        (chip->chg_base + 0x07)

#define BATTERY_CHARGER_STATUS_MASK GENMASK(2, 0)
#define POWER_PATH_STATUS_REG	(MISC_BASE + 0x0B)

#define MEM_IF_TIMEOUT_MS	5000

enum wa_flags {
	PMI8998_V1_REV_WA,
	PMI8998_V2_REV_WA,
};

enum pmic_rev_offsets {
	DIG_MINOR = 0x0,
	DIG_MAJOR = 0x1,
	ANA_MINOR = 0x2,
	ANA_MAJOR = 0x3,
};
enum pmic_rev {
	DIG_REV_1 = 0x1,
	DIG_REV_2 = 0x2,
	DIG_REV_3 = 0x3,
};

enum charger_status{
	TRICKLE_CHARGE = 0,
	PRE_CHARGE,
	FAST_CHARGE,
	FULLON_CHARGE,
	TAPER_CHARGE,
	TERMINATE_CHARGE,
	INHIBIT_CHARGE,
	DISABLE_CHARGE,
};

static enum power_supply_property fg_properties[] = {
	POWER_SUPPLY_PROP_MANUFACTURER,
	POWER_SUPPLY_PROP_MODEL_NAME,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_VOLTAGE_MIN,
	POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN,
	POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN,
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_CHARGE_FULL,
	POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_TEMP_MIN,
	POWER_SUPPLY_PROP_TEMP_MAX,
	POWER_SUPPLY_PROP_TEMP_ALERT_MIN,
	POWER_SUPPLY_PROP_TEMP_ALERT_MAX,
};

struct qcom_fg_chip;

struct qcom_fg_ops {
	int (*get_capacity)(struct qcom_fg_chip *chip, int *);
	int (*get_temperature)(struct qcom_fg_chip *chip, int *);
	int (*get_current)(struct qcom_fg_chip *chip, int *);
	int (*get_voltage)(struct qcom_fg_chip *chip, int *);
	int (*get_batt_status)(struct qcom_fg_chip *chip, int *);
	int (*get_health_status)(struct qcom_fg_chip *chip, int *);
	int (*get_temp_threshold)(struct qcom_fg_chip *chip,
			enum power_supply_property psp, int *);
	int (*set_temp_threshold)(struct qcom_fg_chip *chip,
			enum power_supply_property psp, int);
};

struct qcom_fg_chip {
	struct device *dev;
	unsigned int base;
	unsigned int chg_base;
	struct regmap *regmap;
	struct mutex lock;
	unsigned int subtype;
	const struct qcom_fg_ops *ops;

	struct power_supply *bms_psy;

	u8 revision[4];
	bool ima_supported;
	struct completion sram_access_granted;

	int batt_cap_uah;
	int batt_max_voltage_uv;
	int batt_min_voltage_uv;

	int health;
	int status;
};
