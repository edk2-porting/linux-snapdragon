#include <linux/module.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/of_platform.h>
#include <linux/slab.h>
#include <linux/qcom_scm.h>

#define LMH_NODE_DCVS			0x44435653

#define LMH_SUB_FN_THERMAL		0x54484D4C
#define LMH_SUB_FN_CRNT			0x43524E54
#define LMH_SUB_FN_REL			0x52454C00
#define LMH_SUB_FN_BCL			0x42434C00
#define LMH_SUB_FN_GENERAL		0x47454E00

#define LMH_ALGO_MODE_ENABLE		0x454E424C
#define LMH_HI_THRESHOLD		0x48494748
#define LMH_LOW_THRESHOLD		0x4C4F5700
#define LMH_ARM_THRESHOLD		0x41524D00

struct lmh_hw_data {
	u32 payload[5];
	u32 payload_size;
	u32 node_id;
};

static void update_payload(struct lmh_hw_data *lmh_data, u32 fn, u32 reg, u32 val)
{

	lmh_data->payload[0] = fn;
	lmh_data->payload[1] = 0;
	lmh_data->payload[2] = reg;
	lmh_data->payload[3] = 1;
	lmh_data->payload[4] = val;
}

static int lmh_probe(struct platform_device *pdev)
{
	struct device *dev;
	struct device_node *np;
	struct lmh_hw_data *lmh_data;
	u32 node_id;
	int ret;

	dev = &pdev->dev;
	np = dev->of_node;
	if (!np)
		return -EINVAL;

	lmh_data = devm_kzalloc(dev, sizeof(*lmh_data), GFP_KERNEL);
	if(!lmh_data)
		return -ENOMEM;

	ret = of_property_read_u32(np, "qcom,node-id", &node_id);
	if (ret)
		return -ENODEV;

	lmh_data->node_id = node_id;
	/* Payload size is five bytes for now */
	lmh_data->payload_size = 5 * sizeof(u32);

	platform_set_drvdata(pdev, lmh_data);
	/* Downstream enables regulator. Ask Bjorn how to do this upstream */
	
	if (!qcom_scm_limit_dcvsh_available())
		return -EINVAL;

	/* Enable Thermal Algorithm */
	update_payload(lmh_data, LMH_SUB_FN_THERMAL, LMH_ALGO_MODE_ENABLE, 1);
	ret = qcom_scm_lmh_limit_dcvsh(lmh_data->payload, lmh_data->payload_size, LMH_NODE_DCVS, lmh_data->node_id, 0);
	if (ret) {
		dev_err(dev, "Error %d enabling thermal subfunction\n", ret);
		return ret;
	}

	ret = qcom_scm_lmh_limit_profile_change(0x1);
	if (ret) {
		dev_err(dev, "Error %d changing profile\n", ret);
		return ret;
	}

	/* Set default thermal trips */
	update_payload(lmh_data, LMH_SUB_FN_THERMAL, LMH_ARM_THRESHOLD, 65000);
	ret = qcom_scm_lmh_limit_dcvsh(lmh_data->payload, lmh_data->payload_size, LMH_NODE_DCVS, lmh_data->node_id, 0);
	if (ret) {
		dev_err(dev, "Error setting thermal ARM thershold%d \n", ret);
		return ret;
	}
	update_payload(lmh_data, LMH_SUB_FN_THERMAL, LMH_HI_THRESHOLD, 95000);
	ret = qcom_scm_lmh_limit_dcvsh(lmh_data->payload, lmh_data->payload_size, LMH_NODE_DCVS, lmh_data->node_id, 0);
	if (ret) {
		dev_err(dev, "Error setting thermal HI thershold%d \n", ret);
		return ret;
	}
	update_payload(lmh_data, LMH_SUB_FN_THERMAL, LMH_LOW_THRESHOLD, 94500);
	ret = qcom_scm_lmh_limit_dcvsh(lmh_data->payload, lmh_data->payload_size, LMH_NODE_DCVS, lmh_data->node_id, 0);
	if (ret) {
		dev_err(dev, "Error setting thermal ARM thershold%d \n", ret);
		return ret;
	}

	return 0;
}

static int lmh_remove(struct platform_device *pdev) {

	return 0;
}

static const struct of_device_id lmh_table[] = {
	{ .compatible = "qcom,msm-hw-limits", },
	{},
};

static struct platform_driver lmh_driver = {
	.probe = lmh_probe,
	.remove = lmh_remove,
	.driver = {
		.name = "qcom-lmh",
		.of_match_table = lmh_table,
	},
};
module_platform_driver(lmh_driver);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("QCOM LMH driver");
