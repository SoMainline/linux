// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2018-2020, The Linux Foundation. All rights reserved.
 * Copyright (c) 2021, Sony Corporation
 * Copyright (c) 2023, Linaro Limited
 */

#include <linux/module.h>
#include <linux/of_irq.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/of_regulator.h>
#include <linux/regulator/machine.h>

#include <dt-bindings/regulator/qcom,amoled-regulator.h>

/* Register definitions */
#define PERIPH_TYPE			0x04
#define PERIPH_TYPE_IBB			0x20
#define PERIPH_TYPE_AB			0x24
/* OLEDB is also part of this block, but HLOS doesn't touch it at all. */

#define IBB_REG_BASE			0xf800
#define AB_REG_BASE			0xf900

/* AB */
#define AB_LDO_PD_CTL			(AB_REG_BASE + 0x78)
 #define AB_LDO_PD_CTL_PULLDN_EN	BIT(7)

/* IBB */
#define IBB_PD_CTL			(IBB_REG_BASE + 0x47)
 #define IBB_PD_CTL_ENABLE_PD		BIT(7)

#define IBB_DUAL_PHASE_CTL		(IBB_REG_BASE + 0x70)
 #define IBB_DUAL_PHASE_CTL_MASK	GENMASK(2, 0)
  #define FORCE_SINGLE_PHASE_BIT	BIT(0)
  #define FORCE_DUAL_PHASE_BIT		BIT(1)
  #define AUTO_DUAL_PHASE_BIT		BIT(2)

#define SINGLE_PHASE_ILIMIT_UA	30000


struct qpnp_amoled_regulator {
	struct device *dev;
	struct regmap *regmap;
	struct regulator_desc desc;
	struct regulator_dev *rdev;
	unsigned int type;
	unsigned int mode;
	bool enabled;
	int uV;
	u16 base;

	bool has_phase_control; /* IBB */
	bool single_phase; /* IBB */
	bool pd_control; /* AB/IBB */
};

struct amoled_regulator_data {
	const char			*name;
	u8				type;
	u16				base;
	const struct regulator_desc	*desc;
	bool				has_phase_control;
};

/* The regulators are wholly HW-controlled, U/I constraints are set before Linux boots. */
static int qpnp_amoled_regulator_is_enabled(struct regulator_dev *rdev)
{
	struct qpnp_amoled_regulator *vreg = rdev_get_drvdata(rdev);

	return vreg->enabled;
}

static int qpnp_amoled_regulator_enable(struct regulator_dev *rdev)
{
	struct qpnp_amoled_regulator *vreg = rdev_get_drvdata(rdev);

	vreg->enabled = true;

	return 0;
}

static int qpnp_amoled_regulator_disable(struct regulator_dev *rdev)
{
	struct qpnp_amoled_regulator *vreg = rdev_get_drvdata(rdev);

	vreg->enabled = false;

	return 0;
}

static int qpnp_amoled_regulator_set_voltage(struct regulator_dev *rdev,
					     int min_uV, int max_uV,
					     unsigned int *selector)
{
	struct qpnp_amoled_regulator *vreg = rdev_get_drvdata(rdev);
	vreg->uV = min_uV;

	return 0;
}

static int qpnp_amoled_regulator_get_voltage(struct regulator_dev *rdev)
{
	struct qpnp_amoled_regulator *vreg = rdev_get_drvdata(rdev);

	return vreg->uV;
}

static int qpnp_ab_ibb_pd_control(struct qpnp_amoled_regulator *vreg, bool enable)
{
	u16 addr, bit;

	if (!vreg->pd_control)
		return 0;

	if (vreg->type == PERIPH_TYPE_AB) {
		addr = AB_LDO_PD_CTL;
		bit = enable ? AB_LDO_PD_CTL_PULLDN_EN : 0;
	} else if (vreg->type == PERIPH_TYPE_IBB) {
		addr = IBB_PD_CTL;
		bit = enable ? IBB_PD_CTL_ENABLE_PD : 0;
	} else {
		/* How did we even get here? */
		return -EINVAL;
	}

	/* Downstream writes the register on ABB, but it's just one bit that we're about.. */
	return regmap_update_bits(vreg->regmap, addr, bit, bit);
}

/* NOTE: AB and IBB modes should be matching, otherwise you may only get 1 power rail.. */
static int qpnp_ab_ibb_regulator_set_mode(struct regulator_dev *rdev,
					  unsigned int mode)
{
	bool enable = mode & (REGULATOR_MODE_NORMAL | REGULATOR_MODE_STANDBY);
	struct qpnp_amoled_regulator *vreg = rdev_get_drvdata(rdev);
	int ret = 0;

	if (mode == vreg->mode || mode == vreg->mode)
		return 0;

	ret = qpnp_ab_ibb_pd_control(vreg, enable);
	if (!ret)
		vreg->mode = mode;

	return ret;
}

static unsigned int qpnp_amoled_regulator_get_mode(struct regulator_dev *rdev)
{
	struct qpnp_amoled_regulator *vreg = rdev_get_drvdata(rdev);

	return vreg->mode;
}


static int qcom_ibb_set_phase_control(struct qpnp_amoled_regulator *vreg, u8 ibb_phase)
{
	if (!vreg->has_phase_control)
		return 0;

	return regmap_update_bits(vreg->regmap, IBB_DUAL_PHASE_CTL,
				  IBB_DUAL_PHASE_CTL_MASK, ibb_phase);
}

static int qpnp_ibb_regulator_set_load(struct regulator_dev *rdev,
				       int load_uA)
{
	struct qpnp_amoled_regulator *vreg = rdev_get_drvdata(rdev);
	u8 ibb_phase;

	/* For single phase IBB, PHASE_CTL is configured only once. */
	if (vreg->single_phase)
		return 0;

	if (load_uA < 0)
		return -EINVAL;

	ibb_phase = load_uA > SINGLE_PHASE_ILIMIT_UA ? FORCE_DUAL_PHASE_BIT : AUTO_DUAL_PHASE_BIT;

	return qcom_ibb_set_phase_control(vreg, ibb_phase);
}

static unsigned int qcom_amoled_regulator_of_map_mode(unsigned int amoled_mode)
{
	unsigned int mode;

	switch (amoled_mode) {
	case AMOLED_REGULATOR_MODE_ON:
		mode = REGULATOR_MODE_NORMAL;
		break;
	case AMOLED_REGULATOR_MODE_LPM:
		mode = REGULATOR_MODE_STANDBY;
		break;
	case AMOLED_REGULATOR_MODE_OFF:
		mode = REGULATOR_MODE_IDLE;
		break;
	}

	return REGULATOR_MODE_NORMAL;
}

static struct regulator_ops qpnp_amoled_ab_ops = {
	.enable		= qpnp_amoled_regulator_enable,
	.disable	= qpnp_amoled_regulator_disable,
	.is_enabled	= qpnp_amoled_regulator_is_enabled,
	.set_voltage	= qpnp_amoled_regulator_set_voltage,
	.get_voltage	= qpnp_amoled_regulator_get_voltage,
	.set_mode	= qpnp_ab_ibb_regulator_set_mode,
	.get_mode	= qpnp_amoled_regulator_get_mode,
};

static const struct regulator_desc pm8150b_ab_desc = {
	.owner		= THIS_MODULE,
	.type		= REGULATOR_VOLTAGE,
	.min_uV		= 4600000,
	.uV_step	= 100000,
	.n_voltages 	= 7,
	.of_map_mode	= qcom_amoled_regulator_of_map_mode,
	.ops		= &qpnp_amoled_ab_ops,
};

static struct regulator_ops qpnp_amoled_ibb_ops = {
	.enable		= qpnp_amoled_regulator_enable,
	.disable	= qpnp_amoled_regulator_disable,
	.is_enabled	= qpnp_amoled_regulator_is_enabled,
	.set_voltage	= qpnp_amoled_regulator_set_voltage,
	.get_voltage	= qpnp_amoled_regulator_get_voltage,
	.set_mode	= qpnp_ab_ibb_regulator_set_mode,
	.get_mode	= qpnp_amoled_regulator_get_mode,
	.set_load	= qpnp_ibb_regulator_set_load,
};

static const struct regulator_desc pm8150b_ibb_desc = {
	.owner		= THIS_MODULE,
	.type		= REGULATOR_VOLTAGE,
	.min_uV		= 1400000,
	.uV_step	= 100000,
	.n_voltages 	= 53,
	.of_map_mode	= qcom_amoled_regulator_of_map_mode,
	.ops		= &qpnp_amoled_ibb_ops,
};

static const struct regulator_desc pm8350b_ibb_desc = {
	.owner		= THIS_MODULE,
	.type		= REGULATOR_VOLTAGE,
	.min_uV		= 1400000,
	.uV_step	= 100000,
	.n_voltages 	= 41,
	.of_map_mode	= qcom_amoled_regulator_of_map_mode,
	.ops		= &qpnp_amoled_ibb_ops,
};

static const struct amoled_regulator_data pm8150b_labibb_data[] = {
	{ "ab", PERIPH_TYPE_AB, AB_REG_BASE, &pm8150b_ab_desc, false },
	{ "ibb", PERIPH_TYPE_IBB, IBB_REG_BASE, &pm8150b_ibb_desc, false },
	{ },
};

static const struct amoled_regulator_data pm8350b_labibb_data[] = {
	{ "ab", PERIPH_TYPE_AB, AB_REG_BASE, &pm8150b_ab_desc, false },
	{ "ibb", PERIPH_TYPE_IBB, IBB_REG_BASE, &pm8350b_ibb_desc, true },
	{ },
};

static const struct of_device_id qcom_amoled_match_table[] = {
	{ .compatible = "qcom,pm8150b-amoled-regulator", &pm8150b_labibb_data },
	{ .compatible = "qcom,pm8350b-amoled-regulator", &pm8350b_labibb_data },
	{ },
};

static int qpnp_amoled_regulator_probe(struct platform_device *pdev)
{
	const struct amoled_regulator_data *reg_data;
	const struct of_device_id *match;
	struct device_node *reg_node;
	struct regulator_config cfg = {};
	struct regmap *reg_regmap;
	struct qpnp_amoled_regulator *vreg;
	unsigned int type;
	u8 ibb_phase;
	int ret;

	reg_regmap = dev_get_regmap(pdev->dev.parent, NULL);
	if (!reg_regmap)
		return dev_err_probe(&pdev->dev, -ENODEV, "Couldn't get parent's regmap");

	match = of_match_device(qcom_amoled_match_table, &pdev->dev);
	if (!match)
		return -ENODEV;

	vreg = devm_kzalloc(&pdev->dev, sizeof(*vreg), GFP_KERNEL);
	if (!vreg)
		return -ENOMEM;

	for (reg_data = match->data; reg_data->name; reg_data++) {
		/*
		 * Check if we're interacting with the correct regulator..
		 * A mistake here could potentially cost one a new display panel!
		 */
		ret = regmap_read(reg_regmap, reg_data->base + PERIPH_TYPE, &type);
		if (ret)
			return ret;

		if (WARN_ON(type != reg_data->type))
			return dev_err_probe(&pdev->dev, -EINVAL,
					     "Found invalid periph type: 0x%x", type);

		vreg = devm_kzalloc(&pdev->dev, sizeof(*vreg), GFP_KERNEL);
		if (!vreg)
			return -ENOMEM;

		reg_node = of_get_child_by_name(pdev->dev.of_node, reg_data->name);
		if (!reg_node)
			return dev_err_probe(&pdev->dev, -EINVAL,
					     "Couldn't get %s node", reg_data->name);

		vreg->regmap = reg_regmap;
		vreg->dev = &pdev->dev;
		vreg->base = reg_data->base;
		vreg->type = reg_data->type;

		memcpy(&vreg->desc, reg_data->desc, sizeof(vreg->desc));
		vreg->desc.of_match = reg_data->name;
		vreg->desc.name = reg_data->name;

		cfg.dev = vreg->dev;
		cfg.driver_data = vreg;
		cfg.regmap = vreg->regmap;

		vreg->pd_control = of_property_read_bool(reg_node, "qcom,aod-pd-control");

		if (vreg->type == PERIPH_TYPE_IBB)
			vreg->single_phase = of_property_read_bool(reg_node,
								   "qcom,ibb-single-phase");

		vreg->rdev = devm_regulator_register(vreg->dev, &vreg->desc, &cfg);
		if (IS_ERR(vreg->rdev))
			return dev_err_probe(&pdev->dev, PTR_ERR(vreg->rdev),
					     "Could not register regulator %s : %d\n",
					     reg_data->name, ret);
	}

	ibb_phase = vreg->single_phase ? FORCE_SINGLE_PHASE_BIT : AUTO_DUAL_PHASE_BIT;

	return qcom_ibb_set_phase_control(vreg, ibb_phase);
}

static struct platform_driver qpnp_amoled_regulator_driver = {
	.driver = {
		.name = "qpnp-amoled-regulator",
		.of_match_table = qcom_amoled_match_table,
	},
	.probe = qpnp_amoled_regulator_probe,
};
module_platform_driver(qpnp_amoled_regulator_driver);

MODULE_DESCRIPTION("QPNP AMOLED regulator driver");
MODULE_LICENSE("GPL");
